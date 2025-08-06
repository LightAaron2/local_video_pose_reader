
# 下面是按照上面建议修改后的完整脚本。主要改动有：

# 1. **拆分时间戳**：`img_timestamps` 和 `pose_timestamps` 两个独立数据集。
# 2. **延后启动写线程**：在 `run()` 中、摄像头和 ROS 订阅都准备好后再启动 `_hdf5_writer`。
# 3. **写逻辑对应新数据集**：分别往 `img_ds`/`img_ts_ds` 和 `pose_ds`/`pose_ts_ds` 写入。

import asyncio
import aiohttp
from aiohttp import web, WSMsgType
import cv2
import uuid
import json
import time
import logging
import threading
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from collections import deque
import sys
import os
import signal
import h5py

# 注释掉机械臂相关导入，因为现在只需要读取SLAM数据
# sys.path.append('/home/onestar/jiawei/BestMan_Xarm/')
# from Robotics_API.Bestman_Real_Xarm7 import *
# from Robotics_API.Pose import Pose

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 全局变量
exit_flag = False
slam_data_buffer = deque(maxlen=20)
slam_lock = threading.Lock()

# 捕捉退出信号
def signal_handler(signum, frame):
    global exit_flag
    print(f"收到信号 {signum}，正在退出...")
    exit_flag = True

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# HDF5 输出初始化
output_dir = "./recorded_data"
os.makedirs(output_dir, exist_ok=True)
h5f = h5py.File(os.path.join(output_dir, "record.hdf5"), "w")
H, W = 1080, 1920  # 根据实际分辨率调整

# 可扩展数据集：图像、图像时间戳、SLAM 位姿、位姿时间戳
img_ds     = h5f.create_dataset("images",         (0, H, W, 3),   maxshape=(None, H, W, 3),
                                 chunks=(1, H, W, 3), dtype="uint8", compression="gzip")
img_ts_ds  = h5f.create_dataset("img_timestamps", (0,),            maxshape=(None,),
                                 dtype="float64")
pose_ds    = h5f.create_dataset("poses",          (0, 7),          maxshape=(None, 7),
                                 dtype="float32")
pose_ts_ds = h5f.create_dataset("pose_timestamps",(0,),            maxshape=(None,),
                                 dtype="float64")

class CameraStreamTrack:
    def __init__(self, camera_index, topic_name):
        self.camera_index = camera_index
        self.topic_name = topic_name
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        try:
            self.subscriber = rospy.Subscriber(
                topic_name,
                Image,
                self.image_callback,
                queue_size=1
            )
            print(f"[Camera{camera_index}] ROS话题订阅成功: {topic_name}")
        except Exception as e:
            logger.error(f"[Camera{camera_index}] ROS订阅失败: {e}")
            self.subscriber = None

    def image_callback(self, msg):
        try:
            data = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == 'bgr8':
                cv_image = data.reshape((msg.height, msg.width, 3))
            elif msg.encoding == 'rgb8':
                rgb = data.reshape((msg.height, msg.width, 3))
                cv_image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'mono8':
                gray = data.reshape((msg.height, msg.width))
                cv_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                # 通用处理
                bpp = msg.step // msg.width
                if bpp == 1:
                    gray = data.reshape((msg.height, msg.width))
                    cv_image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                else:
                    rgb = data.reshape((msg.height, msg.width, bpp))
                    cv_image = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            with self.frame_lock:
                self.latest_frame = cv_image
        except Exception as e:
            logger.error(f"[Camera{self.camera_index}] 图像处理失败: {e}")

    def get_latest_frame(self):
        with self.frame_lock:
            return None if self.latest_frame is None else self.latest_frame.copy()

    def stop(self):
        if self.subscriber:
            try:
                self.subscriber.unregister()
            except Exception:
                pass
        print(f"[Camera{self.camera_index}] 已停止")

def detect_available_cameras():
    """扫描 ROS 上的摄像头话题，并返回配置信息列表"""
    cameras = []
    try:
        rospy.wait_for_service('/rosout', timeout=2.0)
    except rospy.ROSException:
        pass
    topics = []
    try:
        topics = rospy.get_published_topics()
    except Exception:
        pass
    # 过滤 sensor_msgs/Image 类型的摄像头话题
    candidates = [t for t, mt in topics if mt=='sensor_msgs/Image']
    for idx, topic in enumerate(sorted(set(candidates))):
        cameras.append({'index': idx, 'topic_name': topic})
        print(f"✓ 发现摄像头 #{idx}: {topic}")
    return cameras

def slam_data_callback(msg):
    """SLAM 位姿回调，入队等待写线程消费"""
    try:
        p = msg.pose.position
        o = msg.pose.orientation
        slam = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
        with slam_lock:
            slam_data_buffer.append((time.time(), slam))
    except Exception as e:
        logger.error(f"SLAM 回调异常: {e}")

class LocalBackend:
    def __init__(self):
        self.app = web.Application()
        self.cameras = {}
        self.websockets = set()
        self.runner = None
        self.site = None
        self.setup_routes()

    def setup_routes(self):
        self.app.router.add_static('/static', './static', name='static')
        self.app.router.add_get('/', self.index_handler)
        self.app.router.add_get('/ws', self.websocket_handler)
        self.app.router.add_get('/camera/{camera_id}/frame', self.camera_frame_handler)
        self.app.router.add_get('/camera/{camera_id}/stream', self.camera_stream_handler)

    async def index_handler(self, req):
        return web.FileResponse('./static/index.html')

    async def websocket_handler(self, req):
        ws = web.WebSocketResponse()
        await ws.prepare(req)
        self.websockets.add(ws)
        try:
            async for msg in ws:
                # 可根据 type 处理更多消息
                pass
        finally:
            self.websockets.discard(ws)
        return ws

    async def camera_frame_handler(self, req):
        camera_id = int(req.match_info['camera_id'])
        if camera_id not in self.cameras:
            return web.Response(status=404, text="Camera not found")
        frame = self.cameras[camera_id].get_latest_frame()
        if frame is None:
            return web.Response(status=404, text="No frame")
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY,80])
        return web.Response(body=buf.tobytes(), content_type='image/jpeg')

    async def camera_stream_handler(self, req):
        camera_id = int(req.match_info['camera_id'])
        if camera_id not in self.cameras:
            return web.Response(status=404, text="Camera not found")
        resp = web.StreamResponse(
            status=200, reason='OK',
            headers={'Content-Type':'multipart/x-mixed-replace; boundary=frame'}
        )
        await resp.prepare(req)
        while not exit_flag:
            frame = self.cameras[camera_id].get_latest_frame()
            if frame is not None:
                _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY,70])
                await resp.write(b"--frame\r\n")
                await resp.write(b"Content-Type: image/jpeg\r\n\r\n")
                await resp.write(buf.tobytes())
                await resp.write(b"\r\n")
            await asyncio.sleep(0.033)
        await resp.write_eof()
        return resp

    async def broadcast_slam_data(self):
        while not exit_flag:
            with slam_lock:
                buf = list(slam_data_buffer)
            if buf and self.websockets:
                _, latest = buf[-1]
                for ws in list(self.websockets):
                    try:
                        await ws.send_json({"type":"slam", "data": latest})
                    except:
                        self.websockets.discard(ws)
            await asyncio.sleep(0.1)


    def init_cameras(self):
        for cfg in detect_available_cameras():
            cam = CameraStreamTrack(cfg['index'], cfg['topic_name'])
            self.cameras[cfg['index']] = cam

    def init_ros_subscribers(self):
        rospy.Subscriber("/xv_sdk/xv_dev/slam/visual_pose", PoseStamped, slam_data_callback)

    async def run(self, host='localhost', port=8080):
        # 初始化摄像头、ROS 订阅
        self.init_cameras()
        self.init_ros_subscribers()

        asyncio.create_task(self.broadcast_slam_data())

        # 启动 aiohttp 服务
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, host, port)
        await self.site.start()
        print(f"Server running at http://{host}:{port}")

        # 在所有资源就绪后启动 HDF5 写线程
        threading.Thread(target=self._hdf5_writer, daemon=True).start()
        print("HDF5 写线程已启动")

        # 保持循环
        while not exit_flag:
            await asyncio.sleep(1)

        await self.cleanup()

    async def cleanup(self):
        print("清理资源...")
        for cam in self.cameras.values():
            cam.stop()
        for ws in list(self.websockets):
            await ws.close()
        if self.site:
            await self.site.stop()
        if self.runner:
            await self.runner.cleanup()
        print("资源清理完成")

    def _hdf5_writer(self):
        global h5f, img_ds, img_ts_ds, pose_ds, pose_ts_ds
        while not exit_flag:
            # 写图像数据
            for cam in self.cameras.values():
                frame = cam.get_latest_frame()
                if frame is not None:
                    n = img_ds.shape[0]
                    img_ds.resize(n+1, axis=0)
                    img_ds[n]     = frame
                    img_ts_ds.resize(n+1, axis=0)
                    img_ts_ds[n]  = time.time()

            # 写 SLAM 位姿
            with slam_lock:
                while slam_data_buffer:
                    t, slam = slam_data_buffer.popleft()
                    m = pose_ds.shape[0]
                    pose_ds.resize(m+1, axis=0)
                    pose_ds[m]      = slam
                    pose_ts_ds.resize(m+1, axis=0)
                    pose_ts_ds[m]   = t

            time.sleep(0.01)

        h5f.close()

async def main():
    try:
        rospy.init_node('local_backend_node', anonymous=True)
        print("ROS 节点初始化成功")
    except Exception:
        print("ROS 初始化失败，继续运行")
    backend = LocalBackend()
    try:
        await backend.run()
    except KeyboardInterrupt:
        pass
    finally:
        global exit_flag
        exit_flag = True
        print("程序退出")

if __name__ == "__main__":
    asyncio.run(main())
