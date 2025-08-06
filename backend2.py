#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
本地数据采集后端（FastUMI 简化版）
* 通过 WebSocket 控制开始 / 停止采集
* 采集视频帧 + SLAM 位姿到 HDF5
"""
import asyncio, aiohttp
from aiohttp import web, WSMsgType
import cv2, uuid, json, time, logging, threading, signal, os, sys
import h5py, numpy as np, rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from collections import deque

import matplotlib.pyplot as plt


plt.rcParams['font.family'] = 'Noto Sans CJK JP'   # 名称以 fc-list 输出为准
plt.rcParams['axes.unicode_minus'] = False         # 避免负号显示为方框

logging.basicConfig(level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# ---------- 采集相关全局状态 ----------
recording_enabled = False          # 是否正在采集
record_lock       = threading.Lock()
h5f               = None           # 当前 HDF5 句柄
img_ds = img_ts_ds = pose_ds = pose_ts_ds = None

# ---------- 退出标志 ----------
exit_flag = False
def signal_handler(sig, _):                          # Ctrl-C 优雅退出
    global exit_flag
    logger.warning("收到信号 %s, 即将退出…", sig)
    exit_flag = True
signal.signal(signal.SIGINT,  signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# ---------- SLAM 数据缓存 ----------
slam_buf  = deque(maxlen=50)
slam_lock = threading.Lock()

# ---------- 摄像头订阅 ----------
class CameraStream:
    def __init__(self, idx, topic):
        self.idx  = idx
        self.topic= topic
        self._frm = None
        self._lock= threading.Lock()
        self.sub  = rospy.Subscriber(topic, Image, self.on_img, queue_size=1)

    def on_img(self, msg):
        try:
            data = np.frombuffer(msg.data, np.uint8)
            if msg.encoding in ('bgr8', 'rgb8'):
                img = data.reshape(msg.height, msg.width, 3)
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:  # mono8 等
                gray= data.reshape(msg.height, msg.width)
                img = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            with self._lock:
                self._frm = img
        except Exception as e:
            logger.error("Camera%d 解码失败: %s", self.idx, e)

    def frame(self):
        with self._lock:
            return None if self._frm is None else self._frm.copy()

# ---------- ROS SLAM 回调 ----------
def slam_cb(msg: PoseStamped):
    p, o = msg.pose.position, msg.pose.orientation
    rec  = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
    with slam_lock:
        slam_buf.append((time.time(), rec))

# ---------- HDF5 工具 ----------
def _open_h5():
    global h5f, img_ds, img_ts_ds, pose_ds, pose_ts_ds
    ts   = time.strftime("%Y%m%d_%H%M%S")
    outd = "./recorded_data"
    os.makedirs(outd, exist_ok=True)
    h5f  = h5py.File(os.path.join(outd, f"record_{ts}.hdf5"), "w")
    H, W = 1080, 1920            # 可按实际分辨率调整
    img_ds     = h5f.create_dataset("images", (0, H, W, 3),
                 maxshape=(None, H, W, 3), chunks=(1, H, W, 3),
                 dtype="uint8", compression="gzip")
    img_ts_ds  = h5f.create_dataset("img_ts", (0,), maxshape=(None,),
                 dtype="float64")
    pose_ds    = h5f.create_dataset("poses", (0, 7), maxshape=(None, 7),
                 dtype="float32")
    pose_ts_ds = h5f.create_dataset("pose_ts", (0,), maxshape=(None,),
                 dtype="float64")
    logger.info("HDF5 文件已创建: %s", h5f.filename)

def _close_h5():
    """
    关闭当前 HDF5，并将所有句柄重置为 None，防止后续误用
    """
    global h5f, img_ds, img_ts_ds, pose_ds, pose_ts_ds
    if h5f:                      # 先确保真的打开过
        h5f.close()
        logger.info("HDF5 已关闭: %s", h5f.filename)
    # 重置所有句柄
    h5f = img_ds = img_ts_ds = pose_ds = pose_ts_ds = None

# ---------- 后端主体 ----------
class Backend:
    def __init__(self):
        self.app   = web.Application()
        self.ws_set= set()
        self.cams  = {}
        self._setup_routes()

    # ---- 路由 ----
    def _setup_routes(self):
        self.app.router.add_static('/static', './static')
        self.app.router.add_get('/', self.page)
        self.app.router.add_get('/ws', self.ws_handler)
        self.app.router.add_get('/camera/{cid}/stream', self.mjpg)

    async def page(self, _):          # 首页
        return web.FileResponse('./static/index.html')

    async def ws_handler(self, req):
        ws = web.WebSocketResponse()
        await ws.prepare(req)
        self.ws_set.add(ws)
        try:
            async for m in ws:
                if m.type is WSMsgType.TEXT:
                    await self._handle_ws(ws, json.loads(m.data))
        finally:
            self.ws_set.discard(ws)
        return ws

    async def _handle_ws(self, ws, data):
        global recording_enabled
        if data.get("type")=="command":
            action = data.get("action")
            if action=="start_record" and not recording_enabled:
                with record_lock:
                    recording_enabled = True
                    _open_h5()
                await ws.send_json({"type":"record_status","state":"started"})
                logger.info("开始采集")
            elif action=="stop_record" and recording_enabled:
                with record_lock:
                    recording_enabled = False
                    _close_h5()
                await ws.send_json({"type":"record_status","state":"stopped"})
                logger.info("停止采集")
        # 其余指令可在此扩展

    async def mjpg(self, req):
        cid = int(req.match_info['cid'])
        if cid not in self.cams:
            return web.Response(text="Camera not found", status=404)
        resp = web.StreamResponse(status=200,
             headers={'Content-Type':
                      'multipart/x-mixed-replace; boundary=frame'})
        await resp.prepare(req)
        while not exit_flag:
            frm = self.cams[cid].frame()
            if frm is not None:
                _, buf = cv2.imencode('.jpg', frm,[cv2.IMWRITE_JPEG_QUALITY,80])
                await resp.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n")
                await resp.write(buf.tobytes()+b"\r\n")
            await asyncio.sleep(0.033)
        return resp

    # ---- 线程：广播 SLAM ----
    async def _broadcast_slam(self):
        while not exit_flag:
            with slam_lock:
                pkt = slam_buf[-1] if slam_buf else None
            if pkt and self.ws_set:
                _, data = pkt
                for w in list(self.ws_set):
                    try:
                        await w.send_json({"type":"slam","data":data})
                    except:
                        self.ws_set.discard(w)
            await asyncio.sleep(0.1)

    # ---- 线程：写 HDF5 ----
    def _writer(self):
        """
        独立线程：把缓存中的视频帧 & SLAM 位姿写入 HDF5
        采集中才写；没在采集直接 sleep。
        """
        global img_ds, img_ts_ds, pose_ds, pose_ts_ds
        while not exit_flag:
            # ---------- 只在采集状态下工作 ----------
            if recording_enabled and h5f:
                with record_lock:          # 保护共享资源
                    # 由于锁住，所以 stop_record 不会在此期间关闭文件
                    if not (recording_enabled and h5f):
                        # 状态已变，立刻跳过
                        time.sleep(0.01)
                        continue

                    # ---- 写视频帧 ----
                    for cam in self.cams.values():
                        frm = cam.frame()
                        if frm is not None:
                            n = img_ds.shape[0]
                            img_ds.resize(n + 1, 0)
                            img_ds[n] = frm
                            img_ts_ds.resize(n + 1, 0)
                            img_ts_ds[n] = time.time()

                    # ---- 写 SLAM ----
                    with slam_lock:
                        while slam_buf:
                            t, slam = slam_buf.popleft()
                            m = pose_ds.shape[0]
                            pose_ds.resize(m + 1, 0)
                            pose_ds[m] = slam
                            pose_ts_ds.resize(m + 1, 0)
                            pose_ts_ds[m] = t
            else:
                # 未采集，轻量 sleep，避免空转
                time.sleep(0.05)
        with record_lock:
            if h5f:
                _close_h5()

    async def run(self, host="0.0.0.0", port=8080):
        # ROS 初始化
        try:
            rospy.init_node('local_backend', anonymous=True)
        except rospy.ROSException:
            logger.warning("ROS 初始化失败，继续运行")
        # 摄像头发现
        topics = rospy.get_published_topics()
        cams   = [t for t,m in topics if m=="sensor_msgs/Image"]
        for idx, topic in enumerate(cams):
            self.cams[idx] = CameraStream(idx, topic)
            logger.info("检测到摄像头: %s", topic)
        rospy.Subscriber("/xv_sdk/xv_dev/slam/visual_pose",
                         PoseStamped, slam_cb, queue_size=10)
        # aiohttp
        runner = web.AppRunner(self.app); await runner.setup()
        site   = web.TCPSite(runner, host, port); await site.start()
        logger.info("Backend 运行在 http://%s:%d", host, port)
        # 启动后台协程 / 线程
        asyncio.create_task(self._broadcast_slam())
        threading.Thread(target=self._writer, daemon=True).start()
        # 主循环
        while not exit_flag:
            await asyncio.sleep(1)
        await runner.cleanup()

# ---------- 入口 ----------
if __name__ == "__main__":
    asyncio.run(Backend().run())
