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
trajectory_buffer = deque(maxlen=20)
pose_buffer = deque(maxlen=20)
buffer_lock = threading.Lock()
pose_lock = threading.Lock()

# SLAM数据相关
slam_data_buffer = deque(maxlen=20)
slam_lock = threading.Lock()

# 添加信号处理器
def signal_handler(signum, frame):
    global exit_flag
    print(f"收到信号 {signum}，正在退出...")
    exit_flag = True

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

class CameraStreamTrack:
    def __init__(self, camera_index, topic_name):
        self.camera_index = camera_index
        self.topic_name = topic_name
        self.is_active = True
        self.frame_count = 0
        self.start_time = time.time()
        
        # ROS相关
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # 订阅ROS话题
        try:
            self.subscriber = rospy.Subscriber(
                topic_name, 
                Image, 
                self.image_callback, 
                queue_size=1
            )
            print(f"[Camera{camera_index}] ROS话题订阅成功: {topic_name}")
        except Exception as e:
            logger.error(f"[Camera{camera_index}] ROS话题订阅失败: {e}")
            self.subscriber = None
    
    def image_callback(self, msg):
        """ROS图像回调函数 - 纯numpy实现"""
        try:
            # 直接从ROS消息获取图像数据
            height = msg.height
            width = msg.width
            step = msg.step
            
            # 根据编码格式处理
            if msg.encoding == 'bgr8':
                data = np.frombuffer(msg.data, dtype=np.uint8)
                cv_image = data.reshape((height, width, 3))
                
            elif msg.encoding == 'rgb8':
                data = np.frombuffer(msg.data, dtype=np.uint8)
                rgb_image = data.reshape((height, width, 3))
                cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                
            elif msg.encoding == 'mono8':
                data = np.frombuffer(msg.data, dtype=np.uint8)
                gray_image = data.reshape((height, width))
                cv_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
                
            else:
                # 对于其他编码格式，尝试通用方法
                bytes_per_pixel = step // width
                data = np.frombuffer(msg.data, dtype=np.uint8)
                
                if bytes_per_pixel == 1:
                    cv_image = data.reshape((height, width))
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
                elif bytes_per_pixel == 3:
                    rgb_image = data.reshape((height, width, 3))
                    cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                else:
                    logger.error(f"[Camera{self.camera_index}] 无法处理的图像格式: {msg.encoding}")
                    return
            
            with self.frame_lock:
                self.latest_frame = cv_image
                
        except Exception as e:
            logger.error(f"[Camera{self.camera_index}] 图像处理失败: {e}")
    
    def get_latest_frame(self):
        """获取最新帧"""
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
        return None

    def stop(self):
        """停止摄像头"""
        self.is_active = False
        if hasattr(self, 'subscriber') and self.subscriber:
            try:
                self.subscriber.unregister()
            except Exception as e:
                logger.error(f"[Camera{self.camera_index}] 取消订阅失败: {e}")
        print(f"[Camera{self.camera_index}] 摄像头已停止")

def detect_available_cameras():
    """检测系统中可用的ROS摄像头话题"""
    available_cameras = []
    
    print("正在检测可用的ROS摄像头话题...")
    
    # 检查ROS是否可用
    try:
        # 使用更短的超时时间
        rospy.wait_for_service('/rosout', timeout=2.0)
        print("ROS master连接成功")
    except rospy.ROSException as e:
        logger.warning(f"ROS master连接失败: {e}")
        print("继续尝试获取话题，但可能无法正常工作")
    
    # 获取话题列表，添加超时保护
    try:
        # 使用线程来避免阻塞
        topics = []
        def get_topics():
            nonlocal topics
            try:
                topics = rospy.get_published_topics()
            except Exception as e:
                logger.error(f"获取话题列表失败: {e}")
                topics = []
        
        # 在单独的线程中运行，避免阻塞主线程
        import threading
        thread = threading.Thread(target=get_topics)
        thread.daemon = True
        thread.start()
        thread.join(timeout=5.0)  # 5秒超时
        
        if thread.is_alive():
            logger.warning("获取话题列表超时，使用空列表")
            topics = []
        
        print(f"发现 {len(topics)} 个话题")
        
    except Exception as e:
        logger.error(f"获取话题列表异常: {e}")
        topics = []
    
    # 过滤摄像头话题
    camera_topics = []
    
    # 模式1: /camX/usb_cam/image_raw
    pattern1_topics = [
        (topic, msg_type) for topic, msg_type in topics
        if topic.count('/') == 3 and topic.endswith("/usb_cam/image_raw") 
        and msg_type == 'sensor_msgs/Image'
    ]
    camera_topics.extend(pattern1_topics)
    
    # 模式2: /xv_sdk/xv_dev/color_camera/image_color (XVision SDK)
    pattern2_topics = [
        (topic, msg_type) for topic, msg_type in topics
        if "color_camera" in topic and "image_color" in topic
        and msg_type == 'sensor_msgs/Image'
    ]
    camera_topics.extend(pattern2_topics)
    
    # 模式3: 其他常见的摄像头话题模式
    pattern3_topics = [
        (topic, msg_type) for topic, msg_type in topics
        if any(keyword in topic.lower() for keyword in ["camera", "cam", "image"]) 
        and msg_type == 'sensor_msgs/Image'
        and topic not in [t[0] for t in camera_topics]
    ]
    camera_topics.extend(pattern3_topics)
    
    # 去重并排序
    unique_topics = list(set(camera_topics))
    unique_topics.sort(key=lambda x: x[0])
    
    for i, (topic, msg_type) in enumerate(unique_topics):
        print(f"✓ 发现摄像头话题 {i}: {topic}")
        
        camera_config = {
            'index': i,
            'topic_name': topic,
            'msg_type': msg_type,
            'display_name': f"ROS-Camera-{i}"
        }
        
        available_cameras.append(camera_config)
    
    print(f"检测完成！找到 {len(available_cameras)} 个摄像头话题")
    return available_cameras

def slam_data_callback(msg):
    """SLAM数据回调函数"""
    global slam_data_buffer
    
    try:
        # 从PoseStamped消息中提取位置和姿态
        pose = msg.pose
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        
        # 构建SLAM数据 [x, y, z, qx, qy, qz, qw]
        slam_data = [x, y, z, qx, qy, qz, qw]
        
        with slam_lock:
            slam_data_buffer.append((time.time(), slam_data))
            
        logger.debug(f"收到SLAM数据: [{x:.3f}, {y:.3f}, {z:.3f}]")
        
    except Exception as e:
        logger.error(f"处理SLAM数据失败: {e}")

def video_callback(msg):
    """摄像头数据回调函数 - 用于夹爪检测"""
    try:
        # 直接从ROS消息获取图像数据
        height = msg.height
        width = msg.width
        step = msg.step
        
        # 根据编码格式处理
        if msg.encoding == 'bgr8':
            data = np.frombuffer(msg.data, dtype=np.uint8)
            cv_image = data.reshape((height, width, 3))
            
        elif msg.encoding == 'rgb8':
            data = np.frombuffer(msg.data, dtype=np.uint8)
            rgb_image = data.reshape((height, width, 3))
            cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            
        elif msg.encoding == 'mono8':
            data = np.frombuffer(msg.data, dtype=np.uint8)
            gray_image = data.reshape((height, width))
            cv_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
            
        else:
            # 对于其他编码格式，尝试通用方法
            bytes_per_pixel = step // width
            data = np.frombuffer(msg.data, dtype=np.uint8)
            
            if bytes_per_pixel == 1:
                cv_image = data.reshape((height, width))
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            elif bytes_per_pixel == 3:
                rgb_image = data.reshape((height, width, 3))
                cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            else:
                logger.error(f"无法处理的图像格式: {msg.encoding}")
                return
        
        # 这里可以添加夹爪检测逻辑
        # gripper_width = get_gripper_width(cv_image)
        # gripper_raw = (gripper_width / max_gripper_width)
        
        logger.debug("摄像头数据处理完成")
        
    except Exception as e:
        logger.error(f"摄像头数据处理失败: {e}")

class LocalBackend:
    def __init__(self):
        self.app = web.Application()
        self.cameras = {}
        self.websockets = set()
        self.runner = None
        self.site = None
        self.setup_routes()
        
    def setup_routes(self):
        """设置路由"""
        # 静态文件服务
        self.app.router.add_static('/static', path='./static', name='static')
        
        # WebSocket端点
        self.app.router.add_get('/ws', self.websocket_handler)
        
        # 主页
        self.app.router.add_get('/', self.index_handler)
        
        # 摄像头图像端点
        self.app.router.add_get('/camera/{camera_id}/frame', self.camera_frame_handler)
        
        # 摄像头视频流端点
        self.app.router.add_get('/camera/{camera_id}/stream', self.camera_stream_handler)
    
    async def index_handler(self, request):
        """主页处理器"""
        return web.FileResponse('./static/index.html')
    
    async def websocket_handler(self, request):
        """WebSocket处理器"""
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        
        self.websockets.add(ws)
        print(f"新的WebSocket连接，当前连接数: {len(self.websockets)}")
        
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        await self.handle_websocket_message(ws, data)
                    except json.JSONDecodeError:
                        logger.error(f"WebSocket消息解析失败: {msg.data}")
                elif msg.type == WSMsgType.ERROR:
                    logger.error(f"WebSocket错误: {ws.exception()}")
        except Exception as e:
            logger.error(f"WebSocket处理异常: {e}")
        finally:
            self.websockets.discard(ws)
            print(f"WebSocket连接关闭，当前连接数: {len(self.websockets)}")
        
        return ws
    
    async def handle_websocket_message(self, ws, data):
        """处理WebSocket消息"""
        global trajectory_buffer
        
        if data.get("type") == "trajectory":
            # 处理trajectory数据
            trajectory_data = data.get("data")
            if isinstance(trajectory_data, list) and len(trajectory_data) >= 6:
                with buffer_lock:
                    trajectory_buffer.append((time.time(), trajectory_data))
                logger.debug(f"收到trajectory数据: {trajectory_data}")
        
        elif data.get("type") == "ping":
            # 处理ping消息
            ping_id = data.get("ping_id")
            sent_at = data.get("sent_at")
            if ping_id and sent_at:
                pong_msg = {
                    "type": "pong",
                    "ping_id": ping_id,
                    "sent_at": sent_at
                }
                await ws.send_json(pong_msg)
    
    async def camera_frame_handler(self, request):
        """摄像头图像处理器"""
        #camera_id = request.match_info['camera_id']
        try:
            camera_id = int(request.match_info['camera_id'])
        except ValueError:
            return web.Response(status=400, text="Invalid camera id")
        
        if camera_id not in self.cameras:
            return web.Response(status=404, text="Camera not found")
        
        camera = self.cameras[camera_id]
        frame = camera.get_latest_frame()
        
        if frame is None:
            return web.Response(status=404, text="No frame available")
        
        # 编码图像为JPEG
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        
        return web.Response(
            body=buffer.tobytes(),
            content_type='image/jpeg'
        )
    
    async def camera_stream_handler(self, request):
        """摄像头视频流处理器 - MJPEG流"""
        #camera_id = request.match_info['camera_id']
        
        try:
            camera_id = int(request.match_info['camera_id'])
        except ValueError:
            return web.Response(status=400, text="Invalid camera ID")

        if camera_id not in self.cameras:
            return web.Response(status=404, text="Camera not found")
        
        camera = self.cameras[camera_id]
        
        # 设置MJPEG流的响应头
        response = web.StreamResponse(
            status=200,
            reason='OK',
            headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                'Cache-Control': 'no-cache',
                'Connection': 'close',
            }
        )
        
        await response.prepare(request)
        
        try:
            while not exit_flag:
                frame = camera.get_latest_frame()
                if frame is not None:
                    # 编码图像为JPEG
                    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    
                    # 发送MJPEG帧
                    await response.write(
                        f'--frame\r\n'
                        f'Content-Type: image/jpeg\r\n'
                        f'Content-Length: {len(buffer)}\r\n'
                        f'\r\n'.encode()
                    )
                    await response.write(buffer.tobytes())
                    await response.write(b'\r\n')
                
                await asyncio.sleep(0.033)  # ~30 FPS
                
        except Exception as e:
            logger.error(f"视频流处理异常: {e}")
        finally:
            await response.write_eof()
        
        return response
    
    async def broadcast_slam_data(self):
        """广播SLAM数据到所有WebSocket客户端"""
        while not exit_flag:
            try:
                with slam_lock:
                    slam_buffer_list = list(slam_data_buffer)
                
                if slam_buffer_list and self.websockets:
                    _, latest_slam = slam_buffer_list[-1]
                    if isinstance(latest_slam, np.ndarray):
                        latest_slam = latest_slam.tolist()
                    
                    message = {
                        "type": "slam",
                        "data": latest_slam
                    }
                    
                    # 广播到所有连接的客户端
                    for ws in list(self.websockets):
                        try:
                            await ws.send_json(message)
                        except Exception as e:
                            logger.error(f"发送SLAM数据失败: {e}")
                            self.websockets.discard(ws)
                
                await asyncio.sleep(0.1)  # 10Hz发送频率
                
            except Exception as e:
                logger.error(f"广播SLAM数据时出错: {e}")
                await asyncio.sleep(1)
    
    def init_cameras(self):
        """初始化摄像头"""
        try:
            available_cameras = detect_available_cameras()
            
            for camera_config in available_cameras:
                camera = CameraStreamTrack(
                    camera_config['index'],
                    camera_config['topic_name']
                )
                self.cameras[camera_config['index']] = camera
                print(f"初始化摄像头 {camera_config['index']}: {camera_config['topic_name']}")
        except Exception as e:
            logger.error(f"初始化摄像头失败: {e}")
    
    async def run(self, host='localhost', port=8080):
        """运行服务器"""
        try:
            # 初始化摄像头
            self.init_cameras()

            # 启动SLAM数据广播任务
            asyncio.create_task(self.broadcast_slam_data())

            # 启动ROS订阅
            self.init_ros_subscribers()

            print(f"本地后端服务器启动在 http://{host}:{port}")

            # 用aiohttp推荐的异步方式启动
            self.runner = web.AppRunner(self.app)
            await self.runner.setup()
            self.site = web.TCPSite(self.runner, host, port)
            await self.site.start()

            print("服务器启动成功，开始主循环")
            # 阻塞住主协程，直到进程终止
            while not exit_flag:
                await asyncio.sleep(1)
                
        except Exception as e:
            logger.error(f"服务器运行异常: {e}")
            import traceback
            traceback.print_exc()
        finally:
            await self.cleanup()
    
    async def cleanup(self):
        """清理资源"""
        print("正在清理资源...")
        
        # 停止摄像头
        for camera in self.cameras.values():
            camera.stop()
        
        # 关闭WebSocket连接
        for ws in list(self.websockets):
            try:
                await ws.close()
            except Exception as e:
                logger.error(f"关闭WebSocket失败: {e}")
        
        # 停止服务器
        if self.site:
            await self.site.stop()
        if self.runner:
            await self.runner.cleanup()
        
        print("资源清理完成")
    
    def init_ros_subscribers(self):
        """初始化ROS订阅器"""
        try:
            # 订阅SLAM数据
            rospy.Subscriber(
                "/xv_sdk/xv_dev/slam/visual_pose", 
                PoseStamped, 
                slam_data_callback, 
                queue_size=100
            )
            print("SLAM数据订阅器初始化成功")
            
            # 订阅摄像头数据（如果需要夹爪检测）
            rospy.Subscriber(
                "/xv_sdk/xv_dev/color_camera/image_color", 
                Image, 
                video_callback, 
                queue_size=100
            )
            print("摄像头数据订阅器初始化成功")
            
        except Exception as e:
            logger.error(f"ROS订阅器初始化失败: {e}")

async def main():
    """主函数"""
    global exit_flag
    
    # 初始化ROS节点
    try:
        rospy.init_node('local_backend_node', anonymous=True)
        print("ROS节点初始化成功")
    except Exception as ros_error:
        logger.error(f"ROS节点初始化失败: {ros_error}")
        print("继续运行，但ROS功能可能不可用")
    
    # 创建并运行后端服务
    backend = LocalBackend()
    
    try:
        await backend.run()
    except KeyboardInterrupt:
        print("收到中断信号，正在退出...")
    finally:
        exit_flag = True
        print("程序已退出")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序异常退出: {e}") 