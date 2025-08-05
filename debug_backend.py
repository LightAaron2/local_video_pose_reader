#!/usr/bin/env python3
"""
调试版本的backend.py
逐步测试各个组件以定位问题
"""

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

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

print("=== 开始调试 ===")
print("1. 导入完成")

# 全局变量
exit_flag = False
trajectory_buffer = deque(maxlen=20)
pose_buffer = deque(maxlen=20)
buffer_lock = threading.Lock()
pose_lock = threading.Lock()

# SLAM数据相关
slam_data_buffer = deque(maxlen=20)
slam_lock = threading.Lock()

print("2. 全局变量初始化完成")

# 添加信号处理器
def signal_handler(signum, frame):
    global exit_flag
    print(f"收到信号 {signum}，正在退出...")
    exit_flag = True

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

print("3. 信号处理器设置完成")

def slam_data_callback(msg):
    """SLAM数据回调函数"""
    global slam_data_buffer
    
    try:
        # 从PoseStamped消息中提取位置和姿态
        pose = msg.pose
        # print(f"pose: {pose}")
        x, y, z = pose.position.x, pose.position.y, pose.position.z
        qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        
        # 构建SLAM数据 [x, y, z, qx, qy, qz, qw]
        slam_data = [x, y, z, qx, qy, qz, qw]
        # print(f"slam_data: {slam_data}")
        with slam_lock:
            slam_data_buffer.append((time.time(), slam_data))
            
        logger.debug(f"收到SLAM数据: [{x:.3f}, {y:.3f}, {z:.3f}]")
        
    except Exception as e:
        logger.error(f"处理SLAM数据失败: {e}")

print("4. 回调函数定义完成")

class CameraStreamTrack:
    def __init__(self, camera_index, topic_name, is_mock=False):
        self.camera_index = camera_index
        self.topic_name = topic_name
        self.is_active = True
        self.frame_count = 0
        self.start_time = time.time()
        self.is_mock = is_mock
        
        # ROS相关
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        if is_mock:
            # 创建模拟图像
            self.create_mock_frame()
            print(f"[Camera{camera_index}] 模拟摄像头初始化完成")
        else:
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
                # 如果ROS订阅失败，转为模拟模式
                self.is_mock = True
                self.create_mock_frame()
                print(f"[Camera{camera_index}] 转为模拟模式")
    
    def create_mock_frame(self):
        """创建模拟图像帧"""
        # 创建一个彩色的测试图像
        height, width = 480, 640
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        
        # 添加一些彩色条纹
        for i in range(0, height, 40):
            color = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)][(i // 40) % 5]
            frame[i:i+40, :] = color
        
        # 添加文字
        font = cv2.FONT_HERSHEY_SIMPLEX
        text = f"Mock Camera {self.camera_index}"
        cv2.putText(frame, text, (50, height//2), font, 1, (255, 255, 255), 2)
        
        # 添加时间戳
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(frame, timestamp, (50, height-50), font, 0.7, (255, 255, 255), 2)
        
        with self.frame_lock:
            self.latest_frame = frame
    
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
            if self.is_mock:
                # 模拟摄像头需要动态更新
                self.create_mock_frame()
            
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

def detect_available_cameras(ros_available=False):
    """检测系统中可用的ROS摄像头话题"""
    available_cameras = []
    
    print("正在检测可用的摄像头...")
    
    if not ros_available:
        print("ROS不可用，创建模拟摄像头")
        # 创建模拟摄像头
        mock_cameras = [
            {'index': '0', 'topic': '/mock/camera/left', 'type': 'mock', 'is_mock': True},
            {'index': '1', 'topic': '/mock/camera/right', 'type': 'mock', 'is_mock': True}
        ]
        return mock_cameras
    
    # 检查ROS是否可用
    try:
        # 使用更短的超时时间
        rospy.wait_for_service('/rosout', timeout=2.0)
        print("ROS master连接成功")
    except rospy.ROSException as e:
        logger.warning(f"ROS master连接失败: {e}")
        return available_cameras
    
    try:
        def get_topics():
            try:
                # 获取所有话题
                topics = rospy.get_published_topics()
                return topics
            except Exception as e:
                logger.error(f"获取ROS话题失败: {e}")
                return []
        
        # 获取所有话题
        topics = get_topics()
        
        # 查找摄像头相关话题
        camera_topics = []
        for topic_name, topic_type in topics:
            if 'image' in topic_name.lower() or 'camera' in topic_name.lower():
                if 'Image' in topic_type:
                    camera_topics.append((topic_name, topic_type))
        
        # 为每个摄像头话题创建摄像头对象
        for i, (topic_name, topic_type) in enumerate(camera_topics):
            camera_info = {
                'index': str(i),
                'topic': topic_name,
                'type': topic_type
            }
            available_cameras.append(camera_info)
            print(f"发现摄像头 {i}: {topic_name} ({topic_type})")
        
        if not available_cameras:
            print("未发现摄像头话题，创建默认摄像头")
            # 创建默认摄像头
            default_cameras = [
                {'index': '0', 'topic': '/xv_sdk/xv_dev/camera/left/image_raw', 'type': 'sensor_msgs/Image'},
                {'index': '1', 'topic': '/xv_sdk/xv_dev/camera/right/image_raw', 'type': 'sensor_msgs/Image'}
            ]
            available_cameras = default_cameras
        
    except Exception as e:
        logger.error(f"检测摄像头时出错: {e}")
    
    return available_cameras

print("5. 摄像头类定义完成")

class LocalBackend:
    def __init__(self):
        print("5. 开始初始化LocalBackend")
        self.app = web.Application()
        self.cameras = {}
        self.websockets = set()
        self.runner = None
        self.site = None
        self.setup_routes()
        print("6. LocalBackend初始化完成")
        
    def setup_routes(self):
        """设置路由"""
        print("7. 开始设置路由")
        # 静态文件服务
        self.app.router.add_static('/static', path='/home/onestar/Documents/Git/local_video&pose_reader/local_video&pose_reader/static', name='static')
        
        # WebSocket端点
        self.app.router.add_get('/ws', self.websocket_handler)
        
        # 主页
        self.app.router.add_get('/', self.index_handler)
        
        # 简单SLAM监控页面
        self.app.router.add_get('/simple', self.simple_handler)
        
        # 视频流测试页面
        self.app.router.add_get('/test', self.test_handler)
        
        # 摄像头图像端点
        self.app.router.add_get('/camera/{camera_id}/frame', self.camera_frame_handler)
        
        # 摄像头视频流端点
        self.app.router.add_get('/camera/{camera_id}/stream', self.camera_stream_handler)
        
        # 摄像头状态检查端点
        self.app.router.add_get('/camera/status', self.camera_status_handler)
        
        print("8. 路由设置完成")
    
    async def index_handler(self, request):
        """主页处理器"""
        return web.FileResponse('./static/index.html')
    
    async def simple_handler(self, request):
        """简单SLAM监控页面处理器"""
        return web.FileResponse('./static/simple_slam.html')
    
    async def test_handler(self, request):
        """视频流测试页面处理器"""
        return web.FileResponse('./static/test_video.html')
    
    async def camera_frame_handler(self, request):
        """摄像头图像处理器"""
        camera_id = request.match_info['camera_id']
        
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
        camera_id = request.match_info['camera_id']
        
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
    
    async def camera_status_handler(self, request):
        """摄像头状态检查处理器"""
        status = {
            'cameras': {},
            'total': len(self.cameras)
        }
        
        for camera_id, camera in self.cameras.items():
            frame = camera.get_latest_frame()
            status['cameras'][camera_id] = {
                'topic': camera.topic_name,
                'has_frame': frame is not None,
                'frame_shape': frame.shape if frame is not None else None,
                'is_active': camera.is_active
            }
        
        return web.json_response(status)
    
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
    
    async def broadcast_slam_data(self):
        """广播SLAM数据到所有WebSocket客户端"""
        print("9. 开始SLAM数据广播任务")
        while not exit_flag:
            try:
                with slam_lock:
                    slam_buffer_list = list(slam_data_buffer)
                    # print(f"SLAM数据缓冲区长度: {len(slam_buffer_list)}")
                
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
                            print(f"发送SLAM数据: {latest_slam}")
                        except Exception as e:
                            logger.error(f"发送SLAM数据失败: {e}")
                            self.websockets.discard(ws)
                
                await asyncio.sleep(0.1)  # 10Hz发送频率
                
            except Exception as e:
                logger.error(f"广播SLAM数据时出错: {e}")
                await asyncio.sleep(1)
    
    def video_callback(self, msg):
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

    def init_ros_subscribers(self):
        """初始化ROS订阅器"""
        print("10. 开始初始化ROS订阅器")
        try:
            # 订阅SLAM数据
            rospy.Subscriber(
                "/xv_sdk/xv_dev/slam/visual_pose", 
                PoseStamped, 
                slam_data_callback, 
                queue_size=100
            )
            print("SLAM数据订阅器初始化成功")

            rospy.Subscriber(
                "/xv_sdk/xv_dev/color_camera/image_color", 
                Image, 
                self.video_callback, 
                queue_size=100
            )
            print("摄像头数据订阅器初始化成功")
            
        except Exception as e:
            logger.error(f"ROS订阅器初始化失败: {e}")
        print("11. ROS订阅器初始化完成")

    
    
    def init_cameras(self, ros_available=False):
        """初始化摄像头"""
        print("12. 开始初始化摄像头")
        try:
            # 检测可用的摄像头
            available_cameras = detect_available_cameras(ros_available)
            
            # 初始化摄像头对象
            for camera_info in available_cameras:
                camera_index = camera_info['index']
                topic_name = camera_info['topic']
                is_mock = camera_info.get('is_mock', False)
                
                camera = CameraStreamTrack(
                    camera_index=camera_index,
                    topic_name=topic_name,
                    is_mock=is_mock
                )
                
                self.cameras[camera_index] = camera
                print(f"摄像头 {camera_index} 初始化完成: {topic_name} {'(模拟)' if is_mock else ''}")
            
            print(f"共初始化 {len(self.cameras)} 个摄像头")
            
        except Exception as e:
            logger.error(f"摄像头初始化失败: {e}")
        print("13. 摄像头初始化完成")
    
    async def run(self, host='localhost', port=8080, ros_available=False):
        """运行服务器"""
        print("14. 开始运行服务器")
        try:
            # 启动SLAM数据广播任务
            asyncio.create_task(self.broadcast_slam_data())

            if ros_available:
                # 启动ROS订阅
                self.init_ros_subscribers()
                # 等待一段时间让ROS话题稳定
                await asyncio.sleep(2)
            else:
                print("ROS不可用，跳过ROS订阅初始化")

            # 初始化摄像头
            self.init_cameras(ros_available)

            print(f"本地后端服务器启动在 http://{host}:{port}")

            # 用aiohttp推荐的异步方式启动
            self.runner = web.AppRunner(self.app)
            await self.runner.setup()
            self.site = web.TCPSite(self.runner, host, port)
            await self.site.start()

            print("15. 服务器启动成功，开始主循环")
            # 阻塞住主协程，直到进程终止
            while not exit_flag:
                await asyncio.sleep(1)
                
        except Exception as e:
            logger.error(f"服务器运行异常: {e}")
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

async def main():
    """主函数"""
    global exit_flag
    
    print("16. 开始main函数")
    
    # 初始化ROS节点
    ros_available = False
    try:
        rospy.init_node('local_backend_node', anonymous=True)
        print("ROS节点初始化成功")
        print("17. ROS节点初始化成功")
        ros_available = True
    except Exception as ros_error:
        logger.error(f"ROS节点初始化失败: {ros_error}")
        print("继续运行，但ROS功能可能不可用")
        print("17. ROS节点初始化失败，但继续运行")
        ros_available = False
    
    # 创建并运行后端服务
    backend = LocalBackend()
    
    try:
        print("18. 开始运行后端服务")
        await backend.run(ros_available=ros_available)
    except KeyboardInterrupt:
        print("收到中断信号，正在退出...")
    finally:
        exit_flag = True
        print("程序已退出")

if __name__ == "__main__":
    print("19. 程序入口点")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序异常退出: {e}")
        import traceback
        traceback.print_exc() 