#!/usr/bin/env python3
"""
SLAM数据读取测试脚本
用于验证ROS话题订阅和SLAM数据处理功能
"""

import rospy
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

def slam_callback(msg):
    """SLAM数据回调函数"""
    pose = msg.pose
    x, y, z = pose.position.x, pose.position.y, pose.position.z
    qx, qy, qz, qw = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
    
    print(f"收到SLAM数据: 位置[{x:.3f}, {y:.3f}, {z:.3f}] 四元数[{qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}]")

def image_callback(msg):
    """图像数据回调函数"""
    print(f"收到图像数据: {msg.width}x{msg.height}, 编码: {msg.encoding}")

def main():
    """主函数"""
    print("启动SLAM数据测试...")
    
    # 初始化ROS节点
    rospy.init_node('slam_test_node', anonymous=True)
    
    # 订阅SLAM数据
    slam_sub = rospy.Subscriber(
        "/xv_sdk/xv_dev/slam/visual_pose", 
        PoseStamped, 
        slam_callback, 
        queue_size=10
    )
    print("已订阅SLAM话题: /xv_sdk/xv_dev/slam/visual_pose")
    
    # 订阅图像数据
    image_sub = rospy.Subscriber(
        "/xv_sdk/xv_dev/color_camera/image_color", 
        Image, 
        image_callback, 
        queue_size=10
    )
    print("已订阅图像话题: /xv_sdk/xv_dev/color_camera/image_color")
    
    print("等待数据... (按Ctrl+C退出)")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n测试结束")
    finally:
        slam_sub.unregister()
        image_sub.unregister()

if __name__ == "__main__":
    main() 