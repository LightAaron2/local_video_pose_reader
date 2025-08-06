#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版 HDF5 数据读取脚本
用于快速查看 backend1.py 生成的 HDF5 文件
"""

import h5py
import numpy as np
import matplotlib.pyplot as plt
import cv2
import os
from datetime import datetime

def quick_view(file_path):
    """快速查看 HDF5 文件内容"""
    if not os.path.exists(file_path):
        print(f"✗ 文件不存在: {file_path}")
        return
    
    print(f"正在读取: {file_path}")
    
    with h5py.File(file_path, 'r') as f:
        print("\n=== 数据集概览 ===")
        for key in f.keys():
            dataset = f[key]
            print(f"{key}: {dataset.shape} ({dataset.dtype})")
        
        print("\n=== 详细信息 ===")
        
        # 图像信息
        if 'images' in f:
            images = f['images']
            print(f"图像数量: {len(images)}")
            print(f"图像尺寸: {images.shape[1:]} (H×W×C)")
            print(f"图像大小: {images.nbytes / (1024*1024):.2f} MB")
        
        # 时间戳信息
        if 'img_timestamps' in f:
            img_ts = f['img_timestamps'][:]
            if len(img_ts) > 0:
                start_time = datetime.fromtimestamp(img_ts[0])
                end_time = datetime.fromtimestamp(img_ts[-1])
                duration = img_ts[-1] - img_ts[0]
                print(f"\n图像时间范围:")
                print(f"  开始: {start_time}")
                print(f"  结束: {end_time}")
                print(f"  持续时间: {duration:.2f} 秒")
                print(f"  平均帧率: {len(img_ts)/duration:.2f} FPS")
        
        # 位姿信息
        if 'poses' in f:
            poses = f['poses']
            print(f"\n位姿数量: {len(poses)}")
            print(f"位姿格式: [x, y, z, qx, qy, qz, qw]")
            print(f"位姿大小: {poses.nbytes / (1024*1024):.2f} MB")
        
        if 'pose_timestamps' in f:
            pose_ts = f['pose_timestamps'][:]
            if len(pose_ts) > 0:
                start_time = datetime.fromtimestamp(pose_ts[0])
                end_time = datetime.fromtimestamp(pose_ts[-1])
                duration = pose_ts[-1] - pose_ts[0]
                print(f"\n位姿时间范围:")
                print(f"  开始: {start_time}")
                print(f"  结束: {end_time}")
                print(f"  持续时间: {duration:.2f} 秒")
                print(f"  平均频率: {len(pose_ts)/duration:.2f} Hz")

def show_sample_image(file_path, index=0):
    """显示样本图像"""
    if not os.path.exists(file_path):
        print(f"✗ 文件不存在: {file_path}")
        return
    
    with h5py.File(file_path, 'r') as f:
        if 'images' not in f:
            print("✗ 没有图像数据")
            return
        
        images = f['images']
        if index >= len(images):
            print(f"✗ 索引 {index} 超出范围 (0-{len(images)-1})")
            return
        
        img = images[index]
        print(f"显示图像 #{index}, 形状: {img.shape}")
        
        # 转换为RGB格式显示
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        plt.figure(figsize=(12, 8))
        plt.imshow(img_rgb)
        plt.title(f'样本图像 #{index}')
        plt.axis('off')
        plt.show()

def plot_trajectory(file_path):
    """绘制位姿轨迹"""
    if not os.path.exists(file_path):
        print(f"✗ 文件不存在: {file_path}")
        return
    
    with h5py.File(file_path, 'r') as f:
        if 'poses' not in f:
            print("✗ 没有位姿数据")
            return
        
        poses = f['poses'][:]
        print(f"绘制轨迹，共 {len(poses)} 个位姿")
        
        # 提取位置信息
        positions = poses[:, :3]
        
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # XY 平面轨迹
        ax1.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2)
        ax1.scatter(positions[0, 0], positions[0, 1], c='g', s=100, label='起点')
        ax1.scatter(positions[-1, 0], positions[-1, 1], c='r', s=100, label='终点')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_title('XY 平面轨迹')
        ax1.legend()
        ax1.grid(True)
        
        # 高度变化
        ax2.plot(range(len(positions)), positions[:, 2], 'b-', linewidth=2)
        ax2.set_xlabel('时间步')
        ax2.set_ylabel('Z (高度)')
        ax2.set_title('高度变化')
        ax2.grid(True)
        
        plt.tight_layout()
        plt.show()

def export_basic_data(file_path, output_dir="./exported_basic"):
    """导出基本数据"""
    if not os.path.exists(file_path):
        print(f"✗ 文件不存在: {file_path}")
        return
    
    os.makedirs(output_dir, exist_ok=True)
    print(f"导出数据到: {output_dir}")
    
    with h5py.File(file_path, 'r') as f:
        # 导出位姿数据
        if 'poses' in f:
            poses = f['poses'][:]
            np.save(os.path.join(output_dir, 'poses.npy'), poses)
            print(f"✓ 位姿数据已导出")
        
        # 导出时间戳
        if 'pose_timestamps' in f:
            pose_ts = f['pose_timestamps'][:]
            np.save(os.path.join(output_dir, 'pose_timestamps.npy'), pose_ts)
            print(f"✓ 位姿时间戳已导出")
        
        if 'img_timestamps' in f:
            img_ts = f['img_timestamps'][:]
            np.save(os.path.join(output_dir, 'img_timestamps.npy'), img_ts)
            print(f"✓ 图像时间戳已导出")
        
        # 导出前几张图像作为样本
        if 'images' in f:
            images = f['images']
            sample_count = min(10, len(images))
            img_dir = os.path.join(output_dir, 'sample_images')
            os.makedirs(img_dir, exist_ok=True)
            
            for i in range(sample_count):
                img = images[i]
                cv2.imwrite(os.path.join(img_dir, f'sample_{i:02d}.jpg'), img)
            print(f"✓ {sample_count} 张样本图像已导出")

def main():
    import sys
    
    if len(sys.argv) < 2:
        print("用法:")
        print("  python simple_hdf5_reader.py <hdf5文件路径>")
        print("  python simple_hdf5_reader.py <hdf5文件路径> --view")
        print("  python simple_hdf5_reader.py <hdf5文件路径> --image <索引>")
        print("  python simple_hdf5_reader.py <hdf5文件路径> --trajectory")
        print("  python simple_hdf5_reader.py <hdf5文件路径> --export")
        return
    
    file_path = sys.argv[1]
    
    if '--view' in sys.argv:
        quick_view(file_path)
    elif '--image' in sys.argv:
        try:
            index = int(sys.argv[sys.argv.index('--image') + 1])
            show_sample_image(file_path, index)
        except (ValueError, IndexError):
            show_sample_image(file_path, 0)
    elif '--trajectory' in sys.argv:
        plot_trajectory(file_path)
    elif '--export' in sys.argv:
        export_basic_data(file_path)
    else:
        quick_view(file_path)

if __name__ == "__main__":
    main() 