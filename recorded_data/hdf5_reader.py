#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HDF5 数据读取脚本
用于读取和可视化 backend1.py 生成的 HDF5 文件
"""

import h5py
import numpy as np
import matplotlib.pyplot as plt
import cv2
import argparse
import os
from datetime import datetime
import json

class HDF5Reader:
    def __init__(self, file_path):
        """初始化 HDF5 读取器"""
        self.file_path = file_path
        self.h5f = None
        self.datasets = {}
        
    def open_file(self):
        """打开 HDF5 文件"""
        try:
            self.h5f = h5py.File(self.file_path, 'r')
            print(f"✓ 成功打开文件: {self.file_path}")
            return True
        except Exception as e:
            print(f"✗ 打开文件失败: {e}")
            return False
    
    def close_file(self):
        """关闭 HDF5 文件"""
        if self.h5f:
            self.h5f.close()
            print("✓ 文件已关闭")
    
    def get_dataset_info(self):
        """获取数据集信息"""
        if not self.h5f:
            print("✗ 文件未打开")
            return
        
        print("\n=== 数据集信息 ===")
        for key in self.h5f.keys():
            dataset = self.h5f[key]
            print(f"数据集: {key}")
            print(f"  形状: {dataset.shape}")
            print(f"  数据类型: {dataset.dtype}")
            print(f"  压缩: {dataset.compression}")
            if hasattr(dataset, 'chunks'):
                print(f"  分块: {dataset.chunks}")
            print()
    
    def get_data_summary(self):
        """获取数据摘要"""
        if not self.h5f:
            return
        
        summary = {}
        for key in self.h5f.keys():
            dataset = self.h5f[key]
            summary[key] = {
                'shape': dataset.shape,
                'dtype': str(dataset.dtype),
                'size_mb': dataset.nbytes / (1024 * 1024)
            }
        
        print("\n=== 数据摘要 ===")
        total_size = 0
        for key, info in summary.items():
            print(f"{key}:")
            print(f"  形状: {info['shape']}")
            print(f"  数据类型: {info['dtype']}")
            print(f"  大小: {info['size_mb']:.2f} MB")
            total_size += info['size_mb']
            print()
        
        print(f"总文件大小: {total_size:.2f} MB")
        return summary
    
    def get_timestamp_range(self):
        """获取时间戳范围"""
        if not self.h5f:
            return
        
        print("\n=== 时间戳范围 ===")
        
        if 'img_timestamps' in self.h5f:
            img_ts = self.h5f['img_timestamps'][:]
            if len(img_ts) > 0:
                start_time = datetime.fromtimestamp(img_ts[0])
                end_time = datetime.fromtimestamp(img_ts[-1])
                duration = img_ts[-1] - img_ts[0]
                print(f"图像时间戳:")
                print(f"  开始: {start_time}")
                print(f"  结束: {end_time}")
                print(f"  持续时间: {duration:.2f} 秒")
                print(f"  图像数量: {len(img_ts)}")
                print()
        
        if 'pose_timestamps' in self.h5f:
            pose_ts = self.h5f['pose_timestamps'][:]
            if len(pose_ts) > 0:
                start_time = datetime.fromtimestamp(pose_ts[0])
                end_time = datetime.fromtimestamp(pose_ts[-1])
                duration = pose_ts[-1] - pose_ts[0]
                print(f"位姿时间戳:")
                print(f"  开始: {start_time}")
                print(f"  结束: {end_time}")
                print(f"  持续时间: {duration:.2f} 秒")
                print(f"  位姿数量: {len(pose_ts)}")
                print()
    
    def show_image(self, index=0, save_path=None):
        """显示指定索引的图像"""
        if not self.h5f or 'images' not in self.h5f:
            print("✗ 图像数据集不存在")
            return
        
        images = self.h5f['images']
        if index >= len(images):
            print(f"✗ 索引 {index} 超出范围 (0-{len(images)-1})")
            return
        
        img = images[index]
        print(f"显示图像 #{index}, 形状: {img.shape}")
        
        # OpenCV 使用 BGR 格式，matplotlib 使用 RGB 格式
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        plt.figure(figsize=(12, 8))
        plt.imshow(img_rgb)
        plt.title(f'图像 #{index}')
        plt.axis('off')
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✓ 图像已保存到: {save_path}")
        else:
            plt.show()
        plt.close()
    
    def show_pose_trajectory(self, save_path=None):
        """显示位姿轨迹"""
        if not self.h5f or 'poses' not in self.h5f:
            print("✗ 位姿数据集不存在")
            return
        
        poses = self.h5f['poses'][:]
        print(f"显示位姿轨迹，共 {len(poses)} 个位姿")
        
        # 提取位置信息 (前3个元素是 x, y, z)
        positions = poses[:, :3]
        
        fig = plt.figure(figsize=(15, 10))
        
        # 3D 轨迹图
        ax1 = fig.add_subplot(221, projection='3d')
        ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', linewidth=2)
        ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c='g', s=100, label='起点')
        ax1.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='r', s=100, label='终点')
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        ax1.set_title('3D 轨迹')
        ax1.legend()
        
        # XY 平面投影
        ax2 = fig.add_subplot(222)
        ax2.plot(positions[:, 0], positions[:, 1], 'b-', linewidth=2)
        ax2.scatter(positions[0, 0], positions[0, 1], c='g', s=100, label='起点')
        ax2.scatter(positions[-1, 0], positions[-1, 1], c='r', s=100, label='终点')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_title('XY 平面投影')
        ax2.legend()
        ax2.grid(True)
        
        # XZ 平面投影
        ax3 = fig.add_subplot(223)
        ax3.plot(positions[:, 0], positions[:, 2], 'b-', linewidth=2)
        ax3.scatter(positions[0, 0], positions[0, 2], c='g', s=100, label='起点')
        ax3.scatter(positions[-1, 0], positions[-1, 2], c='r', s=100, label='终点')
        ax3.set_xlabel('X')
        ax3.set_ylabel('Z')
        ax3.set_title('XZ 平面投影')
        ax3.legend()
        ax3.grid(True)
        
        # YZ 平面投影
        ax4 = fig.add_subplot(224)
        ax4.plot(positions[:, 1], positions[:, 2], 'b-', linewidth=2)
        ax4.scatter(positions[0, 1], positions[0, 2], c='g', s=100, label='起点')
        ax4.scatter(positions[-1, 1], positions[-1, 2], c='r', s=100, label='终点')
        ax4.set_xlabel('Y')
        ax4.set_ylabel('Z')
        ax4.set_title('YZ 平面投影')
        ax4.legend()
        ax4.grid(True)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"✓ 轨迹图已保存到: {save_path}")
        else:
            plt.show()
        plt.close()
    
    def export_data(self, output_dir="./exported_data"):
        """导出数据到其他格式"""
        if not self.h5f:
            print("✗ 文件未打开")
            return
        
        os.makedirs(output_dir, exist_ok=True)
        print(f"导出数据到: {output_dir}")
        
        # 导出时间戳信息
        if 'img_timestamps' in self.h5f:
            img_ts = self.h5f['img_timestamps'][:]
            np.save(os.path.join(output_dir, 'img_timestamps.npy'), img_ts)
            print(f"✓ 图像时间戳已导出")
        
        if 'pose_timestamps' in self.h5f:
            pose_ts = self.h5f['pose_timestamps'][:]
            np.save(os.path.join(output_dir, 'pose_timestamps.npy'), pose_ts)
            print(f"✓ 位姿时间戳已导出")
        
        # 导出位姿数据
        if 'poses' in self.h5f:
            poses = self.h5f['poses'][:]
            np.save(os.path.join(output_dir, 'poses.npy'), poses)
            print(f"✓ 位姿数据已导出")
        
        # 导出图像 (可选，因为文件可能很大)
        if 'images' in self.h5f:
            images = self.h5f['images']
            if len(images) <= 100:  # 只导出前100张图像
                img_dir = os.path.join(output_dir, 'images')
                os.makedirs(img_dir, exist_ok=True)
                for i in range(min(len(images), 100)):
                    img = images[i]
                    cv2.imwrite(os.path.join(img_dir, f'image_{i:04d}.jpg'), img)
                print(f"✓ 前 {min(len(images), 100)} 张图像已导出")
            else:
                print(f"⚠ 图像数量过多 ({len(images)})，跳过图像导出")
        
        # 导出数据摘要
        summary = self.get_data_summary()
        with open(os.path.join(output_dir, 'summary.json'), 'w', encoding='utf-8') as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        print(f"✓ 数据摘要已导出")
    
    def interactive_mode(self):
        """交互模式"""
        print("\n=== 交互模式 ===")
        print("命令:")
        print("  info - 显示数据集信息")
        print("  summary - 显示数据摘要")
        print("  time - 显示时间戳范围")
        print("  show <index> - 显示指定索引的图像")
        print("  trajectory - 显示位姿轨迹")
        print("  export - 导出数据")
        print("  quit - 退出")
        
        while True:
            try:
                cmd = input("\n请输入命令: ").strip().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'quit':
                    break
                elif cmd[0] == 'info':
                    self.get_dataset_info()
                elif cmd[0] == 'summary':
                    self.get_data_summary()
                elif cmd[0] == 'time':
                    self.get_timestamp_range()
                elif cmd[0] == 'show':
                    if len(cmd) > 1:
                        try:
                            index = int(cmd[1])
                            self.show_image(index)
                        except ValueError:
                            print("✗ 无效的索引")
                    else:
                        self.show_image(0)
                elif cmd[0] == 'trajectory':
                    self.show_pose_trajectory()
                elif cmd[0] == 'export':
                    self.export_data()
                else:
                    print("✗ 未知命令")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"✗ 错误: {e}")

def main():
    parser = argparse.ArgumentParser(description='HDF5 数据读取工具')
    parser.add_argument('file_path', help='HDF5 文件路径')
    parser.add_argument('--info', action='store_true', help='显示数据集信息')
    parser.add_argument('--summary', action='store_true', help='显示数据摘要')
    parser.add_argument('--time', action='store_true', help='显示时间戳范围')
    parser.add_argument('--show-image', type=int, metavar='INDEX', help='显示指定索引的图像')
    parser.add_argument('--trajectory', action='store_true', help='显示位姿轨迹')
    parser.add_argument('--export', action='store_true', help='导出数据')
    parser.add_argument('--interactive', '-i', action='store_true', help='进入交互模式')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.file_path):
        print(f"✗ 文件不存在: {args.file_path}")
        return
    
    reader = HDF5Reader(args.file_path)
    
    if not reader.open_file():
        return
    
    try:
        if args.info:
            reader.get_dataset_info()
        if args.summary:
            reader.get_data_summary()
        if args.time:
            reader.get_timestamp_range()
        if args.show_image is not None:
            reader.show_image(args.show_image)
        if args.trajectory:
            reader.show_pose_trajectory()
        if args.export:
            reader.export_data()
        if args.interactive or not any([args.info, args.summary, args.time, args.show_image is not None, args.trajectory, args.export]):
            reader.interactive_mode()
    finally:
        reader.close_file()

if __name__ == "__main__":
    main() 