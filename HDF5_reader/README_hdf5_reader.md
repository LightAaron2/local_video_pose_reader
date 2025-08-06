# HDF5 数据读取工具

这个工具用于读取和可视化 `backend1.py` 生成的 HDF5 文件。

## 文件结构

你的 HDF5 文件包含以下数据集：

- `images`: 图像数据 (形状: N, 1080, 1920, 3)
- `img_timestamps`: 图像时间戳 (形状: N,)
- `poses`: SLAM 位姿数据 (形状: M, 7) - 包含位置(x,y,z)和四元数(x,y,z,w)
- `pose_timestamps`: 位姿时间戳 (形状: M,)

## 脚本说明

### 1. `hdf5_reader.py` - 完整功能版本

功能最全面的读取工具，支持交互模式。

#### 安装依赖

```bash
pip install h5py numpy matplotlib opencv-python
```

#### 使用方法

**基本用法：**

```bash
# 进入交互模式
python hdf5_reader.py ./recorded_data/record.hdf5

# 显示数据集信息
python hdf5_reader.py ./recorded_data/record.hdf5 --info

# 显示数据摘要
python hdf5_reader.py ./recorded_data/record.hdf5 --summary

# 显示时间戳范围
python hdf5_reader.py ./recorded_data/record.hdf5 --time

# 显示指定索引的图像
python hdf5_reader.py ./recorded_data/record.hdf5 --show-image 5

# 显示位姿轨迹
python hdf5_reader.py ./recorded_data/record.hdf5 --trajectory

# 导出数据
python hdf5_reader.py ./recorded_data/record.hdf5 --export
```

**交互模式命令：**

- `info` - 显示数据集信息
- `summary` - 显示数据摘要
- `time` - 显示时间戳范围
- `show <index>` - 显示指定索引的图像
- `trajectory` - 显示位姿轨迹
- `export` - 导出数据
- `quit` - 退出

### 2. `simple_hdf5_reader.py` - 简化版本

轻量级工具，适合快速查看。

#### 使用方法

```bash
# 快速查看文件内容
python simple_hdf5_reader.py ./recorded_data/record.hdf5

# 显示样本图像
python simple_hdf5_reader.py ./recorded_data/record.hdf5 --image 0

# 绘制轨迹
python simple_hdf5_reader.py ./recorded_data/record.hdf5 --trajectory

# 导出基本数据
python simple_hdf5_reader.py ./recorded_data/record.hdf5 --export
```

## 功能特性

### 数据查看

- 显示数据集形状、类型、大小
- 计算平均帧率和频率
- 显示时间戳范围

### 图像显示

- 支持显示任意索引的图像
- 自动处理 BGR 到 RGB 的转换
- 可保存图像到文件

### 轨迹可视化

- 3D 轨迹图
- 三个平面的 2D 投影
- 高度变化图
- 起点和终点标记

### 数据导出

- 导出为 NumPy 数组 (.npy)
- 导出图像为 JPG 格式
- 生成数据摘要 JSON 文件

## 示例输出

```
=== 数据集信息 ===
数据集: images
  形状: (1500, 1080, 1920, 3)
  数据类型: uint8
  压缩: gzip

数据集: img_timestamps
  形状: (1500,)
  数据类型: float64

数据集: poses
  形状: (3000, 7)
  数据类型: float32

数据集: pose_timestamps
  形状: (3000,)
  数据类型: float64

=== 数据摘要 ===
images:
  形状: (1500, 1080, 1920, 3)
  数据类型: uint8
  大小: 8910.00 MB

img_timestamps:
  形状: (1500,)
  数据类型: float64
  大小: 0.01 MB

poses:
  形状: (3000, 7)
  数据类型: float32
  大小: 0.08 MB

pose_timestamps:
  形状: (3000,)
  数据类型: float64
  大小: 0.02 MB

总文件大小: 8910.11 MB
```

## 注意事项

1. **内存使用**: 大文件可能需要较多内存，建议先使用 `--info` 查看文件大小
2. **图像显示**: 需要图形界面环境，在服务器上可能需要设置 `DISPLAY`
3. **文件路径**: 确保 HDF5 文件路径正确
4. **依赖安装**: 确保安装了所有必要的 Python 包

## 故障排除

### 常见问题

1. **文件不存在**

   ```
   ✗ 文件不存在: ./recorded_data/record.hdf5
   ```

   解决：检查文件路径是否正确

2. **缺少依赖**

   ```
   ModuleNotFoundError: No module named 'h5py'
   ```

   解决：安装依赖包

   ```bash
   pip install h5py numpy matplotlib opencv-python
   ```

3. **显示问题**

   ```
   matplotlib: No display found
   ```

   解决：在服务器上使用 `--export` 保存图像，或设置 `DISPLAY` 环境变量

4. **内存不足**
   ```
   MemoryError
   ```
   解决：使用 `--info` 先查看文件大小，或分批处理数据

## 扩展功能

如果需要更多功能，可以基于现有脚本进行扩展：

1. **数据过滤**: 按时间范围筛选数据
2. **数据同步**: 将图像和位姿数据按时间戳对齐
3. **统计分析**: 计算轨迹的统计信息
4. **格式转换**: 转换为其他格式（如 ROS bag）
5. **实时播放**: 按时间顺序播放图像序列
