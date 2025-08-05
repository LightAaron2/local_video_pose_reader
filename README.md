# 本地遥操作系统
http://localhost:8080/simple
这是一个完全本地化的遥操作系统，集成了摄像头数据读取、机械臂控制和 Web 界面。

## 功能特性

- 🎥 **实时摄像头显示**：支持多种 ROS 摄像头话题
- 📍 **SLAM 数据读取**：实时读取和显示 SLAM 位置数据
- 🌐 **本地 Web 界面**：美观的现代化界面
- ⚡ **低延迟通信**：本地 WebSocket 通信
- 🔄 **自动重连**：网络断开自动重连
- 🎮 **多种控制方式**：滑块控制、键盘快捷键

## 系统架构

```
前端 (HTML/JS)
    ↕ WebSocket
本地后端 (Python + aiohttp)
    ↕ ROS话题
ROS摄像头节点
    ↕ SLAM数据
XVision SLAM节点
```

## 安装和运行

### 1. 安装依赖

```bash
cd local_video&pose_reader
pip install -r requirements.txt
```

### 2. 启动后端服务

```bash
python backend.py
```

后端服务将在 `http://localhost:8080` 启动。

### 3. 访问前端界面

打开浏览器访问：`http://localhost:8080`

## 使用说明

### 控制界面

- **位置控制**：X、Y、Z 轴位置滑块
- **姿态控制**：Roll、Pitch、Yaw 角度滑块
- **夹爪控制**：夹爪宽度滑块

### 数据监控

- **SLAM 数据**：实时显示从 XVision 获取的 SLAM 位置和姿态数据
- **连接状态**：显示 WebSocket 连接状态
- **延迟监控**：显示通信延迟

### 快捷键

- `空格键`：切换轨迹发送状态
- `R键`：重置所有控制
- `ESC键`：紧急停止

### 状态监控

- **连接状态**：显示 WebSocket 连接状态
- **Pose 状态**：显示机械臂 pose 数据接收状态
- **延迟监控**：显示通信延迟

## 配置说明

### SLAM 数据配置

系统会自动订阅 XVision 的 SLAM 数据话题：

```python
# SLAM数据话题
"/xv_sdk/xv_dev/slam/visual_pose"

# 摄像头数据话题（可选，用于夹爪检测）
"/xv_sdk/xv_dev/color_camera/image_color"
```

### 摄像头配置

系统会自动检测可用的 ROS 摄像头话题，支持以下模式：

- `/camX/usb_cam/image_raw`
- `/xv_sdk/xv_dev/color_camera/image_color`
- 其他包含"camera"、"cam"、"image"关键词的话题

## 技术特性

### 后端特性

- **异步处理**：使用 asyncio 处理并发请求
- **ROS 集成**：纯 numpy 图像处理，无需 cv_bridge
- **SLAM 数据读取**：实时读取 XVision SLAM 数据
- **WebSocket 通信**：实时双向通信

### 前端特性

- **响应式设计**：支持桌面和移动设备
- **实时更新**：摄像头图像和 pose 数据实时显示
- **用户友好**：直观的控制界面和状态显示
- **错误处理**：完善的错误处理和用户提示

## 故障排除

### 常见问题

1. **摄像头无显示**

   - 检查 ROS 话题是否正常发布
   - 确认摄像头话题名称是否正确

2. **SLAM 数据无显示**

   - 检查 XVision SLAM 话题是否正常发布
   - 确认话题名称是否正确

3. **WebSocket 连接失败**
   - 检查后端服务是否正常启动
   - 确认端口 8080 未被占用

### 日志查看

后端会输出详细的日志信息，包括：

- ROS 节点初始化状态
- 摄像头检测结果
- WebSocket 连接状态
- SLAM 数据接收状态

## 开发说明

### 项目结构

```
local_video&pose_reader/
├── backend.py              # 后端主程序
├── static/                 # 前端静态文件
│   ├── index.html         # 主页面
│   ├── styles.css         # 样式文件
│   ├── websocket.js       # WebSocket通信
│   ├── controls.js        # 控制逻辑
│   └── main.js           # 主应用逻辑
├── requirements.txt        # Python依赖
└── README.md             # 项目说明
```

### 扩展功能

- 添加更多摄像头支持
- 实现 SLAM 轨迹录制和回放
- 添加夹爪检测功能
- 支持更多 SLAM 系统

## 许可证

本项目仅供学习和研究使用。
