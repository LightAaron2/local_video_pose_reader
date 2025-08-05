// 全局变量
let ws = null;
let cameras = [];
let slamData = null;
let startTime = Date.now();
let dataFpsCounter = 0;
let lastFpsTime = Date.now();

// DOM元素
const elements = {
    connectionStatus: document.getElementById('connection-status'),
    connectionText: document.getElementById('connection-text'),
    videoGrid: document.getElementById('video-grid'),
    posX: document.getElementById('pos-x'),
    posY: document.getElementById('pos-y'),
    posZ: document.getElementById('pos-z'),
    quatX: document.getElementById('quat-x'),
    quatY: document.getElementById('quat-y'),
    quatZ: document.getElementById('quat-z'),
    quatW: document.getElementById('quat-w'),
    wsStatus: document.getElementById('ws-status'),
    cameraCount: document.getElementById('camera-count'),
    dataFps: document.getElementById('data-fps'),
    latency: document.getElementById('latency'),
    memoryUsage: document.getElementById('memory-usage'),
    uptime: document.getElementById('uptime'),
    logContent: document.getElementById('log-content'),
    loadingOverlay: document.getElementById('loading-overlay'),
    fullscreenModal: document.getElementById('fullscreen-modal'),
    fullscreenVideo: document.getElementById('fullscreen-video')
};

// 初始化
document.addEventListener('DOMContentLoaded', function() {
    log('系统初始化开始', 'info');
    
    // 初始化WebSocket连接
    initWebSocket();
    
    // 初始化事件监听器
    initEventListeners();
    
    // 开始性能监控
    startPerformanceMonitoring();
    
    // 隐藏加载动画
    setTimeout(() => {
        elements.loadingOverlay.style.display = 'none';
    }, 2000);
    
    log('系统初始化完成', 'success');
});

// 初始化WebSocket连接
function initWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;
    
    log(`正在连接WebSocket: ${wsUrl}`, 'info');
    
    ws = new WebSocket(wsUrl);
    
    ws.onopen = function() {
        log('WebSocket连接成功', 'success');
        updateConnectionStatus('online', '已连接');
        elements.wsStatus.textContent = '已连接';
        
        // 发送ping消息测试连接
        sendPing();
    };
    
    ws.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            handleWebSocketMessage(data);
        } catch (error) {
            log(`WebSocket消息解析失败: ${error}`, 'error');
        }
    };
    
    ws.onclose = function() {
        log('WebSocket连接断开', 'warning');
        updateConnectionStatus('offline', '连接断开');
        elements.wsStatus.textContent = '断开';
        
        // 5秒后尝试重连
        setTimeout(initWebSocket, 5000);
    };
    
    ws.onerror = function(error) {
        log(`WebSocket错误: ${error}`, 'error');
        updateConnectionStatus('offline', '连接错误');
    };
}

// 处理WebSocket消息
function handleWebSocketMessage(data) {
    switch (data.type) {
        case 'slam':
            handleSlamData(data.data);
            break;
        case 'pong':
            handlePong(data);
            break;
        case 'camera_list':
            handleCameraList(data.cameras);
            break;
        default:
            log(`未知消息类型: ${data.type}`, 'warning');
    }
}

// 处理SLAM数据
function handleSlamData(data) {
    if (Array.isArray(data) && data.length >= 7) {
        slamData = data;
        
        // 更新位置显示
        elements.posX.textContent = data[0].toFixed(3);
        elements.posY.textContent = data[1].toFixed(3);
        elements.posZ.textContent = data[2].toFixed(3);
        
        // 更新四元数显示
        elements.quatX.textContent = data[3].toFixed(3);
        elements.quatY.textContent = data[4].toFixed(3);
        elements.quatZ.textContent = data[5].toFixed(3);
        elements.quatW.textContent = data[6].toFixed(3);
        
        // 更新数据频率
        dataFpsCounter++;
        const now = Date.now();
        if (now - lastFpsTime >= 1000) {
            elements.dataFps.textContent = `${dataFpsCounter} Hz`;
            dataFpsCounter = 0;
            lastFpsTime = now;
        }
    }
}

// 处理Pong响应
function handlePong(data) {
    const latency = Date.now() - data.sent_at;
    elements.latency.textContent = `${latency}ms`;
}

// 处理摄像头列表
function handleCameraList(cameraList) {
    cameras = cameraList || [];
    elements.cameraCount.textContent = cameras.length;
    
    // 更新视频网格
    updateVideoGrid();
    
    log(`发现 ${cameras.length} 个摄像头`, 'info');
}

// 更新视频网格
function updateVideoGrid() {
    elements.videoGrid.innerHTML = '';
    
    if (cameras.length === 0) {
        elements.videoGrid.innerHTML = `
            <div class="video-item" style="display: flex; align-items: center; justify-content: center; color: #666;">
                <div>暂无摄像头数据</div>
            </div>
        `;
        return;
    }
    
    cameras.forEach((camera, index) => {
        const videoItem = document.createElement('div');
        videoItem.className = 'video-item';
        videoItem.innerHTML = `
            <img src="/camera/${camera.index}/stream" alt="摄像头 ${camera.index}" 
                 onerror="this.style.display='none'; this.nextElementSibling.style.display='flex';">
            <div class="video-overlay" style="display: none; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); color: white; background: rgba(0,0,0,0.7); padding: 10px; border-radius: 5px;">
                摄像头 ${camera.index} 连接失败
            </div>
            <div class="video-overlay">摄像头 ${camera.index}</div>
        `;
        
        // 添加点击事件用于全屏
        videoItem.addEventListener('click', () => {
            openFullscreen(`/camera/${camera.index}/stream`, `摄像头 ${camera.index}`);
        });
        
        elements.videoGrid.appendChild(videoItem);
    });
}

// 发送Ping消息
function sendPing() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        const pingMessage = {
            type: 'ping',
            ping_id: Date.now().toString(),
            sent_at: Date.now()
        };
        ws.send(JSON.stringify(pingMessage));
    }
}

// 更新连接状态
function updateConnectionStatus(status, text) {
    elements.connectionStatus.className = `status-dot ${status}`;
    elements.connectionText.textContent = text;
}

// 初始化事件监听器
function initEventListeners() {
    // 刷新摄像头按钮
    document.getElementById('refresh-cameras').addEventListener('click', () => {
        log('正在刷新摄像头...', 'info');
        updateVideoGrid();
    });
    
    // 全屏按钮
    document.getElementById('fullscreen-btn').addEventListener('click', () => {
        if (cameras.length > 0) {
            openFullscreen(`/camera/${cameras[0].index}/stream`, `摄像头 ${cameras[0].index}`);
        }
    });
    
    // 重置SLAM按钮
    document.getElementById('reset-slam').addEventListener('click', () => {
        log('重置SLAM数据', 'info');
        slamData = null;
        elements.posX.textContent = '0.000';
        elements.posY.textContent = '0.000';
        elements.posZ.textContent = '0.000';
        elements.quatX.textContent = '0.000';
        elements.quatY.textContent = '0.000';
        elements.quatZ.textContent = '0.000';
        elements.quatW.textContent = '1.000';
    });
    
    // 导出数据按钮
    document.getElementById('export-data').addEventListener('click', () => {
        exportSlamData();
    });
    
    // 清空日志按钮
    document.getElementById('clear-log').addEventListener('click', () => {
        elements.logContent.innerHTML = '';
        log('日志已清空', 'info');
    });
    
    // 导出日志按钮
    document.getElementById('export-log').addEventListener('click', () => {
        exportLog();
    });
    
    // 关闭模态框
    document.getElementById('close-modal').addEventListener('click', () => {
        elements.fullscreenModal.style.display = 'none';
    });
    
    // 点击模态框外部关闭
    elements.fullscreenModal.addEventListener('click', (e) => {
        if (e.target === elements.fullscreenModal) {
            elements.fullscreenModal.style.display = 'none';
        }
    });
}

// 打开全屏视频
function openFullscreen(videoSrc, title) {
    elements.fullscreenVideo.src = videoSrc;
    elements.fullscreenModal.style.display = 'block';
    log(`打开全屏视频: ${title}`, 'info');
}

// 导出SLAM数据
function exportSlamData() {
    if (!slamData) {
        log('没有SLAM数据可导出', 'warning');
        return;
    }
    
    const data = {
        timestamp: new Date().toISOString(),
        slam_data: slamData,
        position: {
            x: slamData[0],
            y: slamData[1],
            z: slamData[2]
        },
        orientation: {
            x: slamData[3],
            y: slamData[4],
            z: slamData[5],
            w: slamData[6]
        }
    };
    
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `slam_data_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.json`;
    a.click();
    URL.revokeObjectURL(url);
    
    log('SLAM数据已导出', 'success');
}

// 导出日志
function exportLog() {
    const logText = elements.logContent.innerText;
    const blob = new Blob([logText], { type: 'text/plain' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `system_log_${new Date().toISOString().slice(0, 19).replace(/:/g, '-')}.txt`;
    a.click();
    URL.revokeObjectURL(url);
    
    log('系统日志已导出', 'success');
}

// 添加日志
function log(message, type = 'info') {
    const timestamp = new Date().toLocaleTimeString();
    const logEntry = document.createElement('div');
    logEntry.className = `log-entry ${type}`;
    logEntry.textContent = `[${timestamp}] ${message}`;
    
    elements.logContent.appendChild(logEntry);
    elements.logContent.scrollTop = elements.logContent.scrollHeight;
    
    // 限制日志条目数量
    const logEntries = elements.logContent.querySelectorAll('.log-entry');
    if (logEntries.length > 100) {
        logEntries[0].remove();
    }
}

// 开始性能监控
function startPerformanceMonitoring() {
    setInterval(() => {
        // 更新运行时间
        const uptime = Date.now() - startTime;
        const hours = Math.floor(uptime / 3600000);
        const minutes = Math.floor((uptime % 3600000) / 60000);
        const seconds = Math.floor((uptime % 60000) / 1000);
        elements.uptime.textContent = `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}:${seconds.toString().padStart(2, '0')}`;
        
        // 模拟内存使用（实际项目中应该从后端获取）
        const memoryUsage = Math.floor(Math.random() * 50) + 100;
        elements.memoryUsage.textContent = `${memoryUsage} MB`;
        
    }, 1000);
}

// 定期发送ping
setInterval(sendPing, 5000);
