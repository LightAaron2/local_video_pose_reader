// 控制管理器
class ControlManager {
    constructor() {
        this.isSending = false;
        this.controlData = {
            position: { x: 0, y: 0, z: 0 },
            orientation: { roll: 0, pitch: 0, yaw: 0 },
            gripper: 0.5
        };
        
        this.init();
    }
    
    init() {
        // 这里可以添加额外的控制功能
        console.log('控制管理器初始化完成');
    }
    
    // 发送轨迹数据
    sendTrajectory(trajectoryData) {
        if (window.ws && window.ws.readyState === WebSocket.OPEN) {
            const message = {
                type: 'trajectory',
                data: trajectoryData
            };
            window.ws.send(JSON.stringify(message));
            return true;
        }
        return false;
    }
    
    // 重置控制
    resetControls() {
        this.controlData = {
            position: { x: 0, y: 0, z: 0 },
            orientation: { roll: 0, pitch: 0, yaw: 0 },
            gripper: 0.5
        };
        console.log('控制已重置');
    }
    
    // 紧急停止
    emergencyStop() {
        this.isSending = false;
        console.log('紧急停止已触发');
        
        // 发送紧急停止信号
        if (window.ws && window.ws.readyState === WebSocket.OPEN) {
            const message = {
                type: 'emergency_stop',
                timestamp: Date.now()
            };
            window.ws.send(JSON.stringify(message));
        }
    }
    
    // 切换发送状态
    toggleSending() {
        this.isSending = !this.isSending;
        console.log(`发送状态: ${this.isSending ? '开启' : '关闭'}`);
    }
    
    // 获取当前控制数据
    getControlData() {
        return { ...this.controlData };
    }
    
    // 更新控制数据
    updateControlData(newData) {
        this.controlData = { ...this.controlData, ...newData };
    }
}

// 创建全局控制管理器实例
const controlManager = new ControlManager();

// 键盘快捷键处理
document.addEventListener('keydown', (e) => {
    switch (e.key) {
        case 'Escape':
            // ESC键紧急停止
            e.preventDefault();
            controlManager.emergencyStop();
            break;
            
        case ' ':
            // 空格键切换发送状态
            e.preventDefault();
            controlManager.toggleSending();
            break;
            
        case 'r':
        case 'R':
            // R键重置控制
            e.preventDefault();
            controlManager.resetControls();
            break;
            
        case 'f':
        case 'F':
            // F键全屏
            e.preventDefault();
            if (cameras && cameras.length > 0) {
                openFullscreen(`/camera/${cameras[0].index}/stream`, `摄像头 ${cameras[0].index}`);
            }
            break;
    }
});

// 导出供其他模块使用
window.controlManager = controlManager;
