// WebSocket管理器
class WebSocketManager {
    constructor() {
        this.ws = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 5000;
        this.isConnecting = false;
        
        // 回调函数
        this.onConnect = null;
        this.onDisconnect = null;
        this.onMessage = null;
        this.onError = null;
    }
    
    connect() {
        if (this.isConnecting || this.ws?.readyState === WebSocket.OPEN) {
            return;
        }
        
        this.isConnecting = true;
        
        try {
            const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
            const wsUrl = `${protocol}//${window.location.host}/ws`;
            
            this.ws = new WebSocket(wsUrl);
            
            this.ws.onopen = () => {
                this.isConnecting = false;
                this.reconnectAttempts = 0;
                console.log('WebSocket连接成功');
                
                if (this.onConnect) {
                    this.onConnect();
                }
            };
            
            this.ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    
                    if (this.onMessage) {
                        this.onMessage(data);
                    }
                } catch (error) {
                    console.error('WebSocket消息解析失败:', error);
                }
            };
            
            this.ws.onclose = (event) => {
                this.isConnecting = false;
                console.log('WebSocket连接断开:', event.code, event.reason);
                
                if (this.onDisconnect) {
                    this.onDisconnect();
                }
                
                // 自动重连
                this.attemptReconnect();
            };
            
            this.ws.onerror = (error) => {
                this.isConnecting = false;
                console.error('WebSocket错误:', error);
                
                if (this.onError) {
                    this.onError(error);
                }
            };
            
        } catch (error) {
            this.isConnecting = false;
            console.error('WebSocket连接失败:', error);
            
            if (this.onError) {
                this.onError(error);
            }
        }
    }
    
    disconnect() {
        if (this.ws) {
            this.ws.close();
            this.ws = null;
        }
        this.isConnecting = false;
    }
    
    send(data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            try {
                const message = typeof data === 'string' ? data : JSON.stringify(data);
                this.ws.send(message);
                return true;
            } catch (error) {
                console.error('发送消息失败:', error);
                return false;
            }
        } else {
            console.warn('WebSocket未连接，无法发送消息');
            return false;
        }
    }
    
    attemptReconnect() {
        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            console.log('达到最大重连次数，停止重连');
            return;
        }
        
        this.reconnectAttempts++;
        console.log(`尝试重连 (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`);
        
        setTimeout(() => {
            this.connect();
        }, this.reconnectDelay);
    }
    
    getLatency() {
        // 这里可以实现延迟测量逻辑
        return 0;
    }
    
    isConnected() {
        return this.ws && this.ws.readyState === WebSocket.OPEN;
    }
}

// 创建全局WebSocket管理器实例
const wsManager = new WebSocketManager();

// 导出供其他模块使用
window.wsManager = wsManager;
