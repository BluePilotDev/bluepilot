import type { WebSocketMessage } from "@/types";

type MessageHandler = (message: WebSocketMessage) => void;

// Determine WebSocket URL based on environment
function getWebSocketUrl(): string {
  const { hostname, port, protocol } = window.location;
  const wsProtocol = protocol === "https:" ? "wss:" : "ws:";

  // If running on localhost (dev/preview mode), use the proxy path
  if (hostname === "localhost" || hostname === "127.0.0.1") {
    // Connect through Vite's WebSocket proxy at /ws
    return `${wsProtocol}//${hostname}:${port}`;
  }

  // Running directly on device - connect to WebSocket server on port 8089
  return `${wsProtocol}//${hostname}:8089`;
}

export class WebSocketService {
  private ws: WebSocket | null = null;
  private reconnectTimer: number | null = null;
  private reconnectAttempts = 0;
  private maxReconnectAttempts = 10;
  private reconnectDelay = 5000;
  private handlers: Set<MessageHandler> = new Set();
  private connected = false;
  private wsUrl: string;

  constructor() {
    this.wsUrl = getWebSocketUrl();
  }

  connect(): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      console.log("WebSocket already connected");
      return;
    }

    console.log("Connecting to WebSocket:", this.wsUrl);

    try {
      this.ws = new WebSocket(this.wsUrl);

      this.ws.onopen = () => {
        console.log("WebSocket connected");
        this.connected = true;
        this.reconnectAttempts = 0;
        this.notifyHandlers({
          type: "connection",
          data: { connected: true },
        });
      };

      this.ws.onmessage = (event) => {
        try {
          const message: WebSocketMessage = JSON.parse(event.data);
          this.notifyHandlers(message);
        } catch (error) {
          console.error("Failed to parse WebSocket message:", error);
        }
      };

      this.ws.onerror = (error) => {
        console.error("WebSocket error:", error);
      };

      this.ws.onclose = () => {
        console.log("WebSocket disconnected");
        this.connected = false;
        this.ws = null;
        this.notifyHandlers({
          type: "connection",
          data: { connected: false },
        });
        this.scheduleReconnect();
      };
    } catch (error) {
      console.error("Failed to create WebSocket:", error);
      this.scheduleReconnect();
    }
  }

  disconnect(): void {
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }

    if (this.ws) {
      this.ws.close();
      this.ws = null;
    }

    this.connected = false;
  }

  isConnected(): boolean {
    return this.connected;
  }

  send(message: WebSocketMessage): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      this.ws.send(JSON.stringify(message));
    } else {
      console.warn("WebSocket not connected, cannot send message");
    }
  }

  subscribe(handler: MessageHandler): () => void {
    this.handlers.add(handler);

    // Return unsubscribe function
    return () => {
      this.handlers.delete(handler);
    };
  }

  private notifyHandlers(message: WebSocketMessage): void {
    this.handlers.forEach((handler) => {
      try {
        handler(message);
      } catch (error) {
        console.error("Error in WebSocket handler:", error);
      }
    });
  }

  private scheduleReconnect(): void {
    if (this.reconnectAttempts >= this.maxReconnectAttempts) {
      console.log("Max reconnect attempts reached, giving up");
      return;
    }

    if (this.reconnectTimer !== null) {
      return; // Already scheduled
    }

    this.reconnectAttempts++;
    const delay = this.reconnectDelay * this.reconnectAttempts;

    console.log(
      `Scheduling reconnect attempt ${this.reconnectAttempts} in ${delay}ms`
    );

    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, delay);
  }
}

// Singleton instance
let wsInstance: WebSocketService | null = null;

export const getWebSocketService = (): WebSocketService => {
  if (!wsInstance) {
    wsInstance = new WebSocketService();
  }
  return wsInstance;
};

export default WebSocketService;
