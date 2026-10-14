import type { CalibrationCommand } from "../types";

type WebSocketHandlers = {
  onOpen: () => void;
  onClose: () => void;
  onMessage: (message: string) => void;
};

export class ImuWebSocket {
  private socket: WebSocket | null = null;

  public constructor(private readonly handlers: WebSocketHandlers) {}

  public connect(): void {
    this.socket = new WebSocket(`ws://${location.host}/ws`);
    this.socket.onopen = this.handlers.onOpen;
    this.socket.onclose = () => {
      this.handlers.onClose();
      window.setTimeout(() => this.connect(), 1800);
    };
    this.socket.onerror = () => this.socket?.close();
    this.socket.onmessage = (event) => this.handlers.onMessage(event.data);
  }

  public send(command: CalibrationCommand): void {
    if (this.socket?.readyState === WebSocket.OPEN) this.socket.send(command);
  }
}
