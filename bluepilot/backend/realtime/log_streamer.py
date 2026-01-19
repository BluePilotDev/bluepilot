#!/usr/bin/env python3
"""
BluePilot Log Streamer

Streams manager logs in real-time via WebSocket
"""

import logging
import threading
from typing import Optional

from bluepilot.backend.logs import parse_manager_log_line

logger = logging.getLogger(__name__)


class LogStreamer:
    """Streams manager logs to WebSocket clients"""

    def __init__(self, websocket_broadcaster):
        """
        Initialize log streamer

        Args:
            websocket_broadcaster: WebSocketBroadcaster instance
        """
        self.broadcaster = websocket_broadcaster
        self.thread: Optional[threading.Thread] = None
        self.running = False
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._sock = None

    def start(self):
        """Start streaming logs"""
        with self._lock:
            if self.running:
                logger.debug("Log streamer already running")
                return False

            try:
                import cereal.messaging as messaging

                self._sock = messaging.sub_sock('logMessage', timeout=1000, conflate=True)
            except Exception as exc:
                logger.error("Unable to access logMessage stream: %s", exc)
                self._broadcast_status('error', 'log stream unavailable')
                return False

            self.running = True
            self._stop_event.clear()

            self.thread = threading.Thread(target=self._read_logs, daemon=True)
            self.thread.start()

            logger.info("Log streamer started")
            self._broadcast_status('started')
            return True

    def stop(self):
        """Stop streaming logs"""
        with self._lock:
            if not self.running:
                logger.debug("Log streamer not running")
                return False

            self.running = False
            self._stop_event.set()

            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
                finally:
                    self._sock = None

            if self.thread and self.thread.is_alive():
                self.thread.join(timeout=1.5)
                self.thread = None

            logger.info("Log streamer stopped")
            self._broadcast_status('stopped')
            return True

    def is_running(self):
        """Check if streamer is running"""
        return self.running

    def _read_logs(self):
        """Read logs from messaging stream and broadcast"""
        try:
            import cereal.messaging as messaging
        except Exception as exc:
            logger.error("Unable to import messaging for log stream: %s", exc)
            self._broadcast_status('error', 'messaging unavailable')
            self.running = False
            return

        sock = self._sock or messaging.sub_sock('logMessage', timeout=1000, conflate=True)

        try:
            while self.running:
                msg = messaging.recv_one_or_none(sock)
                if msg is None:
                    if self._stop_event.wait(timeout=0.1):
                        break
                    continue

                if msg.which() != 'logMessage':
                    continue

                formatted = parse_manager_log_line(msg.logMessage)
                if formatted:
                    from bluepilot.backend.realtime.websocket import WebSocketEvent
                    self.broadcaster.broadcast(WebSocketEvent.LOG_LINE, {
                        'line': formatted
                    })

        except Exception as exc:
            logger.error("Error reading log stream: %s", exc)
            self._broadcast_status('error', 'Log stream error')
        finally:
            self.running = False
            self._stop_event.set()
            if self._sock and sock is not self._sock:
                try:
                    sock.close()
                except Exception:
                    pass
            self._sock = None

    def _broadcast_status(self, status, message=None):
        """Broadcast stream status change"""
        from bluepilot.backend.realtime.websocket import WebSocketEvent
        data = {'status': status}
        if message:
            data['message'] = message
        self.broadcaster.broadcast(WebSocketEvent.LOG_STREAM_STATUS, data)


# Global log streamer instance
_log_streamer: Optional[LogStreamer] = None


def get_log_streamer(websocket_broadcaster=None):
    """Get or create global log streamer instance"""
    global _log_streamer

    if _log_streamer is None and websocket_broadcaster is not None:
        _log_streamer = LogStreamer(websocket_broadcaster)

    return _log_streamer
