"""
BluePilot Backend Realtime Module
WebSocket broadcasting and real-time event notifications
"""

# Re-export for backwards compatibility
from .websocket import WebSocketBroadcaster, WebSocketEvent

__all__ = [
    'WebSocketBroadcaster',
    'WebSocketEvent',
]
