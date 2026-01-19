#!/usr/bin/env python3
"""
BluePilot WebSocket Broadcaster

Shared module for broadcasting WebSocket events from any process.
Can be used by:
- web_routes_server.py (in-process broadcasting)
- route_preprocessor.py (cross-process broadcasting via HTTP)
- Any other service that needs to notify the frontend
"""

import json
import logging
from datetime import datetime

logger = logging.getLogger(__name__)


class WebSocketEvent:
    """WebSocket event types for real-time updates"""
    ROUTES_UPDATED = 'routes_updated'
    ROUTE_ADDED = 'route_added'
    ROUTE_DELETED = 'route_deleted'
    ROUTE_PRESERVED = 'route_preserved'
    ROUTE_UNPRESERVED = 'route_unpreserved'
    STATUS_CHANGED = 'status_changed'
    PROCESSING_UPDATE = 'processing_update'
    PROCESSING_STARTED = 'processing_started'
    PROCESSING_COMPLETED = 'processing_completed'
    ROUTE_EXPORT_UPDATE = 'route_export_update'
    EXPORT_PROGRESS = 'export_progress'  # Export/backup progress updates
    EXPORT_COMPLETE = 'export_complete'  # Export/backup completed
    EXPORT_ERROR = 'export_error'  # Export/backup error
    CACHE_CLEARED = 'cache_cleared'
    DISK_UPDATED = 'disk_updated'  # Disk space changed
    FFMPEG_LOG = 'ffmpeg_log'  # Real-time FFmpeg debug logs
    PARAM_UPDATED = 'param_updated'  # Parameter value changed
    SYSTEM_METRICS_UPDATED = 'system_metrics_updated'  # System metrics updated
    LOG_LINE = 'log_line'  # Manager log line
    LOG_STREAM_STATUS = 'log_stream_status'  # Log stream status changed
    LOG_DOWNLOAD_UPDATE = 'log_download_update'  # qlog/rlog download progress


class WebSocketBroadcaster:
    """
    WebSocket broadcaster that can work both in-process and cross-process

    In-process: Direct access to websocket_clients set
    Cross-process: HTTP POST to web server's internal broadcast endpoint
    """

    def __init__(self, websocket_clients=None, loop=None, http_fallback_port=8088, server_state=None):
        """
        Initialize broadcaster

        Args:
            websocket_clients: Set of connected WebSocket clients (in-process only)
            loop: Event loop for async operations (in-process only)
            http_fallback_port: Port for HTTP fallback when cross-process
            server_state: Optional ServerState instance for live client access
        """
        self.server_state = server_state
        self.websocket_clients = websocket_clients
        self.loop = loop
        self.http_fallback_port = http_fallback_port
        if self.server_state is not None:
            self.use_http_fallback = False
        else:
            self.use_http_fallback = websocket_clients is None

    def broadcast(self, event_type, data=None):
        """
        Broadcast event to all connected WebSocket clients

        Args:
            event_type: Type of event (use WebSocketEvent constants)
            data: Event data dictionary
        """
        if not self.use_http_fallback and not self._has_clients():
            return  # No clients connected

        event_data = {
            'type': event_type,
            'timestamp': datetime.now().isoformat(),
            'data': data or {}
        }

        if self.use_http_fallback:
            self._broadcast_via_http(event_data)
        else:
            self._broadcast_in_process(event_data)

    def _broadcast_in_process(self, event_data):
        """Broadcast to clients in the same process"""
        import asyncio

        if not self.loop:
            return

        clients = self._get_clients()
        if not clients:
            return

        # Send to all connected clients
        for client in clients:
            try:
                # Use asyncio.run_coroutine_threadsafe to send from this thread to the event loop
                asyncio.run_coroutine_threadsafe(client.send(json.dumps(event_data)), self.loop)
            except Exception as e:
                logger.debug(f"Failed to send to client: {e}")
                # Remove failed client so future broadcasts skip it
                self._remove_client(client)

    def _broadcast_via_http(self, event_data):
        """Broadcast via HTTP to the web server (cross-process)"""
        try:
            import socket
            import urllib.request
            import urllib.error

            # Send to local web server's internal broadcast endpoint
            url = f'http://127.0.0.1:{self.http_fallback_port}/_internal/broadcast'

            req = urllib.request.Request(
                url,
                data=json.dumps(event_data).encode('utf-8'),
                headers={'Content-Type': 'application/json'},
                method='POST'
            )

            # Set a short timeout since this is localhost
            with urllib.request.urlopen(req, timeout=1) as response:
                if response.status != 200:
                    logger.debug(f"Broadcast HTTP request returned status {response.status}")

        except urllib.error.URLError as e:
            # Server might not be running, that's okay
            logger.debug(f"Could not broadcast via HTTP (server may not be running): {e}")
        except socket.timeout:
            logger.debug("Broadcast HTTP request timed out")
        except Exception as e:
            logger.debug(f"Error broadcasting via HTTP: {e}")

    def _get_clients(self):
        """Return a snapshot list of active WebSocket clients."""
        if self.server_state is not None:
            return self.server_state.get_websocket_clients()
        if self.websocket_clients:
            return list(self.websocket_clients)
        return []

    def _has_clients(self):
        """Check if there are active WebSocket clients."""
        if self.server_state is not None:
            return bool(self.server_state.get_websocket_clients())
        return bool(self.websocket_clients)

    def _remove_client(self, client):
        """Drop a client from the active set using the appropriate state container."""
        try:
            if self.server_state is not None:
                self.server_state.remove_websocket_client(client)
            elif self.websocket_clients:
                self.websocket_clients.discard(client)
        except Exception:
            pass

    def broadcast_processing_update(self, route_base, status, progress=None, message=None):
        """
        Broadcast a processing update for a specific route

        Args:
            route_base: Base name of the route being processed
            status: Status string ('processing', 'completed', 'failed')
            progress: Progress percentage (0-100) or None
            message: Optional status message
        """
        data = {
            'route_base': route_base,
            'status': status,
        }

        if progress is not None:
            data['progress'] = progress

        if message:
            data['message'] = message

        self.broadcast(WebSocketEvent.PROCESSING_UPDATE, data)

    def broadcast_processing_started(self, total_routes):
        """Broadcast that batch processing has started"""
        self.broadcast(WebSocketEvent.PROCESSING_STARTED, {
            'total_routes': total_routes
        })

    def broadcast_processing_completed(self, processed_count, total_time):
        """Broadcast that batch processing has completed"""
        self.broadcast(WebSocketEvent.PROCESSING_COMPLETED, {
            'processed_count': processed_count,
            'total_time': total_time
        })

    def broadcast_route_added(self, route_base, route_data=None):
        """Broadcast that a new route has been added"""
        data = {'route_base': route_base}
        if route_data:
            data.update(route_data)
        self.broadcast(WebSocketEvent.ROUTE_ADDED, data)

    def broadcast_routes_updated(self):
        """Broadcast that the route list should be refreshed"""
        self.broadcast(WebSocketEvent.ROUTES_UPDATED, {})

    def broadcast_ffmpeg_log(self, route_info, log_type, message, pid=None):
        """
        Broadcast FFmpeg debug log in real-time

        Args:
            route_info: Route identifier (route_base:segment:camera)
            log_type: Type of log ('stdout', 'stderr', 'start', 'end', 'error')
            message: Log message content
            pid: Process ID (optional)
        """
        data = {
            'route_info': route_info,
            'log_type': log_type,
            'message': message,
        }
        if pid:
            data['pid'] = pid
        self.broadcast(WebSocketEvent.FFMPEG_LOG, data)
