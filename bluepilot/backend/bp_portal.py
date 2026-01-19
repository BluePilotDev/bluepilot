#!/usr/bin/env python3
"""
BluePilot Backend Server
Main HTTP server with modular backend architecture

This server handles:
- Route management and browsing
- Video streaming and processing (HLS, exports, remux)
- System metrics and health monitoring
- WebSocket real-time updates
- Cache management
- Route preservation and deletion risk

All utility functions have been modularized into dedicated modules.
"""

import os
import json
import mimetypes
import subprocess
import sys
import shutil
import time
import signal
import atexit
from pathlib import Path
from datetime import datetime, timedelta, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import logging
import re
import asyncio
import threading
import requests

try:
    import psutil
except ImportError:
    psutil = None

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s [%(name)s]: %(message)s',
    force=True
)
logger = logging.getLogger(__name__)

# Add parent directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), "../.."))

# ============================================================================
# Import All Modular Components
# ============================================================================

# Configuration
from bluepilot.backend.config import (
    ROUTES_DIR, WEBAPP_DIR, WEBSOCKET_PORT,
    FFMPEG_BINARY,
    MAX_CONCURRENT_FFMPEG,
    THUMBNAIL_CACHE, REMUX_CACHE, METRICS_CACHE,
    ROUTE_EXPORT_CACHE,
    CAMERA_FILES,
    WEBSOCKET_HOST,
    RATE_LIMIT_WINDOW_SECONDS,
    RATE_LIMIT_REQUESTS_PER_SECOND_OFFROAD, RATE_LIMIT_REQUESTS_PER_SECOND_ONROAD,
    FAVORITE_SETTINGS_FILE,
)

# Core server components
from bluepilot.backend.core import ServerState
from bluepilot.backend.core import process_manager, lifecycle

# Network utilities
from bluepilot.backend.network import (
    is_onroad, should_server_run,
    get_wifi_ip, get_all_wifi_ips, get_connection_type,
)

# Route utilities
from bluepilot.backend.routes import (
    # Processing
    reverse_geocode,
    get_route_gps_metrics,
    generate_thumbnail,
    kill_existing_process,
    # Parsing
    get_route_base_name,
    parse_route_datetime,
    # Segments
    get_route_segments, get_file_size,
    format_size, get_disk_space_info,
    # Scanner
    scan_routes,
    # Metadata builder
    build_route_metadata,
)

# Video processing
from bluepilot.backend.video import (
    FFmpegProcess,
    get_video_duration, get_video_duration_from_cache,
    prefetch_next_segments,
    route_export_key, get_export_output_path,
    generate_route_export_filename, export_is_up_to_date,
    format_route_export_status, broadcast_route_export_update,
    generate_route_export, stream_route_export,
)

# System metrics
from bluepilot.backend.system import get_system_metrics

# Params management
from bluepilot.backend.params.params_manager import (
    get_all_params, get_params_by_category, get_param_value,
    set_param_value, search_params, READONLY_PARAMS, CRITICAL_PARAMS
)
from bluepilot.backend.params.params_watcher import ParamsWatcher

# Cache management
from bluepilot.backend.cache import (
    get_all_cache_sizes, cleanup_old_cache,
)

# Storage and preservation
from bluepilot.backend.storage import (
    get_cached_deletion_data,
    check_route_preserve_status, set_route_preserve,
    clear_deletion_data_cache,
)

# Log extraction
from bluepilot.backend.logs import (
    extract_log_messages, extract_cereal_messages,
    read_recent_manager_logs,
)

# File operations
from bluepilot.backend.utils.file_ops import (
    get_free_disk_space, has_sufficient_disk_space,
)

# Power management
from bluepilot.backend.utils.power import (
    enable_performance_mode,
    check_and_restore_power_save,
)

# WebSocket
from bluepilot.backend.realtime import WebSocketBroadcaster, WebSocketEvent

# Log download handlers
from bluepilot.backend.handlers.log_downloads import (
    handle_qlog_download, handle_rlog_download
)

# Params - import from params_manager to get fallback support
from bluepilot.backend.params.params_manager import Params
params = Params()


def restart_ui_process():
    """Attempt to restart the UI process by signaling the running binary."""
    process_names = {'ui', 'bluepilot-ui', 'bp-ui'}

    if psutil:
        try:
            targets = []
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                name = (proc.info.get('name') or '').lower()
                cmdline = ' '.join(proc.info.get('cmdline') or [])
                if name in process_names or 'selfdrive/ui' in cmdline or 'bluepilot-ui' in cmdline:
                    targets.append(proc)

            if targets:
                for proc in targets:
                    try:
                        logger.info(f"Sending SIGINT to UI process {proc.pid} ({proc.info.get('name')})")
                        proc.send_signal(signal.SIGINT)
                    except (psutil.NoSuchProcess, psutil.AccessDenied) as exc:
                        logger.warning(f"Failed to signal UI process {proc.pid}: {exc}")
                return True, 'Restart signal sent to UI process'
        except Exception as exc:
            logger.warning(f"psutil UI restart attempt failed: {exc}")

    # Fallback to pkill if psutil is unavailable or no process matched
    try:
        result = subprocess.run(
            ['pkill', '-2', '-f', 'selfdrive/ui'],
            capture_output=True,
            text=True,
            timeout=3
        )
        if result.returncode == 0:
            logger.info('pkill successfully signaled UI process')
            return True, 'Restart signal sent via pkill'
        if result.returncode == 1:
            logger.warning('pkill could not find a UI process to signal')
            return False, 'UI process not found'
        logger.warning(f"pkill returned unexpected code {result.returncode}: {result.stderr.strip()}")
    except FileNotFoundError:
        logger.warning('pkill not available for UI restart fallback')
    except Exception as exc:
        logger.error(f"Failed to invoke pkill for UI restart: {exc}")

    return False, 'Unable to signal UI process'

# Disk space deletion thresholds
try:
    from openpilot.system.loggerd.deleter import MIN_BYTES, MIN_PERCENT
except ImportError:
    MIN_BYTES = 5 * 1024 * 1024 * 1024  # 5 GB
    MIN_PERCENT = 10

# WebSocket availability - checked dynamically to handle runtime installation
WEBSOCKETS_AVAILABLE = False

# ============================================================================
# Global Server State
# ============================================================================

# Initialize global server state
server_state = ServerState()


def broadcast_websocket_event(event_type, data=None):
    """Broadcast event to all connected WebSocket clients (thread-safe)"""
    broadcaster = server_state.get_broadcaster()
    if broadcaster:
        try:
            broadcaster.broadcast(event_type, data)
        except Exception as e:
            logger.error(f"Error broadcasting WebSocket event {event_type}: {e}")


async def websocket_handler(websocket):
    """Handle WebSocket connections (thread-safe)"""
    try:
        import websockets
    except ImportError:
        logger.error("websockets not available in handler")
        return

    try:
        # Thread-safe: Add client to active connections
        client_count = server_state.add_websocket_client(websocket)
        logger.info(f"WebSocket client connected. Total clients: {client_count}")

        # Send initial status
        try:
            initial_status = {
                'type': 'connection_established',
                'timestamp': datetime.now().isoformat(),
                'data': {
                    'status': 'online',
                    'routes_count': 0
                }
            }
            await websocket.send(json.dumps(initial_status))
        except Exception as e:
            logger.warning(f"Failed to send initial status: {e}")

        # Keep connection alive with heartbeat and listen for messages
        async def send_heartbeats():
            while True:
                try:
                    await asyncio.sleep(10.0)
                    heartbeat = {
                        'type': 'heartbeat',
                        'timestamp': datetime.now().isoformat(),
                        'data': {}
                    }
                    await websocket.send(json.dumps(heartbeat))
                except Exception as e:
                    logger.debug(f"WebSocket heartbeat send failed: {e}")
                    break

        async def receive_messages():
            try:
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        msg_type = data.get('type')

                        # Handle pong responses to heartbeat
                        if msg_type == 'pong':
                            logger.debug("Received pong from client")
                        else:
                            logger.debug(f"Received message from client: {msg_type}")
                    except json.JSONDecodeError:
                        logger.warning(f"Invalid JSON from client: {message}")
            except Exception as e:
                logger.debug(f"WebSocket receive error: {e}")

        # Run both tasks concurrently
        await asyncio.gather(
            send_heartbeats(),
            receive_messages(),
            return_exceptions=True
        )

    except Exception as e:
        logger.error(f"WebSocket handler error: {e}", exc_info=True)
    finally:
        # Thread-safe: Remove client from active connections
        client_count = server_state.remove_websocket_client(websocket)
        logger.info(f"WebSocket client disconnected. Remaining clients: {client_count}")


async def start_websocket_server():
    """Start the WebSocket server"""
    try:
        import websockets
    except ImportError:
        logger.warning("WebSocket server not started - websockets library not available")
        return

    try:
        logger.info(f"Starting WebSocket server on {WEBSOCKET_HOST}:{WEBSOCKET_PORT} (websockets {websockets.__version__})")

        # Try to bind, retry once on port conflict
        retry_count = 0
        max_retries = 2
        server = None

        while retry_count < max_retries:
            try:
                # websockets 11+ uses origins=None to allow all origins (Safari compatible)
                # process_request was removed in websockets 11+
                server = await websockets.serve(
                    websocket_handler,
                    WEBSOCKET_HOST,
                    WEBSOCKET_PORT,
                    ping_interval=30,
                    ping_timeout=10,
                    close_timeout=5,
                    compression=None,  # Disable permessage-deflate for Safari compatibility
                    origins=None,  # Allow all origins (Safari compatible)
                )
                logger.info(f"WebSocket server started on port {WEBSOCKET_PORT}")
                # Update global flag now that websocket server is confirmed running
                global WEBSOCKETS_AVAILABLE
                WEBSOCKETS_AVAILABLE = True
                break
            except OSError as e:
                if e.errno in (98, 48):  # Address already in use (Linux=98, macOS=48)
                    logger.warning(f"Port {WEBSOCKET_PORT} in use, attempting to kill existing process...")
                    if process_manager.kill_port_process(WEBSOCKET_PORT):
                        logger.info(f"Killed process on port {WEBSOCKET_PORT}, retrying...")
                        retry_count += 1
                        await asyncio.sleep(1)
                    else:
                        logger.error(f"Failed to free port {WEBSOCKET_PORT}")
                        raise
                else:
                    raise

        if server is None:
            logger.error("Failed to start WebSocket server after retries")
            return

        # Keep server running
        await asyncio.Future()  # Run forever
    except Exception as e:
        logger.error(f"WebSocket server error: {e}", exc_info=True)


def start_websocket_server_thread():
    """Start WebSocket server in a separate thread (thread-safe)"""
    try:
        import websockets
    except ImportError:
        logger.warning("WebSocket server thread not started - websockets library not available")
        return

    try:
        # Create new event loop for this thread
        ws_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(ws_loop)

        # Register loop with server state (thread-safe)
        server_state.set_websocket_loop(ws_loop)
        logger.info("WebSocket event loop registered")

        # Start the WebSocket server
        ws_loop.run_until_complete(start_websocket_server())
    except Exception as e:
        logger.error(f"WebSocket server thread error: {e}", exc_info=True)
    finally:
        try:
            ws_loop = server_state.get_websocket_loop()
            if ws_loop:
                ws_loop.close()
        except Exception as e:
            logger.debug(f"Error closing WebSocket loop: {e}")


class ReuseAddressHTTPServer(ThreadingHTTPServer):
    """Multi-threaded HTTPServer with SO_REUSEADDR to prevent 'Address already in use' errors

    Uses ThreadingHTTPServer to handle multiple concurrent requests (especially video streaming).
    Each request gets its own thread, preventing long-running video streams from blocking the UI.
    """
    allow_reuse_address = True
    daemon_threads = True  # Don't wait for threads on shutdown

    # Limit concurrent threads to prevent resource exhaustion
    # Video streaming can use a lot of memory, so cap at reasonable limit
    request_queue_size = 10


class WebRoutesHandler(BaseHTTPRequestHandler):
    """HTTP request handler for web routes server"""

    def log_message(self, format, *args):
        """Override to use logger - log at DEBUG level to reduce verbosity"""
        pass
        # logger.debug(f"{self.address_string()} - {format % args}")

    def send_cors_headers(self):
        """Send CORS headers for local network access"""
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, DELETE, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def _deny_rate_limit(self, payload, retry_after):
        """Send a uniform 429 response for rate limiting"""
        self.send_response(429)
        self.send_header('Retry-After', str(retry_after))
        self.send_cors_headers()
        self.end_headers()
        self.wfile.write(json.dumps(payload).encode())

    def _enforce_rate_limit(self, path, api_only=False, detailed=False, skip_internal=False):
        """Apply rate limiting consistently across verbs"""
        if skip_internal and path.startswith('/_internal/'):
            return True
        if api_only and not path.startswith('/api/'):
            return True

        client_ip = self.client_address[0]
        is_allowed, retry_after = process_manager.check_rate_limit(
            client_ip,
            is_onroad_func=is_onroad,
            max_offroad=RATE_LIMIT_REQUESTS_PER_SECOND_OFFROAD,
            max_onroad=RATE_LIMIT_REQUESTS_PER_SECOND_ONROAD,
            window_seconds=RATE_LIMIT_WINDOW_SECONDS
        )
        if is_allowed:
            return True

        if detailed:
            onroad = is_onroad()
            limit = RATE_LIMIT_REQUESTS_PER_SECOND_ONROAD if onroad else RATE_LIMIT_REQUESTS_PER_SECOND_OFFROAD
            payload = {
                'success': False,
                'error': 'Rate limit exceeded',
                'details': f'Too many requests from {client_ip}',
                'retry_after_seconds': retry_after,
                'limit': f'{limit} requests per second',
                'hint': f'Please wait {retry_after}s before trying again',
                'reason': 'onroad_protection' if onroad else 'rate_limit',
                'timestamp': datetime.now().isoformat()
            }
        else:
            payload = {
                'error': 'Rate limit exceeded',
                'retry_after': retry_after
            }

        self._deny_rate_limit(payload, retry_after)
        return False

    def send_json_response(self, data, status=200):
        """Send JSON response with consistent error format"""
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_cors_headers()
        self.end_headers()

        # Add timestamp to all responses for debugging
        if 'timestamp' not in data:
            data['timestamp'] = datetime.now().isoformat()

        # Ensure error responses have consistent format
        if status >= 400 and 'error' in data:
            if 'success' not in data:
                data['success'] = False
        elif status < 400:
            if 'success' not in data and 'error' not in data:
                data['success'] = True

        self.wfile.write(json.dumps(data).encode())

    def send_file_response(self, filepath, mime_type=None, download_filename=None):
        """Send file response with optional byte-range support"""
        if not os.path.exists(filepath):
            self.send_json_response({'error': 'File not found'}, 404)
            return

        file_size = os.path.getsize(filepath)

        # Parse range header
        range_header = self.headers.get('Range')
        if range_header and range_header.startswith('bytes='):
            try:
                # Parse bytes=start-end format more robustly
                range_spec = range_header.replace('bytes=', '').strip()
                range_parts = range_spec.split('-')

                if len(range_parts) >= 1 and range_parts[0]:
                    start = int(range_parts[0])
                else:
                    start = 0

                if len(range_parts) >= 2 and range_parts[1]:
                    end = int(range_parts[1])
                else:
                    end = file_size - 1

                # Validate range
                if start >= file_size or end >= file_size or start > end:
                    # Invalid range, serve entire file
                    start = 0
                    end = file_size - 1

                length = end - start + 1

                self.send_response(206)  # Partial Content
                self.send_header('Content-Range', f'bytes {start}-{end}/{file_size}')
                self.send_header('Content-Length', str(length))
            except (ValueError, IndexError) as e:
                # Malformed range header, serve entire file
                logger.warning(f"Malformed range header '{range_header}': {e}")
                start = 0
                length = file_size
                self.send_response(200)
                self.send_header('Content-Length', str(file_size))
        else:
            start = 0
            length = file_size
            self.send_response(200)
            self.send_header('Content-Length', str(file_size))

        # Determine MIME type
        if mime_type is None:
            mime_type, _ = mimetypes.guess_type(filepath)
            if mime_type is None:
                if filepath.endswith('.hevc'):
                    # HEVC files - use MP4 container MIME with HEVC codec hint for Safari
                    mime_type = 'video/mp4; codecs="hvc1"'
                elif filepath.endswith('.ts'):
                    # MPEG-TS files - proper MIME type for transport streams
                    mime_type = 'video/mp2t'
                else:
                    mime_type = 'application/octet-stream'

        self.send_header('Content-Type', mime_type)
        self.send_header('Accept-Ranges', 'bytes')
        if download_filename:
            safe_name = download_filename.replace('"', '')
            self.send_header('Content-Disposition', f'attachment; filename="{safe_name}"')

        # Add cache-control headers to prevent stale JS/HTML/CSS from being cached
        # This ensures clients always get the latest version after updates
        if filepath.endswith(('.js', '.html', '.htm', '.css')):
            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Expires', '0')

        self.send_cors_headers()
        self.end_headers()

        # Send file data
        try:
            with open(filepath, 'rb') as f:
                f.seek(start)
                remaining = length
                while remaining > 0:
                    chunk_size = min(8192, remaining)
                    chunk = f.read(chunk_size)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    remaining -= len(chunk)
        except (IOError, OSError) as e:
            logger.error(f"Error sending file {filepath}: {e}")
            # Connection might be broken, don't try to send error response
            return
        except Exception as e:
            logger.error(f"Unexpected error sending file {filepath}: {e}")
            return

    def send_remuxed_hevc(self, hevc_path, route_base, segment_num, camera, debug_mode=False):
        """Remux raw HEVC elementary stream to MP4 container for browser playback

        Uses progressive streaming to start playback immediately while remuxing.

        Args:
            hevc_path: Path to the HEVC video file
            route_base: Route identifier
            segment_num: Segment number
            camera: Camera type
            debug_mode: If True, stream FFmpeg logs to websocket with verbose logging
        """
        if not os.path.exists(hevc_path):
            self.send_json_response({'error': 'HEVC file not found'}, 404)
            return

        # Create cache filename
        cache_filename = f"{route_base}_{segment_num}_{camera}.mp4"
        cache_path = os.path.join(REMUX_CACHE, cache_filename)

        # Check if cached MP4 exists and is newer than source
        if os.path.exists(cache_path):
            cache_mtime = os.path.getmtime(cache_path)
            source_mtime = os.path.getmtime(hevc_path)
            if cache_mtime >= source_mtime:
                logger.info(f"Serving cached MP4: {cache_filename}")
                # Update access time for LRU cache management
                os.utime(cache_path, None)
                self.send_file_response(cache_path)

                # Trigger prefetch of next segments (non-blocking)
                prefetch_next_segments(route_base, segment_num, camera, server_state, count=2)
                return

        # Check cache size and cleanup if needed
        try:
            cleanup_old_cache()
        except Exception as e:
            logger.warning(f"Cache cleanup failed: {e}")

        # Check disk space before remuxing (estimate 2x source file size)
        source_size = os.path.getsize(hevc_path)
        estimated_output_size = source_size * 2  # Conservative estimate
        cache_dir = "/data" if os.path.exists("/data") else os.path.expanduser("~")

        if not has_sufficient_disk_space(estimated_output_size, cache_dir, min_free_gb=0.5):
            free_space = get_free_disk_space(cache_dir)
            free_gb = free_space / (1024**3) if free_space else 0
            logger.error(f"Insufficient disk space to remux {cache_filename}: {free_gb:.1f}GB free")
            self.send_json_response({
                'error': 'Insufficient disk space for video processing',
                'details': f'Only {free_gb:.1f}GB free, need ~{estimated_output_size/(1024**3):.1f}GB',
                'hint': 'Clear cache via settings or delete old routes',
                'free_space_gb': round(free_gb, 1),
                'required_gb': round(estimated_output_size/(1024**3), 1)
            }, 507)  # HTTP 507 Insufficient Storage
            return

        # Enable performance mode for fast remuxing
        enable_performance_mode()

        # Remux using FFmpeg with progressive streaming - use context manager for guaranteed cleanup
        logger.info(f"Remuxing HEVC to MP4 (streaming): {cache_filename}")

        # Use FFmpegProcess context manager for guaranteed cleanup
        route_info = f"{route_base}:{segment_num}:{camera}"
        try:
            # Check if ffmpeg is available
            if FFMPEG_BINARY is None:
                logger.error("FFmpeg not available, cannot stream video")
                self.send_json_response({
                    'error': 'FFmpeg not installed on server',
                    'details': 'Video conversion tool is not available',
                    'hint': 'Contact system administrator or install FFmpeg',
                    'technical_hint': 'apt-get install ffmpeg'
                }, 500)
                return

            with FFmpegProcess(route_info, server_state, max_concurrent=MAX_CONCURRENT_FFMPEG, stream_logs=debug_mode) as ffmpeg_mgr:
                # Use FFmpeg to remux raw HEVC to MP4 container
                # Stream to stdout while also writing to cache file
                # Optimized for Safari HLS compatibility with smaller fragments
                cmd = [
                    FFMPEG_BINARY,
                    '-loglevel', 'error',  # Reduce stderr output to prevent buffer deadlock (overridden in debug mode)
                    '-f', 'hevc',
                    '-r', '20',  # Comma camera framerate (20 fps)
                    '-i', hevc_path,
                    '-c', 'copy',  # Copy all streams (video + audio if available)
                    # Safari HLS buffer-friendly fragmentation:
                    # Create smaller fragments (every 2 seconds) to prevent buffer overflow
                    # Safari's MediaSource buffer can't handle 75MB segments
                    '-movflags', 'frag_every_frame+empty_moov+default_base_moof+omit_tfhd_offset',
                    '-frag_duration', '2000000',  # 2 seconds per fragment (in microseconds)
                    # Timing flags critical for Safari HLS:
                    '-fflags', '+genpts+igndts',  # Generate PTS and ignore DTS for cleaner timestamps
                    '-avoid_negative_ts', 'make_zero',  # Ensure timestamps start at 0
                    '-start_at_zero',  # Force first timestamp to be 0 (Safari requirement)
                    '-vsync', 'cfr',  # Constant frame rate mode (ensures regular timestamps)
                    '-video_track_timescale', '90000',  # Standard timescale for better compatibility
                    '-max_muxing_queue_size', '1024',  # Prevent muxing queue overflow
                    '-f', 'mp4',
                    '-'  # Output to stdout
                ]

                # Start FFmpeg process using context manager (debug_mode enables verbose logging and websocket streaming)
                process = ffmpeg_mgr.start(cmd, debug_mode=debug_mode)

                # Send HTTP headers
                self.send_response(200)
                self.send_header('Content-Type', 'video/mp4')
                self.send_header('Accept-Ranges', 'bytes')
                self.send_cors_headers()
                self.end_headers()

                # Trigger prefetch of next segments (non-blocking, before streaming)
                # This ensures prefetch starts while current segment is being sent
                prefetch_next_segments(route_base, segment_num, camera, server_state, count=2)

                # Stream output while also saving to cache
                with open(cache_path, 'wb') as cache_file:
                    while True:
                        chunk = process.stdout.read(8192)
                        if not chunk:
                            break

                        # Send to client
                        try:
                            self.wfile.write(chunk)
                        except (BrokenPipeError, ConnectionResetError) as e:
                            logger.warning(f"Client disconnected during streaming: {e}")
                            process.kill()
                            try:
                                process.wait(timeout=2)
                            except subprocess.TimeoutExpired:
                                pass  # Context manager will handle cleanup
                            break

                        # Save to cache
                        cache_file.write(chunk)

                # Wait for process to complete
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    logger.warning("FFmpeg process timeout, context manager will handle cleanup")

                # Check result
                if process.returncode != 0:
                    stderr = process.stderr.read().decode('utf-8', errors='ignore')

                    # Exit -9 (SIGKILL) means the process was killed, possibly by OOM killer or cleanup
                    if process.returncode == -9:
                        logger.error(f"FFmpeg was killed (exit -9) - likely out of memory")
                        logger.error(f"HEVC file: {hevc_path}, size: {os.path.getsize(hevc_path) / (1024*1024):.1f}MB")
                        logger.error("This can happen if:")
                        logger.error("  1. System is out of memory (check dmesg for OOM killer messages)")
                        logger.error("  2. FFmpeg process exceeded resource limits")
                        logger.error("  3. Server cleanup was triggered during playback")
                        logger.error(f"FFmpeg stderr: {stderr[:500]}")
                    else:
                        logger.error(f"FFmpeg remux failed (exit {process.returncode}): {stderr[:500]}")

                    # Clean up incomplete cache file
                    if os.path.exists(cache_path):
                        try:
                            os.remove(cache_path)
                        except OSError as e:
                            logger.debug(f"Error removing incomplete cache file: {e}")
                else:
                    logger.info(f"Remux successful: {cache_filename}")

        except RuntimeError as e:
            # Too many concurrent processes
            current_count = server_state.get_ffmpeg_count()
            logger.warning(f"Cannot start FFmpeg: {e}")
            self.send_json_response({
                'error': 'Server busy processing other videos',
                'details': f'{current_count} of {MAX_CONCURRENT_FFMPEG} video streams in use',
                'hint': 'Please wait a moment and try again',
                'active_processes': current_count,
                'max_processes': MAX_CONCURRENT_FFMPEG,
                'retry_after': 5  # Suggest 5 second retry
            }, 503)
            return

        except FileNotFoundError:
            logger.error("FFmpeg not found - install with: apt-get install ffmpeg")
            self.send_json_response({
                'error': 'FFmpeg not installed on server',
                'details': 'Video conversion tool is not available',
                'hint': 'Contact system administrator or install FFmpeg',
                'technical_hint': 'apt-get install ffmpeg'
            }, 500)
            return

        except Exception as e:
            logger.error(f"Unexpected error remuxing {hevc_path}: {e}", exc_info=True)
            # Clean up incomplete cache file
            if os.path.exists(cache_path):
                try:
                    os.remove(cache_path)
                except OSError as cleanup_err:
                    logger.debug(f"Error cleaning up cache file: {cleanup_err}")

            # Try to serve raw HEVC as fallback
            logger.warning(f"Remuxing failed, attempting to serve raw HEVC as fallback")
            try:
                self.send_file_response(hevc_path, 'video/mp4; codecs="hev1"')
                return
            except Exception as fallback_error:
                logger.error(f"Fallback to raw HEVC also failed: {fallback_error}")
                self.send_json_response({
                    'error': 'Video conversion failed',
                    'details': str(e),
                    'hint': 'Try refreshing the page or selecting a different segment',
                    'fallback_attempted': True,
                    'fallback_error': str(fallback_error)
                }, 500)

    def do_OPTIONS(self):
        """Handle OPTIONS for CORS preflight"""
        self.send_response(200)
        self.send_cors_headers()
        self.end_headers()

    def do_GET(self):
        """Handle GET requests"""
        try:
            parsed = urlparse(self.path)
            path = parsed.path

            # Rate limiting check (per-second burst protection) - only for API calls
            # Static resources (JS, CSS, images) are not rate-limited
            if not self._enforce_rate_limit(path, api_only=True, detailed=True):
                return

            # Check if server is enabled (always run when enabled, just rate-limited onroad)
            if not should_server_run():
                self.send_json_response({
                    'error': 'Server disabled',
                    'details': 'Web routes server is currently disabled',
                    'hint': 'Enable server in settings to access routes'
                }, 503)
                return

            # Onroad: Block route-related endpoints but allow settings/params
            # Settings can be modified while driving, routes cannot be browsed
            onroad = is_onroad()
            if onroad and path.startswith('/api/'):
                # Endpoints allowed when onroad (status, params, settings, system info)
                allowed_onroad_prefixes = [
                    '/api/status',
                    '/api/health',
                    '/api/params',
                    '/api/system',
                    '/api/logs',
                    '/api/manager-logs',
                    '/api/websocket_status',
                    '/api/drive-stats',
                    '/api/panels',
                ]
                is_allowed_onroad = any(path.startswith(ep) for ep in allowed_onroad_prefixes)

                if not is_allowed_onroad:
                    # Block route-related operations when onroad
                    self.send_json_response({
                        'error': 'Routes disabled while driving for safety',
                        'onroad': True,
                        'readonly_mode': False,
                        'hint': 'Park the vehicle to access routes and videos'
                    }, 403)
                    return

            # SPA routes - serve index.html for frontend routes
            # This allows direct navigation and page refresh to work
            SPA_ROUTES = {'/', '/index.html', '/settings', '/parameters', '/routes', '/logs'}

            # Route handlers
            if path in SPA_ROUTES or path.startswith('/settings/'):
                self.send_file_response(str(WEBAPP_DIR / 'index.html'), 'text/html')
                return

            # API routes - separate if/elif chain since SPA routes return early
            if path == '/api/health':
                # Health check endpoint for monitoring
                try:
                    onroad = is_onroad()
                    ffmpeg_count = server_state.get_ffmpeg_count()
                    ws_clients = len(server_state.get_websocket_clients())
                    error_summary = server_state.get_error_summary()
                    uptime = server_state.get_server_uptime()

                    # Server is unhealthy if there are recent CRITICAL errors
                    recent_critical = [e for e in server_state.get_recent_errors(limit=10)
                                      if e['level'] == 'CRITICAL']
                    is_healthy = len(recent_critical) == 0

                    self.send_json_response({
                        'healthy': is_healthy,
                        'status': 'onroad' if onroad else 'online',
                        'onroad': onroad,
                        'server_enabled': should_server_run(),
                        'uptime_seconds': int(uptime),
                        'ffmpeg_available_slots': MAX_CONCURRENT_FFMPEG - ffmpeg_count,
                        'websocket_clients': ws_clients,
                        'errors': {
                            'total': error_summary['total'],
                            'critical': error_summary.get('CRITICAL', 0),
                            'errors': error_summary.get('ERROR', 0),
                            'warnings': error_summary.get('WARNING', 0),
                            'has_recent_critical': len(recent_critical) > 0
                        }
                    })
                except Exception as e:
                    logger.error(f"Health check failed: {e}", exc_info=True)
                    self.send_json_response({
                        'healthy': False,
                        'error': str(e)
                    }, 500)

            elif path == '/api/status':
                # Basic status endpoint (lightweight, always available)
                onroad = is_onroad()

                # Fast route count (no full scan - just count unique base routes)
                # Full route scanning is handled by:
                # - /api/routes endpoint (when viewing routes page)
                # - Background preprocessor (every 30s, broadcasts via WebSocket)
                routes_count = 0
                if os.path.exists(ROUTES_DIR):
                    try:
                        # Count unique route base names (not individual segments)
                        unique_routes = set()
                        for entry in os.listdir(ROUTES_DIR):
                            if (os.path.isdir(os.path.join(ROUTES_DIR, entry))
                                and '--' in entry
                                and entry not in ('boot', 'crash')):
                                # Extract base route name (removes segment number)
                                base_name = get_route_base_name(entry)
                                unique_routes.add(base_name)
                        routes_count = len(unique_routes)
                    except Exception:
                        pass

                # Count params by listing directory
                params_count = 0
                try:
                    params_dir = "/data/params/d" if os.path.exists("/data/params/d") else None
                    if params_dir:
                        params_count = len(os.listdir(params_dir))
                except Exception:
                    pass

                self.send_json_response({
                    'status': 'onroad' if onroad else 'online',
                    'onroad': onroad,
                    'routes_count': routes_count,
                    'params_count': params_count,
                    'routes_dir': ROUTES_DIR,
                    'routes_dir_exists': os.path.exists(ROUTES_DIR),
                    'isMetric': params.get_bool("IsMetric"),
                    'timestamp': datetime.now().isoformat()
                })

            elif path == '/api/system/metrics':
                # System metrics endpoint (CPU, memory, temperature, disk)
                # Frontend expects: SystemMetrics interface format
                try:
                    raw_metrics = get_system_metrics(
                        cache_sizes_func=get_all_cache_sizes,
                        ffmpeg_state=server_state,
                        max_ffmpeg=MAX_CONCURRENT_FFMPEG
                    )

                    # Get system uptime
                    uptime_seconds = 0
                    try:
                        with open('/proc/uptime', 'r') as f:
                            uptime_seconds = int(float(f.read().split()[0]))
                    except Exception as e:
                        logger.debug(f"Could not read uptime: {e}")

                    # Transform to frontend's expected format
                    formatted_metrics = {
                        'cpu_load': raw_metrics.get('cpu', {}).get('load_1min', 0),
                        'cpu_cores': raw_metrics.get('cpu', {}).get('core_count', 0),
                        'memory_used': raw_metrics.get('memory', {}).get('used_bytes', 0),
                        'memory_total': raw_metrics.get('memory', {}).get('total_bytes', 0),
                        'memory_percent': raw_metrics.get('memory', {}).get('percent_used', 0),
                        'disk_used': raw_metrics.get('disk', {}).get('/data', {}).get('used_bytes', 0),
                        'disk_total': raw_metrics.get('disk', {}).get('/data', {}).get('total_bytes', 0),
                        'disk_percent': raw_metrics.get('disk', {}).get('/data', {}).get('percent_used', 0),
                        'temperature': raw_metrics.get('temperature', {}).get('celsius'),
                        'ffmpeg_processes': raw_metrics.get('ffmpeg', {}).get('active_processes', 0),
                        'cache_size': raw_metrics.get('cache', {}).get('total_bytes', 0),
                        'uptime_seconds': uptime_seconds,
                    }

                    self.send_json_response(formatted_metrics)
                except Exception as e:
                    logger.error(f"Error getting system metrics: {e}", exc_info=True)
                    self.send_json_response({
                        'error': str(e),
                        'timestamp': datetime.now().isoformat()
                    }, 500)

            elif path == '/api/system/device-info':
                # Device information endpoint (dongle ID, serial, versions)
                try:
                    device_info = {}

                    # Get DongleId from params
                    try:
                        dongle_id_bytes = params.get("DongleId")
                        if dongle_id_bytes:
                            if isinstance(dongle_id_bytes, bytes):
                                dongle_id = dongle_id_bytes.decode('utf-8').strip()
                                device_info['dongle_id'] = dongle_id if dongle_id else None
                            elif isinstance(dongle_id_bytes, str):
                                dongle_id = dongle_id_bytes.strip()
                                device_info['dongle_id'] = dongle_id if dongle_id else None
                            else:
                                logger.warning(f"DongleId has unexpected type: {type(dongle_id_bytes)}")
                                device_info['dongle_id'] = None
                        else:
                            logger.debug("DongleId not found in params (device not registered?)")
                            device_info['dongle_id'] = None
                    except Exception as e:
                        logger.warning(f"Error getting DongleId: {e}", exc_info=True)
                        device_info['dongle_id'] = None

                    # Get HardwareSerial from params
                    try:
                        serial_bytes = params.get("HardwareSerial")
                        if serial_bytes:
                            if isinstance(serial_bytes, bytes):
                                serial = serial_bytes.decode('utf-8').strip()
                                device_info['serial'] = serial if serial else None
                            elif isinstance(serial_bytes, str):
                                serial = serial_bytes.strip()
                                device_info['serial'] = serial if serial else None
                            else:
                                logger.warning(f"HardwareSerial has unexpected type: {type(serial_bytes)}")
                                device_info['serial'] = None
                        else:
                            logger.debug("HardwareSerial not found in params")
                            device_info['serial'] = None
                    except Exception as e:
                        logger.warning(f"Error getting HardwareSerial: {e}", exc_info=True)
                        device_info['serial'] = None

                    # Get BluePilot Version from BPVERSION file
                    try:
                        bp_version_path = os.path.join(os.path.dirname(__file__), '../../BPVERSION')
                        if os.path.exists(bp_version_path):
                            with open(bp_version_path, 'r') as f:
                                device_info['bp_version'] = f.read().strip()
                        else:
                            device_info['bp_version'] = None
                    except Exception as e:
                        logger.debug(f"Error reading BPVERSION: {e}")
                        device_info['bp_version'] = None

                    # Get Openpilot Version from common/version.h file
                    try:
                        op_version_path = os.path.join(os.path.dirname(__file__), '../../common/version.h')
                        if os.path.exists(op_version_path):
                            with open(op_version_path, 'r') as f:
                                content = f.read()
                                # Extract version from #define COMMA_VERSION "0.10.1"
                                import re
                                match = re.search(r'#define\s+COMMA_VERSION\s+"([^"]+)"', content)
                                if match:
                                    device_info['op_version'] = match.group(1)
                                else:
                                    device_info['op_version'] = None
                        else:
                            device_info['op_version'] = None
                    except Exception as e:
                        logger.debug(f"Error reading openpilot version: {e}")
                        device_info['op_version'] = None

                    # Get SunnyPilot Version from version.h file
                    try:
                        sp_version_path = os.path.join(os.path.dirname(__file__), '../../sunnypilot/common/version.h')
                        if os.path.exists(sp_version_path):
                            with open(sp_version_path, 'r') as f:
                                content = f.read()
                                # Extract version from #define SUNNYPILOT_VERSION "2025.003.000"
                                import re
                                match = re.search(r'#define\s+SUNNYPILOT_VERSION\s+"([^"]+)"', content)
                                if match:
                                    device_info['sp_version'] = match.group(1)
                                else:
                                    device_info['sp_version'] = None
                        else:
                            device_info['sp_version'] = None
                    except Exception as e:
                        logger.debug(f"Error reading SunnyPilot version: {e}")
                        device_info['sp_version'] = None

                    self.send_json_response(device_info)
                except Exception as e:
                    logger.error(f"Error getting device info: {e}", exc_info=True)
                    self.send_json_response({
                        'error': str(e),
                        'timestamp': datetime.now().isoformat()
                    }, 500)

            elif path == '/api/websocket_status':
                # WebSocket availability status endpoint
                ws_clients = len(server_state.get_websocket_clients())
                self.send_json_response({
                    'websockets_available': WEBSOCKETS_AVAILABLE,
                    'websocket_clients': ws_clients,
                    'websocket_port': WEBSOCKET_PORT if WEBSOCKETS_AVAILABLE else None
                })

            elif path == '/api/status/detailed':
                # Detailed status endpoint with connection info
                try:
                    onroad = is_onroad()
                    connection_type = get_connection_type()
                    wifi_ip = get_wifi_ip()
                    all_wifi_ips = get_all_wifi_ips()
                    try:
                        tethering_enabled = params.get_bool("EnableTethering")
                    except Exception:
                        tethering_enabled = False

                    # Server uptime
                    try:
                        import psutil
                        process = psutil.Process(os.getpid())
                        uptime_seconds = time.time() - process.create_time()
                    except Exception as e:
                        logger.debug(f"Could not get uptime: {e}")
                        uptime_seconds = int(server_state.get_server_uptime())

                    # WebSocket client count (thread-safe)
                    ws_clients = len(server_state.get_websocket_clients())

                    # FFmpeg info (thread-safe)
                    ffmpeg_count = server_state.get_ffmpeg_count()

                    # Rate limit info
                    current_limit = RATE_LIMIT_REQUESTS_PER_SECOND_ONROAD if onroad else RATE_LIMIT_REQUESTS_PER_SECOND_OFFROAD

                    self.send_json_response({
                    'status': 'onroad' if onroad else 'online',
                    'onroad': onroad,
                    'server': {
                        'uptime_seconds': int(uptime_seconds),
                        'version': '1.0.0',
                        'python_version': sys.version.split()[0],
                        'bind_address': '0.0.0.0'  # Always binds to all interfaces
                    },
                    'network': {
                        'connection_type': connection_type,
                        'wifi_ip': wifi_ip,
                        'wifi_available': wifi_ip is not None,
                        'all_wifi_ips': [{'interface': iface, 'ip': ip} for iface, ip in all_wifi_ips],
                        'tethering_enabled': tethering_enabled,
                        'port': int(params.get("BPPortalPort") or "8088")
                    },
                    'clients': {
                        'websocket_connected': ws_clients,
                        'websocket_available': WEBSOCKETS_AVAILABLE
                    },
                    'rate_limit': {
                        'requests_per_minute': current_limit,
                        'window_seconds': RATE_LIMIT_WINDOW_SECONDS,
                        'mode': 'global' if onroad else 'per_ip'
                    },
                    'ffmpeg': {
                        'active_processes': ffmpeg_count,
                        'max_processes': MAX_CONCURRENT_FFMPEG,
                        'available_slots': MAX_CONCURRENT_FFMPEG - ffmpeg_count,
                        'utilization_percent': int((ffmpeg_count / MAX_CONCURRENT_FFMPEG) * 100) if MAX_CONCURRENT_FFMPEG > 0 else 0
                    },
                    'features': {
                        'read_only': onroad,  # Onroad = read-only mode
                        'write_operations_enabled': not onroad,
                        'websocket_enabled': WEBSOCKETS_AVAILABLE
                    }
                    })
                except Exception as e:
                    logger.error(f"Error in /api/status/detailed: {e}", exc_info=True)
                    self.send_json_response({
                        'success': False,
                        'error': 'Failed to get detailed status',
                        'details': str(e)
                    }, 500)

            elif path == '/api/logs':
                # Error logs endpoint for debugging
                try:
                    # Parse query parameters for filtering
                    query = urlparse(self.path).query
                    params_dict = dict(param.split('=') for param in query.split('&') if '=' in param) if query else {}

                    limit = int(params_dict.get('limit', 20))
                    level = params_dict.get('level', None)  # 'ERROR', 'WARNING', 'CRITICAL'

                    errors = server_state.get_recent_errors(limit=limit, level=level)
                    error_summary = server_state.get_error_summary()

                    self.send_json_response({
                        'success': True,
                        'logs': errors,
                        'summary': error_summary,
                        'server_uptime': int(server_state.get_server_uptime())
                    })
                except Exception as e:
                    logger.error(f"Error retrieving logs: {e}", exc_info=True)
                    self.send_json_response({
                        'success': False,
                        'error': 'Failed to retrieve logs',
                        'details': str(e)
                    }, 500)

            elif path == '/api/last-error':
                # Get the most recent crash/error from the crashes file
                # This is the same file shown in the BP Developer Panel
                try:
                    crash_file = '/data/community/crashes/error.txt'

                    if not os.path.exists(crash_file):
                        self.send_json_response({
                            'success': True,
                            'has_error': False,
                            'message': 'No crash file found'
                        })
                        return

                    # Get file stats
                    file_stat = os.stat(crash_file)
                    file_size = file_stat.st_size
                    file_mtime = file_stat.st_mtime

                    # Skip if file is empty
                    if file_size == 0:
                        self.send_json_response({
                            'success': True,
                            'has_error': False,
                            'message': 'Crash file is empty'
                        })
                        return

                    # Limit file size to 100KB for safety
                    if file_size > 100 * 1024:
                        # Read only the last 100KB
                        with open(crash_file, 'r', encoding='utf-8', errors='replace') as f:
                            f.seek(max(0, file_size - 100 * 1024))
                            content = f.read()
                    else:
                        with open(crash_file, 'r', encoding='utf-8', errors='replace') as f:
                            content = f.read()

                    # Parse the error content to extract useful info
                    lines = content.strip().split('\n')

                    # Try to extract timestamp and error type from content
                    timestamp = None
                    error_type = 'ERROR'
                    error_message = lines[0] if lines else 'Unknown error'

                    # Look for common timestamp patterns
                    for line in lines[:10]:  # Check first 10 lines for timestamp
                        # Pattern: 2024-01-15 10:30:45 or similar
                        ts_match = re.search(r'(\d{4}-\d{2}-\d{2}[T\s]\d{2}:\d{2}:\d{2})', line)
                        if ts_match:
                            timestamp = ts_match.group(1)
                            break

                    # Use file modification time if no timestamp found in content
                    if not timestamp:
                        timestamp = datetime.fromtimestamp(file_mtime).isoformat()

                    # Determine error type from content
                    content_lower = content.lower()
                    if 'critical' in content_lower or 'fatal' in content_lower:
                        error_type = 'CRITICAL'
                    elif 'exception' in content_lower or 'traceback' in content_lower:
                        error_type = 'ERROR'
                    elif 'warning' in content_lower:
                        error_type = 'WARNING'

                    self.send_json_response({
                        'success': True,
                        'has_error': True,
                        'error': {
                            'timestamp': timestamp,
                            'level': error_type,
                            'message': error_message[:500],  # First line, truncated
                            'details': content[:5000] if len(content) > 500 else None,  # Full content if longer
                            'file_path': crash_file,
                            'file_size': file_size,
                            'file_modified': file_mtime
                        }
                    })

                except Exception as e:
                    logger.error(f"Error reading crash file: {e}", exc_info=True)
                    self.send_json_response({
                        'success': False,
                        'error': 'Failed to read crash file',
                        'details': str(e)
                    }, 500)

            elif path.startswith('/api/system/metrics'):
                # System health metrics
                try:
                    metrics = get_system_metrics()
                    self.send_json_response({
                        'success': True,
                        'metrics': metrics
                    })
                except Exception as e:
                    logger.exception("Error getting system metrics")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path == '/api/params':
                # Get all parameters
                try:
                    all_params = get_all_params(params)
                    self.send_json_response({
                        'success': True,
                        'params': all_params
                    })
                except Exception as e:
                    logger.exception("Error getting params")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path == '/api/params/categories':
                # Get parameters organized by category
                try:
                    categorized = get_params_by_category(params)
                    self.send_json_response({
                        'success': True,
                        'categories': categorized
                    })
                except Exception as e:
                    logger.exception("Error getting params by category")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path.startswith('/api/params/search'):
                # Search parameters
                try:
                    query = parse_qs(parsed.query).get('q', [''])[0]
                    if not query:
                        self.send_json_response({
                            'success': False,
                            'error': 'Missing query parameter "q"'
                        }, 400)
                        return

                    results = search_params(query, params)
                    self.send_json_response({
                        'success': True,
                        'query': query,
                        'results': results
                    })
                except Exception as e:
                    logger.exception("Error searching params")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path == '/api/manager-logs':
                # Get recent manager logs from swaglog files (non-streaming)
                try:
                    manager_ok, manager_output = read_recent_manager_logs()
                except Exception as exc:  # pragma: no cover - defensive
                    logger.exception("Error reading manager logs")
                    manager_ok = False
                    manager_output = str(exc)

                payload = {
                    'timestamp': datetime.now().isoformat(),
                    'output': manager_output,
                    'source': 'swaglog',
                    'success': manager_ok,
                }

                if manager_ok:
                    self.send_json_response(payload)
                else:
                    payload['error'] = manager_output
                    self.send_json_response(payload, 500)

            elif path == '/api/manager-logs/stream/status':
                # Get log streaming status
                try:
                    from bluepilot.backend.realtime.log_streamer import get_log_streamer

                    streamer = get_log_streamer()
                    is_running = streamer.is_running() if streamer else False

                    self.send_json_response({
                        'success': True,
                        'running': is_running
                    })
                except Exception as e:
                    logger.exception("Error getting log stream status")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path.startswith('/api/params/get/'):
                # Get specific parameter
                try:
                    param_key = path.split('/api/params/get/')[1].strip('/')
                    if not param_key:
                        self.send_json_response({
                            'success': False,
                            'error': 'Parameter key required'
                        }, 400)
                        return

                    value = get_param_value(param_key, params)
                    self.send_json_response({
                        'success': True,
                        'key': param_key,
                        'value': value,
                        'readonly': param_key in READONLY_PARAMS,
                        'critical': param_key in CRITICAL_PARAMS
                    })
                except Exception as e:
                    logger.exception(f"Error getting param {param_key}")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path.startswith('/api/routes/') and '/video/' not in path:
                # Get specific route details
                route_base = path.split('/api/routes/')[1].strip('/')
                segments = get_route_segments(route_base)

                if not segments:
                    self.send_json_response({'success': False, 'error': 'Route not found'}, 404)
                    return

                payload = build_route_metadata(route_base, segments, params)
                self.send_json_response(payload)

            elif path == '/api/routes':
                # Fast route list (cached data only, no processing)
                routes = scan_routes()
                disk_info = get_disk_space_info()
                self.send_json_response({
                    'success': True,
                    'routes': routes,
                    'total': len(routes),
                    'disk_space': disk_info
                })

            elif path == '/api/disk-space':
                # Get current disk space status
                # Frontend expects: DiskSpace interface format
                disk_info = get_disk_space_info()
                self.send_json_response({
                    'total': disk_info.get('total_bytes', 0),
                    'used': disk_info.get('used_bytes', 0),
                    'free': disk_info.get('available_bytes', 0),
                    'percent': 100 - disk_info.get('available_percent', 0)
                })

            elif path == '/api/disk-analysis':
                # Get comprehensive disk analysis with deletion predictions
                disk_info = get_disk_space_info()
                deletion_data = get_cached_deletion_data()

                # Calculate protected vs non-preserved space
                preserved_bytes = 0
                non_preserved_bytes = 0

                for seg_name in deletion_data['all_segments']:
                    seg_path = os.path.join(ROUTES_DIR, seg_name)
                    seg_size = get_file_size(seg_path)

                    if seg_name in deletion_data['preserved_set']:
                        preserved_bytes += seg_size
                    else:
                        non_preserved_bytes += seg_size

                # Get next segments to be deleted
                next_to_delete = []
                for i, seg_name in enumerate(deletion_data['deletion_queue'][:10]):
                    seg_path = os.path.join(ROUTES_DIR, seg_name)
                    route_base = get_route_base_name(seg_name)

                    next_to_delete.append({
                        'segment': seg_name,
                        'route_base': route_base,
                        'size_bytes': get_file_size(seg_path),
                        'rank': i + 1
                    })

                # Get cache sizes
                cache_sizes = get_all_cache_sizes()

                self.send_json_response({
                    'success': True,
                    'disk': {
                        'total_bytes': disk_info['total_bytes'],
                        'used_bytes': disk_info['used_bytes'],
                        'free_bytes': disk_info['available_bytes'],
                        'routes_bytes': preserved_bytes + non_preserved_bytes,
                        'preserved_bytes': preserved_bytes,
                        'non_preserved_bytes': non_preserved_bytes,
                        'deletion_threshold_bytes': MIN_BYTES,
                        'space_until_threshold': max(0, disk_info['available_bytes'] - MIN_BYTES),
                        'deletion_active': disk_info['available_bytes'] < MIN_BYTES,
                        'warning_level': disk_info['warning_level'],
                        'formatted': {
                            'total': format_size(disk_info['total_bytes']),
                            'used': format_size(disk_info['used_bytes']),
                            'free': format_size(disk_info['available_bytes']),
                            'routes': format_size(preserved_bytes + non_preserved_bytes),
                            'preserved': format_size(preserved_bytes),
                            'non_preserved': format_size(non_preserved_bytes),
                            'threshold': format_size(MIN_BYTES),
                        }
                    },
                    'cache': {
                        'remux_bytes': cache_sizes['remux_bytes'],
                        'thumbnail_bytes': cache_sizes['thumbnail_bytes'],
                        'metrics_bytes': cache_sizes['metrics_bytes'],
                        'drive_stats_bytes': cache_sizes['drive_stats_bytes'],
                        'fingerprint_bytes': cache_sizes['fingerprint_bytes'],
                        'export_bytes': cache_sizes['export_bytes'],
                        'total_bytes': cache_sizes['total_bytes'],
                        'formatted': cache_sizes['formatted']
                    },
                    'deletion_queue': {
                        'total_segments': deletion_data['total_count'],
                        'protected_segments': deletion_data['protected_count'],
                        'at_risk_segments': deletion_data['at_risk_count'],
                        'next_10_to_delete': next_to_delete
                    }
                })

            elif path.startswith('/api/geocode/'):
                # Geocode a specific route: /api/geocode/{route_base}
                route_base = path.split('/api/geocode/')[1].strip('/')
                segments = get_route_segments(route_base)

                if not segments:
                    self.send_json_response({'error': 'Route not found'}, 404)
                    return

                # Get GPS coordinates (should be cached)
                gps_metrics = get_route_gps_metrics(route_base, segments, include_coordinates=True)

                if not gps_metrics.get('has_gps_data'):
                    self.send_json_response({
                        'success': True,
                        'baseName': route_base,
                        'startLocation': None,
                        'endLocation': None,
                        'hasGpsData': False
                    })
                    return

                coordinates = gps_metrics.get('coordinates', [])
                if coordinates:
                    start_coord = coordinates[0]
                    end_coord = coordinates[-1]

                    start_location = reverse_geocode(start_coord['lat'], start_coord['lon'])
                    end_location = reverse_geocode(end_coord['lat'], end_coord['lon'])

                    # Save location names to GPS metrics cache for future use
                    cache_file = os.path.join(METRICS_CACHE, f"{route_base}.json")
                    try:
                        if os.path.exists(cache_file):
                            with open(cache_file, 'r') as f:
                                cached_data = json.load(f)

                            cached_data['start_location'] = start_location
                            cached_data['end_location'] = end_location

                            with open(cache_file, 'w') as f:
                                json.dump(cached_data, f)
                    except Exception as e:
                        logger.warning(f"Error saving location names to cache: {e}")
                else:
                    start_location = None
                    end_location = None

                self.send_json_response({
                    'success': True,
                    'baseName': route_base,
                    'startLocation': start_location,
                    'endLocation': end_location,
                    'hasGpsData': True
                })

            elif path.startswith('/api/hls/'):
                # Generate HLS manifest for route playback
                # Parse: /api/hls/{route_base}/{camera}/playlist.m3u8
                parts = path.split('/')[3:]  # Skip '', 'api', 'hls'
                if len(parts) < 3 or not parts[2].endswith('.m3u8'):
                    self.send_json_response({'error': 'Invalid HLS path'}, 400)
                    return

                route_base = parts[0]
                camera = parts[1]

                # Get all segments for this route
                segments = get_route_segments(route_base)
                if not segments:
                    self.send_json_response({'error': 'Route not found'}, 404)
                    return

                camera_file = CAMERA_FILES.get(camera)
                if not camera_file:
                    self.send_json_response({'error': 'Invalid camera type'}, 400)
                    return

                # Get base URL for absolute paths (Safari requires this)
                host = self.headers.get('Host', 'localhost:5050')
                # Determine protocol (check X-Forwarded-Proto for proxy scenarios)
                proto = self.headers.get('X-Forwarded-Proto', 'http')
                base_url = f"{proto}://{host}"

                # Generate HLS playlist with accurate durations for Safari compatibility
                playlist = "#EXTM3U\n"
                playlist += "#EXT-X-VERSION:7\n"  # Version 7 supports fragmented MP4
                playlist += "#EXT-X-INDEPENDENT-SEGMENTS\n"
                playlist += "#EXT-X-TARGETDURATION:61\n"  # Max duration (rounded up for safety)
                playlist += "#EXT-X-MEDIA-SEQUENCE:0\n"
                playlist += "#EXT-X-DISCONTINUITY-SEQUENCE:0\n"
                playlist += "#EXT-X-PLAYLIST-TYPE:VOD\n"

                # For fragmented MP4, we don't need EXT-X-MAP since each segment is independent
                # The frag_keyframe+empty_moov flags make each segment self-contained

                # Attempt to derive a real start time for EXT-X-PROGRAM-DATE-TIME markers
                route_start_dt = parse_route_datetime(route_base)
                if route_start_dt is not None:
                    # Treat stored timestamps as UTC for consistent program-date markers
                    route_start_dt = route_start_dt.replace(tzinfo=timezone.utc)
                else:
                    try:
                        first_seg_path = segments[0]['path']
                        route_start_dt = datetime.fromtimestamp(
                            os.path.getmtime(first_seg_path),
                            tz=timezone.utc
                        )
                    except Exception:
                        route_start_dt = None

                program_date_cursor = route_start_dt
                max_duration = 0
                segment_count = 0
                for idx, seg in enumerate(segments):
                    seg_path = os.path.join(seg['path'], camera_file)
                    if os.path.exists(seg_path):
                        segment_num = seg['segment']

                        # Try to get duration from cached MP4 first (most accurate)
                        duration = get_video_duration_from_cache(route_base, segment_num, camera)

                        # If not cached, analyze the raw file
                        if duration is None:
                            duration = get_video_duration(seg_path)

                        if duration is None or duration <= 0:
                            duration = 60.0

                        max_duration = max(max_duration, duration)

                        if idx > 0:
                            playlist += "#EXT-X-DISCONTINUITY\n"

                        if program_date_cursor is not None:
                            iso_cursor = program_date_cursor.isoformat().replace('+00:00', 'Z')
                            playlist += f"#EXT-X-PROGRAM-DATE-TIME:{iso_cursor}\n"
                            program_date_cursor += timedelta(seconds=duration)

                        # Use accurate duration in EXTINF (required for Safari seeking/scrubbing)
                        playlist += f"#EXTINF:{duration:.3f},\n"
                        # Use absolute URLs for better Safari compatibility
                        playlist += f"{base_url}/api/video/{route_base}/{segment_num}/{camera}\n"
                        segment_count += 1

                if segment_count == 0:
                    self.send_json_response({'error': 'No video segments available'}, 404)
                    return

                # Update target duration if needed (must be >= max segment duration)
                if max_duration > 61:
                    # Regenerate with correct target duration
                    target_duration = int(max_duration) + 1
                    playlist = playlist.replace(
                        "#EXT-X-TARGETDURATION:61\n",
                        f"#EXT-X-TARGETDURATION:{target_duration}\n"
                    )

                playlist += "#EXT-X-ENDLIST\n"

                # Send playlist
                self.send_response(200)
                self.send_header('Content-Type', 'application/vnd.apple.mpegurl')
                self.send_header('Cache-Control', 'no-cache')  # Prevent stale playlists
                self.send_cors_headers()
                self.end_headers()
                self.wfile.write(playlist.encode())

                logger.debug(f"HLS playlist generated for {route_base}/{camera}: {len(segments)} segments, max duration {max_duration:.3f}s")

            elif path.startswith('/api/video/'):
                # Parse: /api/video/{route_base}/{segment}/{camera}
                parts = path.split('/')[3:]  # Skip '', 'api', 'video'
                if len(parts) < 3:
                    self.send_json_response({'error': 'Invalid video path'}, 400)
                    return

                route_base = parts[0]
                try:
                    segment_num = int(parts[1])
                except ValueError:
                    self.send_json_response({'error': 'Invalid segment number'}, 400)
                    return
                camera = parts[2]

                # Check for debug mode in query parameters
                query_params = parse_qs(parsed.query)
                debug_mode = query_params.get('debug', ['false'])[0].lower() == 'true'

                # Get segment
                segments = get_route_segments(route_base)
                segment_data = next((s for s in segments if s['segment'] == segment_num), None)

                if not segment_data:
                    self.send_json_response({'error': 'Segment not found'}, 404)
                    return

                camera_filename = CAMERA_FILES.get(camera)
                if not camera_filename:
                    self.send_json_response({'error': 'Invalid camera type'}, 400)
                    return

                video_path = os.path.join(segment_data['path'], camera_filename)

                # For HEVC files, remux to MP4 for browser compatibility
                if camera in ['front', 'wide', 'driver']:
                    self.send_remuxed_hevc(video_path, route_base, segment_num, camera, debug_mode=debug_mode)
                else:
                    self.send_file_response(video_path)

            elif path.startswith('/api/route-export-stream/'):
                # Streaming endpoint: /api/route-export-stream/{route_base}/{camera}
                parts = path.split('/')[3:]
                if len(parts) < 2:
                    self.send_json_response({'error': 'Invalid route export stream path'}, 400)
                    return

                route_base = parts[0]
                camera = parts[1]

                if camera not in CAMERA_FILES:
                    self.send_json_response({'error': 'Invalid camera type'}, 400)
                    return

                try:
                    stream_route_export(route_base, camera, self, server_state)
                except FileNotFoundError as e:
                    self.send_json_response({'error': str(e)}, 404)
                except ValueError as e:
                    self.send_json_response({'error': str(e)}, 400)
                except Exception as e:
                    logger.error(f"Error streaming route export: {e}", exc_info=True)
                    self.send_json_response({'error': f'Stream failed: {str(e)}'}, 500)

            elif path.startswith('/api/route-export/'):
                # Status endpoint for route exports: /api/route-export/{route_base}/{camera}
                parts = path.split('/')[3:]
                if len(parts) < 2:
                    self.send_json_response({'error': 'Invalid route export path'}, 400)
                    return

                route_base = parts[0]
                camera = parts[1]

                payload, status_code = self.build_route_export_status(route_base, camera)
                self.send_json_response(payload, status_code)

            elif path.startswith('/api/download/route/'):
                # Download full route: /api/download/route/{route_base}/{camera}
                parts = path.split('/')[4:]  # Skip '', 'api', 'download', 'route'
                if len(parts) < 2:
                    self.send_json_response({'error': 'Invalid download path'}, 400)
                    return

                route_base = parts[0]
                camera = parts[1]

                # Get all segments for this route
                segments = get_route_segments(route_base)
                if not segments:
                    self.send_json_response({'error': 'Route not found'}, 404)
                    return

                self.download_full_route(route_base, camera, segments)

            elif path.startswith('/api/download/qlog/'):
                # Download qlog: /api/download/qlog/{route_base}
                handle_qlog_download(self, path, get_route_segments, server_state)
                return

            elif path.startswith('/api/download/rlog/'):
                # Download rlog: /api/download/rlog/{route_base}
                handle_rlog_download(self, path, get_route_segments, server_state)
                return

            elif path.startswith('/api/route-coordinates/'):
                # New endpoint: /api/route-coordinates/{route_base}
                route_base = path.split('/api/route-coordinates/')[1].strip('/')
                segments = get_route_segments(route_base)

                if not segments:
                    self.send_json_response({'error': 'Route not found'}, 404)
                    return

                # Extract GPS coordinates with caching (separate from metrics cache)
                gps_data = get_route_gps_metrics(route_base, segments, include_coordinates=True)

                if gps_data.get('has_gps_data') and 'coordinates' in gps_data:
                    self.send_json_response({
                        'success': True,
                        'baseName': route_base,
                        'coordinates': gps_data['coordinates'],
                        'pointCount': len(gps_data['coordinates'])
                    })
                else:
                    self.send_json_response({
                        'success': False,
                        'error': 'No GPS data available for this route'
                    }, 404)

            elif path.startswith('/api/route/') and '/camera-sizes' in path:
                # Camera sizes endpoint: /api/route/{route_base}/camera-sizes
                route_base = path.split('/api/route/')[1].replace('/camera-sizes', '').strip('/')
                segments = get_route_segments(route_base)

                if not segments:
                    self.send_json_response({'error': 'Route not found'}, 404)
                    return

                # Calculate total size for each camera and log file across all segments
                camera_sizes = {
                    'front': 0,
                    'wide': 0,
                    'driver': 0,
                    'lq': 0,
                    'qlog': 0,
                    'rlog': 0
                }

                for segment in segments:
                    segment_path = segment['path']
                    # Camera sizes
                    for camera, filename in CAMERA_FILES.items():
                        file_path = os.path.join(segment_path, filename)
                        if os.path.exists(file_path):
                            try:
                                camera_sizes[camera] += os.path.getsize(file_path)
                            except Exception as e:
                                logger.debug(f"Error getting size for {file_path}: {e}")

                    # Log file sizes
                    for log_type in ['qlog', 'rlog']:
                        log_path = os.path.join(segment_path, f'{log_type}.zst')
                        if os.path.exists(log_path):
                            try:
                                camera_sizes[log_type] += os.path.getsize(log_path)
                            except Exception as e:
                                logger.debug(f"Error getting size for {log_path}: {e}")

                self.send_json_response({
                    'success': True,
                    'baseName': route_base,
                    'front': camera_sizes['front'],
                    'wide': camera_sizes['wide'],
                    'driver': camera_sizes['driver'],
                    'lq': camera_sizes['lq'],
                    'qlog': camera_sizes['qlog'],
                    'rlog': camera_sizes['rlog']
                })

            elif path.startswith('/api/logs/'):
                # Log messages endpoint: /api/logs/{route_base}/{segment}/{log_type}
                # Query params: search, level (info/warning/error/all), max
                parts = path.split('/')[3:]  # Skip '', 'api', 'logs'
                if len(parts) < 3:
                    self.send_json_response({'error': 'Invalid log path. Format: /api/logs/{route}/{segment}/{qlog|rlog}'}, 400)
                    return

                route_base = parts[0]
                try:
                    segment_num = int(parts[1])
                except ValueError:
                    self.send_json_response({'error': 'Invalid segment number'}, 400)
                    return

                log_type = parts[2].lower()
                if log_type not in ('qlog', 'rlog'):
                    self.send_json_response({'error': 'Invalid log type. Use qlog or rlog'}, 400)
                    return

                # Get segment data
                segments = get_route_segments(route_base)
                segment_data = next((s for s in segments if s['segment'] == segment_num), None)

                if not segment_data:
                    self.send_json_response({'error': 'Segment not found'}, 404)
                    return

                # Map log type to filename
                log_files = {
                    'qlog': 'qlog.zst',
                    'rlog': 'rlog.zst'
                }

                log_path = os.path.join(segment_data['path'], log_files[log_type])

                if not os.path.exists(log_path):
                    self.send_json_response({
                        'success': False,
                        'error': f'{log_type} file not found for this segment'
                    }, 404)
                    return

                # Parse query parameters
                query_params = parse_qs(parsed.query)
                search_query = query_params.get('search', [None])[0]
                level_filter = query_params.get('level', ['all'])[0]
                try:
                    max_messages = int(query_params.get('max', [500])[0])
                    max_messages = min(max_messages, 5000)  # Cap at 5000 for safety
                except ValueError:
                    max_messages = 500

                # Extract log messages
                result = extract_log_messages(log_path, search_query, level_filter, max_messages)
                self.send_json_response(result)

            elif path.startswith('/api/cereal/'):
                # Cereal data endpoint: /api/cereal/{route_base}/{segment}/{log_type}/{message_type}
                parts = path.split('/')[3:]  # Skip '', 'api', 'cereal'
                if len(parts) < 4:
                    self.send_json_response({'error': 'Invalid cereal path. Format: /api/cereal/{route}/{segment}/{qlog|rlog}/{message_type}'}, 400)
                    return

                route_base = parts[0]
                try:
                    segment_num = int(parts[1])
                except ValueError:
                    self.send_json_response({'error': 'Invalid segment number'}, 400)
                    return

                log_type = parts[2].lower()
                if log_type not in ('qlog', 'rlog'):
                    self.send_json_response({'error': 'Invalid log type. Use qlog or rlog'}, 400)
                    return

                message_type = parts[3]

                # Get segment data
                segments = get_route_segments(route_base)
                segment_data = next((s for s in segments if s['segment'] == segment_num), None)

                if not segment_data:
                    self.send_json_response({'error': 'Segment not found'}, 404)
                    return

                # Map log type to filename
                log_files = {
                    'qlog': 'qlog.zst',
                    'rlog': 'rlog.zst'
                }

                log_path = os.path.join(segment_data['path'], log_files[log_type])

                if not os.path.exists(log_path):
                    self.send_json_response({
                        'success': False,
                        'error': f'{log_type} file not found for this segment'
                    }, 404)
                    return

                # Extract cereal messages
                result = extract_cereal_messages(log_path, message_type)
                self.send_json_response(result)

            elif path == '/api/panels':
                # List all available panel configurations
                try:
                    panel_dir = Path(__file__).parent.parent.parent / 'selfdrive' / 'ui' / 'bluepilot' / 'menus'
                    panel_files = sorted(panel_dir.glob('bp_*_panel.json'))

                    # Exclude network panel as requested
                    exclude_panels = ['bp_network_panel.json']

                    # Define Qt settings order (matching selfdrive/ui/bluepilot/qt/offroad/settings.cc)
                    panel_order = [
                        'bp_device_panel',
                        'bp_toggles_panel',
                        'bp_steering_panel',
                        'bp_cruise_panel',
                        'bp_visuals_panel',
                        'bp_display_panel',
                        'bp_vehicle_panel',
                        'bp_developer_panel',
                    ]

                    # Load panel data
                    panel_data_map = {}
                    for panel_file in panel_files:
                        if panel_file.name in exclude_panels:
                            continue

                        try:
                            with open(panel_file, 'r') as f:
                                panel_data = json.load(f)
                                panel_data_map[panel_file.stem] = {
                                    'id': panel_file.stem,  # e.g., 'bp_device_panel'
                                    'name': panel_data.get('menuName', panel_file.stem),
                                    'description': panel_data.get('menuDescription', ''),
                                    'icon': panel_data.get('menuIcon', '')
                                }
                        except Exception as e:
                            logger.warning(f"Failed to load panel {panel_file.name}: {e}")

                    # Sort panels according to Qt order
                    panels = []
                    for panel_id in panel_order:
                        if panel_id in panel_data_map:
                            panels.append(panel_data_map[panel_id])

                    # Add any panels not in the order list (for extensibility)
                    for panel_id, panel_info in panel_data_map.items():
                        if panel_id not in panel_order:
                            panels.append(panel_info)

                    self.send_json_response({
                        'success': True,
                        'panels': panels
                    })
                except Exception as e:
                    logger.error(f"Error listing panels: {e}", exc_info=True)
                    self.send_json_response({'success': False, 'error': str(e)}, 500)

            elif path.startswith('/api/panels/'):
                # Get specific panel configuration
                panel_id = path.split('/api/panels/')[1].strip('/')

                try:
                    panel_dir = Path(__file__).parent.parent.parent / 'selfdrive' / 'ui' / 'bluepilot' / 'menus'
                    panel_file = panel_dir / f'{panel_id}.json'

                    if not panel_file.exists():
                        self.send_json_response({'success': False, 'error': 'Panel not found'}, 404)
                        return

                    with open(panel_file, 'r') as f:
                        panel_data = json.load(f)

                    self.send_json_response({
                        'success': True,
                        'panel': panel_data
                    })
                except Exception as e:
                    logger.error(f"Error loading panel {panel_id}: {e}", exc_info=True)
                    self.send_json_response({'success': False, 'error': str(e)}, 500)

            elif path == '/api/panel-state':
                # Get current device state for panel conditionals
                try:
                    # Helper function to safely get boolean params
                    def safe_get_bool(key, default=False):
                        try:
                            return params.get_bool(key)
                        except Exception:
                            return default

                    # Helper function to safely check if param exists and has value
                    def param_exists(key):
                        try:
                            val = params.get(key)
                            return val is not None and val != b''
                        except Exception:
                            return False

                    onroad = is_onroad()

                    # Basic state
                    state = {
                        'isOnroad': onroad,
                        'isOffroad': not onroad,
                        'hasCarParams': param_exists("CarParams"),
                    }

                    # Parse CarParams if available
                    if param_exists("CarParams"):
                        try:
                            from cereal import car
                            car_params_bytes = params.get("CarParams")
                            car_params = car.CarParams.from_bytes(car_params_bytes)

                            # Add vehicle-specific state
                            state['hasLongitudinalControl'] = car_params.openpilotLongitudinalControl
                            state['hasBlindSpotMonitoring'] = len(car_params.enableBsm) > 0
                            state['isAngleSteering'] = car_params.steerControlType == car.CarParams.SteerControlType.angle
                            state['isPcmCruise'] = car_params.pcmCruise

                            # Brand detection
                            car_name = car_params.carName.lower() if car_params.carName else ''
                            car_fingerprint = car_params.carFingerprint.lower() if car_params.carFingerprint else ''

                            state['brandEquals'] = {}
                            for brand in ['ford', 'hyundai', 'toyota', 'gm', 'honda', 'chrysler', 'mazda', 'nissan', 'subaru', 'volkswagen', 'tesla', 'rivian']:
                                state['brandEquals'][brand] = brand in car_name or brand in car_fingerprint

                            # MADS limited brand check (brands that have limitations with MADS)
                            # Typically Honda, Acura, and some others
                            state['isMadsLimitedBrand'] = 'honda' in car_name or 'acura' in car_name

                            # Check for alpha longitudinal availability
                            state['hasAlphaLongitudinalAvailable'] = not state['hasLongitudinalControl']

                        except Exception as e:
                            logger.warning(f"Error parsing CarParams: {e}")

                    # Parse CarParamsSP if available for ICBM availability
                    icbm_available = False
                    if param_exists("CarParamsSP"):
                        try:
                            from cereal import custom
                            car_params_sp_bytes = params.get("CarParamsSP")
                            car_params_sp = custom.CarParamsSP.from_bytes(car_params_sp_bytes)
                            icbm_available = car_params_sp.intelligentCruiseButtonManagementAvailable
                        except Exception as e:
                            logger.warning(f"Error parsing CarParamsSP: {e}")

                    # Check for specific features
                    # ICBM available = car supports it (from CarParamsSP)
                    state['isICBMAvailable'] = icbm_available
                    # ICBM active = car supports it AND user has enabled it (matches SunnyPilot C++ logic)
                    icbm_user_enabled = safe_get_bool("IntelligentCruiseButtonManagement")
                    state['hasIntelligentCruiseButtonManagement'] = icbm_available and icbm_user_enabled

                    # Branch detection
                    state['isReleaseBranch'] = param_exists("ReleaseNotes")
                    state['isTestedBranch'] = safe_get_bool("IsTestedBranch")
                    state['isDevelopmentBranch'] = not state['isReleaseBranch'] and not state['isTestedBranch']

                    self.send_json_response({
                        'success': True,
                        'state': state
                    })
                except Exception as e:
                    logger.error(f"Error getting panel state: {e}", exc_info=True)
                    self.send_json_response({'success': False, 'error': str(e)}, 500)

            elif path == '/api/params/backup':
                # Get only params with BACKUP flag for backup/restore
                try:
                    # Get all params with their metadata (including attributes)
                    all_params = get_all_params(params)

                    # Filter to only params with BACKUP attribute and extract their values
                    backup_data = {}
                    backup_keys = []

                    for key, param_entry in all_params.items():
                        attributes = param_entry.get('attributes', [])
                        if 'BACKUP' in attributes:
                            backup_keys.append(key)
                            value = param_entry.get('value')

                            # Store value for backup
                            if value is not None:
                                # Check if it's already a dict with _binary flag
                                if isinstance(value, dict) and value.get('_binary'):
                                    backup_data[key] = value
                                elif isinstance(value, (str, int, float, bool)):
                                    backup_data[key] = value
                                else:
                                    # For other types, store as JSON string
                                    backup_data[key] = json.dumps(value)

                    self.send_json_response({
                        'success': True,
                        'params': backup_data,
                        'count': len(backup_data),
                        'backup_params_list': sorted(backup_keys)
                    })

                except Exception as e:
                    logger.error(f"Error getting backup params: {e}", exc_info=True)
                    self.send_json_response({'success': False, 'error': str(e)}, 500)

            elif path == '/api/favorite_settings':
                # Get favorite settings from file
                try:
                    if os.path.exists(FAVORITE_SETTINGS_FILE):
                        with open(FAVORITE_SETTINGS_FILE, 'r') as f:
                            favorites = json.load(f)
                        self.send_json_response({
                            'success': True,
                            'favorites': favorites
                        })
                    else:
                        # Return empty array if file doesn't exist yet
                        self.send_json_response({
                            'success': True,
                            'favorites': []
                        })
                except json.JSONDecodeError as e:
                    logger.error(f"Error parsing favorites file: {e}")
                    self.send_json_response({
                        'success': True,
                        'favorites': []
                    })
                except Exception as e:
                    logger.error(f"Error reading favorites: {e}", exc_info=True)
                    self.send_json_response({'success': False, 'error': str(e)}, 500)

            elif path == '/api/drive-stats':
                # Get aggregate drive statistics from ApiCache_DriveStats param
                # This matches the Qt widget behavior which caches API responses in params
                try:
                    # Try to get cached stats from ApiCache_DriveStats param
                    cached_stats = None
                    try:
                        cached_stats_data = params.get("ApiCache_DriveStats")
                        logger.info(f"ApiCache_DriveStats: type={type(cached_stats_data)}, len={len(cached_stats_data) if cached_stats_data else 0}")

                        # Handle both cases: bytes (fallback Params) and dict (openpilot Params with auto-deserialization)
                        if isinstance(cached_stats_data, dict):
                            # Already deserialized by openpilot Params
                            cached_stats = cached_stats_data
                            logger.info(f"Got pre-deserialized stats: all={cached_stats.get('all', {}).get('routes', 0)} routes")
                        elif cached_stats_data and isinstance(cached_stats_data, bytes):
                            # Raw bytes from fallback Params - need to decode and parse
                            cached_stats_str = cached_stats_data.decode('utf-8').strip()
                            logger.info(f"Decoded stats string: {cached_stats_str[:100]}")
                            if cached_stats_str:
                                cached_stats = json.loads(cached_stats_str)
                                logger.info(f"Successfully parsed cached stats: all={cached_stats.get('all', {}).get('routes', 0)} routes")
                    except Exception as e:
                        logger.error(f"Error reading ApiCache_DriveStats param: {e}", exc_info=True)
                        cached_stats = None

                    if cached_stats:
                        # Parse response (format: {all: {routes, distance, minutes}, week: {...}})
                        # Convert both "all" and "week" stats to frontend format
                        def convert_stats(stats_data):
                            """Convert API stats to frontend format"""
                            distance_miles = stats_data.get('distance', 0)
                            distance_meters = distance_miles * 1609.34
                            duration_minutes = stats_data.get('minutes', 0)
                            duration_seconds = duration_minutes * 60
                            routes = stats_data.get('routes', 0)

                            # Calculate average speed if we have both distance and duration
                            avg_speed_ms = distance_meters / duration_seconds if duration_seconds > 0 else 0

                            return {
                                'routes': routes,
                                'distance': distance_meters,
                                'distanceMiles': distance_miles,  # Keep original for reference
                                'duration': duration_seconds,
                                'durationMinutes': duration_minutes,  # Keep original for reference
                                'averageSpeed': avg_speed_ms,  # m/s
                            }

                        all_stats = convert_stats(cached_stats.get('all', {}))
                        week_stats = convert_stats(cached_stats.get('week', {}))

                        result = {
                            'success': True,
                            'all': all_stats,
                            'week': week_stats,
                            'source': 'param_cache',
                            'timestamp': datetime.now().isoformat()
                        }

                        self.send_json_response(result)
                        return

                    # No cached data available, return zeros
                    logger.debug("No cached drive stats available in ApiCache_DriveStats param")
                    zero_stats = {
                        'routes': 0,
                        'distance': 0,
                        'distanceMiles': 0,
                        'duration': 0,
                        'durationMinutes': 0,
                        'averageSpeed': 0,
                    }
                    self.send_json_response({
                        'success': True,
                        'all': zero_stats,
                        'week': zero_stats.copy(),
                        'source': 'no_cache',
                        'info': 'No cached data available. Stats will populate when Qt UI fetches them.'
                    })

                except Exception as e:
                    logger.error(f"Error reading drive stats from param: {e}", exc_info=True)
                    self.send_json_response({'success': False, 'error': str(e)}, 500)

            elif path == '/api/file-content':
                # Get file content for FileViewer control
                try:
                    query_params = parse_qs(parsed.query)
                    file_path = query_params.get('path', [None])[0]

                    if not file_path:
                        self.send_json_response({'success': False, 'error': 'Missing path parameter'}, 400)
                        return

                    # Security: Only allow reading specific file types from specific directories
                    allowed_extensions = ['.md', '.txt', '.log', '.json', '.yaml', '.yml', '.conf']
                    allowed_dirs = [
                        '/data/openpilot',
                        '/data/logs',
                        '/data/community/crashes',
                    ]

                    # Resolve to absolute path and check if it's within allowed directories
                    if not os.path.isabs(file_path):
                        # If relative path, assume it's relative to /data/openpilot
                        file_path = os.path.join('/data/openpilot', file_path)

                    abs_path = os.path.abspath(file_path)

                    # Check if path is in allowed directories
                    is_allowed = any(abs_path.startswith(allowed_dir) for allowed_dir in allowed_dirs)

                    if not is_allowed:
                        self.send_json_response({
                            'success': False,
                            'error': 'Access denied: File path not in allowed directories'
                        }, 403)
                        return

                    # Check file extension
                    file_ext = os.path.splitext(abs_path)[1].lower()
                    if file_ext and file_ext not in allowed_extensions:
                        self.send_json_response({
                            'success': False,
                            'error': f'Access denied: File type {file_ext} not allowed'
                        }, 403)
                        return

                    # Read file
                    if not os.path.exists(abs_path):
                        self.send_json_response({
                            'success': False,
                            'error': 'File not found'
                        }, 404)
                        return

                    # Limit file size to 1MB for safety
                    file_size = os.path.getsize(abs_path)
                    if file_size > 1024 * 1024:
                        self.send_json_response({
                            'success': False,
                            'error': 'File too large (max 1MB)'
                        }, 413)
                        return

                    try:
                        with open(abs_path, 'r', encoding='utf-8') as f:
                            content = f.read()

                        self.send_json_response({
                            'success': True,
                            'content': content,
                            'path': abs_path,
                            'size': file_size
                        })
                    except UnicodeDecodeError:
                        # Try reading as binary if UTF-8 fails
                        with open(abs_path, 'r', encoding='latin-1') as f:
                            content = f.read()

                        self.send_json_response({
                            'success': True,
                            'content': content,
                            'path': abs_path,
                            'size': file_size,
                            'encoding': 'latin-1'
                        })

                except Exception as e:
                    logger.error(f"Error reading file: {e}", exc_info=True)
                    self.send_json_response({'success': False, 'error': str(e)}, 500)

            elif path.startswith('/api/thumbnail/'):
                route_base = path.split('/api/thumbnail/')[1].strip('/')

                # Try to generate thumbnail if it doesn't exist
                thumbnail_path = generate_thumbnail(route_base)

                if thumbnail_path and os.path.exists(thumbnail_path):
                    self.send_file_response(thumbnail_path, 'image/jpeg')
                else:
                    # Return a placeholder 1x1 transparent PNG instead of 404
                    # This prevents broken image icons in the UI
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/png')
                    self.send_cors_headers()
                    self.end_headers()
                    # 1x1 transparent PNG (67 bytes)
                    self.wfile.write(b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01\x08\x06\x00\x00\x00\x1f\x15\xc4\x89\x00\x00\x00\nIDATx\x9cc\x00\x01\x00\x00\x05\x00\x01\r\n-\xb4\x00\x00\x00\x00IEND\xaeB`\x82')

            else:
                # Serve static files or SPA routes
                file_path = WEBAPP_DIR / path.lstrip('/')
                if file_path.exists() and file_path.is_file():
                    # Serve static file (JS, CSS, images, etc.)
                    self.send_file_response(str(file_path))
                else:
                    # Serve index.html for SPA routes (e.g., /routes, /parameters, /settings)
                    # This allows React Router to handle client-side routing
                    index_path = WEBAPP_DIR / 'index.html'
                    if index_path.exists():
                        self.send_file_response(str(index_path), 'text/html')
                    else:
                        self.send_json_response({'error': 'Web app not found'}, 404)

        except (BrokenPipeError, ConnectionResetError) as e:
            # Client disconnected - don't log as error
            logger.debug(f"Client disconnected during GET {self.path}: {e}")
        except Exception as e:
            logger.error(f"Error handling GET request to {self.path}: {e}", exc_info=True)
            try:
                self.send_json_response({'error': 'Internal server error', 'details': str(e)}, 500)
            except Exception:
                # Connection broken, can't send response
                pass

    def build_route_export_status(self, route_base, camera, segments=None):
        """Build status payload for route export operations"""
        if camera not in CAMERA_FILES:
            return {'error': 'Invalid camera type'}, 400

        if segments is None:
            segments = get_route_segments(route_base)

        if not segments:
            return {'error': 'Route not found'}, 404

        ready, export_path = export_is_up_to_date(route_base, camera, segments)
        key = route_export_key(route_base, camera)

        if ready:
            info = server_state.complete_route_export(key, export_path)
            broadcast_route_export_update(route_base, camera, info, export_path)
        else:
            info = server_state.get_route_export_status(key)
            if info and info.get('status') == 'ready':
                path = info.get('path') or export_path
                if not path or not os.path.exists(path):
                    server_state.clear_route_export(key)
                    info = None

        if info is None:
            thread_active = server_state.get_route_export_thread(key) is not None
            now = time.time()
            info = {
                'status': 'processing' if thread_active else 'idle',
                'message': 'Preparing video' if thread_active else 'Ready to generate video',
                'progress': 0.0,
                'path': export_path if ready else None,
                'started_at': now if thread_active else None,
                'updated_at': now
            }

        payload = format_route_export_status(
            route_base,
            camera,
            info,
            export_path if ready else None
        )
        return payload, 200

    def download_full_route(self, route_base, camera, segments):
        """Download previously generated full route export"""
        try:
            payload, status_code = self.build_route_export_status(route_base, camera, segments)
            if status_code != 200:
                self.send_json_response(payload, status_code)
                return

            if payload.get('status') != 'ready':
                response = dict(payload)
                response['error'] = 'Route export not ready'
                self.send_json_response(response, 409)
                return

            export_path = get_export_output_path(route_base, camera)
            if not os.path.exists(export_path):
                message = 'Generated video file not found'
                logger.error(f"{message}: {export_path}")
                info = server_state.fail_route_export(route_export_key(route_base, camera), message)
                broadcast_route_export_update(route_base, camera, info)
                response = dict(payload)
                response['status'] = 'error'
                response['error'] = message
                self.send_json_response(response, 500)
                return

            filename = generate_route_export_filename(route_base, camera)
            self.send_file_response(export_path, 'video/mp4', download_filename=filename)

        except Exception as e:
            logger.error(f"Error downloading full route {route_base}: {e}", exc_info=True)
            self.send_json_response({
                'error': 'Failed to download route',
                'details': str(e)
            }, 500)

    def do_POST(self):
        """Handle POST requests"""
        try:
            parsed = urlparse(self.path)
            path = parsed.path

            # Rate limiting check (skip for internal endpoints)
            if not self._enforce_rate_limit(path, skip_internal=True):
                return

            # Internal broadcast endpoint (for cross-process communication)
            # No authentication check - only listens on localhost
            if path == '/_internal/broadcast':
                try:
                    content_length = int(self.headers.get('Content-Length', 0))
                    if content_length > 0:
                        body = self.rfile.read(content_length)
                        event_data = json.loads(body.decode('utf-8'))

                        # Broadcast to all connected WebSocket clients
                        broadcaster_instance = server_state.get_broadcaster()
                        if broadcaster_instance:
                            broadcaster_instance.broadcast(event_data.get('type'), event_data.get('data'))

                    try:
                        self.send_json_response({'success': True})
                    except (BrokenPipeError, ConnectionResetError):
                        # Client disconnected before reading response (fire-and-forget request)
                        # This is expected behavior for internal broadcasts, not an error
                        pass
                except Exception as e:
                    logger.exception("Error handling internal broadcast")
                    try:
                        self.send_json_response({
                            'error': 'Broadcast failed',
                            'details': str(e)
                        }, 500)
                    except (BrokenPipeError, ConnectionResetError):
                        pass  # Client already disconnected
                return

            # Check if server should be running
            if not should_server_run():
                self.send_json_response({
                    'error': 'Server disabled',
                    'details': 'Web routes server is currently disabled',
                    'hint': 'Enable server in settings'
                }, 503)
                return

            # Manager log streaming control endpoints (allowed even when onroad)
            if path == '/api/manager-logs/stream/start':
                # Start streaming manager logs via WebSocket
                try:
                    from bluepilot.backend.realtime.log_streamer import get_log_streamer

                    # Get broadcaster from server state (has direct WebSocket access)
                    broadcaster = server_state.get_broadcaster()
                    if not broadcaster:
                        self.send_json_response({
                            'success': False,
                            'error': 'WebSocket broadcaster not available'
                        }, 503)
                        return

                    # Get or create log streamer
                    streamer = get_log_streamer(broadcaster)

                    if streamer.start():
                        self.send_json_response({
                            'success': True,
                            'message': 'Log streaming started'
                        })
                    else:
                        self.send_json_response({
                            'success': False,
                            'error': 'Log streaming already running or failed to start'
                        })
                except Exception as e:
                    logger.exception("Error starting log stream")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)
                return

            elif path == '/api/manager-logs/stream/stop':
                # Stop streaming manager logs
                try:
                    from bluepilot.backend.realtime.log_streamer import get_log_streamer

                    streamer = get_log_streamer()
                    if streamer and streamer.stop():
                        self.send_json_response({
                            'success': True,
                            'message': 'Log streaming stopped'
                        })
                    else:
                        self.send_json_response({
                            'success': False,
                            'error': 'Log streaming not running'
                        })
                except Exception as e:
                    logger.exception("Error stopping log stream")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)
                return

            # Check if onroad for route-related write operations
            # Params/settings can be modified while driving, routes cannot
            if is_onroad():
                # Allow params write operations when onroad
                allowed_onroad_post = [
                    '/api/params',
                    '/api/manager-logs/stream',
                ]
                is_allowed_onroad = any(path.startswith(ep) for ep in allowed_onroad_post)

                if not is_allowed_onroad:
                    self.send_json_response({
                        'error': 'Route operations not allowed while driving',
                        'details': 'Route modifications are disabled for safety while vehicle is in motion',
                        'hint': 'Park the vehicle to modify routes',
                        'reason': 'safety'
                    }, 503)
                    return

            # Cancel export operations
            if path.startswith('/api/route-export/') and path.endswith('/cancel'):
                parts = path.split('/')[3:]
                if len(parts) < 3:
                    self.send_json_response({'error': 'Invalid cancel path'}, 400)
                    return

                route_base = parts[0]
                camera = parts[1]
                key = route_export_key(route_base, camera)

                # Cancel the operation
                server_state.fail_route_export(key, "Cancelled by user")
                server_state.clear_route_export_thread(key)

                # Clean up export file
                export_path = os.path.join(ROUTE_EXPORT_CACHE, f"{route_base}_{camera}.mp4")
                if os.path.exists(export_path):
                    try:
                        os.remove(export_path)
                        logger.info(f"Cleaned up cancelled export: {export_path}")
                    except Exception as e:
                        logger.error(f"Failed to clean up export file: {e}")

                self.send_json_response({'status': 'cancelled', 'message': 'Export cancelled'})
                return

            elif path.startswith('/api/route-export/'):
                parts = path.split('/')[3:]
                if len(parts) < 2:
                    self.send_json_response({'error': 'Invalid route export path'}, 400)
                    return

                route_base = parts[0]
                camera = parts[1]

                segments = get_route_segments(route_base)
                if not segments:
                    self.send_json_response({'error': 'Route not found'}, 404)
                    return

                if camera not in CAMERA_FILES:
                    self.send_json_response({'error': 'Invalid camera type'}, 400)
                    return

                ready, export_path = export_is_up_to_date(route_base, camera, segments)
                key = route_export_key(route_base, camera)

                if ready:
                    payload, _ = self.build_route_export_status(route_base, camera, segments)
                    self.send_json_response(payload)
                    return

                existing_thread = server_state.get_route_export_thread(key)
                if existing_thread:
                    payload, _ = self.build_route_export_status(route_base, camera, segments)
                    self.send_json_response(payload)
                    return

                started = server_state.start_route_export(key, "Preparing route video")
                if not started:
                    payload, _ = self.build_route_export_status(route_base, camera, segments)
                    self.send_json_response(payload)
                    return

                info = server_state.update_route_export(key, progress=0.05, message="Preparing route video")
                broadcast_route_export_update(route_base, camera, info)

                def progress_callback(progress, message):
                    try:
                        progress_value = max(0.0, min(1.0, float(progress)))
                    except (TypeError, ValueError):
                        progress_value = 0.0
                    info = server_state.update_route_export(key, progress=progress_value, message=message)
                    broadcast_route_export_update(route_base, camera, info)

                def worker():
                    try:
                        export_path_local = generate_route_export(route_base, camera, progress_callback, server_state)
                        info = server_state.complete_route_export(key, export_path_local, message="Video ready")
                        broadcast_route_export_update(route_base, camera, info, export_path_local)
                    except Exception as exc:
                        logger.error(f"Route export failed for {route_base}/{camera}: {exc}", exc_info=True)
                        info = server_state.fail_route_export(key, str(exc))
                        broadcast_route_export_update(route_base, camera, info)
                    finally:
                        server_state.clear_route_export_thread(key)

                thread = threading.Thread(
                    target=worker,
                    daemon=True,
                    name=f"route-export-{route_base}-{camera}"
                )
                server_state.set_route_export_thread(key, thread)
                thread.start()

                payload, _ = self.build_route_export_status(route_base, camera)
                self.send_json_response(payload)
                return

            elif path.startswith('/api/preserve/'):
                # Preserve/unpreserve a route using preserve xattr with disk space checking
                route_base = path.split('/api/preserve/')[1].strip('/')

                # Check current preserve status
                currently_preserved = check_route_preserve_status(route_base)

                # Toggle preserve status
                preserve = not currently_preserved

                # Set or remove preserve xattr (includes disk space checks)
                result = set_route_preserve(route_base, preserve)

                if result['success']:
                    is_preserved = preserve

                    # Include disk space info in response
                    disk_info = get_disk_space_info()

                    self.send_json_response({
                        'success': True,
                        'isPreserved': is_preserved,
                        'affected_segments': result.get('affected_segments', 0),
                        'message': result.get('message'),
                        'disk_space': disk_info
                    })

                    # Broadcast WebSocket event
                    event_type = WebSocketEvent.ROUTE_PRESERVED if is_preserved else WebSocketEvent.ROUTE_UNPRESERVED
                    broadcast_websocket_event(event_type, {
                        'route_base': route_base,
                        'is_preserved': is_preserved,
                        'disk_space': disk_info
                    })

                    # Broadcast disk space update (preserved segments changed)
                    broadcast_websocket_event(WebSocketEvent.DISK_UPDATED, {})
                else:
                    # Return error from set_route_preserve
                    self.send_json_response(result, 400 if 'disk space' in result.get('error', '').lower() else 500)

            elif path == '/api/params/set':
                # Set parameter value
                try:
                    content_length = int(self.headers.get('Content-Length', 0))
                    if content_length == 0:
                        self.send_json_response({
                            'success': False,
                            'error': 'Missing request body'
                        }, 400)
                        return

                    body = self.rfile.read(content_length).decode('utf-8')
                    data = json.loads(body)

                    param_key = data.get('key')
                    param_value = data.get('value')

                    if not param_key:
                        self.send_json_response({
                            'success': False,
                            'error': 'Missing required field: key'
                        }, 400)
                        return

                    if param_value is None:
                        self.send_json_response({
                            'success': False,
                            'error': 'Missing required field: value'
                        }, 400)
                        return

                    # Set the param
                    result = set_param_value(param_key, param_value, params)

                    if result['success']:
                        self.send_json_response(result)
                        # Broadcast param change via WebSocket
                        broadcast_websocket_event(WebSocketEvent.PARAM_UPDATED, {
                            'key': param_key,
                            'value': param_value
                        })
                    else:
                        self.send_json_response(result, 400)

                except json.JSONDecodeError:
                    self.send_json_response({
                        'success': False,
                        'error': 'Invalid JSON in request body'
                    }, 400)
                except Exception as e:
                    logger.exception("Error setting param")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path == '/api/params/restore':
                # Restore BACKUP-flagged params from backup data
                try:
                    content_length = int(self.headers.get('Content-Length', 0))
                    if content_length == 0:
                        self.send_json_response({
                            'success': False,
                            'error': 'Missing request body'
                        }, 400)
                        return

                    body = self.rfile.read(content_length).decode('utf-8')
                    data = json.loads(body)

                    params_data = data.get('params', {})

                    if not params_data:
                        self.send_json_response({
                            'success': False,
                            'error': 'No params data provided'
                        }, 400)
                        return

                    # Use centralized attributes cache to get list of valid BACKUP params
                    from bluepilot.backend.params.params_manager import _load_param_attributes_cache

                    attributes_cache = _load_param_attributes_cache()
                    backup_params = {key for key, attrs in attributes_cache.items() if 'BACKUP' in attrs}

                    restored = []
                    failed = []
                    skipped = []

                    for param_key, param_value in params_data.items():
                        # Only restore params that have BACKUP flag
                        if param_key not in backup_params:
                            skipped.append(param_key)
                            logger.warning(f"Skipping {param_key}: not a BACKUP param")
                            continue

                        try:
                            # Handle binary data
                            if isinstance(param_value, dict) and param_value.get('_binary'):
                                import base64
                                value = base64.b64decode(param_value['data'])
                                params.put(param_key, value)
                            else:
                                # String value
                                params.put(param_key, str(param_value))

                            restored.append(param_key)

                            # Broadcast param change
                            broadcast_websocket_event(WebSocketEvent.PARAM_UPDATED, {
                                'key': param_key,
                                'value': param_value
                            })

                        except Exception as e:
                            logger.error(f"Failed to restore param {param_key}: {e}")
                            failed.append({'param': param_key, 'error': str(e)})

                    self.send_json_response({
                        'success': len(failed) == 0,
                        'restored': restored,
                        'failed': failed,
                        'skipped': skipped,
                        'count': len(restored)
                    })

                except json.JSONDecodeError:
                    self.send_json_response({
                        'success': False,
                        'error': 'Invalid JSON in request body'
                    }, 400)
                except Exception as e:
                    logger.exception("Error restoring params")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path == '/api/favorite_settings':
                # Save favorite settings to file
                try:
                    content_length = int(self.headers.get('Content-Length', 0))
                    if content_length == 0:
                        self.send_json_response({
                            'success': False,
                            'error': 'Missing request body'
                        }, 400)
                        return

                    body = self.rfile.read(content_length).decode('utf-8')
                    data = json.loads(body)

                    favorites = data.get('favorites')
                    if favorites is None:
                        self.send_json_response({
                            'success': False,
                            'error': 'Missing required field: favorites'
                        }, 400)
                        return

                    # Ensure parent directory exists
                    os.makedirs(os.path.dirname(FAVORITE_SETTINGS_FILE), exist_ok=True)

                    # Write favorites to file
                    with open(FAVORITE_SETTINGS_FILE, 'w') as f:
                        json.dump(favorites, f, indent=2)

                    logger.info(f"Saved {len(favorites)} favorites to {FAVORITE_SETTINGS_FILE}")
                    self.send_json_response({
                        'success': True,
                        'count': len(favorites)
                    })

                except json.JSONDecodeError:
                    self.send_json_response({
                        'success': False,
                        'error': 'Invalid JSON in request body'
                    }, 400)
                except Exception as e:
                    logger.exception("Error saving favorites")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path == '/api/panel-command':
                # Execute panel command (reboot, shutdown, set param, etc.)
                try:
                    content_length = int(self.headers.get('Content-Length', 0))
                    if content_length == 0:
                        self.send_json_response({
                            'success': False,
                            'error': 'Missing request body'
                        }, 400)
                        return

                    body = self.rfile.read(content_length).decode('utf-8')
                    data = json.loads(body)

                    action = data.get('action')
                    param = data.get('param')
                    value = data.get('value')
                    params_list = data.get('params', [])

                    if not action:
                        self.send_json_response({
                            'success': False,
                            'error': 'Missing required field: action'
                        }, 400)
                        return

                    # Handle different panel actions
                    if action == 'set_param':
                        # Set a parameter
                        if not param:
                            self.send_json_response({
                                'success': False,
                                'error': 'Missing required field: param'
                            }, 400)
                            return

                        if value is None:
                            self.send_json_response({
                                'success': False,
                                'error': 'Missing required field: value'
                            }, 400)
                            return

                        result = set_param_value(param, value, params)
                        if result['success']:
                            self.send_json_response(result)
                            broadcast_websocket_event(WebSocketEvent.PARAM_UPDATED, {
                                'key': param,
                                'value': value
                            })
                        else:
                            self.send_json_response(result, 400)

                    elif action == 'remove_params':
                        # Remove multiple parameters
                        if not params_list:
                            self.send_json_response({
                                'success': False,
                                'error': 'Missing required field: params (list)'
                            }, 400)
                            return

                        removed = []
                        failed = []
                        for param_key in params_list:
                            try:
                                params.remove(param_key)
                                removed.append(param_key)
                                broadcast_websocket_event(WebSocketEvent.PARAM_UPDATED, {
                                    'key': param_key,
                                    'value': None,
                                    'removed': True
                                })
                            except Exception as e:
                                logger.warning(f"Failed to remove param {param_key}: {e}")
                                failed.append(param_key)

                        self.send_json_response({
                            'success': len(failed) == 0,
                            'removed': removed,
                            'failed': failed
                        })

                    elif action == 'reset_settings':
                        # Reset all sunnypilot/bluepilot settings to default
                        # This is a placeholder - actual implementation would need to define default values
                        self.send_json_response({
                            'success': False,
                            'error': 'Reset settings not yet implemented in web UI',
                            'hint': 'Use the device settings panel for this operation'
                        }, 501)

                    elif action == 'restart_ui':
                        success, message = restart_ui_process()
                        if success:
                            self.send_json_response({
                                'success': True,
                                'message': message
                            })
                            broadcast_websocket_event(WebSocketEvent.STATUS_CHANGED, {
                                'status': 'ui_restart_requested'
                            })
                        else:
                            self.send_json_response({
                                'success': False,
                                'error': message
                            }, 500)

                    elif action == 'manage_ssh_keys':
                        # Manage SSH keys (add/remove GitHub SSH keys)
                        username = data.get('username')
                        remove = data.get('remove', False)

                        if remove:
                            # Remove SSH keys
                            params.remove('GithubUsername')
                            params.remove('GithubSshKeys')
                            self.send_json_response({
                                'success': True,
                                'message': 'SSH keys have been removed'
                            })
                        elif username:
                            # Fetch and add SSH keys from GitHub
                            try:
                                response = requests.get(f'https://github.com/{username}.keys', timeout=10)
                                if response.status_code == 200:
                                    keys = response.text.strip()
                                    if keys:
                                        params.put('GithubUsername', username)
                                        params.put('GithubSshKeys', keys)
                                        self.send_json_response({
                                            'success': True,
                                            'message': f'SSH keys have been added for user: {username}'
                                        })
                                    else:
                                        self.send_json_response({
                                            'success': False,
                                            'error': f"Username '{username}' has no keys on GitHub"
                                        }, 400)
                                else:
                                    self.send_json_response({
                                        'success': False,
                                        'error': f"Username '{username}' doesn't exist on GitHub"
                                    }, 404)
                            except requests.Timeout:
                                self.send_json_response({
                                    'success': False,
                                    'error': 'Request timed out'
                                }, 408)
                            except Exception as e:
                                logger.exception(f"Error fetching SSH keys for {username}")
                                self.send_json_response({
                                    'success': False,
                                    'error': f'Failed to fetch SSH keys: {str(e)}'
                                }, 500)
                        else:
                            # Return current state
                            current_username = params.get('GithubUsername', encoding='utf-8')
                            current_keys = params.get('GithubSshKeys', encoding='utf-8')
                            has_keys = bool(current_keys)

                            self.send_json_response({
                                'success': True,
                                'has_keys': has_keys,
                                'username': current_username if has_keys else None
                            })

                    elif action == 'set_copyparty_password':
                        # Set Copyparty password
                        password = data.get('password', '')

                        if password is None:
                            self.send_json_response({
                                'success': False,
                                'error': 'Missing password field'
                            }, 400)
                            return

                        # Empty string means remove password
                        if password == '':
                            params.remove('CopypartyPassword')
                        else:
                            params.put('CopypartyPassword', password)

                        self.send_json_response({
                            'success': True,
                            'message': 'Password saved. Reboot required to apply changes.',
                            'requires_reboot': True
                        })

                    elif action == 'view_error_log':
                        # View error log
                        error_log_path = '/data/community/crashes/error.log'

                        try:
                            if os.path.exists(error_log_path):
                                with open(error_log_path, 'r') as f:
                                    content = f.read()

                                # Get file modification time
                                mtime = os.path.getmtime(error_log_path)
                                modified_date = datetime.fromtimestamp(mtime).strftime('%d-%b-%Y %I:%M:%S %p').upper()

                                self.send_json_response({
                                    'success': True,
                                    'content': content,
                                    'modified': modified_date
                                })
                            else:
                                self.send_json_response({
                                    'success': True,
                                    'content': '',
                                    'modified': None,
                                    'message': 'No error log found'
                                })
                        except Exception as e:
                            logger.exception("Error reading error log")
                            self.send_json_response({
                                'success': False,
                                'error': f'Failed to read error log: {str(e)}'
                            }, 500)

                    else:
                        # Unsupported actions that require Qt UI
                        qt_only_actions = [
                            'show_driver_camera', 'show_training_guide',
                            'show_language_selector', 'show_regulatory',
                            'search_platform', 'remove_platform'
                        ]

                        if action in qt_only_actions:
                            self.send_json_response({
                                'success': False,
                                'error': f'Action "{action}" requires device UI',
                                'hint': 'Please use the settings panel on your Comma device for this feature'
                            }, 501)
                        else:
                            self.send_json_response({
                                'success': False,
                                'error': f'Unknown action: {action}'
                            }, 400)

                except json.JSONDecodeError:
                    self.send_json_response({
                        'success': False,
                        'error': 'Invalid JSON in request body'
                    }, 400)
                except Exception as e:
                    logger.exception("Error executing panel command")
                    self.send_json_response({
                        'success': False,
                        'error': str(e)
                    }, 500)

            elif path == '/api/clear-cache':
                # Clear all cached data (remuxed videos, thumbnails, GPS metrics, drive stats, fingerprints)
                import shutil

                cleared = {
                    'remux_cache': 0,
                    'thumbnails': 0,
                    'gps_metrics': 0,
                    'gps_coordinates': 0,
                    'drive_stats': 0,
                    'fingerprints': 0
                }

                # Clear remuxed video cache
                if os.path.exists(REMUX_CACHE):
                    for filename in os.listdir(REMUX_CACHE):
                        if filename.endswith('.mp4'):
                            try:
                                os.remove(os.path.join(REMUX_CACHE, filename))
                                cleared['remux_cache'] += 1
                            except Exception as e:
                                logger.warning(f"Error deleting remux cache file {filename}: {e}")

                # Clear thumbnail cache
                if os.path.exists(THUMBNAIL_CACHE):
                    for filename in os.listdir(THUMBNAIL_CACHE):
                        if filename.endswith('.jpg'):
                            try:
                                os.remove(os.path.join(THUMBNAIL_CACHE, filename))
                                cleared['thumbnails'] += 1
                            except Exception as e:
                                logger.warning(f"Error deleting thumbnail {filename}: {e}")

                # Clear GPS metrics cache
                if os.path.exists(METRICS_CACHE):
                    for filename in os.listdir(METRICS_CACHE):
                        filepath = os.path.join(METRICS_CACHE, filename)
                        try:
                            if filename.endswith('_coords.json'):
                                os.remove(filepath)
                                cleared['gps_coordinates'] += 1
                            elif filename.endswith('.json') and filename not in ('geocoding_cache.json', 'preprocessing_state.json'):
                                os.remove(filepath)
                                cleared['gps_metrics'] += 1
                        except Exception as e:
                            logger.warning(f"Error deleting metrics cache file {filename}: {e}")

                # Clear drive stats cache
                from bluepilot.backend.routes.processing import DRIVE_STATS_CACHE
                if os.path.exists(DRIVE_STATS_CACHE):
                    for filename in os.listdir(DRIVE_STATS_CACHE):
                        if filename.endswith('.json'):
                            try:
                                os.remove(os.path.join(DRIVE_STATS_CACHE, filename))
                                cleared['drive_stats'] += 1
                            except Exception as e:
                                logger.warning(f"Error deleting drive stats cache file {filename}: {e}")

                # Clear fingerprint cache
                from bluepilot.backend.routes.processing import FINGERPRINT_CACHE
                if os.path.exists(FINGERPRINT_CACHE):
                    for filename in os.listdir(FINGERPRINT_CACHE):
                        if filename.endswith('.json'):
                            try:
                                os.remove(os.path.join(FINGERPRINT_CACHE, filename))
                                cleared['fingerprints'] += 1
                            except Exception as e:
                                logger.warning(f"Error deleting fingerprint cache file {filename}: {e}")

                # Clear preprocessor state file so routes get reprocessed
                preprocessing_state_file = os.path.join(METRICS_CACHE, 'preprocessing_state.json')
                if os.path.exists(preprocessing_state_file):
                    try:
                        os.remove(preprocessing_state_file)
                        logger.info("Cleared preprocessing state - routes will be reprocessed")
                    except Exception as e:
                        logger.warning(f"Error clearing preprocessing state: {e}")

                # Clear in-memory route cache
                get_route_segments.cache_clear()
                clear_deletion_data_cache()

                logger.info(f"Cache cleared: {cleared}")

                self.send_json_response({
                    'success': True,
                    'cleared': cleared
                })

                # Broadcast WebSocket event
                broadcast_websocket_event(WebSocketEvent.CACHE_CLEARED, {
                    'cleared': cleared
                })
            else:
                self.send_json_response({'error': 'Not found'}, 404)

        except (BrokenPipeError, ConnectionResetError) as e:
            logger.debug(f"Client disconnected during POST {self.path}: {e}")
        except Exception as e:
            logger.error(f"Error handling POST request to {self.path}: {e}", exc_info=True)
            try:
                self.send_json_response({'error': 'Internal server error', 'details': str(e)}, 500)
            except Exception:
                pass

    def do_DELETE(self):
        """Handle DELETE requests"""
        try:
            parsed = urlparse(self.path)
            path = parsed.path

            # Rate limiting check
            if not self._enforce_rate_limit(path):
                return

            # Check if server should be running
            if not should_server_run():
                self.send_json_response({
                    'error': 'Server disabled',
                    'details': 'Web routes server is currently disabled',
                    'hint': 'Enable server in settings'
                }, 503)
                return

            if is_onroad():
                self.send_json_response({
                    'error': 'Operation not allowed while driving',
                    'details': 'Delete operations are disabled for safety while vehicle is in motion',
                    'hint': 'Park the vehicle to delete routes',
                    'reason': 'safety'
                }, 503)
                return

            if path.startswith('/api/delete/'):
                import shutil
                route_base = path.split('/api/delete/')[1].strip('/')
                segments = get_route_segments(route_base)

                if not segments:
                    self.send_json_response({'error': 'Route not found'}, 404)
                    return

                # Delete all segments
                for seg in segments:
                    if os.path.exists(seg['path']):
                        shutil.rmtree(seg['path'])

                # Delete thumbnail
                cache_path = os.path.join(THUMBNAIL_CACHE, f"{route_base}.jpg")
                if os.path.exists(cache_path):
                    os.remove(cache_path)

                # Clear cache
                get_route_segments.cache_clear()
                clear_deletion_data_cache()

                self.send_json_response({
                    'success': True,
                    'deleted': len(segments)
                })

                # Broadcast WebSocket event
                broadcast_websocket_event(WebSocketEvent.ROUTE_DELETED, {
                    'route_base': route_base,
                    'deleted_segments': len(segments)
                })

                # Broadcast disk space update
                broadcast_websocket_event(WebSocketEvent.DISK_UPDATED, {})
            else:
                self.send_json_response({'error': 'Not found'}, 404)

        except (BrokenPipeError, ConnectionResetError) as e:
            logger.debug(f"Client disconnected during DELETE {self.path}: {e}")
        except Exception as e:
            logger.error(f"Error handling DELETE request to {self.path}: {e}", exc_info=True)
            try:
                self.send_json_response({'error': 'Internal server error', 'details': str(e)}, 500)
            except Exception:
                pass


def ensure_dependencies():
    """Delegate dependency installation to lifecycle helpers to avoid drift."""
    global WEBSOCKETS_AVAILABLE
    restart_needed = lifecycle.ensure_dependencies()
    # Refresh availability flag based on the unified lifecycle check
    WEBSOCKETS_AVAILABLE = lifecycle.check_dependencies()
    return restart_needed


def cleanup_on_shutdown():
    """Critical cleanup on server shutdown - thread-safe"""
    logger.info("Server shutting down - performing cleanup...")

    # IMPORTANT: Do NOT disable CPU cores on shutdown!
    # The web server shuts down when:
    # 1. Going onroad (cores MUST stay enabled for openpilot processes)
    # 2. User disabled the server (cores should stay as-is)
    # 3. Device shutdown (doesn't matter, system is shutting down anyway)

    logger.info("Leaving CPU cores in current state (not disabling on shutdown)")

    # Kill any remaining FFmpeg processes tracked by server state
    ffmpeg_count = server_state.get_ffmpeg_count()
    if ffmpeg_count > 0:
        logger.warning(f"Killing {ffmpeg_count} remaining FFmpeg processes")
        ffmpeg_processes = server_state.get_ffmpeg_processes()

        # Try to kill specific tracked processes first
        for pid, info in ffmpeg_processes.items():
            try:
                import psutil
                proc = psutil.Process(pid)
                proc.terminate()
                try:
                    proc.wait(timeout=1)
                except psutil.TimeoutExpired:
                    proc.kill()
                logger.info(f"Killed FFmpeg process {pid} ({info['route']})")
            except (psutil.NoSuchProcess, ImportError):
                pass
            except Exception as e:
                logger.debug(f"Error killing process {pid}: {e}")

        # Fallback: pkill any remaining ffmpeg processes (only as last resort)
        # NOTE: This kills ALL ffmpeg processes, which might include user processes
        # Only use this during actual server shutdown, not during regular cleanup
        try:
            logger.warning("Using pkill -9 as last resort - this kills ALL ffmpeg processes")
            subprocess.run(['pkill', '-9', 'ffmpeg'], timeout=2, capture_output=True)
        except Exception as e:
            logger.debug(f"Error running pkill: {e}")

    logger.info("Cleanup completed")


def signal_handler(signum, frame):
    """Handle termination signals gracefully"""
    signal_name = signal.Signals(signum).name
    logger.info(f"Received signal {signal_name} - initiating graceful shutdown")
    cleanup_on_shutdown()
    sys.exit(0)


def main():
    """Start the server"""
    # Add venv site-packages to sys.path so we can import pycapnp and other venv packages
    # This only affects the web server process, not other openpilot processes
    venv_site_packages = "/usr/local/venv/lib/python3.12/site-packages"
    if os.path.exists(venv_site_packages) and venv_site_packages not in sys.path:
        sys.path.insert(0, venv_site_packages)
        logger.info("Added venv site-packages to sys.path for LogReader support")

    try:
        port = int(params.get("BPPortalPort") or "8088")
    except Exception:
        # BPPortalPort may not be registered in params schema, use default
        port = 8088

    logger.info(f"Starting BluePilot Web Routes Server on port {port}")
    logger.info(f"Routes directory: {ROUTES_DIR}")
    logger.info(f"Web app directory: {WEBAPP_DIR}")

    # Register cleanup handlers for graceful shutdown
    atexit.register(cleanup_on_shutdown)
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    logger.info("Registered graceful shutdown handlers")

    # Check if dependencies are available (don't install yet - server starts first)
    deps_available = lifecycle.check_dependencies()
    if deps_available:
        logger.info("All dependencies available - WebSocket features enabled")
    else:
        logger.info("Some dependencies missing - will install in background after server starts")
        logger.info("HTTP API will work immediately, WebSocket features available after restart")

    # Kill any existing server instances first
    kill_existing_process('web_routes_server.py')

    # Enable all CPU cores for better FFmpeg performance
    enable_performance_mode()

    # Check if server should be disabled due to crashes
    # if not lifecycle.check_and_handle_crashes():
    #     logger.error("Server startup aborted due to excessive crashes")
    #     return

    # Start WebSocket server in separate thread if websockets is available
    try:
        import websockets
        websocket_thread = threading.Thread(
            target=start_websocket_server_thread,
            daemon=True,
            name="WebSocketServer"
        )
        websocket_thread.start()
        logger.info(f"WebSocket server thread started on port {WEBSOCKET_PORT}")

        # Wait for WebSocket to be ready (max 2 seconds)
        if server_state.wait_for_websocket(timeout=2.0):
            logger.info("WebSocket event loop ready")
            # Get WebSocket event loop from server state
            ws_loop = server_state.get_websocket_loop()

            # Initialize broadcaster with in-process WebSocket support
            broadcaster_instance = WebSocketBroadcaster(loop=ws_loop, server_state=server_state)
            server_state.set_broadcaster(broadcaster_instance)
            logger.info("WebSocket broadcaster initialized (in-process mode)")
        else:
            logger.warning("WebSocket not ready after 2 seconds, using HTTP fallback")
            broadcaster_instance = WebSocketBroadcaster(http_fallback_port=port)
            server_state.set_broadcaster(broadcaster_instance)
            logger.info("WebSocket broadcaster initialized (HTTP fallback mode)")

    except ImportError:
        logger.info("WebSocket server not available - HTTP polling will be used")
        broadcaster_instance = WebSocketBroadcaster(http_fallback_port=port)
        server_state.set_broadcaster(broadcaster_instance)
        logger.info("WebSocket broadcaster initialized (HTTP fallback mode)")

    except Exception as e:
        logger.error(f"Failed to start WebSocket server: {e}", exc_info=True)
        logger.warning("Continuing without WebSocket support (HTTP polling will still work)")
        broadcaster_instance = WebSocketBroadcaster(http_fallback_port=port)
        server_state.set_broadcaster(broadcaster_instance)
        logger.info("WebSocket broadcaster initialized (HTTP fallback mode)")

    # Initialize params watcher to monitor external param changes
    def on_param_change(key, value):
        """Callback when a param changes externally"""
        try:
            broadcast_websocket_event(WebSocketEvent.PARAM_UPDATED, {
                'key': key,
                'value': value,
                'source': 'external'  # Indicates change came from outside the web interface
            })
            logger.debug(f"Broadcasted external param change: {key} = {value}")
        except Exception as e:
            logger.error(f"Error broadcasting param change: {e}")

    params_watcher = ParamsWatcher(params, broadcast_callback=on_param_change)
    params_watcher.start()
    logger.info("Params watcher started - monitoring for external parameter changes")

    # Register cleanup for params watcher
    def cleanup_params_watcher():
        logger.info("Stopping params watcher...")
        params_watcher.stop()

    atexit.register(cleanup_params_watcher)

    # Determine bind address - always bind to 0.0.0.0 to support WiFi + hotspot
    # Only wlan IPs are advertised to users (cellular IPs are filtered out)
    wifi_ip = get_wifi_ip()
    all_wifi_ips = get_all_wifi_ips()
    try:
        tethering_enabled = params.get_bool("EnableTethering")
    except Exception:
        # EnableTethering may not be registered in params schema
        tethering_enabled = False

    # Always bind to all interfaces to support both WiFi and hotspot seamlessly
    bind_address = '0.0.0.0'

    logger.info("=" * 60)
    logger.info("WEB SERVER NETWORK CONFIGURATION")
    logger.info(f"Binding to: {bind_address} (supports WiFi + hotspot)")

    if all_wifi_ips:
        logger.info(f"WiFi/hotspot interfaces detected: {', '.join([f'{iface}={ip}' for iface, ip in all_wifi_ips])}")
        logger.info(f"Primary WiFi IP: {wifi_ip or 'N/A'}")
    else:
        logger.warning("No WiFi interfaces detected!")

    if tethering_enabled:
        logger.info("Tethering/hotspot: ENABLED")
    else:
        logger.info("Tethering/hotspot: Disabled")

    logger.info("Note: Only WiFi/hotspot IPs are shown to users (cellular filtered)")
    logger.info("=" * 60)

    # Create HTTP server with socket reuse to prevent "Address already in use" errors
    # Kill any existing process using the port first
    try:
        killed = process_manager.kill_port_process(port)
        if killed:
            logger.info(f"Cleared port {port}, waiting 1 second for socket cleanup...")
            time.sleep(1)
    except Exception as e:
        logger.warning(f"Error checking/killing port {port}: {e}")

    # Try to create server, retry once if port is still in use
    max_retries = 2
    for attempt in range(max_retries):
        try:
            server = ReuseAddressHTTPServer((bind_address, port), WebRoutesHandler)
            server.timeout = 30  # Set timeout to prevent hanging connections
            logger.info(f"Successfully bound to {bind_address}:{port}")
            break
        except OSError as e:
            if e.errno == 98:  # Address already in use
                if attempt < max_retries - 1:
                    logger.warning(f"Port {port} still in use, retrying...")
                    process_manager.kill_port_process(port)
                    time.sleep(2)
                    continue
                else:
                    logger.error(f"Port {port} is still in use after retries. Forcing cleanup...")
                    # Last resort: kill all python processes with web_routes_server
                    try:
                        subprocess.run(['pkill', '-9', '-f', 'web_routes_server.py'], timeout=2)
                        time.sleep(2)
                        logger.info("Killed all web_routes_server processes, attempting final bind...")
                        server = ReuseAddressHTTPServer((bind_address, port), WebRoutesHandler)
                        server.timeout = 30
                        logger.info(f"Successfully bound to {bind_address}:{port} after force cleanup")
                        break
                    except Exception as final_error:
                        logger.error(f"Failed to start server even after force cleanup: {final_error}")
                        logger.error("Manual intervention required. Try: sudo reboot")
                        return
            else:
                raise

    # Track previous onroad status for change detection
    last_onroad_status = [None]  # Use list for closure modification

    # Periodic status monitoring (no longer stops server)
    def monitor_status():
        try:
            # Check if we should restore power save mode
            check_and_restore_power_save()

            # Check for onroad status changes and broadcast via WebSocket
            current_onroad = is_onroad()
            if last_onroad_status[0] is not None and current_onroad != last_onroad_status[0]:
                status_str = 'onroad' if current_onroad else 'online'
                broadcast_websocket_event(WebSocketEvent.STATUS_CHANGED, {
                    'status': status_str,
                    'onroad': current_onroad
                })
                logger.info(f"Device status changed to: {status_str}")

            last_onroad_status[0] = current_onroad
        except Exception as e:
            logger.error(f"Error in status monitor: {e}")

    # Start background dependency installation if needed (after server is ready to start)
    # This ensures the server is responsive immediately while packages install in background
    if not deps_available:
        def on_deps_installed(restart_needed):
            if restart_needed:
                logger.info("Dependencies installed - server will restart on next timeout cycle")
            else:
                logger.info("Dependency installation complete - no restart needed")

        lifecycle.install_dependencies_background(on_complete_callback=on_deps_installed)
        logger.info("Server starting immediately - dependency installation running in background")

    # Start HTTP server - runs continuously until disabled or terminated
    try:
        logger.info(f"Web server starting on {bind_address}:{port}")
        logger.info("Server will run continuously (rate-limited when onroad)")

        # Override handle_timeout to monitor status and power save periodically
        original_handle_timeout = server.handle_timeout

        def custom_handle_timeout():
            monitor_status()  # Check onroad status and broadcast changes

            # Check if restart is needed due to dependency installation
            if lifecycle.is_restart_pending():
                logger.info("Restart pending - triggering graceful restart...")
                try:
                    # Shutdown server before restart (best effort)
                    server.shutdown()
                except Exception as e:
                    logger.warning(f"Error during server shutdown: {e}")
                try:
                    lifecycle.trigger_restart()
                except Exception as e:
                    logger.error(f"Failed to trigger restart: {e}")
                    # Server will continue running, retry on next timeout
                return  # Don't call original handler if restarting

            if original_handle_timeout:
                original_handle_timeout()

        server.handle_timeout = custom_handle_timeout

        server.serve_forever()  # Run indefinitely until process killed
    except KeyboardInterrupt:
        logger.info("Server stopped by user")
        server.shutdown()
    except Exception as e:
        logger.error(f"Server error: {e}")
        # Record this error for monitoring but don't stop the server
        # lifecycle.record_crash()
        logger.info("Server continuing despite error...")
        # Don't shutdown or re-raise - keep server running


if __name__ == '__main__':
    main()
