#!/usr/bin/env python3
"""
BluePilot Backend Server State Management

This module provides thread-safe state management for the BluePilot web routes server.
The ServerState class manages:
- WebSocket client connections and broadcasting
- FFmpeg process tracking and monitoring
- Error logging with circular buffer (last 50 errors)
- Route export operation status and progress tracking
- Videos ZIP creation and download status
- Route backup operation management
- Route import operation tracking
- Server uptime monitoring

All methods are thread-safe using RLock for concurrent access from multiple threads.
"""

import threading
import time
from datetime import datetime


class ServerState:
    """Thread-safe container for server state to prevent race conditions"""
    def __init__(self):
        self._lock = threading.RLock()
        self._cellular_access_enabled_time = None
        self._websocket_clients = set()
        self._websocket_loop = None
        self._broadcaster = None
        self._websocket_ready = threading.Event()  # Signal when WebSocket is ready
        self._ffmpeg_processes = {}  # Track by PID: {pid: {'start_time': ..., 'route': ...}}
        self._active_ffmpeg_count = 0
        self._error_log = []  # Circular buffer of recent errors
        self._max_errors = 50  # Keep last 50 errors
        self._server_start_time = time.time()
        self._route_exports = {}
        self._export_threads = {}

    def get_cellular_enabled_time(self):
        with self._lock:
            return self._cellular_access_enabled_time

    def set_cellular_enabled_time(self, value):
        with self._lock:
            self._cellular_access_enabled_time = value

    def add_websocket_client(self, client):
        with self._lock:
            self._websocket_clients.add(client)
            return len(self._websocket_clients)

    def remove_websocket_client(self, client):
        with self._lock:
            self._websocket_clients.discard(client)
            return len(self._websocket_clients)

    def get_websocket_clients(self):
        with self._lock:
            return list(self._websocket_clients)  # Return copy for safe iteration

    def set_websocket_loop(self, loop):
        with self._lock:
            self._websocket_loop = loop
            if loop:
                self._websocket_ready.set()

    def get_websocket_loop(self):
        with self._lock:
            return self._websocket_loop

    def wait_for_websocket(self, timeout=2.0):
        """Wait for WebSocket to be ready, return True if ready"""
        return self._websocket_ready.wait(timeout)

    def set_broadcaster(self, broadcaster):
        with self._lock:
            self._broadcaster = broadcaster

    def get_broadcaster(self):
        with self._lock:
            return self._broadcaster

    def register_ffmpeg_process(self, pid, route_info):
        """Register FFmpeg process with tracking info"""
        with self._lock:
            self._ffmpeg_processes[pid] = {
                'start_time': time.time(),
                'route': route_info
            }
            self._active_ffmpeg_count += 1
            return self._active_ffmpeg_count

    def unregister_ffmpeg_process(self, pid):
        """Unregister FFmpeg process, return remaining count"""
        with self._lock:
            if pid in self._ffmpeg_processes:
                del self._ffmpeg_processes[pid]
                self._active_ffmpeg_count = max(0, self._active_ffmpeg_count - 1)
            return self._active_ffmpeg_count

    def get_ffmpeg_count(self):
        with self._lock:
            return self._active_ffmpeg_count

    def get_ffmpeg_processes(self):
        """Get copy of FFmpeg process info for monitoring"""
        with self._lock:
            return dict(self._ffmpeg_processes)

    def log_error(self, level, message, details=None, exception_info=None):
        """Log an error to the circular buffer for later retrieval"""
        with self._lock:
            error_entry = {
                'timestamp': datetime.now().isoformat(),
                'unix_time': time.time(),
                'level': level,  # 'ERROR', 'WARNING', 'CRITICAL'
                'message': message,
                'details': details,
                'exception': exception_info
            }
            self._error_log.append(error_entry)

            # Keep only last N errors (circular buffer)
            if len(self._error_log) > self._max_errors:
                self._error_log = self._error_log[-self._max_errors:]

    def get_recent_errors(self, limit=None, level=None):
        """Get recent errors, optionally filtered by level"""
        with self._lock:
            errors = list(self._error_log)  # Copy

            # Filter by level if specified
            if level:
                errors = [e for e in errors if e['level'] == level]

            # Limit results
            if limit:
                errors = errors[-limit:]

            return errors

    def get_error_summary(self):
        """Get summary of errors by level"""
        with self._lock:
            summary = {'ERROR': 0, 'WARNING': 0, 'CRITICAL': 0, 'total': len(self._error_log)}
            for error in self._error_log:
                level = error.get('level', 'ERROR')
                summary[level] = summary.get(level, 0) + 1
            return summary

    def get_server_uptime(self):
        """Get server uptime in seconds"""
        return time.time() - self._server_start_time

    def clear_error_log(self):
        """Clear the error log"""
        with self._lock:
            self._error_log = []

    def get_route_export_status(self, key):
        """Get a copy of the export status for a route"""
        with self._lock:
            info = self._route_exports.get(key)
            return dict(info) if info else None

    def start_route_export(self, key, message="Preparing video"):
        """Mark an export as in-progress unless one is already running"""
        with self._lock:
            info = self._route_exports.get(key)
            if info and info.get('status') == 'processing':
                return False

            now = time.time()
            self._route_exports[key] = {
                'status': 'processing',
                'message': message,
                'progress': 0.0,
                'path': None,
                'started_at': now,
                'updated_at': now
            }
            return True

    def update_route_export(self, key, **updates):
        """Update fields on an export status and return a copy"""
        with self._lock:
            info = self._route_exports.setdefault(key, {})
            if 'started_at' not in info:
                info['started_at'] = updates.get('started_at', time.time())
            info.update(updates)
            info['updated_at'] = time.time()
            self._route_exports[key] = info
            return dict(info)

    def complete_route_export(self, key, path, message="Video ready"):
        """Mark an export as ready"""
        return self.update_route_export(
            key,
            status='ready',
            progress=1.0,
            path=path,
            message=message
        )

    def fail_route_export(self, key, message):
        """Mark an export as failed"""
        return self.update_route_export(key, status='error', message=message)

    def clear_route_export(self, key):
        """Remove export status and thread tracking"""
        with self._lock:
            self._route_exports.pop(key, None)
            self._export_threads.pop(key, None)

    def get_route_export_thread(self, key):
        """Return active export thread if running"""
        with self._lock:
            thread = self._export_threads.get(key)
            if thread and not thread.is_alive():
                self._export_threads.pop(key, None)
                return None
            return thread

    def set_route_export_thread(self, key, thread):
        """Register the worker thread for an export"""
        with self._lock:
            self._export_threads[key] = thread

    def clear_route_export_thread(self, key):
        """Remove export thread tracking"""
        with self._lock:
            self._export_threads.pop(key, None)

    # Videos ZIP operations (similar pattern to route export)
    def get_videos_zip_status(self, route_base):
        """Get status of videos ZIP operation"""
        with self._lock:
            return self._route_exports.get(f"videos_zip:{route_base}", {}).copy()

    def start_videos_zip(self, route_base, message="Creating ZIP"):
        """Start videos ZIP operation"""
        key = f"videos_zip:{route_base}"
        with self._lock:
            if key in self._route_exports:
                return False
            self._route_exports[key] = {
                'status': 'processing',
                'progress': 0.0,
                'message': message,
                'started_at': time.time()
            }
            return True

    def update_videos_zip(self, route_base, **updates):
        """Update videos ZIP operation status"""
        key = f"videos_zip:{route_base}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update(updates)
                return self._route_exports[key].copy()
            return {}

    def complete_videos_zip(self, route_base, path, message="ZIP ready"):
        """Mark videos ZIP as complete"""
        key = f"videos_zip:{route_base}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'ready',
                    'progress': 1.0,
                    'message': message,
                    'path': path,
                    'completed_at': time.time()
                })
                return self._route_exports[key].copy()
            return {}

    def fail_videos_zip(self, route_base, message):
        """Mark videos ZIP as failed"""
        key = f"videos_zip:{route_base}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'error',
                    'message': message
                })

    def clear_videos_zip(self, route_base):
        """Clear videos ZIP status"""
        key = f"videos_zip:{route_base}"
        with self._lock:
            self._route_exports.pop(key, None)

    def cancel_videos_zip(self, route_base):
        """Cancel videos ZIP operation and cleanup"""
        key = f"videos_zip:{route_base}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'cancelled',
                    'message': 'Operation cancelled by user'
                })
                zip_path = self._route_exports[key].get('path')
                self._route_exports.pop(key, None)
                return zip_path
            return None

    # Route Backup operations
    def get_backup_status(self, route_base):
        """Get status of route backup operation"""
        with self._lock:
            return self._route_exports.get(f"backup:{route_base}", {}).copy()

    def start_backup(self, route_base, message="Creating backup"):
        """Start backup operation"""
        key = f"backup:{route_base}"
        with self._lock:
            if key in self._route_exports:
                return False
            self._route_exports[key] = {
                'status': 'processing',
                'progress': 0.0,
                'message': message,
                'started_at': time.time()
            }
            return True

    def update_backup(self, route_base, **updates):
        """Update backup operation status"""
        key = f"backup:{route_base}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update(updates)
                return self._route_exports[key].copy()
            return {}

    def complete_backup(self, route_base, path, message="Backup ready"):
        """Mark backup as complete"""
        key = f"backup:{route_base}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'ready',
                    'progress': 1.0,
                    'message': message,
                    'path': path,
                    'completed_at': time.time()
                })
                return self._route_exports[key].copy()
            return {}

    def fail_backup(self, route_base, message):
        """Mark backup as failed"""
        key = f"backup:{route_base}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'error',
                    'message': message
                })

    def clear_backup(self, route_base):
        """Clear backup status"""
        key = f"backup:{route_base}"
        with self._lock:
            self._route_exports.pop(key, None)

    def cancel_backup(self, route_base):
        """Cancel backup operation and cleanup"""
        key = f"backup:{route_base}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'cancelled',
                    'message': 'Operation cancelled by user'
                })
                backup_path = self._route_exports[key].get('path')
                self._route_exports.pop(key, None)
                return backup_path
            return None

    # Route Import operations
    def get_import_status(self, import_id):
        """Get status of route import operation"""
        with self._lock:
            return self._route_exports.get(f"import:{import_id}", {}).copy()

    def start_import(self, import_id, message="Importing route"):
        """Start import operation"""
        key = f"import:{import_id}"
        with self._lock:
            if key in self._route_exports:
                return False
            self._route_exports[key] = {
                'status': 'processing',
                'progress': 0.0,
                'message': message,
                'started_at': time.time()
            }
            return True

    def update_import(self, import_id, **updates):
        """Update import operation status"""
        key = f"import:{import_id}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update(updates)
                return self._route_exports[key].copy()
            return {}

    def complete_import(self, import_id, route_name, message="Import complete"):
        """Mark import as complete"""
        key = f"import:{import_id}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'completed',
                    'progress': 1.0,
                    'message': message,
                    'routeName': route_name,
                    'completed_at': time.time()
                })
                return self._route_exports[key].copy()
            return {}

    def fail_import(self, import_id, message):
        """Mark import as failed"""
        key = f"import:{import_id}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'error',
                    'message': message
                })

    def clear_import(self, import_id):
        """Clear import status"""
        key = f"import:{import_id}"
        with self._lock:
            self._route_exports.pop(key, None)

    # Log Download operations (qlog/rlog)
    def get_log_download_status(self, route_base, log_type):
        """Get status of log download operation"""
        key = f"log_download:{route_base}:{log_type}"
        with self._lock:
            return self._route_exports.get(key, {}).copy()

    def start_log_download(self, route_base, log_type, total_files, message="Preparing download"):
        """Start log download operation"""
        key = f"log_download:{route_base}:{log_type}"
        with self._lock:
            self._route_exports[key] = {
                'status': 'processing',
                'progress': 0.0,
                'message': message,
                'total_files': total_files,
                'files_processed': 0,
                'started_at': time.time()
            }
            return True

    def update_log_download(self, route_base, log_type, **updates):
        """Update log download operation status"""
        key = f"log_download:{route_base}:{log_type}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update(updates)
                self._route_exports[key]['updated_at'] = time.time()
                return self._route_exports[key].copy()
            return {}

    def complete_log_download(self, route_base, log_type, message="Download ready"):
        """Mark log download as complete"""
        key = f"log_download:{route_base}:{log_type}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'ready',
                    'progress': 1.0,
                    'message': message,
                    'completed_at': time.time()
                })
                return self._route_exports[key].copy()
            return {}

    def fail_log_download(self, route_base, log_type, message):
        """Mark log download as failed"""
        key = f"log_download:{route_base}:{log_type}"
        with self._lock:
            if key in self._route_exports:
                self._route_exports[key].update({
                    'status': 'error',
                    'message': message
                })

    def clear_log_download(self, route_base, log_type):
        """Clear log download status"""
        key = f"log_download:{route_base}:{log_type}"
        with self._lock:
            self._route_exports.pop(key, None)
