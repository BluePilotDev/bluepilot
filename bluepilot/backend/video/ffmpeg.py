#!/usr/bin/env python3
"""
BluePilot Backend FFmpeg Process Management
FFmpeg process lifecycle and log streaming
"""

import subprocess
import threading
import logging

logger = logging.getLogger(__name__)


def stream_ffmpeg_logs(process, route_info, broadcaster):
    """
    Stream FFmpeg stderr output to WebSocket clients in real-time.
    Runs in a separate thread to avoid blocking.

    Args:
        process: subprocess.Popen instance
        route_info: Route identifier string
        broadcaster: WebSocketBroadcaster instance
    """
    if not broadcaster or not process or not process.stderr:
        return

    try:
        # Broadcast process start
        broadcaster.broadcast_ffmpeg_log(
            route_info=route_info,
            log_type='start',
            message=f'FFmpeg process started (PID: {process.pid})',
            pid=process.pid
        )

        # Read stderr line by line and broadcast
        for line in iter(process.stderr.readline, b''):
            if not line:
                break

            try:
                decoded_line = line.decode('utf-8', errors='replace').strip()
                if decoded_line:
                    broadcaster.broadcast_ffmpeg_log(
                        route_info=route_info,
                        log_type='stderr',
                        message=decoded_line,
                        pid=process.pid
                    )
            except Exception as e:
                logger.debug(f"Error broadcasting FFmpeg log line: {e}")

    except Exception as e:
        logger.debug(f"FFmpeg log streaming thread error: {e}")
    finally:
        # Broadcast process end
        if broadcaster:
            try:
                broadcaster.broadcast_ffmpeg_log(
                    route_info=route_info,
                    log_type='end',
                    message=f'FFmpeg process ended (PID: {process.pid})',
                    pid=process.pid
                )
            except Exception as e:
                logger.debug(f"Error broadcasting FFmpeg end: {e}")


class FFmpegProcess:
    """Context manager for FFmpeg processes with guaranteed cleanup"""

    def __init__(self, route_info, server_state, max_concurrent=3, stream_logs=False):
        """
        Initialize FFmpeg process manager

        Args:
            route_info: Route identifier string
            server_state: ServerState instance for process tracking
            max_concurrent: Maximum concurrent FFmpeg processes
            stream_logs: Whether to stream logs to WebSocket
        """
        self.route_info = route_info
        self.server_state = server_state
        self.max_concurrent = max_concurrent
        self.stream_logs = stream_logs
        self.process = None
        self.pid = None
        self.log_thread = None

    def __enter__(self):
        # Check if we can start another process
        current_count = self.server_state.get_ffmpeg_count()
        if current_count >= self.max_concurrent:
            raise RuntimeError(
                f"Too many FFmpeg processes ({current_count}/{self.max_concurrent}). "
                "Please wait and try again."
            )
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Guaranteed cleanup - always runs"""
        if self.process:
            try:
                # Try graceful termination first
                if self.process.poll() is None:
                    self.process.terminate()
                    try:
                        self.process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        # Force kill if graceful termination fails
                        logger.warning(f"FFmpeg process {self.pid} did not terminate gracefully, force killing")
                        self.process.kill()
                        self.process.wait()
            except Exception as e:
                logger.error(f"Error cleaning up FFmpeg process {self.pid}: {e}")
            finally:
                # Wait for log streaming thread to finish
                if self.log_thread and self.log_thread.is_alive():
                    try:
                        self.log_thread.join(timeout=1)
                    except Exception:
                        pass

                # Always unregister
                if self.pid:
                    remaining = self.server_state.unregister_ffmpeg_process(self.pid)
                    logger.info(f"FFmpeg process {self.pid} cleaned up. Remaining: {remaining}")
        return False  # Don't suppress exceptions

    def start(self, cmd, debug_mode=False):
        """
        Start FFmpeg process and register it

        Args:
            cmd: FFmpeg command list
            debug_mode: If True, use verbose logging and stream logs to websocket
        """
        try:
            # Modify command for debug mode
            if debug_mode:
                # Replace -loglevel error with verbose logging for debugging
                modified_cmd = []
                skip_next = False
                for i, arg in enumerate(cmd):
                    if skip_next:
                        skip_next = False
                        continue
                    if arg == '-loglevel':
                        modified_cmd.extend(['-loglevel', 'info'])  # More verbose
                        skip_next = True
                    else:
                        modified_cmd.append(arg)
                cmd = modified_cmd if modified_cmd else cmd

            # Start process with stderr redirected to prevent buffer deadlock
            # FFmpeg outputs progress/warnings to stderr which can fill the pipe buffer
            # causing the process to hang. We only read stderr after completion.
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=65536  # Larger buffer to prevent deadlock during streaming
            )
            self.pid = self.process.pid

            # Register with server state
            count = self.server_state.register_ffmpeg_process(self.pid, self.route_info)
            logger.info(f"Started FFmpeg process {self.pid} for {self.route_info}. Active: {count}")

            # Start log streaming thread if enabled
            if self.stream_logs or debug_mode:
                broadcaster = self.server_state.get_broadcaster()
                if broadcaster:
                    self.log_thread = threading.Thread(
                        target=stream_ffmpeg_logs,
                        args=(self.process, self.route_info, broadcaster),
                        daemon=True
                    )
                    self.log_thread.start()
                    logger.info(f"Started FFmpeg log streaming for {self.route_info}")

            return self.process

        except Exception as e:
            logger.error(f"Failed to start FFmpeg: {e}")
            raise
