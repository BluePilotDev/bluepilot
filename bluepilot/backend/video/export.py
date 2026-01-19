#!/usr/bin/env python3
"""
BluePilot Backend Video Export
Route video export and streaming functionality
"""

import os
import re
import json
import subprocess
import tempfile
import logging
from datetime import datetime, timezone
from functools import lru_cache

from bluepilot.backend.config import (
    CAMERA_FILES,
    CAMERA_LABELS,
    HEVC_CAMERAS,
    ROUTE_EXPORT_CACHE,
    METRICS_CACHE,
    ROUTES_DIR,
    FFMPEG_BINARY,
    MAX_CONCURRENT_FFMPEG,
    FFMPEG_RESERVED_FOR_PLAYBACK,
)
from bluepilot.backend.video.ffmpeg import FFmpegProcess
from bluepilot.backend.realtime import WebSocketEvent
from bluepilot.backend.utils.power import enable_performance_mode
from bluepilot.backend.utils.file_ops import (
    get_free_disk_space,
    has_sufficient_disk_space,
)

logger = logging.getLogger(__name__)


# ============================================================================
# Export Key and Path Management
# ============================================================================

def route_export_key(route_base, camera):
    """Create a stable key for tracking export status"""
    return f"{route_base}:{camera}"


def get_export_output_path(route_base, camera):
    """Return the filesystem path for a generated route export"""
    safe_camera = camera.replace('/', '_')
    return os.path.join(ROUTE_EXPORT_CACHE, f"{route_base}_{safe_camera}.mp4")


def _sanitize_filename_component(component):
    """Sanitize string for safe filename usage"""
    if not component:
        return None
    value = re.sub(r'[^A-Za-z0-9]+', '-', str(component)).strip('-')
    return value or None


# ============================================================================
# Export Filename Generation
# ============================================================================

def generate_route_export_filename(route_base, camera, segments=None):
    """
    Create a descriptive filename for exported route videos.

    Format example: 10032024_143000_Ypsilanti-MI_front.mp4
    """
    components = []

    # Use parsed route datetime when available
    route_dt = parse_route_datetime(route_base)
    if route_dt:
        # Format: MMDDYYYY_HHMMSS
        components.append(route_dt.strftime("%m%d%Y_%H%M%S"))

    # Attempt to include location from cached metrics
    start_location = None
    end_location = None
    metrics_file = os.path.join(METRICS_CACHE, f"{route_base}.json")
    try:
        if os.path.exists(metrics_file):
            with open(metrics_file) as f:
                metrics = json.load(f)
                start_location = metrics.get('start_location')
                end_location = metrics.get('end_location')
    except Exception as e:
        logger.debug(f"Unable to load metrics for {route_base}: {e}")

    location_component = None
    if start_location:
        if end_location and end_location != start_location:
            location_component = f"{start_location} to {end_location}"
        else:
            location_component = start_location
    if location_component:
        sanitized = _sanitize_filename_component(location_component)
        if sanitized:
            components.append(sanitized)

    # Append camera label at the end
    components.append(_sanitize_filename_component(CAMERA_LABELS.get(camera, camera)) or camera)

    # Fallback to route_base_camera if no datetime available
    if not components or len(components) == 1:
        return f"{route_base}_{camera}.mp4"

    filename_base = "_".join(filter(None, components))
    return f"{filename_base}.mp4"


# ============================================================================
# Export Cache Validation
# ============================================================================

def export_is_up_to_date(route_base, camera, segments):
    """Check if the cached export is newer than all source segments"""
    export_path = get_export_output_path(route_base, camera)
    if not os.path.exists(export_path):
        return False, export_path

    export_mtime = os.path.getmtime(export_path)
    camera_file = CAMERA_FILES.get(camera)

    if not camera_file:
        return False, export_path

    for segment in segments:
        source_path = os.path.join(segment['path'], camera_file)
        if not os.path.exists(source_path):
            return False, export_path
        if os.path.getmtime(source_path) > export_mtime:
            return False, export_path

    return True, export_path


# ============================================================================
# Export Status Formatting
# ============================================================================

def format_route_export_status(route_base, camera, info, export_path=None):
    """Format route export status for API and WebSocket payloads"""
    def to_iso(ts):
        if not ts:
            return None
        try:
            return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat()
        except (TypeError, ValueError, OSError):
            return None

    status = info.get('status', 'idle')
    message = info.get('message')
    progress = info.get('progress', 0.0) or 0.0
    try:
        progress = float(progress)
    except (TypeError, ValueError):
        progress = 0.0
    progress = max(0.0, min(1.0, progress))

    export_path = info.get('path') or export_path

    suggested_filename = generate_route_export_filename(route_base, camera)

    payload = {
        'success': True,
        'route': route_base,
        'camera': camera,
        'status': status,
        'message': message,
        'progress': round(progress, 3),
        'progressPercent': int(progress * 100),
        'startedAt': to_iso(info.get('started_at')),
        'updatedAt': to_iso(info.get('updated_at')),
        'downloadUrl': None,
        'filename': None,
        'suggestedFilename': suggested_filename,
        'filenameOnDisk': None
    }

    if status == 'ready' and export_path and os.path.exists(export_path):
        payload['downloadUrl'] = f"/api/download/route/{route_base}/{camera}"
        payload['filenameOnDisk'] = os.path.basename(export_path)
        payload['filename'] = suggested_filename

    if status == 'error':
        payload['error'] = message or 'Video generation failed'

    return payload


# ============================================================================
# WebSocket Broadcasting
# ============================================================================

def broadcast_route_export_update(route_base, camera, info, export_path=None, server_state=None):
    """Broadcast route export status updates via WebSocket"""
    payload = format_route_export_status(route_base, camera, info, export_path)

    if server_state:
        broadcaster = server_state.get_broadcaster()
        if broadcaster:
            try:
                broadcaster.broadcast(WebSocketEvent.ROUTE_EXPORT_UPDATE, payload)
            except Exception as e:
                logger.error(f"Error broadcasting WebSocket event: {e}")


# ============================================================================
# Export Generation
# ============================================================================

def generate_route_export(route_base, camera, progress_callback=None, server_state=None):
    """
    Generate or refresh a full-route export for the given camera.

    Args:
        route_base: Route identifier
        camera: Camera type
        progress_callback: Optional callable accepting (progress: float, message: str)
        server_state: Server state instance for FFmpeg tracking

    Returns:
        Path to the generated export file

    Raises:
        RuntimeError, FileNotFoundError, ValueError on failure
    """
    if camera not in CAMERA_FILES:
        raise ValueError("Invalid camera type")

    if FFMPEG_BINARY is None:
        raise RuntimeError("FFmpeg is not available on this device")

    segments = get_route_segments(route_base)
    if not segments:
        raise FileNotFoundError("Route not found")

    camera_file = CAMERA_FILES[camera]

    video_paths = []
    missing_segments = []
    for seg in sorted(segments, key=lambda s: s['segment']):
        source_path = os.path.join(seg['path'], camera_file)
        if os.path.exists(source_path):
            video_paths.append((seg['segment'], source_path))
        else:
            missing_segments.append(seg['segment'])

    if missing_segments:
        missing_str = ', '.join(f"{seg:03d}" for seg in missing_segments[:10])
        raise FileNotFoundError(f"Missing {camera} video for segments: {missing_str}")

    if not video_paths:
        raise FileNotFoundError(f"No {camera} video segments available")

    up_to_date, export_path = export_is_up_to_date(route_base, camera, segments)
    if up_to_date:
        if progress_callback:
            progress_callback(1.0, "Video ready")
        return export_path

    total_source_size = 0
    for _, path in video_paths:
        try:
            total_source_size += os.path.getsize(path)
        except OSError:
            pass

    estimated_output_size = max(total_source_size, 100 * 1024 * 1024)  # Assume at least 100MB
    export_base = "/data" if os.path.exists("/data") else os.path.expanduser("~")

    if not has_sufficient_disk_space(estimated_output_size, export_base, min_free_gb=1):
        free_space = get_free_disk_space(export_base)
        free_gb = (free_space / (1024 ** 3)) if free_space else 0
        required_gb = estimated_output_size / (1024 ** 3)
        raise RuntimeError(
            f"Insufficient disk space: {free_gb:.1f}GB free, need ~{required_gb:.1f}GB"
        )

    if progress_callback:
        progress_callback(0.05, f"Preparing {len(video_paths)} segments")

    # Avoid starving interactive playback by waiting for free FFmpeg slots
    waited_for_capacity = False
    if MAX_CONCURRENT_FFMPEG > FFMPEG_RESERVED_FOR_PLAYBACK:
        if progress_callback:
            progress_callback(0.08, "Waiting for encoder availability")
        if not wait_for_ffmpeg_capacity(timeout=60.0, server_state=server_state):
            logger.warning(
                f"Export {route_base}:{camera} starting despite FFmpeg saturation"
            )
        else:
            waited_for_capacity = True

    if waited_for_capacity and progress_callback:
        progress_callback(0.1, "Starting video encoding")

    enable_performance_mode()

    route_info = f"export:{route_base}:{camera}"
    temp_output = get_export_output_path(route_base, camera) + ".tmp"
    concat_file = None

    # Ensure previous temp file removed
    if os.path.exists(temp_output):
        try:
            os.remove(temp_output)
        except OSError:
            pass

    try:
        if camera in HEVC_CAMERAS:
            concat_protocol = 'concat:' + '|'.join(path for _, path in video_paths)
            ffmpeg_cmd = [
                FFMPEG_BINARY,
                '-loglevel', 'error',
                '-f', 'hevc',
                '-r', '20',
                '-i', concat_protocol,
                '-c', 'copy',  # Copy all streams (video + audio if available)
                '-movflags', '+faststart',
                '-f', 'mp4',
                temp_output
            ]
        else:
            fd, concat_file = tempfile.mkstemp(prefix='route_concat_', suffix='.txt')
            with os.fdopen(fd, 'w') as concat_handle:
                for _, path in video_paths:
                    safe_path = path.replace("'", "\\'")
                    concat_handle.write(f"file '{safe_path}'\n")

            ffmpeg_cmd = [
                FFMPEG_BINARY,
                '-loglevel', 'error',
                '-f', 'concat',
                '-safe', '0',
                '-i', concat_file,
                '-c', 'copy',
                '-movflags', '+faststart',
                temp_output
            ]

        if progress_callback:
            progress_callback(0.35, "Merging video segments")

        if not server_state:
            raise RuntimeError("Server state required for FFmpeg process management")

        with FFmpegProcess(route_info, server_state, max_concurrent=MAX_CONCURRENT_FFMPEG, stream_logs=True) as ffmpeg_mgr:
            process = ffmpeg_mgr.start(ffmpeg_cmd)
            _, stderr = process.communicate()
            if process.returncode != 0:
                error_output = ''
                if stderr:
                    error_output = stderr.decode('utf-8', errors='ignore')
                raise RuntimeError(f"FFmpeg failed (exit {process.returncode}): {error_output[:400]}")

        os.replace(temp_output, get_export_output_path(route_base, camera))

        if progress_callback:
            progress_callback(1.0, "Video ready")

        return get_export_output_path(route_base, camera)

    finally:
        if concat_file and os.path.exists(concat_file):
            try:
                os.remove(concat_file)
            except OSError:
                pass
        if os.path.exists(temp_output):
            # Keep partially generated output only if process succeeded (rename already done)
            if not os.path.exists(get_export_output_path(route_base, camera)):
                try:
                    os.remove(temp_output)
                except OSError:
                    pass


# ============================================================================
# Export Streaming
# ============================================================================

def stream_route_export(route_base, camera, response_handler, server_state=None):
    """
    Stream a full-route export directly to HTTP response without caching.

    Args:
        route_base: Route identifier
        camera: Camera type
        response_handler: HTTP handler instance with wfile for output
        server_state: Server state instance for FFmpeg tracking

    Raises:
        RuntimeError, FileNotFoundError, ValueError on failure
    """
    if camera not in CAMERA_FILES:
        raise ValueError("Invalid camera type")

    if FFMPEG_BINARY is None:
        raise RuntimeError("FFmpeg is not available on this device")

    segments = get_route_segments(route_base)
    if not segments:
        raise FileNotFoundError("Route not found")

    camera_file = CAMERA_FILES[camera]

    video_paths = []
    missing_segments = []
    for seg in sorted(segments, key=lambda s: s['segment']):
        source_path = os.path.join(seg['path'], camera_file)
        if os.path.exists(source_path):
            video_paths.append((seg['segment'], source_path))
        else:
            missing_segments.append(seg['segment'])

    if missing_segments:
        missing_str = ', '.join(f"{seg:03d}" for seg in missing_segments[:10])
        raise FileNotFoundError(f"Missing {camera} video for segments: {missing_str}")

    if not video_paths:
        raise FileNotFoundError(f"No {camera} video segments available")

    # Set HTTP headers
    suggested_filename = generate_route_export_filename(route_base, camera, segments)
    response_handler.send_response(200)
    response_handler.send_header('Content-Type', 'video/mp4')
    response_handler.send_header('Content-Disposition', f'attachment; filename="{suggested_filename}"')
    response_handler.send_header('Transfer-Encoding', 'chunked')
    response_handler.end_headers()

    concat_file = None

    try:
        enable_performance_mode()

        # Build FFmpeg command to output to stdout
        if camera in HEVC_CAMERAS:
            concat_protocol = 'concat:' + '|'.join(path for _, path in video_paths)
            ffmpeg_cmd = [
                FFMPEG_BINARY,
                '-loglevel', 'error',
                '-f', 'hevc',
                '-r', '20',
                '-i', concat_protocol,
                '-c', 'copy',
                '-movflags', '+faststart+frag_keyframe+empty_moov',  # Enable streaming
                '-f', 'mp4',
                'pipe:1'  # Output to stdout
            ]
        else:
            fd, concat_file = tempfile.mkstemp(prefix='route_concat_', suffix='.txt')
            with os.fdopen(fd, 'w') as concat_handle:
                for _, path in video_paths:
                    safe_path = path.replace("'", "\\'")
                    concat_handle.write(f"file '{safe_path}'\n")

            ffmpeg_cmd = [
                FFMPEG_BINARY,
                '-loglevel', 'error',
                '-f', 'concat',
                '-safe', '0',
                '-i', concat_file,
                '-c', 'copy',
                '-movflags', '+faststart+frag_keyframe+empty_moov',  # Enable streaming
                '-f', 'mp4',
                'pipe:1'  # Output to stdout
            ]

        route_info = f"stream-export:{route_base}:{camera}"

        if not server_state:
            raise RuntimeError("Server state required for FFmpeg process management")

        with FFmpegProcess(route_info, server_state, max_concurrent=MAX_CONCURRENT_FFMPEG, stream_logs=True) as ffmpeg_mgr:
            process = ffmpeg_mgr.start(ffmpeg_cmd)

            # Stream FFmpeg output to HTTP response
            chunk_size = 64 * 1024  # 64KB chunks
            while True:
                chunk = process.stdout.read(chunk_size)
                if not chunk:
                    break
                try:
                    # Send chunk size in hex followed by chunk data (chunked transfer encoding)
                    response_handler.wfile.write(f"{len(chunk):X}\r\n".encode())
                    response_handler.wfile.write(chunk)
                    response_handler.wfile.write(b"\r\n")
                    response_handler.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    # Client disconnected
                    logger.warning(f"Client disconnected during stream: {route_base}/{camera}")
                    process.kill()
                    break

            # Send final chunk
            response_handler.wfile.write(b"0\r\n\r\n")
            response_handler.wfile.flush()

            # Wait for process to finish and check for errors
            process.wait()
            if process.returncode != 0:
                stderr = process.stderr.read().decode('utf-8', errors='ignore') if process.stderr else ''
                logger.error(f"FFmpeg streaming failed (exit {process.returncode}): {stderr[:400]}")
                # Can't send error to client at this point, connection already started

    finally:
        if concat_file and os.path.exists(concat_file):
            try:
                os.remove(concat_file)
            except OSError:
                pass


# ============================================================================
# Helper Functions
# ============================================================================

def parse_route_datetime(route_base):
    """Parse route base name to extract datetime
    Example: 2024-09-18--14-30-00 -> datetime(2024, 9, 18, 14, 30, 0)
    Example: a2a0ccea32023010|2024-09-18--14-30-00 -> datetime(2024, 9, 18, 14, 30, 0)
    Returns None for non-standard route names
    """
    try:
        # Remove dongle ID if present (format: dongle_id|date--time)
        if '|' in route_base:
            route_base = route_base.split('|')[1]

        # Split by --
        parts = route_base.split('--')
        if len(parts) >= 2:
            date_part = parts[0]  # 2024-09-18
            time_part = parts[1]  # 14-30-00

            # Check if date_part looks like a date (YYYY-MM-DD format)
            if len(date_part.split('-')) != 3:
                return None

            # Parse date
            year, month, day = map(int, date_part.split('-'))

            # Validate year is reasonable (not hex like 000000ad)
            if year < 2000 or year > 2100:
                return None

            # Parse time
            time_components = time_part.split('-')
            hour = int(time_components[0]) if len(time_components) > 0 else 0
            minute = int(time_components[1]) if len(time_components) > 1 else 0
            second = int(time_components[2]) if len(time_components) > 2 else 0

            return datetime(year, month, day, hour, minute, second)
    except (ValueError, TypeError):
        # Silently return None for non-standard route names
        pass
    return None


def get_segment_number(route_name):
    """Extract segment number from route name
    Example: 2024-09-18--14-30-00--5 -> 5
    """
    match = re.search(r'--(\d+)$', route_name)
    if match:
        try:
            return int(match.group(1))
        except:
            return 0
    return 0


@lru_cache(maxsize=100)
def get_route_segments(route_base):
    """Get all segments for a route"""
    if not os.path.exists(ROUTES_DIR):
        return []

    segments = []
    pattern = re.compile(f"^{re.escape(route_base)}--\\d+$")

    for entry in os.listdir(ROUTES_DIR):
        if pattern.match(entry):
            entry_path = os.path.join(ROUTES_DIR, entry)
            if os.path.isdir(entry_path):
                seg_num = get_segment_number(entry)
                segments.append({
                    'path': entry_path,
                    'name': entry,
                    'segment': seg_num
                })

    return sorted(segments, key=lambda x: x['segment'])


def wait_for_ffmpeg_capacity(timeout=30.0, reserve=FFMPEG_RESERVED_FOR_PLAYBACK, server_state=None):
    """
    Wait until enough FFmpeg capacity is available for a background task.

    Args:
        timeout: Maximum seconds to wait before giving up (None to wait indefinitely)
        reserve: Slots to keep free for interactive playback
        server_state: Server state instance for checking FFmpeg count

    Returns:
        bool: True if capacity became available, False if timed out
    """
    import time

    if not server_state:
        return True  # Can't check, assume OK

    deadline = None if timeout is None else time.time() + timeout
    min_capacity = max(1, MAX_CONCURRENT_FFMPEG - reserve)

    while True:
        current = server_state.get_ffmpeg_count()
        if current < min_capacity:
            return True

        if deadline and time.time() >= deadline:
            return False

        time.sleep(0.5)
