#!/usr/bin/env python3
"""
Log Download Handlers for BluePilot Web Server
Provides qlog and rlog downloads for routes
"""

import os
import re
import json
import logging
import subprocess
import tempfile

logger = logging.getLogger(__name__)


def _sanitize_filename_component(component):
    """Sanitize string for safe filename usage"""
    if not component:
        return None
    value = re.sub(r'[^A-Za-z0-9]+', '-', str(component)).strip('-')
    return value or None


def _parse_route_datetime(route_base):
    """Parse route base name to extract datetime
    Example: 2024-09-18--14-30-00 -> datetime(2024, 9, 18, 14, 30, 0)
    Example: a2a0ccea32023010|2024-09-18--14-30-00 -> datetime(2024, 9, 18, 14, 30, 0)
    Returns None for non-standard route names
    """
    from datetime import datetime
    try:
        # Remove dongle ID if present (format: dongle_id|date--time)
        if '|' in route_base:
            route_base = route_base.split('|')[1]

        parts = route_base.split('--')
        if len(parts) >= 2:
            date_part = parts[0]  # 2024-09-18
            time_part = parts[1]  # 14-30-00

            if len(date_part.split('-')) != 3:
                return None

            year, month, day = map(int, date_part.split('-'))
            if year < 2000 or year > 2100:
                return None

            time_parts = time_part.split('-')
            if len(time_parts) >= 2:
                hour = int(time_parts[0])
                minute = int(time_parts[1])
                second = int(time_parts[2]) if len(time_parts) >= 3 else 0
                return datetime(year, month, day, hour, minute, second)
        return None
    except (ValueError, IndexError):
        return None


def generate_log_filename(route_base, log_type, segments=None):
    """
    Create a descriptive filename for log files.

    Format example: 10032024_143000_Ypsilanti-MI_qlog.zst

    Args:
        route_base: Route identifier
        log_type: 'qlog' or 'rlog'
        segments: Optional list of segment dictionaries for metrics lookup

    Returns:
        Filename string with date/time, location, and log type
    """
    from bluepilot.backend.config import METRICS_CACHE

    components = []

    # Use parsed route datetime when available
    route_dt = _parse_route_datetime(route_base)
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

    # Append log type at the end
    components.append(log_type)

    # Fallback to route_base_logtype if no datetime available
    if not components or len(components) == 1:
        return f"{route_base}_{log_type}.zst"

    filename_base = "_".join(filter(None, components))
    return f"{filename_base}.zst"


def _broadcast_log_download_update(server_state, route_base, log_type, status_info):
    """Broadcast log download status via WebSocket"""
    if not server_state:
        return
    broadcaster = server_state.get_broadcaster()
    if broadcaster:
        try:
            from bluepilot.backend.realtime import WebSocketEvent
            payload = {
                'route': route_base,
                'logType': log_type,
                **status_info
            }
            broadcaster.broadcast(WebSocketEvent.LOG_DOWNLOAD_UPDATE, payload)
        except Exception as e:
            logger.debug(f"Error broadcasting log download update: {e}")


def handle_qlog_download(handler, path, get_route_segments, server_state=None):
    """Handle GET /api/download/qlog/{route_base}"""
    parts = path.split('/')[4:]
    if len(parts) < 1:
        handler.send_json_response({'error': 'Invalid qlog path'}, 400)
        return

    route_base = parts[0]
    segments = get_route_segments(route_base)
    if not segments:
        handler.send_json_response({'error': 'Route not found'}, 404)
        return

    try:
        # Collect all qlog files from segments
        qlog_files = []
        for segment in sorted(segments, key=lambda s: s['segment']):
            qlog_path = os.path.join(segment['path'], 'qlog.zst')
            if os.path.exists(qlog_path):
                qlog_files.append(qlog_path)

        if not qlog_files:
            handler.send_json_response({'error': 'No qlog files found for this route'}, 404)
            return

        # Generate descriptive filename with date/time and location
        filename = generate_log_filename(route_base, 'qlog', segments)

        # If single segment, serve directly
        if len(qlog_files) == 1:
            handler.send_file_response(qlog_files[0], download_filename=filename)
            return

        # Multiple segments - concatenate qlogs with progress tracking
        # Create temporary combined file on data partition (not /tmp which is tiny tmpfs)
        from bluepilot.backend.config import IMPORT_TEMP_DIR
        temp_fd, temp_path = tempfile.mkstemp(suffix='.zst', prefix='qlog_', dir=IMPORT_TEMP_DIR)

        # Start tracking
        total_files = len(qlog_files)
        if server_state:
            server_state.start_log_download(route_base, 'qlog', total_files, f"Concatenating {total_files} qlog files")
            _broadcast_log_download_update(server_state, route_base, 'qlog', {
                'status': 'processing',
                'progress': 0,
                'message': f"Concatenating {total_files} qlog files",
                'totalFiles': total_files,
                'filesProcessed': 0
            })

        try:
            with os.fdopen(temp_fd, 'wb') as temp_file:
                for i, qlog_file in enumerate(qlog_files):
                    with open(qlog_file, 'rb') as f:
                        temp_file.write(f.read())

                    # Update progress
                    if server_state:
                        progress = (i + 1) / total_files
                        server_state.update_log_download(route_base, 'qlog',
                            progress=progress,
                            files_processed=i + 1,
                            message=f"Processing segment {i + 1} of {total_files}"
                        )
                        _broadcast_log_download_update(server_state, route_base, 'qlog', {
                            'status': 'processing',
                            'progress': progress,
                            'progressPercent': int(progress * 100),
                            'message': f"Processing segment {i + 1} of {total_files}",
                            'totalFiles': total_files,
                            'filesProcessed': i + 1
                        })

            # Mark complete before sending
            if server_state:
                server_state.complete_log_download(route_base, 'qlog', "Download ready")
                _broadcast_log_download_update(server_state, route_base, 'qlog', {
                    'status': 'ready',
                    'progress': 1.0,
                    'progressPercent': 100,
                    'message': 'Download ready',
                    'totalFiles': total_files,
                    'filesProcessed': total_files
                })

            handler.send_file_response(temp_path, download_filename=filename)
        finally:
            # Clean up temp file after sending
            try:
                os.remove(temp_path)
            except OSError:
                pass
            # Clear status after download completes
            if server_state:
                server_state.clear_log_download(route_base, 'qlog')

    except Exception as e:
        logger.error(f"Error serving qlog for {route_base}: {e}", exc_info=True)
        if server_state:
            server_state.fail_log_download(route_base, 'qlog', str(e))
            _broadcast_log_download_update(server_state, route_base, 'qlog', {
                'status': 'error',
                'message': str(e)
            })
            server_state.clear_log_download(route_base, 'qlog')
        handler.send_json_response({'error': f'Failed to serve qlog: {str(e)}'}, 500)


def handle_rlog_download(handler, path, get_route_segments, server_state=None):
    """Handle GET /api/download/rlog/{route_base}"""
    parts = path.split('/')[4:]
    if len(parts) < 1:
        handler.send_json_response({'error': 'Invalid rlog path'}, 400)
        return

    route_base = parts[0]
    segments = get_route_segments(route_base)
    if not segments:
        handler.send_json_response({'error': 'Route not found'}, 404)
        return

    try:
        # Collect all rlog files from segments
        rlog_files = []
        for segment in sorted(segments, key=lambda s: s['segment']):
            rlog_path = os.path.join(segment['path'], 'rlog.zst')
            if os.path.exists(rlog_path):
                rlog_files.append(rlog_path)

        if not rlog_files:
            handler.send_json_response({'error': 'No rlog files found for this route'}, 404)
            return

        # Generate descriptive filename with date/time and location
        filename = generate_log_filename(route_base, 'rlog', segments)

        # If single segment, serve directly
        if len(rlog_files) == 1:
            handler.send_file_response(rlog_files[0], download_filename=filename)
            return

        # Multiple segments - concatenate rlogs with progress tracking
        # Create temporary combined file on data partition (not /tmp which is tiny tmpfs)
        from bluepilot.backend.config import IMPORT_TEMP_DIR
        temp_fd, temp_path = tempfile.mkstemp(suffix='.zst', prefix='rlog_', dir=IMPORT_TEMP_DIR)

        # Start tracking
        total_files = len(rlog_files)
        if server_state:
            server_state.start_log_download(route_base, 'rlog', total_files, f"Concatenating {total_files} rlog files")
            _broadcast_log_download_update(server_state, route_base, 'rlog', {
                'status': 'processing',
                'progress': 0,
                'message': f"Concatenating {total_files} rlog files",
                'totalFiles': total_files,
                'filesProcessed': 0
            })

        try:
            with os.fdopen(temp_fd, 'wb') as temp_file:
                for i, rlog_file in enumerate(rlog_files):
                    with open(rlog_file, 'rb') as f:
                        temp_file.write(f.read())

                    # Update progress
                    if server_state:
                        progress = (i + 1) / total_files
                        server_state.update_log_download(route_base, 'rlog',
                            progress=progress,
                            files_processed=i + 1,
                            message=f"Processing segment {i + 1} of {total_files}"
                        )
                        _broadcast_log_download_update(server_state, route_base, 'rlog', {
                            'status': 'processing',
                            'progress': progress,
                            'progressPercent': int(progress * 100),
                            'message': f"Processing segment {i + 1} of {total_files}",
                            'totalFiles': total_files,
                            'filesProcessed': i + 1
                        })

            # Mark complete before sending
            if server_state:
                server_state.complete_log_download(route_base, 'rlog', "Download ready")
                _broadcast_log_download_update(server_state, route_base, 'rlog', {
                    'status': 'ready',
                    'progress': 1.0,
                    'progressPercent': 100,
                    'message': 'Download ready',
                    'totalFiles': total_files,
                    'filesProcessed': total_files
                })

            handler.send_file_response(temp_path, download_filename=filename)
        finally:
            # Clean up temp file after sending
            try:
                os.remove(temp_path)
            except OSError:
                pass
            # Clear status after download completes
            if server_state:
                server_state.clear_log_download(route_base, 'rlog')

    except Exception as e:
        logger.error(f"Error serving rlog for {route_base}: {e}", exc_info=True)
        if server_state:
            server_state.fail_log_download(route_base, 'rlog', str(e))
            _broadcast_log_download_update(server_state, route_base, 'rlog', {
                'status': 'error',
                'message': str(e)
            })
            server_state.clear_log_download(route_base, 'rlog')
        handler.send_json_response({'error': f'Failed to serve rlog: {str(e)}'}, 500)


def get_log_sizes(route_base, segments):
    """
    Calculate total qlog and rlog sizes for a route

    Args:
        route_base: Route identifier
        segments: List of segment dictionaries

    Returns:
        Dict with 'qlog' and 'rlog' keys containing sizes in bytes
    """
    qlog_size = 0
    rlog_size = 0

    for segment in segments:
        qlog_path = os.path.join(segment['path'], 'qlog.zst')
        if os.path.exists(qlog_path):
            try:
                qlog_size += os.path.getsize(qlog_path)
            except OSError:
                pass

        rlog_path = os.path.join(segment['path'], 'rlog.zst')
        if os.path.exists(rlog_path):
            try:
                rlog_size += os.path.getsize(rlog_path)
            except OSError:
                pass

    return {
        'qlog': qlog_size,
        'rlog': rlog_size
    }
