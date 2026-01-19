#!/usr/bin/env python3
"""
Route metadata helpers shared across endpoints and scanners.
"""

import os
from datetime import datetime, timedelta
from typing import Dict, Any, List

from bluepilot.backend.routes.parsing import (
    parse_route_datetime,
    format_display_date,
    format_time_12hr,
    format_elapsed_time,
)
from bluepilot.backend.routes.segments import (
    get_file_size,
    format_size,
    get_disk_space_info,
)
from bluepilot.backend.routes.processing import (
    get_route_gps_metrics,
    get_route_drive_stats_cached_only,
    get_route_fingerprint,
)
from bluepilot.backend.storage import (
    get_cached_deletion_data,
    calculate_route_deletion_risk,
    check_route_preserve_status,
)
from bluepilot.backend.video.metadata import get_video_files


def build_route_metadata(route_base: str, segments: List[Dict[str, Any]], params) -> Dict[str, Any]:
    """
    Build full route metadata payload used by the /api/routes/{route} endpoint.

    Args:
        route_base: Base route name
        segments: List of segment dicts (from get_route_segments)
        params: Params instance (real or fallback)

    Returns:
        dict payload ready for JSON response
    """
    # Parse datetime
    route_dt = parse_route_datetime(route_base)
    if route_dt is None and segments:
        try:
            mtime = os.path.getmtime(segments[0]['path'])
            route_dt = parse_route_datetime(route_base) or parse_route_datetime(segments[0]['name'].rsplit('--', 1)[0])
            if route_dt is None:
                route_dt = datetime.fromtimestamp(mtime)
        except Exception:
            route_dt = datetime.now()

    # Size/duration
    total_size = sum(get_file_size(seg['path']) for seg in segments)
    duration_seconds = len(segments) * 60
    hours = duration_seconds // 3600
    minutes = (duration_seconds % 3600) // 60
    duration_str = f"{hours}h {minutes}m" if hours > 0 else f"{minutes}m"

    # End time
    end_dt = None
    if segments:
        last_segment = segments[-1]
        last_segment_dt = parse_route_datetime(last_segment['name'].rsplit('--', 1)[0])
        if last_segment_dt:
            end_dt = last_segment_dt + timedelta(minutes=1)
    end_dt = end_dt or (route_dt + timedelta(seconds=duration_seconds))

    # Cameras present
    has_video = {'front': False, 'wide': False, 'driver': False, 'lq': False}
    for seg in segments:
        videos = get_video_files(seg['path'])
        for camera in videos.keys():
            has_video[camera] = True

    # GPS metrics
    gps_metrics = get_route_gps_metrics(route_base, segments, include_coordinates=False) or {'has_gps_data': False}
    mileage_str = None
    avg_speed_str = None
    max_speed_str = None
    start_location = gps_metrics.get('start_location')
    end_location = gps_metrics.get('end_location')
    if gps_metrics.get('has_gps_data'):
        is_metric = False
        try:
            is_metric = params.get_bool("IsMetric")
        except Exception:
            pass

        if is_metric:
            distance_km = gps_metrics.get('total_distance_meters', 0) / 1000
            avg_speed_kmh = gps_metrics.get('avg_speed_ms', 0) * 3.6
            max_speed_kmh = gps_metrics.get('max_speed_ms', 0) * 3.6
            mileage_str = f"{distance_km:.2f} km"
            avg_speed_str = f"{avg_speed_kmh:.1f} km/h"
            max_speed_str = f"{max_speed_kmh:.1f} km/h"
        else:
            distance_miles = gps_metrics.get('total_distance_meters', 0) / 1609.34
            avg_speed_mph = gps_metrics.get('avg_speed_ms', 0) * 2.237
            max_speed_mph = gps_metrics.get('max_speed_ms', 0) * 2.237
            mileage_str = f"{distance_miles:.2f} mi"
            avg_speed_str = f"{avg_speed_mph:.1f} mph"
            max_speed_str = f"{max_speed_mph:.1f} mph"

    # Segments detail
    segments_detail = []
    for seg in segments:
        videos = get_video_files(seg['path'])
        segments_detail.append({
            'number': seg['segment'],
            'name': seg['name'],
            'path': seg['path'],
            'videos': videos
        })

    # Drive stats and fingerprint
    drive_stats = get_route_drive_stats_cached_only(route_base)
    fingerprint_data = get_route_fingerprint(route_base, segments)

    # Deletion risk and preserve status
    deletion_data = get_cached_deletion_data()
    disk_info = get_disk_space_info()
    deletion_risk = calculate_route_deletion_risk(route_base, segments, deletion_data, disk_info)
    is_preserved = check_route_preserve_status(route_base)

    return {
        'success': True,
        'baseName': route_base,
        'id': route_base,
        'displayDate': format_display_date(route_dt),
        'date': format_display_date(route_dt),
        'displayTime': format_time_12hr(route_dt),
        'displayEndTime': format_time_12hr(end_dt),
        'timestamp': route_dt.isoformat(),
        'dateTime': route_dt.isoformat(),
        'elapsedTime': format_elapsed_time(route_dt),
        'start_time': route_dt.isoformat(),
        'end_time': end_dt.isoformat(),
        'duration': duration_str,
        'size': format_size(total_size),
        'sizeBytes': total_size,
        'totalSegments': len(segments_detail),
        'hasVideo': has_video,
        'mileage': mileage_str,
        'distance': mileage_str,
        'avgSpeed': avg_speed_str,
        'topSpeed': max_speed_str,
        'hasGpsData': gps_metrics.get('has_gps_data', False),
        'startLocation': start_location,
        'endLocation': end_location,
        'avg_speed': avg_speed_str,
        'top_speed': max_speed_str,
        'start_location': start_location,
        'end_location': end_location,
        'isPreserved': is_preserved,
        'isStarred': is_preserved,
        'preserved': is_preserved,
        'deletionRisk': deletion_risk,
        'processing': drive_stats is None if drive_stats is not None else False,
        'segments': segments_detail,
        'driveStats': drive_stats if drive_stats else None,
        'fingerprint': fingerprint_data if fingerprint_data else None
    }
