#!/usr/bin/env python3
"""
BluePilot Backend Route Scanner Module

CRITICAL FUNCTION: scan_routes()

This module contains the core route discovery and metadata building function.
The scan_routes() function is responsible for:
- Scanning ROUTES_DIR for all available routes
- Building comprehensive route metadata (datetime, duration, segments)
- Loading GPS metrics from cache (fast lookup, no log processing)
- Calculating deletion risk for each route
- Formatting data for API response
- Returns complete route list with all metadata

This function is called frequently by the web UI and must be fast.
It uses cached data only and does NOT process logs during the scan.
Background preprocessing handles log analysis during idle time.
"""

import os
import json
import logging
from datetime import datetime, timedelta

from bluepilot.backend.utils.params_fallback import get_params_with_defaults

logger = logging.getLogger(__name__)

# Import configuration
from bluepilot.backend.config import (
    ROUTES_DIR,
    METRICS_CACHE,
)

# Import route utilities from modular structure
from bluepilot.backend.routes import (
    get_route_base_name,
    parse_route_datetime,
    format_display_date,
    format_time_12hr,
    format_elapsed_time,
    get_route_segments,
    get_file_size,
    format_size,
    get_disk_space_info,
    get_route_fingerprint,
    get_route_drive_stats_cached_only,
)

# Import storage preservation utilities
from bluepilot.backend.storage import (
    get_cached_deletion_data,
    calculate_route_deletion_risk,
    check_route_preserve_status,
)

# Import video metadata utilities
from bluepilot.backend.video import (
    get_video_files,
)

# Import params for unit preference
params = get_params_with_defaults({"IsMetric": False})


def scan_routes():
    """Scan routes directory and build route metadata

    Returns route list with cached GPS metrics only (fast).
    Does NOT process logs or geocode - that's handled by:
    - Background preprocessor during idle time
    - Individual API endpoints on-demand

    Returns:
        list: List of route dicts with comprehensive metadata including:
            - baseName: Route base name
            - displayDate: Formatted date string
            - displayTime: 12-hour formatted time
            - displayEndTime: 12-hour formatted end time
            - timestamp: ISO timestamp
            - elapsedTime: Human-readable elapsed time
            - segments: Number of segments
            - duration: Formatted duration string
            - size: Formatted total size
            - sizeBytes: Total size in bytes
            - hasVideo: Dict of available cameras
            - isPreserved: Preserve status
            - isStarred: Alias for isPreserved
            - dateTime: ISO datetime for sorting
            - mileage: Formatted distance (metric or imperial)
            - avgSpeed: Formatted average speed
            - topSpeed: Formatted max speed
            - hasGpsData: Whether GPS data is available
            - startLocation: Reverse geocoded start location
            - endLocation: Reverse geocoded end location
            - deletionRisk: Deletion risk assessment
            - fingerprint: Vehicle fingerprint data
            - driveStats: Drive statistics
    """
    if not os.path.exists(ROUTES_DIR):
        logger.warning(f"Routes directory not found: {ROUTES_DIR}")
        return []

    # Calculate deletion queue data (cached)
    deletion_data = get_cached_deletion_data()
    disk_info = get_disk_space_info()

    routes_dict = {}
    processed_bases = set()

    for entry in os.listdir(ROUTES_DIR):
        entry_path = os.path.join(ROUTES_DIR, entry)

        if not os.path.isdir(entry_path):
            continue

        # CRITICAL: Skip non-route directories
        # Routes MUST contain "--" and NOT be in exclusion list
        if entry in ('boot', 'crash') or '--' not in entry:
            continue

        base_name = get_route_base_name(entry)

        # Skip if we've already processed this base route
        if base_name in processed_bases:
            continue

        processed_bases.add(base_name)

        # Parse datetime from base name
        route_dt = parse_route_datetime(base_name)

        # If datetime parsing fails (e.g., dongle ID routes), use fallback
        if route_dt is None:
            # Use directory modification time as fallback (silently)
            try:
                mtime = os.path.getmtime(entry_path)
                route_dt = datetime.fromtimestamp(mtime)
            except:
                logger.warning(f"Could not parse datetime from route: {base_name}, skipping")
                continue

        # Get all segments for this base route
        segments = get_route_segments(base_name)
        if not segments:
            continue

        # Calculate total size across all segments
        total_size = sum(get_file_size(seg['path']) for seg in segments)

        # Check which cameras have footage across all segments
        has_video = {
            'front': False,
            'wide': False,
            'driver': False,
            'lq': False
        }

        for seg in segments:
            videos = get_video_files(seg['path'])
            for camera in videos.keys():
                has_video[camera] = True

        # Check if route is preserved via xattr
        is_preserved = check_route_preserve_status(base_name)

        # Calculate duration (1 minute per segment)
        duration_seconds = len(segments) * 60
        hours = duration_seconds // 3600
        minutes = (duration_seconds % 3600) // 60
        if hours > 0:
            duration_str = f"{hours}h {minutes}m"
        else:
            duration_str = f"{minutes}m"

        # Calculate end time from last segment's timestamp
        # Parse the last segment name to get its timestamp
        last_segment = segments[-1]
        last_segment_dt = parse_route_datetime(last_segment['name'].rsplit('--', 1)[0])
        if last_segment_dt:
            # Add 1 minute to account for the segment's duration
            end_dt = last_segment_dt + timedelta(minutes=1)
        else:
            # Fallback: use start time + duration if parsing fails
            end_dt = route_dt + timedelta(seconds=duration_seconds)

        # Check if GPS metrics are already cached (don't process logs during scan)
        cache_file = os.path.join(METRICS_CACHE, f"{base_name}.json")
        if os.path.exists(cache_file):
            try:
                with open(cache_file) as f:
                    gps_metrics = json.load(f)
            except Exception as e:
                logger.debug(f"Error reading GPS cache for {base_name}: {e}")
                gps_metrics = {'has_gps_data': False}
        else:
            # No cache - return placeholder, let background preprocessor handle it
            gps_metrics = {'has_gps_data': False}

        # Format GPS metrics for display based on user's unit preference
        if gps_metrics['has_gps_data']:
            # Get unit preference (False = imperial/mph, True = metric/km/h)
            is_metric = params.get_bool("IsMetric")

            if is_metric:
                # Metric units
                distance_km = gps_metrics['total_distance_meters'] / 1000
                avg_speed_kmh = gps_metrics['avg_speed_ms'] * 3.6
                max_speed_kmh = gps_metrics['max_speed_ms'] * 3.6

                mileage_str = f"{distance_km:.2f} km"
                avg_speed_str = f"{avg_speed_kmh:.1f} km/h"
                max_speed_str = f"{max_speed_kmh:.1f} km/h"
            else:
                # Imperial units
                distance_miles = gps_metrics['total_distance_meters'] / 1609.34
                avg_speed_mph = gps_metrics['avg_speed_ms'] * 2.237
                max_speed_mph = gps_metrics['max_speed_ms'] * 2.237

                mileage_str = f"{distance_miles:.2f} mi"
                avg_speed_str = f"{avg_speed_mph:.1f} mph"
                max_speed_str = f"{max_speed_mph:.1f} mph"

            # Load location names from cache (saved by background preprocessor or /api/geocode)
            start_location = gps_metrics.get('start_location')
            end_location = gps_metrics.get('end_location')
        else:
            mileage_str = None
            avg_speed_str = None
            max_speed_str = None
            start_location = None
            end_location = None

        # Calculate deletion risk for this route
        deletion_risk = calculate_route_deletion_risk(base_name, segments, deletion_data, disk_info)

        # Get fingerprint data if cached (don't process logs during scan)
        fingerprint_data = get_route_fingerprint(base_name, segments)

        # Get drive statistics if cached (don't extract during scan - too slow)
        drive_stats = get_route_drive_stats_cached_only(base_name)

        # Build route info matching old panel structure
        routes_dict[base_name] = {
            'baseName': base_name,
            'displayDate': format_display_date(route_dt),
            'displayTime': format_time_12hr(route_dt),
            'displayEndTime': format_time_12hr(end_dt),
            'timestamp': route_dt.isoformat(),
            'elapsedTime': format_elapsed_time(route_dt),
            'segments': len(segments),
            'duration': duration_str,
            'size': format_size(total_size),
            'sizeBytes': total_size,
            'hasVideo': has_video,
            'isPreserved': is_preserved,
            'isStarred': is_preserved,
            'dateTime': route_dt.isoformat(),  # For sorting
            # GPS metrics
            'mileage': mileage_str,
            'avgSpeed': avg_speed_str,
            'topSpeed': max_speed_str,
            'hasGpsData': gps_metrics['has_gps_data'],
            # Location names (reverse geocoded)
            'startLocation': start_location,
            'endLocation': end_location,
            # Deletion risk info
            'deletionRisk': deletion_risk,
            # Fingerprint data
            'fingerprint': fingerprint_data if fingerprint_data else None,
            # Drive statistics
            'driveStats': drive_stats if drive_stats else None,
        }

    # Convert to list and sort by datetime (newest first)
    routes = list(routes_dict.values())
    routes.sort(key=lambda x: x['dateTime'], reverse=True)

    logger.info(f"Scanned {len(routes)} routes")
    return routes
