"""
BluePilot Backend Routes Module
Route analysis, GPS metrics, and background preprocessing
"""

# Re-export all functions from processing module for backwards compatibility
from .processing import (
    haversine_distance,
    reverse_geocode,
    extract_gps_metrics_from_segment,
    get_route_gps_metrics,
    generate_thumbnail,
    get_route_fingerprint,
    get_route_drive_stats,
    get_route_drive_stats_cached_only,
    check_processing_status,
    process_route,
    kill_existing_process,
)

# Re-export parsing functions
from .parsing import (
    get_route_base_name,
    get_segment_number,
    parse_route_datetime,
    format_time_12hr,
    format_display_date,
    format_elapsed_time,
)

# Re-export segment functions
from .segments import (
    get_route_segments,
    get_file_size,
    format_size,
    get_disk_space_info,
)

# Re-export scanner function
from .scanner import (
    scan_routes,
)
from .utils import (
    build_route_metadata,
)

__all__ = [
    # Processing
    'haversine_distance',
    'reverse_geocode',
    'extract_gps_metrics_from_segment',
    'get_route_gps_metrics',
    'generate_thumbnail',
    'get_route_fingerprint',
    'get_route_drive_stats',
    'get_route_drive_stats_cached_only',
    'check_processing_status',
    'process_route',
    'kill_existing_process',
    # Parsing
    'get_route_base_name',
    'get_segment_number',
    'parse_route_datetime',
    'format_time_12hr',
    'format_display_date',
    'format_elapsed_time',
    # Segments
    'get_route_segments',
    'get_file_size',
    'format_size',
    'get_disk_space_info',
    # Scanner
    'scan_routes',
    # Utilities
    'build_route_metadata',
]
