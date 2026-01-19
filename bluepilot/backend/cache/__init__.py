#!/usr/bin/env python3
"""
BluePilot Backend Cache Management
Cache metrics and cleanup utilities
"""

# Import all cache metrics functions
from .metrics import (
    get_directory_size,
    get_cache_size,
    get_remux_cache_size,
    get_thumbnail_cache_size,
    get_metrics_cache_size,
    get_drive_stats_cache_size,
    get_fingerprint_cache_size,
    get_export_cache_size,
    get_route_export_cache_size,
    get_all_cache_sizes,
)

# Import cache cleanup functions
from .cleanup import (
    is_route_starred,
    cleanup_old_cache,
    MAX_CACHE_SIZE_GB,
    CACHE_CLEANUP_THRESHOLD,
    CACHE_EXPIRATION_HOURS,
)

__all__ = [
    # Metrics functions
    'get_directory_size',
    'get_cache_size',
    'get_remux_cache_size',
    'get_thumbnail_cache_size',
    'get_metrics_cache_size',
    'get_drive_stats_cache_size',
    'get_fingerprint_cache_size',
    'get_export_cache_size',
    'get_route_export_cache_size',
    'get_all_cache_sizes',

    # Cleanup functions
    'is_route_starred',
    'cleanup_old_cache',

    # Configuration constants
    'MAX_CACHE_SIZE_GB',
    'CACHE_CLEANUP_THRESHOLD',
    'CACHE_EXPIRATION_HOURS',
]
