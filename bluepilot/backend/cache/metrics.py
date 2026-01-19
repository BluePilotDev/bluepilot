#!/usr/bin/env python3
"""
BluePilot Backend Cache Metrics
Functions for measuring cache sizes across different cache directories
"""

import os
import logging

logger = logging.getLogger(__name__)

# Import configuration constants
from bluepilot.backend.config import (
    REMUX_CACHE,
    THUMBNAIL_CACHE,
    METRICS_CACHE,
    ROUTE_EXPORT_CACHE,
)

# Import format_size utility
from bluepilot.backend.routes.segments import format_size


def get_directory_size(directory_path):
    """Get total size of a directory in bytes

    Args:
        directory_path: Path to directory to measure

    Returns:
        int: Total size in bytes
    """
    total = 0
    try:
        if os.path.exists(directory_path):
            for entry in os.scandir(directory_path):
                if entry.is_file():
                    total += entry.stat().st_size
    except OSError as e:
        logger.warning(f"Error calculating directory size for {directory_path}: {e}")
    return total


def get_cache_size():
    """Get total size of remux cache in bytes (deprecated - use get_remux_cache_size)"""
    return get_remux_cache_size()


def get_remux_cache_size():
    """Get total size of remux cache in bytes"""
    return get_directory_size(REMUX_CACHE)


def get_thumbnail_cache_size():
    """Get total size of thumbnail cache in bytes"""
    return get_directory_size(THUMBNAIL_CACHE)


def get_metrics_cache_size():
    """Get total size of GPS metrics cache in bytes"""
    return get_directory_size(METRICS_CACHE)


def get_drive_stats_cache_size():
    """Get total size of drive stats cache in bytes"""
    from bluepilot.backend.routes.processing import DRIVE_STATS_CACHE
    return get_directory_size(DRIVE_STATS_CACHE)


def get_fingerprint_cache_size():
    """Get total size of fingerprint cache in bytes"""
    from bluepilot.backend.routes.processing import FINGERPRINT_CACHE
    return get_directory_size(FINGERPRINT_CACHE)


def get_export_cache_size():
    """Get total size of route export cache in bytes"""
    return get_directory_size(ROUTE_EXPORT_CACHE)


# Alias for compatibility
get_route_export_cache_size = get_export_cache_size


def get_all_cache_sizes():
    """Get sizes of all cache directories

    Returns:
        dict with cache sizes in bytes and formatted strings
    """
    remux_bytes = get_remux_cache_size()
    thumbnail_bytes = get_thumbnail_cache_size()
    metrics_bytes = get_metrics_cache_size()
    drive_stats_bytes = get_drive_stats_cache_size()
    fingerprint_bytes = get_fingerprint_cache_size()
    export_bytes = get_export_cache_size()

    total_bytes = (remux_bytes + thumbnail_bytes + metrics_bytes +
                   drive_stats_bytes + fingerprint_bytes + export_bytes)

    return {
        'remux_bytes': remux_bytes,
        'thumbnail_bytes': thumbnail_bytes,
        'metrics_bytes': metrics_bytes,
        'drive_stats_bytes': drive_stats_bytes,
        'fingerprint_bytes': fingerprint_bytes,
        'export_bytes': export_bytes,
        'total_bytes': total_bytes,
        'formatted': {
            'remux': format_size(remux_bytes),
            'thumbnails': format_size(thumbnail_bytes),
            'metrics': format_size(metrics_bytes),
            'drive_stats': format_size(drive_stats_bytes),
            'fingerprints': format_size(fingerprint_bytes),
            'exports': format_size(export_bytes),
            'total': format_size(total_bytes)
        }
    }
