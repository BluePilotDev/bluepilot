#!/usr/bin/env python3
"""
BluePilot Backend Route Segments
Segment enumeration and file utilities
"""

import os
import re
import logging
from functools import lru_cache

logger = logging.getLogger(__name__)


# Import constants from config
try:
    from bluepilot.backend.config import ROUTES_DIR
except ImportError:
    # Fallback for testing
    ROUTES_DIR = os.path.expanduser("~/comma_data/media/0/realdata")

# Import disk space utilities with fallback
try:
    from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
    from openpilot.system.loggerd.deleter import MIN_BYTES, MIN_PERCENT
    DISK_SPACE_UTILS_AVAILABLE = True
except ImportError:
    DISK_SPACE_UTILS_AVAILABLE = False
    logger.warning("Disk space utilities not available - will use fallback methods")
    MIN_BYTES = 5 * 1024 * 1024 * 1024  # 5 GB
    MIN_PERCENT = 10

# Import from parsing module
from .parsing import get_segment_number


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


def get_file_size(path):
    """Get size of file or directory"""
    if os.path.isfile(path):
        return os.path.getsize(path)

    total = 0
    try:
        for dirpath, dirnames, filenames in os.walk(path):
            for filename in filenames:
                filepath = os.path.join(dirpath, filename)
                if os.path.exists(filepath):
                    total += os.path.getsize(filepath)
    except:
        pass
    return total


def format_size(bytes_size):
    """Format bytes to human readable size"""
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if bytes_size < 1024.0:
            return f"{bytes_size:.1f} {unit}"
        bytes_size /= 1024.0
    return f"{bytes_size:.1f} PB"


def get_disk_space_info():
    """Get current disk space status with safety thresholds

    Returns dict with:
    - available_bytes: Free space in bytes
    - available_percent: Free space as percentage
    - total_bytes: Total disk space
    - is_low: True if below safety thresholds
    - can_record: True if enough space to record next route
    - warning_level: 'critical', 'low', 'medium', or 'ok'
    """
    try:
        if DISK_SPACE_UTILS_AVAILABLE:
            available_bytes = get_available_bytes(default=MIN_BYTES + 1)
            available_percent = get_available_percent(default=MIN_PERCENT + 1)
        else:
            # Fallback using standard library
            stat = os.statvfs(ROUTES_DIR)
            available_bytes = stat.f_bavail * stat.f_frsize
            total_bytes = stat.f_blocks * stat.f_frsize
            available_percent = (available_bytes / total_bytes * 100) if total_bytes > 0 else 0

        # Calculate total bytes for display
        stat = os.statvfs(ROUTES_DIR)
        total_bytes = stat.f_blocks * stat.f_frsize

        # Check if below safety thresholds
        is_low = available_bytes < MIN_BYTES or available_percent < MIN_PERCENT

        # Estimate space needed for next route
        # Typical route: ~60 segments * ~100MB/segment = ~6GB for 1 hour
        # Let's require 2x the minimum (10GB) to safely record next route
        SAFE_RECORDING_BYTES = MIN_BYTES * 2  # 10 GB
        can_record = available_bytes >= SAFE_RECORDING_BYTES

        # Determine warning level
        if available_bytes < MIN_BYTES or available_percent < MIN_PERCENT:
            warning_level = 'critical'  # At cleanup threshold
        elif available_bytes < SAFE_RECORDING_BYTES:
            warning_level = 'low'  # Below safe recording threshold
        elif available_bytes < (MIN_BYTES * 3):  # 15 GB
            warning_level = 'medium'  # Getting low
        else:
            warning_level = 'ok'

        return {
            'available_bytes': available_bytes,
            'available_percent': round(available_percent, 1),
            'total_bytes': total_bytes,
            'used_bytes': total_bytes - available_bytes,
            'is_low': is_low,
            'can_record': can_record,
            'warning_level': warning_level,
            'formatted': {
                'available': format_size(available_bytes),
                'total': format_size(total_bytes),
                'used': format_size(total_bytes - available_bytes)
            }
        }
    except Exception as e:
        logger.error(f"Error getting disk space info: {e}")
        # Return safe defaults on error
        return {
            'available_bytes': MIN_BYTES + 1,
            'available_percent': MIN_PERCENT + 1,
            'total_bytes': 0,
            'used_bytes': 0,
            'is_low': False,
            'can_record': True,
            'warning_level': 'unknown',
            'formatted': {
                'available': 'Unknown',
                'total': 'Unknown',
                'used': 'Unknown'
            }
        }
