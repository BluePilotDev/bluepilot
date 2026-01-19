#!/usr/bin/env python3
"""
BluePilot Backend Storage Preservation
Route preservation, deletion risk calculation, and disk space management
"""

import os
import time
import logging
from functools import lru_cache
from typing import Optional

logger = logging.getLogger(__name__)


# Import constants from config
try:
    from bluepilot.backend.config import ROUTES_DIR
except ImportError:
    # Fallback for testing
    ROUTES_DIR = os.path.expanduser("~/comma_data/media/0/realdata")

# Import xattr for preserve marker support
try:
    from openpilot.system.loggerd.xattr_cache import getxattr, setxattr
    from openpilot.system.loggerd.deleter import PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE, PRESERVE_COUNT, DELETE_LAST
    import xattr as xattr_module
    XATTR_AVAILABLE = True
except ImportError:
    XATTR_AVAILABLE = False
    PRESERVE_COUNT = 5  # Fallback
    DELETE_LAST = ['boot', 'crash']  # Fallback
    logger.warning("xattr not available - preserve functionality will be limited")

# Import disk space utilities
try:
    from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
    from openpilot.system.loggerd.deleter import MIN_BYTES, MIN_PERCENT
    from openpilot.system.loggerd.uploader import listdir_by_creation
    DISK_SPACE_UTILS_AVAILABLE = True
except ImportError:
    DISK_SPACE_UTILS_AVAILABLE = False
    logger.warning("Disk space utilities not available - will use fallback methods")
    MIN_BYTES = 5 * 1024 * 1024 * 1024  # 5 GB
    MIN_PERCENT = 10

    # Fallback implementation
    def listdir_by_creation(d: str):
        if not os.path.isdir(d):
            return []
        try:
            paths = [f for f in os.listdir(d) if os.path.isdir(os.path.join(d, f))]
            return sorted(paths)
        except OSError:
            return []

# Import route utilities
from bluepilot.backend.routes import (
    get_route_segments,
    format_size,
    get_disk_space_info,
)


def has_preserve_xattr_segment(segment_path: str) -> bool:
    """Check if a segment has the preserve xattr

    Args:
        segment_path: Absolute path to the segment directory

    Returns:
        bool: True if segment has preserve xattr, False otherwise
    """
    if not XATTR_AVAILABLE:
        return False
    try:
        return getxattr(segment_path, PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE
    except:
        return False


def get_preserved_segments_set(all_segment_paths: list[str]) -> set[str]:
    """Calculate which segments are protected from deletion

    Mimics the logic from system/loggerd/deleter.py:get_preserved_segments()
    Only the 5 most recent segments with xattr (+2 prior each) are protected

    Args:
        all_segment_paths: List of all segment names (relative to ROUTES_DIR)

    Returns:
        set of segment names (e.g., {"route--24", "route--25", ...})
    """
    if not XATTR_AVAILABLE:
        return set()

    preserved = set()

    # Filter to segments with xattr, reversed (newest first)
    segments_with_xattr = [
        s for s in reversed(all_segment_paths)
        if has_preserve_xattr_segment(os.path.join(ROUTES_DIR, s))
    ]

    # Take only first PRESERVE_COUNT (5) most recent
    for n, seg_path in enumerate(segments_with_xattr):
        if n >= PRESERVE_COUNT:
            break

        # Parse segment number
        date_str, _, seg_str = seg_path.rpartition("--")
        if not date_str:
            continue

        try:
            seg_num = int(seg_str)
        except ValueError:
            continue

        # Preserve segment and 2 prior
        for _seg_num in range(max(0, seg_num - 2), seg_num + 1):
            preserved.add(f"{date_str}--{_seg_num}")

    return preserved


def has_lock_file(segment_path: str) -> bool:
    """Check if segment has .lock file (currently being recorded)

    Args:
        segment_path: Segment name (relative to ROUTES_DIR)

    Returns:
        bool: True if segment has lock file, False otherwise
    """
    try:
        full_path = os.path.join(ROUTES_DIR, segment_path)
        return any(name.endswith(".lock") for name in os.listdir(full_path))
    except:
        return False


def calculate_deletion_queue():
    """Calculate the global deletion queue

    Returns dict with:
    - all_segments: list of all segment paths in deletion order
    - preserved_set: set of protected segment names
    - deletion_queue: list of deletable segments in order (excludes preserved, locked, boot/crash)
    - protected_count: number of protected segments
    - at_risk_count: number of segments that can be deleted
    - total_count: total number of segments
    """
    try:
        # Get all segments sorted by deletion order
        all_segments = listdir_by_creation(ROUTES_DIR)

        # Calculate preserved set
        preserved_set = get_preserved_segments_set(all_segments)

        # Build deletion queue (segments that can actually be deleted)
        deletion_queue = []
        for seg in all_segments:
            # Skip if in DELETE_LAST (boot/crash)
            if seg in DELETE_LAST:
                continue

            # Skip if preserved
            if seg in preserved_set:
                continue

            # Skip if has lock file (currently recording)
            if has_lock_file(seg):
                continue

            deletion_queue.append(seg)

        return {
            'all_segments': all_segments,
            'preserved_set': preserved_set,
            'deletion_queue': deletion_queue,
            'protected_count': len(preserved_set),
            'at_risk_count': len(deletion_queue),
            'total_count': len(all_segments)
        }

    except Exception as e:
        logger.error(f"Error calculating deletion queue: {e}", exc_info=True)
        return {
            'all_segments': [],
            'preserved_set': set(),
            'deletion_queue': [],
            'protected_count': 0,
            'at_risk_count': 0,
            'total_count': 0
        }


def calculate_route_deletion_risk(route_base: str, segments: list[dict], deletion_data: dict, disk_info: dict):
    """Calculate deletion risk for a specific route

    Args:
        route_base: Route base name
        segments: List of segment dicts with 'name' and 'path'
        deletion_data: Output from calculate_deletion_queue()
        disk_info: Output from get_disk_space_info()

    Returns dict with:
        - level: 'safe', 'low', 'medium', 'high', 'critical'
        - rank: Position in deletion queue (1 = next to delete, None if all protected)
        - totalInQueue: Total deletable segments
        - segmentsAtRisk: Number of segments that can be deleted
        - segmentsProtected: Number of protected segments
        - totalSegments: Total segments in route
        - firstAtRiskRank: Rank of first deletable segment
        - protectedSegmentNumbers: List of protected segment numbers
        - atRiskSegmentNumbers: List of at-risk segment numbers
        - isIncomplete: True if route is missing early segments
        - firstSegmentNumber: First segment number in route
        - lastSegmentNumber: Last segment number in route
    """
    preserved_set = deletion_data['preserved_set']
    deletion_queue = deletion_data['deletion_queue']

    # Categorize segments
    protected_segs = []
    at_risk_segs = []

    for seg in segments:
        seg_name = seg['name']
        seg_num = seg['segment']

        if seg_name in preserved_set:
            protected_segs.append(seg_num)
        elif not has_lock_file(seg_name):
            at_risk_segs.append(seg_num)

    # Find rank of first deletable segment
    first_at_risk_rank = None
    if at_risk_segs:
        # Find first at-risk segment in deletion queue
        for seg in segments:
            if seg['segment'] in at_risk_segs:
                try:
                    first_at_risk_rank = deletion_queue.index(seg['name']) + 1
                    break
                except ValueError:
                    continue

    # Calculate risk level
    if not at_risk_segs:
        risk_level = 'safe'
    elif first_at_risk_rank is None:
        risk_level = 'safe'
    else:
        # Calculate based on position in queue and available space
        space_until_deletion = disk_info['available_bytes'] - MIN_BYTES
        avg_segment_size = 100 * 1024 * 1024  # ~100MB
        segments_until_deletion = max(0, space_until_deletion / avg_segment_size)

        if first_at_risk_rank > segments_until_deletion * 2:
            risk_level = 'safe'
        elif first_at_risk_rank > segments_until_deletion:
            risk_level = 'low'
        elif first_at_risk_rank > segments_until_deletion / 2:
            risk_level = 'medium'
        elif first_at_risk_rank > 10:
            risk_level = 'high'
        else:
            risk_level = 'critical'

    # Check if route is incomplete (missing early segments)
    is_incomplete = False
    if segments:
        first_seg_num = min(seg['segment'] for seg in segments)
        if first_seg_num > 0:
            is_incomplete = True

    return {
        'level': risk_level,
        'rank': first_at_risk_rank,
        'totalInQueue': len(deletion_queue),
        'segmentsAtRisk': len(at_risk_segs),
        'segmentsProtected': len(protected_segs),
        'totalSegments': len(segments),
        'firstAtRiskRank': first_at_risk_rank,
        'protectedSegmentNumbers': sorted(protected_segs),
        'atRiskSegmentNumbers': sorted(at_risk_segs),
        'isIncomplete': is_incomplete,
        'firstSegmentNumber': min((seg['segment'] for seg in segments), default=0) if segments else 0,
        'lastSegmentNumber': max((seg['segment'] for seg in segments), default=0) if segments else 0,
    }


# Cache for deletion data with time-based expiry
_deletion_data_cache = None
_deletion_data_cache_time = 0
DELETION_DATA_CACHE_TTL = 60  # 60 seconds


def get_cached_deletion_data():
    """Cached deletion queue calculation with 60s TTL

    Returns:
        dict: Same as calculate_deletion_queue()
    """
    global _deletion_data_cache, _deletion_data_cache_time

    current_time = time.time()

    # Check if cache is still valid
    if _deletion_data_cache is not None and (current_time - _deletion_data_cache_time) < DELETION_DATA_CACHE_TTL:
        return _deletion_data_cache

    # Calculate new data and cache it
    _deletion_data_cache = calculate_deletion_queue()
    _deletion_data_cache_time = current_time

    return _deletion_data_cache


def check_route_preserve_status(route_base):
    """Check if a route has the preserve xattr set

    Returns True if ANY segment of the route has user.preserve xattr

    Args:
        route_base: Route base name (e.g., "2025-10-22--14-30-15")

    Returns:
        bool: True if route is preserved, False otherwise
    """
    if not XATTR_AVAILABLE:
        # Fallback to old .star file method
        star_file = os.path.join(ROUTES_DIR, route_base, '.star')
        return os.path.exists(star_file)

    try:
        segments = get_route_segments(route_base)
        for seg in segments:
            try:
                attr_value = getxattr(seg['path'], PRESERVE_ATTR_NAME)
                if attr_value == PRESERVE_ATTR_VALUE:
                    return True
            except Exception:
                continue
        return False
    except Exception as e:
        logger.debug(f"Error checking preserve status for {route_base}: {e}")
        return False


def set_route_preserve(route_base, preserve=True):
    """Set or remove preserve xattr on all segments of a route

    Args:
        route_base: Route base name (e.g., "2025-10-22--14-30-15")
        preserve: True to preserve, False to unpreserve

    Returns:
        dict with success status and message:
        - success: bool
        - message: str (on success)
        - error: str (on failure)
        - affected_segments: int (on success)
        - total_segments: int (on success)
        - disk_space: dict (if space check fails)
        - details: str (additional info)
        - hint: str (suggestion for user)
    """
    if not XATTR_AVAILABLE:
        return {
            'success': False,
            'error': 'xattr support not available'
        }

    try:
        segments = get_route_segments(route_base)
        if not segments:
            return {
                'success': False,
                'error': 'Route not found'
            }

        # Before preserving, check disk space
        if preserve:
            disk_info = get_disk_space_info()

            # Don't allow preserving if we can't safely record the next route
            if not disk_info['can_record']:
                return {
                    'success': False,
                    'error': 'Insufficient disk space to preserve route',
                    'details': f"Need {format_size(MIN_BYTES * 2)} free to safely record next drive. Currently: {disk_info['formatted']['available']}",
                    'disk_space': disk_info,
                    'hint': 'Delete some routes first to free up space'
                }

            # Warn if disk space is getting low
            if disk_info['warning_level'] in ('low', 'medium'):
                logger.warning(f"Preserving route {route_base} with {disk_info['warning_level']} disk space: {disk_info['formatted']['available']} free")

        # Set or remove preserve xattr on all segments
        affected_segments = 0
        for seg in segments:
            try:
                if preserve:
                    setxattr(seg['path'], PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE)
                    affected_segments += 1
                else:
                    # Remove xattr
                    try:
                        xattr_module.removexattr(seg['path'], PRESERVE_ATTR_NAME)
                        affected_segments += 1
                    except OSError:
                        # Attribute doesn't exist - that's fine
                        pass
            except Exception as e:
                logger.warning(f"Error setting preserve on {seg['name']}: {e}")

        # Clear caches so next scan picks up the change
        get_route_segments.cache_clear()
        clear_deletion_data_cache()

        action = 'preserved' if preserve else 'unpreserved'
        return {
            'success': True,
            'message': f'Route {action} successfully',
            'affected_segments': affected_segments,
            'total_segments': len(segments)
        }

    except Exception as e:
        logger.error(f"Error setting preserve on route {route_base}: {e}", exc_info=True)
        return {
            'success': False,
            'error': str(e)
        }


def clear_deletion_data_cache():
    """Clear the deletion data cache

    Call this when the deletion queue may have changed (e.g., after preserving/unpreserving routes)
    """
    global _deletion_data_cache, _deletion_data_cache_time
    _deletion_data_cache = None
    _deletion_data_cache_time = 0
