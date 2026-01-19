#!/usr/bin/env python3
"""
BluePilot Backend Cache Cleanup
LRU cache cleanup logic with support for starred routes
"""

import os
import time
import logging

logger = logging.getLogger(__name__)

# Import configuration constants
from bluepilot.backend.config import REMUX_CACHE, ROUTES_DIR

# Import xattr for preserve marker support
try:
    from openpilot.system.loggerd.xattr_cache import getxattr
    from openpilot.system.loggerd.deleter import PRESERVE_ATTR_NAME, PRESERVE_ATTR_VALUE
    XATTR_AVAILABLE = True
except ImportError:
    XATTR_AVAILABLE = False
    logger.warning("xattr not available - preserve functionality will be limited")


# Cache configuration constants
MAX_CACHE_SIZE_GB = 5  # Maximum cache size in GB (adjustable - 5GB ~= 70 segments)
CACHE_CLEANUP_THRESHOLD = 0.9  # Cleanup when 90% full
CACHE_EXPIRATION_HOURS = 1  # Remove cached files older than 1 hour (unless starred)


def is_route_starred(route_base):
    """Check if a route is starred/preserved

    Checks for both:
    1. .star file (simple file-based marker)
    2. xattr preserve marker (openpilot system integration)

    Args:
        route_base: Route base name (e.g., "2024-01-01--12-00-00")

    Returns:
        bool: True if route is starred/preserved
    """
    # Check for .star file
    star_file = os.path.join(ROUTES_DIR, route_base, '.star')
    if os.path.exists(star_file):
        return True

    # Check for xattr preserve marker if available
    if XATTR_AVAILABLE:
        try:
            route_path = os.path.join(ROUTES_DIR, route_base)
            if os.path.exists(route_path):
                preserve_value = getxattr(route_path, PRESERVE_ATTR_NAME)
                if preserve_value == PRESERVE_ATTR_VALUE:
                    return True
        except (OSError, KeyError):
            # xattr not set or error reading it
            pass

    return False


def cleanup_old_cache():
    """Remove oldest cached files when cache is too large or expired

    Rules:
    1. Starred routes are never removed due to expiration
    2. Starred routes can be removed if cache size exceeds limit (LRU)
    3. Non-starred routes expire after CACHE_EXPIRATION_HOURS

    This implements a two-tier LRU cleanup:
    - First pass: Remove expired non-starred files
    - Second pass: If still over limit, remove oldest files (non-starred first, then starred)
    """
    # Import here to avoid circular dependency
    from bluepilot.backend.cache.metrics import get_cache_size

    max_bytes = MAX_CACHE_SIZE_GB * 1024 * 1024 * 1024
    current_size = get_cache_size()
    expiration_seconds = CACHE_EXPIRATION_HOURS * 60 * 60
    current_time = time.time()

    # Get all cache files
    cache_files = []
    expired_files = []
    starred_routes = set()

    try:
        if not os.path.exists(REMUX_CACHE):
            logger.info("Cache directory does not exist, nothing to cleanup")
            return

        for entry in os.scandir(REMUX_CACHE):
            if entry.is_file() and entry.name.endswith('.mp4'):
                stat = entry.stat()

                # Extract route base name from cache filename
                # Format: routebase_segment_camera.mp4
                parts = entry.name.rsplit('_', 2)
                if len(parts) >= 2:
                    route_base = parts[0]

                    # Check if route is starred
                    is_starred = is_route_starred(route_base)
                    if is_starred:
                        starred_routes.add(route_base)

                    # Check if file is expired (only for non-starred routes)
                    file_age = current_time - stat.st_mtime
                    if file_age > expiration_seconds and not is_starred:
                        expired_files.append((entry.path, stat.st_size, route_base))
                    else:
                        cache_files.append((entry.path, stat.st_atime, stat.st_size, is_starred))
    except OSError as e:
        logger.error(f"Error scanning cache: {e}", exc_info=True)
        return

    # Remove expired files first (excluding starred routes)
    if expired_files:
        logger.info(f"Removing {len(expired_files)} expired cache files (>{CACHE_EXPIRATION_HOURS}h old, non-starred)")
        for filepath, size, route_base in expired_files:
            try:
                os.remove(filepath)
                current_size -= size
                logger.info(f"Removed expired: {os.path.basename(filepath)}")
            except OSError as e:
                logger.error(f"Error removing expired file {filepath}: {e}", exc_info=True)

    # Check if we still need size-based cleanup
    if current_size < max_bytes * CACHE_CLEANUP_THRESHOLD:
        logger.info(f"Cache within limits: {current_size / 1024 / 1024 / 1024:.2f}GB / {MAX_CACHE_SIZE_GB}GB")
        if starred_routes:
            logger.info(f"Protected starred routes: {', '.join(sorted(starred_routes))}")
        return

    logger.info(f"Cache size cleanup triggered: {current_size / 1024 / 1024 / 1024:.2f}GB / {MAX_CACHE_SIZE_GB}GB")

    # Sort files: non-starred first (by access time), then starred (by access time)
    # This ensures starred routes are only removed as a last resort
    non_starred = [(f, a, s) for f, a, s, starred in cache_files if not starred]
    starred = [(f, a, s) for f, a, s, starred in cache_files if starred]

    non_starred.sort(key=lambda x: x[1])  # Sort by access time (oldest first)
    starred.sort(key=lambda x: x[1])

    # Combine: remove non-starred first, starred only if necessary
    sorted_files = non_starred + starred

    # Remove oldest files until we're under 80% of max
    target_size = max_bytes * 0.8
    for filepath, _, size in sorted_files:
        if current_size <= target_size:
            break
        try:
            os.remove(filepath)
            current_size -= size
            is_from_starred = filepath in [f for f, _, _ in starred]
            prefix = "starred" if is_from_starred else "old"
            logger.info(f"Removed {prefix} cached file: {os.path.basename(filepath)}")
        except OSError as e:
            logger.error(f"Error removing cache file {filepath}: {e}")

    logger.info(f"Cache cleanup complete: {current_size / 1024 / 1024 / 1024:.2f}GB remaining")
