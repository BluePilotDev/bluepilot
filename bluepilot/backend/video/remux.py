#!/usr/bin/env python3
"""
BluePilot Backend Video Remuxing and Prefetching

This module provides video segment remuxing from HEVC to MP4 format with
Safari-optimized fragmentation settings, as well as background prefetching
to improve playback performance.

Key Features:
- Remux HEVC segments to MP4 with proper fragmentation
- Background prefetching of upcoming segments
- Disk space validation before remuxing
- FFmpeg process management with concurrency limits
"""

import os
import subprocess
import threading
import logging
import re
import shutil

from bluepilot.backend.config import (
    REMUX_CACHE,
    ROUTES_DIR,
    FFMPEG_BINARY,
    MAX_CONCURRENT_FFMPEG,
)
from bluepilot.backend.video.ffmpeg import FFmpegProcess
from bluepilot.backend.utils.power import enable_performance_mode

logger = logging.getLogger(__name__)


# ============================================================================
# Helper Functions
# ============================================================================

def get_free_disk_space(path="/data"):
    """Get free disk space in bytes for the given path"""
    try:
        stat = shutil.disk_usage(path)
        return stat.free
    except Exception as e:
        logger.warning(f"Could not check free disk space: {e}")
        return None


def has_sufficient_disk_space(required_bytes, path="/data", min_free_gb=1):
    """Check if there's sufficient disk space for an operation

    Args:
        required_bytes: Estimated bytes needed for operation
        path: Path to check (usually /data)
        min_free_gb: Minimum GB to keep free after operation

    Returns:
        bool: True if sufficient space available
    """
    free_space = get_free_disk_space(path)
    if free_space is None:
        return True  # Can't check, assume OK

    min_free_bytes = min_free_gb * 1024 * 1024 * 1024
    required_total = required_bytes + min_free_bytes

    if free_space < required_total:
        free_gb = free_space / 1024 / 1024 / 1024
        required_gb = required_total / 1024 / 1024 / 1024
        logger.warning(f"Insufficient disk space: {free_gb:.2f}GB free, {required_gb:.2f}GB required")
        return False

    return True


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


def get_route_segments(route_base):
    """Get all segments for a route

    Note: This is a simplified version without caching. For production use,
    consider importing from the main server module if caching is needed.
    """
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


def get_ffmpeg_background_capacity(server_state, ffmpeg_reserved_for_playback=0):
    """Maximum number of background FFmpeg jobs allowed while keeping playback responsive.

    Args:
        server_state: ServerState instance
        ffmpeg_reserved_for_playback: Number of slots to reserve for playback (default: 0)

    Returns:
        int: Maximum background FFmpeg capacity
    """
    return max(1, MAX_CONCURRENT_FFMPEG - ffmpeg_reserved_for_playback)


# ============================================================================
# Segment Prefetch System
# ============================================================================

def remux_segment_to_cache(hevc_path, route_base, segment_num, camera, server_state):
    """
    Remux a single segment to cache without streaming to client.
    Used for background prefetching.

    Args:
        hevc_path: Path to source HEVC file
        route_base: Route base name
        segment_num: Segment number
        camera: Camera type ('front', 'wide', 'driver')
        server_state: ServerState instance for process tracking

    Returns:
        bool: True if successful, False otherwise
    """
    if not os.path.exists(hevc_path):
        return False

    cache_filename = f"{route_base}_{segment_num}_{camera}.mp4"
    cache_path = os.path.join(REMUX_CACHE, cache_filename)

    # Check if already cached
    if os.path.exists(cache_path):
        cache_mtime = os.path.getmtime(cache_path)
        source_mtime = os.path.getmtime(hevc_path)
        if cache_mtime >= source_mtime:
            logger.debug(f"Prefetch: {cache_filename} already cached")
            return True

    # Check disk space
    source_size = os.path.getsize(hevc_path)
    estimated_output_size = source_size * 2
    cache_dir = "/data" if os.path.exists("/data") else os.path.expanduser("~")

    if not has_sufficient_disk_space(estimated_output_size, cache_dir, min_free_gb=0.5):
        logger.debug(f"Prefetch: Insufficient disk space for {cache_filename}")
        return False

    # Remux to cache using FFmpeg
    logger.info(f"Prefetch: Remuxing {cache_filename} to cache")

    route_info = f"prefetch:{route_base}:{segment_num}:{camera}"
    try:
        # Check if ffmpeg is available
        if FFMPEG_BINARY is None:
            logger.error("FFmpeg not available, cannot prefetch video")
            return False

        with FFmpegProcess(route_info, server_state, max_concurrent=MAX_CONCURRENT_FFMPEG) as ffmpeg_mgr:
            # Use same Safari-optimized flags as main remuxing
            cmd = [
                FFMPEG_BINARY,
                '-loglevel', 'error',
                '-f', 'hevc',
                '-r', '20',
                '-i', hevc_path,
                '-c', 'copy',  # Copy all streams (video + audio if available)
                '-movflags', 'frag_every_frame+empty_moov+default_base_moof+omit_tfhd_offset',
                '-frag_duration', '2000000',  # 2 seconds per fragment
                '-fflags', '+genpts+igndts',
                '-avoid_negative_ts', 'make_zero',
                '-start_at_zero',
                '-vsync', 'cfr',
                '-video_track_timescale', '90000',
                '-max_muxing_queue_size', '1024',
                '-f', 'mp4',
                cache_path
            ]

            process = ffmpeg_mgr.start(cmd)

            # Wait for completion (background process)
            try:
                process.wait(timeout=60)  # 60 second timeout for prefetch
            except subprocess.TimeoutExpired:
                logger.warning(f"Prefetch timeout for {cache_filename}")
                return False

            if process.returncode == 0:
                logger.info(f"Prefetch: Successfully cached {cache_filename}")
                return True
            else:
                stderr = process.stderr.read().decode('utf-8', errors='ignore')
                logger.warning(f"Prefetch: FFmpeg failed for {cache_filename}: {stderr[:200]}")
                # Clean up incomplete file
                if os.path.exists(cache_path):
                    try:
                        os.remove(cache_path)
                    except OSError:
                        pass
                return False

    except RuntimeError as e:
        # Too many concurrent processes - this is expected, just skip prefetch
        logger.debug(f"Prefetch: Skipping {cache_filename} - {e}")
        return False
    except Exception as e:
        logger.warning(f"Prefetch: Error remuxing {cache_filename}: {e}")
        return False


def _prefetch_worker(route_base, current_segment, camera, server_state, segments_to_prefetch=2):
    """
    Background worker to prefetch upcoming segments.
    Called in a daemon thread.

    Args:
        route_base: Route base name
        current_segment: Current segment number being played
        camera: Camera type ('front', 'wide', 'driver')
        server_state: ServerState instance for process tracking
        segments_to_prefetch: Number of segments to prefetch ahead (default: 2)
    """
    logger.debug(f"Prefetch worker starting for {route_base} segment {current_segment} camera {camera}")

    try:
        # Get all segments for this route
        segments = get_route_segments(route_base)
        if not segments:
            return

        # Find segments after current
        future_segments = [s for s in segments if s['segment'] > current_segment]
        future_segments.sort(key=lambda x: x['segment'])

        # Prefetch next N segments
        prefetched = 0
        for seg in future_segments[:segments_to_prefetch]:
            segment_num = seg['segment']
            segment_path = seg['path']

            # Map camera to filename
            camera_files = {
                'front': 'fcamera.hevc',
                'wide': 'ecamera.hevc',
                'driver': 'dcamera.hevc'
            }

            if camera not in camera_files:
                continue

            hevc_path = os.path.join(segment_path, camera_files[camera])

            if remux_segment_to_cache(hevc_path, route_base, segment_num, camera, server_state):
                prefetched += 1
            else:
                # If one fails (e.g., too many processes), stop trying
                break

        logger.info(f"Prefetch: Completed {prefetched}/{segments_to_prefetch} segments for {route_base}:{camera}")

    except Exception as e:
        logger.warning(f"Prefetch worker error: {e}", exc_info=True)


def prefetch_next_segments(route_base, current_segment, camera, server_state, count=2):
    """
    Trigger background prefetch of upcoming segments.
    Non-blocking - spawns a daemon thread.

    Args:
        route_base: Route base name
        current_segment: Current segment number being played
        camera: Camera type ('front', 'wide', 'driver')
        server_state: ServerState instance for process tracking
        count: Number of segments to prefetch (default: 2)
    """
    current_ffmpeg = server_state.get_ffmpeg_count()
    if current_ffmpeg >= get_ffmpeg_background_capacity(server_state):
        logger.debug(
            f"Prefetch: Skipping for {route_base}:{camera} (active FFmpeg processes: {current_ffmpeg})"
        )
        return

    thread = threading.Thread(
        target=_prefetch_worker,
        args=(route_base, current_segment, camera, server_state, count),
        daemon=True,
        name=f"prefetch-{route_base}-{current_segment}-{camera}"
    )
    thread.start()
    logger.debug(f"Triggered prefetch for {route_base} segment {current_segment}+{count} camera {camera}")
