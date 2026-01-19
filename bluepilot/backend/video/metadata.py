#!/usr/bin/env python3
"""
BluePilot Video Metadata Module

Functions for retrieving video and log file metadata, including:
- Video duration extraction using FFprobe
- Video file discovery in segments
- Log file discovery in segments
"""

import os
import subprocess
import logging
from functools import lru_cache

from bluepilot.backend.config import (
    FFPROBE_BINARY,
    REMUX_CACHE,
)

logger = logging.getLogger(__name__)


def get_video_duration_from_cache(route_base, segment_num, camera):
    """
    Get duration from cached remuxed MP4 file if available.
    Returns None if cache doesn't exist.
    """
    cache_filename = f"{route_base}_{segment_num}_{camera}.mp4"
    cache_path = os.path.join(REMUX_CACHE, cache_filename)

    if os.path.exists(cache_path) and FFPROBE_BINARY:
        try:
            cmd = [
                FFPROBE_BINARY,
                '-v', 'error',
                '-show_entries', 'format=duration',
                '-of', 'default=noprint_wrappers=1:nokey=1',
                cache_path
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=3)
            if result.returncode == 0 and result.stdout.strip():
                duration = float(result.stdout.strip())
                return round(duration, 3)
        except Exception as e:
            logger.debug(f"Error getting cached duration for {cache_path}: {e}")

    return None


@lru_cache(maxsize=256)
def get_video_duration(filepath):
    """
    Get accurate video duration using ffprobe.
    For raw HEVC files, analyzes the stream to calculate duration.
    Falls back to 60.0 seconds if ffprobe is unavailable or fails.
    Results are cached to avoid repeated ffprobe calls.
    """
    if not FFPROBE_BINARY or not os.path.exists(filepath):
        return 60.0  # Default fallback

    try:
        # Raw HEVC segments always record ~60 seconds at 20 fps; avoid heavy ffprobe
        if filepath.endswith('.hevc'):
            return 60.0
        else:
            # For container formats (mp4, ts), use standard duration query
            cmd = [
                FFPROBE_BINARY,
                '-v', 'error',
                '-show_entries', 'format=duration',
                '-of', 'default=noprint_wrappers=1:nokey=1',
                filepath
            ]
            try:
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            except subprocess.TimeoutExpired:
                logger.debug(f"ffprobe timed out while probing {filepath}, using default duration")
                return 60.0
            if result.returncode == 0 and result.stdout.strip():
                duration = float(result.stdout.strip())
                return round(duration, 3)
    except Exception as e:
        logger.debug(f"Error getting duration for {filepath}: {e}")

    return 60.0  # Fallback to 60 seconds


def get_video_files(segment_path):
    """Get available video files in segment"""
    videos = {}
    video_types = {
        'fcamera.hevc': 'front',
        'ecamera.hevc': 'wide',
        'dcamera.hevc': 'driver',
        'qcamera.ts': 'lq'
    }

    for filename, camera_type in video_types.items():
        filepath = os.path.join(segment_path, filename)
        if os.path.exists(filepath):
            videos[camera_type] = {
                'filename': filename,
                'path': filepath,
                'size': os.path.getsize(filepath)
            }

    return videos


def get_log_files(segment_path):
    """Get available log files in segment"""
    logs = {}
    log_types = {
        'qlog.zst': 'qlog',
        'rlog.zst': 'rlog'
    }

    for filename, log_type in log_types.items():
        filepath = os.path.join(segment_path, filename)
        if os.path.exists(filepath):
            logs[log_type] = {
                'filename': filename,
                'path': filepath,
                'size': os.path.getsize(filepath)
            }

    return logs
