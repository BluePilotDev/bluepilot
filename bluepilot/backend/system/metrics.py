#!/usr/bin/env python3
"""
BluePilot Backend System Metrics
CPU, memory, disk, and temperature monitoring utilities
"""

import os
import shutil
import logging
from datetime import datetime

logger = logging.getLogger(__name__)


def get_system_metrics(cache_sizes_func=None, ffmpeg_state=None, max_ffmpeg=3):
    """Get comprehensive system metrics for monitoring

    Args:
        cache_sizes_func: Optional callable that returns cache size information
        ffmpeg_state: Optional object with get_ffmpeg_count() and get_ffmpeg_processes() methods
        max_ffmpeg: Maximum concurrent FFmpeg processes (default: 3)

    Returns:
        dict: System metrics including CPU, memory, disk, temperature
    """
    metrics = {
        'timestamp': datetime.now().isoformat(),
        'cpu': {},
        'memory': {},
        'disk': {},
        'temperature': {},
        'ffmpeg': {},
    }

    # CPU Usage
    try:
        # Check if /proc/stat exists (Linux)
        if os.path.exists('/proc/stat'):
            with open('/proc/loadavg') as f:
                load = f.read().split()
                metrics['cpu']['load_1min'] = float(load[0])
                metrics['cpu']['load_5min'] = float(load[1])
                metrics['cpu']['load_15min'] = float(load[2])

            # Check CPU core status
            online_cores = []
            for cpu in range(8):  # Check up to 8 cores
                online_path = f'/sys/devices/system/cpu/cpu{cpu}/online'
                if os.path.exists(online_path):
                    with open(online_path) as f:
                        if f.read().strip() == '1':
                            online_cores.append(cpu)
                elif cpu == 0:  # CPU0 is always online
                    online_cores.append(cpu)

            metrics['cpu']['online_cores'] = online_cores
            metrics['cpu']['core_count'] = len(online_cores)
    except Exception as e:
        logger.debug(f"Error reading CPU metrics: {e}")

    # Memory Usage
    try:
        if os.path.exists('/proc/meminfo'):
            with open('/proc/meminfo') as f:
                meminfo = {}
                for line in f:
                    parts = line.split(':')
                    if len(parts) == 2:
                        key = parts[0].strip()
                        value = parts[1].strip().split()[0]  # Get number, ignore unit
                        meminfo[key] = int(value) * 1024  # Convert KB to bytes

            total = meminfo.get('MemTotal', 0)
            available = meminfo.get('MemAvailable', 0)
            used = total - available
            percent = (used / total * 100) if total > 0 else 0

            metrics['memory']['total_bytes'] = total
            metrics['memory']['used_bytes'] = used
            metrics['memory']['available_bytes'] = available
            metrics['memory']['percent_used'] = round(percent, 1)
            metrics['memory']['total_gb'] = round(total / 1024 / 1024 / 1024, 2)
            metrics['memory']['available_gb'] = round(available / 1024 / 1024 / 1024, 2)
    except Exception as e:
        logger.debug(f"Error reading memory metrics: {e}")

    # Disk Usage
    try:
        for path_name, path in [('/data', '/data'), ('home', os.path.expanduser('~'))]:
            if os.path.exists(path):
                stat = shutil.disk_usage(path)
                percent = (stat.used / stat.total * 100) if stat.total > 0 else 0
                metrics['disk'][path_name] = {
                    'total_bytes': stat.total,
                    'used_bytes': stat.used,
                    'free_bytes': stat.free,
                    'percent_used': round(percent, 1),
                    'total_gb': round(stat.total / 1024 / 1024 / 1024, 2),
                    'free_gb': round(stat.free / 1024 / 1024 / 1024, 2),
                }
    except Exception as e:
        logger.debug(f"Error reading disk metrics: {e}")

    # Temperature (if available)
    try:
        thermal_zones = [
            '/sys/class/thermal/thermal_zone0/temp',
            '/sys/class/thermal/thermal_zone1/temp',
        ]
        temps = []
        for zone_path in thermal_zones:
            if os.path.exists(zone_path):
                with open(zone_path) as f:
                    temp_millic = int(f.read().strip())
                    temp_c = temp_millic / 1000
                    temps.append(temp_c)

        if temps:
            metrics['temperature']['celsius'] = round(max(temps), 1)
            metrics['temperature']['fahrenheit'] = round(max(temps) * 9/5 + 32, 1)
    except Exception as e:
        logger.debug(f"Error reading temperature: {e}")

    # FFmpeg process info (if ffmpeg_state provided)
    if ffmpeg_state is not None:
        try:
            metrics['ffmpeg']['active_processes'] = ffmpeg_state.get_ffmpeg_count()
            metrics['ffmpeg']['max_processes'] = max_ffmpeg
            metrics['ffmpeg']['process_details'] = ffmpeg_state.get_ffmpeg_processes()
        except Exception as e:
            logger.debug(f"Error reading FFmpeg state: {e}")
    else:
        metrics['ffmpeg']['active_processes'] = 0
        metrics['ffmpeg']['max_processes'] = max_ffmpeg
        metrics['ffmpeg']['process_details'] = {}

    # Cache sizes (if cache_sizes_func provided)
    if cache_sizes_func is not None:
        try:
            cache_sizes = cache_sizes_func()
            metrics['cache'] = {
                'remux_bytes': cache_sizes['remux_bytes'],
                'thumbnail_bytes': cache_sizes['thumbnail_bytes'],
                'metrics_bytes': cache_sizes['metrics_bytes'],
                'drive_stats_bytes': cache_sizes['drive_stats_bytes'],
                'fingerprint_bytes': cache_sizes['fingerprint_bytes'],
                'export_bytes': cache_sizes['export_bytes'],
                'total_bytes': cache_sizes['total_bytes'],
                'formatted': cache_sizes['formatted'],
                # Legacy fields for backwards compatibility
                'remux_cache_bytes': cache_sizes['remux_bytes'],
                'remux_cache_gb': round(cache_sizes['remux_bytes'] / 1024 / 1024 / 1024, 2),
            }
        except Exception as e:
            logger.debug(f"Error reading cache metrics: {e}")

    return metrics
