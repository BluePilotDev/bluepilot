#!/usr/bin/env python3
"""
BluePilot Backend Power Management
CPU power management for performance and power saving
"""

import os
import time
import logging

logger = logging.getLogger(__name__)

# Power management state
last_activity_time = None
IDLE_TIMEOUT_SECONDS = 300  # 5 minutes of no remuxing = idle


def enable_performance_mode():
    """Enable all CPU cores for FFmpeg remuxing"""
    global last_activity_time

    last_activity_time = time.time()  # Update activity time

    try:
        # Enable CPU cores 4-7 (big cores on Snapdragon 845)
        for cpu in range(4, 8):
            online_path = f'/sys/devices/system/cpu/cpu{cpu}/online'
            if os.path.exists(online_path):
                with open(online_path, 'w') as f:
                    f.write('1')
                logger.info(f"Enabled CPU{cpu}")

        logger.info("Performance mode enabled for video remuxing")
    except (OSError, PermissionError) as e:
        logger.warning(f"Could not enable performance mode: {e}")
        logger.warning("This may slow down video remuxing. Run as root or with proper permissions.")


def restore_power_save():
    """Disable big cores to save power when idle"""
    try:
        # Disable CPU cores 4-7 (big cores)
        disabled_count = 0
        for cpu in range(4, 8):
            online_path = f'/sys/devices/system/cpu/cpu{cpu}/online'
            if os.path.exists(online_path):
                with open(online_path, 'w') as f:
                    f.write('0')
                disabled_count += 1

        if disabled_count > 0:
            logger.info(f"Power save restored - disabled {disabled_count} big cores")
    except (OSError, PermissionError) as e:
        logger.debug(f"Could not restore power save: {e}")


def check_and_restore_power_save(is_onroad_func):
    """Check if idle and restore power save mode (only when offroad)

    Args:
        is_onroad_func: Function to check if device is onroad
    """
    global last_activity_time

    if last_activity_time is None:
        return

    # CRITICAL: Never disable cores when onroad or about to go onroad
    # Check onroad status BEFORE checking idle time
    if is_onroad_func():
        # Device is onroad, keep cores enabled
        last_activity_time = None  # Reset so we don't keep trying
        return

    idle_time = time.time() - last_activity_time
    if idle_time > IDLE_TIMEOUT_SECONDS:
        restore_power_save()
        last_activity_time = None  # Reset so we don't keep trying
