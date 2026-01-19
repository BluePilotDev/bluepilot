#!/usr/bin/env python3
"""
BluePilot Backend File Operations
Atomic and safe file writing utilities
"""

import os
import json
import shutil
import tempfile
import logging

logger = logging.getLogger(__name__)


def atomic_write(filepath, content, mode='w'):
    """
    Write file atomically to prevent corruption from crashes.
    Writes to temp file first, then renames.

    Args:
        filepath: Target file path
        content: Content to write (str or bytes)
        mode: Write mode ('w' for text, 'wb' for binary)

    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Create directory if needed
        os.makedirs(os.path.dirname(filepath), exist_ok=True)

        # Write to temp file first
        temp_fd, temp_path = tempfile.mkstemp(
            dir=os.path.dirname(filepath),
            prefix='.tmp_',
            suffix=os.path.basename(filepath)
        )

        try:
            if 'b' in mode:
                os.write(temp_fd, content if isinstance(content, bytes) else content.encode())
            else:
                os.write(temp_fd, content.encode() if isinstance(content, str) else content)
            os.close(temp_fd)

            # Atomic rename (overwrites target on POSIX)
            os.replace(temp_path, filepath)
            return True

        except Exception as e:
            os.close(temp_fd)
            if os.path.exists(temp_path):
                os.remove(temp_path)
            raise e

    except Exception as e:
        logger.error(f"Atomic write failed for {filepath}: {e}")
        return False


def safe_json_write(filepath, data):
    """Write JSON file atomically"""
    try:
        json_str = json.dumps(data, indent=2)
        return atomic_write(filepath, json_str, mode='w')
    except Exception as e:
        logger.error(f"JSON write failed for {filepath}: {e}")
        return False


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
