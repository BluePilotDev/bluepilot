"""
BluePilot Backend Utilities Module
Utility functions for cache, disk, network, power, and file operations
"""

# Import only modules that exist
from .power import (
    enable_performance_mode,
    restore_power_save,
    check_and_restore_power_save,
)
from .file_ops import (
    atomic_write,
    safe_json_write,
)

__all__ = [
    'enable_performance_mode',
    'restore_power_save',
    'check_and_restore_power_save',
    'atomic_write',
    'safe_json_write',
]
