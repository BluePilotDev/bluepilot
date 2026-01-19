"""
BluePilot Backend Storage Module
Route preservation, deletion risk calculation, and disk space management
"""

from .preservation import (
    has_preserve_xattr_segment,
    get_preserved_segments_set,
    has_lock_file,
    calculate_deletion_queue,
    calculate_route_deletion_risk,
    get_cached_deletion_data,
    check_route_preserve_status,
    set_route_preserve,
    clear_deletion_data_cache,
)

__all__ = [
    'has_preserve_xattr_segment',
    'get_preserved_segments_set',
    'has_lock_file',
    'calculate_deletion_queue',
    'calculate_route_deletion_risk',
    'get_cached_deletion_data',
    'check_route_preserve_status',
    'set_route_preserve',
    'clear_deletion_data_cache',
]
