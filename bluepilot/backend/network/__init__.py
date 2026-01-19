"""
BluePilot Backend Network Module
Network utilities and connection management
"""

from .utils import (
    is_onroad,
    should_server_run,
    get_wifi_ip,
    get_all_wifi_ips,
    get_connection_type,
)

__all__ = [
    'is_onroad',
    'should_server_run',
    'get_wifi_ip',
    'get_all_wifi_ips',
    'get_connection_type',
]
