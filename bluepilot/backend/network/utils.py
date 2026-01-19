#!/usr/bin/env python3
"""
BluePilot Backend Network Utilities
Network interface detection and connection type management
"""

import logging

from bluepilot.backend.utils.params_fallback import get_params_with_defaults

logger = logging.getLogger(__name__)

params = get_params_with_defaults({
    "IsOnRoad": False,
    "BPPortalPort": "8088",
    "BPPortalEnabled": True,
})


def is_onroad():
    """Check if vehicle is currently driving"""
    try:
        return params.get_bool("IsOnRoad")
    except:
        return False


def should_server_run():
    """Check if server should be running (always runs when enabled, rate-limited onroad)"""
    try:
        return params.get_bool("BPPortalEnabled")
    except:
        return True  # Default to running if we can't check


def get_wifi_ip():
    """Get WiFi interface IP address (first wlan interface found)"""
    try:
        import netifaces
        for iface in netifaces.interfaces():
            if iface.startswith('wlan'):
                addrs = netifaces.ifaddresses(iface)
                if netifaces.AF_INET in addrs:
                    for addr in addrs[netifaces.AF_INET]:
                        ip = addr.get('addr')
                        if ip and not ip.startswith('127.'):
                            return ip
    except ImportError:
        # Fallback without netifaces
        import subprocess
        try:
            result = subprocess.run(['ip', 'addr', 'show', 'wlan0'],
                                    capture_output=True, text=True, timeout=2)
            for line in result.stdout.split('\n'):
                if 'inet ' in line:
                    ip = line.strip().split()[1].split('/')[0]
                    return ip
        except:
            pass
    return None


def get_all_wifi_ips():
    """Get all WiFi interface IP addresses (for hotspot support)"""
    ips = []
    try:
        import netifaces
        for iface in netifaces.interfaces():
            # Include wlan interfaces only (not cellular)
            if iface.startswith('wlan'):
                addrs = netifaces.ifaddresses(iface)
                if netifaces.AF_INET in addrs:
                    for addr in addrs[netifaces.AF_INET]:
                        ip = addr.get('addr')
                        if ip and not ip.startswith('127.'):
                            ips.append((iface, ip))
    except ImportError:
        # Fallback: just get wlan0
        wifi_ip = get_wifi_ip()
        if wifi_ip:
            ips.append(('wlan0', wifi_ip))
    return ips


def get_connection_type():
    """Determine current network connection type"""
    try:
        import subprocess
        # Check which interface is being used for default route
        result = subprocess.run(['ip', 'route', 'get', '8.8.8.8'],
                                capture_output=True, text=True, timeout=2)
        output = result.stdout.lower()

        if 'wlan' in output:
            return 'wifi'
        elif 'rmnet' in output or 'ccmni' in output:
            return 'cellular'
        elif 'eth' in output:
            return 'ethernet'
        else:
            return 'unknown'
    except:
        return 'unknown'
