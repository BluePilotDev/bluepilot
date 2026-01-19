#!/usr/bin/env python3
"""
BluePilot Backend Route Parsing
Route name and datetime parsing utilities
"""

import os
import re
from datetime import datetime


def get_route_base_name(route_name):
    """Extract base name from route path (remove --segment suffix)
    Example: 2024-09-18--14-30-00--5 -> 2024-09-18--14-30-00
    """
    # Remove segment number (last --N)
    match = re.match(r'(.+)--\d+$', route_name)
    if match:
        return match.group(1)
    return route_name


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


def parse_route_datetime(route_base):
    """Parse route base name to extract datetime
    Example: 2024-09-18--14-30-00 -> datetime(2024, 9, 18, 14, 30, 0)
    Returns None for non-standard route names (e.g., dongle IDs)
    """
    try:
        # Split by --
        parts = route_base.split('--')
        if len(parts) >= 2:
            date_part = parts[0]  # 2024-09-18
            time_part = parts[1]  # 14-30-00

            # Check if date_part looks like a date (YYYY-MM-DD format)
            if len(date_part.split('-')) != 3:
                return None

            # Parse date
            year, month, day = map(int, date_part.split('-'))

            # Validate year is reasonable (not hex like 000000ad)
            if year < 2000 or year > 2100:
                return None

            # Parse time
            time_components = time_part.split('-')
            hour = int(time_components[0]) if len(time_components) > 0 else 0
            minute = int(time_components[1]) if len(time_components) > 1 else 0
            second = int(time_components[2]) if len(time_components) > 2 else 0

            return datetime(year, month, day, hour, minute, second)
    except (ValueError, TypeError):
        # Silently return None for non-standard route names
        return None

    return None


def format_time_12hr(dt):
    """Format datetime as 12-hour time with AM/PM
    Example: 14:30 -> 2:30 PM
    """
    return dt.strftime("%-I:%M %p" if os.name != 'nt' else "%#I:%M %p")


def format_display_date(dt):
    """Format date as: Thursday - September 17th, 2025"""
    day_name = dt.strftime("%A")
    month_name = dt.strftime("%B")
    day = dt.day
    year = dt.year

    # Add ordinal suffix
    if day % 10 == 1 and day != 11:
        suffix = "st"
    elif day % 10 == 2 and day != 12:
        suffix = "nd"
    elif day % 10 == 3 and day != 13:
        suffix = "rd"
    else:
        suffix = "th"

    return f"{day_name} - {month_name} {day}{suffix}, {year}"


def format_elapsed_time(dt):
    """Format elapsed time since route
    Example: 2 hours ago, 3 days ago, etc.
    """
    now = datetime.now()
    delta = now - dt

    seconds = delta.total_seconds()

    if seconds < 60:
        return "just now"
    elif seconds < 3600:
        minutes = int(seconds / 60)
        return f"{minutes} minute{'s' if minutes != 1 else ''} ago"
    elif seconds < 86400:
        hours = int(seconds / 3600)
        return f"{hours} hour{'s' if hours != 1 else ''} ago"
    elif seconds < 604800:  # 7 days
        days = int(seconds / 86400)
        return f"{days} day{'s' if days != 1 else ''} ago"
    else:
        weeks = int(seconds / 604800)
        return f"{weeks} week{'s' if weeks != 1 else ''} ago"
