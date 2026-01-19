#!/usr/bin/env python3
"""
BluePilot Logs Module

Log extraction and Cereal message processing for BluePilot backend.
Handles parsing, filtering, and serialization of log files.
"""

from bluepilot.backend.logs.extraction import extract_log_messages
from bluepilot.backend.logs.cereal import extract_cereal_messages, serialize_cereal_object
from bluepilot.backend.logs.manager_logs import (
    parse_manager_log_line,
    read_recent_manager_logs,
)

__all__ = [
    'extract_log_messages',
    'extract_cereal_messages',
    'serialize_cereal_object',
    'parse_manager_log_line',
    'read_recent_manager_logs',
]
