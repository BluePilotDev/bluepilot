#!/usr/bin/env python3
"""
BluePilot Log Extraction Module

Extract and filter log messages from rlog/qlog files.
Handles decompression, parsing, and filtering of cloudlog messages.
"""

import os
import sys
import json
import logging

logger = logging.getLogger(__name__)


def extract_log_messages(log_path, search_query=None, level_filter=None, max_messages=500):
    """Extract cloudlog messages from qlog/rlog file

    Args:
        log_path: Path to qlog.zst or rlog.zst file
        search_query: Optional search string to filter messages (case-insensitive)
        level_filter: Optional level filter ('info', 'warning', 'error', 'all')
        max_messages: Maximum number of messages to return

    Returns:
        dict with:
            - messages: List of log messages with timestamps
            - total_count: Total number of log messages found
            - start_time: First message timestamp (seconds)
            - end_time: Last message timestamp (seconds)
    """
    try:
        import zstandard as zstd

        # Add cereal to path if needed
        sys.path.append(os.path.join(os.path.dirname(__file__), "../../.."))
        from cereal import log as capnp_log

        # Read and decompress log file
        with open(log_path, 'rb') as f:
            compressed_data = f.read()

        dctx = zstd.ZstdDecompressor()
        with dctx.stream_reader(compressed_data) as reader:
            decompressed_data = reader.read()

        # Parse capnp events
        events_reader = capnp_log.Event.read_multiple_bytes(decompressed_data)

        log_messages = []
        first_time = None
        last_time = None
        total_log_count = 0

        for event in events_reader:
            try:
                event_type = event.which()

                # Only process log messages
                if event_type not in ('logMessage', 'errorLogMessage'):
                    continue

                total_log_count += 1
                event_time = event.logMonoTime

                # Track first and last timestamps
                if first_time is None:
                    first_time = event_time
                last_time = event_time

                # Determine log level and message
                # Get message text from event
                if event_type == 'errorLogMessage':
                    message = event.errorLogMessage
                    level = 'warning'  # Default errorLogMessage to warning, not error
                else:
                    message = event.logMessage
                    level = 'info'  # Default logMessage to info

                # Try to parse as JSON to get structured log data
                # This applies to both logMessage and errorLogMessage
                try:
                    log_data = json.loads(message)

                    # Extract level from JSON structure
                    if isinstance(log_data, dict) and 'level' in log_data:
                        json_level = log_data['level'].upper()
                        if json_level in ('ERROR', 'CRITICAL', 'FATAL'):
                            level = 'error'
                        elif json_level in ('WARNING', 'WARN'):
                            level = 'warning'
                        elif json_level in ('INFO', 'DEBUG'):
                            level = 'info'
                        else:
                            # Unknown level, keep default based on event type
                            pass
                    # If JSON but no level field, keep default based on event type

                except (json.JSONDecodeError, ValueError, AttributeError, TypeError):
                    # Not JSON or JSON parsing failed
                    # Use keyword detection for non-JSON messages
                    # Don't search in JSON strings to avoid false positives
                    if not message.strip().startswith('{'):
                        message_upper = message.upper()
                        if any(keyword in message_upper for keyword in ['ERROR', 'FATAL', 'CRITICAL', 'EXCEPTION', 'FAILED']):
                            level = 'error'
                        elif any(keyword in message_upper for keyword in ['WARN', 'WARNING', 'CAUTION']):
                            level = 'warning'
                    # Otherwise keep default based on event type

                # Apply level filter
                if level_filter and level_filter != 'all':
                    if level != level_filter:
                        continue

                # Apply search filter
                if search_query:
                    if search_query.lower() not in message.lower():
                        continue

                # Stop if we've reached max messages
                if len(log_messages) >= max_messages:
                    break

                log_messages.append({
                    'timestamp': event_time / 1e9,  # Convert to seconds
                    'level': level,
                    'message': message
                })

            except Exception as e:
                logger.debug(f"Error parsing log event: {e}")
                continue

        return {
            'success': True,
            'messages': log_messages,
            'total_count': total_log_count,
            'returned_count': len(log_messages),
            'start_time': first_time / 1e9 if first_time else None,
            'end_time': last_time / 1e9 if last_time else None,
            'truncated': len(log_messages) >= max_messages
        }

    except ImportError as e:
        logger.exception("Missing required module for log parsing")
        return {
            'success': False,
            'error': f'Missing required module: {str(e)}. Try: pip install zstandard pycapnp',
            'messages': [],
            'total_count': 0,
            'returned_count': 0
        }
    except Exception as e:
        logger.exception(f"Error parsing log file {log_path}")
        import traceback
        return {
            'success': False,
            'error': str(e),
            'traceback': traceback.format_exc(),
            'messages': [],
            'total_count': 0,
            'returned_count': 0
        }
