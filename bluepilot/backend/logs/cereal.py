#!/usr/bin/env python3
"""
BluePilot Cereal Message Processing Module

Extract and serialize Cereal messages from logs.
Handles Cereal message parsing and conversion to JSON-serializable formats.
"""

import os
import sys
import logging

logger = logging.getLogger(__name__)


def extract_cereal_messages(log_path, message_type, max_messages=1000):
    """Extract specific cereal message type from qlog/rlog file

    Args:
        log_path: Path to qlog.zst or rlog.zst file
        message_type: Cereal message type to extract (e.g., 'carState', 'controlsState')
        max_messages: Maximum number of messages to return

    Returns:
        dict with:
            - messages: List of cereal messages with timestamps and data
            - message_type: The requested message type
            - total_count: Total number of messages found
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

        cereal_messages = []
        first_time = None
        last_time = None
        total_count = 0

        for event in events_reader:
            try:
                event_type = event.which()

                # Only process the requested message type
                if event_type != message_type:
                    continue

                total_count += 1
                event_time = event.logMonoTime

                # Track first and last timestamps
                if first_time is None:
                    first_time = event_time
                last_time = event_time

                # Stop if we've reached max messages
                if len(cereal_messages) >= max_messages:
                    break

                # Extract message data
                try:
                    event_obj = getattr(event, event_type)
                    message_data = serialize_cereal_object(event_obj)

                    cereal_messages.append({
                        'timestamp': event_time / 1e9,  # Convert to seconds
                        'data': message_data
                    })

                except Exception as e:
                    logger.debug(f"Could not serialize message {event_type}: {e}")
                    continue

            except Exception as e:
                logger.debug(f"Error parsing cereal event: {e}")
                continue

        return {
            'success': True,
            'messages': cereal_messages,
            'message_type': message_type,
            'total_count': total_count,
            'returned_count': len(cereal_messages),
            'start_time': first_time / 1e9 if first_time else None,
            'end_time': last_time / 1e9 if last_time else None,
            'truncated': len(cereal_messages) >= max_messages
        }

    except ImportError as e:
        logger.exception("Missing required module for cereal parsing")
        return {
            'success': False,
            'error': f'Missing required module: {str(e)}. Try: pip install zstandard pycapnp',
            'messages': [],
            'message_type': message_type,
            'total_count': 0,
            'returned_count': 0
        }
    except Exception as e:
        logger.exception(f"Error parsing cereal file {log_path}")
        import traceback
        return {
            'success': False,
            'error': str(e),
            'traceback': traceback.format_exc(),
            'messages': [],
            'message_type': message_type,
            'total_count': 0,
            'returned_count': 0
        }


def serialize_cereal_object(obj, depth=0):
    """Recursively serialize a cereal object to a dict

    Args:
        obj: The cereal object to serialize
        depth: Current recursion depth (prevents infinite loops)

    Returns:
        JSON-serializable representation of the object
    """
    # Prevent infinite recursion
    if depth > 10:
        return str(obj)

    if obj is None:
        return None

    # Handle primitives
    if isinstance(obj, (bool, int, float, str, bytes)):
        return obj

    # Handle lists/tuples
    if isinstance(obj, (list, tuple)):
        return [serialize_cereal_object(item, depth + 1) for item in obj]

    # Try to serialize capnp struct by extracting fields from to_dict() if available
    try:
        # Many capnp structs have a to_dict() method
        if hasattr(obj, 'to_dict') and callable(obj.to_dict):
            return obj.to_dict()
    except Exception:
        pass

    # Handle capnp structs manually
    result = {}
    try:
        # Check if it has a schema (capnp struct)
        if hasattr(obj, 'schema'):
            schema = obj.schema

            # Get all non-union fields
            try:
                for field in schema.non_union_fields:
                    try:
                        value = getattr(obj, field.name)
                        result[field.name] = serialize_cereal_object(value, depth + 1)
                    except Exception as e:
                        logger.debug(f"Could not serialize field {field.name}: {e}")
                        result[field.name] = None
            except Exception as e:
                logger.debug(f"Error iterating non_union_fields: {e}")

            # Handle union fields
            try:
                which_field = obj.which()
                if which_field:
                    value = getattr(obj, which_field)
                    result[which_field] = serialize_cereal_object(value, depth + 1)
            except Exception:
                pass

            # If we got fields, return them
            if result:
                return result

        # Try alternative approach: use dir() to find all attributes
        # This is a fallback for capnp objects that don't have schema.non_union_fields
        if hasattr(obj, '__dir__'):
            for attr_name in dir(obj):
                # Skip private/magic methods and common capnp internals
                if attr_name.startswith('_') or attr_name in ('schema', 'which', 'to_bytes', 'from_bytes', 'as_builder', 'total_size'):
                    continue

                try:
                    attr_value = getattr(obj, attr_name)
                    # Skip methods
                    if callable(attr_value):
                        continue
                    result[attr_name] = serialize_cereal_object(attr_value, depth + 1)
                except Exception:
                    continue

            if result:
                return result

    except Exception as e:
        logger.debug(f"Error serializing capnp object: {e}")

    # Last resort fallback
    try:
        if hasattr(obj, '__dict__'):
            return {k: serialize_cereal_object(v, depth + 1) for k, v in obj.__dict__.items()}
    except Exception:
        pass

    # Absolute last resort: return string representation
    return str(obj)
