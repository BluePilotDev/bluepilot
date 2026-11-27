#!/usr/bin/env python3
"""
BluePilot Backend Logging Handler
Custom logging handler for capturing errors and warnings for web display
"""

import logging


class ErrorBufferHandler(logging.Handler):
    """Custom logging handler that stores errors in ServerState for web retrieval"""

    def __init__(self, server_state):
        super().__init__(level=logging.WARNING)  # Capture WARNING and above
        self.server_state = server_state

    def emit(self, record):
        """Store log record in server state error buffer"""
        try:
            # Only log WARNING, ERROR, CRITICAL
            if record.levelno >= logging.WARNING:
                level = record.levelname
                message = record.getMessage()

                # Extract exception info if present
                exception_info = None
                if record.exc_info:
                    import traceback
                    exception_info = ''.join(traceback.format_exception(*record.exc_info))

                # Extract details from the log record
                details = {
                    'module': record.module,
                    'function': record.funcName,
                    'line': record.lineno,
                    'thread': record.thread,
                }

                self.server_state.log_error(level, message, details, exception_info)
        except Exception:
            # Don't let logging errors crash the server
            pass
