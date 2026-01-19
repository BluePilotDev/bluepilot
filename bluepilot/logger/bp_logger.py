#!/usr/bin/env python3
import os
import queue
import threading
import logging
import sys
import traceback
import platform
from logging.handlers import RotatingFileHandler
from enum import Enum
from typing import Any
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
# from openpilot.system.hardware.hw import Paths


# Define log levels as an Enum for type safety
class LogLevel(Enum):
  DEBUG = "debug"
  INFO = "info"
  WARNING = "warning"
  ERROR = "error"
  CRITICAL = "critical"


# Global message queue
_log_queue = queue.Queue()
# Global logger instance
_logger = None
# Flag to control the worker thread
_running = False
# Worker thread reference
_worker_thread = None
# Lock for thread safety
_lock = threading.Lock()


def get_default_log_dir():
  """
  Get appropriate log directory based on environment/platform
  """
  # Check if running on a Comma device (has /data directory)
  if os.path.exists("/data") and os.access("/data", os.W_OK):
    return "/data/logs/bp_logger"

  # On macOS, use ~/Library/Logs
  if platform.system() == "Darwin":
    return os.path.expanduser("~/Library/Logs/bluepilot")

  # On Linux (non-Comma), use ~/.local/share/bluepilot/logs
  if platform.system() == "Linux":
    return os.path.expanduser("~/.local/share/bluepilot/logs")

  # On Windows
  if platform.system() == "Windows":
    return os.path.join(os.environ.get("APPDATA", os.path.expanduser("~")), "bluepilot", "logs")

  # Fallback to current directory
  return os.path.join(os.getcwd(), "logs", "bluepilot")


# Default configuration
DEFAULT_CONFIG = {
  "max_file_size_mb": 10,
  "backup_count": 10,
  "log_dir": get_default_log_dir(),  # Dynamic log directory
  "log_filename": "bluepilot.log",
  "format": "%(levelname)s [%(name)s]: %(message)s",
  "date_format": "%Y-%m-%d %H:%M:%S",
  "enabled": True,
}


def exception_handler(exc_type, exc_value, exc_traceback):
  """Global exception handler to log unhandled exceptions."""
  # Don't log KeyboardInterrupt exceptions
  if issubclass(exc_type, KeyboardInterrupt):
    sys.__excepthook__(exc_type, exc_value, exc_traceback)
    return

  # Format the exception traceback as a string
  exc_info = (exc_type, exc_value, exc_traceback)
  tb_lines = traceback.format_exception(*exc_info)
  tb_text = ''.join(tb_lines)

  # Log the exception with critical level
  critical(f"Unhandled exception:\n{tb_text}", console_output=True)

  # Call the original exception hook to maintain default behavior
  sys.__excepthook__(exc_type, exc_value, exc_traceback)


def set_exception_handler():
  """Set the global exception handler."""
  sys.excepthook = exception_handler


def _ensure_log_directory(log_dir: str) -> bool:
  """
  Ensure the log directory exists.
  Returns True if successful, False otherwise.
  """
  try:
    Path(log_dir).mkdir(parents=True, exist_ok=True)
    return True
  except (OSError, PermissionError) as e:
    print(f"Warning: Could not create log directory {log_dir}: {e}")
    return False


def initialize_logger(config: dict[str, Any] | None = None) -> logging.Logger:
  global _logger, _running, _worker_thread

  with _lock:
    if _logger is not None:
      return _logger

    # Set the exception handler
    set_exception_handler()

    # Merge custom config with defaults
    full_config = DEFAULT_CONFIG.copy()
    if config:
      full_config.update(config)

    # Ensure log directory exists, fallback to a temp dir if it fails
    if not _ensure_log_directory(full_config["log_dir"]):
      import tempfile

      fallback_dir = os.path.join(tempfile.gettempdir(), "bluepilot", "logs")
      print(f"Falling back to temporary directory: {fallback_dir}")
      _ensure_log_directory(fallback_dir)
      full_config["log_dir"] = fallback_dir

    # Create log file path
    log_file = os.path.join(full_config["log_dir"], full_config["log_filename"])

    # Set up logger
    logger = logging.getLogger("bluepilot")
    logger.setLevel(logging.DEBUG)

    # Set up formatter
    formatter = logging.Formatter(fmt=full_config["format"], datefmt=full_config["date_format"])

    try:
      # Set up rotating file handler
      file_handler = RotatingFileHandler(log_file, maxBytes=full_config["max_file_size_mb"] * 1024 * 1024, backupCount=full_config["backup_count"])
      file_handler.setFormatter(formatter)
      logger.addHandler(file_handler)
    except (OSError, PermissionError) as e:
      # If file handler fails, add a console handler as fallback
      print(f"Warning: Could not create log file at {log_file}: {e}")
      console_handler = logging.StreamHandler()
      console_handler.setFormatter(formatter)
      logger.addHandler(console_handler)

    # Store global reference
    _logger = logger

    # Start worker thread if enabled
    if full_config["enabled"] and not _running:
      _running = True
      _worker_thread = threading.Thread(target=_process_log_queue, daemon=True, name="BluepilotLoggerThread")
      _worker_thread.start()

    return logger


def _process_log_queue() -> None:
  """Worker thread function to process the log queue."""
  global _running, _logger

  # Ensure logger is initialized
  if _logger is None:
    initialize_logger()

  while _running:
    try:
      # Get log message from queue with timeout
      level, message, console_output, args = _log_queue.get(timeout=0.5)

      # Log the message
      if level == LogLevel.DEBUG:
        _logger.debug(message, *args)
      elif level == LogLevel.INFO:
        _logger.info(message, *args)
      elif level == LogLevel.WARNING:
        _logger.warning(message, *args)
      elif level == LogLevel.ERROR:
        _logger.error(message, *args)
      elif level == LogLevel.CRITICAL:
        _logger.critical(message, *args)

      # Mark task as done
      _log_queue.task_done()
    except queue.Empty:
      # Timeout, just continue
      continue
    except Exception as e:
      # Log errors to cloudlog
      cloudlog.exception(f"BluepilotLogger error processing log message: {e}")
      try:
        _log_queue.task_done()
      except (ValueError, RuntimeError):
        pass


def shutdown() -> None:
  """Shutdown the logger worker thread."""
  global _running, _worker_thread

  with _lock:
    if _running:
      _running = False
      if _worker_thread and _worker_thread.is_alive():
        _worker_thread.join(timeout=1.0)

      # Flush remaining logs
      while not _log_queue.empty():
        try:
          level, message, console_output, args = _log_queue.get(block=False)
          if _logger:
            if level == LogLevel.DEBUG:
              _logger.debug(message, *args)
            elif level == LogLevel.INFO:
              _logger.info(message, *args)
            elif level == LogLevel.WARNING:
              _logger.warning(message, *args)
            elif level == LogLevel.ERROR:
              _logger.error(message, *args)
            elif level == LogLevel.CRITICAL:
              _logger.critical(message, *args)
          _log_queue.task_done()
        except queue.Empty:
          break


def log(level: LogLevel, message: str, *args, console_output: bool = False) -> None:
  try:
    # Ensure logger is initialized
    if not _running:
      initialize_logger()

    # Always print to console if console_output is True
    if console_output:
      print(message)

    # For error and critical messages, process immediately
    if level in [LogLevel.ERROR, LogLevel.CRITICAL]:
      if _logger:
        if level == LogLevel.ERROR:
          _logger.error(message, *args)
        else:
          _logger.critical(message, *args)

        # Force flush the handlers
        for handler in _logger.handlers:
          handler.flush()

      # Also queue it for consistency
      try:
        _log_queue.put((level, message, console_output, args))
      except Exception as e:
        print(f"Error adding message to queue: {e}")
    else:
      # Queue other messages as before
      try:
        _log_queue.put((level, message, console_output, args))
      except Exception as e:
        print(f"Error adding message to queue: {e}")

    # Try to check debug_enabled flag safely
    debug_enabled = False
    try:
      params = Params()
      try:
        debug_enabled = params.check_key("FordPrefEnableDebugLogs") and params.get_bool("FordPrefEnableDebugLogs", default=False)
      except (KeyError, ValueError, TypeError):
        debug_enabled = False
    except (KeyError, ValueError, TypeError):
      pass

    # If debug mode is enabled via params, also log to cloudlog
    if debug_enabled:
      if level == LogLevel.DEBUG:
        cloudlog.debug(message, *args)
      elif level == LogLevel.INFO:
        cloudlog.info(message, *args)
      elif level == LogLevel.WARNING:
        cloudlog.warning(message, *args)
      elif level == LogLevel.ERROR:
        cloudlog.error(message, *args)
      elif level == LogLevel.CRITICAL:
        cloudlog.error(f"CRITICAL: {message}", *args)
  except Exception as e:
    # Last resort fallback
    print(f"Logger error: {e}")
    print(f"Original message: {message}")


# Convenience methods
def debug(message: str, *args, console_output: bool = False) -> None:
  """Log a debug message."""
  log(LogLevel.DEBUG, message, *args, console_output=console_output)


def info(message: str, *args, console_output: bool = False) -> None:
  """Log an info message."""
  log(LogLevel.INFO, message, *args, console_output=console_output)


def warning(message: str, *args, console_output: bool = False) -> None:
  """Log a warning message."""
  log(LogLevel.WARNING, message, *args, console_output=console_output)


def error(message: str, *args, console_output: bool = False) -> None:
  """Log an error message."""
  log(LogLevel.ERROR, message, *args, console_output=console_output)


def critical(message: str, *args, console_output: bool = False) -> None:
  """Log a critical message."""
  log(LogLevel.CRITICAL, message, *args, console_output=console_output)


# Initialize logger on module import
initialize_logger()

# Register cleanup on process exit
import atexit

atexit.register(shutdown)
