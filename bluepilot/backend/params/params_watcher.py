#!/usr/bin/env python3
"""
BluePilot Params Watcher
Monitors parameter changes and broadcasts updates via WebSocket
"""

import os
import time
import logging
import threading
from pathlib import Path
from typing import Dict, Any, Optional, Callable

logger = logging.getLogger(__name__)


class ParamsWatcher:
    """Watch params directory for changes and broadcast updates"""

    def __init__(self, params, broadcast_callback: Optional[Callable] = None):
        """
        Initialize params watcher

        Args:
            params: Params instance
            broadcast_callback: Function to call when params change (key, value)
        """
        self.params = params
        self.broadcast_callback = broadcast_callback
        self.running = False
        self.thread = None
        self.check_interval = 2.0  # Check every 2 seconds
        self.params_cache = {}  # Cache of param values with their modification times

        # Determine params directory
        self.params_dir = Path("/data/params/d") if os.path.exists("/data/params/d") else None

        if self.params_dir:
            logger.info(f"Params watcher initialized for directory: {self.params_dir}")
        else:
            logger.warning("Params directory not found - watcher will not monitor file changes")

    def start(self):
        """Start watching for param changes in background thread"""
        if self.running:
            logger.warning("Params watcher already running")
            return

        if not self.params_dir:
            logger.info("Params directory not available - skipping watcher")
            return

        self.running = True
        self.thread = threading.Thread(target=self._watch_loop, daemon=True)
        self.thread.start()
        logger.info("Params watcher started")

    def stop(self):
        """Stop watching for param changes"""
        if not self.running:
            return

        self.running = False
        if self.thread:
            self.thread.join(timeout=5.0)
        logger.info("Params watcher stopped")

    def _watch_loop(self):
        """Main watch loop - runs in background thread"""
        logger.info("Params watcher loop starting")

        # Initialize cache with current params
        self._update_cache()

        while self.running:
            try:
                self._check_for_changes()
            except Exception as e:
                logger.error(f"Error in params watcher loop: {e}")

            # Sleep in small increments to allow quick shutdown
            for _ in range(int(self.check_interval * 10)):
                if not self.running:
                    break
                time.sleep(0.1)

        logger.info("Params watcher loop ended")

    def _update_cache(self):
        """Update cache with current param files and their modification times"""
        if not self.params_dir or not self.params_dir.exists():
            return

        try:
            for param_file in self.params_dir.iterdir():
                if param_file.is_file():
                    param_key = param_file.name
                    mtime = param_file.stat().st_mtime

                    # Store both mtime and value
                    try:
                        value = self._read_param_value(param_key)
                        self.params_cache[param_key] = {
                            'mtime': mtime,
                            'value': value
                        }
                    except Exception as e:
                        logger.debug(f"Could not read param {param_key}: {e}")

        except Exception as e:
            logger.error(f"Error updating params cache: {e}")

    def _read_param_value(self, key: str) -> Any:
        """Read a param value using the Params instance"""
        try:
            # Try boolean first
            if key.endswith("Enabled") or key.endswith("Toggle") or key in ["IsOnRoad", "IsOffroad", "Passive"]:
                return self.params.get_bool(key)

            # Try getting as string
            value = self.params.get(key, encoding='utf-8')

            if value is None or value == b'':
                return None

            if isinstance(value, bytes):
                value = value.decode('utf-8', errors='replace')

            # Try to parse as number
            if value.isdigit():
                return int(value)

            try:
                return float(value)
            except ValueError:
                pass

            return value

        except Exception:
            return None

    def _check_for_changes(self):
        """Check for changes in params directory"""
        if not self.params_dir or not self.params_dir.exists():
            return

        try:
            current_files = set()

            # Check all param files
            for param_file in self.params_dir.iterdir():
                if not param_file.is_file():
                    continue

                param_key = param_file.name
                current_files.add(param_key)
                mtime = param_file.stat().st_mtime

                # Check if this is a new param or has been modified
                if param_key not in self.params_cache:
                    # New param
                    value = self._read_param_value(param_key)
                    self.params_cache[param_key] = {
                        'mtime': mtime,
                        'value': value
                    }
                    self._broadcast_change(param_key, value)
                    logger.debug(f"New param detected: {param_key} = {value}")

                elif mtime > self.params_cache[param_key]['mtime']:
                    # Param was modified
                    old_value = self.params_cache[param_key]['value']
                    new_value = self._read_param_value(param_key)

                    # Only broadcast if value actually changed
                    if new_value != old_value:
                        self.params_cache[param_key] = {
                            'mtime': mtime,
                            'value': new_value
                        }
                        self._broadcast_change(param_key, new_value)
                        logger.info(f"Param changed: {param_key} = {old_value} -> {new_value}")

            # Check for deleted params
            deleted_keys = set(self.params_cache.keys()) - current_files
            for key in deleted_keys:
                del self.params_cache[key]
                self._broadcast_change(key, None)
                logger.debug(f"Param deleted: {key}")

        except Exception as e:
            logger.error(f"Error checking for param changes: {e}")

    def _broadcast_change(self, key: str, value: Any):
        """Broadcast a param change via the callback"""
        if self.broadcast_callback:
            try:
                self.broadcast_callback(key, value)
            except Exception as e:
                logger.error(f"Error broadcasting param change for {key}: {e}")
