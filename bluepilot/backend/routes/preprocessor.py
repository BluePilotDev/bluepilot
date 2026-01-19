#!/usr/bin/env python3
"""
BluePilot Route Preprocessor

Background service that processes routes during idle time:
- Extracts GPS metrics (mileage, speed)
- Reverse geocodes locations
- Generates thumbnails
- Only runs when device is idle (screen off + not driving)
"""

import os
import sys
import time
import json
import logging

# Add parent directory to path for imports
sys.path.append(os.path.join(os.path.dirname(__file__), "../../.."))

try:
    from openpilot.common.params import Params
except ImportError:
    print("Error: Cannot import openpilot modules. Make sure you're running from the device.")
    sys.exit(1)

# Import shared route processing functions
from bluepilot.backend.routes.processing import (
    ROUTES_DIR,
    METRICS_CACHE,
    THUMBNAIL_CACHE,
    check_processing_status,
    process_route,
)

# Import route scanning and segment functions
from bluepilot.backend.routes.scanner import scan_routes
from bluepilot.backend.routes.segments import get_route_segments

# Import WebSocket broadcaster for cross-process communication
from bluepilot.backend.realtime import WebSocketBroadcaster

# Configure logging
logging.basicConfig(
    level=logging.ERROR,
    format='%(levelname)s [%(name)s]: %(message)s',
    force=True  # Force reconfiguration if already configured
)
logger = logging.getLogger("route_preprocessor")

# Processing state file
STATE_FILE = os.path.join(METRICS_CACHE, "preprocessing_state.json")

params = Params()

# Initialize broadcaster for cross-process WebSocket communication
broadcaster = None


def load_processing_state():
    """Load the last processing state"""
    try:
        if os.path.exists(STATE_FILE):
            with open(STATE_FILE) as f:
                return json.load(f)
    except Exception as e:
        logger.warning(f"Error loading state: {e}")

    return {
        'last_processed': [],
        'total_processed': 0
    }


def save_processing_state(state):
    """Save processing state"""
    try:
        with open(STATE_FILE, 'w') as f:
            json.dump(state, f)
    except Exception as e:
        logger.warning(f"Error saving state: {e}")


def is_device_idle():
    """Check if device is idle (screen off + not driving)"""
    try:
        # Check if onroad (driving)
        onroad = params.get_bool("IsOnroad")
        if onroad:
            return False

        # Check screen state (awake param)
        awake = params.get_bool("IsDriverViewEnabled") or params.get_bool("IsEngaged")
        if awake:
            return False

        # Device is idle
        return True
    except Exception as e:
        logger.debug(f"Error checking idle state: {e}")
        return False


def monitor_new_routes(known_routes_set):
    """Monitor for new routes and broadcast when found

    Args:
        known_routes_set: Set of known route base names

    Returns:
        Set of newly discovered routes
    """
    # Get current routes
    current_routes = scan_routes()
    current_routes_set = {r['baseName'] for r in current_routes}

    # Find new routes
    new_routes = current_routes_set - known_routes_set

    if new_routes and broadcaster:
        logger.info(f"Found {len(new_routes)} new routes: {', '.join(sorted(new_routes))}")

        # Broadcast each new route with its data
        for route_base in new_routes:
            route_data = next((r for r in current_routes if r['baseName'] == route_base), None)
            if route_data:
                broadcaster.broadcast_route_added(route_base, route_data)
                logger.info(f"Broadcasted new route: {route_base}")

    return new_routes


# Processing status check and route processing functions imported from route_processing module


def preprocess_routes_batch(max_routes=5, max_time_seconds=300):
    """Preprocess routes in batch during idle time

    Args:
        max_routes: Maximum number of routes to process in this batch
        max_time_seconds: Maximum time to spend processing

    Returns:
        dict: Statistics about processing
    """
    start_time = time.monotonic()
    state = load_processing_state()

    # Get all routes (fast, cached data only)
    routes = scan_routes()

    if not routes:
        logger.info("No routes to process")
        return {'processed': 0, 'skipped': 0, 'remaining': 0, 'time': 0}

    # Find routes that haven't been processed yet
    processed_set = set(state.get('last_processed', []))
    unprocessed = [r for r in routes if r['baseName'] not in processed_set]

    if not unprocessed:
        logger.info(f"All {len(routes)} routes already preprocessed")
        return {'processed': 0, 'skipped': len(routes), 'remaining': 0, 'time': 0}

    logger.info(f"Found {len(unprocessed)} unprocessed routes (of {len(routes)} total)")

    # Broadcast processing started
    if broadcaster:
        broadcaster.broadcast_processing_started(len(unprocessed[:max_routes]))

    # Process routes one at a time
    processed_count = 0
    for idx, route in enumerate(unprocessed[:max_routes]):
        # Check if still idle
        if not is_device_idle():
            logger.info("Device no longer idle, pausing preprocessing")
            break

        # Check time limit
        elapsed = time.monotonic() - start_time
        if elapsed > max_time_seconds:
            logger.info(f"Time limit reached ({max_time_seconds}s), pausing")
            break

        # Broadcast processing update
        route_base = route['baseName']
        progress = int((idx / min(len(unprocessed), max_routes)) * 100)
        if broadcaster:
            broadcaster.broadcast_processing_update(
                route_base,
                'processing',
                progress=progress,
                message=f"Processing {idx+1}/{min(len(unprocessed), max_routes)}"
            )

        # Preprocess this route using shared processing function
        segments = get_route_segments(route_base)
        if segments and process_route(route_base, segments, check_idle_fn=is_device_idle):
            processed_set.add(route_base)
            processed_count += 1
            state['total_processed'] = state.get('total_processed', 0) + 1

            # Broadcast completion for this route
            if broadcaster:
                broadcaster.broadcast_processing_update(route_base, 'completed', progress=100)

        # Small delay between routes
        time.sleep(0.5)

    # Save state
    state['last_processed'] = list(processed_set)
    save_processing_state(state)

    elapsed = time.monotonic() - start_time
    logger.info(f"Batch complete: {processed_count} routes in {elapsed:.1f}s")

    # Broadcast batch completion
    if broadcaster:
        broadcaster.broadcast_processing_completed(processed_count, elapsed)

    return {
        'processed': processed_count,
        'remaining': len(unprocessed) - processed_count,
        'time': elapsed
    }


def main():
    """Main background processing loop"""
    global broadcaster

    # Add venv site-packages to sys.path so we can import pycapnp and other venv packages
    # This only affects the preprocessor process, not other openpilot processes
    venv_site_packages = "/usr/local/venv/lib/python3.12/site-packages"
    if os.path.exists(venv_site_packages) and venv_site_packages not in sys.path:
        sys.path.insert(0, venv_site_packages)
        logger.info(f"Added venv site-packages to sys.path for LogReader support")

    logger.info("BluePilot Route Preprocessor starting...")
    logger.info(f"Routes directory: {ROUTES_DIR}")
    logger.info(f"Metrics cache: {METRICS_CACHE}")

    # Kill any existing preprocessor instances first
    from bluepilot.backend.routes.processing import kill_existing_process
    kill_existing_process('route_preprocessor.py')

    # Ensure cache directory exists
    os.makedirs(METRICS_CACHE, exist_ok=True)

    # Initialize broadcaster for cross-process WebSocket communication
    # Get web server port from params
    try:
        web_port = int(params.get("BPPortalPort") or "8088")
    except:
        web_port = 8088

    broadcaster = WebSocketBroadcaster(http_fallback_port=web_port)
    logger.info(f"WebSocket broadcaster initialized (HTTP fallback to port {web_port})")

    # Main loop - check every 30 seconds for new routes and processing opportunities
    check_interval = 30

    # Track known routes
    known_routes = set(r['baseName'] for r in scan_routes())
    logger.info(f"Initial route count: {len(known_routes)}")

    while True:
        try:
            # Check for new routes
            new_routes = monitor_new_routes(known_routes)
            if new_routes:
                known_routes.update(new_routes)

            # Check if device is idle
            if is_device_idle():
                logger.info("Device idle, starting background processing...")

                # Process up to 5 routes or 5 minutes, whichever comes first
                stats = preprocess_routes_batch(max_routes=5, max_time_seconds=300)

                if stats['remaining'] > 0:
                    logger.info(f"{stats['remaining']} routes remaining for next idle period")
                else:
                    logger.info("All routes preprocessed!")
            else:
                logger.debug("Device not idle (driving or screen on)")

        except KeyboardInterrupt:
            logger.info("Preprocessor stopped by user")
            break
        except Exception as e:
            logger.error(f"Error in main loop: {e}")

        # Wait before next check
        time.sleep(check_interval)


if __name__ == "__main__":
    main()
