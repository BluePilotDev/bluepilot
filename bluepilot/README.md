# BluePilot Web Routes Panel

Modern web-based interface for viewing and playing route videos directly in your browser, eliminating the Qt video playback issues.

**Dependencies:** `websockets` (Python package for real-time updates, with HTTP polling fallback) - Auto-installed on first run

**Note:** For read-only environments, install dependencies before deployment: `uv sync --extra dev`

## Quick Start

### 1. Enable the Web Server

In the BluePilot UI, go to the Routes panel and tap "Start Server". The server will:
- Start automatically when enabled
- Stop automatically when you start driving (safety feature)
- Restart when you stop driving

### 2. Access the Web Interface

**On your phone/tablet:**
1. Connect to the same WiFi network as your Comma device
2. Open the URL shown in the Routes panel (e.g., `http://192.168.1.100:8088`)
3. Bookmark it for easy access

**Or use the QR code** (coming soon) to quickly open on mobile devices.

## Features

### Route Browsing
- View all your routes organized by date
- See duration, size, and segment count
- Star your favorite routes
- Search and filter routes
- Delete routes you don't need

### Video Playback
- Native HEVC playback in modern browsers (Safari recommended)
- Low Quality (LQ) H.264 fallback for maximum compatibility
- Switch between cameras (Front, Wide, Driver, LQ)
- Automatic segment transitions
- Scrub through timeline
- Keyboard shortcuts (Space, Arrow keys, F for fullscreen)
- Touch-friendly controls for mobile

### Real-Time Updates
- **WebSocket support** (port 8089) for instant updates
- **HTTP polling fallback** if WebSocket unavailable
- Live route processing progress
- Instant UI updates on route changes
- Auto-reconnection on connection loss

### Safety & Security
- Server is read-only when driving (modification APIs blocked)
- Default WiFi-only binding (secure by default)
- Optional cellular access (time-limited, requires explicit opt-in)
- Lightweight architecture (stdlib only, minimal memory footprint)
- No performance impact on driving functions

## Browser Compatibility

| Browser | HEVC Support | LQ (H.264) | Status |
|---------|--------------|------------|--------|
| Safari (iOS/macOS) | Native | ✅ | ✅ Full support |
| Chrome (Android) | Varies | ✅ | ⚠️ HEVC device-dependent |
| Edge (Windows) | Varies | ✅ | ⚠️ HEVC device-dependent |
| Firefox | Limited | ✅ | ⚠️ Use LQ camera |

**Recommended:** Safari on iPhone/iPad for best HEVC experience, or use LQ camera for universal H.264 compatibility.

## Architecture

### Overview

The web routes server is a multi-component system designed for reliability, performance, and zero dependencies:

```
bluepilot/
├── backend/
│   ├── web_routes_server.py    # Main HTTP/WebSocket server (4000+ lines)
│   ├── route_processing.py     # Shared route processing logic
│   ├── route_preprocessor.py   # Background idle-time processor
│   └── websocket_broadcaster.py # Cross-process WebSocket events
├── web/
│   ├── src/                     # Source files
│   │   ├── index.html
│   │   ├── styles.css
│   │   └── app.js
│   └── public/                  # Deployed files (committed)
├── test_web_routes.py           # Local testing script
└── setup_web_routes.py          # Setup verification
```

### Core Components

#### 1. Web Routes Server (`web_routes_server.py`)
**Purpose:** Main HTTP API server with real-time WebSocket support

**Key Features:**
- **Zero-dependency HTTP server** using Python stdlib `http.server`
- **WebSocket support** (optional, with HTTP polling fallback)
- **Byte-range request support** for video seeking/scrubbing
- **Thread-safe architecture** with proper locking for concurrent access
- **Cellular access control** with time-limited access
- **WiFi-only binding** by default (secure default)
- **Onroad safety check** (read-only API during driving)
- **Server state management** with error buffering for diagnostics

**Classes:**
- `ServerState`: Global state management (routes cache, WebSocket clients, broadcaster)
- `ReuseAddressHTTPServer`: HTTPServer with SO_REUSEADDR for clean restarts
- `WebRoutesHandler`: Request handler with full REST API implementation

**Lifecycle:**
1. Port configuration from params (default: 8088)
2. WebSocket server starts on port 8089 (separate thread)
3. HTTP server binds to WiFi IP (or 0.0.0.0 if cellular allowed)
4. Periodic status monitoring (onroad detection, cellular timeout)
5. Graceful shutdown with cleanup

#### 2. Route Processing (`route_processing.py`)
**Purpose:** Shared logic for extracting route metadata and processing segments

**Key Functions:**
- `haversine_distance()`: GPS coordinate distance calculation
- `reverse_geocode()`: Convert GPS to location names (Nominatim API)
- `extract_gps_metrics_from_segment()`: Parse rlogs for GPS data
- `get_route_gps_metrics()`: Calculate total mileage, speed from all segments
- `generate_thumbnail()`: Extract thumbnails from qcamera streams
- `process_route()`: Full route processing pipeline
- `check_processing_status()`: Recovery from interrupted processing

**Caching Strategy:**
- `/data/bluepilot/routes/metrics_cache/` - GPS metrics and geocoding
- `/data/bluepilot/routes/thumbs_cache/` - Thumbnails
- Atomic writes to prevent corruption from crashes
- LRU caching for frequent route access

#### 3. Route Preprocessor (`route_preprocessor.py`)
**Purpose:** Background service for idle-time route processing

**Behavior:**
- Only runs when device is idle (screen off + not driving)
- Processes unprocessed routes in order
- Broadcasts WebSocket events for real-time UI updates
- Saves state for resumable processing
- Rate-limited geocoding (1 req/sec for Nominatim compliance)

**Integration:**
- Launched by openpilot manager
- Uses WebSocketBroadcaster for cross-process updates
- Shares route_processing.py functions with web server

#### 4. WebSocket Broadcaster (`websocket_broadcaster.py`)
**Purpose:** Unified event broadcasting for real-time UI updates

**Modes:**
- **In-process:** Direct access to WebSocket clients (web_routes_server.py)
- **Cross-process:** HTTP POST to `/_internal/broadcast` (route_preprocessor.py)

**Event Types:**
- `routes_updated` - Route list changed
- `route_added` - New route detected
- `route_deleted` - Route removed
- `route_preserved` - Route marked as preserved
- `processing_update` - Processing progress for a route
- `processing_started` - Batch processing started
- `processing_completed` - Batch processing finished
- `status_changed` - Device status changed (onroad/offroad)
- `cache_cleared` - Cache cleared

### Frontend (`web/`)

**Architecture:**
- **Vanilla JavaScript** (zero npm dependencies)
- **Modern CSS** (BluePilot dark theme)
- **Responsive design** for mobile and desktop
- **Progressive enhancement** (works without WebSocket)

**Communication:**
- Primary: WebSocket (port 8089) for real-time updates
- Fallback: HTTP polling (5-second interval)
- Auto-reconnection on connection loss

### Qt Integration

**Panel:** `selfdrive/ui/bluepilot/qt/offroad/routes_panel/bp_routes_panel.cc`

**Controls:**
- Enable/disable server toggle
- Port configuration
- QR code generation (coming soon)
- Server status display
- WiFi IP address display

**Safety Features:**
- Server lifecycle managed by openpilot manager
- Automatic shutdown on drive start (via params)
- Graceful restart on drive end

## API Endpoints

### Core Endpoints

#### Health & Status
- `GET /api/health` - Basic health check (minimal response)
- `GET /api/status` - Server status summary (onroad, routes count, errors)
- `GET /api/status/detailed` - Detailed status (uptime, memory, processing, etc.)
- `GET /api/logs` - Recent log messages (last 100 lines)
- `GET /api/system/metrics` - System metrics (CPU, memory, disk)

#### Routes Management
- `GET /api/routes` - List all routes with metadata
  - Returns: Array of route objects with date, duration, size, segments, preserved status
  - Cached for performance (refreshes on route changes)
- `GET /api/routes/:baseName` - Get detailed route info
  - Returns: Full route details with GPS metrics, location, thumbnails
- `GET /api/disk-space` - Get disk space information
- `GET /api/route-coordinates/:baseName` - Get GPS coordinates for route
  - Returns: Array of coordinate points for map display

#### Video Streaming
- `GET /api/video/:baseName/:segment/:camera` - Stream video segment
  - Supports byte-range requests for seeking/scrubbing
  - Camera types: `fcamera` (front), `wide_camera`, `driver_camera`, `lq` (H.264)
  - MIME types: `video/mp4; codecs="hvc1"` for HEVC, `video/mp2t` for H.264
- `GET /api/hls/:baseName/:segment/:camera/playlist.m3u8` - HLS playlist (future)
- `GET /api/download/route/:baseName` - Download entire route as tar.bz2
- `GET /api/download/segment/:baseName/:segment` - Download single segment

#### Thumbnails & Media
- `GET /api/thumbnail/:baseName` - Get route thumbnail (JPEG)
  - Cached thumbnails from qcamera streams
  - Auto-generated if not in cache

#### Location Services
- `GET /api/geocode/:lat/:lon` - Reverse geocode GPS coordinates
  - Uses Nominatim API with rate limiting (1 req/sec)
  - Caches results for performance

#### Data Access
- `GET /api/logs/:baseName/:segment` - Get rlog data for segment
  - Returns raw rlog file for analysis
- `GET /api/cereal/:baseName/:segment` - Get parsed cereal messages
  - Returns JSON array of decoded messages

### Modification Endpoints

#### Route Preservation
- `POST /api/preserve/:baseName` - Toggle route preservation
  - Uses xattr metadata to mark routes as preserved
  - Preserved routes survive cleanup/deletion
  - Broadcasts WebSocket event: `route_preserved` / `route_unpreserved`

#### Cache Management
- `POST /api/clear-cache` - Clear all cached data
  - Clears metrics cache and thumbnails
  - Forces re-processing on next access
  - Broadcasts WebSocket event: `cache_cleared`

#### Route Deletion
- `DELETE /api/delete/:baseName` - Delete route and all segments
  - Removes route directory and all files
  - Updates cache
  - Broadcasts WebSocket event: `route_deleted`

### Internal Endpoints

#### WebSocket Broadcasting
- `POST /_internal/broadcast` - Internal WebSocket broadcast endpoint
  - Used by route_preprocessor for cross-process events
  - Not intended for external use

### Response Format

All API endpoints return JSON with consistent structure:

```json
{
  "success": true,           // Boolean status
  "timestamp": "2025-...",   // ISO 8601 timestamp
  "data": { ... }            // Endpoint-specific data
}
```

Error responses (4xx/5xx):
```json
{
  "success": false,
  "error": "Error message",
  "timestamp": "2025-..."
}
```

### Safety Features

**Onroad Protection:**
- All modification endpoints (POST/DELETE) return 403 Forbidden when driving
- Read-only access allowed during driving for safety
- `/api/status` shows onroad status

**Network Security:**
- Default: Binds to WiFi interface only
- Cellular access requires explicit opt-in (time-limited)
- CORS headers for local network access

## Development

### Testing Locally

No dependencies to install - uses Python standard library only:

```bash
cd /data/openpilot  # or your openpilot directory
python3 bluepilot/test_web_routes.py
```

Then open http://localhost:8088 in your browser.

### Building Web App

```bash
cd bluepilot/web
./build.sh
```

This copies files from `src/` to `public/` (which is committed to the repo).

### Modifying the UI

1. Edit files in `bluepilot/web/src/`
2. Run `./build.sh` to update `public/`
3. Refresh browser to see changes
4. Commit both `src/` and `public/` directories

## Configuration

### Parameters

Set via Comma device UI (Routes panel) or params API:

**Portal Control:**
- `BPPortalEnabled` (bool) - Enable/disable portal
  - Managed by Qt panel toggle
  - Server lifecycle controlled by openpilot manager

- `BPPortalPort` (int) - HTTP server port (default: 8088)
  - WebSocket server runs on port + 1 (default: 8089)

**Network Access:**
  - When enabled: Binds to 0.0.0.0 (all interfaces)
  - When disabled: Binds to WiFi IP only (secure default)
  - Time-limited access (configurable timeout)

**Cellular Access Timeout:**
- `BPPortalCellularTimeout` (int) - Cellular access timeout in minutes (default: 30)
- `BPPortalCellularEnabledTime` (int) - Timestamp when cellular was enabled

**Route Preservation:**
- Uses xattr filesystem attributes (not params)
- Preserved status stored in file metadata
- Survives server restarts and device reboots

### File Locations

**On Device:**
- Routes: `/data/media/0/realdata/`
- Metrics cache: `/data/bluepilot/routes/metrics_cache/`
- Thumbnails: `/data/bluepilot/routes/thumbs_cache/`
- Geocoding cache: `/data/bluepilot/routes/metrics_cache/geocoding_cache.json`

**Development:**
- Routes: `~/comma_data/media/0/realdata/`
- Caches: `~/comma_data/bluepilot/routes/...`

## Troubleshooting

### Server won't start
- Check that port 8088 is not in use: `lsof -i :8088`
- Check process manager logs: `journalctl -u manager -f | grep bp_portal`
- Verify BPPortalEnabled param is true

### Can't access from phone
- Ensure phone and Comma device are on same WiFi network
- Check firewall settings
- Try accessing from Comma device first: `curl http://localhost:8088/api/status`

### Video won't play
- Try switching to LQ camera (uses H.264 instead of HEVC)
- Check browser HEVC support (Safari on iOS recommended)
- Verify video files exist in `/data/media/0/realdata/`
- Check browser console for errors (F12)

### Routes showing "boot" or other invalid directories
- This should be filtered automatically by the backend
- Check backend is running latest version
- Verify routes directory permissions

### Server uses too much memory
- Server is very lightweight (stdlib only, no framework overhead)
- Check active connections: `netstat -an | grep 8088`
- Restart server via UI toggle

## Security Notes

- Server only accessible on local network (not exposed to internet)
- No authentication by default (coming soon as optional feature)
- Onroad safety check prevents use while driving
- Read-only access to route files (except delete)

## Technical Details

### Route Processing Pipeline

**On-demand processing (web_routes_server.py):**
1. Route accessed via API
2. Check metrics cache for existing data
3. If missing, extract GPS metrics from rlogs
4. Reverse geocode location (rate-limited)
5. Cache results for future access
6. Return route data

**Background processing (route_preprocessor.py):**
1. Device becomes idle (screen off + not driving)
2. Scan for unprocessed routes
3. Process routes in order (oldest first)
4. Broadcast WebSocket events for UI updates
5. Save state for resumable processing
6. Stop when device becomes active

**Processing Status Recovery:**
- Handles interrupted processing from crashes/reboots
- Checks for partial cache files
- Re-processes incomplete routes
- Atomic writes prevent cache corruption

### Thread Safety

**ServerState Class:**
- Thread-safe route cache with RLock
- Thread-safe WebSocket client management
- Thread-safe broadcaster access
- Synchronized error buffer for diagnostics

**WebSocket Server:**
- Runs in separate daemon thread
- Asyncio event loop for async operations
- Thread-safe event broadcasting
- Graceful client disconnection handling

### Performance Optimizations

**Caching:**
- LRU cache for route data (`@lru_cache`)
- Persistent disk cache for GPS metrics
- Persistent disk cache for thumbnails
- In-memory geocoding cache

**Lazy Loading:**
- Routes scanned on first access
- GPS metrics calculated on demand
- Thumbnails generated when requested
- Segment data loaded as needed

**Efficient File I/O:**
- Byte-range support for video seeking
- Chunked responses for large files
- Atomic writes to prevent corruption
- Proper file handle cleanup

## Future Enhancements

**Completed:**
- [x] Zero-dependency Python stdlib HTTP server
- [x] WebSocket real-time updates
- [x] LQ camera H.264 fallback for compatibility
- [x] Thumbnail support from cache
- [x] Route filtering and proper date/time formatting
- [x] Background route preprocessing
- [x] Cross-process WebSocket broadcasting
- [x] Route preservation with xattr metadata
- [x] Cellular access control (time-limited)
- [x] Onroad safety checks
- [x] Detailed system metrics API

**Planned:**
- [ ] QR code generation for mobile access
- [ ] Thumbnail generation if cache is empty
- [ ] HLS/DASH playlist support for better streaming
- [ ] Progressive Web App (PWA) installation
- [ ] Remote access via Comma Connect integration
- [ ] Optional authentication for shared devices
- [ ] Drive analytics and statistics dashboard
- [ ] Export routes to external storage (USB/cloud)
- [ ] Multi-route video stitching
- [ ] Advanced search and filtering
- [ ] Route comparison and diff tools

## Credits

Built with:
- Python standard library (`http.server`)
- Vanilla JavaScript (zero dependencies)
- Modern CSS (BluePilot dark theme)

Developed for BluePilot (Ford-specific OpenPilot fork)
