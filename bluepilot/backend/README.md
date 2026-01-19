# BluePilot Portal Backend

The BluePilot Portal backend provides HTTP and WebSocket APIs for route management, video streaming, export/backup functionality, and system monitoring.

## Overview

The backend has been refactored from `web_routes_server.py` to `bp_portal.py` to support future expansion beyond just routes handling.

### Current Capabilities

- **Route Management**: Browse, view, and manage driving routes
- **Video Streaming**: Stream HEVC/H.265 video with browser-compatible remuxing
- **Export & Backup**: Create route exports and backups with compression
- **System Monitoring**: Track cache sizes, disk space, and system health
- **Real-time Updates**: WebSocket support for live progress updates
- **Cache Management**: Intelligent caching with LRU eviction

### Future Expansion (Planned)

- Analytics and telemetry endpoints
- Machine learning model serving
- Advanced route processing pipelines
- Integration with external services (maps, weather, etc.)
- Multi-device synchronization

## Architecture

### Directory Structure

```
bluepilot/backend/
├── bp_portal.py                   # Main entry point
├── web_routes_server.py           # Legacy implementation (kept for reference)
├── config.py                      # Centralized configuration (NEW)
│
├── routes/                        # Route processing and analysis
│   ├── __init__.py
│   ├── processing.py              # Route analysis and GPS metrics
│   └── preprocessor.py            # Background route preprocessing
│
├── realtime/                      # Real-time communications
│   ├── __init__.py
│   └── websocket.py               # WebSocket event broadcasting
│
├── core/                          # Core functionality
│   ├── __init__.py
│   └── logging_handler.py         # Error logging for web display
│
├── utils/                         # Utility modules
│   ├── __init__.py
│   ├── file_ops.py                # Atomic file operations
│   └── power.py                   # CPU power management
│
├── video/                         # Video processing (placeholder)
│   └── __init__.py
│
└── handlers/                      # HTTP endpoint handlers
    ├── __init__.py
    └── log_downloads.py           # Download qlog/rlog helpers
```

### Process Configuration

The backend server is managed by the openpilot process manager:

**Process Name**: `bp_portal`
**Module**: `bluepilot.backend.bp_portal`
**Condition**: Runs when `BPPortalEnabled` param is true

See [process_config.py:198](../../system/manager/process_config.py#L198)

## Migration Notes

### Phase 1: Renaming (COMPLETED)

- Created `bp_portal.py` as new entry point
- Updated process_config.py to use new process name
- Created modular directory structure (core/, utils/, video/, routes/, realtime/)
- Extracted initial modules:
  - `config.py` - Configuration constants
  - `core/logging_handler.py` - Error logging
  - `utils/file_ops.py` - Atomic file operations
  - `utils/power.py` - CPU power management

### Phase 2: Module Organization (COMPLETED)

- Organized route-related files into `routes/` directory
  - `routes/processing.py` - Route analysis and GPS metrics (was `route_processing.py`)
  - `routes/preprocessor.py` - Background preprocessing (was `route_preprocessor.py`)
- Organized real-time communications into `realtime/` directory
  - `realtime/websocket.py` - WebSocket broadcaster (was `websocket_broadcaster.py`)
- Updated all import paths throughout the codebase
- Updated process_config.py for new module paths
- 100% feature parity maintained

The current implementation delegates to `web_routes_server.py` while providing a clean modular structure. Future work will migrate functionality from the monolithic server into focused modules:

**Planned Modules:**

`core/`
- `server_state.py` - Thread-safe server state management
- `http_handler.py` - Base HTTP request handler
- `cors.py` - CORS utilities

`video/`
- `ffmpeg.py` - FFmpeg process management
- `remux.py` - HEVC to MP4 remuxing
- `prefetch.py` - Segment prefetching
- `export.py` - Route video export generation

`utils/`
- `cache.py` - Cache management and cleanup
- `disk.py` - Disk space monitoring
- `network.py` - Network utilities (WiFi, hotspot, onroad detection)

`handlers/`
- `routes.py` - Route listing and details
- `video.py` - Video streaming endpoints
- `cache.py` - Cache management endpoints
- `system.py` - System metrics and health
- `timeline.py` - Timeline and statistics

### Phase 3: Handler Modularization (PLANNED)

Break down the monolithic HTTP handler into focused endpoint handlers, similar to the existing `handlers/` pattern.

## Development

### Running the Server

The server is automatically managed by the openpilot process manager when enabled via params.

Manual testing:
```bash
cd /data/openpilot  # or your openpilot directory
python3 -m bluepilot.backend.bp_portal
```

### Configuration

All configuration is centralized in [config.py](config.py):

- `ROUTES_DIR` - Location of driving routes
- `WEBAPP_DIR` - Web frontend directory
- `*_CACHE` - Cache directories for different data types
- `WEBSOCKET_PORT` - WebSocket server port (8089)
- `DEFAULT_PORT` - HTTP server port (8088)

### Adding New Features

1. **New HTTP endpoints**: Add handler functions in `handlers/` directory
2. **New utilities**: Add to appropriate module in `core/`, `utils/`, or `video/`
3. **New configuration**: Add to `config.py`
4. **New dependencies**: Update backend requirements

### Code Quality

- Use type hints where appropriate
- Add docstrings for all public functions
- Keep modules focused and single-purpose
- Avoid circular dependencies
- Test changes with actual route data

## API Endpoints

### Routes
- `GET /api/routes` - List all routes
- `GET /api/routes/{route}` - Get route details
- `DELETE /api/routes/{route}` - Delete route

### Video Streaming
- `GET /api/stream/{route}/{segment}/{camera}` - Stream video segment
- `POST /api/export/{route}/{camera}` - Generate route export

### Export & Backup
- `POST /api/videos-zip/{route}` - Create videos ZIP archive
- `POST /api/route-backup/{route}` - Create route backup
- `POST /api/route-import` - Import route backup

### System
- `GET /api/system` - System metrics and health
- `GET /api/cache` - Cache sizes and statistics
- `POST /api/cache/clear` - Clear cache

### WebSocket
- `ws://device_ip:8089` - Real-time event stream

## Legacy Files

### web_routes_server.py

The original monolithic implementation (~6000 lines) is kept for reference during migration. It contains:
- All HTTP request handling logic
- Video processing and streaming
- FFmpeg process management
- Cache management
- System monitoring
- WebSocket integration

**Status**: Active (bp_portal.py delegates to this)
**Future**: Will be deprecated once modularization is complete

## Contributing

When adding new backend functionality:

1. Consider if it fits in existing modules or needs a new one
2. Keep modules small and focused (< 500 lines ideal)
3. Update this README with new features
4. Add tests for new functionality
5. Document API endpoints

## References

- Process Manager: [system/manager/process_config.py](../../system/manager/process_config.py)
- Route Processing: [route_processing.py](route_processing.py)
- WebSocket Broadcasting: [websocket_broadcaster.py](websocket_broadcaster.py)
- Frontend: [bluepilot/web/](../web/)
