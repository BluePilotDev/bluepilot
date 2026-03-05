# AGENTS.md - BluePilot Codebase Guide

**Version:** 5.0.0 → 6.0.0 (in development)
**Last Updated:** 2026-02-16
**Target Audience:** AI agents, developers, and contributors

This document provides a comprehensive guide to understanding, navigating, and modifying the BluePilot codebase. It is designed to help AI agents and human developers quickly understand the architecture, conventions, and key systems.

---

## Table of Contents

1. [Project Identity](#project-identity)
2. [Core Architecture Principle](#core-architecture-principle)
3. [Directory Structure](#directory-structure)
4. [Ford-Specific Control System](#ford-specific-control-system)
5. [BluePilot Portal](#bluepilot-portal)
6. [Parameter System](#parameter-system)
7. [SunnyPilot Features](#sunnypilot-features)
8. [UI System Architecture](#ui-system-architecture)
9. [Messaging System](#messaging-system)
10. [Build System](#build-system)
11. [Development Conventions](#development-conventions)
12. [Testing](#testing)
13. [Logging](#logging)
14. [Quick Reference Guide](#quick-reference-guide)
15. [Critical Safety Notes](#critical-safety-notes)

---

## Project Identity

**BluePilot** is a Ford-focused fork of SunnyPilot (which itself forks OpenPilot/commaai).

### Key Information
- **Current Version:** 5.0.0
- **Based On:** SunnyPilot 2025.003.0.0
- **Repository:** https://github.com/BluePilotDev/bluepilot
- **Current Branch:** `bp-6.0-ui-refactor` (active development)
- **Main Dev Branch:** `bp-dev`
- **Platform:** comma 3X device (TICI) and MICI devices
- **AGNOS:** 13.1
- **Focus:** Ford-specific enhancements for lateral/longitudinal control, hybrid vehicle support, and enhanced UI

### What Makes BluePilot Unique
- Advanced Ford-specific lateral control using 4-signal CAN messaging
- Hybrid vehicle battery monitoring and power flow visualization
- Blindspot monitoring integration
- Advanced lane positioning (ALP) with configurable in-lane offset
- Web-based BluePilot Portal for route management and device configuration
- High/low curvature mode tuning for different driving scenarios

---

## Core Architecture Principle

**THIS IS THE MOST IMPORTANT CONCEPT FOR WORKING IN THIS CODEBASE.**

BluePilot uses a **three-layer inheritance architecture** that keeps the codebase in sync with upstream SunnyPilot and OpenPilot while maintaining Ford-specific customizations.

### Three-Layer Architecture

```
Layer 1: Stock OpenPilot (upstream base)
         ↓
Layer 2: SunnyPilot extensions (intermediate fork)
         ↓
Layer 3: BluePilot customizations (top-level Ford-specific)
```

### Design Principles

1. **Minimize changes to stock openpilot and sunnypilot files.** When changes are necessary (e.g., to wire in BluePilot classes), wrap them in comments clearly stating what the change is for. This makes upstream merges straightforward - you can search for BluePilot comments to find all touchpoints.
2. BluePilot classes use the `*BP` suffix (e.g., `AlertRendererBP`, `HudRendererBP`)
3. SunnyPilot classes use the `*SP` suffix (e.g., `HudRendererSP`, `DriverStateRendererSP`)
4. Override classes live in parallel directory structures (`selfdrive/ui/bp/` mirrors `selfdrive/ui/onroad/`)
5. Conditional imports in wiring files (like `layouts/main.py`) swap stock classes for BP versions
6. New processes are ADDED alongside stock processes via `procs +=` in `process_config.py`, never replacing them
7. Feature flags via Params system allow toggling BP features without code changes

### Commenting Changes to Upstream Files

When you must modify a stock openpilot or sunnypilot file, wrap the change with comments:

```python
# BluePilot: import BP alert renderer override
from openpilot.selfdrive.ui.bp.onroad.alert_renderer_bp import AlertRendererBP
# End BluePilot

# BluePilot: swap stock renderer for BP version
self.alert_renderer = AlertRendererBP()
# End BluePilot
```

This pattern makes it easy to:
- Find all BluePilot touchpoints during upstream merges (`grep -r "BluePilot:"`)
- Understand why each change exists
- Resolve merge conflicts quickly

### UI Inheritance Chain Example

```
Widget (openpilot base)
├── AlertRenderer (stock) → AlertRendererBP (BluePilot)
├── HudRenderer (stock) → HudRendererSP (SunnyPilot) → HudRendererBP (BluePilot)
├── ModelRenderer (stock) → ModelRendererBP (BluePilot)
├── DriverStateRenderer (stock) → DriverStateRendererSP (SunnyPilot)
└── CameraView → AugmentedRoadView (stock) → AugmentedRoadViewBP (BluePilot + BlindspotRendererMixin)
```

### How Conditional Wiring Works

In `selfdrive/ui/layouts/main.py`:
```python
if gui_app.sunnypilot_ui():
    from openpilot.selfdrive.ui.bp.onroad.augmented_road_view_bp import AugmentedRoadViewBP as AugmentedRoadView
```

This import aliasing replaces stock classes with BP versions at the layout wiring level. The BP classes inherit from stock, so all stock functionality is preserved while BP additions are layered on top.

### Why This Matters

- **Clean Upstream Merges:** SunnyPilot/OpenPilot updates can be merged cleanly since stock files are untouched
- **Fallback Behavior:** If BluePilot features fail, stock behavior remains as fallback
- **Runtime Toggling:** Features can be toggled on/off via Params without code changes
- **Clear Separation:** Each layer handles its own responsibilities without polluting other layers
- **Maintainability:** Bug fixes and features can be isolated to specific layers

---

## Directory Structure

```
BluePilotDev/
├── bluepilot/                    # BluePilot-specific code (portal, params, logger, UI widgets)
│   ├── backend/                  # HTTP/WebSocket server for BluePilot Portal
│   │   ├── bp_portal.py         # Main portal server entry point (~3984 lines)
│   │   ├── config.py            # Centralized server configuration
│   │   ├── core/                # Thread-safe state, logging, lifecycle
│   │   ├── routes/              # Route processing pipeline (GPS, geocoding, scanning)
│   │   ├── video/               # FFmpeg management, HEVC→MP4 remux, export
│   │   ├── realtime/            # WebSocket broadcaster, log streaming
│   │   ├── cache/               # LRU metrics/thumbnail/remux caching
│   │   ├── logs/                # Cereal message parsing
│   │   ├── network/             # WiFi IP detection, onroad checks
│   │   ├── storage/             # xattr-based route preservation
│   │   ├── system/              # CPU/memory/disk metrics
│   │   ├── params/              # Parameter manager with fallback
│   │   ├── handlers/            # HTTP endpoint handlers
│   │   └── utils/               # File ops, power management
│   ├── params/                   # BluePilot parameter definitions
│   │   ├── bp_params.py         # Parameter initialization system
│   │   └── params.json          # 60+ parameter definitions (types, defaults, min/max)
│   ├── logger/                   # Queue-based rotating file logger
│   │   └── bp_logger.py         # Thread-safe logging with platform-aware paths
│   ├── ui/                       # Raylib-based UI components (6.0 refactor)
│   │   ├── lib/                 # Colors, constants, carrier mapping
│   │   └── widgets/             # Sidebar, metric cards, network cards, buttons
│   ├── web/                      # Web frontend (vanilla JS, zero npm deps)
│   │   ├── src/                 # Source HTML/CSS/JS
│   │   └── public/              # Deployed web assets
│   └── setup_web_routes.py      # Setup verification script
│
├── selfdrive/                    # Core driving logic (extended from openpilot)
│   ├── ui/                       # UI system with three-layer inheritance
│   │   ├── onroad/              # Stock openpilot onroad renderers (Layer 1)
│   │   │   ├── alert_renderer.py
│   │   │   ├── augmented_road_view.py
│   │   │   ├── cameraview.py
│   │   │   ├── driver_state.py
│   │   │   ├── hud_renderer.py
│   │   │   └── model_renderer.py
│   │   ├── sunnypilot/          # SunnyPilot extensions (Layer 2)
│   │   │   ├── onroad/
│   │   │   │   ├── hud_renderer.py      # HudRendererSP
│   │   │   │   ├── driver_state.py      # DriverStateRendererSP
│   │   │   │   ├── model_renderer.py    # ModelRendererSP (mixin)
│   │   │   │   └── developer_ui/        # Dev diagnostic overlay
│   │   │   ├── layouts/settings/        # 19+ SP settings screens
│   │   │   └── mici/                    # MICI device SP components
│   │   ├── bp/                  # BluePilot overrides (Layer 3) ← MAIN BP UI CODE
│   │   │   ├── onroad/          # TICI device BP onroad renderers
│   │   │   │   ├── alert_renderer_bp.py       # Pill-shaped alerts
│   │   │   │   ├── augmented_road_view_bp.py  # Road view + blindspot
│   │   │   │   ├── blindspot_renderer.py      # Blindspot mixin
│   │   │   │   ├── hud_renderer_bp.py         # HUD + torque/powerflow/battery/road name
│   │   │   │   ├── hybrid_battery_gauge.py    # Battery SOC/voltage/amps display
│   │   │   │   ├── model_renderer_bp.py       # Path smoothing, torque coloring
│   │   │   │   └── powerflow_gauge.py         # Hybrid power flow arch gauge
│   │   │   ├── mici/onroad/    # MICI device BP renderers (smaller screen)
│   │   │   ├── layouts/settings/bluepilot.py  # BP settings menu (20+ toggles)
│   │   │   └── widgets/        # Float controls, QR dialog, input dialogs
│   │   ├── mici/               # MICI device base components
│   │   ├── layouts/            # Main layout wiring (conditional imports here)
│   │   │   ├── main.py         # THE KEY FILE - swaps stock for BP classes
│   │   │   ├── home.py
│   │   │   ├── sidebar.py
│   │   │   └── settings/
│   │   ├── lib/                # UI utility library
│   │   ├── widgets/            # Common UI widgets
│   │   └── translations/       # 32 language packs
│   ├── car/                     # Vehicle interfaces (mostly upstream)
│   ├── controls/                # Control loops (stock, extended by SP ControlsExt)
│   ├── modeld/                  # ML model inference
│   ├── monitoring/              # Driver monitoring
│   ├── selfdrived/              # Main self-drive daemon
│   └── assets/                  # Icons, images, sounds
│
├── sunnypilot/                   # SunnyPilot integration layer
│   ├── mads/                    # Modular Assistive Driving System (engagement state machine)
│   ├── mapd/                    # OSM map data management
│   ├── modeld/                  # Legacy SNPE model daemon
│   ├── modeld_v2/               # Tinygrad model daemon
│   ├── models/                  # Model selection/download manager (86+ models)
│   ├── sunnylink/               # Cloud connectivity, backups, device registration
│   ├── selfdrive/               # Control extensions
│   │   ├── controls/
│   │   │   ├── controlsd_ext.py          # ControlsExt - patches controlsd without modifying it
│   │   │   └── lib/
│   │   │       ├── nnlc/                 # Neural Network Lateral Control
│   │   │       ├── speed_limit/          # Speed Limit Assist (SLA)
│   │   │       ├── smart_cruise_control/ # Vision/map-based cruise
│   │   │       └── dec/                  # Dynamic Excessive Cranking (decel planner)
│   │   ├── car/
│   │   │   └── intelligent_cruise_button_management/  # ICBM
│   │   └── locationd/                    # Advanced localization
│   └── system/                  # Hardware extensions
│
├── opendbc_repo/                 # Vehicle protocol definitions (DBC files)
│   └── opendbc/car/ford/
│       └── carcontroller.py     # Ford CAN/CAN-FD controller (~1028 lines)
│                                # Contains BP lateral/longitudinal control logic
│                                # Feature-flagged via Params (not inheritance)
│
├── system/                       # System services
│   ├── manager/
│   │   ├── process_config.py    # Process registry (BP processes added via procs +=)
│   │   └── manager.py           # Process lifecycle manager
│   ├── camerad/                 # Camera capture
│   ├── loggerd/                 # Data logging
│   ├── athena/                  # Comma cloud connectivity
│   ├── updated/                 # Software update system
│   └── ui/                      # System-level UI
│
├── common/                       # Shared utilities (58 subdirectories)
├── cereal/                       # Communication protocol (Cap'n Proto messages)
├── panda/                        # Vehicle CAN bus interface hardware/firmware
├── msgq_repo/                    # Message queue system
├── rednose_repo/                 # State estimation library
├── tinygrad_repo/                # Neural network framework
│
├── BPVERSION                     # Current version (5.0.0)
├── BP_CHANGES.json               # Structured changelog
├── BP-5.0-RELEASE.md             # Release notes
├── SConstruct                    # SCons build configuration
├── pyproject.toml                # Python project config (3.11-3.13)
└── .env                          # Dev environment (ZMQ=1 on macOS)
```

---

## Ford-Specific Control System

Ford vehicles require special CAN messaging protocols for lateral and longitudinal control. BluePilot implements these in `opendbc_repo/opendbc/car/ford/carcontroller.py`.

### Lateral Control (Steering)

BluePilot uses Ford's **4-signal lateral control protocol**:

1. **Curvature** - Desired path curvature
2. **Path Angle** - Angle to target path
3. **Path Offset** - Lateral offset from lane center
4. **Curvature Rate** - Rate of change in curvature

#### Key Features
- **Anti-Overshoot Logic:** Prevents oscillation after steering corrections
- **Ford-Specific Acceleration Limits:** Respects vehicle's lateral acceleration constraints
- **Human Turn Detection:** 3-second latch when driver overrides steering
- **Advanced Lane Positioning (ALP):** Configurable in-lane offset (-0.5m to +0.5m)
- **High/Low Curvature Modes:** Separate PID gains for different curvature scenarios
- **Lane Change Factor Tuning:** Adjustable responsiveness during lane changes

#### Tuning Parameters (exposed in UI)
- `disable_BP_lat_UI` - Enable/disable BP lateral control
- `lane_change_factor_high` - High-speed lane change factor (0.5-1.0)
- `custom_path_offset` - In-lane positioning (-0.5 to 0.5m)
- `LC_PID_gain_UI` - Low curvature PID gain (0.0-5.0)
- `HC_PID_gain_UI` - High curvature PID gain (0.0-5.0)
- `pc_blend_ratio_high_C_UI` - Predicted curvature blend for high curvature (0.0-1.0)
- `pc_blend_ratio_low_C_UI` - Predicted curvature blend for low curvature (0.0-1.0)
- `enable_human_turn_detection` - Human steering override detection

### Longitudinal Control (Speed/Braking)

BluePilot implements intelligent speed and braking control for Ford vehicles.

#### Key Features
- **Lead-Aware Gas Limiting:** Won't accelerate into traffic ahead
- **Rate-Limited Transitions:** Smooth acceleration/deceleration transitions
- **Smart Brake/Precharge Actuation:** Gradual pressure buildup with hysteresis
- **Creep Compensation:** Smooth stop/start behavior
- **Urban Speed Threshold:** BP long only activates above 45 mph with no slow lead
- **CRITICAL SAFETY:** Never allows gas and brake simultaneously

#### Tuning Parameters (exposed in UI)
- `disable_BP_long_UI` - Enable/disable BP longitudinal control

### Implementation Location

All Ford-specific control logic is in:
```
opendbc_repo/opendbc/car/ford/carcontroller.py
```

**Note:** Unlike UI components, carcontroller.py uses **feature flags via Params** rather than inheritance. This is because it directly interfaces with vehicle CAN protocols and must maintain compatibility with stock OpenPilot's vehicle interface architecture.

---

## BluePilot Portal

The BluePilot Portal is a web-based interface for managing routes, viewing system metrics, and configuring device settings. It runs on the comma 3X/MICI device and is accessible via browser.

### Architecture

- **Zero-Dependency HTTP Server:** Uses Python stdlib `http.server`
- **WebSocket Support:** Optional real-time updates (port 8089)
- **HTTP Polling Fallback:** Works without WebSocket support
- **Access URL:** `http://[device-ip]:8088`
- **Onroad Safety:** Modification endpoints return 403 when driving
- **Thread-Safe:** Uses RLock on all mutable state

### Main Components

#### Backend Server
- **Location:** `bluepilot/backend/bp_portal.py` (~3984 lines)
- **Config:** `bluepilot/backend/config.py`
- **Architecture:** Modular route/video/realtime/cache/system subsystems

#### Web Frontend
- **Location:** `bluepilot/web/`
- **Technology:** Vanilla JavaScript, zero npm dependencies
- **Source:** `bluepilot/web/src/`
- **Deployed:** `bluepilot/web/public/`

### API Endpoints

#### Route Management
- `GET /api/routes` - List all routes with metadata (location, distance, duration)
- `GET /api/video/:baseName/:segment/:camera` - Stream video with byte-range support
- `GET /api/thumbnail/:baseName` - Get route thumbnail image
- `POST /api/preserve/:baseName` - Toggle route preservation (prevent auto-deletion)
- `DELETE /api/delete/:baseName` - Delete route (blocked when driving)

#### System Information
- `GET /api/system/metrics` - CPU/memory/disk usage
- `GET /api/system/info` - Device info, version, network status

#### WebSocket Events
- `routes_updated` - Route list changed
- `processing_update` - Route processing progress
- `export_progress` - Video export progress
- `param_updated` - Parameter changed
- `system_metrics` - Real-time system metrics

### Background Processing

The portal includes intelligent background processing:

- **Route Preprocessor:** Runs when device is idle (screen off + not driving)
- **GPS Metrics Extraction:** Parse cereal logs for route metadata
- **Reverse Geocoding:** Convert GPS to location names (rate-limited for Nominatim)
- **Persistent Caching:** Metrics, thumbnails, remuxed videos cached across reboots
- **Video Remuxing:** HEVC→MP4 conversion for browser compatibility

### Video Processing

- **FFmpeg Management:** Automatic FFmpeg binary detection
- **HEVC→MP4 Remux:** Browser-compatible video conversion
- **Export Functionality:** Package routes for download
- **Byte-Range Support:** Efficient video streaming

---

## Parameter System

BluePilot uses a comprehensive parameter system for configuration and feature toggling. Parameters are persistent across reboots and can be modified via UI or API.

### Files

- **Definitions:** `bluepilot/params/params.json` (60+ parameters)
- **Initialization:** `bluepilot/params/bp_params.py`

### Parameter Types

- `bool` - Boolean flag (true/false)
- `int` - Integer value with min/max clamping
- `float` - Floating-point value with min/max clamping
- `string` - Text value
- `time` - Timestamp value
- `json` - JSON object
- `bytes` - Binary data

### Parameter Categories

#### Ford Preferences (`FordPref*`)
- `FordPrefHybridVehicle` - Enable hybrid vehicle features
- `FordPrefEnableDebugLogs` - Enable debug logging

#### BluePilot Portal (`BPPortal*`)
- `BPPortalEnabled` - Enable portal server
- `BPPortalPort` - HTTP server port (default 8088)
- `BPPortalWebSocketPort` - WebSocket port (default 8089)

#### BluePilot Features (`BP*`)
- `disable_BP_lat_UI` - Disable BP lateral control
- `disable_BP_long_UI` - Disable BP longitudinal control
- `lane_change_factor_high` - Lane change factor (0.5-1.0)
- `custom_path_offset` - In-lane offset (-0.5 to 0.5m)
- `LC_PID_gain_UI` - Low curvature PID gain (0.0-5.0)
- `HC_PID_gain_UI` - High curvature PID gain (0.0-5.0)

#### Display Toggles
- `ShowTorque` - Display steering torque gauge
- `ShowBatteryGauge` - Display hybrid battery gauge
- `ShowPowerFlowGauge` - Display power flow gauge
- `BlindSpot` - Enable blindspot display
- `RoadNameToggle` - Display current road name

### Param Property Mapping

Parameters map to two main property structures:

- **`ccProp`** - CarController properties (control system tuning)
- **`interfaceProp`** - UI properties (display toggles, visual settings)

### Usage Example

```python
from bluepilot.params.bp_params import BPParams

bp_params = BPParams()

# Read parameter
is_hybrid = bp_params.get_bool("FordPrefHybridVehicle")

# Write parameter
bp_params.put("ShowTorque", "1")

# Get numeric with clamping
pid_gain = bp_params.get_float("LC_PID_gain_UI", min_val=0.0, max_val=5.0)
```

---

## SunnyPilot Features

BluePilot inherits and extends features from SunnyPilot. These features are fully integrated and available to BluePilot users.

### MADS - Modular Assistive Driving System
- **Location:** `sunnypilot/mads/`
- **Purpose:** Decoupled lateral/longitudinal engagement state machine
- **Benefit:** Independent control of steering and speed systems

### NNLC - Neural Network Lateral Control
- **Location:** `sunnypilot/selfdrive/controls/lib/nnlc/`
- **Purpose:** ML-based steering control
- **Benefit:** Smoother, more natural steering behavior

### ICBM - Intelligent Cruise Button Management
- **Location:** `sunnypilot/selfdrive/car/intelligent_cruise_button_management/`
- **Purpose:** Automatic speed matching based on cruise button presses
- **Benefit:** Simplifies speed adjustments

### SLA - Speed Limit Assist
- **Location:** `sunnypilot/selfdrive/controls/lib/speed_limit/`
- **Purpose:** Vision + map + PCM fusion for speed limit detection
- **Benefit:** Automatic speed limit awareness

### MAPD - Map Data Management
- **Location:** `sunnypilot/mapd/`
- **Purpose:** OpenStreetMap data download and management
- **Benefit:** Enhanced navigation and speed limit data

### Sunnylink
- **Location:** `sunnypilot/sunnylink/`
- **Purpose:** Cloud connectivity, backups, device registration
- **Benefit:** Remote device management and data backup

### Model Manager
- **Location:** `sunnypilot/models/`
- **Purpose:** 86+ driving model selection with async download
- **Benefit:** Choose optimal model for driving conditions

### ControlsExt
- **Location:** `sunnypilot/selfdrive/controls/controlsd_ext.py`
- **Purpose:** Extends controlsd without modifying stock code
- **Benefit:** Clean integration of SP control features

---

## UI System Architecture

The UI system demonstrates the three-layer inheritance architecture in action. Understanding this is critical for UI development.

### Layer 1: Stock OpenPilot
**Location:** `selfdrive/ui/onroad/`

Base renderers:
- `alert_renderer.py` - Alert display
- `augmented_road_view.py` - Main road view
- `cameraview.py` - Camera feed
- `driver_state.py` - Driver monitoring display
- `hud_renderer.py` - Heads-up display
- `model_renderer.py` - Model path visualization

### Layer 2: SunnyPilot Extensions
**Location:** `selfdrive/ui/sunnypilot/onroad/`

Extended renderers:
- `hud_renderer.py` → `HudRendererSP` - Extended HUD features
- `driver_state.py` → `DriverStateRendererSP` - Enhanced driver monitoring
- `model_renderer.py` → `ModelRendererSP` - Model visualization mixin
- `developer_ui/` - Developer diagnostic overlay

### Layer 3: BluePilot Customizations
**Location:** `selfdrive/ui/bp/onroad/` (TICI) and `selfdrive/ui/bp/mici/onroad/` (MICI)

#### TICI Device Renderers
- `alert_renderer_bp.py` → `AlertRendererBP` - Pill-shaped alerts with custom styling
- `augmented_road_view_bp.py` → `AugmentedRoadViewBP` - Road view + blindspot integration
- `blindspot_renderer.py` → `BlindspotRendererMixin` - Blindspot visualization mixin
- `hud_renderer_bp.py` → `HudRendererBP` - HUD + torque/powerflow/battery/road name
- `hybrid_battery_gauge.py` → `HybridBatteryGauge` - Battery SOC/voltage/amps display
- `model_renderer_bp.py` → `ModelRendererBP` - Path smoothing, torque coloring
- `powerflow_gauge.py` → `PowerFlowGauge` - Hybrid power flow arch gauge

#### MICI Device Renderers
**Location:** `selfdrive/ui/bp/mici/onroad/`
- Smaller screen adaptations of TICI renderers
- Simplified layouts for MICI display constraints

### Layout Wiring
**Location:** `selfdrive/ui/layouts/main.py`

This is the **key file** that wires everything together using conditional imports:

```python
if gui_app.sunnypilot_ui():
    from openpilot.selfdrive.ui.bp.onroad.augmented_road_view_bp import AugmentedRoadViewBP as AugmentedRoadView
    from openpilot.selfdrive.ui.bp.onroad.alert_renderer_bp import AlertRendererBP as AlertRenderer
    from openpilot.selfdrive.ui.bp.onroad.hud_renderer_bp import HudRendererBP as HudRenderer
    from openpilot.selfdrive.ui.bp.onroad.model_renderer_bp import ModelRendererBP as ModelRenderer
else:
    # Fall back to stock renderers
    from openpilot.selfdrive.ui.onroad.augmented_road_view import AugmentedRoadView
    from openpilot.selfdrive.ui.onroad.alert_renderer import AlertRenderer
    from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer
    from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer
```

### Settings UI
**Location:** `selfdrive/ui/bp/layouts/settings/bluepilot.py`

The BluePilot settings menu contains 20+ toggles for:
- Control system tuning (lateral/longitudinal)
- Display preferences (gauges, overlays)
- Feature enablement (blindspot, road names)
- Debug options

---

## Messaging System

BluePilot uses Cereal (Cap'n Proto) for inter-process communication. Messages flow between processes over ZMQ (or msgq on device).

### Message Types

#### Stock OpenPilot Messages
- `carState` - Current vehicle state (speed, steering angle, etc.)
- `carControl` - Control commands (gas, brake, steer)
- `selfdriveState` - Self-drive system state
- `modelV2` - Driving model output
- `liveCalibration` - Camera calibration
- `liveParameters` - Live parameter estimates

#### SunnyPilot Extensions
- `carControlSP` - SunnyPilot control extensions
- `selfdriveStateSP` - SP state extensions
- `longitudinalPlanSP` - SP longitudinal planning
- `carParamsSP` - SP car parameters

#### BluePilot Extensions
- `carStateBP` - BluePilot state additions
  - `hybridDrive` - Hybrid system state
  - `hybridBattery` - Battery SOC/voltage/current
  - `brakeLightStatus` - Brake light status
  - `blindspot` - Blindspot detection

### Message Flow Example

```
camerad → modelV2 → modeld → selfdriveState → controlsd → carControl → carcontroller.py → CAN bus
                                                    ↓
                                              carStateBP (from CAN)
                                                    ↓
                                              UI renderers (display)
```

### Cereal Schema Location
**Location:** `cereal/` directory

To add new messages or fields, modify the `.capnp` schema files and rebuild.

---

## Build System

BluePilot uses SCons for building the project.

### Build Configuration
- **Main File:** `SConstruct`
- **Python Support:** 3.11-3.13
- **C/C++ Compilation:** For performance-critical code
- **macOS Development:** `ZMQ=1` in `.env` (uses ZMQ instead of msgq)

### Common Build Commands

```bash
# Full build
scons -j$(nproc)

# Build specific target
scons -j$(nproc) selfdrive/ui/

# Clean build
scons -c

# Run tests
pytest -n auto
```

### Development Environment

For macOS development, set in `.env`:
```
ZMQ=1
```

This uses ZMQ instead of msgq for message passing, which is compatible with macOS.

---

## Development Conventions

Follow these conventions to maintain codebase consistency and enable clean upstream merges.

### File Naming
- BluePilot files use `_bp` suffix: `alert_renderer_bp.py`, `hud_renderer_bp.py`
- SunnyPilot files use `_sp` suffix: `controlsd_ext.py` (or `SP` in class name)
- Stock OpenPilot files have no suffix

### Class Naming
- BluePilot classes use `BP` suffix: `AlertRendererBP`, `HudRendererBP`
- SunnyPilot classes use `SP` suffix: `HudRendererSP`, `ControlsExt`
- Stock OpenPilot classes have no suffix

### Directory Structure
- BluePilot UI: `selfdrive/ui/bp/`
- BluePilot processes: `bluepilot/`
- SunnyPilot extensions: `sunnypilot/`
- Stock OpenPilot: `selfdrive/`, `system/`, `common/`

### Modification Rules

**DO:**
- Create new files that inherit from stock classes
- Use conditional imports in layout wiring files
- Add new processes via `procs +=` in `process_config.py`
- Use feature flags via Params for runtime toggling
- Place Ford-specific logic in `opendbc_repo/opendbc/car/ford/`
- Wrap any changes to upstream files in `# BluePilot: <description>` / `# End BluePilot` comments

**DON'T:**
- Make unnecessary changes to stock OpenPilot or SunnyPilot files
- Replace stock processes (always add alongside)
- Use compile-time flags for features (use Params instead)
- Modify upstream files without comment markers (makes merges difficult)

### Parameter Naming
- Ford-specific: `FordPref*` prefix
- BluePilot features: `BP*` prefix
- Display toggles: `Show*` or descriptive names
- Control tuning: descriptive names with `_UI` suffix for user-facing params

### Code Style
- Follow PEP 8 for Python code
- Use type hints where appropriate
- Document complex logic with comments
- Include docstrings for public methods

---

## Testing

BluePilot uses pytest for testing.

### Test Configuration
**Location:** `pyproject.toml`

```toml
[tool.pytest.ini_options]
testpaths = ["selfdrive", "system", "tools", "common"]
addopts = "-n auto"
markers = [
    "slow: marks tests as slow",
    "tici: marks tests that require TICI device"
]
```

### Running Tests

```bash
# Run all tests with parallelization
pytest -n auto

# Run specific test file
pytest selfdrive/test/test_car_models.py

# Run with markers
pytest -m "not slow"  # Skip slow tests
pytest -m tici        # Only device tests

# Run with coverage
pytest --cov=selfdrive --cov-report=html
```

### Test Locations
- Process-specific tests in module directories
- Portal testing: `bluepilot/test_web_routes.py` (uses mock Params for dev machines)
- Integration tests in `selfdrive/test/`

### Writing Tests

When adding new features, include tests:
- Unit tests for individual functions
- Integration tests for process interactions
- UI tests for renderer behavior (if applicable)

---

## Logging

BluePilot includes a sophisticated logging system for debugging and monitoring.

### BluePilot Logger
**Location:** `bluepilot/logger/bp_logger.py`

#### Features
- Queue-based threading for non-blocking logs
- Rotating file logs (10MB × 10 backups)
- Platform-aware log paths
- Thread-safe operation

#### Log Paths

**TICI Device:**
```
/data/logs/bp_logger/bluepilot.log
```

**macOS:**
```
~/Library/Logs/bluepilot/bluepilot.log
```

**Linux:**
```
~/.local/share/bluepilot/logs/bluepilot.log
```

### Log Levels

- `DEBUG` - Detailed diagnostic information (enabled via `FordPrefEnableDebugLogs`)
- `INFO` - General information messages
- `WARNING` - Warning messages
- `ERROR` - Error messages
- `CRITICAL` - Critical errors

### Usage Example

```python
from bluepilot.logger.bp_logger import get_logger

logger = get_logger(__name__)

logger.info("Starting BluePilot Portal")
logger.debug(f"Route count: {len(routes)}")
logger.warning("Failed to geocode route")
logger.error("FFmpeg process failed", exc_info=True)
```

### Debug Logging

Enable debug logs via parameter:
```python
bp_params.put("FordPrefEnableDebugLogs", "1")
```

### Viewing Logs

**On Device:**
```bash
tail -f /data/logs/bp_logger/bluepilot.log
```

**On Dev Machine:**
```bash
tail -f ~/Library/Logs/bluepilot/bluepilot.log  # macOS
tail -f ~/.local/share/bluepilot/logs/bluepilot.log  # Linux
```

---

## Quick Reference Guide

This section provides quick answers to common development tasks.

### Adding a New Onroad UI Element

1. Create new file in `selfdrive/ui/bp/onroad/` (e.g., `my_widget_bp.py`)
2. Inherit from stock Widget class
3. Override `render()` method with BP additions
4. Wire into `augmented_road_view_bp.py`
5. Add feature toggle in `bluepilot/params/params.json`

**Example:**
```python
from openpilot.selfdrive.ui.widgets import Widget

class MyWidgetBP(Widget):
    def __init__(self, parent, enabled_param="ShowMyWidget"):
        super().__init__(parent)
        self.enabled_param = enabled_param

    def render(self, painter):
        if not self.params.get_bool(self.enabled_param):
            return
        # Your rendering code here
```

### Adding a New Settings Toggle

1. Define parameter in `bluepilot/params/params.json`:
```json
{
    "name": "MyNewFeature",
    "type": "bool",
    "default": "0",
    "description": "Enable my new feature"
}
```

2. Add toggle in `selfdrive/ui/bp/layouts/settings/bluepilot.py`:
```python
self.add_toggle("My New Feature", "MyNewFeature")
```

3. Read parameter in your code:
```python
enabled = bp_params.get_bool("MyNewFeature")
```

### Modifying Ford Lateral Control

**File:** `opendbc_repo/opendbc/car/ford/carcontroller.py`

1. Locate the lateral control section (search for "4-signal lateral control")
2. Add parameter to `bluepilot/params/params.json` if needed
3. Read parameter in `carcontroller.py`
4. Modify control logic with feature flag check
5. Test thoroughly on vehicle

### Adding a New Web API Endpoint

**File:** `bluepilot/backend/bp_portal.py`

1. Add handler method:
```python
def handle_my_endpoint(self, query_params):
    # Your logic here
    return {"success": True, "data": result}
```

2. Register route in `do_GET()` or `do_POST()`:
```python
elif self.path.startswith("/api/my-endpoint"):
    data = self.handle_my_endpoint(query_params)
    self.send_json_response(data)
```

3. Add corresponding frontend code in `bluepilot/web/src/`

### Adding a New BluePilot Process

1. Create process in `bluepilot/my_process/my_process.py`
2. Register in `system/manager/process_config.py`:
```python
procs += [
    ("my_process", ("bluepilot.my_process.my_process", ["MyProcess"])),
]
```
3. Handle lifecycle (start/stop/restart)
4. Add tests in `bluepilot/my_process/test_my_process.py`

### Modifying Alert Display

**File:** `selfdrive/ui/bp/onroad/alert_renderer_bp.py`

1. Locate `AlertRendererBP` class
2. Override `render()` method or specific alert methods
3. Modify colors, shapes, text, or positioning
4. Test with different alert types

### Modifying HUD Elements

**File:** `selfdrive/ui/bp/onroad/hud_renderer_bp.py`

1. Locate `HudRendererBP` class
2. Add/modify gauge in appropriate method:
   - `render_speed_limit()`
   - `render_max_speed()`
   - `render_blindspot()` (if using BlindspotRendererMixin)
   - `render_road_name()`
3. Add feature toggle parameter
4. Test on device

### Key Files by Task

| Task | File(s) |
|------|---------|
| **Add onroad UI element** | `selfdrive/ui/bp/onroad/` + wire in `augmented_road_view_bp.py` |
| **Add settings toggle** | `selfdrive/ui/bp/layouts/settings/bluepilot.py` + `bluepilot/params/params.json` |
| **Modify Ford lateral control** | `opendbc_repo/opendbc/car/ford/carcontroller.py` |
| **Modify Ford longitudinal control** | `opendbc_repo/opendbc/car/ford/carcontroller.py` |
| **Add web API endpoint** | `bluepilot/backend/bp_portal.py` |
| **Add BluePilot process** | `bluepilot/` + `system/manager/process_config.py` |
| **Modify alert display** | `selfdrive/ui/bp/onroad/alert_renderer_bp.py` |
| **Modify HUD elements** | `selfdrive/ui/bp/onroad/hud_renderer_bp.py` |
| **Add parameter** | `bluepilot/params/params.json` + `bluepilot/params/bp_params.py` |
| **Change UI layout wiring** | `selfdrive/ui/layouts/main.py` |
| **MICI device UI changes** | `selfdrive/ui/bp/mici/onroad/` |
| **Add hybrid gauge** | `selfdrive/ui/bp/onroad/hybrid_battery_gauge.py` or `powerflow_gauge.py` |
| **Modify blindspot display** | `selfdrive/ui/bp/onroad/blindspot_renderer.py` |
| **Add route processing** | `bluepilot/backend/routes/` |
| **Add video processing** | `bluepilot/backend/video/` |

---

## Critical Safety Notes

BluePilot is an advanced driver assistance system (ADAS). Safety is paramount. Follow these critical safety rules when developing.

### Control System Safety

1. **NEVER allow gas and brake simultaneously**
   - Implement mutex logic in carcontroller.py
   - If gas > 0, ensure brake = 0
   - If brake > 0, ensure gas = 0

2. **Rate-limit all control signals**
   - Sudden changes can destabilize vehicle
   - Use gradual transitions with configurable ROC limits
   - Test limits thoroughly on vehicle

3. **Lead-aware operation**
   - Never accelerate into traffic ahead
   - Implement minimum following distance
   - Respect user-configured following distance

4. **Human override detection**
   - Detect and respect driver steering input
   - Implement 3-second override latch
   - Never fight driver input

5. **Brake precharge safety**
   - Use gradual pressure buildup
   - Avoid sudden brake application
   - Implement hysteresis to prevent oscillation

6. **Urban speed threshold**
   - BP longitudinal only activates above 45 mph with no slow lead
   - Prevents BP control in complex low-speed scenarios
   - Ensures driver maintains control in parking lots, intersections, etc.

### Portal Safety

1. **Onroad modification lockout**
   - Block all modification endpoints when vehicle is moving
   - Return HTTP 403 for dangerous operations
   - Allow read-only access when driving

2. **Parameter validation**
   - Clamp numeric values to safe ranges
   - Validate parameter types before writing
   - Reject invalid parameter values

3. **Thread safety**
   - Use RLock for all mutable state
   - Prevent race conditions in concurrent operations
   - Ensure atomic operations for critical state changes

### Testing Safety

1. **Test in simulation first**
   - Use carla or replay for initial testing
   - Verify control logic before vehicle testing

2. **Start with conservative tuning**
   - Use lower PID gains initially
   - Gradually increase to optimal values
   - Monitor for instability

3. **Test in safe environments**
   - Empty roads or parking lots for initial tests
   - Gradually progress to more complex scenarios
   - Always have driver ready to take over

4. **Monitor for edge cases**
   - Test in various weather conditions
   - Test with different traffic scenarios
   - Test with worn road markings

### Development Safety Rules

1. **Never modify control logic without thorough understanding**
   - Review existing code carefully
   - Understand Ford CAN protocol requirements
   - Consult with experienced developers

2. **Document safety-critical changes**
   - Add comments explaining safety considerations
   - Document testing performed
   - Note any limitations or edge cases

3. **Use feature flags for new control features**
   - Allow disabling via parameter
   - Provide fallback to stock behavior
   - Enable gradual rollout and testing

4. **Maintain upstream compatibility**
   - Don't break stock OpenPilot safety features
   - Preserve fallback behavior
   - Test with BP features disabled

---

## Conclusion

This document provides a comprehensive guide to the BluePilot codebase. Key takeaways:

1. **Inheritance Architecture:** Three-layer design keeps codebase maintainable and upstream-compatible
2. **Ford-Specific Control:** Advanced lateral/longitudinal control in carcontroller.py
3. **Parameter System:** Flexible feature toggling via Params
4. **BluePilot Portal:** Web-based management and monitoring
5. **Safety First:** Always prioritize safety in control system development

### Additional Resources

- **Repository:** https://github.com/BluePilotDev/bluepilot
- **Release Notes:** `BP-5.0-RELEASE.md`
- **Changelog:** `BP_CHANGES.json`
- **Version:** `BPVERSION`

### Getting Help

- Review existing code for patterns and examples
- Check parameter definitions in `bluepilot/params/params.json`
- Examine carcontroller.py for control system logic
- Study layout wiring in `selfdrive/ui/layouts/main.py`
- Read release notes for recent changes

### Contributing

When contributing to BluePilot:
1. Follow the inheritance architecture
2. Use proper naming conventions (`_bp` suffix, `BP` class suffix)
3. Add parameters for new features
4. Include tests for new functionality
5. Document safety considerations
6. Test thoroughly before submitting

---

**Document Version:** 1.0
**Last Updated:** 2026-02-16
**BluePilot Version:** 5.0.0 → 6.0.0 (in development)