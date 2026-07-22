# Lateral Debug Graph on Your Phone — User Guide

The steering-angle debug view, on a screen big enough to actually read.

## What it is

A live graph of **Desired vs Actual steering angle** — the same comparison the
on-device lateral debug screen shows — served by the device to any phone on its
network. 20 updates per second, about 50 ms behind the wheel. Use it to watch how
tightly the car tracks its own commands while tuning, without squinting at the
device screen.

## Using it

1. Enable **Web Routes Server** in settings (the same toggle that powers route
   browsing).
2. Connect your phone to the device's hotspot (or have both on the same WiFi).
3. Open **`http://192.168.43.1:8088/lateral`** (hotspot) or `http://<device-ip>:8088/lateral`.
4. Add it to your home screen if you like — it behaves like an app and keeps your
   screen awake while open.

On screen: current mph, whether lateral is active, the live desired-minus-actual
error in degrees, your low/high adjustment factors, and the auto-calibration
status line. **Tap anywhere on the graph to freeze it** for a closer look; tap
again to resume. The scale adapts automatically — gentle highway corrections and
full-lock parking maneuvers both stay readable.

## Good to know

- **View-only.** Nothing on this page changes any setting; adjustments stay on
  the device menus.
- It costs nothing while closed: the data feed starts when the first phone
  connects and shuts off ten seconds after the last one leaves.
- If the connection drops (screen lock, walking away), the page shows
  "waiting for data…" and reconnects by itself — a frozen trace is never
  silently presented as live.
- Works while driving; that's the point. Have a passenger hold the phone.
