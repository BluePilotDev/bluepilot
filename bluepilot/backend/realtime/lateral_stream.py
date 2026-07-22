"""
BluePilot portal: live lateral-debug feed for the phone graph (/lateral).

The MICI lateral debug screen shows desired vs actual steering angle on a display
that is honestly too small for more than a gut feeling. This module gives the same
signals to any phone on the device's hotspot/LAN as a 20 Hz snapshot stream the
portal serves over SSE.

Design mirrors realtime/log_streamer.py: messaging is imported lazily (the portal
must keep working on hosts without cereal), a single background reader thread owns
the SubMaster, and it starts on the first subscriber and stops after a short idle
so the portal costs nothing while nobody is watching.
"""

import threading
import time
import logging

logger = logging.getLogger(__name__)

_RATE_HZ = 20.0
_IDLE_STOP_S = 10.0     # reader stops this long after the last subscriber detaches


class LateralFeed:
    """Singleton owner of the messaging reader. Thread-safe snapshot access."""

    _instance = None
    _instance_lock = threading.Lock()

    @classmethod
    def instance(cls) -> "LateralFeed":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self):
        self._lock = threading.Lock()
        self._thread = None
        self._subscribers = 0
        self._last_sub_gone = 0.0
        self._seq = 0
        self._sample = {}

    # -- subscriber lifecycle -------------------------------------------------------------
    def attach(self):
        with self._lock:
            self._subscribers += 1
            if self._thread is None or not self._thread.is_alive():
                self._thread = threading.Thread(target=self._run, daemon=True,
                                                name="lateral_feed")
                self._thread.start()

    def detach(self):
        with self._lock:
            self._subscribers = max(0, self._subscribers - 1)
            if self._subscribers == 0:
                self._last_sub_gone = time.monotonic()

    def snapshot(self, last_seq: int):
        """(seq, sample) if newer than last_seq else (last_seq, None)."""
        with self._lock:
            if self._seq == last_seq:
                return last_seq, None
            return self._seq, dict(self._sample)

    # -- reader ---------------------------------------------------------------------------
    def _should_stop(self) -> bool:
        with self._lock:
            return (self._subscribers == 0
                    and time.monotonic() - self._last_sub_gone > _IDLE_STOP_S)

    def _run(self):
        try:
            import cereal.messaging as messaging
        except Exception as exc:
            logger.error("lateral feed: messaging unavailable: %s", exc)
            return
        try:
            sm = messaging.SubMaster(['carState', 'carControl', 'controllerStateBP'])
        except Exception as exc:
            logger.error("lateral feed: SubMaster failed: %s", exc)
            return
        logger.info("lateral feed: reader started")
        period = 1.0 / _RATE_HZ
        while not self._should_stop():
            t0 = time.monotonic()
            try:
                sm.update(int(period * 1000))
                cs = sm['carState']
                cc = sm['carControl']
                st = sm['controllerStateBP']
                sample = {
                    't': time.time(),
                    'desired_deg': float(cc.actuators.steeringAngleDeg),
                    'actual_deg': float(cs.steeringAngleDeg),
                    'v_mph': float(cs.vEgo) * 2.23694,
                    'lat_active': bool(cc.latActive),
                    'torque_nm': float(cs.steeringTorque),
                    'low_factor': float(st.bmsLowSpeedAdjustmentFactor),
                    'high_factor': float(st.bmsHighSpeedAdjustmentFactor),
                    # Fields other branches own read defensively: the page shows a dash
                    # instead of this stream dying on a schema without them (the auto-cal
                    # status string ships with the ford-angle-autocal branch).
                    'autocal': str(getattr(st, 'bmsAngleAutoCalState', '')),
                    'alive': bool(sm.alive['carState'] and sm.alive['carControl']),
                }
                with self._lock:
                    self._sample = sample
                    self._seq += 1
            except Exception as exc:
                # keep the reader alive through transient messaging hiccups
                logger.debug("lateral feed: update error: %s", exc)
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)
        logger.info("lateral feed: reader stopped (idle)")


def serve_sse(handler):
    """Write the SSE stream onto a BaseHTTPRequestHandler until the client leaves.

    One thread per watching phone (the portal is a ThreadingHTTPServer); frames are
    only sent when the feed sequence advances, so a paused car costs near nothing.
    """
    import json as _json

    feed = LateralFeed.instance()
    feed.attach()
    try:
        handler.send_response(200)
        handler.send_header('Content-Type', 'text/event-stream')
        handler.send_header('Cache-Control', 'no-cache')
        handler.send_header('Connection', 'keep-alive')
        handler.send_header('Access-Control-Allow-Origin', '*')
        handler.end_headers()
        seq = 0
        last_beat = time.monotonic()
        while True:
            seq, sample = feed.snapshot(seq)
            now = time.monotonic()
            if sample is not None:
                payload = f"data: {_json.dumps(sample, separators=(',', ':'))}\n\n"
                handler.wfile.write(payload.encode())
                handler.wfile.flush()
                last_beat = now
            elif now - last_beat > 5.0:
                # comment-frame keepalive so phones detect a dead link promptly
                handler.wfile.write(b": keepalive\n\n")
                handler.wfile.flush()
                last_beat = now
            time.sleep(1.0 / _RATE_HZ / 2.0)
    except (BrokenPipeError, ConnectionResetError, OSError):
        pass  # phone left / screen locked — normal end of stream
    finally:
        feed.detach()
