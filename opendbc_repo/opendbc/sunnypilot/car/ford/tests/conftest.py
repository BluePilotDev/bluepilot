"""Test bootstrap for environments without the compiled msgq extension.

These tests exercise pure control logic, but importing lateral_curv_ext pulls in
cereal.messaging, whose real implementation needs the msgq C++ extension (built on
device/CI, not on plain dev machines). If the real import works, nothing here has any
effect. If it fails, register a minimal stub so the module under test can import; the
tests themselves always patch SubMaster with their own fake.
"""
import sys
import types

try:
  import cereal.messaging  # noqa: F401
except Exception:
  _messaging = types.ModuleType("cereal.messaging")

  class _StubSubMaster:
    def __init__(self, *args, **kwargs):
      raise RuntimeError("cereal.messaging stub: tests must patch SubMaster (see _FakeSubMaster)")

  _messaging.SubMaster = _StubSubMaster
  _cereal = sys.modules.get("cereal") or types.ModuleType("cereal")
  _cereal.messaging = _messaging
  sys.modules["cereal"] = _cereal
  sys.modules["cereal.messaging"] = _messaging
