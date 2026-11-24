"""
BluePilot: the settings a host published this frame, by key (HOSTS.md). The lateral code reads every
setting through params/manifest.py's value(), which parses the published text and takes the manifest
default for a key that is missing, empty or unparsable. A host publishes every per-frame key anyway,
using its own registry default for one never written.
"""


class ParamStore:
  def __init__(self, pairs):
    """pairs: the (key, value) settings a host published this frame, value as bytes or text."""
    self._values = dict(pairs)

  def get(self, key):
    """The published value as text, or None if the host didn't publish the key."""
    v = self._values.get(key)
    return v.decode("utf-8", errors="replace") if isinstance(v, (bytes, bytearray)) else v
