"""BluePilot (bp-full): settings the host publishes in CarControlSP.params."""


def published_bool(CC_SP, key: str, defaults: dict[str, bool]) -> bool:
  """The published value of a bool setting, or its default (BP7's) from defaults when the host doesn't publish it."""
  for p in CC_SP.params:
    if p.key == key:
      v = p.value.decode() if isinstance(p.value, (bytes, bytearray)) else str(p.value)
      return v.strip().lower() in ("1", "true")
  return defaults[key]
