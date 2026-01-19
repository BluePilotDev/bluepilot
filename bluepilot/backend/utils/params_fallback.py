#!/usr/bin/env python3
"""
Params fallback helper for environments without openpilot Params.
Provides a minimal mock with sensible defaults for backend utilities.
"""

import logging
from typing import Dict, Any

logger = logging.getLogger(__name__)


def get_params_with_defaults(defaults: Dict[str, Any] | None = None):
    """Return openpilot Params when available, otherwise a lightweight mock."""
    defaults = defaults or {}
    try:
        from openpilot.common.params import Params  # type: ignore
        return Params()
    except ImportError:
        logger.debug("openpilot Params unavailable, using fallback")
    except Exception as exc:  # pragma: no cover - defensive
        logger.error(f"Unexpected Params import error: {exc}")

    class Params:
        def __init__(self, initial: Dict[str, Any] | None = None):
            self._params = dict(initial or {})

        def get_bool(self, key: str):
            return bool(self._params.get(key, False))

        def get(self, key: str, encoding=None):
            value = self._params.get(key, "")
            if encoding and isinstance(value, str):
                return value.encode(encoding)
            return value if isinstance(value, str) else str(value)

        def put(self, key: str, value):
            self._params[key] = value

        def put_bool(self, key: str, value: bool):
            self._params[key] = bool(value)

        def remove(self, key: str):
            self._params.pop(key, None)

    return Params(defaults)
