"""Select the model catalog's tinygrad without changing the stock build runtime."""

import os
from pathlib import Path
import sys

PLAIN_MAGIC = b"SPTGPLN1"
V26_TINYGRAD_REF = "5d186bdb43c2bcc6c56b6ececf427c34bada0de9"
V26_RUNTIME_PATH = Path(__file__).resolve().parent / "tinygrad_v26_repo"


def is_plain_model(path: str) -> bool:
  with open(path, "rb") as model_file:
    return model_file.read(len(PLAIN_MAGIC)) == PLAIN_MAGIC


def configure_catalog_runtime() -> None:
  """Run before any tinygrad imports in the custom modeld process."""
  override = os.environ.get("COMBINED_MODEL_PKL")
  if override and os.path.isfile(override):
    use_v26 = is_plain_model(override)
  else:
    from openpilot.sunnypilot.models.helpers import get_active_bundle
    bundle = get_active_bundle()
    use_v26 = bundle is not None and bundle.minimumSelectorVersion == 21
  if not use_v26:
    return
  if not (V26_RUNTIME_PATH / "tinygrad" / "__init__.py").is_file():
    raise RuntimeError("The v26 model runtime is missing. Run git submodule update --init --recursive.")
  if (loaded := sys.modules.get("tinygrad")) is not None:
    if Path(loaded.__file__).resolve().is_relative_to(V26_RUNTIME_PATH):
      return
    raise RuntimeError("The v26 model runtime must be selected before importing tinygrad.")
  sys.path.insert(0, str(V26_RUNTIME_PATH))
