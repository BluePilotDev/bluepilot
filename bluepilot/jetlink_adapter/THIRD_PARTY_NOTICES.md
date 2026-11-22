# JetLink: third-party notices

BluePilot's JetLink integration uses and adapts code from two projects by Zeph Leggett, both under the MIT License.

| Code | Source | Copyright | License |
| --- | --- | --- | --- |
| `jetlink_repo/` (submodule, pinned to `v0.9.0`, `9cfde98eec1d7cc714d800504944cf1a3944513d`) | [zoompilot/jetlink](https://github.com/zoompilot/jetlink) | Copyright (c) 2026 Zeph Leggett | MIT, `jetlink_repo/LICENSE` |
| `bluepilot/jetlink_adapter/__init__.py` (the guard, `_Off`, the hooks by name, `link_on()`, the `Adapter` overrides) | [zoompilot](https://github.com/zoompilot/zoompilot) `openpilot/sunnypilot/jetlink_adapter/__init__.py` | Copyright (c) 2026- Zeph Leggett | MIT |
| `bluepilot/system/manager/restarting_process.py` (`RestartingPythonProcess`) | zoompilot `openpilot/sunnypilot/system/manager/processes.py` | Copyright (c) 2026- Zeph Leggett | MIT |
| `bluepilot/selfdrive/selfdrived/jetlink_events.py` (`JetlinkEvents`) and the two alerts in `bluepilot/selfdrive/selfdrived/events_bp.py` | zoompilot `openpilot/sunnypilot/selfdrive/selfdrived/accelerator_events.py`, `events_zp.py` | Copyright (c) 2026- Zeph Leggett | MIT |
| `bluepilot/jetlink_adapter/catalog.py` (`JetlinkModelFetcher`, `BigCatalogCache`, `PickCarryOver`) | zoompilot `openpilot/sunnypilot/models/fetcher_zp.py`, `default_bootstrap.py` | Copyright (c) 2026- Zeph Leggett | MIT |
| `openpilot/selfdrive/ui/bp/lib/jetlink_status.py` (the progress, port and status lines) | zoompilot `openpilot/selfdrive/ui/sunnypilot/accelerator_link.py` | Copyright (c) 2026- Zeph Leggett | MIT |
| The `with_blendful()` approach in the adapter | zoompilot `openpilot/sunnypilot/modeld_v2/model_settings.py` | Copyright (c) 2026- Zeph Leggett | MIT |

zoompilot's own files are offered under the MIT License by their author (zoompilot's `NOTICE.md`, "zoompilot's own
contributions"). The rest of BluePilot stays under its existing licenses (`LICENSE`, `LICENSE.md`).

## MIT License

Copyright (c) 2026 Zeph Leggett

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
