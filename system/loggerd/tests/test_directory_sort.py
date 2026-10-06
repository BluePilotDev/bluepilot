from pathlib import Path

import pytest

from openpilot.system.loggerd.uploader import get_directory_sort, listdir_by_creation


@pytest.mark.parametrize("year", [2023, 2024, 2025, 2026, 2030])
def test_legacy_routes_sort_before_counter_routes(year):
  legacy = f"{year}-10-04--20-28-40--0"
  counter = "00000003--aaaaaaaaaa--0"
  assert sorted([counter, legacy], key=get_directory_sort) == [legacy, counter]


@pytest.mark.parametrize("route", ["2026-10-04--20-28-40", "00000003--aaaaaaaaaa"])
def test_segments_sort_numerically(route):
  expected = [f"{route}--{segment}" for segment in [0, 1, 9, 10, 20]]
  unsorted = [f"{route}--{segment}" for segment in [20, 0, 10, 9, 1]]
  assert sorted(unsorted, key=get_directory_sort) == expected


def test_mixed_routes_list_in_creation_order(tmp_path: Path):
  expected = [
    "2024-12-31--23-59-59--0",
    "2025-01-01--00-00-00--0",
    "2026-10-04--20-28-40--9",
    "2026-10-04--20-28-40--10",
    "2026-10-05--00-00-00--0",
    "00000003--aaaaaaaaaa--9",
    "00000003--aaaaaaaaaa--10",
    "00000004--bbbbbbbbbb--0",
  ]
  for name in reversed(expected):
    (tmp_path / name).mkdir()
  (tmp_path / "not-a-directory").touch()
  assert listdir_by_creation(str(tmp_path)) == expected
