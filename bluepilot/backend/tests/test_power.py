import pytest

from bluepilot.backend.utils import power


@pytest.fixture
def restored(monkeypatch):
    """Record calls to restore_power_save instead of writing to sysfs."""
    calls = []
    monkeypatch.setattr(power, "restore_power_save", lambda: calls.append(True))
    return calls


def idle_for(monkeypatch, seconds):
    monkeypatch.setattr(power, "last_activity_time", power.time.time() - seconds)


def test_restores_power_save_when_idle_and_offroad(monkeypatch, restored):
    idle_for(monkeypatch, power.IDLE_TIMEOUT_SECONDS + 1)
    power.check_and_restore_power_save(lambda: False)
    assert restored == [True]


def test_never_restores_power_save_while_onroad(monkeypatch, restored):
    idle_for(monkeypatch, power.IDLE_TIMEOUT_SECONDS + 1)
    power.check_and_restore_power_save(lambda: True)
    assert restored == []


def test_keeps_performance_mode_until_idle_timeout(monkeypatch, restored):
    idle_for(monkeypatch, power.IDLE_TIMEOUT_SECONDS - 60)
    power.check_and_restore_power_save(lambda: False)
    assert restored == []
