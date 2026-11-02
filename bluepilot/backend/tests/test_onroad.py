from bluepilot.backend.network import utils


class UnknownKeyName(Exception):
    pass


class StrictParams:
    """Like the real Params: reading a key that isn't declared raises."""

    def __init__(self, values):
        self._values = values

    def get_bool(self, key):
        if key not in self._values:
            raise UnknownKeyName(key)
        return self._values[key]


def test_onroad_when_device_is_onroad(monkeypatch):
    monkeypatch.setattr(utils, "params", StrictParams({"IsOnroad": True}))
    assert utils.is_onroad() is True


def test_offroad_when_device_is_offroad(monkeypatch):
    monkeypatch.setattr(utils, "params", StrictParams({"IsOnroad": False}))
    assert utils.is_onroad() is False


def test_assumes_onroad_when_state_cannot_be_read(monkeypatch, caplog):
    # Fail closed: a broken read must lock the portal, not unlock it.
    monkeypatch.setattr(utils, "params", StrictParams({}))
    assert utils.is_onroad() is True
    assert any(r.levelname == "WARNING" for r in caplog.records)
