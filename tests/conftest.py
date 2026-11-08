"""Shared pytest fixtures."""
from __future__ import annotations

import pytest

from drone_stack.bus import MessageBus
from drone_stack.utils.config import Config


@pytest.fixture
def config() -> Config:
    """The built-in default configuration (simulation mode)."""
    return Config.load()


@pytest.fixture
def bus() -> MessageBus:
    return MessageBus()


@pytest.fixture(autouse=True)
def _isolated_recordings(tmp_path_factory, monkeypatch) -> None:
    """Keep every test away from the aircraft's real recordings/.

    Tests build GcsHub(Config.load()), and its FlightRecorder resolves the
    repo-relative "recordings" dir - on the Pi, the live one. Constructing a
    recorder sweeps that dir and adopts its owed renders, so a pytest run in
    ~/drone_stack on 2026-09-23 deleted the running HD render's temp file and
    the frames of the flight it was rendering. Config.load() honours
    DRONE_<SECTION>_<KEY>, so this reaches every hub a test builds.
    """
    monkeypatch.setenv("DRONE_RECORDING_DIR",
                       str(tmp_path_factory.mktemp("recordings")))


@pytest.fixture(autouse=True)
def _isolated_receipts(tmp_path_factory, monkeypatch) -> None:
    """Keep every test away from the aircraft's real delivery ledger and key.

    Same hazard as recordings/ above: a release path exercised by a test on
    the Pi would otherwise append to the live receipts chain (a record the
    chain can never forget) and sign it with the drone's real key.
    """
    import sys
    d = tmp_path_factory.mktemp("receipts")
    monkeypatch.setenv("AERIX_RECEIPT_DIR", str(d / "receipts"))
    monkeypatch.setenv("AERIX_RECEIPT_KEY", str(d / "keys" / "k.pem"))
    # The live order file (order_store.py) - /run/aerix-ble on the Pi.
    monkeypatch.setenv("AERIX_ORDER_DIR", str(d / "orders"))
    # ...and its token-free spent markers, which otherwise leave a RAM order
    # dir for the persistent ble_handshake/ - on a host whose /tmp is tmpfs,
    # the live one.
    monkeypatch.setenv("AERIX_SPENT_DIR", str(d / "spent"))
    ble = sys.modules.get("drone_stack.ble_handshake.drone_ble_peripheral")
    if ble is not None:
        monkeypatch.setattr(ble, "_LEDGER", None)
