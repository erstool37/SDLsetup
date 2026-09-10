#!/usr/bin/env python3
"""Import this FIRST -- before anything from ``scripts`` or ``tools`` -- in every
test that can reach an ``--execute`` path. It makes the real hardware
unreachable and the machine's live state unwritable, and it RAISES rather than
skipping: a tripwire that quietly allows the call is not a tripwire.

WHY IT EXISTS
  On 2026-09-09 the rig was ARMED. ``configs/config.yaml`` carried
  ``allow_actuation: true`` in both the ``environment`` and ``circulator``
  sections for a supervised kinetics run. ``dev/test.sh`` was then run
  wholesale, and ``test_start_kinetics.py``'s case named "execute=True is
  REFUSED while the config gates are shut" resolved that shipped config, found
  the gates OPEN, and fell straight through to ``hold_environment.run(args)`` --
  which has no transport injected, so it built a real Modbus client against the
  real PLC. Opening the circulator's port was next, and that hardware-resets the
  MCU, underneath a run that was already holding the loop.

  The test was not wrong about what it wanted to assert. It was wrong to let a
  config file decide whether a test touches hardware. Both halves of the fix are
  needed and both are here: the tests state their own gates instead of reading
  the shipped file, and this module removes the ability to construct a real
  transport at all.

WHAT IT BLOCKS
  Every real transport in this tree is built by a LATE import inside the method
  that needs it -- ``from pymodbus.client import ModbusTcpClient`` in
  ``tools/environment/plc.py`` (``PlcClient._ensure_transport``) and
  ``ModbusSerialClient`` in ``tools/circulator/link.py``
  (``SerialLink._build_transport``). A late import re-reads the attribute off
  the module on every call, so replacing those two names on ``pymodbus.client``
  catches both, whatever the config says. ``serial.Serial`` and the TCP forms of
  ``socket.connect`` / ``socket.create_connection`` are backstops for a path
  that stops going through pymodbus. AF_UNIX is deliberately left alone: no lab
  instrument is on a unix socket, and blocking it would break unrelated
  machinery for nothing.

WHAT IT REDIRECTS
  Three on-disk surfaces belong to the machine, not to a test process:

  * ``~/.sdl_lab/occupancy`` -- a live run's claim. A test must neither be
    REFUSED by the real claim (``hold_environment`` claims ``environment``, so
    every integration test would fail while a real hold is running) nor be able
    to clear it.
  * ``~/.sdl_lab/environment/last_reading.json`` -- what the dashboard renders.
    A test writing there shows the operator a fabricated chamber state, and the
    channel's ``command.json`` lives in the same directory.
  * the ``dataset/`` run tree. ``SDL_DATA_ROOT`` alone is NOT enough --
    ``runs.data_root`` resolves ``data.root`` from config.yaml FIRST, and the
    shipped file sets it to ``dataset``, so a test using the real config wrote
    its runs into the repo tree beside real ones. ``runs.data_root`` is
    redirected outright.

  All three move into one throwaway directory per test process.

    import no_hardware            # first import in the file
    no_hardware.selftest()        # asserts the tripwire actually fires
"""
from __future__ import annotations

import os
import socket
import sys
import tempfile
from pathlib import Path
from typing import Any

REPO = Path(__file__).resolve().parents[2]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

#: One throwaway tree per test process, holding everything a run would write.
TMP = Path(tempfile.mkdtemp(prefix="sdl-no-hardware-"))
PUBLISH_DIR = TMP / "environment"
OCCUPANCY_DIR = TMP / "occupancy"
DATA_ROOT = TMP / "dataset"

_INET_FAMILIES = (socket.AF_INET, socket.AF_INET6)
_installed = False


class HardwareTripwire(BaseException):
    """A test tried to reach real hardware. Nothing was opened.

    Derived from ``BaseException``, not ``Exception``, and that is the whole
    point. Every I/O boundary in this tree swallows ``Exception`` deliberately
    -- ``PlcClient.connect`` turns a refused socket into ``return False``,
    ``read_block`` turns a transport fault into ``read_ok=False``,
    ``start_kinetics._plan`` notes a failed read and carries on. Every one of
    those would absorb this tripwire and let the test continue believing the
    hardware was merely unreachable. It has to be louder than the code it is
    guarding, so it sits beside ``KeyboardInterrupt``.
    """


def _refuse(what: str) -> Any:
    """A stand-in for a real transport constructor: it only ever raises."""
    def constructor(*args: Any, **kwargs: Any) -> Any:
        raise HardwareTripwire(
            "%s was constructed from a test. dev/tests/no_hardware.py blocks "
            "every real transport -- drive the code over FakePlc / FakeSerial "
            "through the _plc_transport / _circ_transport seams instead. "
            "args=%r kwargs=%r" % (what, args, kwargs))
    return constructor


_real_connect = socket.socket.connect
_real_connect_ex = socket.socket.connect_ex
_real_create_connection = socket.create_connection


def _guarded_connect(self: socket.socket, address: Any, *args: Any,
                     **kwargs: Any) -> Any:
    if self.family in _INET_FAMILIES:
        raise HardwareTripwire(
            "a test opened a TCP connection to %r. The PLC is at 169.254.33.33; "
            "no test may reach it. Use FakePlc." % (address,))
    return _real_connect(self, address, *args, **kwargs)


def _guarded_connect_ex(self: socket.socket, address: Any, *args: Any,
                        **kwargs: Any) -> Any:
    if self.family in _INET_FAMILIES:
        raise HardwareTripwire(
            "a test called connect_ex(%r) on a TCP socket." % (address,))
    return _real_connect_ex(self, address, *args, **kwargs)


def _guarded_create_connection(address: Any, *args: Any, **kwargs: Any) -> Any:
    raise HardwareTripwire(
        "a test called socket.create_connection(%r)." % (address,))


def install() -> None:
    """Block the transports and redirect the state. Idempotent."""
    global _installed
    if _installed:
        return
    _installed = True

    for directory in (PUBLISH_DIR, OCCUPANCY_DIR, DATA_ROOT):
        directory.mkdir(parents=True, exist_ok=True)
    # Set before tools.runs resolves it -- which is why this module has to be
    # imported ahead of anything under tools/ or scripts/.
    os.environ["SDL_DATA_ROOT"] = str(DATA_ROOT)

    from tools import occupancy, runs
    from tools.environment import plc

    # Both are module globals read at call time (plc._publish_path,
    # occupancy._path, channel.channel_dir), and both say so in their
    # docstrings. Reassignment is the supported test seam.
    plc.PUBLISH_DIR = PUBLISH_DIR
    occupancy.OCCUPANCY_DIR = OCCUPANCY_DIR

    # data.root in configs/config.yaml outranks SDL_DATA_ROOT, so the env var
    # above only covers a test that supplies its own config. Replace the
    # resolver so no test can write a run into the repo's dataset/ tree, where
    # it would sit among real ones with a real-looking manifest.
    runs.data_root = lambda config=None: DATA_ROOT

    try:
        import pymodbus.client as pymodbus_client
    except ImportError:
        pass          # the late import inside the tree fails too; still safe
    else:
        pymodbus_client.ModbusTcpClient = _refuse(
            "pymodbus ModbusTcpClient (a real Modbus TCP socket to the PLC)")
        pymodbus_client.ModbusSerialClient = _refuse(
            "pymodbus ModbusSerialClient (the circulator's real serial port -- "
            "opening it hardware-resets the MCU)")

    try:
        import serial
    except ImportError:
        pass
    else:
        serial.Serial = _refuse("serial.Serial (a real serial port)")

    socket.socket.connect = _guarded_connect
    socket.socket.connect_ex = _guarded_connect_ex
    socket.create_connection = _guarded_create_connection


def selftest() -> None:
    """Prove the tripwire fires. Raises if it does not.

    Asked because a guard that cannot fail has not guarded anything: if the late
    import in ``plc.py`` ever stops resolving through ``pymodbus.client``, every
    test above this line goes quiet and starts talking to the PLC.
    """
    install()
    from tools.environment.plc import PlcClient, PlcSettings

    # connect() swallows Exception by design, so this only fires because
    # HardwareTripwire is a BaseException. If that ever changes, this call comes
    # back as a quiet False and the assertion below is the only thing that says
    # so.
    client = PlcClient(PlcSettings(host="169.254.33.33"))
    try:
        client.connect()
    except HardwareTripwire:
        pass
    else:
        raise AssertionError(
            "TRIPWIRE DID NOT FIRE: PlcClient.connect() reached its transport "
            "against the real PLC address without raising. Do not run this "
            "suite until it does.")

    for family in (socket.AF_INET, socket.AF_INET6):
        probe = socket.socket(family, socket.SOCK_STREAM)
        try:
            probe.connect(("169.254.33.33", 502))
        except HardwareTripwire:
            pass
        else:
            raise AssertionError(
                "TRIPWIRE DID NOT FIRE: a raw %r socket connected." % family)
        finally:
            probe.close()


def describe() -> str:
    return ("no_hardware: transports blocked; state redirected to %s "
            "(publish=%s occupancy=%s data=%s)"
            % (TMP, PUBLISH_DIR.name, OCCUPANCY_DIR.name, DATA_ROOT.name))


install()
