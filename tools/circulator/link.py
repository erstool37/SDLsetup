"""The circulator's serial transport. Opening it is an ACTUATING operation.

.. danger::

   **OPENING THE PORT HARDWARE-RESETS THE MICROCONTROLLER.**

   The board is reached through an FTDI FT232R USB-serial bridge, and on such a
   board **DTR is capacitively coupled to /RESET**. The operating system
   asserts DTR when the port is opened -- before a single byte is sent -- so the
   open itself resets the MCU. There is no read-only identify query, no
   handshake, and no way to "just have a look".

   This module therefore treats ``open()`` the way the arm layer treats a
   commanded pose: gated behind an explicit permission flag
   (``allow_actuation``), refused without it, counted, and logged as a reset
   event. :meth:`SerialLink.port_present` exists so presence can be answered
   without any of that -- it uses ``stat`` only and never opens anything.

   After a reset the firmware needs time to boot. ``boot_settle_s`` is that
   window, and a frame attempted inside it raises rather than being sent into a
   booting MCU.

Design
======

**The transport is injected.** ``SerialLink(settings, transport=...)`` takes
any object with ``connect()``, ``close()`` and
``write_registers(address, values, device_id=...)``. The real client is built
lazily, by :meth:`_build_transport`, and **only** on a permitted open -- so a
dry-run run never constructs one, and the vendor package is imported inside
that method rather than at module level. Keeping the vendor import next to the
call that opens the port is what makes "import the package" and "touch the
hardware" two visibly separate events. :class:`FakeSerial` is what every test
uses.

**The write is verified.** A Modbus FC16 (write-multiple-registers) response
echoes the start address and the register count. The prior code assigned the
return value and never looked at it, so a refused or misdirected write was
indistinguishable from a successful one. :class:`WriteResult` carries the echo
and an ``ok`` that depends on it.

    **A failed write reports; it does not decide.** ``outcome="failed"`` is
returned, not raised, because whether to retry, abort, or carry on is the
calling script's decision and never this layer's. The exceptions this module *does* raise are
refusals to act at all -- no permission, no port, not open, still booting.
"""
from __future__ import annotations

import dataclasses
import inspect
import os
import time
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Any

from .codec import REGISTER_MAX
from .safety import ActuationNotAllowed, CirculatorError, SafetyError

if TYPE_CHECKING:  # pragma: no cover - typing only, never imported at runtime
    from .circulator import CirculatorSettings

#: Unit-id keyword names across client generations, most current first. The
#: installed client takes ``device_id``; older ones took ``slave`` and, before
#: that, ``unit``. Picked by introspection rather than by try/except so a real
#: TypeError from the call is not swallowed as a version difference.
_UNIT_KWARGS = ("device_id", "slave", "unit")


@dataclasses.dataclass(frozen=True)
class WriteResult:
    """What one register write actually did. Data, not a verdict.

    **Three outcomes, named explicitly**, because the interesting failure is not
    a rejected write -- it is a *plan* being mistaken for a completed one:

    ``"planned"``    a frame was composed and **nothing was sent**. Dry run.
    ``"confirmed"``  sent, and the FC16 echo verified against what was sent.
    ``"failed"``     sent or attempted, and **not** verified.

    :attr:`ok` is *derived*: it is True only for ``"confirmed"``. So a
    successful dry run reports ``ok=False, outcome="planned"``, and **that is
    not a bug** -- it is the honest answer to "was the setpoint written?", which
    is no. A caller who checks only ``ok`` therefore cannot mistake a plan for a
    write; it fails safe rather than merely being documented.

    :attr:`dry_run` is likewise derived from ``outcome``, never an independent
    field, so the two can never disagree.
    """

    PLANNED = "planned"
    CONFIRMED = "confirmed"
    FAILED = "failed"
    OUTCOMES = (PLANNED, CONFIRMED, FAILED)

    outcome: str
    address: int
    values: list[int]
    echo_address: int | None = None
    echo_count: int | None = None
    error: str | None = None

    def __post_init__(self) -> None:
        # A typo'd outcome would make `ok` silently False forever, which is the
        # quiet-wrong-answer class this whole type exists to remove.
        if self.outcome not in self.OUTCOMES:
            raise CirculatorError(
                f"outcome must be one of {self.OUTCOMES}, got {self.outcome!r}")
        if self.outcome == self.CONFIRMED and self.error is not None:
            raise CirculatorError(
                f"a confirmed write cannot also carry an error ({self.error!r})")

    @property
    def ok(self) -> bool:
        """True ONLY for a verified write. False for a dry run -- see the class docstring."""
        return self.outcome == self.CONFIRMED

    @property
    def dry_run(self) -> bool:
        """True when nothing was sent. Derived from ``outcome``, never stored."""
        return self.outcome == self.PLANNED

    @property
    def sent(self) -> bool:
        """Did a frame actually leave the host? False only for a plan."""
        return self.outcome != self.PLANNED

    def as_dict(self) -> dict:
        # Built explicitly, not via dataclasses.asdict: `ok` and `dry_run` are
        # properties, and a status page that lost them would show a plan and a
        # confirmed write as the same thing.
        return {"outcome": self.outcome, "ok": self.ok, "dry_run": self.dry_run,
                "sent": self.sent, "address": self.address,
                "values": list(self.values), "echo_address": self.echo_address,
                "echo_count": self.echo_count, "error": self.error}

    def describe(self) -> str:
        frame = ", ".join("0x%04X" % word for word in self.values)
        what = {self.PLANNED: "WOULD WRITE (nothing sent)",
                self.CONFIRMED: "wrote and confirmed",
                self.FAILED: "FAILED"}[self.outcome]
        tail = "" if self.error is None else "  [%s]" % self.error
        return "%s %d..%d = [%s]%s" % (
            what, self.address, self.address + len(self.values) - 1, frame, tail)


class SerialLink:
    """One serial session to the circulator. See the module docstring first."""

    def __init__(
        self,
        settings: CirculatorSettings,
        transport: Any = None,
        *,
        log: Callable[[str, str], None] | None = None,
        clock: Callable[[], float] | None = None,
        sleep: Callable[[float], None] | None = None,
    ) -> None:
        self.settings = settings
        self._transport = transport
        self._log_fn = log
        self._clock = clock or time.monotonic
        self._sleep = sleep or time.sleep
        self._open = False
        self._open_count = 0
        self._settle_deadline: float = 0.0
        self._last_event: dict | None = None

    # -- observation, which never touches the port ------------------------
    @property
    def transport(self) -> Any:
        """The injected or lazily built transport, or None if none exists yet.

        Reading this NEVER builds one -- a status page must be able to ask.
        """
        return self._transport

    @property
    def is_open(self) -> bool:
        return self._open

    @property
    def open_count(self) -> int:
        """Ports opened in this session -- i.e. **MCU reset events**."""
        return self._open_count

    @property
    def last_event(self) -> dict | None:
        return self._last_event

    @property
    def settle_remaining_s(self) -> float:
        """Seconds left of the post-reset boot window. 0.0 when clear or closed."""
        if not self._open:
            return 0.0
        return max(0.0, self._settle_deadline - self._clock())

    def port_present(self) -> dict:
        """Is the configured device node there? Answered by ``stat`` ALONE.

        This must never open the port: an open resets the MCU. So the answer is
        deliberately weak -- it says a path exists, not that anything is
        listening on it, and it cannot identify the device. That is the honest
        limit of what is knowable without actuating.
        """
        port = self.settings.port
        if port is None:
            return {"port": None, "present": False, "method": "stat",
                    "note": "no port configured; nothing was opened",
                    "detail": "TODO(operator): record the device path after "
                              "usbipd attach makes the bridge visible to WSL"}
        path = Path(str(port))
        try:
            present = path.exists()
            mode = os.stat(path).st_mode if present else None
        except OSError as exc:
            return {"port": str(port), "present": False, "method": "stat",
                    "error": str(exc), "note": "nothing was opened"}
        return {"port": str(port), "present": bool(present), "method": "stat",
                "st_mode": mode,
                "note": "path presence only -- the port was NOT opened, because "
                        "opening it asserts DTR and resets the MCU"}

    # -- actuation --------------------------------------------------------
    def _build_transport(self) -> Any:
        """Construct the real client. Called only on a permitted open."""
        # Deferred on purpose: see the module docstring. Importing the vendor
        # package is harmless, but keeping it here means the only code path that
        # can reach the hardware is also the only one that loads its driver.
        from pymodbus.client import ModbusSerialClient

        settings = self.settings
        return ModbusSerialClient(
            str(settings.port),
            baudrate=int(settings.baudrate),
            parity=str(settings.parity),
            stopbits=settings.stopbits,
            bytesize=int(settings.bytesize),
            timeout=float(settings.timeout_s),
        )

    def open(self, settle: bool = True) -> dict:
        """Open the port. **This resets the microcontroller.**

        Refuses unless ``allow_actuation`` is set and a port is configured.
        Idempotent while already open, because a redundant open would be a
        second, unasked-for reset.

        ``settle=True`` blocks for the remainder of ``boot_settle_s`` before
        returning, which is what a live run wants. ``settle=False`` returns
        immediately; the window is still enforced by
        :meth:`write_registers`, so skipping the wait cannot skip the guard.
        """
        if self._open:
            return dict(self._last_event or {}, already_open=True)
        settings = self.settings
        if not settings.allow_actuation:
            raise ActuationNotAllowed(
                "refusing to open the circulator port: allow_actuation is off. "
                "Opening this port asserts DTR on the FT232R bridge, which is "
                "capacitively coupled to /RESET -- the open itself RESETS the MCU. "
                "There is no read-only identify query; use port_present() to ask "
                "whether the device node exists."
            )
        if settings.port is None:
            raise CirculatorError(
                "refusing to open the circulator port: no port is configured. "
                "TODO(operator): make the FT232R bridge visible to WSL "
                "(usbipd bind --busid 2-1, then usbipd attach --wsl --busid 2-1, "
                "both needing Windows admin) and record the resulting device path."
            )
        if self._transport is None:
            self._transport = self._build_transport()
        self._emit("OPENING %s -- ASSERTS DTR, RESETS THE MCU" % settings.port, "warn")
        try:
            connected = self._transport.connect()
        except Exception as exc:
            raise CirculatorError(
                f"could not open the circulator port {settings.port}: {exc}"
            ) from exc
        if connected is False:
            raise CirculatorError(
                f"could not open the circulator port {settings.port}: the transport "
                f"reported failure. Nothing further was attempted."
            )
        self._open = True
        self._open_count += 1
        self._settle_deadline = self._clock() + float(settings.boot_settle_s)
        self._last_event = {
            "action": "open",
            "port": str(settings.port),
            "mcu_reset": True,
            "reason": "DTR asserted on open; DTR is coupled to /RESET on the FT232R",
            "open_count": self._open_count,
            "boot_settle_s": float(settings.boot_settle_s),
        }
        self._emit("open: MCU reset, waiting %.3g s for boot"
                   % float(settings.boot_settle_s))
        if settle:
            self.wait_for_settle()
        return dict(self._last_event)

    def wait_for_settle(self) -> float:
        """Block until the post-reset boot window has elapsed. Returns seconds waited."""
        remaining = self.settle_remaining_s
        if remaining > 0:
            self._sleep(remaining)
        return remaining

    def write_registers(self, address: int, values: Sequence[int]) -> WriteResult:
        """Write holding registers and CHECK WHAT CAME BACK.

        Returns a :class:`WriteResult`; a rejected or mismatched write is
        ``ok=False``, not an exception, because the response is a measurement
        and what to do about it is the caller's decision.

        Raises only when the write must not be attempted at all: the link is not
        open, or the MCU is still inside its post-reset boot window.
        """
        frame = self._check_frame(address, values)
        if not self._open:
            raise CirculatorError(
                "the circulator link is not open. Refusing to open it implicitly: "
                "an open RESETS the MCU, so it has to be an explicit, logged call."
            )
        remaining = self.settle_remaining_s
        if remaining > 0:
            raise SafetyError(
                f"refusing to write {remaining:.3g} s into the MCU's post-reset boot "
                f"window (boot_settle_s={float(self.settings.boot_settle_s):g}). The "
                f"port open reset the board; a frame sent while it is still booting is "
                f"neither delivered nor reported as lost."
            )
        try:
            response = self._send(address, frame)
        except Exception as exc:
            self._emit("write %d failed: %s" % (address, exc), "error")
            return WriteResult(outcome=WriteResult.FAILED, address=int(address),
                               values=frame,
                               error="transport raised %s: %s"
                                     % (type(exc).__name__, exc))
        return self._judge(int(address), frame, response)

    def close(self) -> None:
        """Close the port. Never raises -- a failed close must not mask a run's result."""
        if self._transport is not None and self._open:
            try:
                self._transport.close()
            except Exception as exc:
                self._emit("close failed: %s" % exc, "warn")
        self._open = False
        self._settle_deadline = 0.0

    def __enter__(self) -> SerialLink:
        self.open()
        return self

    def __exit__(self, *exc_info: object) -> None:
        self.close()

    # -- internals --------------------------------------------------------
    def _emit(self, message: str, level: str = "info") -> None:
        if self._log_fn is not None:
            self._log_fn(message, level)

    def _check_frame(self, address: int, values: Sequence[int]) -> list[int]:
        if isinstance(address, bool) or not isinstance(address, int):
            raise CirculatorError(f"register address must be an int, got {address!r}")
        if address < 0:
            raise CirculatorError(f"register address must be non-negative, got {address}")
        frame: list[int] = []
        for index, raw in enumerate(values):
            if isinstance(raw, bool) or not isinstance(raw, int):
                raise CirculatorError(f"register {index}: not an int: {raw!r}")
            if not 0 <= raw <= REGISTER_MAX:
                raise CirculatorError(
                    f"register {index}: {raw!r} outside 0..0x{REGISTER_MAX:04X}")
            frame.append(raw)
        if not frame:
            raise CirculatorError("refusing to write an empty register block")
        return frame

    def _unit_kwarg(self) -> str:
        """Which keyword this client generation uses for the slave/unit id."""
        try:
            parameters = inspect.signature(self._transport.write_registers).parameters
        except (TypeError, ValueError):
            return _UNIT_KWARGS[0]
        for name in _UNIT_KWARGS:
            if name in parameters:
                return name
        return _UNIT_KWARGS[0]

    def _send(self, address: int, frame: list[int]) -> Any:
        kwargs = {self._unit_kwarg(): int(self.settings.unit_id)}
        return self._transport.write_registers(int(address), frame, **kwargs)

    def _judge(self, address: int, frame: list[int], response: Any) -> WriteResult:
        echo_address = getattr(response, "address", None)
        echo_count = getattr(response, "count", None)
        echo_address = int(echo_address) if isinstance(echo_address, int) else None
        echo_count = int(echo_count) if isinstance(echo_count, int) else None
        is_error = False
        checker = getattr(response, "isError", None)
        if callable(checker):
            try:
                is_error = bool(checker())
            except Exception:
                is_error = True

        problems: list[str] = []
        if response is None:
            problems.append("no response object was returned")
        if is_error:
            problems.append("the device returned a Modbus exception response (%r)"
                            % (response,))
        if echo_address is None:
            problems.append("the response echoed no start address")
        elif echo_address != address:
            problems.append("the response echoed start address %d, not %d"
                            % (echo_address, address))
        if echo_count is None:
            problems.append("the response echoed no register count")
        elif echo_count != len(frame):
            problems.append("the response echoed a count of %d, not %d"
                            % (echo_count, len(frame)))

        if problems:
            self._emit("write %d NOT confirmed: %s" % (address, "; ".join(problems)),
                       "error")
            return WriteResult(outcome=WriteResult.FAILED, address=address,
                               values=frame,
                               echo_address=echo_address, echo_count=echo_count,
                               error="; ".join(problems))
        self._emit("write %d confirmed (%d registers echoed)" % (address, len(frame)))
        return WriteResult(outcome=WriteResult.CONFIRMED, address=address,
                           values=frame,
                           echo_address=echo_address, echo_count=echo_count)


# ---------------------------------------------------------------------------
# the fake every test uses
# ---------------------------------------------------------------------------

class _FakeResponse:
    """Stands in for a write-multiple-registers response."""

    def __init__(self, address: int, count: int, error: bool = False) -> None:
        self.address = address
        self.count = count
        self._error = error

    def isError(self) -> bool:  # noqa: N802 - the vendor spelling
        return self._error

    def __repr__(self) -> str:
        return "_FakeResponse(address=%r, count=%r, error=%r)" % (
            self.address, self.count, self._error)


class FakeSerial:
    """A recording stand-in for the real client. **The only transport tests use.**

    ``open_count`` is the number the tests care about most: on this hardware
    each open is an MCU reset, so a test asserting ``open_count == 0`` is
    asserting that no reset happened.

    Failure injection: ``fail_open`` (connect raises), ``fail_write``
    (write raises), ``error_response`` (a Modbus exception response), and
    ``echo_address`` / ``echo_count`` to echo something other than what was
    sent -- the case the prior code could not have noticed.
    """

    def __init__(self, *, fail_open: bool = False, fail_write: bool = False,
                 error_response: bool = False, echo_address: int | None = None,
                 echo_count: int | None = None) -> None:
        self.fail_open = fail_open
        self.fail_write = fail_write
        self.error_response = error_response
        self.echo_address = echo_address
        self.echo_count = echo_count
        self.open_count = 0
        self.close_count = 0
        self.connected = False
        self.frames: list[dict] = []

    def connect(self) -> bool:
        if self.fail_open:
            raise OSError("FakeSerial: refusing to connect (fail_open)")
        self.open_count += 1
        self.connected = True
        return True

    def close(self) -> None:
        self.close_count += 1
        self.connected = False

    def write_registers(self, address: int, values: Sequence[int], *,
                        device_id: int = 1,
                        no_response_expected: bool = False) -> _FakeResponse:
        if not self.connected:
            raise OSError("FakeSerial: write on a port that was never opened")
        frame = list(values)
        self.frames.append({"address": address, "values": frame,
                            "device_id": device_id})
        if self.fail_write:
            raise OSError("FakeSerial: write failed (fail_write)")
        echo_address = address if self.echo_address is None else self.echo_address
        echo_count = len(frame) if self.echo_count is None else self.echo_count
        return _FakeResponse(echo_address, echo_count, self.error_response)


__all__ = ["FakeSerial", "SerialLink", "WriteResult"]
