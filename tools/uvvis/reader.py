"""High-level device API for the BMG LABTECH SPECTROstar Nano.

This is the layer SDL workflows should import. It wraps :mod:`backend` (the
WSL→Windows COM bridge) in a device object with explicit, safe operations, and
keeps every hardware-moving call behind a motion gate.

    from tools.nodes.uv_vis import SpectroStarNano

    reader = SpectroStarNano(allow_motion=True)
    with reader.session():
        reader.plate_out()          # load a plate by hand or by arm
        reader.plate_in()
        reader.run_protocol("BCA 1")

All control goes over **DDE** (:mod:`dde`). The ActiveX/COM path was removed on
2026-07-29: it could only issue parameterless commands, so `Run` and `Temp`
returned success while never reaching the instrument. DDE handles both, so there
is one path instead of two silently-unequal ones.

Every command is proven against the control software's run log, because neither
transport reports failure reliably on its own.
"""
from __future__ import annotations

import re
import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

from .. import occupancy
from . import dde
from .config import Incubation, Interlock, Shaking, UvVisConfig, WellSelection
from .layout import LayoutPlan, windows_path

#: The control software's run log — the ONLY trustworthy confirmation that a
#: command reached the reader. See :meth:`SpectroStarNano.log_activity`.
RUN_LOG = Path(
    "/mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/SPECTROstar Nano.log"
)

DEFAULT_LOG_DIR = Path("/home/lamp/SDLsetup/dataset/uv_vis_runs")


class ReaderError(RuntimeError):
    """A reader operation failed."""


class MotionNotAllowed(PermissionError):
    """A hardware-moving operation was attempted while the gate was closed."""


@dataclass
class CarrierState:
    """Best-known plate-carrier position.

    Tracked optimistically: the control software's OUT status strings never
    marshal back through PowerShell, so we record what we commanded, not what
    the instrument reports. ``unknown`` until the first successful move.
    """

    position: str = "unknown"          # unknown | in | out
    changed_at: str | None = None

    def set(self, position: str) -> None:
        self.position = position
        self.changed_at = datetime.now(timezone.utc).isoformat(timespec="seconds")


@dataclass
class SpectroStarNano:
    """The reader as an object.

    :param allow_motion: master gate. Every method that physically moves the
        carrier or drives the incubator raises :class:`MotionNotAllowed` unless
        this is True. Mirrors ``allow_arm_motion`` on the robot arm.
    :param reader: control-software service name. Must be the UNDERSCORE form.
    :param timeout_s: per-command bridge timeout. The first command of a session
        may pay a ~45 s control-software launch, so keep this generous.
    """

    allow_motion: bool = False
    reader: str = "SPECTROstar_Nano"
    timeout_s: float = 240.0
    log_dir: Path = field(default_factory=lambda: DEFAULT_LOG_DIR)
    carrier: CarrierState = field(default_factory=CarrierState)
    interlock: Interlock = field(default_factory=Interlock)
    require_interlock: bool = True
    config: UvVisConfig | None = None

    def __post_init__(self) -> None:
        self.log_dir = Path(self.log_dir)
        # No transport config needed: DDE commands are stateless invocations
        # of DDEClient.exe, verified against the run log.

    @classmethod
    def from_config(cls, config: UvVisConfig) -> SpectroStarNano:
        """Build a reader from a :class:`UvVisConfig` — the orchestrator entry point."""
        return cls(
            allow_motion=config.allow_motion,
            reader=config.reader,
            timeout_s=config.timeout_s,
            log_dir=config.log_path,
            require_interlock=config.require_interlock,
            config=config,
        )

    @property
    def wells(self) -> WellSelection | None:
        """The configured well selection, if a config was supplied."""
        return self.config.wells if self.config else None

    # ------------------------------------------------------------ interlock

    def refresh_interlock(self) -> Interlock:
        """Re-read what the reader can sense and return the interlock."""
        self.interlock.reader_present = self.present()
        self.interlock.carrier = self.carrier.position
        return self.interlock

    def status_code(self) -> int:
        """The interlock as a single integer — see :class:`Interlock.code`."""
        return self.refresh_interlock().code

    def _require_clear(self, what: str) -> None:
        """Refuse carrier motion unless the arm is clear -- observed, then declared.

        Two checks, and the order matters. The first is what the arm is
        *actually doing right now*, read from :mod:`tools.occupancy`: the arm
        claims that registry before every commanded move, from whatever process
        it is running in, so a live claim means the arm is moving even if
        nobody remembered to update a flag. The second is the older declared
        handshake, kept because a parked-but-not-claiming arm is still not
        proof of clearance.
        """
        # Observed: is the arm moving at this instant, in any process?
        occupancy.require_free("uv_vis")
        if not self.require_interlock:
            return
        self.interlock.reader_present = self.present()
        if not self.interlock.arm_clear:
            raise MotionNotAllowed(
                f"{what} refused: the robot arm has not declared itself clear of "
                "the reader. Call interlock.set_arm_clear(True) once it has "
                "retreated, or construct with require_interlock=False."
            )

    # ------------------------------------------------------------- internals

    def _occupy(self, doing: str):
        """Hold the carrier for one motion, so the arm is refused meanwhile."""
        return occupancy.claim("uv_vis", doing=doing)

    def _require_motion(self, what: str) -> None:
        if not self.allow_motion:
            raise MotionNotAllowed(
                f"{what} moves hardware and allow_motion is False. Enable it "
                "explicitly, and confirm the robot arm is clear of the reader."
            )

    def _call(self, action: str, **kwargs) -> dict:
        """Dispatch one command over DDE, verified against the run log."""
        try:
            if action == "connect":
                dde.ensure_control_running()
                return {"ok": True, "action": action,
                        "control_running": dde.control_running()}
            if action == "status":
                return {"ok": True, "action": action,
                        "present": dde.reader_present(),
                        "control_running": dde.control_running()}
            if action in ("init", "plate_in", "plate_out"):
                verb = {"init": "Init", "plate_in": "PlateIn",
                        "plate_out": "PlateOut"}[action]
                return dde.send(verb).as_dict()
            if action == "temp":
                return dde.send("Temp", str(kwargs["target"]),
                                verify=False).as_dict()
            if action == "run":
                ids = [kwargs.get(k, "") for k in
                       ("plate_id1", "plate_id2", "plate_id3")]
                return dde.run_protocol(
                    kwargs["protocol"], *[i for i in ids if i]
                ).as_dict()
            if action == "import_layout":
                return dde.send(
                    "ImportLayout", kwargs["protocol"], kwargs["db_path"],
                    kwargs["win_path"], verify=False,
                ).as_dict()
            if action == "edit_layout":
                return dde.send(
                    "EditLayout", kwargs["protocol"], kwargs["layout_string"],
                    verify=False,
                ).as_dict()
            if action == "shake":
                return dde.send("Shake", *kwargs["args"], verify=False).as_dict()
            raise ReaderError(f"unknown action {action!r}")
        except dde.DdeError as exc:
            raise ReaderError(str(exc)) from exc

    # -------------------------------------------------------------- presence

    def present(self) -> bool:
        """True if Windows currently enumerates the reader.

        Cheap pre-flight; avoids a long COM timeout when the instrument is
        simply powered off.
        """
        return dde.reader_present()

    def require_present(self) -> None:
        if not self.present():
            raise ReaderError(
                "reader not enumerated by Windows — powered off or unplugged "
                "(expected PnP id USB\\VID_0483&PID_A29A\\0601-003639)"
            )

    # -------------------------------------------------------------- sessions

    def connect(self) -> dict:
        """Open a control-software connection.

        The reader audibly initialises on connect; the software performs its own
        Init and EEPROM read, which takes roughly 11 s.
        """
        self.require_present()
        return self._call("connect")

    @contextmanager
    def session(self):
        """Context manager that connects first and leaves the software running.

        The bridge opens and closes a connection per command anyway, so this is
        about failing fast on a dead instrument rather than holding a handle.
        """
        self.connect()
        try:
            yield self
        finally:
            pass

    def status(self) -> dict:
        """Reader status.

        The returned ``status`` string is essentially always empty — the
        control's OUT parameters do not marshal back through PowerShell. Treat
        "no exception" as the health signal, and use ``present()`` for presence.
        """
        return self._call("status")

    # ------------------------------------------------------- carrier motion

    def init(self) -> dict:
        """Initialise the reader; this homes the plate carrier."""
        self._require_motion("init")
        self._require_clear("init")
        self.interlock.busy = True
        try:
            # Held for the whole motion: the arm is refused until the
            # carrier has finished moving, not merely until it started.
            with self._occupy("carrier init"):
                out = self._call("init")
        except Exception as exc:
            self.interlock.error = str(exc)
            raise
        finally:
            self.interlock.busy = False
        self.carrier.set("in")
        self.interlock.carrier = "in"
        self.interlock.error = None
        return out

    def plate_out(self) -> dict:
        """Move the plate carrier OUT (open) — the load/unload position."""
        self._require_motion("plate_out")
        self._require_clear("plate_out")
        self.interlock.busy = True
        try:
            # Held for the whole motion: the arm is refused until the
            # carrier has finished moving, not merely until it started.
            with self._occupy("carrier plate_out"):
                out = self._call("plate_out")
        except Exception as exc:
            self.interlock.error = str(exc)
            raise
        finally:
            self.interlock.busy = False
        self.carrier.set("out")
        self.interlock.carrier = "out"
        self.interlock.error = None
        return out

    def plate_in(self) -> dict:
        """Move the plate carrier IN (close) — the measuring position."""
        self._require_motion("plate_in")
        self._require_clear("plate_in")
        self.interlock.busy = True
        try:
            # Held for the whole motion: the arm is refused until the
            # carrier has finished moving, not merely until it started.
            with self._occupy("carrier plate_in"):
                out = self._call("plate_in")
        except Exception as exc:
            self.interlock.error = str(exc)
            raise
        finally:
            self.interlock.busy = False
        self.carrier.set("in")
        self.interlock.carrier = "in"
        self.interlock.error = None
        return out

    @contextmanager
    def open_carrier(self):
        """Open the carrier for loading and guarantee it closes again.

        Use this around any arm hand-off so a raised exception cannot leave the
        drawer open with the arm still moving::

            with reader.open_carrier():
                arm.place_plate()
        """
        self.plate_out()
        try:
            yield self
        finally:
            try:
                self.plate_in()
            except Exception:                       # never mask the real error
                pass

    # ------------------------------------------------------------- layout

    def import_layout(self, protocol: str, lb_path: str | Path) -> dict:
        """Import a ``.lb`` layout file into a named protocol.

        Fact 2: this **requires** the protocol-definition directory as the
        middle positional argument (``self.config.protocol_db_path`` when a
        config was supplied, else the vendor default) — omitting it fails with
        "cannot access test protocol database". ``lb_path`` is converted from
        its WSL form to a Windows path automatically.
        """
        self._require_motion("import_layout")
        db_path = (self.config.protocol_db_path if self.config else
                   r"C:\Program Files (x86)\BMG\SPECTROstar Nano\User\Definit")
        return self._call(
            "import_layout", protocol=protocol, db_path=db_path,
            win_path=windows_path(lb_path),
        )

    def edit_layout(self, protocol: str, layout_string: str) -> dict:
        """Set a protocol's layout from an inline string (same syntax as the
        ``.lb`` file body — fact 3), rather than importing a file."""
        self._require_motion("edit_layout")
        return self._call("edit_layout", protocol=protocol,
                          layout_string=layout_string)

    def apply_layout(self, protocol: str, plan: LayoutPlan) -> Path:
        """Write ``plan`` to a timestamped ``.lb`` file under
        ``config.layout_dir`` and import it into ``protocol``. Returns the
        written (WSL) path."""
        layout_dir = Path(self.config.layout_dir) if self.config else \
            Path("/home/lamp/SDLsetup/dataset/uv_vis_runs/layouts")
        layout_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        safe_protocol = re.sub(r"\s+", "_", protocol.strip()) or "layout"
        lb_path = layout_dir / f"{stamp}_{safe_protocol}.lb"
        plan.write_lb(lb_path)
        self.import_layout(protocol, lb_path)
        return lb_path

    # ------------------------------------------------------------ incubator

    def set_temperature(self, target_c: float) -> dict:
        """Set the incubator target, validated through :class:`Incubation`
        (roughly 25–45 °C; BMG advise a target at least 5 °C above ambient).
        Use :meth:`temperature_off`/:meth:`monitor_temperature` for ``0``/``0.1``.

        NOT yet verified against this instrument.
        """
        incubation = Incubation(target_c=target_c)
        incubation.validate()
        self._require_motion("set_temperature")
        return self._call("temp", target=incubation.to_command_value())

    def temperature_off(self) -> dict:
        """Switch the incubator off (``Temp 0``)."""
        self._require_motion("temperature_off")
        return self._call("temp", target=Incubation(target_c=None).to_command_value())

    def monitor_temperature(self) -> dict:
        """Enable temperature *monitoring* without heating (``Temp 0.1``)."""
        self._require_motion("monitor_temperature")
        return self._call(
            "temp", target=Incubation(monitor_only=True).to_command_value()
        )

    #: ``Reader response: [$XX $XX ...]`` blocks in the run log.
    _TEMP_RESPONSE_RE = re.compile(r"Reader response:\s*\[(.*?)\]")
    _HEX_BYTE_RE = re.compile(r"\$([0-9A-Fa-f]{2})")

    #: Plausible incubator reading in 1/10 degC. 0 = sensor idle; the extended
    #: incubator tops out at 65.0 degC, so anything above ~70 degC is a decode
    #: error, not a hot reader.
    _TEMP_RANGE_DECI_C = (0, 700)

    @classmethod
    def _decode_temperatures(cls, raw):
        """Decode the three incubator channels out of one reader status frame.

        ENCODING RESOLVED 2026-07-31 by live measurement -- big-endian 16-bit
        words in 1/10 degC at byte pairs (12,13), (14,15), (16,17).

        It was previously ambiguous: little-endian at (13,14)(15,16)(17,18)
        reproduced every reading taken at ambient, because the companion byte
        of any value below 25.5 degC is $00. Heating the incubator to 30.0 degC
        separated them, on two independent frames:

            [.. $09 $00 $FB $00 $FB $01 $02 $07 ..]
                big-endian    -> 25.1 / 25.1 / 25.8   plausible
                little-endian -> 25.1 / 50.7 / 179.4  impossible

            [.. $09 $01 $2C $01 $31 $01 $2D $00 ..]
                big-endian    -> 30.0 / 30.5 / 30.1   matches the 30.0 target
                little-endian -> 30.0 / 30.5 /  4.5   impossible while heating

        The range check below is kept as a guard against a future firmware
        changing the frame layout -- not as an arbiter between candidates. A
        wrong temperature is worse than no temperature on a surface that heats
        samples, so an out-of-range word raises rather than being returned.
        """
        lo, hi = cls._TEMP_RANGE_DECI_C
        vals = tuple(
            (raw[i] << 8) | raw[i + 1] for i in (12, 14, 16)
        )
        bad = [v for v in vals if not lo <= v <= hi]
        if bad:
            raise ReaderError(
                "implausible incubator temperature(s) %s (1/10 degC) decoded "
                "from frame %s -- the byte layout has changed, or this is not "
                "a status response." % (bad, [hex(b) for b in raw])
            )
        return tuple((v / 10.0) if v else None for v in vals)

    def temperatures(self):
        """The three incubator sensor channels in degC, newest frame in the log.

        ``None`` per channel when that sensor reads 0 (monitoring off for that
        channel); ``None`` overall when no status frame has been logged yet.
        Raises :class:`ReaderError` when the frame cannot be decoded
        unambiguously -- see :meth:`_decode_temperatures`.
        """
        try:
            text = RUN_LOG.read_bytes().decode("cp949", errors="replace")
        except OSError:
            return None
        blocks = self._TEMP_RESPONSE_RE.findall(text)
        if not blocks:
            return None
        raw = [int(b, 16) for b in self._HEX_BYTE_RE.findall(blocks[-1])]
        if len(raw) < 19:
            return None
        return self._decode_temperatures(raw)

    # -------------------------------------------------------------- shaking

    def shake(self, shaking: Shaking | None = None, **kwargs) -> dict:
        """Shake the plate (fact 6). This ACTUATES the reader — gated behind
        ``allow_motion`` and the arm-clear interlock exactly like carrier
        motion.

        UNVERIFIED-ON-DDE: only the script-language form (``R_Shake``) is
        documented in the vendor help; the ``Shake <mode> <rpm> <time_s>
        [<x> <y>]`` positional-argument shape sent here has not been confirmed
        against a live instrument.
        """
        shk = shaking if shaking is not None else Shaking(**kwargs)
        shk.validate()
        self._require_motion("shake")
        self._require_clear("shake")
        return self._call("shake", args=shk.to_args())

    # ---------------------------------------------------------- measurement

    # --------------------------------------------------------------- proof

    def log_activity(self, needle: str, tail: int = 4000) -> int:
        """Count occurrences of ``needle`` in the control software's run log.

        Why this exists: the ActiveX OUT status string never marshals back
        through PowerShell, so a command that silently did nothing is
        indistinguishable from one that worked — both raise no exception and
        return an empty status. The run log is the instrument's own record and
        is the only honest confirmation available.
        """
        try:
            text = RUN_LOG.read_bytes().decode("cp949", errors="replace")
        except OSError:
            return 0
        return text.lower().count(needle.lower())

    def run_protocol(
        self,
        protocol: str,
        plate_id1: str = "",
        plate_id2: str = "",
        plate_id3: str = "",
        settle_s: float = 2.0,
        verify: bool = True,
    ) -> dict:
        """Run a measurement protocol defined in the control software.

        Raises :class:`ReaderError` if the control software's run log shows no
        ``(Run command)`` — see :meth:`log_activity` for why the return value
        alone cannot be trusted. Set ``verify=False`` only when deliberately
        probing call shapes.

        ``protocol`` must already exist in the control software — this selects
        one by name and cannot define measurement parameters. Author protocols
        in the SPECTROstar Nano UI (Microplate tab) or copy one of the shipped
        examples under ``Example Test Runs``.

        NOT yet verified against this instrument: the argument layout is
        implemented as ``@('Run', protocol, ids…)`` by analogy with the verified
        plate commands, but no protocol has been executed through it.
        """
        if not protocol:
            raise ValueError("protocol name is required")
        self._require_motion("run_protocol")
        if self.require_interlock:
            self.refresh_interlock()
            if not self.interlock.ready_to_measure:
                raise MotionNotAllowed(
                    "run_protocol refused: interlock not ready "
                    f"({self.interlock.describe()}). A measurement needs the "
                    "reader present, the carrier IN, a plate declared loaded, "
                    "and the arm declared clear."
                )
        started = time.time()
        out = self._call(
            "run", protocol=protocol,
            plate_id1=plate_id1, plate_id2=plate_id2, plate_id3=plate_id3,
        )
        time.sleep(settle_s)
        out["duration_s"] = round(time.time() - started, 1)
        # dde.run_protocol already verified the (Run command) marker and waited
        # for "End of test run", so log_confirmed here is real evidence.
        out.setdefault("log_confirmed", False)
        if verify and not out["log_confirmed"]:
            raise ReaderError(
                f"run_protocol({protocol!r}) was not confirmed in the control "
                "software's run log — the command did not reach the reader."
            )
        out["wells_measured"] = dde.wells_measured()
        return out
