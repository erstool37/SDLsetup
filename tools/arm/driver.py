"""The only place in the codebase that touches the xArm Python SDK.

Everything above this file works with :class:`~tools.arm.safety.ValidatedMove`
objects and plain numbers; everything below it is UFACTORY's ``XArmAPI``. If
you find yourself importing ``xarm`` anywhere else, that is the bug.

The SDK import is lazy, so this module -- and therefore every package that
imports it -- loads fine on a machine with no SDK and no arm. Constructing an
:class:`XArmConnection` does not connect either; :meth:`XArmConnection.connect`
is the first line that talks to hardware.

What this layer does and does not decide
========================================

It *reports* and *transports*. It does not choose targets, retry, or interpret.
Refusals here are of two kinds only, both mechanical:

* a request that is not a :class:`ValidatedMove` (guards were skipped), and
* a controller that reports an error code (see below).

**Faults are never cleared automatically.** A latched ``error_code`` may be the
record of a previous collision, and clearing it to "get going again" destroys
the only evidence that something hit something. ``clear_errors=True`` is an
explicit operator decision, surfaced as a CLI flag, never a default.

The one exception is :meth:`recover`, used by the retreat path: once the
operator's plate is sitting under a fixed objective, lowering it is safer than
leaving it there, so retreat clears the fault it needs to clear and says so.
"""
from __future__ import annotations

import time
from collections.abc import Sequence
from typing import Any

from .safety import SafetyError, ValidatedMove

#: Seconds to wait after constructing XArmAPI before its async report thread has
#: populated ``connected`` / ``error_code``. Measured on this controller.
CONNECT_SETTLE_S = 1.2

#: After settling, poll this many times for a latched error to surface.
ERROR_PROBE_TRIES = 8
ERROR_PROBE_INTERVAL_S = 0.25

#: TCP control port, used for the read-only reachability probe.
CONTROL_PORT = 502

#: BIO Gripper G2 control modes (Modbus register 0x010A; **persists across a
#: power cycle**, which is why the mode is configuration here rather than
#: something a call flips on the fly -- see ``ArmSettings.control_mode``).
#:
#: Mode 0 has no position servo at all: the firmware reads the commanded
#: position only as a threshold, opening fully above 90 and closing fully below
#: it. A 96 mm request in mode 0 therefore returns code 0 and opens to 150 --
#: it does not fail, it silently means something else. Mode 1 is the real
#: position mode (position + speed + force).
BIO_MODE_OPEN_CLOSE = 0
BIO_MODE_POSITION = 1

#: Jaw opening span, in mm, with the fingers in their default orientation.
#: These are the *span between the jaws*, not a per-jaw coordinate: the gripper
#: takes one position register, both jaws move together, and the grasp stays
#: centred on the tool axis. There is no way to command one jaw.
#: (Reversing the fingers mechanically changes the span to 4-83 mm; that is a
#: hardware change and would need these numbers changed with it.)
BIO_POS_MIN_MM = 71
BIO_POS_MAX_MM = 150

#: Speed bounds the SDK enforces on the position path (it silently clamps to
#: this window). Force is a percentage of the 20 N maximum.
BIO_POS_SPEED_MIN = 500
BIO_POS_SPEED_MAX = 4000
BIO_FORCE_MIN = 1
BIO_FORCE_MAX = 100

#: Gap between the two position readbacks that decide whether the jaws have
#: actually stopped, and how far apart those readings may be and still count as
#: stopped. Needed because the SDK's motion wait can return before the gripper
#: has raised its MOTION bit -- see :meth:`XArmConnection.bio_position`.
BIO_READBACK_SETTLE_S = 0.25
BIO_SETTLED_TOL_MM = 1.0



#: The controller's fault-free refusal: it accepts a move, may travel part of
#: it, stops, and reports this with ``error_code`` still 0. Measured repeatedly
#: on this rig, most recently 2026-08-07 on a Z rise at (366.56, -16.76) where
#: ``get_inverse_kinematics`` succeeded at all 27 sampled points along the line
#: -- so IK does NOT predict it and cannot be used to avoid it.
REFUSAL_CODE = -9
#: Re-arming and re-issuing recovers it: successive runs of the same leg made
#: monotone progress (z 61.4 -> 170.0 -> ...). Bounded, because a refusal that
#: never clears is a real limit and must surface rather than loop.
REFUSAL_RETRIES = 3
REFUSAL_RETRY_SETTLE_S = 0.4

class ArmError(RuntimeError):
    """The controller refused a command, or is in a state we must not drive from."""


def reachable(host: str, *, timeout_s: float = 0.6) -> bool:
    """Read-only reachability probe: open and immediately close a TCP socket.

    Sends no command and moves nothing. Used by status displays so a powered-off
    arm reports fast instead of blocking on an SDK connect.
    """
    import socket

    try:
        with socket.create_connection((host, CONTROL_PORT), timeout=timeout_s):
            return True
    except OSError:
        return False


class XArmConnection:
    """A lazily-opened xArm session.

    ``live=False`` (the default everywhere) makes this a null transport: it
    never imports the SDK, never opens a socket, and every motion method
    returns without commanding anything. That is what makes dry-run plans
    genuinely incapable of moving the arm rather than merely unlikely to.
    """

    def __init__(self, host: str, *, live: bool = False, clear_errors: bool = False,
                 settle_s: float = CONNECT_SETTLE_S, log=None) -> None:
        self.host = host
        self.live = bool(live)
        self.clear_errors = bool(clear_errors)
        self.settle_s = float(settle_s)
        self._api: Any | None = None
        self._log = log or (lambda message: print(message, flush=True))
        #: Has anything been commanded through this session? Lives here, not on
        #: Arm, because several Arm views can share one connection
        #: (Arm.with_envelope) and a retreat must fire if *any* of them moved.
        self.engaged = False
        #: Has motion_enable/set_mode/set_state been run on this session?
        self._armed = False
        #: Has the arm's real pose been checked against its envelope since this
        #: session started commanding motion? See XArmConnection.send.
        self.start_verified = False
        #: BIO gripper control mode this session has already written, or None.
        #: The mode is device-persistent state and writing it reboots the
        #: gripper MCU, so it is asserted once per session rather than on every
        #: actuation. It cannot be read back to do better than this -- see
        #: :meth:`bio_enable`.
        self._bio_mode_written: int | None = None

    # -- lifecycle -------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._api is not None and bool(getattr(self._api, "connected", False))

    def connect(self, *, arm: bool = False) -> bool:
        """Open the session. Returns None in dry-run, the raw API otherwise.

        **Connecting does not arm the controller.** ``arm=False`` (the default)
        opens the socket and reads state; it does not call ``motion_enable`` or
        change mode/state, so ``sdl-robot controller`` and ``gripper status``
        really are read-only. Motion paths pass ``arm=True``, which runs the
        enable sequence once per session.

        This split is not cosmetic: the retired ``scripts/gripper.py`` connected
        without arming, and folding it into a connect that always armed would
        have made a documented read-only command change hardware state.

        Returns whether a live session is open. It deliberately does **not**
        hand back the ``XArmAPI`` object: a public accessor for the raw handle
        would be a supported path to ``set_position`` that skips
        :mod:`.safety`, which is the thing this layer exists to prevent.
        """
        if not self.live:
            return False
        if self._api is not None:
            if arm:
                self._arm_controller(self._api)
            return True

        from xarm.wrapper import XArmAPI  # lazy: only imported for live runs

        api = XArmAPI(self.host, is_radian=False)
        time.sleep(self.settle_s)
        if not api.connected:
            raise ArmError(f"xArm {self.host} did not connect")

        for _ in range(ERROR_PROBE_TRIES):
            if api.error_code:
                break
            time.sleep(ERROR_PROBE_INTERVAL_S)

        if api.error_code:
            if not self.clear_errors:
                raise ArmError(
                    f"controller reports error_code={api.error_code}. This is NOT cleared "
                    f"automatically -- it may record a previous collision. Inspect the arm, "
                    f"then re-run with --clear-errors if it is safe."
                )
            self._log(f"  [arm] clearing error_code={api.error_code} at operator request")
            api.clean_warn()
            api.clean_error()

        self._api = api
        if arm:
            self._arm_controller(api)
        return True

    def _open(self, *, arm: bool = False) -> Any | None:
        """The raw handle, for this module's own use. Never returned upward."""
        return self._api if self.connect(arm=arm) else None

    def _arm_controller(self, api: Any) -> None:
        """Enable motion and put the controller in position/ready mode. Once."""
        if self._armed:
            return
        self._log(f"  [arm] enabling motion on {self.host}")
        api.motion_enable(True)
        api.set_mode(0)   # position control
        api.set_state(0)  # ready
        self._armed = True

    def disconnect(self) -> None:
        if self._api is not None:
            try:
                self._api.disconnect()
            except Exception:
                pass
            self._api = None
            self._armed = False
            # The next session cannot assume the gripper mode survived, and has
            # no way to read it back, so it re-asserts once.
            self._bio_mode_written = None

    def __enter__(self) -> XArmConnection:
        return self

    def __exit__(self, *_exc: object) -> None:
        self.disconnect()

    # -- reads (never move anything) --------------------------------------
    def read_pose(self) -> list[float]:
        """Fresh Cartesian pose from the controller, mm/deg.

        This is what :meth:`Envelope.check_readback` must be fed. It is a real
        round-trip to the machine every call -- no caching, deliberately.
        """
        api = self._open()
        if api is None:
            raise ArmError("read_pose() needs a live connection (live=False)")
        code, pose = api.get_position(is_radian=False)
        if code != 0:
            raise ArmError(f"get_position failed: code={code}")
        return [float(v) for v in pose]

    def read_joints(self) -> list[float]:
        api = self._open()
        if api is None:
            raise ArmError("read_joints() needs a live connection (live=False)")
        code, angles = api.get_servo_angle(is_radian=False)
        if code != 0:
            raise ArmError(f"get_servo_angle failed: code={code}")
        return [float(v) for v in angles]

    def status(self) -> dict:
        """Controller state as data. Reports; decides nothing.

        Reachability is probed even in dry-run: it is a read-only TCP connect
        that sends nothing and moves nothing, and "is the arm powered on?" is
        the first question the panel has to answer. Without it a dry-run node
        reports ``reachable=None``, which is indistinguishable from "unplugged".
        """
        if not self.live:
            return {"host": self.host, "live": False, "connected": False,
                    "reachable": reachable(self.host)}
        if not reachable(self.host):
            return {"host": self.host, "live": True, "connected": False, "reachable": False}
        api = self._open()
        if api is None:
            return {"host": self.host, "live": True, "connected": False, "reachable": True}
        return {
            "host": self.host, "live": True, "reachable": True,
            "connected": bool(api.connected), "error_code": api.error_code,
            "warn_code": api.warn_code, "state": api.state, "mode": api.mode,
            "pose": self.read_pose(),
        }

    # -- motion (the guarded path) ----------------------------------------
    def send(self, move: ValidatedMove) -> None:
        """Command one validated Cartesian pose, blocking until it completes.

        Accepts *only* a :class:`ValidatedMove`. That type cannot be built
        without passing an :class:`~tools.arm.safety.Envelope`, so
        there is no code path from a caller to ``set_position`` that skips the
        guards.
        """
        if not isinstance(move, ValidatedMove):
            raise SafetyError(
                f"driver.send() takes a ValidatedMove, got {type(move).__name__}. "
                f"Build it with Envelope.validate(pose, speed, label) so the motion "
                f"guards run."
            )
        api = self._open(arm=True)
        if api is None:
            return  # dry-run: validated, printed by the caller, never commanded
        self.engaged = True
        pose = move.pose
        for attempt in range(1, REFUSAL_RETRIES + 2):
            code = api.set_position(x=pose[0], y=pose[1], z=pose[2], roll=pose[3],
                                    pitch=pose[4], yaw=pose[5],
                                    speed=move.speed_mm_s, wait=True)
            if code == 0 and not api.error_code:
                return
            # A LATCHED FAULT IS NEVER RETRIED. error_code 31 is a collision:
            # re-commanding the move that caused it drives the arm back into
            # whatever it hit. Only the fault-free refusal below is recoverable.
            if api.error_code or code != REFUSAL_CODE or attempt > REFUSAL_RETRIES:
                break
            self._log(f"  [arm] refusal (code={code}, no fault) on {move.label!r}; "
                      f"re-arming and retrying {attempt}/{REFUSAL_RETRIES}")
            self._arm_controller(api)
            time.sleep(REFUSAL_RETRY_SETTLE_S)
        raise ArmError(f"set_position({move.label}) failed: code={code}, "
                       f"error_code={api.error_code}")

    def send_joints(self, angles: Sequence[float], speed_deg_s: float, label: str) -> None:
        """Command joint angles. Used only by workspace-traversal procedures.

        Joint space has no Cartesian envelope to check against, so the caller
        must have obtained these angles from a taught location -- never from
        arithmetic on a pose. The procedures that use this print the full plan
        and require ``--execute``.
        """
        api = self._open(arm=True)
        if api is None:
            return
        self.engaged = True
        code = api.set_servo_angle(angle=list(angles), speed=speed_deg_s, wait=True,
                                   is_radian=False)
        if code != 0 or api.error_code:
            raise ArmError(f"set_servo_angle({label}) failed: code={code}, "
                           f"error_code={api.error_code}")

    # -- emergency ---------------------------------------------------------
    def halt(self) -> None:
        """Cancel a move still in flight. Best effort; never raises."""
        if self._api is None:
            return
        try:
            self._api.set_state(4)  # STOP
        except Exception:
            pass

    def recover(self) -> None:
        """Clear faults and re-enable, so a retreat can lower the plate.

        Only the retreat path calls this. Leaving the plate raised against a
        fixed objective is the worse of the two risks, and the clearing is
        announced rather than silent.
        """
        api = self._api
        if api is None:
            return
        if api.error_code:
            self._log(f"  [safety] clearing error_code={api.error_code} so the plate "
                      f"can be lowered")
            api.clean_warn()
            api.clean_error()
        api.motion_enable(True)
        api.set_mode(0)
        api.set_state(0)

    # -- BIO Gripper G2 ----------------------------------------------------
    # Gripper actuation is jaw-only: the arm body does not move. It is treated
    # as always-allowed on this surface (established operating policy), but it
    # is still actuation, so every call is logged and a controller fault still
    # aborts it.

    def bio_error(self) -> str:
        """Best-effort BIO gripper error read, for diagnostics."""
        api = self._api
        if api is None:
            return "<not connected>"
        try:
            return str(api.get_bio_gripper_error())
        except Exception as exc:
            return f"<unreadable: {exc!r}>"

    def bio_status(self) -> Any:
        api = self._open()
        if api is None:
            return None
        return api.get_bio_gripper_status()

    @staticmethod
    def _bio_read_control_mode(api: Any) -> int | None:
        """The gripper's reported control mode, or None if it will not say.

        ``XArmAPI`` defines ``set_bio_gripper_control_mode`` and no getter, and
        its alias map covers only ``set``/``get_bio_gripper_position``, so
        ``hasattr(api, "get_bio_gripper_control_mode")`` is False. The getter
        exists one layer down, on the inner object, and is the same call the
        SDK makes internally (``x3/gripper.py:743``).

        Reaching past the wrapper is why every step here is guarded. A future
        SDK that renames the attribute, or a controller in simulation mode --
        where the decorator returns a bare ``0`` instead of the documented
        ``(code, mode)`` tuple -- must degrade to "unverified", never to a
        crash in the middle of a gripper sequence.
        """
        reader = getattr(getattr(api, "_arm", None),
                         "get_bio_gripper_control_mode", None)
        if reader is None:
            return None
        try:
            reply = reader()
        except Exception:
            return None
        if not isinstance(reply, (tuple, list)) or len(reply) != 2:
            return None      # simulation mode returns a scalar
        code, reported = reply
        return int(reported) if code == 0 and reported in (0, 1, 2) else None

    def bio_enable(self, speed: int, mode: int = BIO_MODE_OPEN_CLOSE) -> None:
        """Put the BIO Gripper G2 into ``mode`` and enable it.

        Clears latched *gripper* faults (a power-on artefact) but still refuses
        to actuate while the *controller* reports an error.

        The mode is written **once per session**, not per actuation.
        ``set_bio_gripper_control_mode`` is not a cheap setter: it rewrites
        state that outlives the session (the mode persists across a power cycle)
        and reboots the gripper MCU, costing a second of blocking sleeps inside
        the SDK. Until 2026-08-11 this ran on every grip and release.

        A switch is confirmed two ways. The write's own return code is checked
        below -- an illegal register address comes back non-zero, which is how
        the SDK itself detects an unsupported BIO version
        (``x3/gripper.py:1038``) -- and the mode is then read back off the
        device by :meth:`_bio_read_control_mode`.

        The write and the read do not use the same register: the SDK writes
        ``0x110A`` (``x3/gripper.py:725``) and reads ``0x010A`` (``:748``, the
        address the vendor manual names). That looked like a defect until the
        same split turned up in the position path, which writes ``0x0700`` /
        ``0x0C00`` and reads ``0x0702`` -- so a command/status address split is
        this firmware's habit, not evidence of a bug. It does mean the
        read-back is corroboration rather than proof.

        The write comes before the enable, because the vendor manual requires
        it: "After switching modes, the gripper needs to be re-enabled."
        """
        # The gripper is its own device on the controller: enabling the jaws
        # does not arm the arm, and this deliberately does not pass arm=True.
        api = self._open()
        if api is None:
            return
        # Read the controller fault BEFORE clearing anything. The retired
        # gripper.py cleared warn+error first and then tested error_code, so its
        # "refuse while faulted" check could never fire. Clearing a latched
        # controller fault also destroys the record of whatever caused it, which
        # the standing rule forbids doing automatically.
        latched = api.error_code
        if latched and not self.clear_errors:
            raise ArmError(
                f"controller reports error_code={latched}; not actuating the gripper. "
                f"This is NOT cleared automatically -- it may record a previous "
                f"collision. Inspect the arm, then re-run with --clear-errors if safe.")
        if latched:
            self._log(f"  [arm] clearing error_code={latched} at operator request")
            api.clean_warn()
            api.clean_error()
        # The gripper's own fault is a power-on artefact and is always cleared.
        api.clean_bio_gripper_error()
        mode = int(mode)
        if self._bio_mode_written == mode:
            mode_ret = 0
        else:
            # Announced, because this mutates state that outlives the session.
            self._log(f"  [gripper] asserting control mode {mode} "
                      f"(persists across power cycles; gripper MCU reboots)")
            mode_ret = api.set_bio_gripper_control_mode(mode)
            if mode_ret == 0:
                reported = self._bio_read_control_mode(api)
                if reported is not None and reported != mode:
                    raise ArmError(
                        f"asked the gripper for control mode {mode}, it reports "
                        f"{reported}. Refusing to actuate: in the wrong mode a "
                        f"position command does not fail, it just opens or "
                        f"closes fully.")
                self._log(f"  [gripper] control mode "
                          f"{'confirmed' if reported == mode else 'set, unconfirmed'}")
            self._bio_mode_written = mode if mode_ret == 0 else None
        enable_ret = api.set_bio_gripper_enable(True)
        api.set_bio_gripper_speed(int(speed))
        if api.error_code:
            raise ArmError(f"controller error_code={api.error_code} appeared while enabling "
                           f"the gripper; not actuating")
        if mode_ret != 0 or enable_ret != 0:
            raise ArmError(
                f"BIO gripper enable failed: control_mode ret={mode_ret}, "
                f"enable ret={enable_ret}, bio_err={self.bio_error()}"
            )

    def bio_move(self, direction: str, speed: int,
                 mode: int = BIO_MODE_OPEN_CLOSE) -> dict:
        """Open or close the jaws fully. ``direction`` is 'open' or 'close'.

        **Verified on hardware in mode 0 only.** In mode 0 this is the exact
        sequence the plate cycle has run since 2026-06-30.

        In mode 1 it is expected to still work -- ``open_bio_gripper`` and
        ``close_bio_gripper`` resolve to 150 mm and 71 mm on the SDK's legacy
        register path (``x3/gripper.py:820-829``), which converts mm to encoder
        counts when the gripper is in position mode -- but that is *inference
        from reading the SDK*, not an observation. It is also the path the SDK
        falls back to when a gripper rejects the G2 register, so it is not the
        primary route for this device. **Flipping ``gripper.control_mode`` to 1
        therefore moves the live-verified plate cycle onto an untested path.**
        Dry-run the cycle and watch the first real grip before trusting it.
        """
        if direction not in ("open", "close"):
            raise ValueError(f"direction must be 'open' or 'close', got {direction!r}")
        self.bio_enable(speed, mode=mode)
        api = self._open()
        if api is None:
            return {"action": direction, "live": False, "code": None}
        if direction == "open":
            code = api.open_bio_gripper(speed=int(speed), wait=True)
        else:
            code = api.close_bio_gripper(speed=int(speed), wait=True)
        result = {"action": direction, "live": True, "code": code,
                  "bio_status": api.get_bio_gripper_status(),
                  "bio_error": self.bio_error()}
        if code != 0 or api.error_code != 0:
            raise ArmError(f"bio {direction} failed: code={code}, "
                           f"error_code={api.error_code}, bio_err={result['bio_error']}")
        return result

    def bio_opening(self) -> float | None:
        """Current jaw span in mm, or None when not connected.

        **Trustworthy only in position mode.** The SDK converts the feedback
        register to mm whenever the gripper reports hardware version 2, but it
        applies the *forward* conversion only when the gripper is in position
        mode -- so in open/close mode the setter and the getter do not agree on
        units, and this number has not been checked against a ruler there.
        Callers that surface it outside position mode must say so; see
        :meth:`tools.arm.Arm.opening`.
        """
        api = self._open()
        if api is None:
            return None
        code, pos = api.get_bio_gripper_g2_position()
        if code != 0:
            raise ArmError(f"could not read the jaw position: code={code}, "
                           f"bio_err={self.bio_error()}")
        return pos

    def bio_position(self, opening_mm: float, speed: int, force: int,
                     mode: int = BIO_MODE_POSITION) -> dict:
        """Drive the jaws to a specific span, in mm.

        Refuses out-of-range spans rather than passing them down. The SDK
        silently clamps to 71-150, so an 8 mm typo would return code 0 and close
        to 71 -- a wrong grasp that reports success. Same reasoning for the mode
        check: in open/close mode this call would still "succeed" and simply
        open or close fully.
        """
        if mode != BIO_MODE_POSITION:
            raise ArmError(
                f"the gripper is configured for control_mode={mode} (open/close). "
                f"A position setpoint has no meaning there -- the firmware would "
                f"read {opening_mm} mm as a threshold and go fully "
                f"{'open' if opening_mm > 90 else 'closed'}. Set gripper.control_mode "
                f"to {BIO_MODE_POSITION} in arm.json (or pass control_mode=1) first.")
        if not BIO_POS_MIN_MM <= opening_mm <= BIO_POS_MAX_MM:
            raise ArmError(
                f"jaw span {opening_mm} mm is outside the gripper's "
                f"{BIO_POS_MIN_MM}-{BIO_POS_MAX_MM} mm range. The SDK would clamp "
                f"this silently and report success.")
        if not BIO_FORCE_MIN <= force <= BIO_FORCE_MAX:
            raise ArmError(f"force {force} is outside {BIO_FORCE_MIN}-{BIO_FORCE_MAX} "
                           f"(percent of the 20 N maximum)")
        speed = int(speed)
        if not BIO_POS_SPEED_MIN <= speed <= BIO_POS_SPEED_MAX:
            raise ArmError(f"speed {speed} is outside "
                           f"{BIO_POS_SPEED_MIN}-{BIO_POS_SPEED_MAX} on the position "
                           f"path (the SDK would clamp it silently)")

        self.bio_enable(speed, mode=mode)
        api = self._open()
        if api is None:
            return {"action": "position", "live": False, "code": None,
                    "target_mm": opening_mm}
        code = api.set_bio_gripper_g2_position(
            float(opening_mm), speed=speed, force=int(force), wait=True)
        result = {"action": "position", "live": True, "code": code,
                  "target_mm": opening_mm, "speed": speed, "force": force,
                  "bio_status": api.get_bio_gripper_status(),
                  "bio_error": self.bio_error()}
        if code != 0 or api.error_code != 0:
            raise ArmError(f"bio position {opening_mm} mm failed: code={code}, "
                           f"error_code={api.error_code}, bio_err={result['bio_error']}")
        # Read back rather than trust the return code: the jaws stop early on
        # whatever they meet. This is telemetry, not a verdict -- a gap between
        # target and actual is equally consistent with a stall, too little
        # force, or an obstruction, and separating those needs a tolerance and
        # an expected object width that only the caller has.
        #
        # Read twice. The SDK's wait=True polls the status register with no
        # initial dwell (x3/gripper.py:648), so if the gripper has not yet
        # raised its MOTION bit the wait returns on the first poll and the
        # first reading is taken mid-travel. Two readings that agree mean the
        # jaws have stopped; ones that disagree mean this number is in flight,
        # and saying so beats reporting it as a measurement.
        first = self.bio_opening()
        time.sleep(BIO_READBACK_SETTLE_S)
        result["actual_mm"] = self.bio_opening()
        result["settled"] = (
            first is not None and result["actual_mm"] is not None
            and abs(result["actual_mm"] - first) <= BIO_SETTLED_TOL_MM)
        return result


__all__ = [
    "ArmError",
    "BIO_FORCE_MAX",
    "BIO_FORCE_MIN",
    "BIO_READBACK_SETTLE_S",
    "BIO_SETTLED_TOL_MM",
    "BIO_MODE_OPEN_CLOSE",
    "BIO_MODE_POSITION",
    "BIO_POS_MAX_MM",
    "BIO_POS_MIN_MM",
    "BIO_POS_SPEED_MAX",
    "BIO_POS_SPEED_MIN",
    "CONNECT_SETTLE_S",
    "CONTROL_PORT",
    "XArmConnection",
    "reachable",
]
