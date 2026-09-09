"""Dashboard adapter for the environment PLC. A thin shell over the device layer.

It holds no hardware logic. Everything it does is a call the environment
package's public surface (:mod:`tools.environment.api`) already offers, so the
dashboard cannot drive the CLICK in a way a phase script could not. It imports
:mod:`tools.environment.plc` and friends *directly* rather than through ``api``
only because ``api`` re-exports this node -- importing it back would be a cycle.
That is the same arrangement :mod:`tools.circulator.node` uses.

**It must never raise into the bus.** :meth:`status` catches everything and
returns a dict carrying an ``error`` field; :meth:`command` returns
``{"ok": False, "refused": ...}`` for a refusal and ``{"ok": False,
"error": ...}`` for anything else. The precedent is documented: the previous
UV-Vis node's constructor raised ``TypeError`` and **the dashboard could not
start at all** once that node was registered, and no check short of building the
lab caught it. So ``EnvironmentNode()`` takes no required arguments, touches no
hardware, and does not even resolve its config until something asks.

Why status() does not hold a Modbus session
===========================================

**The CLICK accepts at most three concurrent Modbus TCP clients and refuses the
fourth.** A status page polling on a timer would spend one of those three
permanently, and a run would then find the PLC unreachable for a reason nothing
in its own logs explains.

So :meth:`status` renders whatever the *run* published
(:func:`~.plc.latest_published`, with an ``age_s`` computed on read so a stale
file cannot claim to be fresh). It opens a transient session of its own only
when **both** hold:

* ``environment.dashboard_poll_plc`` is true in ``configs/config.yaml``, and
* no :mod:`tools.occupancy` claim on ``"environment"`` exists.

The occupancy check is what makes the first knob safe to turn on: the PLC's
self-conflict entry exists precisely to protect the three-client limit, so a
dashboard that respects it cannot steal a run's socket.

.. warning::

   **``enable_pid`` is deliberately NOT a dashboard command, and must not be
   added.**

   Enabling the ladder PID hands control of the chamber's heater and humidifier
   to the loop. That is an actuation, and the standing rule on this lab surface
   is that anything which can physically actuate hardware is confirmed
   deliberately -- not clicked on a status page. ``allow_actuation`` gating is
   not sufficient mitigation; the circulator's ``open`` was dropped from its own
   dashboard surface for exactly this reason.

   ``disable_pid`` **stays**. Turning the loop off is the safe direction, it is
   what an abort does, and an operator watching a chamber misbehave needs it
   reachable without finding a terminal.
"""
from __future__ import annotations

from typing import Any

from .. import occupancy
from ..node import Node
from .plc import PlcClient, PlcSettings, latest_published
from .safety import ActuationNotAllowed, PlcError, SafetyError


class EnvironmentNode(Node):
    """Enclosure temperature and humidity, supervised over Modbus TCP.

    ``in_range_tol_c`` / ``in_range_tol_pct`` are **not defaulted to a number.**
    How close to setpoint counts as "in range" is an operator decision about this
    chamber, and no measurement in this repo states one. Left unset, ``in_range``
    reports ``None`` -- "not known" rather than a fabricated verdict -- and the
    status card says why.

    TODO(operator): declare the in-range tolerances for this chamber, in degC
    and %RH, and pass them here (or add them to ``configs/config.yaml`` and wire
    them through ``PlcSettings``).
    """

    kind = "environment"

    def __init__(self, name: str = "environment", *, config: Any = None,
                 settings: PlcSettings | None = None,
                 in_range_tol_c: float | None = None,
                 in_range_tol_pct: float | None = None) -> None:
        # Nothing here may raise or touch hardware: build_lab() calls
        # EnvironmentNode() with no arguments, and a constructor that throws
        # takes the whole dashboard down with it.
        super().__init__(name)
        self._config_source = config
        self._settings = settings
        self._last: dict | None = None
        self.in_range_tol_c = in_range_tol_c
        self.in_range_tol_pct = in_range_tol_pct

    # -- the device -------------------------------------------------------
    @property
    def settings(self) -> PlcSettings:
        """Resolved on first use, so importing or registering this node needs no
        config file and no PLC."""
        if self._settings is None:
            self._settings = PlcSettings.from_config(self._config_source)
        return self._settings

    def client(self) -> PlcClient:
        """A fresh, unconnected client. Never cached -- see the module docstring
        on the three-client limit."""
        return PlcClient(self.settings)

    # -- status -----------------------------------------------------------
    def status(self) -> dict:
        try:
            settings = self.settings
        except Exception as exc:                                      # noqa: BLE001
            self.state = "error"
            return self.base_status(
                implemented=True, error=str(exc), age_s=None, in_range=None,
                pid_enabled=None, clients_limit_reached=None,
                summary="environment PLC (configuration error)")

        try:
            return self._status(settings)
        except Exception as exc:                                      # noqa: BLE001
            # Anything at all: a corrupt published record, a filesystem fault,
            # a transport that raised where it should have reported. The bus
            # gets a dict, always.
            self.state = "error"
            return self.base_status(
                implemented=True, error="%s: %s" % (type(exc).__name__, exc),
                age_s=None, in_range=None, pid_enabled=None,
                clients_limit_reached=None,
                summary="environment PLC (status unavailable)")

    def _status(self, settings: PlcSettings) -> dict:
        published: dict | None = None
        publish_error: str | None = None
        try:
            published = latest_published()
        except PlcError as exc:
            publish_error = str(exc)

        polled = False
        poll_skipped: str | None = None
        holders = occupancy.blockers("environment")
        if not settings.dashboard_poll_plc:
            poll_skipped = ("environment.dashboard_poll_plc is false; rendering the "
                            "reading the run published instead of opening a session")
        elif holders:
            poll_skipped = ("a run holds the environment claim (%s); the CLICK accepts "
                            "only %d concurrent clients, so this page will not take one"
                            % ("; ".join("%s pid %d" % (c.device, c.pid)
                                         for c in holders), 3))
        else:
            published, polled, poll_error = self._transient_read(published)
            if poll_error is not None:
                publish_error = poll_error

        channels = (published or {}).get("channels") or {}
        temp = channels.get("temp_filtered_c")
        rh = channels.get("rh_filtered_pct")
        temp_sp = channels.get("temp_sp_c")
        rh_sp = channels.get("rh_sp_pct")
        # DF9 / DF10, from the same reading -- read-only, published so the
        # Chamber Control PWM-duty trend can be drawn. Not a second session.
        temp_pid_output_c = channels.get("temp_pid_output_c")
        rh_pid_output_pct = channels.get("rh_pid_output_pct")
        age_s = (published or {}).get("age_s")
        pid = (published or {}).get("pid_enabled")

        if published is None:
            self.state = "offline"
        elif isinstance(age_s, (int, float)) and age_s > settings.plc_stale_s:
            self.state = "offline"
        else:
            self.state = "online"

        diag = self._client_limit_facts()
        return self.base_status(
            summary="AutomationDirect CLICK PLC -- two hand-written ladder PID "
                    "loops (enclosure temperature, RH). This layer supervises: "
                    "it reads sensors and writes setpoints, it does not close "
                    "the loop.",
            implemented=True,
            host="%s:%d" % (settings.host, settings.port),
            allow_actuation=settings.allow_actuation,
            dashboard_poll_plc=settings.dashboard_poll_plc,
            polled_plc=polled,
            poll_skipped=poll_skipped,
            temp_c=temp,
            temp_sp_c=temp_sp,
            rh_pct=rh,
            rh_sp_pct=rh_sp,
            temp_pid_output_c=temp_pid_output_c,
            rh_pid_output_pct=rh_pid_output_pct,
            in_range=self._in_range(temp, temp_sp, rh, rh_sp),
            in_range_tolerance={"temp_c": self.in_range_tol_c,
                                "rh_pct": self.in_range_tol_pct,
                                "note": "unset -> in_range is None. TODO(operator): "
                                        "declare this chamber's in-range tolerances"},
            pid_enabled=pid,
            age_s=age_s,
            stale_after_s=settings.plc_stale_s,
            clients_limit_reached=diag["clients_limit_reached"],
            clients_limit_note=diag["note"],
            authentication="none -- the PLC project has [UserAccounts] Disable=1",
            error=publish_error,
            last_event=self._last,
        )

    def _transient_read(self, published: dict | None):
        """One connect -> read -> publish -> close. Returns ``(published, polled, error)``."""
        client = self.client()
        try:
            if not client.connect():
                return published, False, client.last_error
            record = client.read_block()
            client.publish_last_reading(record)
            fresh = dict(record.as_dict())
            fresh["age_s"] = 0.0
            return fresh, True, (None if record.read_ok else client.last_error)
        except Exception as exc:                                      # noqa: BLE001
            return published, False, "%s: %s" % (type(exc).__name__, exc)
        finally:
            client.close()

    def _client_limit_facts(self) -> dict:
        """Whether the CLICK's client limit was hit. It is ``None``, not False.

        The flag exists on the PLC as ``SC92`` (``Port_1_Clients_Limit``) and its
        **Modbus address is unknown**, so ``diagnostics()`` reports it
        unavailable. Reporting False here would assert something never read; the
        only honest answer is "not known", plus the one symptom that is
        observable -- a refused connection.
        """
        try:
            diag = self.client().diagnostics()
        except Exception as exc:                                      # noqa: BLE001
            return {"clients_limit_reached": None,
                    "note": "diagnostics unavailable: %s" % exc}
        native = diag.get("native_diagnostics", {}).get("SC92", {})
        last = diag.get("last_error")
        return {
            "clients_limit_reached": None,
            "note": "%s (%s). Observable symptom instead: a refused connection, "
                    "last_error=%r. Vendor-confirmed limit: %d concurrent clients."
                    % (native.get("status", "unavailable: address unknown"),
                       native.get("todo", ""), last,
                       diag.get("max_concurrent_clients", 3)),
        }

    def _in_range(self, temp: Any, temp_sp: Any, rh: Any, rh_sp: Any) -> bool | None:
        """Tri-state. ``None`` means not known, which is not the same as False."""
        verdicts = []
        for value, setpoint, tol in ((temp, temp_sp, self.in_range_tol_c),
                                     (rh, rh_sp, self.in_range_tol_pct)):
            if tol is None:
                continue
            if not isinstance(value, (int, float)) or not isinstance(setpoint, (int, float)):
                return None
            verdicts.append(abs(float(value) - float(setpoint)) <= float(tol))
        if not verdicts:
            return None
        return all(verdicts)

    # -- commands ---------------------------------------------------------
    def commands(self) -> list[str]:
        """The dashboard surface. NO ``enable_pid`` -- see the class docstring."""
        return ["status", "read", "set_temperature", "set_humidity", "disable_pid"]

    def command(self, name: str, **kwargs: Any) -> dict:
        try:
            if name == "status":
                return self.status()
            if name == "read":
                return self._record("read", self._read_once())
            if name == "set_temperature":
                return self._record("set_temperature",
                                    self._write("temp_sp_c", kwargs.get("target_c")))
            if name == "set_humidity":
                return self._record("set_humidity",
                                    self._write("rh_sp_pct", kwargs.get("target_pct")))
            if name == "disable_pid":
                # Always allowed: off is the safe direction, and a supervisor
                # that may not write must still be one that can stop.
                return self._record("disable_pid", self._set_pid(False))
            if name == "enable_pid":
                return {"action": name, "ok": False, "refused":
                        "enable_pid is not a dashboard command and will not be "
                        "added: enabling the ladder PID hands the chamber's "
                        "heater and humidifier to the loop, which is an "
                        "actuation that must be made deliberately from a script. "
                        "disable_pid is available, because off is the safe "
                        "direction."}
        except (ActuationNotAllowed, SafetyError, occupancy.Busy) as exc:
            self.log("%s refused: %s" % (name, exc), "warn")
            return {"action": name, "ok": False, "refused": str(exc)}
        except Exception as exc:  # report on the bus, never raise into it
            self.log("%s failed: %s" % (name, exc), "error")
            return {"action": name, "ok": False, "error": str(exc)}
        return {"action": name, "ok": False,
                "error": "no such environment command: %r" % name}

    def _read_once(self) -> dict:
        """One claimed, transient read, published for whoever renders next."""
        with occupancy.claim("environment", doing="dashboard read"):
            client = self.client()
            try:
                if not client.connect():
                    raise SafetyError(
                        "could not open a Modbus session: %s" % client.last_error)
                record = client.read_block()
                client.publish_last_reading(record)
                return record.as_dict()
            finally:
                client.close()

    def _write(self, field: str, target: Any):
        if target is None:
            raise SafetyError(
                "%s needs a target value; refusing to guess one" % field)
        # Bound FIRST -- before a claim is taken and before a socket is opened.
        # A bad number is a refusal, not an operation that got as far as the wire.
        sp = self.settings.limits.validate(field, target)
        if not self.settings.allow_actuation:
            # A plan constructs no transport and opens nothing, so it needs no
            # claim either: there is no session to contend for.
            return self.client().write_float32(sp, dry_run=True)
        with occupancy.claim("environment", doing="dashboard %s" % field):
            client = self.client()
            try:
                if not client.connect():
                    raise SafetyError(
                        "could not open a Modbus session: %s" % client.last_error)
                return client.write_float32(sp, dry_run=False)
            finally:
                client.close()

    def _set_pid(self, enabled: bool):
        with occupancy.claim("environment", doing="dashboard set_pid(%r)" % enabled):
            client = self.client()
            try:
                if not client.connect():
                    raise SafetyError(
                        "could not open a Modbus session: %s" % client.last_error)
                return client.set_pid(enabled)
            finally:
                client.close()

    @staticmethod
    def _as_dict(payload: Any) -> Any:
        """A JSON-serialisable view of whatever the device layer returned.

        ``tools.environment.plc.WriteResult`` has no ``as_dict()`` of its own and
        this node may not add one to it, so the fields are lifted here --
        including the derived ``ok`` and ``dry_run``, which
        ``dataclasses.asdict`` would drop. A status page that lost them would
        show a plan and a confirmed write as the same thing.
        """
        if hasattr(payload, "as_dict"):
            return payload.as_dict()
        if hasattr(payload, "outcome") and hasattr(payload, "values"):
            return {"outcome": payload.outcome, "ok": payload.ok,
                    "dry_run": payload.dry_run, "address": payload.address,
                    "values": list(payload.values),
                    "readback": (None if payload.readback is None
                                 else list(payload.readback)),
                    "error": payload.error, "field": payload.field,
                    "value": payload.value, "describe": payload.describe()}
        return payload

    def _record(self, action: str, payload: Any) -> dict:
        result = self._as_dict(payload)
        # The envelope's `ok` answers "did the command run without refusal or
        # error", which a dry-run plan DOES. It deliberately does NOT mirror
        # WriteResult.ok, which answers the narrower "was the setpoint actually
        # written" and is False for a plan. So `outcome` is lifted to the top
        # level: a caller must never have to infer a plan from a bool.
        outcome = result.get("outcome") if isinstance(result, dict) else None
        ran = outcome != "failed"
        out = {"action": action, "ok": bool(ran), "result": result}
        if outcome is not None:
            out["outcome"] = outcome
        self._last = out
        note = {"planned": "PLANNED (nothing sent)",
                "confirmed": "confirmed",
                "failed": "NOT CONFIRMED"}.get(outcome or "", "ok")
        self.log("%s: %s" % (action, note), "info" if ran else "warn")
        self.publish_status()
        return out


__all__ = ["EnvironmentNode"]
