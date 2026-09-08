"""Dashboard adapter for the circulator. A thin shell over :class:`Circulator`.

It holds no hardware logic: every method delegates to
:mod:`tools.circulator.api`, so the dashboard cannot drive the circulator in a
way a phase script could not.

**It must never raise into the bus.** :meth:`status` catches everything and
returns a dict carrying an ``error`` field; :meth:`command` returns
``{"ok": False, "refused": ...}`` for a refusal and ``{"ok": False,
"error": ...}`` for anything else. The precedent is documented: the previous
UV-Vis node's constructor raised ``TypeError`` and **the dashboard could not
start at all** once that node was registered. So the device here is built
lazily, on first use, and constructing this node touches nothing.

.. warning::

   **Opening the port is deliberately NOT reachable from the dashboard, and
   must not be added.**

   On this device opening the serial port **hardware-resets the
   microcontroller**: the FTDI FT232R bridge has DTR capacitively coupled to
   ``/RESET`` and the OS asserts DTR on open, before any byte is sent. A
   one-click web button that resets an MCU is not acceptable, and
   ``allow_actuation`` gating is **not** sufficient mitigation -- the standing
   rule on this lab surface is that anything which can physically actuate
   hardware is confirmed deliberately, not exposed as a control on a status
   page.

   So :meth:`commands` offers ``status``, ``port_present``, ``plan_setpoint``
   and ``set_setpoint`` only. There is no ``open`` and no ``close``. A *script*
   may open the port explicitly; the dashboard may not -- **not even
   transitively**, which is why ``set_setpoint`` refuses rather than opening a
   closed link on demand.

   ``open_count`` stays on the status card: it tells an operator how many times
   the MCU has been reset this session, which is exactly the number worth
   watching. ``status`` and ``port_present`` never open anything.
"""
from __future__ import annotations

from typing import Any

from ..node import Node
from .circulator import Circulator, CirculatorSettings
from .safety import ActuationNotAllowed, CirculatorError, SafetyError


class CirculatorNode(Node):
    kind = "circulator"

    def __init__(self, name: str = "circulator", *, config: Any = None,
                 settings: CirculatorSettings | None = None) -> None:
        super().__init__(name)
        self._config_source = config
        self._settings = settings
        self._device: Circulator | None = None
        self._last: dict | None = None

    # -- the device -------------------------------------------------------
    @property
    def device(self) -> Circulator:
        """Built on first use, so importing or registering this node needs no hardware."""
        if self._device is None:
            if self._settings is not None:
                self._device = Circulator(self._settings, log=self._device_log)
            else:
                self._device = Circulator.from_config(self._config_source,
                                                      log=self._device_log)
        return self._device

    def _device_log(self, message: str, level: str = "info") -> None:
        self.log(message, level)

    # -- status -----------------------------------------------------------
    def status(self) -> dict:
        try:
            device = self.device
            presence = device.port_present()
        except Exception as exc:
            self.state = "error"
            return self.base_status(
                implemented=True, present=False, error=str(exc),
                summary="circulator temperature setpoint (configuration error)")
        settings = device.settings
        self.state = "online" if presence.get("present") else "offline"
        return self.base_status(
            summary="custom MCU circulator -- temperature SETPOINT ONLY, write-only, "
                    "no read-back. Opening the port RESETS the MCU.",
            implemented=True,
            port=presence.get("port"),
            port_present=bool(presence.get("present")),
            port_note=presence.get("note"),
            allow_actuation=settings.allow_actuation,
            open_count=device.open_count,
            link_open=device.link.is_open,
            settle_remaining_s=round(device.link.settle_remaining_s, 3),
            command_range_c=[settings.command_min_c, settings.command_max_c],
            command_range_provenance="INFERRED from 49,147 logged rows of the prior "
                                     "system; no datasheet exists",
            readback="none -- this device reports no temperature, status or alarm",
            last_event=self._last,
        )

    # -- commands ---------------------------------------------------------
    def commands(self) -> list[str]:
        """The dashboard surface. NO ``open``/``close`` -- see the class docstring."""
        return ["status", "port_present", "plan_setpoint", "set_setpoint"]

    def command(self, name: str, **kwargs: Any) -> dict:
        try:
            if name == "status":
                return self.status()
            if name == "port_present":
                return self._record("port_present", self.device.port_present())
            if name == "plan_setpoint":
                target = kwargs["target_c"]
                return self._record("plan_setpoint",
                                    self.device.plan_setpoint(self.device.bound(target)))
            if name == "set_setpoint":
                target = kwargs["target_c"]
                device = self.device
                # Bound FIRST, so a bad number is refused before anything else.
                sp = device.bound(target)
                if device.settings.allow_actuation and not device.link.is_open:
                    # The write path would raise here anyway; refusing explicitly
                    # names the reason instead of surfacing it as an "error", and
                    # makes it unmistakable that the dashboard will NOT open the
                    # port on demand -- opening it resets the MCU.
                    raise ActuationNotAllowed(
                        "refusing to set the setpoint: the port is not open, and the "
                        "dashboard does not open it -- opening asserts DTR on the "
                        "FT232R bridge and hardware-RESETS the MCU. A script must "
                        "open the link explicitly first."
                    )
                return self._record("set_setpoint", device.write_setpoint(sp))
        except (ActuationNotAllowed, SafetyError) as exc:
            self.log(f"{name} refused: {exc}", "warn")
            return {"action": name, "ok": False, "refused": str(exc)}
        except Exception as exc:  # report on the bus, never raise into it
            self.log(f"{name} failed: {exc}", "error")
            return {"action": name, "ok": False, "error": str(exc)}
        return {"action": name, "ok": False,
                "error": "no such circulator command: %r" % name}

    def _record(self, action: str, payload: Any) -> dict:
        result = payload.as_dict() if hasattr(payload, "as_dict") else payload
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
        self.log(f"{action}: {note}", "info" if ran else "warn")
        self.publish_status()
        return out


__all__ = ["CirculatorError", "CirculatorNode"]
