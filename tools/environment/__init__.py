"""Enclosure temperature and humidity — an AutomationDirect CLICK PLC over Modbus TCP.

The PLC runs **two hand-written ladder PID loops** — one for enclosure
temperature, one for relative humidity — reads the two 4-20 mA sensors, and
computes both outputs itself. **This package is a supervisor, not a controller:**
it reads sensors, writes the two setpoints, and forwards the temperature loop's
output to the circulator. It does not close a loop.

    from tools import environment

    print(environment.provenance())              # measured vs inferred, per register
    reading = environment.read("configs/config.yaml")
    print(reading.temp_filtered_c, reading.temp_sp_c, reading.pid_enabled)

    environment.set_temperature(24.0, "configs/config.yaml", dry_run=True)

Layers, lowest to highest:

``registers``   the register map: DF1..DF23 at ``28672 + 2*(n-1)``, coil C1, the
                float32 codec (low word at the LOWER address). The address
                formula is **THIRD_PARTY, not vendor-certified**
``reading``     ``EnvironmentReading`` + ``decode_block`` — pure decoding, no I/O.
                A short block is not an error: it sets ``partial`` and decodes
                what arrived
``safety``      ``SetpointLimits`` + ``BoundedSetpoint`` — the STRICT ``(0, 30)``
                degC and ``(0, 100)`` %RH bounds, enforced by type; both
                endpoints refused. Plus ``RateLimiter``, with an injected clock
``plc``         ``PlcClient`` + ``PlcSettings`` — the only transport. One FC16 of
                two registers per setpoint, read straight back and compared
``relay``       ``Relay`` + ``RelayPolicy`` — DF9 out to the circulator, one
                iteration per ``step()``. The loop belongs to a phase script
``node``        Lab node wrapper for the dashboard (thin; never raises into the
                bus; never holds a Modbus session by default)
``api``         the public surface; **import from here**

This package **reports**. It never decides: no retry, no substituted value, no
clamp. A refused setpoint raises; a failed write comes back with its outcome, and
the phase script chooses what to do. The two exceptions to "no decision" are both
pre-declared by the operator before a run: ``allow_actuation``, and the relay's
``RelayPolicy``.

Three facts worth knowing before touching anything here:

* **The CLICK accepts at most three concurrent Modbus TCP clients** and refuses
  the fourth (vendor-confirmed). Sessions are short, the dashboard renders a
  published reading rather than polling, and ``tools.occupancy`` carries an
  ``"environment"`` self-conflict to protect the limit.
* **``read_ok`` is not a sufficient gate.** ``decode_block`` sets it True for a
  short block, so a reading missing the channel you want still reports
  ``read_ok=True``. Check ``partial`` and the channel's presence.
* **The PLC has no authentication** — its project file sets
  ``[UserAccounts] Disable=1``. Anything that can reach the host can write it.

See ``README.md`` beside this file for the register table, the provenance of
every number, and what is still ``MEANING UNKNOWN``.
"""
from . import plc, reading, registers, relay, safety
from .api import (
    BLOCK_COUNT,
    BY_FIELD,
    C1_COIL,
    CONFIG_SECTION,
    CONFIRMED,
    COOPERATIVE_WATCHDOG_RESIDUAL,
    DEFAULT_DEVICE_ID,
    DEFAULT_HOST,
    DEFAULT_PORT,
    DF_BASE,
    FAILED,
    MAX_CONCURRENT_CLIENTS,
    NATIVE_DIAGNOSTICS,
    PLANNED,
    REGISTERS,
    UNKNOWN_DF,
    WRITABLE,
    ActuationNotAllowed,
    BoundedSetpoint,
    ChannelQuality,
    CirculatorSafetyError,
    CoilSpec,
    CommsLost,
    Environment,
    EnvironmentNode,
    EnvironmentReading,
    FailSafeIncomplete,
    FakePlc,
    PlcClient,
    PlcError,
    PlcSettings,
    Provenance,
    RateLimited,
    RateLimiter,
    RegisterSpec,
    Relay,
    RelayPolicy,
    RelayRecord,
    SafetyError,
    SetpointLimits,
    WriteResult,
    decode_block,
    df_address,
    diagnostics,
    disable_pid,
    enable_pid,
    f32_to_regs,
    latest_published,
    pid_enabled,
    provenance,
    provenance_table,
    publish_last_reading,
    read,
    regs_to_f32,
    set_humidity,
    set_temperature,
)

__all__ = [
    "BLOCK_COUNT",
    "BY_FIELD",
    "C1_COIL",
    "CONFIG_SECTION",
    "CONFIRMED",
    "COOPERATIVE_WATCHDOG_RESIDUAL",
    "DEFAULT_DEVICE_ID",
    "DEFAULT_HOST",
    "DEFAULT_PORT",
    "DF_BASE",
    "FAILED",
    "MAX_CONCURRENT_CLIENTS",
    "NATIVE_DIAGNOSTICS",
    "PLANNED",
    "REGISTERS",
    "UNKNOWN_DF",
    "WRITABLE",
    "ActuationNotAllowed",
    "BoundedSetpoint",
    "ChannelQuality",
    "CirculatorSafetyError",
    "CoilSpec",
    "CommsLost",
    "Environment",
    "EnvironmentNode",
    "EnvironmentReading",
    "FailSafeIncomplete",
    "FakePlc",
    "PlcClient",
    "PlcError",
    "PlcSettings",
    "Provenance",
    "RateLimited",
    "RateLimiter",
    "RegisterSpec",
    "Relay",
    "RelayPolicy",
    "RelayRecord",
    "SafetyError",
    "SetpointLimits",
    "WriteResult",
    "decode_block",
    "df_address",
    "diagnostics",
    "disable_pid",
    "enable_pid",
    "f32_to_regs",
    "latest_published",
    "pid_enabled",
    "plc",
    "provenance",
    "provenance_table",
    "publish_last_reading",
    "read",
    "reading",
    "registers",
    "regs_to_f32",
    "relay",
    "safety",
    "set_humidity",
    "set_temperature",
]
