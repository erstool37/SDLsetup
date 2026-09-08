# environment — AutomationDirect CLICK PLC (enclosure temperature + humidity)

**This package supervises; it does not control.** The PLC runs two hand-written
ladder PID loops, reads the two 4-20 mA sensors, and computes both outputs
itself. This layer reads registers, writes the two setpoints, and forwards the
temperature loop's output to the circulator. Import from
[`api.py`](api.py) — everything callable from outside is there.

```python
from tools import environment

print(environment.provenance())               # measured vs inferred, per register
reading = environment.read("configs/config.yaml")
print(reading.temp_filtered_c, reading.temp_sp_c, reading.pid_enabled)
environment.set_temperature(24.0, "configs/config.yaml", dry_run=True)
```

## Read this before trusting a number

| Fact | Consequence |
|---|---|
| The address formula is **THIRD_PARTY, not vendor-certified** | every value below is well supported and not guaranteed |
| **`read_ok` is True for a SHORT block** | a reading missing your channel still reports `read_ok=True` |
| The **PLC has no authentication** (`[UserAccounts] Disable=1`) | anything that reaches the host can write setpoints |
| The CLICK accepts **at most 3 concurrent Modbus TCP clients** (vendor-confirmed) | sessions are short; the dashboard renders a published reading |
| `SC90`–`SC95` and `SD41` **addresses are unknown** | `diagnostics()` reports them `unavailable`, never a guess |
| The float32 **word order fails silently** if reversed | never hand the pair over backwards; see below |

## The register map

Addresses are **0-based protocol addresses** — what pymodbus takes directly, no
off-by-one adjustment.

```
DF n  ->  28672 + 2 * (n - 1)          # a float32 spans TWO registers
C 1   ->  16384
```

The formula comes from the open-source `numat/clickplc` driver, corroborated by
an Ignition forum thread, and cross-validated at six points against the
recovered working code for this rig. AutomationDirect publishes no static
address table for the CLICK, so its provenance is `THIRD_PARTY`.

**`TODO(operator): confirm via CLICK Address Picker with "Display MODBUS
Address" checked.`** That is the one action that would upgrade every row below
from `THIRD_PARTY` to `VENDOR`.

| DF | addr | field | PLC nickname | units | rw | provenance | verified live | valid range |
|---|---|---|---|---|---|---|---|---|
| 1 | 28672 | `rh_raw_pct` | RH | % | R | THIRD_PARTY | no | 0..100 |
| 2 | 28674 | `temp_raw_c` | Temp | C | R | THIRD_PARTY | no | -40..180 |
| 3 | 28676 | *(none)* | (AD3, no nickname) | UNKNOWN | R | **UNKNOWN** | no | – |
| 4 | 28678 | *(none)* | (AD4, no nickname) | UNKNOWN | R | **UNKNOWN** | no | – |
| 5 | 28680 | `temp_sp_c` | Temp SP | C | **RW** | THIRD_PARTY | no | – |
| 6 | 28682 | `rh_sp_pct` | RH SP | % | **RW** | THIRD_PARTY | no | – |
| 7 | 28684 | `temp_error_c` | Temp Error | C | R | THIRD_PARTY | no | – |
| 8 | 28686 | `rh_error_pct` | RH Error | % | R | THIRD_PARTY | no | – |
| 9 | 28688 | `temp_pid_output_c` | Tem PID Output | C | R | THIRD_PARTY | no | – |
| 10 | 28690 | `rh_pid_output_pct` | RH PID Output | % | R | THIRD_PARTY | no | – |
| 11 | 28692 | `temp_kp` | Temp Kp | – | R *(policy)* | THIRD_PARTY | no | – |
| 12 | 28694 | `temp_ti` | Temp Ti | – | R *(policy)* | THIRD_PARTY | no | – |
| 13 | 28696 | `rh_kp` | RH Kp | – | R *(policy)* | THIRD_PARTY | no | – |
| 14 | 28698 | `rh_ti` | RH Ti | – | R *(policy)* | THIRD_PARTY | no | – |
| 15 | 28700 | `temp_old_error_c` | Temp Old Error | C | R *(policy)* | THIRD_PARTY | no | – |
| 16 | 28702 | `rh_old_error_pct` | RH Old Error | % | R *(policy)* | THIRD_PARTY | no | – |
| 17 | 28704 | `alpha` | Alpha | – | R *(policy)* | THIRD_PARTY | no | – |
| 18 | 28706 | `temp_filtered_c` | Temp Filtered | C | R | THIRD_PARTY | no | – |
| 19 | 28708 | `rh_filtered_pct` | RH Filtered | % | R | THIRD_PARTY | no | – |
| 20 | 28710 | `temp_output_ub_c` | Temp Output UB | C | R *(policy)* | INFERRED | no | – |
| 21 | 28712 | `temp_output_lb_c` | Temp Output LB | C | R *(policy)* | INFERRED | no | – |
| 22 | 28714 | `rh_output_ub_pct` | RH Output UB | % | R *(policy)* | INFERRED | no | – |
| 23 | 28716 | `rh_output_lb_pct` | RH Output LB | % | R *(policy)* | INFERRED | no | – |
| C1 | 16384 | `pid_auto` | PID Auto | – | **RW** | THIRD_PARTY | no | – |

`provenance()` prints this table from the code, so it cannot drift from what the
codec actually uses. Prefer it over this copy.

### DF3 and DF4 are MEANING UNKNOWN

Both are wired analogue inputs the PLC scales 0-20 mA to 0..100, and **neither
has a nickname in the PLC project's own table.** What they measure is unknown.
They carry `field=None`, so no physical name can attach to them by accident, and
a decoded reading keeps their values in a separate `unknown_channels` dict —
never merged into `channels`. Guessing that one is a second thermocouple is
exactly how a wrong label becomes permanent.

The PLC's own `[CPUBuild]` scaling lines are carried verbatim, so a reading can
report *what* it does not understand:

```
AD3=DF3,20.0,0.0,100.0,0.0,1,0.02442,0
AD4=DF4,20.0,0.0,100.0,0.0,1,0.02442,0
```

`TODO(operator): identify what is wired to AD3 and AD4, or confirm nothing is.`

### DF11–DF17 and DF20–DF23 are read-only BY POLICY, not by capability

The PLC would accept writes to these — the PID gains, the filter alpha, the
stored previous error, and the output bounds. **This layer must not write them.**
The PID on this rig is hand-written in ladder logic (the CLICK's built-in PID
instruction block is entirely factory-default and unused), so these tune a
bespoke loop whose structure cannot be read back from the controller. Changing a
gain of a loop you cannot read is not tuning, it is guessing.

`WRITABLE` therefore contains exactly two entries: `temp_sp_c` and `rh_sp_pct`.

## Word order — the part that fails silently

Bytes within each 16-bit register are big-endian. Across the pair, **the
low-order 16 bits occupy the FIRST (lower-addressed) register.** Verified by
live round-trip:

```
25.0  ==  0x41C80000  ->  register[0] = 0x0000   (LSW, low address)
                          register[1] = 0x41C8   (MSW, high address)
```

Reversed, nothing raises: `regs_to_f32(0x41C8, 0x0000)` returns `2.36e-41`, and
a subtler pair returns a number that looks like a plausible reading. The live
failure this fixed was a write of `25.3` that read back as `25.3587` because only
the high word was sent. `dev/tests/test_environment_codec.py` asserts five
live-captured known answers *and* asserts the codec is not word-order invariant,
rather than reasoning about endianness.

## Setpoint bounds are a STRICT open interval, from the MATLAB GUI

```
0 < temp_sp_c  < 30     degC
0 < rh_sp_pct  < 100    %RH
```

The MATLAB GUI this layer replaces enforced those bounds **strictly** — a
setpoint of exactly `0` or exactly `30` was refused there, and is refused here.
The Python port that came after that GUI dropped every bound; restoring them is
the point of `safety.py`.

`configs/config.yaml` expresses them as a min/max pair, and a min/max pair
*reads* as inclusive. It is not. `SetpointLimits.validate` compares
`low < value < high`, never `<=`. **Do not "fix" this to inclusive:** doing so
quietly restores a setpoint the original system rejected, and the config file
cannot state the distinction on its own.

They are **code ceilings**: config may tighten them for a run and may never
loosen them. `SetpointLimits` refuses to construct a widened copy, so no moment
exists at which a widened limit is available to use.

**Guards raise, never clamp.** A setpoint silently moved from 45 C to 30 C is a
run whose record says one thing and whose chamber did another.

Two more caps, both **CHOSEN and UNMEASURED**: `max_setpoint_step_c: 5.0` /
`max_setpoint_step_pct: 10.0` (so a typo like `25 -> 2.5` is refused rather than
driving the loop across its range in one write) and
`min_setpoint_interval_s: 10.0`. `TODO(operator): measure the chamber's step
response and set these from it.`

Loop cadence: `relay_period_s: 1.0`, kept because the prior Python ran at 1 s.
The MATLAB original before it used 3 s.

## The relay — who owns which decision

The PLC computes `DF9` (`temp_pid_output_c`); the circulator is that loop's
actuator and receives `DF9` as its bath setpoint. Humidity is actuated by the
PLC itself as a PWM duty cycle and never transits this host, so there is no
humidity relay.

| Owner | Decision |
|---|---|
| **PLC** | the control law, and the value |
| **`RelayPolicy`, declared by the operator before the run** | the comms-loss setpoint, both staleness windows, the large-step flag threshold |
| **`Relay.step()`** | forward the PLC's value unchanged, or refuse — and execute the declared fail-safe |
| **The phase script** | the loop: cadence, iteration count, and what a `CommsLost` means for the run |

`step()` runs **exactly one iteration per call and returns a `RelayRecord`.**
`RelayRecord.as_dict()` is shaped for `run.record("environment", "relay", …)`.

What it will not do: smooth, slew-limit, clamp, retry, substitute, or hold. A
jump larger than `large_step_c` is recorded as a **quality fact** and forwarded
unchanged — shaping the value would silently alter the loop the PLC is running,
and the PLC would have no way to know.

`safe_setpoint_c` is **required and has no code default.** A safe setpoint nobody
declared is not safe, so `RelayPolicy` raises while it is unset rather than
picking one, and it is validated against the circulator's command bound at
construction — not at write time, which would surface the problem during an
abort. `TODO(operator): declare circulator.safe_setpoint_c in configs/config.yaml.`

**Never hold the last value.** A real log from 2026-01-11 ends with the forwarded
output pinned at exactly `30.000` while the measured temperature read `11.4` C.
The loop had saturated and the prior system kept forwarding it, because it had no
watchdog at all. On PLC staleness the relay writes the declared safe setpoint,
disables the PID, and raises `CommsLost`.

`__exit__` writes the safe setpoint and drives `C1` False **even while an
exception is propagating**, preserving the one real interlock the prior system
had (a `try/finally` writing `C1 = False`), and it suppresses nothing.

### The honest residual

**If the circulator's serial link is dead, the safe setpoint cannot be
delivered.** The relay writes it and reports the outcome; the MCU keeps whatever
value it last received, and that device has no read-back of any kind, so "we
wrote it" is the most that can ever be said. Recovering from that state needs a
person at the rig or a PLC-side ladder change. Nothing in software closes this
gap — it is available as `Relay.HONEST_RESIDUAL`, to be passed to `run.note()`,
because silence there would read as "handled".

### What this layer cannot protect against

**Every watchdog in the relay is cooperative, and cannot act if the host loop
stops.** `Relay.step()` is where PLC staleness, circulator write staleness, and
out-of-bound refusals are all detected — so they fire only while a phase script
keeps calling `step()`. If that process hangs, is `SIGKILL`ed, or loses power,
no `step()` runs, nothing fires, the PLC ladder keeps running its own loop, and
the bath holds whatever setpoint it last received.

Closing this gap needs a **PLC-ladder heartbeat timeout** — the CLICK's own
`SD41` (`_Port1_No_Comm_Time`) would let the ladder notice that this supervisor
stopped talking and drive itself safe. **Its Modbus address is unknown and was
not guessed** (an invented address reads some other register and reports it as a
link flag), so that heartbeat **does not exist yet**. `TODO(operator): establish
the SD41 address via the CLICK Address Picker, or add a ladder-side heartbeat.`

No background watchdog thread is added in Python on purpose: a tool that runs a
loop nobody started is a tool that decides, and it is invisible to the run
record. The residual is exposed as `Relay.COOPERATIVE_WATCHDOG_RESIDUAL` and
`hold_environment.py` records it into every run via `run.note()`.

## Diagnostics we cannot read

The CLICK's own comms flags exist and their nicknames are known. **Their Modbus
addresses are not**, so `diagnostics()` reports every one as
`unavailable: address unknown` rather than reading some other register and
calling it a link flag.

| Bit | Nickname | Address |
|---|---|---|
| `SC90` | `Port_1_Ready_Flag` | **unknown** |
| `SC91` | `Port_1_Error_Flag` | **unknown** |
| `SC92` | `Port_1_Clients_Limit` | **unknown** |
| `SC93` | `Port_1_IP_Resolved` | **unknown** |
| `SC94` | `Port_1_Link_Flag` | **unknown** |
| `SC95` | `Port_1_100MBIT_Flag` | **unknown** |
| `SD41` | `_Port1_No_Comm_Time` | **unknown** |

`TODO(operator): confirm via CLICK Address Picker with "Display MODBUS Address"
checked.` Until then `clients_limit_reached` is `None` — not `False` — and the
only observable symptom of the limit is a refused connection.

## Session discipline and the 3-client limit

The CLICK accepts at most **three** concurrent Modbus TCP clients and refuses the
fourth (vendor-confirmed, CLICK and CLICK PLUS manuals). So:

- `api` helpers open one session, do one thing, and close. There is no cached
  module-level session. A script doing several operations holds its own
  `PlcClient` and passes it as `client=`.
- A run publishes its reading (`publish_last_reading`) and the dashboard node
  **renders that** rather than polling. `age_s` is computed on read, so a stale
  file cannot claim to be fresh.
- `EnvironmentNode.status()` opens a transient session only when
  `environment.dashboard_poll_plc` is true **and** no `tools.occupancy` claim on
  `"environment"` exists. The PLC's self-conflict entry in `occupancy.CONFLICTS`
  exists for exactly this.
- `enable_pid` is **not** a dashboard command and must not be added: enabling the
  ladder PID hands the chamber's heater and humidifier to the loop, which is an
  actuation that must be deliberate. `disable_pid` stays, because off is the safe
  direction.

`device_id: 1` is an **INFERENCE**, not documented: the prior code never set a
unit id, so this is the pymodbus library default rather than a value read off the
PLC. `TODO(operator): confirm the CLICK's Modbus device id in its project file.`

## Files

| File | Holds |
|---|---|
| [`registers.py`](registers.py) | the map, the codec, `provenance_table()` |
| [`reading.py`](reading.py) | `EnvironmentReading`, `decode_block` — pure, no I/O |
| [`safety.py`](safety.py) | `SetpointLimits`, `BoundedSetpoint`, `RateLimiter` |
| [`plc.py`](plc.py) | `PlcClient`, `PlcSettings`, `WriteResult`, `FakePlc` |
| [`relay.py`](relay.py) | `Relay`, `RelayPolicy`, `RelayRecord` |
| [`node.py`](node.py) | the dashboard adapter (thin; never raises into the bus) |
| [`api.py`](api.py) | **the public surface** |

Tests: `dev/tests/test_environment_registers.py`, `_codec.py`, `_safety.py`,
`_relay.py`, `_config.py`, and `_integration.py` (real Modbus framing against a
dependency-free CLICK emulator on localhost — no PLC involved).
