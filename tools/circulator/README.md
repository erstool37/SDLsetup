# tools/circulator

Temperature **setpoint** sink: a custom MCU board on Modbus RTU over serial.
Write-only, one register block, no read-back.

## ⚠️ OPENING THE PORT RESETS THE MICROCONTROLLER

The board is reached through an **FTDI FT232R** USB-serial bridge. On such a
board **DTR is capacitively coupled to `/RESET`, and the OS asserts DTR when the
port is opened — before a single byte is sent.** So the act of opening the port
hardware-resets the MCU.

There is **no read-only identify query.** You cannot "just check what it is".

Consequences, all enforced in code:

- `open()` is an **actuating operation**: refused unless `allow_actuation` is
  set, counted in `open_count`, and logged as a reset event.
- `port_present()` answers presence with **`stat` only** and never opens
  anything. It can say a path exists; it cannot say what is on the other end.
  That is the honest limit of what is knowable without actuating.
- After a reset the firmware boots. `boot_settle_s` is that window and a frame
  attempted inside it **raises**.
- No test in this repo opens a real port. Every test uses `FakeSerial`, whose
  `open_count` exists precisely so a test can assert *no reset happened*.

## What is known

**Exactly one transaction.** A write of **4 holding registers at 0-based
address 980**, carrying an IEEE-754 **float64** Celsius setpoint.

Transport, verbatim from the prior working code:

```python
ModbusSerialClient('COM3', baudrate=9600, parity='N', stopbits=1, bytesize=8, timeout=1)
```

`COM3` was a Windows name and does not carry over. Slave/unit id was **never
specified** in either the Python or the MATLAB version, so both used the
library default of **1** — that is an **inference**, not a documented fact.

### The encoding

**float64, byteorder = LITTLE, wordorder = LITTLE.** Worked example, `25.0`:

| step | bytes / words |
|---|---|
| IEEE-754 float64, MSB first | `40 39 00 00 00 00 00 00` |
| split into four 16-bit groups | `4039  0000  0000  0000` |
| bytes swapped inside each group | `3940  0000  0000  0000` |
| register order reversed | `0000  0000  0000  3940` |

so registers 980..983 receive `[0x0000, 0x0000, 0x0000, 0x3940]`.

Verified bit-identical to the prior system's own expression (MATLAB
`flip(typecast(flip(typecast(x,'uint8')),'uint16'))`) on all seven known
answers plus a spread of magnitudes — see
`dev/tests/test_circulator_codec.py`.

> **This is NOT the PLC's encoding.** The PLC uses **float32, byteorder BIG,
> wordorder LITTLE**. Two devices, two encodings, on the same bench, both
> carrying temperatures. Reusing one device's helper for the other yields a
> well-formed frame with a meaningless value in it and neither protocol
> complains. This is the obvious future bug; it is why the codecs are separate
> modules.

### The command bound: `10.0 < setpoint < 30.0` °C — **strict**

**Both endpoints are refused.** Exactly `10.0` and exactly `30.0` raise.

**The floor is MEASURED. The ceiling is still INFERRED.** The two halves stopped
sharing a provenance on 2026-09-07, and the difference is the point of this
section.

| Half | Provenance | Evidence |
|---|---|---|
| `10.0` floor | **MEASURED** | The live PLC's `DF21 Temp Output LB`, read read-only on 2026-09-07 — five samples, `read_holding_registers(28672, count=46)`, `device_id=1`, `C1 PID Auto` OFF, nothing written. The PLC clamps its own temperature-PID output to `[10.0, 30.0]` °C, so **10 °C is the lowest value the circulator can legitimately receive.** |
| `30.0` ceiling | **INFERRED** | The top of the prior system's observed output envelope across all 49,147 rows of its logs (`T output ∈ [0.000, 30.000]`). It agrees with the measured `DF20 Temp Output UB = 30.0`, which is corroboration — but the log envelope alone was never independent evidence. |

**The floor superseded an inference that was wrong.** It used to be `0.0`, taken
from the low end of that same log envelope. But `0.000` in those logs was the
*prior system's own behaviour*, not the PLC's clamp, and the PLC clamps at
`10.0`. Do not restore `0.0` from any log-derived argument: the measurement
outranks it. The four measured clamps live in
`tools/environment/registers.py` → `MEASURED_OUTPUT_CLAMPS`.

**The code ceiling in `safety.py` was deliberately left at `0.0`.**
`COMMAND_MIN_C` is the widest *range* the layer permits and `config.yaml`
tightens it to the measured `10.0`. It was not raised because DF21 is a fact
about what the *PLC emits*, not about what *this board tolerates*, and raising a
code ceiling is a hardware-clearance decision about a board whose own
protections are unknown. Two consequences:

- `RelayPolicy` validates `safe_setpoint_c` against `CommandLimits()` — the code
  ceiling `(0, 30)` — **not** against the tightened config range. A declared
  safe setpoint in `(0, 10]` passes policy construction and is then **refused
  during an abort**. `scripts/environment/hold_environment.py` re-checks it
  against the resolved range before its loop starts; nothing else does.
- `TODO(operator)`: decide whether `COMMAND_MIN_C` should become `10.0` too —
  one line in `safety.py` plus its assertion in
  `dev/tests/test_circulator_safety.py`, closing the gap at the type level.

There is still no datasheet.

**The maximum is partly an artifact.** The 2026-01-11 log ends with the
temperature output pinned at exactly `30.000` while the measured temperature
read 11.4 °C — saturated against the clamp during an aborted run. `30.000` is
therefore a value the *broken* system produced. The number is kept because a
bound cannot honestly be looser than what the hardware has already been driven
to; it is **not** evidence that 30.0 is safe.

**Why strict:**

- the MATLAB original enforced `Temp SP ∈ (0, 30)` as an open interval,
  refusing exactly 0.0 and 30.0;
- the PLC-side setpoint guard is strict too, so no caller has to remember which
  device behaves which way at its limits;
- and `30.000` is precisely what a pinned, failing controller emits — the worst
  candidate for a legal command. The measurement extends this rather than
  softening it: `10.000` is now known to be the PLC's *other* clamp, so a value
  sitting exactly on a clamp is a saturation signature at either end, and the
  strictness holds at both for one reason instead of two.

**Why the bound is entirely ours:** the board is custom and undocumented, so
whether it has an over-temperature cutoff or a setpoint clamp of its own is
*unknowable*. It cannot be assumed to protect itself.

`config.yaml` may **tighten** the range for a run. It may never widen it.

**The bound is enforced at the wire, not only at `bound()` (G6, 2026-09-08).**
`SerialLink._check_frame` re-decodes every frame and re-validates the
temperature against the sink's resolved `CommandLimits` before anything is
sent. So the public `link.encode_frame()`/`write_registers()` surface cannot
put an out-of-bound value on the wire — neither a bare float `35.0`, nor a
value inside the code ceiling `(0,30)` but outside a tightened run bound
`(10,30)` (e.g. `5.0`). Only forging a `_SetpointFrame` through the private
`_FRAME_TOKEN`, `object.__setattr__`, or a private name bypasses it — the
documented Python residual, not an ordinary-code path.

**One deliberate asymmetry, so it does not read later as an inconsistency:** a
configured *range* of exactly `[0.0, 30.0]` is legal because it merely equals
the code ceiling, while the *values* `0.0` and `30.0` are still not commandable
within it. The code ceilings are exact float literals, so the range check
compares against them **exactly** — it no longer adds `EPS` (G1, 2026-09-08):
`+EPS` on the ceiling let `CommandLimits(max_c=30.0000005)` construct and then
let a real 2.5e-7 overshoot pass `validate()`. The value check remains strict
with no `EPS` at all.
The same asymmetry now applies one step in: `[10.0, 30.0]` is a legal *range*
because it tightens the ceiling, and `10.0` itself is still not a commandable
*value*.

> **Do not mirror `tools/environment/registers.py`'s `valid_min`/`valid_max`.**
> Those are checked *inclusively*, but they are the PLC's **sensor scaling**
> bounds (0..100 %RH, −40..180 °C), where an exact 100.0 is a legitimate
> *reading*. A **command** bound is a different quantity and is exclusive.
> `CommandLimits` is the single source of truth for what may be sent here.

### `parity` must be quoted in the config

`parity: N` unquoted parses to boolean **`False`** — the repo's parser reads
YAML 1.1 false-words `{false, no, off, n}`, and `_parse_scalar("N") -> False`
is verified in `dev/tests/test_circulator_safety.py`. It must be written
`parity: "N"`. `CirculatorSettings` accepts `"N"`, `"E"`, `"O"` only and
**raises** on anything else, including `False`, so the mistake surfaces as a
clear refusal rather than a baffling framing failure. Mark/space parity is
excluded: never used on this board and not verifiable here.

### Empty config keys parse to `{}`, not `None`

The repo's stdlib parser cannot tell `key:` (meaning null) from `key:` opening
a nested block, so an intentionally-blank key arrives as an **empty dict**.
`configs/config.yaml` ships `port:` and `safe_setpoint_c:` deliberately blank.
`CirculatorSettings.from_config` normalises an empty mapping to `None`
(→ "no port configured", which is refused cleanly); a **non-empty** mapping
where a scalar belongs **raises** rather than falling through. Without that,
`resolve(cast=str)` casts `{}` to the literal path `"{}"` — observed here
before it was fixed. `safe_setpoint_c` belongs to the relay layer, not this
one: it is never read here and no default is adopted for it.

## What is NOT known

The device has **no vendor, no model, no manual, no firmware source, no
schematic, no register map and no BOM.** The string `RW-3` in the prior code
was a **MATLAB cell-divider comment** — a section name written beside `%% PLC`
and `%% video` — and **not a model number.** No commercial documentation
applies.

Therefore `MEANING UNKNOWN` for:

- mixing / stirrer, flow and RPM registers — no address established for any;
- **read-back of any kind**: no temperature, no status word, no alarm. The
  prior code's only read was *commented out*, and its write's return value was
  assigned and never checked. Do not add a read on the assumption that a
  controller must have one;
- the official meaning of address 980, and of every neighbouring address —
  adjacency in a register space is not evidence;
- the DB9 signal assignment on the board side, and whether DTR could be left
  deasserted (`TODO(operator)`: trace it);
- whether the firmware range-checks the setpoint at all;
- the true unit/slave id.

## Layers

| Module | Job |
|---|---|
| `codec.py` | the one verified encoding; `SETPOINT_ADDRESS = 980`, `SETPOINT_COUNT = 4` |
| `safety.py` | `CommandLimits` + `BoundedSetpoint` — the bound, enforced by type |
| `link.py` | the only transport; opening is actuation, every write is checked against the FC16 echo; `FakeSerial` |
| `circulator.py` | `Circulator` + `CirculatorSettings` — what orchestration calls |
| `node.py` | dashboard adapter; thin, never raises into the bus. **No `open`** — see below |
| `api.py` | the public surface — **import from here** |
| `circulator.json` | hardware inventory, with provenance on every field |

`tools/` reports; `scripts/` decides. This package has no retry, no fallback
setpoint and no clamp: a refused bound **raises**, a failed write returns
`outcome="failed"` with the observed echo, and the phase script chooses.

### A write has three outcomes, and `ok` means only one of them

`WriteResult.outcome` is one of:

| `outcome` | meaning | `ok` |
|---|---|---|
| `"planned"` | a frame was composed and **nothing was sent** (dry run) | `False` |
| `"confirmed"` | sent, and the FC16 echo verified against what was sent | `True` |
| `"failed"` | sent or attempted, and **not** verified | `False` |

**`ok` is a derived property, true only for `"confirmed"`.** So a perfectly
successful dry run reports `ok=False` — **this is not a bug.** It is the honest
answer to "was the setpoint written?", which is no. The point is structural: a
caller who checks only `ok` *cannot* mistake a plan for a completed write, so
the mistake is impossible by default rather than merely warned about. `dry_run`
and `sent` are likewise derived from `outcome`, so they can never disagree with
it, and an unrecognised `outcome` raises rather than silently reading as
`ok=False`.

## The dashboard cannot open the port

`CirculatorNode.commands()` offers **`status`, `port_present`, `plan_setpoint`
and `set_setpoint` only.** There is deliberately **no `open` and no `close`**,
and they must not be added.

A one-click web button that hardware-resets an MCU is not acceptable, and
`allow_actuation` gating is **not** sufficient mitigation: the standing rule on
this lab surface is that anything which can physically actuate hardware is
confirmed deliberately, not exposed as a control on a status page.

The dashboard may not open the port **even transitively** — so `set_setpoint`
on a closed link returns `{"ok": False, "refused": "..."}` naming the MCU
reset, rather than opening the link on demand. A *script* may open it
explicitly.

`open_count` stays on the status card: it tells an operator how many times the
MCU has been reset this session, which is exactly the number worth watching.
`status` and `port_present` never open anything.

## Use

```python
from tools import circulator

dev = circulator.Circulator.from_config("configs/config.yaml")
print(dev.settings.describe())        # every value with the layer it came from
print(dev.port_present())             # stat only; opens nothing

sp = dev.bound(24.774)                # raises unless 10.0 < v < 30.0 (strict)
result = dev.write_setpoint(sp)       # dry-run plan unless allow_actuation

if result.outcome == "confirmed":     # equivalently: result.ok
    ...                               # the setpoint really is on the device
elif result.outcome == "planned":
    ...                               # nothing was sent; this was a dry run
else:                                 # "failed"
    ...                               # the SCRIPT decides what to do
```

Branch on `outcome`, not on `not result.ok` — `ok` is False for a dry run too,
so `if not result.ok` treats a plan and a failure alike.

Dry run (`allow_actuation=False`, the default) does not merely skip the write:
it **constructs no transport and never opens the port**, so the MCU is never
reset. `write_setpoint` accepts **only** a `BoundedSetpoint`, so a bare `float`
cannot reach the wire.

Live use adds an explicit, logged open:

```python
with circulator.Circulator.from_config(cfg) as dev:   # opens -> RESETS THE MCU
    result = dev.write_setpoint(dev.bound(24.774))    # waits out boot_settle_s
```

## Making the device reachable (operator steps)

The device is **not currently attached to WSL** and is not visible to Linux. It
must be forwarded from Windows with `usbipd`; **both commands need Windows
admin**:

```powershell
usbipd bind   --busid 2-1          # FTDI FT232R, VID:PID 0403:6001
usbipd attach --wsl --busid 2-1
```

Then, in WSL:

```bash
ls -l /dev/ttyUSB*                 # find the node -- does NOT open it
```

- `TODO(operator)`: record the resulting device path and set `circulator.port`
  in `configs/config.yaml`. It is deliberately blank: it depends on which busid
  was attached this session, and inventing one would open the **wrong** device.
- `TODO(operator)`: measure the real post-reset boot time and replace
  `boot_settle_s` (3.0 s, a conventional allowance for an AVR-class bootloader
  — **not measured on this board**) with the observed value plus margin.
- `pymodbus` is required for a live open and is pinned in `requirements.txt`
  (`pymodbus==3.15.0`, `pyserial==3.5`). The pin matters: 3.15 renamed the
  unit-id keyword from `slave=` to `device_id=`. `link.py` picks the right one
  by introspecting the client's signature, so it works either side of that
  rename, but the pin is what keeps the rest of the tree consistent.

## Tests

```bash
dev/lint.sh
dev/test.sh
```

| Test | Fences |
|---|---|
| `test_circulator_codec.py` | all seven known answers, both directions; non-finite refused — including `NaN`, which the prior code encoded to `[0,0,0,0xF87F]` and sent |
| `test_circulator_link.py` | dry run never opens and never builds a transport; open refused without permission or a port; one reset per session; a write inside the boot window raises; an echo mismatch is `ok=False`, not an exception; `port_present` never opens |
| `test_circulator_safety.py` | the strict bound — both endpoints refused, just inside accepted; the token cannot be built directly; `write_setpoint` refuses a plain float; config cannot widen the ceiling; a blank key is `None` and an unquoted `parity: N` (i.e. `False`) raises |
| `test_circulator_imports.py` | importing the package imports no vendor driver — with a control case proving the check is real |
