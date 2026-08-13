# uv_vis — BMG LABTECH SPECTROstar Nano

UV-Vis absorbance microplate reader. **Implemented** (was vacant until 2026-07-29).

## The instrument

Read from the reader's own `HardwareConfig.log`, not from a datasheet:

| | |
|---|---|
| Reader serial | `0601-003639` |
| Spectrometer | VS70 module, firmware 3.02, serial 2593 |
| Detector | Hamamatsu **S11071** CCD, max wavelength **1050 nm** |
| Incubator | built in — 3 channels (TMP117 sensors) + humidity sensor |
| Windows PnP id | `USB\VID_0483&PID_A29A\0601-003639`, class `MPRWinUSB` |

Full-spectrum absorbance: the CCD reads the whole range per well, so a "spectrum"
is one measurement, not a monochromator sweep.

## Control path

```
WSL Python  ->  bmg_bridge.ps1 (32-bit powershell.exe)
            ->  COM: BMG_ActiveX.BMGRemoteControl
            ->  SPECTROstar Nano control software
            ->  reader
```

Two hard constraints:

- **32-bit only.** `BMG_ActiveX.ocx` is an x86 in-proc COM server; 64-bit
  PowerShell cannot load it. The bridge refuses to run under a 64-bit host.
- **Interop is logon-session bound.** Same failure mode as the cameras — a
  bridge call from an ssh-launched process can die with
  `UtilAcceptVsock accept4 failed 110` once that ssh session ends. Drive
  unattended sequences from the operator's desktop login.

### COM surface

Enumerated live from the registered control (2026-07-29) — not guessed:

```
OpenConnection(string, Variant)   OpenConnectionV(Variant, Variant)
CloseConnection()                 CloseConnectionWithoutTerminatingControlSoftware()
Execute(Variant, Variant)         ExecuteAndWait(Variant, Variant)
sExecute(10x string, Variant)     sExecuteAndWait(10x string, Variant)
GetInfo(string, Variant)          GetInfoV(Variant, Variant)
GetVersion(Variant)
```

The trailing `Variant` is a **by-reference OUT status string**: empty = success,
non-empty = error text. Every call in the bridge passes `[ref]`.

Command verbs confirmed as literals inside `SPECTROstar_Nano.exe`:
`Init`, `PlateIn`, `PlateOut`, `Run`, `Temp`, `GetInfo`.

## Commands

| Command | Moves hardware | Notes |
|---|---|---|
| `status` | no | reader presence + control-software status |
| `version` | no | `GetVersion`; works with no reader attached |
| `init` | **yes** | homes the carrier |
| `plate_out` | **yes** | eject carrier — the "open" |
| `plate_in` | **yes** | retract carrier — the "close" |
| `run_protocol` | **yes** | runs a named protocol, then collects exports |
| `set_temperature` | **yes** | incubator setpoint, °C |
| `list_data` / `latest_data` | no | inspect the export directory |

### Safety gate

Everything that moves is refused unless `uv_vis.allow_uv_vis_motion: true` in
`config.yaml` — mirroring `allow_arm_motion`. **Never open the carrier while the
robot arm is over the reader.**

## Data and logging

- `run_protocol` snapshots the export dir before the run, then copies whatever
  is new into `log_dir/<UTC timestamp>_<protocol>/`.
- Every command that changes state appends a JSON line to `log_dir/runs.jsonl`
  (`ts`, `node`, `event`, plus command fields, duration, artifact paths).
- Defaults: export `…/BMG/SPECTROstar Nano/User/Data`, log `/home/lamp/SDLsetup/dataset/uv_vis_runs`.

## Configuration

```yaml
uv_vis:
  reader: SPECTROstar Nano
  allow_uv_vis_motion: false
  export_dir: /mnt/c/Program Files (x86)/BMG/SPECTROstar Nano/User/Data
  log_dir: /home/lamp/SDLsetup/dataset/uv_vis_runs
  timeout_s: 180.0
```

## The ActiveX call contract — read this before touching the bridge

Driving `BMG_ActiveX.BMGRemoteControl` from PowerShell has four traps. All four
were established empirically on 2026-07-29 and are restated in
`bmg_bridge.ps1`'s header. Do not "simplify" any of them.

**1. Commands must be ARRAYS.** This is the one that matters:

```powershell
$com.ExecuteAndWait(@('PlateOut'), [ref]$r)   # works
$com.ExecuteAndWait('PlateOut',   [ref]$r)   # 0x8000FFFF E_UNEXPECTED
```

`Execute`/`ExecuteAndWait` take `VT_BYREF|VT_VARIANT`. `@(...)` marshals as a
SAFEARRAY of VARIANT, which is what the control expects and what Python's
comtypes produces automatically from a list (hence nobody documents it).
A bare string throws `E_UNEXPECTED`, which misleadingly reads as "not
connected". Parameters are extra elements: `@('Temp','37')`,
`@('Run', $protocol, $plateId)`.

**2. Use the `…V` overloads to connect.** `OpenConnection`/`GetInfo` take
`VT_LPSTR` (raw `char*`), which PowerShell cannot marshal — it sends `BSTR` and
gets `0x80020008 DISP_E_BADVARTYPE`. `OpenConnectionV`/`GetInfoV` take VARIANT
and work. The service name is `SPECTROstar_Nano`, with an **underscore**.

**3. Plain verbs, never `R_`-prefixed ones.** The `R_PlateOut` family belongs to
the `.btc` script language, and BMG's manual states script mode is unavailable
while the software is in ActiveX/DDE mode. Use `Init`, `PlateIn`, `PlateOut`,
`Run`, `Temp`.

**4. Three things crash the process outright** — an AccessViolation is a
corrupted-state exception that `try/catch` cannot intercept, so the host dies
with exit 5 and an exception that will not even stringify:
`$com.GetType()` / `Get-Member` / `Type.InvokeMember` (its
`IDispatch::GetIDsOfNames` faults on any name it does not implement);
`ExecuteAndWait2` and `sExecuteAndWait2`; and driving the control from
cscript/VBScript at all.

Reading results back is impossible from PowerShell: every trailing OUT status
string returns empty even on calls that demonstrably reached the reader. Success
is therefore inferred from "no exception", and must be corroborated by an
observable — the control software's run log (`SPECTROstar Nano.log`) records
`(Plate command) / Processing Plate command ...` for each accepted command, and
that log is the ground truth for verification.

## Where measurement data actually goes

Runs are written into `User/Data/MeasurementData.abs`, a ~28 MB "ABS0LUTE
DATABASE" — **not** individual per-run export files. Diffing the export
directory therefore does not capture results; data must be exported through the
control software. `cmd_run_protocol`'s artifact collection is written against
the export-directory model and needs rework for the database model.

## Not yet verified against hardware


**Verified working 2026-07-29:** reader detection, `connect`, `status`, the
safety gate, and **plate carrier open/close** (`PlateOut`/`PlateIn`), the last
confirmed by `(Plate command)` entries in the reader's run log.

Still unverified:

1. `run_protocol` — the `@('Run', protocol, ids…)` argument layout is untested
   against a real protocol, and a protocol must exist in the control software.
2. Result capture — see "Where measurement data actually goes" above; the
   current export-diff logic is known to be wrong for this instrument.
3. `set_temperature` (`@('Temp', degC)`).
4. Reading any status text back — believed impossible from PowerShell.

Note the vendor help file `ExeDLL/SPECTROstar_Nano.chm` documents only the
`.btc` script language and contains **no** ActiveX/DDE reference; decompile it
with `hh.exe -decompile <dir> <chm>` if needed.

## Two control paths, and why they cannot be mixed

BMG's manual states verbatim: *"The script mode is not available when the program
is used in ActiveX or DDE mode, e.g. as part of a robotic system."* A single
control-software session is therefore **either** ActiveX-driven **or**
script-driven. This is not a preference — it is why an earlier `/s` launch
initialised the reader and then silently ignored every script line: an ActiveX
session had already put the software in ActiveX/DDE mode.

| | ActiveX (`reader.py`) | Script (`script.py`) |
|---|---|---|
| Carrier open/close | yes (**verified**) | `R_PlateOut` / `R_PlateIn` |
| Select a protocol by name | yes | yes |
| **Define which wells** | **no** | **yes** — `R_EditLayout` |
| Set standard concentrations | no | yes — `R_EditConcAndVol` |
| Read values back | no | yes — `R_GetData` |
| Latency | fast, per command | whole-run, ~45 s startup |

Practical split: drive the arm hand-off and carrier motion over ActiveX, then
close that session and run a generated script for the measurement itself.
`run_script(kill_existing=True)` enforces the handover.

## Well selection — the script workaround

`R_EditLayout` rewrites a protocol's layout at runtime, so wells become a
parameter instead of something drawn by hand in the UI:

```python
assay = AssayConfig(protocol="BCA 1", blank_wells=["H12"],
                    standard_wells=["A1","A2","A3"],
                    standard_concentrations=[0, 250, 500])
script = build_assay_script(assay, WellSelection.parse("B1:B6", PlateSpec.wells96()))
run_script(script, name="bca_run")
wait_for_marker("RUNEND")
```

which generates `R_EditLayout "BCA 1" "EmptyLayout A1=S1 A2=S2 A3=S3 H12=B
B1=X1 … B6=X6"`, runs it, then reads every well back. `EmptyLayout` is always
prepended — a stale layout is invisible in the results and would silently
mis-assign wells. Content codes: `B` blank, `S<n>` standard, `X<n>` sample.

Use `build_list_protocols_script()` to discover what protocols exist
(`R_GetProtocolNames`), since they cannot be read out of the Paradox `.DB`
tables.

## Getting data out

There are no per-run export files — results go into `MeasurementData.abs`, a
proprietary database. The route out is `R_GetData` per well, echoed into the Run
Log with `AddToMemo "DATA <well>=<value>"`, then
`parse_run_log_data()` on the CP949-encoded log. `R_GetData` requires the
software's *"save measurement data in Absolute Database format"* option to
remain on (Settings | Data Output).

## Saturation analysis

`analyse_absorbances()` returns a `SaturationReport`. A concentration is
**never** attached to a reading that failed its range check — saturated wells
report `concentration=None` plus a suggested dilution, rather than a plausible
lie. Absorbance is logarithmic (A=2 → 1% transmitted, A=3 → 0.1%), so linearity
degrades before the electronics clip; the default ceiling is 2.5 with a warning
at 2.0 and a detection floor at 0.01.

`report.orchestrator_view()` is the compact decision payload:

```python
{'ok': False, 'n_saturated': 1, 'saturated_wells': ['A4'],
 'needs_dilution': {'A4': 2.8, 'A6': 2.0}, 'action': 'dilute_and_reread'}
```

`action` is one of `proceed` / `dilute_and_reread` / `review`.
`write_reports(report, dir)` emits JSON and YAML side by side.

## Plate library (`plates.py`)

73 vendor plate geometries (`Stamm/Platedef.DB`, shipped as
`plate_library.json`) — heights, well diameters, corner coordinates. **BMG
publish no maximum accepted plate height**; `TALLEST_DEFINED_MM` is only the
tallest plate the vendor ships a *definition* for, an empirical lower bound on
drawer clearance, not a ceiling. Several entries carry vendor-side `null`
height/depth/shape fields (e.g. `SARSTEDT 96`) — recorded as `None`, never guessed.

```yaml
uv_vis:
  plate: {rows: 8, columns: 12, name: "GREINER 96 F-BOTTOM"}
```
```python
from sdlsetup.nodes.uv_vis import find_plate, search_plates, clearance_report, PlateSpec

d = find_plate("GREINER 96 F-BOTTOM")
plate = PlateSpec.from_library("GREINER 96 F-BOTTOM")   # carries d in .definition
tall_96s = search_plates(well_count=96, min_height_mm=14.0)
clearance_report(d)   # facts only, for an arm-integration layer to size clearance
```

## Incubation and shaking (`config.py` + `reader.py`)

`Incubation` validates the 25.0–45.0 °C target range; `Shaking` validates the
per-mode rpm/time caps (orbital/double-orbital 100–700, up to 1100 with
`high_speed=True`; linear up to 800 with `high_speed`; meander 100–300 always).
`SpectroStarNano.shake()` is **UNVERIFIED-ON-DDE** — only the script-language
`R_Shake` form is documented; the DDE `Shake` argument shape is implemented by
analogy.

```yaml
uv_vis:
  incubation: {target_c: 37.0}
  shaking: {mode: orbital, frequency_rpm: 400, time_s: 30}
```
```python
reader.set_temperature(37.0)          # validated via Incubation, ACTUATES
reader.monitor_temperature()          # Temp 0.1, sensor readback only
reader.temperatures()                 # (23.8, 23.8, 24.1) parsed from the run log
reader.shake(mode="orbital", frequency_rpm=400, time_s=30)   # ACTUATES
```

## Well layouts (`layout.py`)

`LayoutPlan` builds `.lb` layout text/files for `ImportLayout`/`EditLayout`
(fact 2/3) — `ImportLayout` **requires** the protocol-definition directory as
its middle argument (`config.protocol_db_path`), not just the protocol name
and the layout path.

```yaml
uv_vis:
  protocol_db_path: 'C:\Program Files (x86)\BMG\SPECTROstar Nano\User\Definit'
  layout_dir: /home/lamp/SDLsetup/dataset/uv_vis_runs/layouts
```
```python
from sdlsetup.nodes.uv_vis import LayoutPlan, measure_wells

plan = LayoutPlan.grid(rows=5, columns=3, replicates_along="column")  # 5 samples x 3 reps
reader.apply_layout("Protein", plan)                 # writes + ImportLayout, ACTUATES
result = measure_wells(config, plan, protocol="Protein")   # apply -> run -> analyse
```
