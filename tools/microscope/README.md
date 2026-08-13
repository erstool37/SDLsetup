# microscope — Leica K3C + TIS DFK 33UX264

Everything needed to operate the microscope: both cameras, the live stream, and
the optics measurements (focus, marker detection). Packaged as the
`microscope-photo` CLI, and as `Microscope` for orchestration.

```python
from sdlsetup.devices import microscope

scope = microscope.Microscope.from_config("config.yaml")
frame = scope.grab_frame(Path("shot.jpg"))     # next published live frame
sample = scope.measure_focus(frame.require())  # sharpness + quality flags
```

| Module | Holds |
|---|---|
| `api.py` | **main** — `Microscope`, `Frame`, and the module-level shorthands |
| `capture.py` | camera classes, the capture batch, and the capture CLI |
| `backends/` | per-vendor programs (GenTL, TWAIN, tisgrabber) + their `.ps1` bridges |
| `viewer.py` | the live monitor web UI on `:8766` |
| `focus.py` | sharpness measurement and the lit-region mask |
| `marker.py` | locate the alignment mark (dark or red), with quality flags |
| `imaging.py` | array primitives: grayscale, Otsu, components, sharpness |
| `batch.py` | per-camera capture modules |
| `node.py` | dashboard adapter — thin, delegates to `api` |
| `cameras.json` | hardware inventory |

The bridges in `backends/` resolve their helper `.py` files with `$PSScriptRoot`,
so each `.ps1` and its `_capture_file.py` must stay in that directory together.

## Two ways to get a frame, and the difference matters

`capture()` drives the cameras directly: full control, full resolution, several
seconds, and it needs exclusive device access.

`grab_frame()` copies a frame the live session already published to
`/tmp/sdl_microscope_stream`. It configures nothing but does not contend for the
camera, which is what makes it the right source during a closed loop.

Two measured facts about the stream, both load-bearing: the file mtime is
**publish** time, not exposure time (publishing runs ~3.0 s apart while a capture
takes ~2 s), so taking the **second** frame after a move is what guarantees it
shows the pose the arm is in now; and a frame appears before it is finished being
written, so it is copied only once its size and mtime have settled and the JPEG
end-of-image marker is present.

## Hardware paths

The current camera paths are:

- Leica `K3C`, serial `700011170655`, through the installed Leica USB3 Vision GenTL producer on Windows.
- The Imaging Source `DFK 33UX264`, serial `48424295`, through the legacy publication-era `tisgrabber_x64.dll` sample stack.
- USBIPD bus `1-20`, VID:PID `199e:9089`.
- Output files are written to `/home/lamp/SDLsetup/dataset/captures`.

The Leica `K3C` is detected by Windows as bus `1-19`, VID:PID `1711:1460`, and can be USB-shared to WSL. It is not exposed as a standard V4L camera. The configured capture path uses `leica_gentl_capture.ps1`, which opens `C:\Windows\twain_64\Leica Microsystems\bin64\bgapi2_usb.cti` from Windows Python. LAS X is not required for this path and can block the camera if it is holding K3C exclusively.

The TIS path uses the original code structure (`device.xml`, `IC_StartLive`, `IC_SnapImage`, `IC_SaveImage`) and applies the current manual `exp2` profile inside the grabber session before snapping. On 2026-06-22 the TIS software paths could save files, but saved TIS frames were visually noise-dominated; verify the physical TIS optical path before treating TIS as complete.

A LAS X UI automation bridge is available at `leica_lasx_ui.ps1`. It can press the visible LAS X acquisition controls from script, but LAS X still controls where captured data is stored. Configure LAS X Project/Data Exporter settings first if you need files written to disk automatically.

## Commands

Package-style entry points:

```bash
microscope-photo --help
python3 -m sdlsetup.devices.microscope --help
```

Optional editable install for a persistent shell command:

```bash
cd /home/lamp/SDLsetup
python3 -m pip install -e .
microscope-photo --help
```

List configured cameras and detected TIS devices:

```bash
./microscope-photo list
```

Show camera, USBIPD, and WSL USB diagnostics:

```bash
./microscope-photo status
```

Capture one image from the TIS camera:

```bash
./microscope-photo capture --camera tis_dfk33ux264
```

Capture one image from the Leica K3C through Windows GenTL:

```bash
./microscope-photo capture --camera leica_k3c
```

List Leica GenTL sources without capturing:

```bash
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w src/sdlsetup/devices/microscope/backends/leica_gentl_capture.ps1)" -Action list
```

Start a live browser stream for the TIS camera:

```bash
./microscope-photo stream --camera tis_dfk33ux264
```

Open this in Windows:

```text
http://localhost:8765/
```

Single-frame health check:

```text
http://localhost:8765/snapshot.jpg
```

Check Leica LAS X acquisition UI state:

```bash
./microscope-photo lasx status
```

Trigger the LAS X `Capture Image` control and then stop it:

```bash
./microscope-photo lasx capture --wait-seconds 3
```

Trigger the LAS X main `Acquire` control and then stop it:

```bash
./microscope-photo lasx acquire --wait-seconds 3
```

Force LAS X acquisition toggles off:

```bash
./microscope-photo lasx stop
```

Capture all enabled cameras as close together as software scheduling allows:

```bash
./microscope-photo capture --camera all
```

Use an explicit batch ID so every camera in a batch shares the same filename prefix and metadata value:

```bash
./microscope-photo capture --camera all --batch-id test_001
```

Read or set TIS properties:

```bash
./microscope-photo props tis_dfk33ux264 ExposureAuto ExposureTime GainAuto Gain
./microscope-photo set-prop tis_dfk33ux264 ExposureAuto=Off ExposureTime=1000
```

## Timing

`capture --camera all` starts one worker per enabled camera, waits for every worker at a start barrier, and releases them together under one `batch_id`. This is near-simultaneous software triggering, not hardware-synchronized exposure. True simultaneous exposure requires a shared hardware trigger line or verified vendor trigger APIs for both cameras.

## Live Streaming

The `stream` command serves a browser MJPEG stream using repeated JPEG captures from the camera. It uses only Python standard-library modules and the installed TIS `ic4-ctrl.exe`, so it does not require OpenCV or a Linux `/dev/video*` node.

This is a software preview stream, not a low-latency hardware video pipeline. On this machine, a single JPEG frame was verified at `2448 x 2048`.

## WSL Access

Both camera USB devices are shared through `usbipd-win`, which allows WSL attachment when needed:

```powershell
usbipd list
usbipd attach --wsl --busid 1-19
usbipd attach --wsl --busid 1-20
```

The TIS workflow does not require attaching the device into Linux. The WSL Python script calls Windows Python and the legacy TIS DLL, which can access the Windows camera driver and write output back into `/home/lamp/SDLsetup/dataset/captures`.

The Leica K3C can be attached into WSL as a USB device, but current checks showed no `/dev/video*` camera node. Use the Windows GenTL bridge for file-producing K3C captures.

## Leica Adapter

Three Leica control paths are present:

- `leica_gentl_capture.ps1` captures K3C directly through Leica's installed GenTL producer. This is the verified file-producing path.
- `leica_twain_capture.ps1` captures an image file through the Windows Leica TWAIN driver. It looks for `LEICA_TWAIN_PYTHON` first, then local conda paths with `pytwain` installed. The default config writes archived files to `/home/lamp/SDLsetup/dataset/captures` and updates `/home/lamp/SDLsetup/dataset/captures/vertical_online_image.jpg`.
- `leica_lasx_ui.ps1` toggles visible LAS X acquisition controls. LAS X still controls project/export storage for this path.

The tested LAS X UI bridge can toggle these enabled LAS X controls:

- `CheckBoxSingleImageMode`
- `StateInfoCaptureImage`
- `ToggleButtonAcquire`

Current testing confirmed that the controls can be toggled from script and returned to `Off`. Earlier LAS X UI automation did not produce an exported image file in `Desktop`, `Documents`, or `Pictures`, and the LAS X command logs did not record a new exported acquisition. Use the GenTL bridge for file-producing Leica captures unless LAS X project/export storage has been configured and verified.

The external-command placeholders `{output}`, `{output_win}`, `{output_dir}`, `{output_dir_win}`, `{tool_dir}`, `{tool_dir_win}`, `{timestamp}`, `{batch_id}`, and `{camera}` are expanded by the Python CLI.

---

# Command reference

_Folded in from the retired `docs/microscope-photo.md`._

The microscope photo layer lives in `src/sdlsetup/devices/microscope` and installs
the `microscope-photo` command into the repo's pyenv `main` virtualenv.

## Environment

```bash
cd /home/lamp/SDLsetup
pyenv activate main
python -m pip install -e ".[dev]"
```

For noninteractive shells, use:

```bash
~/.pyenv/bin/pyenv exec microscope-photo --help
```

## Root Runtime Config

The outermost runtime control file is `/home/lamp/SDLsetup/config.yaml`. The
packaged `src/sdlsetup/devices/microscope/cameras.json` file remains the camera
hardware inventory; `config.yaml` controls command defaults such as capture
camera selection, output directory, schedule/autoshoot interval, live-session
capture interval, monitor refresh rate, and ports. CLI flags still override the
root config for one-off runs.

The fastest localhost display path is:

```bash
microscope-photo live-session start
```

Its frame files are overwritten in `/tmp/sdl_microscope_stream`, while archive
copies go to `/home/lamp/camera_captures` on the configured save interval. The
monitor's `Take Photo` button copies the latest displayed live frame files into
`/home/lamp/camera_captures/user-saved`, controlled by `user_saved_dir` in
`config.yaml`; it does not trigger a new hardware exposure. Edit
`live_session.stream_interval_s` in `config.yaml` to change the target capture
start interval; actual frame rate is still limited by the camera drivers and
current exposure settings.

## Capture

Normal lab captures write to `/home/lamp/camera_captures`:

```bash
microscope-photo capture --camera leica_k3c
microscope-photo capture --camera tis_dfk33ux264
microscope-photo capture --camera all
```

Programmatic capture from Python:

```python
from pathlib import Path

from sdlsetup.devices.microscope import (
    LeicaK3CModule,
    TisDFK33Module,
    capture_both,
)

leica = LeicaK3CModule(output_dir=Path("/home/lamp/camera_captures"))
tis = TisDFK33Module(output_dir=Path("/home/lamp/camera_captures"))

leica_result = leica.capture(batch_id="manual_leica")
tis_result = tis.capture(batch_id="manual_tis")
both_results = capture_both(batch_id="manual_both")
```

The default TIS camera path intentionally uses the previous lab stack from the
publication-era code: `tisgrabber_x64.dll`, `device.xml`, `IC_StartLive`,
`IC_SnapImage`, and `IC_SaveImage`. The package applies the current manual
`exp2` profile inside the legacy grabber session (`Exposure=2.0 s`,
`Gain=48.0`, `Brightness=240`) and records the legacy `Exposure/Gain` values in
the per-capture metadata. After capture it restores IC4 auto exposure/gain.

On 2026-06-22 the TIS camera was reachable through all tested software paths
(`tisgrabber_x64.dll`, `ic4-ctrl`, and OpenCV/DirectShow), but every saved TIS
frame was visually noise-dominated rather than a sample image. This means the
software can open and save from the device, but the physical TIS optical path,
illumination, lens cap, or selected live source still needs to be checked before
calling TIS capture verified.

The Leica K3C path uses the installed USB3 Vision GenTL producer directly:
`C:\Windows\twain_64\Leica Microsystems\bin64\bgapi2_usb.cti`. This avoids the
fragile TWAIN UI path and does not require LAS X to be running. LAS X/LCS can
hold the K3C exclusively, so close LAS X or stop the hidden `LCS.exe -Embedding`
process before automated GenTL capture if the camera reports access denied.

The current Leica microscope default is `ExposureUs=60000`, `Gain=1.0`, JPEG
quality `95`. On 2026-06-22 this produced a clear crystal image with mean
brightness about `52` and saturation about `0.15%`.

Verification captures should go under `test/images`:

```bash
microscope-photo capture --camera all --output-dir test/images --batch-id sample_manual
```

Captured files are checked for image signal after they are written. A file that
exists but is effectively black is reported as a failed capture, and the
per-image metadata JSON includes `image_signal.mean`, `stddev`, `min`, and `max`.
The default mean-brightness threshold is `40.0`; adjust a camera's
`signal_min_mean` in `cameras.json` if a deliberately dark assay needs a lower
threshold. For TIS, mean brightness alone is not sufficient proof because a
noise-dominated frame can pass the brightness threshold; visually inspect the
latest saved file or the monitor before treating it as a correct capture.

## Legacy Path Diagnostics

The original publication-era Leica data on this machine was LAS X XLEF export
with `TIF + LOF` files under the staged LAS X data folders. The original TIS
horizontal capture used `tisgrabber_x64.dll` with `device.xml`.

Use this read-only command to compare the current `lamp` settings with those
legacy paths:

```bash
microscope-photo diagnose
```

The most important Leica check is whether the current LAS X user data includes
`Type: TIF` as well as `Type: LOF` in the XLEF export configuration. The current
camera profile should also show K3C serial `700011170655`, exposure `0.027406`,
gain `1`, and 1536x1024 binning.

## Scheduled Capture

```bash
microscope-photo schedule --camera all --count 2 --interval-s 1 --output-dir test/images --schedule-id sample_schedule
```

The schedule command writes one manifest JSON file plus the captured images and
per-image metadata JSON files.

## Fast Live Session

Use this for the normal microscopy monitoring session:

```bash
microscope-photo live-session start
```

The live session replaces the older pattern of running `monitor` and
`autoshoot` separately. Defaults:

```text
Live monitor:        http://127.0.0.1:8766/
Temporary image dir: /tmp/sdl_microscope_stream
Archive image dir:   /home/lamp/camera_captures
User-saved dir:      /home/lamp/camera_captures/user-saved
Monitor refresh:     500 ms
Live capture target: `config.yaml` `live_session.stream_interval_s` (currently 3.0 s), limited by camera capture time
Archive save cycle:  31 s
```

The browser monitor reads only the temporary image directory. Live frames use
fixed filenames such as `current_leica_k3c.jpg` and
`current_tis_dfk33ux264.jpg`, so they are overwritten instead of accumulating.
The capture loop stages each frame under `.staging` and atomically replaces the
visible `current_*` file after capture finishes, which avoids serving partially
written files.

The archive cycle does not trigger extra hardware captures. It periodically
copies the latest temporary live frame into `/home/lamp/camera_captures` with a
timestamped `live_save_*` filename. The `Take Photo` button also avoids extra
hardware captures; it copies the current displayed Leica/TIS files to the
configured `user_saved_dir` with timestamped filenames.

Useful controls:

```bash
microscope-photo live-session status
microscope-photo live-session stop
microscope-photo live-session start --save-on-start
microscope-photo live-session start --stream-interval-s 0.5 --save-interval-s 31 --refresh-ms 500
```

The live session is a fast still-image preview loop, not a true hardware video
stream. Actual frame rate is limited by the camera drivers and current exposure
settings. The TIS preview path temporarily uses `settle_s=0` and disables image
signal analysis for speed; use explicit `capture` or `schedule` for data that
needs normal validation.

## Local Monitor

```bash
microscope-photo monitor --output-dir /home/lamp/camera_captures --host 127.0.0.1 --port 8765
```

Open `http://127.0.0.1:8765/` on Windows or WSL. The page displays the latest
Leica and TIS capture files, includes a `Take Photo` button that copies the
current displayed files to `monitor.user_saved_dir`, and includes a reserved
robot-status panel for future telemetry.

For lab internal-network viewing, bind to all WSL interfaces:

```bash
microscope-photo monitor --output-dir /home/lamp/camera_captures --lan --port 8765
```

To print candidate URLs without starting the server:

```bash
microscope-photo lan-info --lan --port 8765
```

The command prints local, WSL, and Windows LAN URL candidates. On WSL2, other
machines on the lab LAN may need Windows firewall/portproxy or WSL mirrored
networking before they can reach the WSL-bound server.

Current LAN proxy setup on 2026-06-22:

```text
Windows listen: 0.0.0.0:8766
WSL target:     172.20.163.26:8766
Firewall rule:  SDL Microscope Monitor 8766, TCP 8766, Domain/Private
```

If Windows itself can open `http://127.0.0.1:8766/` or the WSL URL but another
LAN machine cannot open the Windows LAN URL, refresh the Windows portproxy from
an administrator shell:

```powershell
$Port = 8766
$WslIp = "172.20.163.26"
netsh interface portproxy delete v4tov4 listenaddress=0.0.0.0 listenport=$Port
netsh interface portproxy add v4tov4 listenaddress=0.0.0.0 listenport=$Port connectaddress=$WslIp connectport=$Port
New-NetFirewallRule -DisplayName "SDL Microscope Monitor 8766" -Direction Inbound -Action Allow -Protocol TCP -LocalPort $Port -Profile Domain,Private
```

If the firewall rule already exists, use:

```powershell
Set-NetFirewallRule -DisplayName "SDL Microscope Monitor 8766" -Enabled True -Direction Inbound -Action Allow -Profile Domain,Private
```

## Parameter Inspection

TIS camera properties are available through the installed `ic4-ctrl.exe` path:

```bash
microscope-photo props tis_dfk33ux264 ExposureAuto ExposureTime GainAuto Gain
```

Leica K3C/TWAIN read-only source and capability inspection:

```bash
microscope-photo twain sources
microscope-photo twain capabilities
```

The Leica TWAIN bridge currently captures image files and reports TWAIN
capabilities. Do not change Leica/LAS X acquisition settings persistently unless
the target setting and expected effect are explicitly requested.

Leica K3C/GenTL read-only source inspection:

```bash
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w src/sdlsetup/devices/microscope/backends/leica_gentl_capture.ps1)" -Action list
```

The GenTL helper requires a Windows Python with:

```powershell
python -m pip install harvesters genicam numpy pillow
```
