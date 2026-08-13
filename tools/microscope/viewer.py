"""Browser monitor for latest microscope capture files."""

# ruff: noqa: E501

from __future__ import annotations

import html
import http.server
import json
import mimetypes
import shutil
import socket
import ssl
import subprocess
import threading
import time
from pathlib import Path
from urllib.parse import urlparse

CAMERA_PATTERNS = {
    "leica": ("*_leica_k3c.jpg", "*_leica_k3c.jpeg", "*_leica_k3c.png"),
    "tis": ("*_tis_dfk33ux264.png", "*_tis_dfk33ux264.jpg", "*_tis_dfk33ux264.jpeg"),
}
DEFAULT_CERT_DIR = Path.home() / ".sdl_lab" / "microscope_monitor" / "certs"


class RecordingState:
    """Thread-shared toggle for live-session auto-saving (recording).

    The capture loop and the HTTP server run in the same process on separate
    threads. Live streaming is always on; this only gates the periodic archive
    (auto-save) so a browser button can start/stop recording without ever
    interrupting the stream.
    """

    def __init__(
        self,
        *,
        save_interval_s: float,
        save_prefix: str = "live_save",
        archive_dir: str | None = None,
        enabled: bool = False,
        can_record: bool = True,
    ) -> None:
        self._event = threading.Event()
        self.lock = threading.Lock()
        self.save_interval_s = save_interval_s
        self.save_prefix = save_prefix
        self.archive_dir = archive_dir
        self.can_record = can_record
        self.generation = 0  # bumped on every toggle so the loop can re-sync its schedule
        self.saved_batches = 0
        self.saved_files = 0
        self.last_saved_at: float | None = None
        if enabled and can_record:
            self._event.set()

    def is_recording(self) -> bool:
        return self._event.is_set()

    def start(self) -> bool:
        if not self.can_record:
            return False
        with self.lock:
            if not self._event.is_set():
                self._event.set()
                self.generation += 1
        return True

    def stop(self) -> None:
        with self.lock:
            if self._event.is_set():
                self._event.clear()
                self.generation += 1

    def note_saved(self, archived: list) -> None:
        with self.lock:
            self.saved_batches += 1
            self.saved_files += len(archived or [])
            self.last_saved_at = time.time()

    def status(self) -> dict[str, object]:
        with self.lock:
            return {
                "recording": self._event.is_set(),
                "can_record": self.can_record,
                "save_interval_s": self.save_interval_s,
                "archive_dir": self.archive_dir,
                "saved_batches": self.saved_batches,
                "saved_files": self.saved_files,
                "last_saved_at": self.last_saved_at,
            }

PAGE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Microscope Monitor</title>
<style>
  :root{
    --bg:#0a0e13; --bg2:#0d141c;
    --surface:#121a23; --surface-2:#0e151d; --line:#202c38; --line-2:#2a3947;
    --text:#e8eef4; --muted:#8a9aab; --faint:#5e6e7d;
    --accent:#49c5e6; --ok:#57d08a; --warn:#f0b54a; --rec:#ff5f5f;
    --radius:14px; --radius-sm:10px; --shadow:0 6px 24px rgba(0,0,0,.35);
  }
  *{box-sizing:border-box}
  html,body{height:100%}
  body{
    margin:0; color:var(--text);
    font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
    background:
      radial-gradient(1100px 560px at 82% -12%, rgba(73,197,230,.07), transparent 60%),
      linear-gradient(180deg,var(--bg2),var(--bg));
    min-height:100vh; -webkit-font-smoothing:antialiased;
  }
  .topbar{
    position:sticky; top:0; z-index:10;
    display:flex; align-items:center; justify-content:space-between;
    padding:13px 22px; border-bottom:1px solid var(--line);
    background:rgba(10,14,19,.72); backdrop-filter:blur(10px);
  }
  .brand{display:flex; align-items:center; gap:11px}
  .brand h1{margin:0; font-size:15.5px; font-weight:650; letter-spacing:.2px}
  .live-dot{width:9px;height:9px;border-radius:50%;background:var(--ok);animation:pulse 2s infinite}
  @keyframes pulse{0%{box-shadow:0 0 0 0 rgba(87,208,138,.5)}70%{box-shadow:0 0 0 7px rgba(87,208,138,0)}100%{box-shadow:0 0 0 0 rgba(87,208,138,0)}}
  .rec-flag{display:none;align-items:center;gap:7px;font-size:11px;font-weight:800;letter-spacing:1.2px;color:var(--rec);
    border:1px solid var(--rec);border-radius:20px;padding:2px 10px}
  .rec-flag.show{display:inline-flex}
  .rec-flag::before{content:"";width:7px;height:7px;border-radius:50%;background:var(--rec);animation:blink 1.2s infinite}
  @keyframes blink{0%,100%{opacity:1}50%{opacity:.25}}
  .updated{color:var(--muted);font-size:12.5px;white-space:nowrap;font-variant-numeric:tabular-nums}
  .layout{
    display:grid; grid-template-columns:minmax(0,1fr) 312px; gap:18px;
    padding:18px 22px; max-width:1480px; margin:0 auto;
  }
  .cams{display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:16px; align-content:start}
  .cam-card{background:linear-gradient(180deg,var(--surface),var(--surface-2));
    border:1px solid var(--line); border-radius:var(--radius); overflow:hidden; box-shadow:var(--shadow)}
  .cam-head{display:flex; align-items:center; justify-content:space-between; gap:10px; padding:11px 14px; border-bottom:1px solid var(--line)}
  .cam-title{display:flex; align-items:center; gap:9px; font-weight:650; font-size:13.5px}
  .cam-meta{color:var(--muted); font-size:12px; font-variant-numeric:tabular-nums; overflow:hidden; text-overflow:ellipsis; white-space:nowrap}
  .dot{width:8px;height:8px;border-radius:50%;background:var(--faint);flex:none}
  .dot.ok{background:var(--ok);box-shadow:0 0 8px rgba(87,208,138,.7)}
  .dot.warn{background:var(--warn)} .dot.off{background:#46525f}
  .cam-view{
    height:clamp(200px,40vh,430px);
    background:repeating-conic-gradient(#0b1116 0% 25%, #0c1318 0% 50%) 0/22px 22px;
    display:flex; align-items:center; justify-content:center; padding:8px;
  }
  .cam-view img{max-width:100%; max-height:100%; object-fit:contain; border-radius:6px; display:block; transition:opacity .2s}
  .cam-view img.noimg{opacity:0}
  .side{display:flex; flex-direction:column; gap:16px; min-width:0}
  .panel{background:linear-gradient(180deg,var(--surface),var(--surface-2)); border:1px solid var(--line);
    border-radius:var(--radius); box-shadow:var(--shadow); overflow:hidden}
  .panel-head{display:flex; align-items:center; justify-content:space-between; padding:12px 14px; border-bottom:1px solid var(--line)}
  .panel-head h2{margin:0; font-size:12px; font-weight:700; letter-spacing:.5px; text-transform:uppercase; color:var(--muted)}
  .tag{font-size:11px; font-weight:700; color:var(--ok); border:1px solid var(--line-2); border-radius:20px; padding:2px 9px}
  .tag.muted{color:var(--faint)}
  .panel-body{padding:14px}
  .btn-row{display:flex; gap:10px; margin-bottom:12px}
  .btn{height:42px; border-radius:var(--radius-sm); border:1px solid var(--line-2); background:#16212c; color:var(--text);
    font-weight:650; font-size:13.5px; cursor:pointer; display:inline-flex; align-items:center; justify-content:center; gap:8px;
    transition:border-color .15s,background .15s}
  .btn:hover{border-color:var(--accent); background:#1a2836}
  .btn:disabled{opacity:.55; cursor:wait}
  .btn-rec{flex:1}
  .btn-photo{flex:0 0 auto; padding:0 16px; background:transparent}
  .btn-rec .rdot{width:11px;height:11px;border-radius:50%;background:var(--rec);box-shadow:0 0 8px rgba(255,95,95,.5)}
  .btn-rec.on{background:rgba(255,95,95,.15); border-color:var(--rec); color:#ffdada}
  .btn-rec.on .rdot{animation:blink 1.1s infinite}
  .status-line{color:var(--muted); font-size:12.5px; margin-bottom:13px; min-height:1em}
  .kv-grid{display:grid; gap:9px}
  .kv{display:grid; grid-template-columns:80px minmax(0,1fr); gap:10px; align-items:baseline; font-size:12.5px}
  .kv span{color:var(--faint)}
  .kv b{font-weight:600; color:var(--text); overflow-wrap:anywhere}
  .kv b.path{font-family:ui-monospace,Menlo,Consolas,monospace; font-size:11.5px; color:var(--muted)}
  @media (max-width:1040px){
    .layout{grid-template-columns:1fr}
    .cams{grid-template-columns:1fr}
    .cam-view{height:clamp(180px,36vh,340px)}
  }
</style>
</head>
<body>
  <header class="topbar">
    <div class="brand">
      <span class="live-dot"></span>
      <h1>Microscope&nbsp;Monitor</h1>
      <span class="rec-flag" id="rec-flag">REC</span>
    </div>
    <div class="updated" id="updated">connecting…</div>
  </header>
  <main class="layout">
    <section class="cams">
      <article class="cam-card">
        <div class="cam-head">
          <div class="cam-title"><span class="dot" id="leica-dot"></span> Leica K3C</div>
          <div class="cam-meta" id="leica-meta">—</div>
        </div>
        <div class="cam-view"><img id="leica-image" alt="Leica K3C live view"></div>
      </article>
      <article class="cam-card">
        <div class="cam-head">
          <div class="cam-title"><span class="dot" id="tis-dot"></span> TIS DFK 33UX264</div>
          <div class="cam-meta" id="tis-meta">—</div>
        </div>
        <div class="cam-view"><img id="tis-image" alt="TIS camera live view"></div>
      </article>
    </section>
    <aside class="side">
      <section class="panel">
        <div class="panel-head"><h2>Capture</h2><span class="tag" id="cap-tag">ready</span></div>
        <div class="panel-body">
          <div class="btn-row">
            <button class="btn btn-rec" id="record-button" type="button"><span class="rdot"></span><span id="record-label">Start Record</span></button>
            <button class="btn btn-photo" id="save-button" type="button">Take Photo</button>
          </div>
          <div class="status-line" id="record-status">stopped</div>
          <div class="kv-grid">
            <div class="kv"><span>Save</span><b id="save-status">idle</b></div>
            <div class="kv"><span>Recorded</span><b id="record-count">0 / 0</b></div>
            <div class="kv"><span>Root</span><b id="root" class="path">—</b></div>
            <div class="kv"><span>Saved to</span><b id="save-dir" class="path">__SAVE_DIR__</b></div>
          </div>
        </div>
      </section>
      <section class="panel">
        <div class="panel-head"><h2>Robot</h2><span class="tag muted">reserved</span></div>
        <div class="panel-body">
          <div class="kv-grid">
            <div class="kv"><span>State</span><b id="robot-state">—</b></div>
            <div class="kv"><span>Source</span><b id="robot-source">—</b></div>
            <div class="kv"><span>Message</span><b id="robot-message">—</b></div>
          </div>
        </div>
      </section>
    </aside>
  </main>
  <script>
    const refreshMs = __REFRESH_MS__;
    function $(id){return document.getElementById(id);}
    function setText(id,v){const n=$(id); if(n) n.textContent=v;}
    function fmtBytes(b){if(b==null)return ''; const u=['B','KB','MB','GB']; let i=0,x=b;
      while(x>=1024&&i<u.length-1){x/=1024;i++;} return (x<10&&i>0?x.toFixed(1):Math.round(x))+' '+u[i];}
    function cameraLine(c){if(!c||!c.path)return 'no signal'; return `${c.name} · ${c.age_s.toFixed(1)}s · ${fmtBytes(c.size_bytes)}`;}
    function camDot(id,c){const d=$(id); if(!d)return; const fresh=c&&c.path&&c.age_s!=null&&c.age_s<15;
      d.className='dot '+(fresh?'ok':(c&&c.path?'warn':'off'));}

    let recording=false;
    function applyRecord(s){if(!s)return; recording=!!s.recording;
      const btn=$('record-button');
      if(btn){btn.classList.toggle('on',recording); btn.disabled=s.can_record===false;}
      setText('record-label', recording?'Stop Record':'Start Record');
      $('rec-flag').classList.toggle('show',recording);
      if(s.can_record===false) setText('record-status','archiving disabled');
      else setText('record-status', recording?('recording every '+s.save_interval_s+'s'):'stopped');
      setText('record-count', (s.saved_batches||0)+' batches · '+(s.saved_files||0)+' files');
    }
    async function toggleRecord(){const btn=$('record-button'); if(btn)btn.disabled=true;
      try{const ep=recording?'/api/record/stop':'/api/record/start';
        const r=await fetch(ep,{method:'POST'}); applyRecord(await r.json());}
      catch(e){setText('record-status','record failed');}
      finally{const b=$('record-button'); if(b)b.disabled=false;}
    }
    async function saveLatest(){const b=$('save-button'); if(b)b.disabled=true; setText('save-status','saving…');
      try{const r=await fetch('/api/save-latest',{method:'POST'}); const res=await r.json();
        if(!r.ok) throw new Error(res.error||r.statusText);
        const sc=res.saved?res.saved.length:0, mc=res.missing?res.missing.length:0;
        setText('save-status', mc?`saved ${sc}, missing ${mc}`:`saved ${sc}`);
        setText('save-dir', res.save_dir);}
      catch(e){setText('save-status','save failed');}
      finally{if(b)b.disabled=false;}
    }
    async function tick(){const now=Date.now();
      $('leica-image').src='/image/leica?t='+now; $('tis-image').src='/image/tis?t='+now;
      try{const s=await fetch('/api/status?t='+now).then(r=>r.json());
        setText('updated','updated '+new Date(s.generated_at*1000).toLocaleTimeString());
        setText('leica-meta',cameraLine(s.cameras.leica)); setText('tis-meta',cameraLine(s.cameras.tis));
        camDot('leica-dot',s.cameras.leica); camDot('tis-dot',s.cameras.tis);
        setText('root',s.root);
        setText('robot-state',s.robot.state); setText('robot-source',s.robot.source); setText('robot-message',s.robot.message);
        if(s.recording) applyRecord(s.recording);
      }catch(e){setText('updated','status unavailable');}
    }
    window.addEventListener('load',()=>{
      const sb=$('save-button'); if(sb) sb.addEventListener('click',saveLatest);
      const rb=$('record-button'); if(rb) rb.addEventListener('click',toggleRecord);
      ['leica-image','tis-image'].forEach(id=>{const im=$(id); if(!im)return;
        im.addEventListener('error',()=>im.classList.add('noimg'));
        im.addEventListener('load',()=>im.classList.remove('noimg'));});
      tick(); setInterval(tick,refreshMs);
    });
  </script>
</body>
</html>
"""


def latest_file(root: Path, camera: str) -> Path | None:
    paths: list[Path] = []
    for pattern in CAMERA_PATTERNS[camera]:
        paths.extend(path for path in root.glob(pattern) if path.is_file())
    if camera == "leica":
        online = root / "vertical_online_image.jpg"
        if online.is_file():
            paths.append(online)
    if not paths:
        return None
    return max(paths, key=lambda path: path.stat().st_mtime)


def camera_status(root: Path, camera: str) -> dict[str, object]:
    path = latest_file(root, camera)
    if path is None:
        return {"path": None}
    stat = path.stat()
    return {
        "name": path.name,
        "path": str(path),
        "size_bytes": stat.st_size,
        "mtime": stat.st_mtime,
        "age_s": max(0.0, time.time() - stat.st_mtime),
    }


def _image_metadata_path(path: Path) -> Path:
    return path.with_suffix(path.suffix + ".json")


def _read_image_metadata(path: Path) -> dict[str, object]:
    metadata_path = _image_metadata_path(path)
    if not metadata_path.exists():
        return {}
    try:
        data = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"source_metadata_error": str(exc)}
    return data if isinstance(data, dict) else {"source_metadata": data}


def save_latest_images(
    *,
    root: Path,
    save_dir: Path,
    cameras: tuple[str, ...] = ("leica", "tis"),
    timestamp: str | None = None,
) -> dict[str, object]:
    root = root.expanduser().resolve()
    save_dir = save_dir.expanduser().resolve()
    stamp = timestamp or time.strftime("%Y%m%d_%H%M%S")
    save_dir.mkdir(parents=True, exist_ok=True)

    saved: list[dict[str, object]] = []
    missing: list[dict[str, object]] = []
    for camera in cameras:
        if camera not in CAMERA_PATTERNS:
            missing.append({"camera": camera, "reason": "unknown camera"})
            continue
        source = latest_file(root, camera)
        if source is None:
            missing.append({"camera": camera, "reason": "no latest image"})
            continue

        extension = source.suffix.lower() or ".jpg"
        destination = save_dir / f"{stamp}_{camera}{extension}"
        index = 1
        while destination.exists():
            destination = save_dir / f"{stamp}_{camera}_{index}{extension}"
            index += 1

        shutil.copy2(source, destination)
        metadata = _read_image_metadata(source)
        metadata.update(
            {
                "saved_at": stamp,
                "saved_camera_slot": camera,
                "saved_from": str(source),
                "saved_path": str(destination),
            }
        )
        metadata_path = _image_metadata_path(destination)
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        saved.append(
            {
                "camera": camera,
                "source": str(source),
                "path": str(destination),
                "metadata": str(metadata_path),
            }
        )

    return {
        "saved_at": stamp,
        "save_dir": str(save_dir),
        "saved": saved,
        "missing": missing,
    }


def monitor_status(root: Path) -> dict[str, object]:
    return {
        "generated_at": time.time(),
        "root": str(root),
        "cameras": {
            "leica": camera_status(root, "leica"),
            "tis": camera_status(root, "tis"),
        },
        "robot": {
            "state": "not wired",
            "source": "reserved for future robot telemetry",
            "message": "No robot status adapter is configured in this monitor yet.",
        },
    }


def _ipv4_from_hostname() -> set[str]:
    addresses: set[str] = set()
    try:
        _, _, host_addresses = socket.gethostbyname_ex(socket.gethostname())
        addresses.update(host_addresses)
    except OSError:
        pass
    return addresses


def _ipv4_from_hostname_command() -> set[str]:
    try:
        cp = subprocess.run(
            ["hostname", "-I"],
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=3,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return set()
    if cp.returncode != 0:
        return set()
    return {part.strip() for part in cp.stdout.split() if part.strip()}


def _primary_route_ipv4() -> set[str]:
    addresses: set[str] = set()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        addresses.add(sock.getsockname()[0])
    except OSError:
        pass
    finally:
        sock.close()
    return addresses


def _windows_ipv4_addresses() -> set[str]:
    command = (
        "Get-NetIPAddress -AddressFamily IPv4 | "
        "Where-Object { $_.IPAddress -notlike '127.*' -and "
        "$_.IPAddress -notlike '169.254.*' } | "
        "Select-Object -ExpandProperty IPAddress"
    )
    try:
        cp = subprocess.run(
            ["powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-Command", command],
            text=True,
            encoding="utf-8",
            errors="replace",
            capture_output=True,
            timeout=8,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        return set()
    if cp.returncode != 0:
        return set()
    return {line.strip() for line in cp.stdout.splitlines() if line.strip()}


def _public_ipv4(addresses: set[str]) -> list[str]:
    return sorted(
        address
        for address in addresses
        if address and not address.startswith("127.") and ":" not in address
    )


def monitor_urls(host: str, port: int, *, scheme: str = "http") -> dict[str, object]:
    wsl_addresses = _public_ipv4(
        _ipv4_from_hostname() | _ipv4_from_hostname_command() | _primary_route_ipv4()
    )
    windows_addresses = _public_ipv4(_windows_ipv4_addresses())
    local_urls = [f"{scheme}://127.0.0.1:{port}/", f"{scheme}://localhost:{port}/"]
    if host not in {"0.0.0.0", "", "127.0.0.1", "localhost"}:
        local_urls.insert(0, f"{scheme}://{host}:{port}/")
    return {
        "bind_host": host,
        "port": port,
        "scheme": scheme,
        "local_urls": local_urls,
        "wsl_urls": [f"{scheme}://{address}:{port}/" for address in wsl_addresses],
        "windows_lan_urls": [f"{scheme}://{address}:{port}/" for address in windows_addresses],
        "note": (
            "For same-machine use, open a local URL. For another device on the lab LAN, "
            "try a Windows LAN URL first. WSL2 may require Windows firewall/portproxy "
            "or mirrored networking for LAN clients."
        ),
    }


def _valid_ipv4_addresses(addresses: set[str]) -> list[str]:
    valid: list[str] = []
    for address in sorted(addresses):
        parts = address.split(".")
        if len(parts) != 4:
            continue
        try:
            numbers = [int(part) for part in parts]
        except ValueError:
            continue
        if all(0 <= number <= 255 for number in numbers):
            valid.append(address)
    return valid


def certificate_hosts() -> tuple[list[str], list[str]]:
    ip_addresses = _valid_ipv4_addresses(
        {"127.0.0.1"}
        | _ipv4_from_hostname()
        | _ipv4_from_hostname_command()
        | _primary_route_ipv4()
        | _windows_ipv4_addresses()
    )
    dns_names = ["localhost", socket.gethostname()]
    return ip_addresses, sorted({name for name in dns_names if name})


def _openssl_config(ip_addresses: list[str], dns_names: list[str]) -> str:
    alt_lines: list[str] = []
    for index, name in enumerate(dns_names, start=1):
        alt_lines.append(f"DNS.{index} = {name}")
    for index, address in enumerate(ip_addresses, start=1):
        alt_lines.append(f"IP.{index} = {address}")
    alt_names = "\n".join(alt_lines)
    return f"""[req]
default_bits = 2048
prompt = no
default_md = sha256
distinguished_name = dn
x509_extensions = v3_req

[dn]
CN = SDL Microscope Monitor

[v3_req]
subjectAltName = @alt_names
basicConstraints = CA:FALSE
keyUsage = digitalSignature, keyEncipherment
extendedKeyUsage = serverAuth

[alt_names]
{alt_names}
"""


def ensure_self_signed_cert(
    *,
    cert_file: Path | None = None,
    key_file: Path | None = None,
) -> tuple[Path, Path]:
    cert = (cert_file or DEFAULT_CERT_DIR / "monitor.crt").expanduser()
    key = (key_file or DEFAULT_CERT_DIR / "monitor.key").expanduser()
    if cert.exists() and key.exists():
        return cert, key

    cert.parent.mkdir(parents=True, exist_ok=True)
    key.parent.mkdir(parents=True, exist_ok=True)
    ip_addresses, dns_names = certificate_hosts()
    config = cert.parent / "monitor-openssl.cnf"
    config.write_text(_openssl_config(ip_addresses, dns_names), encoding="utf-8")

    cp = subprocess.run(
        [
            "openssl",
            "req",
            "-x509",
            "-nodes",
            "-newkey",
            "rsa:2048",
            "-days",
            "3650",
            "-keyout",
            str(key),
            "-out",
            str(cert),
            "-config",
            str(config),
        ],
        text=True,
        encoding="utf-8",
        errors="replace",
        capture_output=True,
        timeout=30,
        check=False,
    )
    if cp.returncode != 0:
        message = (cp.stderr or cp.stdout).strip()
        raise RuntimeError(f"failed to generate HTTPS certificate: {message}")
    key.chmod(0o600)
    cert.chmod(0o644)
    return cert, key


class MonitorHandler(http.server.BaseHTTPRequestHandler):
    root: Path
    refresh_ms: int
    quiet: bool
    user_saved_dir: Path
    recording_state: RecordingState | None = None

    def log_message(self, fmt: str, *values: object) -> None:
        if not self.quiet:
            super().log_message(fmt, *values)

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in {"/", "/index.html"}:
            self._send_page()
            return
        if path == "/api/status":
            status = monitor_status(self.root)
            if self.recording_state is not None:
                status["recording"] = self.recording_state.status()
            self._send_json(status)
            return
        if path == "/api/record/status":
            if self.recording_state is None:
                self._send_json({"error": "recording not available"}, status=404)
            else:
                self._send_json(self.recording_state.status())
            return
        if path.startswith("/image/"):
            self._send_image(path.removeprefix("/image/"))
            return
        self.send_error(404)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        content_length = int(self.headers.get("Content-Length") or 0)
        if content_length:
            self.rfile.read(content_length)
        if path == "/api/save-latest":
            self._send_save_latest()
            return
        if path in {"/api/record/start", "/api/record/stop"}:
            self._send_record_toggle(start=path.endswith("/start"))
            return
        self.send_error(404)

    def _send_record_toggle(self, *, start: bool) -> None:
        if self.recording_state is None:
            self._send_json({"error": "recording not available"}, status=404)
            return
        if start:
            ok = self.recording_state.start()
            self._send_json(self.recording_state.status(), status=200 if ok else 409)
            return
        self.recording_state.stop()
        self._send_json(self.recording_state.status())

    def _send_save_latest(self) -> None:
        try:
            result = save_latest_images(root=self.root, save_dir=self.user_saved_dir)
        except OSError as exc:
            self._send_json({"error": str(exc)}, status=500)
            return
        self._send_json(result)

    def _send_page(self) -> None:
        body = (
            PAGE
            .replace("__REFRESH_MS__", str(self.refresh_ms))
            .replace("__SAVE_DIR__", html.escape(str(self.user_saved_dir)))
        ).encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, data: dict[str, object], *, status: int = 200) -> None:
        body = json.dumps(data, indent=2).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _send_image(self, camera: str) -> None:
        if camera not in CAMERA_PATTERNS:
            self.send_error(404)
            return
        path = latest_file(self.root, camera)
        if path is None:
            self.send_error(404, f"No latest image for {html.escape(camera)}")
            return
        body = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", mimetypes.guess_type(path.name)[0] or "image/jpeg")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)


def serve_monitor(
    *,
    root: Path,
    host: str,
    port: int,
    refresh_ms: int,
    quiet: bool,
    https: bool = False,
    cert_file: Path | None = None,
    key_file: Path | None = None,
    user_saved_dir: Path | None = None,
    recording_state: RecordingState | None = None,
) -> None:
    image_root = root.expanduser().resolve()
    save_root = (user_saved_dir or image_root / "user-saved").expanduser().resolve()
    handler = type(
        "ConfiguredMonitorHandler",
        (MonitorHandler,),
        {
            "root": image_root,
            "refresh_ms": refresh_ms,
            "quiet": quiet,
            "user_saved_dir": save_root,
            "recording_state": recording_state,
        },
    )
    server = http.server.ThreadingHTTPServer((host, port), handler)
    scheme = "https" if https else "http"
    cert: Path | None = None
    key: Path | None = None
    if https:
        cert, key = ensure_self_signed_cert(cert_file=cert_file, key_file=key_file)
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(certfile=cert, keyfile=key)
        server.socket = context.wrap_socket(server.socket, server_side=True)

    urls = monitor_urls(host, port, scheme=scheme)
    print(f"Serving microscope monitor on {scheme}://{host}:{port}")
    if https:
        print("HTTPS uses a local self-signed certificate; browsers will show a warning.")
        print(f"Certificate: {cert}")
        print(f"Private key: {key}")
    for group in ("local_urls", "wsl_urls", "windows_lan_urls"):
        values = urls[group]
        if values:
            print(f"{group}:")
            for value in values:
                print(f"  {value}")
    print(f"LAN note: {urls['note']}")
    print(f"Image root: {handler.root}")
    print(f"User-saved root: {handler.user_saved_dir}")
    try:
        server.serve_forever()
    finally:
        server.server_close()
