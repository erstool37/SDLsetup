"""Static dashboard page (vanilla JS; data via fetch/SSE)."""

PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SDL Lab</title>
<style>
  :root{color-scheme:dark;--bg:#0b0f14;--panel:#141b23;--panel2:#0f151c;--line:#26313c;
        --text:#e6edf3;--muted:#90a0b0;--accent:#5fb3d4;--ok:#62c98a;--warn:#e0b341;--err:#e3675f;--vacant:#6b7785;}
  *{box-sizing:border-box}
  body{margin:0;min-height:100vh;font-family:Arial,Helvetica,sans-serif;background:var(--bg);color:var(--text)}
  header{height:54px;display:flex;align-items:center;gap:16px;padding:0 18px;border-bottom:1px solid var(--line);background:#0e141b}
  header h1{font-size:16px;margin:0;font-weight:700}
  .tabs{display:flex;gap:6px;margin-left:10px}
  .tab{padding:7px 14px;border:1px solid var(--line);border-bottom:none;background:var(--panel2);color:var(--muted);cursor:pointer;font-weight:700;font-size:13px}
  .tab.active{background:var(--panel);color:var(--text)}
  #clock{margin-left:auto;color:var(--muted);font-size:12px}
  main{padding:14px}
  .view{display:none}.view.active{display:block}
  .cards{display:grid;grid-template-columns:repeat(auto-fill,minmax(240px,1fr));gap:12px}
  .card{border:1px solid var(--line);background:var(--panel);padding:12px}
  .card h3{margin:0 0 8px;font-size:14px;display:flex;align-items:center;gap:8px;justify-content:space-between}
  .badge{font-size:11px;font-weight:700;padding:2px 8px;border-radius:10px;border:1px solid var(--line)}
  .s-online{color:var(--ok);border-color:var(--ok)}.s-offline{color:var(--err);border-color:var(--err)}
  .s-vacant{color:var(--vacant);border-color:var(--vacant)}.s-busy{color:var(--warn);border-color:var(--warn)}
  .s-error{color:var(--err);border-color:var(--err)}.s-init{color:var(--muted)}
  .kv{font-size:12px;color:var(--muted);line-height:1.55;word-break:break-word}
  .kv b{color:var(--text);font-weight:600}
  .term{margin-top:14px;border:1px solid var(--line);background:#05080b}
  .term-head{display:flex;align-items:center;gap:10px;padding:8px 12px;border-bottom:1px solid var(--line);font-size:13px;font-weight:700}
  #current{color:var(--accent);font-weight:700}
  #log{height:34vh;overflow:auto;padding:10px 12px;font-family:ui-monospace,Menlo,Consolas,monospace;font-size:12px;line-height:1.5}
  .ln{white-space:pre-wrap}.ln .t{color:var(--muted)}.ln .src{color:var(--accent)}
  .ln.error .m{color:var(--err)}.ln.warn .m{color:var(--warn)}
  .cam-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(360px,1fr));gap:14px}
  .cam{border:1px solid var(--line);background:var(--panel)}
  .cam .h{display:flex;justify-content:space-between;align-items:center;padding:8px 12px;border-bottom:1px solid var(--line);font-weight:700;font-size:13px}
  .cam .wrap{background:#05080b;height:42vh;display:flex;align-items:center;justify-content:center;overflow:hidden}
  .cam img{width:100%;height:100%;object-fit:contain;display:block}
  .controls{display:flex;gap:10px;align-items:center;flex-wrap:wrap;margin-bottom:14px}
  button{height:34px;padding:0 14px;border:1px solid #3b6f88;background:#173247;color:var(--text);font-weight:700;cursor:pointer}
  button:disabled{opacity:.55;cursor:wait}
  button.rec{border-color:#7a3b3b;background:#3a1d1d}
  .pill{font-size:12px;color:var(--muted)}
  .pill.on{color:var(--err);font-weight:700}
  .placeholder{border:1px dashed #46525f;background:var(--panel2);padding:24px;color:var(--muted);text-align:center}
.motion{border-radius:8px;padding:10px 12px;margin:0 0 12px 0;border:1px solid #2a2f3a;background:#161a22}
.motion.busy{border-color:#7a5c17;background:#221c0e}
.motion.idle{border-color:#24402c;background:#111a14}
.mhead{font-weight:700;letter-spacing:.04em;margin-bottom:6px}
.motion.busy .mhead{color:#ffcc55}
.motion.idle .mhead{color:#6fcf8a}
.mrow{display:flex;align-items:center;gap:8px;padding:2px 0;font-size:13px}
.mrow.off{opacity:.45}
.dot{width:9px;height:9px;border-radius:50%;background:#3a4150;flex:none}
.dot.on{background:#ffcc55;box-shadow:0 0 6px #ffcc55}
.doing{color:#c8cede}
.since{color:#79808f;font-size:12px;margin-left:auto}
.warn{color:#ff8b6a;font-size:12px}
.mnote{margin-top:6px;color:#ff8b6a;font-size:12px}

/* ---- overview: cards left, small bench window right ----------------
   Flex, not grid: the bench pane's width is a JS-driven flex-basis so it
   can be dragged, and localStorage remembers it across reloads.        */
.ov{display:flex;gap:0;align-items:stretch}
.ov-main{flex:1 1 auto;min-width:320px;padding-right:14px}
.ov-resizer{flex:0 0 10px;margin:0 -3px;cursor:col-resize;position:relative;
  touch-action:none}
.ov-resizer::after{content:"";position:absolute;top:0;bottom:0;left:4px;width:2px;
  background:var(--line);transition:background .15s}
.ov-resizer:hover::after,.ov-resizer.dragging::after{background:var(--accent)}
@media (max-width:1100px){.ov{flex-direction:column}
  .ov-main{padding-right:0}
  .ov-resizer{display:none}
  .bench-wrap{flex:1 1 auto !important;max-width:none !important;position:static}}

.bench-wrap{flex:0 0 340px;min-width:260px;max-width:min(70vw,900px);
  position:sticky;top:12px;background:linear-gradient(180deg,#151922,#12151c);
  border:1px solid #232936;border-radius:12px;padding:10px 11px 11px}
.bench-head{display:flex;align-items:baseline;gap:8px;margin-bottom:8px}
.bench-title{font-weight:650;letter-spacing:.10em;text-transform:uppercase;
  font-size:10px;color:#8b94a7}
.bench-dim{margin-left:auto;font-size:10px;color:#5f6675;font-variant-numeric:tabular-nums}

.bench{position:relative;width:100%;border-radius:7px;
  background:
    linear-gradient(90deg,#1a1f29 1px,transparent 1px) 0 0/25% 100%,
    linear-gradient(180deg,#1a1f29 1px,transparent 1px) 0 0/100% 25%,
    #10131a;
  border:1px solid #262d3b;overflow:hidden}
.slot{position:absolute;border-radius:4px;padding:3px 4px;
  border:1px solid #313949;background:#181d27;overflow:hidden;
  display:flex;flex-direction:column;justify-content:center;
  transition:border-color .2s,box-shadow .2s,background .2s}
.slot .nm{font-size:9.5px;font-weight:600;color:#dfe4ee;line-height:1.1;
  white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.slot .sb{font-size:8px;color:#6b7383;line-height:1.1;white-space:nowrap}
.slot .act{font-size:8px;color:#e0a72c;line-height:1.15;margin-top:1px;
  white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.slot .pulse{position:absolute;top:3px;right:3px;width:6px;height:6px;
  border-radius:50%;background:#3a4150}
.slot.on   {border-color:#2f5e43}
.slot.on   .pulse{background:#3f8f5f}
.slot.busy {border-color:#7a5c17;background:#201c10;
  box-shadow:0 0 0 1px rgba(224,167,44,.3),0 0 16px -4px rgba(224,167,44,.55)}
.slot.busy .pulse{background:#e0a72c;animation:beat 1.1s ease-out infinite}
.slot.off  {opacity:.5}
.slot.vacant{border-style:dashed;opacity:.4}
@keyframes beat{0%{box-shadow:0 0 0 0 rgba(224,167,44,.5)}
  70%{box-shadow:0 0 0 7px rgba(224,167,44,0)}100%{box-shadow:0 0 0 0 rgba(224,167,44,0)}}
.slot.arm{border-radius:50%}
.bench-link{position:absolute;border-top:1px dashed #4a3a16;pointer-events:none;opacity:.45}
.bench-link.hot{border-top-color:#e0a72c;opacity:1}
.legend{display:flex;flex-wrap:wrap;gap:4px 10px;margin-top:7px;
  color:#6b7383;font-size:9.5px}
.legend span{display:inline-flex;align-items:center;gap:4px}
.legend i{width:7px;height:7px;border-radius:2px;display:inline-block}
.lg.on{background:#3f8f5f} .lg.busy{background:#e0a72c}
.lg.off{background:#3a4150} .lg.vacant{background:#2a2f3a;border:1px dashed #454c5c}
.bench-note{margin-top:6px;font-size:10px;color:#6b7383;line-height:1.35}
.bench-note.hot{color:#ff8b6a}
@media (prefers-reduced-motion: reduce){.slot.busy .pulse{animation:none}}
/* ---- Chamber Control tab (scoped light theme) ----------------------- */
#kinetics{--cc-ground:#f4f6f8;--cc-surf:#ffffff;--cc-ink:#1b2530;--cc-mut:#5a6b7b;
  --cc-line:#e3e8ee;--cc-accent:#2563a8;--cc-amber:#c96a1e;
  --cc-good:#2e7d52;--cc-warn:#b7791f;--cc-off:#8a97a4;
  background:var(--cc-ground);color:var(--cc-ink);padding:16px;border-radius:12px;
  font-variant-numeric:tabular-nums;-webkit-font-smoothing:antialiased}
#kinetics *{box-sizing:border-box}
#kinetics .cc-head{display:flex;align-items:center;gap:12px;margin-bottom:14px}
#kinetics .cc-title{font-size:17px;font-weight:700;letter-spacing:.01em}
#kinetics .cc-pill{font-size:11px;font-weight:700;padding:3px 10px;border-radius:11px;
  border:1px solid var(--cc-line);color:var(--cc-mut);background:var(--cc-surf)}
#kinetics .cc-pill.on{color:#fff;background:var(--cc-amber);border-color:var(--cc-amber)}
#kinetics .cc-pill.off{color:var(--cc-mut);background:#eef1f5}
#kinetics .cc-pill.warn{color:var(--cc-warn);border-color:var(--cc-warn)}
#kinetics .cc-fresh{margin-left:auto;font-size:12px;color:var(--cc-mut);text-align:right;line-height:1.35}
#kinetics .cc-fresh .stale{color:#b03a3a;font-weight:700}
#kinetics .cc-card{background:var(--cc-surf);border:1px solid var(--cc-line);border-radius:12px;
  padding:14px;box-shadow:0 1px 2px rgba(27,37,48,.05);margin-bottom:14px}
#kinetics .cc-tiles{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:12px}
#kinetics .cc-tile{border:1px solid var(--cc-line);border-radius:10px;padding:10px 12px;background:#fbfcfe}
#kinetics .cc-tile.amber{background:#fdf6ee;border-color:#eccfae}
#kinetics .cc-lbl{font-size:10.5px;font-weight:700;text-transform:uppercase;letter-spacing:.05em;color:var(--cc-mut)}
#kinetics .cc-big{font-size:26px;font-weight:700;line-height:1.15;margin-top:2px}
#kinetics .cc-big .u{font-size:12px;font-weight:500;color:var(--cc-mut)}
#kinetics .cc-sp{font-size:11.5px;color:var(--cc-mut);margin-top:2px}
#kinetics .cc-sp b{color:var(--cc-ink);font-weight:600}
#kinetics .cc-bar{height:5px;border-radius:3px;background:#e7ecf1;margin-top:7px;position:relative;overflow:hidden}
#kinetics .cc-bar .ctr{position:absolute;left:50%;top:0;bottom:0;width:1px;background:#c2ccd6}
#kinetics .cc-bar .fill{position:absolute;top:0;bottom:0;border-radius:3px}
#kinetics .cc-out{font-size:20px;font-weight:700;line-height:1.15;margin-top:2px}
#kinetics .cc-out .u{font-size:11px;font-weight:500;color:var(--cc-mut)}
#kinetics .cc-out.amber{color:var(--cc-amber)}
#kinetics .cc-sub{margin-top:12px;border:1px solid var(--cc-line);border-left:3px solid var(--cc-accent);
  border-radius:8px;background:#f7f9fb;padding:9px 11px}
#kinetics .cc-sub-h{font-size:11px;font-weight:700;text-transform:uppercase;letter-spacing:.04em;color:var(--cc-mut);margin-bottom:5px}
#kinetics .cc-sub-row{font-size:12px;color:var(--cc-ink);line-height:1.7;display:flex;flex-wrap:wrap;gap:6px 12px;align-items:center}
#kinetics .cc-sub-row b{font-weight:600}
#kinetics .cc-mini{font-size:10.5px;font-weight:700;padding:1px 7px;border-radius:9px;border:1px solid var(--cc-line);color:var(--cc-mut);background:#fff}
#kinetics .cc-mini.good{color:var(--cc-good);border-color:var(--cc-good)}
#kinetics .cc-note{font-size:11.5px;color:var(--cc-amber);font-weight:600;margin-top:6px}
#kinetics .cc-trends{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:12px;margin-bottom:14px}
#kinetics .cc-chart{background:var(--cc-surf);border:1px solid var(--cc-line);border-radius:12px;padding:10px 12px;box-shadow:0 1px 2px rgba(27,37,48,.05)}
#kinetics .cc-clbl{font-size:11px;color:var(--cc-mut);margin-bottom:6px;font-weight:600}
#kinetics .cc-chart canvas{width:100%;height:150px;display:block}
#kinetics .cc-foot{font-size:11.5px;color:var(--cc-mut);line-height:1.5;padding:2px 2px 0}
#kinetics .cc-foot b{color:var(--cc-ink)}
/* control card. Every row wraps -- the body must never scroll sideways. */
#kinetics .cc-ctrl{display:flex;flex-wrap:wrap;align-items:center;gap:8px 16px}
#kinetics .cc-grp{display:flex;flex-wrap:wrap;align-items:center;gap:6px}
#kinetics .cc-btn{font:inherit;font-size:12px;font-weight:700;padding:5px 11px;border-radius:8px;
  border:1px solid var(--cc-line);background:var(--cc-surf);color:var(--cc-ink);cursor:pointer}
#kinetics .cc-btn:hover:not(:disabled){border-color:var(--cc-accent);color:var(--cc-accent)}
#kinetics .cc-btn:disabled{color:var(--cc-off);background:#eef1f5;cursor:not-allowed}
#kinetics .cc-btn.arm{color:#fff;background:var(--cc-amber);border-color:var(--cc-amber)}
#kinetics .cc-in{font:inherit;font-size:12px;width:82px;padding:4px 6px;border-radius:7px;
  border:1px solid var(--cc-line);background:var(--cc-surf);color:var(--cc-ink)}
#kinetics .cc-cmd{font-size:11.5px;color:var(--cc-mut);line-height:1.6;margin-top:7px;word-break:break-word}
#kinetics .cc-cmd.good{color:var(--cc-good);font-weight:600}
#kinetics .cc-cmd.warn{color:var(--cc-amber);font-weight:600}
#kinetics .cc-cmd.err{color:#b03a3a;font-weight:600}

</style>
</head>
<body>
<header>
  <h1>SDL&nbsp;Lab</h1>
  <div class="tabs">
    <div class="tab active" data-view="overview">Overview</div>
    <div class="tab" data-view="cameras">Cameras</div>
    <div class="tab" data-view="cctv">CCTV</div>
    <div class="tab" data-view="kinetics">Chamber Control</div>
  </div>
  <span id="clock">connecting…</span>
</header>
<main>
  <section id="overview" class="view active">
    <div class="ov">
      <div class="ov-main">
        <div id="motion" class="motion"></div>
        <div class="cards" id="cards"></div>
      </div>
      <div class="ov-resizer" id="ov-resizer" title="Drag to resize the bench panel"></div>
      <aside class="bench-wrap" id="bench-wrap">
        <div class="bench-head">
          <span class="bench-title">Bench</span>
          <span class="bench-dim" id="bench-dim">&mdash;</span>
        </div>
        <div id="bench" class="bench"></div>
        <div class="legend">
          <span><i class="lg on"></i>online</span><span><i class="lg busy"></i>in operation</span>
          <span><i class="lg off"></i>offline</span><span><i class="lg vacant"></i>not installed</span>
        </div>
        <div id="bench-note" class="bench-note"></div>
      </aside>
    </div>
    <div class="term">
      <div class="term-head">Operation terminal — now: <span id="current">idle</span></div>
      <div id="log"></div>
    </div>
  </section>

  <section id="cameras" class="view">
    <div class="controls">
      <button id="btn-start">Start Record</button>
      <button id="btn-pause">Pause Record</button>
      <button id="btn-photo">Take Photo</button>
      <span class="pill" id="rec-pill">recording: off</span>
      <span class="pill" id="cam-msg"></span>
    </div>
    <div class="cam-grid">
      <div class="cam"><div class="h"><span>Leica K3C</span><span class="pill" id="age-leica">—</span></div>
        <div class="wrap"><img id="img-leica" alt="Leica"></div></div>
      <div class="cam"><div class="h"><span>TIS DFK 33UX264</span><span class="pill" id="age-tis">—</span></div>
        <div class="wrap"><img id="img-tis" alt="TIS"></div></div>
    </div>
  </section>

  <section id="cctv" class="view">
    <div class="cam-grid">
      <div class="cam"><div class="h"><span>Whole-setup overview</span><span class="pill" id="cctv-state">—</span></div>
        <div class="wrap"><div class="placeholder">CCTV node is vacant.<br>Wire a USB/IP overview camera in <code>tools/cctv</code>.</div></div></div>
    </div>
  </section>
  <section id="kinetics" class="view">
    <div class="cc-head">
      <span class="cc-title">Chamber Control</span>
      <span id="cc-pid" class="cc-pill off">PID &mdash;</span>
      <span id="cc-fresh" class="cc-fresh">&mdash;</span>
    </div>
    <div class="cc-card">
      <div class="cc-sub-h">제어</div>
      <div class="cc-ctrl">
        <span id="cc-ctrl-run" class="cc-pill off">컨트롤러 &mdash;</span>
        <span class="cc-grp"><span class="cc-lbl">PID</span>
          <button id="cc-pid-on" class="cc-btn" disabled>PID ON</button>
          <button id="cc-pid-off" class="cc-btn">PID OFF</button></span>
        <span class="cc-grp"><span class="cc-lbl">설정값</span>
          <input id="cc-sp-temp" class="cc-in" type="number" step="0.1" title="온도 설정값 °C"><span class="cc-sp">°C</span>
          <input id="cc-sp-rh" class="cc-in" type="number" step="0.1" title="습도 설정값 %RH"><span class="cc-sp">%RH</span>
          <button id="cc-sp-apply" class="cc-btn">적용</button>
          <button id="cc-preset-paper" class="cc-btn">논문 25 / 93</button>
          <button id="cc-preset-95" class="cc-btn">25 / 95</button></span>
        <span class="cc-grp"><span class="cc-lbl">밸브</span>
          <button id="cc-valve-auto" class="cc-btn" disabled>자동</button>
          <button id="cc-valve-open" class="cc-btn" disabled>강제 개방</button>
          <button id="cc-valve-close" class="cc-btn" disabled>강제 폐쇄</button>
          <span id="cc-valve-note" class="cc-sp"></span></span>
      </div>
      <div id="cc-pending" class="cc-cmd"></div>
      <div id="cc-cmd-msg" class="cc-cmd"></div>
    </div>
    <div class="cc-card">
      <div class="cc-tiles" id="cc-tiles"><div class="cc-tile"><div class="cc-lbl">상태</div><div class="cc-sp">상태 수신 대기 중…</div></div></div>
      <div class="cc-sub" id="cc-circ"></div>
    </div>
    <div class="cc-trends">
      <div class="cc-chart"><div class="cc-clbl">온도 · 설정값(점선) &mdash; °C</div><canvas id="kin-chart-temp"></canvas></div>
      <div class="cc-chart"><div class="cc-clbl">습도 · 설정값(점선) &mdash; %RH</div><canvas id="kin-chart-rh"></canvas></div>
      <div class="cc-chart"><div class="cc-clbl">PWM duty &mdash; RH 밸브 (DF10) · amber &mdash; %</div><canvas id="kin-chart-pwm"></canvas></div>
    </div>
    <div class="cc-foot">결정화 kinetics 계산: <b>미연결</b> (combination_side.m, MATLAB, unported) &mdash; 부피·개수·속도 수치는 표시하지 않습니다.</div>
  </section>
</main>
<script>
const CAM_NODE = "cameras";
loadBench();
function $(id){return document.getElementById(id)}
function fmtTime(ts){return new Date(ts*1000).toLocaleTimeString()}

/* ---- bench panel resize ----------------------------------------------
   .bench-wrap's width is a flex-basis in px, dragged via #ov-resizer and
   persisted in localStorage so it survives a reload. Below the 1100px
   breakpoint the CSS switches .ov to a column and hides the handle, so
   this only ever runs where the two-pane layout is actually shown.     */
const BENCH_MIN = 260, BENCH_MAX_FRAC = 0.7, OV_MAIN_MIN = 320;
const BENCH_KEY = "sdl.benchWidthPx";
function clampBenchWidth(px, ovWidth){
  const max = Math.min(ovWidth * BENCH_MAX_FRAC, ovWidth - OV_MAIN_MIN - 10);
  return Math.max(BENCH_MIN, Math.min(px, Math.max(BENCH_MIN, max)));
}
function applyBenchWidth(px){
  const bw=$("bench-wrap"); if(bw) bw.style.flexBasis=px+"px";
}
(function initBenchResize(){
  const stored = parseFloat(localStorage.getItem(BENCH_KEY));
  if(!isNaN(stored)) applyBenchWidth(stored);
  const handle=$("ov-resizer"), ov=document.querySelector(".ov");
  if(!handle || !ov) return;
  let dragging=false;
  handle.addEventListener("pointerdown", e=>{
    dragging=true; handle.classList.add("dragging");
    handle.setPointerCapture(e.pointerId);
    e.preventDefault();
  });
  handle.addEventListener("pointermove", e=>{
    if(!dragging) return;
    const rect=ov.getBoundingClientRect();
    const px=clampBenchWidth(rect.right - e.clientX, rect.width);
    applyBenchWidth(px);
  });
  function stop(e){
    if(!dragging) return;
    dragging=false; handle.classList.remove("dragging");
    const bw=$("bench-wrap");
    if(bw) localStorage.setItem(BENCH_KEY, parseFloat(bw.style.flexBasis)||"");
  }
  handle.addEventListener("pointerup", stop);
  handle.addEventListener("pointercancel", stop);
  window.addEventListener("resize", ()=>{
    const bw=$("bench-wrap"); if(!bw || !bw.style.flexBasis) return;
    const rect=ov.getBoundingClientRect();
    applyBenchWidth(clampBenchWidth(parseFloat(bw.style.flexBasis), rect.width));
  });
})();

document.querySelectorAll(".tab").forEach(t=>t.onclick=()=>{
  document.querySelectorAll(".tab").forEach(x=>x.classList.remove("active"));
  document.querySelectorAll(".view").forEach(x=>x.classList.remove("active"));
  t.classList.add("active"); $(t.dataset.view).classList.add("active");
});

/* ---- bench diagram --------------------------------------------------
   Layout comes from /api/bench (static), live state from /api/status.
   A slot is one of four states, and they are checked in this order:
     vacant  the node reports it is not implemented
     busy    something is claiming it in tools.occupancy RIGHT NOW
     on      reachable / connected
     off     everything else
   "busy" wins over "on" deliberately: what is moving matters more than what
   is merely plugged in.                                                  */
let BENCH = null;

async function loadBench(){
  try{ BENCH = await (await fetch("/api/bench")).json(); }
  catch(e){ BENCH = null; }
}

function slotState(item, nodes, occ){
  const node = nodes ? Object.values(nodes).find(n=>
      n.name===item.device || n.kind===item.device ||
      (item.device==="microscope" && n.kind==="camera") ||
      (item.device==="uv_vis" && n.kind==="uv_vis")) : null;
  const claim = occ ? occ[item.device] : null;
  if(claim && claim.busy) return {cls:"busy", act:(claim.doing||item.verb)};
  if(node && node.state==="vacant") return {cls:"vacant", act:""};
  if(node && (node.connected===true || node.state==="online"))
    return {cls:"on", act:""};
  if(!node) return {cls:"vacant", act:""};
  return {cls:"off", act:""};
}

function renderBench(nodes, occ){
  const el=$("bench"); if(!el||!BENCH) return;
  const W=BENCH.width_mm, D=BENCH.depth_mm;
  el.style.aspectRatio = W+" / "+D;          // real bench proportions
  const dim=$("bench-dim"); if(dim) dim.textContent=W+" x "+D+" mm";
  el.innerHTML="";
  const centres={};
  BENCH.items.forEach(it=>{
    const st=slotState(it,nodes,occ);
    const d=document.createElement("div");
    d.className="slot "+st.cls+(it.shape==="arm"?" arm":"");
    d.style.left=(100*it.x/W)+"%"; d.style.top=(100*it.y/D)+"%";
    d.style.width=(100*it.w/W)+"%"; d.style.height=(100*it.h/D)+"%";
    const small=(it.w/W < 0.16 || it.h/D < 0.18);   // arm box is tiny; label only
    d.innerHTML=`<span class="pulse"></span><div class="nm">${it.label}</div>`
      +(small?"":`<div class="sb">${it.sub} mm</div>`)
      +(st.act&&!small?`<div class="act">${st.act}</div>`:"");
    d.title=`${it.label} — ${it.w}x${it.h} mm at (${it.x},${it.y}) — ${st.cls}`
      +(st.act?": "+st.act:"");
    el.appendChild(d);
    centres[it.device]={x:100*(it.x+it.w/2)/W, y:100*(it.y+it.h/2)/D};
  });
  // draw the "must never move together" link, lit when either end is busy
  (BENCH.conflicts||[]).forEach(([a,b])=>{
    const A=centres[a], B=centres[b]; if(!A||!B) return;
    const hot=(occ&&((occ[a]&&occ[a].busy)||(occ[b]&&occ[b].busy)));
    const dx=B.x-A.x, dy=B.y-A.y;
    const len=Math.sqrt(dx*dx+dy*dy), ang=Math.atan2(dy,dx)*180/Math.PI;
    const l=document.createElement("div");
    l.className="bench-link"+(hot?" hot":"");
    l.style.left=A.x+"%"; l.style.top=A.y+"%"; l.style.width=len+"%";
    l.style.transform=`rotate(${ang}deg)`; l.style.transformOrigin="0 0";
    l.title=`${a} and ${b} share space and must never move together`;
    el.appendChild(l);
  });
  const busy=Object.values(occ||{}).filter(d=>d.busy);
  const note=$("bench-note");
  if(busy.length===0){ note.className="bench-note"; note.textContent="Nothing is moving."; }
  else if(busy.length===1){ note.className="bench-note";
    note.textContent=busy[0].device+" is in operation — "+(busy[0].doing||"")+
      " ("+Math.round(busy[0].age_s)+"s)"; }
  else { note.className="bench-note hot";
    note.textContent="More than one instrument is moving: "+
      busy.map(d=>d.device).join(", ")+
      ". The arm and the UV-Vis carrier share space and must never overlap."; }
}

function renderMotion(occ){
  const el=$("motion"); if(!el) return;
  const devices=Object.values(occ||{});
  const moving=devices.filter(d=>d.busy);
  let head, cls;
  if(moving.length===0){ cls="idle"; head="IDLE &mdash; nothing is moving"; }
  else { cls="busy"; head="MOVING &mdash; "+moving.map(d=>d.device).join(", "); }
  let rows="";
  devices.sort((a,b)=>a.device.localeCompare(b.device)).forEach(d=>{
    if(d.busy){
      const warn=d.long_running?' <span class="warn">long-running</span>':"";
      const dead=d.alive===false?' <span class="warn">process gone</span>':"";
      rows+=`<div class="mrow"><span class="dot on"></span><b>${d.device}</b>`
          + `<span class="doing">${d.doing||"moving"}</span>`
          + `<span class="since">${Math.round(d.age_s)}s &middot; pid ${d.pid}`
          + `${d.phase?" &middot; "+d.phase:""}</span>${warn}${dead}</div>`;
    } else {
      rows+=`<div class="mrow off"><span class="dot"></span><b>${d.device}</b>`
          + `<span class="doing">idle</span></div>`;
    }
  });
  el.className="motion "+cls;
  el.innerHTML=`<div class="mhead">${head}</div>${rows}`
    + (moving.length>1?'<div class="mnote">More than one instrument is moving. '
      + 'The arm and the UV-Vis carrier share space and must never overlap.</div>':"");
}

/* Card fields are filtered here, at render time, not at the source: node.py
   adapters keep reporting everything they always did (status() is also used
   for logging/debugging), this just trims what the *card* shows. Dropped:
   commands / planned_commands (the advertised command list, incl. vacant
   stubs), envelope (long, operators never read it), gripper (open/close
   speed), locations (the taught-location list -- the "location field" the
   card never needs). Kept implicitly: kind, summary, and every presence/
   connected/power/state-ish diagnostic field a device happens to report,
   plus anything else not named here (e.g. a camera's per-camera age_s). */
const CARD_SKIP = new Set([
  "name", "kind", "state", "commands", "planned_commands",
  "envelope", "gripper", "locations",
]);
function renderCards(nodes){
  const el=$("cards"); el.innerHTML="";
  Object.values(nodes).forEach(n=>{
    // environment/circulator live only under Chamber Control (Task C)
    if(n.kind==="environment"||n.kind==="circulator") return;
    const st=(n.state||"init");
    const card=document.createElement("div"); card.className="card";
    let rows="";
    Object.keys(n).forEach(k=>{ if(CARD_SKIP.has(k))return;
      let v=n[k]; if(typeof v==="object") v=JSON.stringify(v);
      rows+=`<div class="kv"><b>${k}</b>: ${v}</div>`; });
    card.innerHTML=`<h3>${n.name} <span class="badge s-${st}">${st}</span></h3>
      <div class="kv"><b>kind</b>: ${n.kind}</div>${rows}`;
    el.appendChild(card);
  });
}
async function pollStatus(){
  try{const r=await fetch("/api/status");const d=await r.json();
    renderCards(d.nodes); renderMotion(d.occupancy);
    renderBench(d.nodes, d.occupancy);
    onStatus(d);
    $("clock").textContent="updated "+fmtTime(d.ts);
    const cam=Object.values(d.nodes).find(n=>n.kind==="camera");
    if(cam){const rec=cam.recording===true; $("rec-pill").textContent="recording: "+(rec?"ON":"off");
      $("rec-pill").className="pill"+(rec?" on":"");
      if(cam.cameras){ for(const c of ["leica","tis"]){const a=cam.cameras[c]?cam.cameras[c].age_s:null;
        $("age-"+c).textContent=(a==null?"offline":a+"s ago"); } } }
    const cctv=Object.values(d.nodes).find(n=>n.kind==="cctv"); if(cctv)$("cctv-state").textContent=cctv.state;
  }catch(e){$("clock").textContent="status unavailable";}
}
function addLog(m){
  const log=$("log"); const near=log.scrollHeight-log.scrollTop-log.clientHeight<40;
  const div=document.createElement("div"); div.className="ln "+(m.level||"info");
  div.innerHTML=`<span class="t">${fmtTime(m.ts)}</span> <span class="src">[${m.source}]</span> <span class="m">${m.message}</span>`;
  log.appendChild(div); while(log.childElementCount>500)log.removeChild(log.firstChild);
  if(near)log.scrollTop=log.scrollHeight;
  $("current").textContent=`[${m.source}] ${m.message}`;
}
function connectSSE(){
  const es=new EventSource("/events");
  es.onmessage=e=>{try{const m=JSON.parse(e.data); if(m.topic==="ops")addLog(m); if(m.topic==="status")pollStatus();}catch(_){}}
  es.onerror=()=>{setTimeout(connectSSE,3000); es.close();}
}
function refreshCams(){
  if(!$("cameras").classList.contains("active"))return;
  const t=Date.now();
  $("img-leica").src="/camera/leica?t="+t; $("img-tis").src="/camera/tis?t="+t;
}
async function cmd(action,btn){
  if(btn){btn.disabled=true;} $("cam-msg").textContent=action+"…";
  try{const r=await fetch("/command/"+CAM_NODE+"/"+action,{method:"POST"});const d=await r.json();
    $("cam-msg").textContent=d.ok?action+" ok":("error: "+(d.error||""));}
  catch(e){$("cam-msg").textContent="error";}
  finally{if(btn)btn.disabled=false; pollStatus();}
}
$("btn-start").onclick=e=>cmd("start_record",e.target);
$("btn-pause").onclick=e=>cmd("pause_record",e.target);
$("btn-photo").onclick=e=>cmd("take_photo",e.target);


/* ---- Chamber Control tab --------------------------------------------
   Reads ONLY /api/status (never opens its own PLC/Modbus session -- the
   CLICK caps at 3 concurrent clients). Every value below is a real field
   published by tools.environment.node / tools.circulator.node, including
   DF9 (temp_pid_output_c) and DF10 (rh_pid_output_pct = the RH PID output =
   PWM duty %). No kinetics computation exists in the repo, so no
   volume/count/rate number is shown.                                     */
const TREND = [];              // client-side ring buffer of /api/status samples
const TREND_CAP = 300;
function knum(v, dgt){return (typeof v==="number" && isFinite(v)) ? v.toFixed(dgt==null?2:dgt) : "—";}
function pushTrend(ts, env){
  const n = v => (typeof v==="number" && isFinite(v)) ? v : null;
  TREND.push({t:ts, temp:n(env.temp_c), tempSp:n(env.temp_sp_c),
              rh:n(env.rh_pct), rhSp:n(env.rh_sp_pct),
              tOut:n(env.temp_pid_output_c), pwm:n(env.rh_pid_output_pct)});
  while(TREND.length > TREND_CAP) TREND.shift();
}
function onStatus(d){
  const nodes = d.nodes || {};
  const env  = Object.values(nodes).find(n=>n.kind==="environment");
  const circ = Object.values(nodes).find(n=>n.kind==="circulator");
  if(env) pushTrend(d.ts, env);
  renderChamber(env, circ);
  renderControl(env);
  if($("kinetics").classList.contains("active")) drawTrends();
}
function devBar(err, span, amber){
  if(err==null) return '<div class="cc-bar"><div class="ctr"></div></div>';
  const p = Math.min(Math.abs(err)/span, 1) * 50;
  const good = Math.abs(err) <= span*0.5;
  const col = good ? "#2e7d52" : (amber ? "#c96a1e" : "#b7791f");
  const left = err>=0 ? 50 : 50-p;
  return '<div class="cc-bar"><div class="ctr"></div>'
    + '<div class="fill" style="left:'+left+'%;width:'+p+'%;background:'+col+'"></div></div>';
}
function renderChamber(env, circ){
  const pidEl=$("cc-pid"), frEl=$("cc-fresh"), tEl=$("cc-tiles"), cEl=$("cc-circ");
  if(!env){
    if(pidEl){pidEl.className="cc-pill off"; pidEl.textContent="PID —";}
    if(frEl) frEl.textContent="environment 노드 없음";
    if(tEl) tEl.innerHTML='<div class="cc-tile"><div class="cc-sp">environment 노드를 찾지 못했습니다.</div></div>';
    if(cEl) cEl.innerHTML="";
    return;
  }
  // PID overall pill: amber-on / grey-off
  if(pidEl){
    if(env.pid_enabled===true){pidEl.className="cc-pill on"; pidEl.textContent="PID ON";}
    else if(env.pid_enabled===false){pidEl.className="cc-pill off"; pidEl.textContent="PID OFF";}
    else {pidEl.className="cc-pill warn"; pidEl.textContent="PID 미상";}
  }
  // freshness + one-line live-poll honesty
  if(frEl){
    const stale=(typeof env.age_s==="number" && typeof env.stale_after_s==="number" && env.age_s>env.stale_after_s);
    let line1 = (env.age_s==null) ? "age 미상"
      : (stale ? '<span class="stale">STALE '+knum(env.age_s,0)+'s</span>' : 'fresh · '+knum(env.age_s,0)+'s');
    let line2 = env.poll_skipped ? '<br>live poll off / run holds claim' : "";
    frEl.innerHTML = line1 + line2;
  }
  // key tiles -- only the important values
  const tErr=(typeof env.temp_c==="number" && typeof env.temp_sp_c==="number")?(env.temp_c-env.temp_sp_c):null;
  const hErr=(typeof env.rh_pct==="number" && typeof env.rh_sp_pct==="number")?(env.rh_pct-env.rh_sp_pct):null;
  if(tEl) tEl.innerHTML =
      '<div class="cc-tile"><div class="cc-lbl">Temperature</div>'
    + '<div class="cc-big">'+knum(env.temp_c,1)+'<span class="u"> °C</span></div>'
    + '<div class="cc-sp">SP <b>'+knum(env.temp_sp_c,1)+' °C</b> · Δ '+(tErr==null?"—":(tErr>=0?"+":"")+tErr.toFixed(2))+'</div>'
    + devBar(tErr,2,false)+'</div>'
    + '<div class="cc-tile"><div class="cc-lbl">Humidity</div>'
    + '<div class="cc-big">'+knum(env.rh_pct,1)+'<span class="u"> %RH</span></div>'
    + '<div class="cc-sp">SP <b>'+knum(env.rh_sp_pct,1)+' %RH</b> · Δ '+(hErr==null?"—":(hErr>=0?"+":"")+hErr.toFixed(2))+'</div>'
    + devBar(hErr,5,false)+'</div>'
    + '<div class="cc-tile"><div class="cc-lbl">T output (bath cmd)</div>'
    + '<div class="cc-out">'+knum(env.temp_pid_output_c,1)+'<span class="u"> °C</span></div>'
    + '<div class="cc-sp">DF9 → 순환기</div></div>'
    + '<div class="cc-tile amber"><div class="cc-lbl">RH output — PWM duty</div>'
    + '<div class="cc-out amber">'+knum(env.rh_pid_output_pct,1)+'<span class="u"> %</span></div>'
    + '<div class="cc-sp">DF10 · 가습 밸브 duty</div></div>';
  // nested circulator sub-card (subordinate)
  if(cEl){
    if(!circ){ cEl.innerHTML='<div class="cc-sub-h">Circulator (수조)</div><div class="cc-sub-row">circulator 노드를 찾지 못했습니다.</div>'; }
    else {
      const lk = circ.link_open===true ? '<span class="cc-mini good">link open</span>' : '<span class="cc-mini">link closed</span>';
      const rng = Array.isArray(circ.command_range_c) ? (circ.command_range_c[0]+"–"+circ.command_range_c[1]+" °C") : "—";
      cEl.innerHTML =
          '<div class="cc-sub-h">Circulator — bath actuator (subordinate)</div>'
        + '<div class="cc-sub-row">last bath cmd <b>'+knum(env.temp_pid_output_c,1)+' °C</b> '+lk
        + ' &nbsp; range <b>'+rng+'</b></div>'
        + '<div class="cc-note">수조 온도 읽기 없음 (write-only, no readback)</div>';
    }
  }
}
/* ---- Chamber Control: the control card ------------------------------
   Every button POSTs to /command/environment/<name>; nothing here decides.
   While the supervising loop holds the environment claim the node can only
   ENQUEUE a request, so the reply is "queued as #N" and the real outcome
   arrives in the next status poll's last_command block.

   No browser dialog is used anywhere in here -- not confirm, not alert, not
   prompt: a modal blocks the page and cannot be driven from automation. The
   PID ON confirm is a two-click arm on the button itself.                */
const SP_DIRTY = {temp:false, rh:false};   // never overwrite what was typed in
const PID_ARM_MS = 5000;
let pidArmedUntil = 0;
let cmdLocal = null;      // {seq, text} -- what THIS browser just posted
function ccMsg(text, cls){
  const el=$("cc-cmd-msg"); if(!el) return;
  el.className="cc-cmd"+(cls?" "+cls:""); el.textContent=text;
}
function ccDisarmPid(){
  const b=$("cc-pid-on"); if(!b) return;
  b.classList.remove("arm"); b.textContent="PID ON"; pidArmedUntil=0;
}
async function envCmd(action, body, btn){
  if(btn) btn.disabled=true;
  ccMsg(action+" 전송 중…","");
  try{
    const r=await fetch("/command/environment/"+action,
      {method:"POST", headers:{"Content-Type":"application/json"},
       body:JSON.stringify(body||{})});
    const d=await r.json(); const res=d.result||{};
    if(res.queued===true){
      // Accepted, not applied. The outcome line below replaces this as soon as
      // the controller publishes a last_command with this seq or newer.
      cmdLocal={seq:res.seq, text:"#"+res.seq+" "+action+" → queued"};
      ccMsg(cmdLocal.text,"");
    }else if(res.refused){ cmdLocal=null; ccMsg(action+" 거부: "+res.refused,"warn"); }
    else if(res.ok===true){ cmdLocal=null; ccMsg(action+" → "+(res.outcome||"ok"),"good"); }
    else { cmdLocal=null; ccMsg(action+" 실패: "+(res.error||d.error||"unknown"),"err"); }
  }catch(e){ cmdLocal=null; ccMsg(action+" 전송 실패","err"); }
  finally{ if(btn) btn.disabled=false; pollStatus(); }
}
function renderControl(env){
  const run=$("cc-ctrl-run"); if(!run) return;
  const ctl=(env && env.controller) ? env.controller : null;
  const running=!!(env && env.controller_running);
  // The pill reads controller_running -- the same occupancy claim command()
  // branches on -- so a button disabled here really would have been refused.
  if(running){
    run.className="cc-pill on";
    run.textContent="컨트롤러 실행 중"+((ctl && ctl.pid) ? " · pid "+ctl.pid : "");
    run.title=(ctl && ctl.run_dir) ? ctl.run_dir : "";
  }else{
    run.className="cc-pill off";
    run.textContent="컨트롤러 없음 — start_kinetics --execute 필요";
    run.title="";
  }
  const onBtn=$("cc-pid-on");
  if(onBtn){
    onBtn.disabled=!running;
    onBtn.title=running ? "실행 중인 감시 루프에 PID ON 요청"
      : "컨트롤러 없음: 감시 루프 없이 ladder PID를 켜면 수조가 무명령 상태로 남습니다";
    if(!running) ccDisarmPid();
  }
  // The inputs follow the published setpoints until the operator touches them.
  const t=$("cc-sp-temp"), h=$("cc-sp-rh");
  const num=(v)=>(typeof v==="number" && isFinite(v)) ? v.toFixed(1) : "";
  if(t && !SP_DIRTY.temp) t.value=num(env ? env.temp_sp_c : null);
  if(h && !SP_DIRTY.rh)   h.value=num(env ? env.rh_sp_pct : null);
  // Valve: only the controller knows whether it can be forced, and today it
  // reports "unavailable" with a reason. Show the reason, do not guess.
  const mode=ctl ? ctl.valve_mode : null;
  const usable=!!(running && mode && mode!=="unavailable");
  const note=(ctl && ctl.valve_note) ? ctl.valve_note
    : (running ? "컨트롤러가 밸브 모드를 보고하지 않습니다" : "컨트롤러 없음");
  ["auto","open","close"].forEach(m=>{
    const b=$("cc-valve-"+m); if(!b) return;
    b.disabled=!usable; b.title=usable ? ("밸브 모드 "+m+" 요청") : note;
  });
  const vn=$("cc-valve-note");
  if(vn) vn.textContent=usable ? ("현재 "+mode) : note;
  // Outcome line. A published last_command whose seq caught up with what this
  // browser posted replaces the local "queued" text; an older one does not.
  const lc=(env && env.last_command) ? env.last_command : null;
  if(lc && (!cmdLocal || (typeof lc.seq==="number" && lc.seq>=cmdLocal.seq))){
    cmdLocal=null;
    const cls=(lc.outcome==="confirmed") ? "good"
      : (lc.outcome==="failed") ? "err"
      : (lc.outcome==="refused" || lc.outcome==="stale") ? "warn" : "";
    ccMsg("#"+lc.seq+" "+lc.name+" → "+lc.outcome+(lc.detail ? ": "+lc.detail : ""), cls);
  }
  const pend=$("cc-pending");
  if(pend){
    const pc=(env && env.pending_command) ? env.pending_command : null;
    if(!pc){ pend.className="cc-cmd"; pend.textContent=""; }
    else if(pc.error){ pend.className="cc-cmd err"; pend.textContent="대기 명령 판독 불가: "+pc.error; }
    else { pend.className="cc-cmd warn";
           pend.textContent="대기 중 · seq "+pc.seq+" · "+knum(pc.age_s,0)+"s"; }
  }
}
function ccPreset(temp, rh){
  $("cc-sp-temp").value=temp.toFixed(1); $("cc-sp-rh").value=rh.toFixed(1);
  // Filled on purpose, so status must not overwrite it -- and a preset SENDS
  // nothing; 적용 is still the only thing that posts.
  SP_DIRTY.temp=true; SP_DIRTY.rh=true;
  ccMsg("프리셋 입력됨 — 적용을 눌러야 전송됩니다","");
}
["temp","rh"].forEach(k=>{
  const el=$(k==="temp" ? "cc-sp-temp" : "cc-sp-rh");
  if(el) el.addEventListener("input", ()=>{SP_DIRTY[k]=true;});
});
$("cc-pid-on").onclick=e=>{
  const b=e.currentTarget;
  if(Date.now()<=pidArmedUntil){ ccDisarmPid(); envCmd("enable_pid",{},b); return; }
  pidArmedUntil=Date.now()+PID_ARM_MS;
  b.classList.add("arm"); b.textContent="정말 켤까요? (다시 클릭)";
  setTimeout(()=>{ if(Date.now()>pidArmedUntil) ccDisarmPid(); }, PID_ARM_MS+50);
};
$("cc-pid-off").onclick=e=>envCmd("disable_pid",{},e.currentTarget);
$("cc-sp-apply").onclick=e=>{
  const body={};
  const t=parseFloat($("cc-sp-temp").value), h=parseFloat($("cc-sp-rh").value);
  if(SP_DIRTY.temp && isFinite(t)) body.temp_c=t;
  if(SP_DIRTY.rh && isFinite(h)) body.rh_pct=h;
  if(!("temp_c" in body) && !("rh_pct" in body)){ ccMsg("바뀐 설정값이 없습니다","warn"); return; }
  SP_DIRTY.temp=false; SP_DIRTY.rh=false;
  envCmd("set_setpoint", body, e.currentTarget);
};
$("cc-preset-paper").onclick=()=>ccPreset(25,93);
$("cc-preset-95").onclick=()=>ccPreset(25,95);
["auto","open","close"].forEach(m=>{
  const b=$("cc-valve-"+m); if(b) b.onclick=e=>envCmd("valve",{mode:m},e.currentTarget);
});

function drawLineChart(cv, series, unit){
  const ctx = cv.getContext("2d"); if(!ctx) return;
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth || 300, h = cv.clientHeight || 150;
  cv.width = w*dpr; cv.height = h*dpr; ctx.setTransform(dpr,0,0,dpr,0,0);
  ctx.clearRect(0,0,w,h);
  let vals = [];
  series.forEach(s => s.data.forEach(v => { if(v!=null) vals.push(v); }));
  if(vals.length === 0){ ctx.fillStyle="#8a97a4"; ctx.font="12px Arial";
    ctx.fillText("데이터 수집 중…", 12, 22); return; }
  let mn = Math.min(...vals), mx = Math.max(...vals);
  if(mn === mx){ mn -= 1; mx += 1; }
  const padL=42, padR=10, padT=12, padB=18;
  const n = Math.max(2, series[0].data.length);
  const X = i => padL + (w-padL-padR) * (n<=1 ? 0 : i/(n-1));
  const Y = v => padT + (h-padT-padB) * (1 - (v-mn)/(mx-mn));
  ctx.strokeStyle="#dbe2ea"; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(padL,padT); ctx.lineTo(padL,h-padB); ctx.lineTo(w-padR,h-padB); ctx.stroke();
  ctx.fillStyle="#5a6b7b"; ctx.font="10px Arial";
  ctx.fillText(mx.toFixed(1), 4, padT+8);
  ctx.fillText(mn.toFixed(1), 4, h-padB);
  ctx.fillText(unit, 4, (padT+h-padB)/2);
  series.forEach(s => {
    ctx.strokeStyle = s.color; ctx.lineWidth = 1.6;
    ctx.setLineDash(s.dashed ? [5,4] : []);
    ctx.beginPath(); let started = false;
    s.data.forEach((v,i) => {
      if(v == null){ started = false; return; }
      const x = X(i), y = Y(v);
      if(!started){ ctx.moveTo(x,y); started = true; } else ctx.lineTo(x,y);
    });
    ctx.stroke();
  });
  ctx.setLineDash([]);
}
function drawTrends(){
  const cT = $("kin-chart-temp");
  if(cT) drawLineChart(cT, [
    {data:TREND.map(p=>p.temp),   color:"#2563a8", dashed:false},
    {data:TREND.map(p=>p.tempSp), color:"#8a97a4", dashed:true}], "°C");
  const cH = $("kin-chart-rh");
  if(cH) drawLineChart(cH, [
    {data:TREND.map(p=>p.rh),   color:"#2f8f6f", dashed:false},
    {data:TREND.map(p=>p.rhSp), color:"#8a97a4", dashed:true}], "%RH");
  const cP = $("kin-chart-pwm");
  if(cP) drawLineChart(cP, [
    {data:TREND.map(p=>p.pwm), color:"#c96a1e", dashed:false}], "%");
}
const kinTab = document.querySelector('.tab[data-view="kinetics"]');
if(kinTab) kinTab.addEventListener("click", () => drawTrends());
window.addEventListener("resize", () => { if($("kinetics").classList.contains("active")) drawTrends(); });


pollStatus(); setInterval(pollStatus,2000);
connectSSE(); setInterval(refreshCams,1000);
</script>
</body></html>
"""
