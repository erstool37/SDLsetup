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
</style>
</head>
<body>
<header>
  <h1>SDL&nbsp;Lab</h1>
  <div class="tabs">
    <div class="tab active" data-view="overview">Overview</div>
    <div class="tab" data-view="cameras">Cameras</div>
    <div class="tab" data-view="cctv">CCTV</div>
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

pollStatus(); setInterval(pollStatus,2000);
connectSSE(); setInterval(refreshCams,1000);
</script>
</body></html>
"""
