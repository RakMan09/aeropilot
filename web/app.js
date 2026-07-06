"use strict";

const STATES = ["IDLE", "ARMED", "BOOST", "COAST", "APOGEE", "DESCENT",
                "LANDED", "SAFE"];
const STATE_COLORS = {
  IDLE: "#6b7c91", ARMED: "#f5c451", BOOST: "#ff8a3d", COAST: "#35c9e6",
  APOGEE: "#d977ff", DESCENT: "#4f8cff", LANDED: "#3ddc84", SAFE: "#ff4d5e"
};

const el = (id) => document.getElementById(id);

// ---- Replay state ----
let data = null;         // {meta, frames}
let ts = [], alts = [], vels = [];
let events = [];         // [{t, text}]
let duration = 0;
let currentT = 0;
let playing = false;
let speed = 2;
let lastFrameTime = 0;
let loggedUpTo = -1;

// ---- Charts ----
function setupCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  canvas.width = Math.max(1, Math.floor(rect.width * dpr));
  canvas.height = Math.max(1, Math.floor(rect.height * dpr));
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, w: rect.width, h: rect.height };
}

function niceRange(min, max) {
  if (min === max) { min -= 1; max += 1; }
  const pad = (max - min) * 0.12;
  return [min - pad, max + pad];
}

function drawChart(canvas, series, color, unit, markers) {
  const { ctx, w, h } = setupCanvas(canvas);
  ctx.clearRect(0, 0, w, h);

  const padL = 46, padR = 14, padT = 12, padB = 24;
  const plotW = w - padL - padR;
  const plotH = h - padT - padB;

  let ymin = Math.min(...series), ymax = Math.max(...series);
  [ymin, ymax] = niceRange(ymin, ymax);

  const xAt = (t) => padL + (duration > 0 ? (t / duration) * plotW : 0);
  const yAt = (v) => padT + plotH - ((v - ymin) / (ymax - ymin)) * plotH;

  // grid + y labels
  ctx.strokeStyle = "#172330";
  ctx.fillStyle = "#6b7c91";
  ctx.font = "10px ui-monospace, monospace";
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const v = ymin + (i / 4) * (ymax - ymin);
    const y = yAt(v);
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
    ctx.fillText(v.toFixed(0), 6, y + 3);
  }
  // x ticks (seconds)
  const xticks = Math.min(8, Math.ceil(duration));
  for (let i = 0; i <= xticks; i++) {
    const t = (i / xticks) * duration;
    const x = xAt(t);
    ctx.fillText(t.toFixed(0) + "s", x - 6, h - 8);
  }

  // zero line
  if (ymin < 0 && ymax > 0) {
    ctx.strokeStyle = "#2b3a4d";
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(padL, yAt(0)); ctx.lineTo(w - padR, yAt(0)); ctx.stroke();
    ctx.setLineDash([]);
  }

  // markers (apogee / deploy / safe) — stagger labels to avoid overlap
  (markers || []).forEach((m, i) => {
    if (m.t == null || m.t > duration) return;
    const x = xAt(m.t);
    ctx.strokeStyle = m.color;
    ctx.setLineDash([3, 3]);
    ctx.globalAlpha = 0.7;
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
    ctx.fillStyle = m.color;
    ctx.fillText(m.label, x + 3, padT + 10 + i * 12);
  });

  // full (dim) path
  const drawPath = (limitT, style, width, alpha) => {
    ctx.strokeStyle = style; ctx.lineWidth = width; ctx.globalAlpha = alpha;
    ctx.beginPath();
    let started = false;
    for (let i = 0; i < ts.length; i++) {
      if (limitT != null && ts[i] > limitT) break;
      const x = xAt(ts[i]), y = yAt(series[i]);
      if (!started) { ctx.moveTo(x, y); started = true; } else { ctx.lineTo(x, y); }
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
  };
  drawPath(null, "#24344a", 1.5, 1);       // whole trajectory, dim
  drawPath(currentT, color, 2.4, 1);       // flown so far, bright

  // current dot
  let idx = 0;
  for (let i = 0; i < ts.length; i++) { if (ts[i] <= currentT) idx = i; else break; }
  const cx = xAt(ts[idx]), cy = yAt(series[idx]);
  ctx.fillStyle = color;
  ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI * 2); ctx.fill();
  ctx.globalAlpha = 0.25;
  ctx.beginPath(); ctx.arc(cx, cy, 9, 0, Math.PI * 2); ctx.fill();
  ctx.globalAlpha = 1;

  return series[idx];
}

// ---- UI updates ----
function frameAt(t) {
  let idx = 0;
  for (let i = 0; i < ts.length; i++) { if (ts[i] <= t) idx = i; else break; }
  return data.frames[idx];
}

function buildStateStrip() {
  const strip = el("stateStrip");
  strip.innerHTML = "";
  STATES.forEach((s) => {
    const span = document.createElement("span");
    span.className = "pill";
    span.dataset.state = s;
    span.textContent = s;
    strip.appendChild(span);
  });
}

function updateStrip(activeName, seen) {
  document.querySelectorAll(".pill").forEach((p) => {
    const s = p.dataset.state;
    p.classList.toggle("done", seen.has(s));
    const isActive = s === activeName;
    p.classList.toggle("active", isActive);
    p.style.background = isActive ? STATE_COLORS[s] : "";
    p.style.borderColor = isActive ? STATE_COLORS[s] : "";
  });
}

function renderLog(t) {
  const list = el("logList");
  // append any events whose time has passed and aren't logged
  for (let i = loggedUpTo + 1; i < events.length; i++) {
    if (events[i].t <= t) {
      const li = document.createElement("li");
      li.innerHTML = `<span class="t">T+${events[i].t.toFixed(2)}s</span>${events[i].text}`;
      list.insertBefore(li, list.firstChild);
      loggedUpTo = i;
    } else break;
  }
}

function update() {
  const f = frameAt(currentT);
  const altNow = drawChart(el("altChart"), alts, "#35c9e6", "m", chartMarkers());
  const velNow = drawChart(el("velChart"), vels, "#8affc1", "m/s", chartMarkers());

  el("clock").textContent = "T+" + currentT.toFixed(2) + "s";
  el("alt-now").textContent = altNow.toFixed(1) + " m";
  el("vel-now").textContent = velNow.toFixed(1) + " m/s";

  const stateName = f.state_name;
  const color = STATE_COLORS[stateName] || "#6b7c91";
  const hero = el("state-name");
  hero.textContent = stateName;
  hero.style.color = color;
  document.querySelector(".state-hero").style.borderLeftColor = color;

  el("r-alt").textContent = f.alt.toFixed(1) + " m";
  el("r-vel").textContent = f.vel.toFixed(1) + " m/s";
  el("r-batt").textContent = (f.batt_mv / 1000).toFixed(2) + " V";
  const dep = el("r-deploy");
  dep.textContent = f.deployed ? "DEPLOYED" : "STOWED";
  dep.className = "r-val" + (f.deployed ? " armed" : "");
  if (f.safe) { hero.style.color = STATE_COLORS.SAFE; }

  const seen = new Set();
  for (const fr of data.frames) { if (fr.t <= currentT) seen.add(fr.state_name); }
  updateStrip(stateName, seen);
  renderLog(currentT);

  el("scrub").value = duration > 0 ? Math.round((currentT / duration) * 1000) : 0;
}

function chartMarkers() {
  const m = [];
  if (data.meta.apogee_t != null)
    m.push({ t: data.meta.apogee_t, color: STATE_COLORS.APOGEE, label: "apogee" });
  if (data.meta.deploy_t != null)
    m.push({ t: data.meta.deploy_t, color: STATE_COLORS.LANDED, label: "deploy" });
  const safeFrame = data.frames.find((f) => f.safe);
  if (safeFrame) m.push({ t: safeFrame.t, color: STATE_COLORS.SAFE, label: "SAFE" });
  return m;
}

// ---- Playback ----
function tick(now) {
  if (!playing) return;
  const dt = (now - lastFrameTime) / 1000;
  lastFrameTime = now;
  currentT += dt * speed;
  if (currentT >= duration) {
    currentT = duration;
    playing = false;
    el("play").innerHTML = "&#9658; Replay";
  }
  update();
  if (playing) requestAnimationFrame(tick);
}

function play() {
  if (currentT >= duration) currentT = 0;
  if (currentT === 0) resetLog();
  playing = true;
  el("play").innerHTML = "&#10073;&#10073; Pause";
  lastFrameTime = performance.now();
  requestAnimationFrame(tick);
}

function pause() {
  playing = false;
  el("play").innerHTML = "&#9658; Launch";
}

function resetLog() {
  el("logList").innerHTML = "";
  loggedUpTo = -1;
}

function restart() {
  currentT = 0;
  resetLog();
  update();
}

// ---- Data loading ----
function buildEvents() {
  events = [];
  let prev = null;
  for (const f of data.frames) {
    if (f.state_name !== prev) {
      let text = "Entered " + f.state_name;
      if (f.state_name === "APOGEE") text = "Apogee &mdash; parachute DEPLOY";
      if (f.state_name === "SAFE") text = "Watchdog fault &rarr; SAFE state";
      if (f.state_name === "BOOST") text = "Liftoff detected";
      if (f.state_name === "LANDED") text = "Touchdown";
      events.push({ t: f.t, text });
      prev = f.state_name;
    }
  }
}

function fillMetrics() {
  const m = data.meta;
  el("m-apogee").textContent = m.observed_apogee_m != null
    ? m.observed_apogee_m.toFixed(1) + " m" : "--";
  el("m-error").textContent = m.apogee_error_m != null
    ? (m.apogee_error_m >= 0 ? "+" : "") + m.apogee_error_m.toFixed(1) + " m" : "n/a";
  el("m-frames").textContent = m.frames_ok;
  el("m-bad").textContent = (m.frames_bad + m.words_bad);
}

async function loadDataset(url) {
  pause();
  const res = await fetch(url + "?v=" + Date.now());
  data = await res.json();
  ts = data.frames.map((f) => f.t);
  alts = data.frames.map((f) => f.alt);
  vels = data.frames.map((f) => f.vel);
  duration = data.meta.duration_s || (ts.length ? ts[ts.length - 1] : 0);
  currentT = 0;
  buildEvents();
  fillMetrics();
  resetLog();
  update();
}

// ---- Wiring ----
function init() {
  buildStateStrip();

  el("play").addEventListener("click", () => (playing ? pause() : play()));
  el("restart").addEventListener("click", restart);

  el("dataset").addEventListener("change", (e) => loadDataset(e.target.value));

  el("speeds").addEventListener("click", (e) => {
    const b = e.target.closest("button");
    if (!b) return;
    speed = parseFloat(b.dataset.speed);
    document.querySelectorAll("#speeds button").forEach((x) =>
      x.classList.toggle("active", x === b));
  });

  el("scrub").addEventListener("input", (e) => {
    pause();
    currentT = (e.target.value / 1000) * duration;
    // rebuild log up to this point
    resetLog();
    for (let i = 0; i < events.length; i++) {
      if (events[i].t <= currentT) { loggedUpTo = i; }
    }
    // re-render whole log
    el("logList").innerHTML = "";
    for (let i = 0; i <= loggedUpTo; i++) {
      const li = document.createElement("li");
      li.innerHTML = `<span class="t">T+${events[i].t.toFixed(2)}s</span>${events[i].text}`;
      el("logList").insertBefore(li, el("logList").firstChild);
    }
    update();
  });

  window.addEventListener("resize", () => update());

  // repo link (best-effort from current host)
  const host = location.hostname;
  if (host.endsWith("github.io")) {
    const user = host.split(".")[0];
    const repo = location.pathname.split("/").filter(Boolean)[0] || "aeropilot";
    el("repoLink").href = `https://github.com/${user}/${repo}`;
  } else {
    el("repoLink").href = "https://github.com/rakman09/aeropilot";
  }

  loadDataset(el("dataset").value);
}

document.addEventListener("DOMContentLoaded", init);
