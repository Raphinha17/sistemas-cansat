/* Estación terrena Project Hellmary: interfaz web. Sin dependencias externas (uPlot local).
   Recibe por WebSocket (/ws) los mensajes del backend: hello, packet, event, status, reset. */
"use strict";

const $ = (id) => document.getElementById(id);
const STATE_LABELS = ["AUTOPRUEBA", "TIERRA", "ASCENSO", "CAÍDA", "DESPLIEGUE", "DESCENSO", "ATERRIZADO"];
const ST_DESCENSO = 5;
const MAX_POINTS = 40000;           // por serie; se recorta lo más viejo
const REDRAW_MS = 150;              // las gráficas se redibujan como máximo ~7 veces por segundo

const S = {
  ws: null, wsOk: false,
  d: { m: [], h: [], vz: [], ax: [], ay: [], az: [], ro: [], pi: [], ya: [] },
  paused: false, winS: 120, dirty: false,
  stats: null, source: null, rec: null, speed: 1,
  last: null, estado: null, stateTimes: {},
  clock: { mission: 0, at: 0, running: false },
  flagsDef: [], flagEls: {},
};

/* ---------------- utilidades ---------------- */
const fmt = (v, d = 1) => {
  if (v == null || Number.isNaN(v)) return "—";
  const s = v.toFixed(d);
  return /^-0(\.0+)?$/.test(s) ? s.slice(1) : s;           // sin "-0.0"
};
const pad2 = (n) => String(n).padStart(2, "0");
function fmtT(s) {                                   // segundos -> T+MM:SS (o H:MM:SS)
  s = Math.max(0, Math.floor(s || 0));
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), r = s % 60;
  return "T+" + (h ? h + ":" + pad2(m) : pad2(m)) + ":" + pad2(r);
}
function fmtMS(s) {                                  // eje X de las gráficas
  s = Math.max(0, Math.round(s));
  return pad2(Math.floor(s / 60)) + ":" + pad2(s % 60);
}
function fmtWall(epoch) {
  if (!epoch) return "";
  const d = new Date(epoch * 1000);
  return pad2(d.getHours()) + ":" + pad2(d.getMinutes()) + ":" + pad2(d.getSeconds());
}
function lsGet(k) { try { return localStorage.getItem(k); } catch (e) { return null; } }
function lsSet(k, v) { try { localStorage.setItem(k, v); } catch (e) { /* sin almacenamiento */ } }

/* ---------------- línea de tiempo ---------------- */
function buildTimeline() {
  const tl = $("timeline");
  tl.innerHTML = "";
  STATE_LABELS.forEach((name, i) => {
    const el = document.createElement("div");
    el.className = "st";
    el.id = "st" + i;
    el.innerHTML = `<div class="n">${i} · ${name}</div><div class="t"></div>`;
    tl.appendChild(el);
  });
}
function updateTimeline() {
  STATE_LABELS.forEach((_, i) => {
    const el = $("st" + i), t = S.stateTimes[i];
    el.classList.toggle("current", S.estado === i);
    el.classList.toggle("visited", !!t && S.estado !== i);
    el.querySelector(".t").textContent = t ? `${fmtWall(t.w)} · ${fmtT(t.m)}` : "";
  });
}

/* ---------------- banderas y eventos ---------------- */
function buildFlags(defs) {
  S.flagsDef = defs;
  const box = $("flags");
  box.innerHTML = "";
  S.flagEls = {};
  defs.forEach((f) => {
    const el = document.createElement("div");
    el.className = "flag " + f.level;
    el.innerHTML = `<i></i><span>${f.text}</span>`;
    box.appendChild(el);
    S.flagEls[f.key] = el;
  });
}
function updateFlags(bits) {
  S.flagsDef.forEach((f) => S.flagEls[f.key].classList.toggle("on", !!(bits & f.bit)));
}
function addEvent(e, prepend = true) {
  const li = document.createElement("li");
  li.className = e.level || "info";
  li.innerHTML = `<span class="w">${fmtWall(e.w)}</span><span class="m">${fmtT(e.m)}</span><span class="x"></span>`;
  li.querySelector(".x").textContent = e.text;
  const ul = $("events");
  if (prepend) ul.prepend(li); else ul.appendChild(li);
  while (ul.children.length > 300) ul.removeChild(ul.lastChild);
}

/* ---------------- encabezado y tarjetas ---------------- */
function applyStats(s) {
  S.stats = s;
  const pill = $("linkpill");
  if (s.link === "ok") { pill.className = "pill ok"; pill.textContent = "ENLACE OK"; }
  else if (s.link === "perdido") { pill.className = "pill bad"; pill.textContent = "ENLACE PERDIDO"; }
  else { pill.className = "pill wait"; pill.textContent = "ESPERANDO DATOS"; }
  $("m-rssi").textContent = fmt(s.rssi, 1);
  $("m-snr").textContent = fmt(s.snr, 2);
  $("m-loss").textContent = s.received ? fmt(s.loss_pct, 1) : "—";
  $("m-lostn").textContent = s.received ? `% (${s.lost})` : "%";
  $("m-rx").textContent = s.received;
  $("m-pps").textContent = s.pps ? `${s.pps.toFixed(1)}/s` : "";
  $("v-hmax").textContent = fmt(s.h_max, 1);
  $("techstats").textContent =
    `inválidas: ${s.invalid} · errores de radio: ${s.radio_errors} · duplicados: ${s.dups} · reinicios de secuencia: ${s.seq_restarts}`;
  // Cronómetro de misión: tiempo del último paquete; corre mientras el enlace está bien.
  S.clock = { mission: s.mission_s, at: performance.now(), running: s.link === "ok" };
  if (s.state_times) {
    S.stateTimes = {};
    Object.keys(s.state_times).forEach((k) => (S.stateTimes[+k] = s.state_times[k]));
  }
  if (s.estado != null) S.estado = s.estado;
  updateTimeline();
  updateVz();
  updateBanner();
}
function updateVz() {
  const s = S.stats, card = $("c-vz");
  const v = s ? s.vz_avg : null;
  $("v-vz").textContent = fmt(v, 1);
  card.classList.remove("ok", "warn");
  // Verde dentro de 8 a 12 m/s de descenso; ámbar fuera de rango. Solo en DESCENSO.
  if (v != null && S.estado === ST_DESCENSO) card.classList.add(Math.abs(v) >= 8 && Math.abs(v) <= 12 ? "ok" : "warn");
  $("s-vz").textContent = S.last ? `inst. ${fmt(S.last.vz, 1)} m/s · descenso esperado 8 a 12` : "descenso esperado 8 a 12 m/s";
}
function updateCards(p) {
  $("v-h").textContent = fmt(p.h, 1);
  $("v-p").textContent = fmt(p.p / 100, 1);
  $("s-p").textContent = `${p.p} Pa`;
  $("v-t").textContent = fmt(p.T, 1);
  const bat = $("c-b");
  if (p.b > 0) {                                    // 0 = la placa no mide la pila: no es alarma
    $("v-b").textContent = fmt(p.b, 1); $("u-b").textContent = "V"; $("s-b").innerHTML = "&nbsp;";
    bat.classList.remove("nd");
  } else {
    $("v-b").textContent = "N/D"; $("u-b").textContent = ""; $("s-b").textContent = "no medida por la placa";
    bat.classList.add("nd");
  }
  $("a-ro").textContent = fmt(p.ro, 1) + "°";
  $("a-pi").textContent = fmt(p.pi, 1) + "°";
  $("a-ya").textContent = fmt(p.ya, 0) + "°";
  const incl = inclinacion(p.ro, p.pi);              // ángulo del eje z con la vertical
  $("a-in").textContent = fmt(incl, 0) + "°";
  $("a-incl").classList.toggle("warn", incl > 45);
}
function inclinacion(roll, pitch) {
  const r = (roll * Math.PI) / 180, p = (pitch * Math.PI) / 180;
  return (Math.acos(Math.max(-1, Math.min(1, Math.cos(r) * Math.cos(p)))) * 180) / Math.PI;
}
function updateBanner() {
  const b = $("banner");
  let text = null, cls = "";
  if (!S.wsOk) { text = "SIN CONEXIÓN CON EL SERVIDOR — reintentando…"; }
  else if (S.source && !S.source.ok && S.source.message) { text = "FUENTE DE DATOS: " + S.source.message; cls = "warn"; }
  else if (S.stats && S.stats.link === "perdido") {
    text = `ENLACE PERDIDO — sin paquetes hace ${Math.round(S.stats.silence_s || 0)} s`;
  }
  b.hidden = text === null;
  b.className = cls;
  if (text) b.textContent = text;
  document.body.classList.toggle("stale", !S.wsOk || (S.stats && S.stats.link === "perdido"));
}
function updateRec() {
  const r = S.rec, st = $("recstate"), btn = $("recbtn");
  const on = !!(r && r.active);
  st.className = "recstate " + (on ? "on" : "off");
  if (on) {
    const el = Math.max(0, Date.now() / 1000 - r.started);
    st.textContent = `● GRABANDO ${fmtT(el).slice(2)} · ${r.csv} · ${r.packets} paq.`;
  } else {
    st.textContent = r && r.csv ? `○ SIN GRABAR (última: ${r.csv})` : "○ SIN GRABAR";
  }
  btn.className = "recbtn " + (on ? "stop" : "start");
  btn.textContent = on ? "FINALIZAR TOMA DE DATOS" : "INICIAR TOMA DE DATOS";
}
function applyStatus(m) {
  S.source = m.source;
  S.speed = (m.source && m.source.speed) || 1;
  S.rec = m.rec;
  $("src").textContent = m.source ? m.source.name : "—";
  applyStats(m.s);
  updateRec();
}

/* ---------------- datos de las gráficas ---------------- */
let prevYaw = null;
function pushPoint(p) {
  const d = S.d;
  // El yaw da la vuelta en ±180°: se inserta un hueco para que no dibuje una línea vertical.
  if (prevYaw !== null && Math.abs(p.ya - prevYaw) > 180 && d.m.length) {
    const m = p.m - 1e-4;
    d.m.push(m); d.h.push(p.h); d.vz.push(p.vz);
    d.ax.push(p.ax); d.ay.push(p.ay); d.az.push(p.az);
    d.ro.push(p.ro); d.pi.push(p.pi); d.ya.push(null);
  }
  prevYaw = p.ya;
  d.m.push(p.m); d.h.push(p.h); d.vz.push(p.vz);
  d.ax.push(p.ax); d.ay.push(p.ay); d.az.push(p.az);
  d.ro.push(p.ro); d.pi.push(p.pi); d.ya.push(p.ya);
  if (d.m.length > MAX_POINTS) Object.keys(d).forEach((k) => d[k].splice(0, MAX_POINTS / 4));
  S.dirty = true;
}
function clearData() { Object.keys(S.d).forEach((k) => (S.d[k] = [])); prevYaw = null; S.dirty = true; }

function lowerBound(arr, x) {
  let lo = 0, hi = arr.length;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (arr[mid] < x) lo = mid + 1; else hi = mid; }
  return lo;
}

/* ---------------- gráficas (uPlot) ---------------- */
/* Tema visual: "plano" (predeterminado) o "clasico" (el anterior, intacto). La clase t-plano
   en <html> la pone un script del <head>; aquí se leen los colores que no son CSS. */
const INK = "#0c1830", INK2 = "#3a4764", PAPER = "#f1e9d2", ALERT = "#ff6a13";
const isPlano = () => document.documentElement.classList.contains("t-plano");
const axStyle = () => isPlano()
  ? { stroke: INK, grid: { stroke: "rgba(12,24,48,.16)", width: 1 }, ticks: { stroke: INK, width: 1 },
      font: "500 13px 'IBM Plex Mono', 'DejaVu Sans Mono', monospace" }
  : { stroke: "#9fb0c3", grid: { stroke: "#1e2b3b", width: 1 }, ticks: { stroke: "#1e2b3b" }, font: "14px Segoe UI, Arial" };
const charts = [];

function makeChart(id, series, extra) {
  const host = $(id).querySelector(".plot");
  const opts = Object.assign({
    width: 400, height: 200,
    legend: { show: true },
    cursor: { drag: { x: false, y: false }, points: { show: false } },
    scales: { x: { time: false } },
    axes: [Object.assign({}, axStyle(), { values: (u, v) => v.map(fmtMS) }), Object.assign({}, axStyle(), { size: 56 })],
    series: [{ label: "t", value: (u, v) => (v == null ? "—" : fmtT(v)) }].concat(
      series.map((s) => ({ label: s.label, points: { show: false },
                           // plano: líneas de tinta con distinto trazo (continua, guiones, puntos)
                           stroke: isPlano() ? s.ink : s.color, width: isPlano() ? (s.ancho || 2) : 2.5,
                           dash: isPlano() ? s.dash : undefined,
                           value: (u, v) => (v == null ? "—" : v.toFixed(s.dec == null ? 1 : s.dec)) }))),
  }, extra || {});
  const u = new uPlot(opts, [[], ...series.map(() => [])], host);
  const c = { u, host, keys: series.map((s) => s.key) };
  charts.push(c);
  c.ro = new ResizeObserver(() => fit(c));
  c.ro.observe(host);
  return c;
}
function fit(c) {
  const legend = c.u.root.querySelector(".u-legend");
  const lh = legend ? legend.offsetHeight : 0;
  c.u.setSize({ width: Math.max(80, c.host.clientWidth), height: Math.max(60, c.host.clientHeight - lh - 2) });
}
function buildCharts() {
  const vzBand = (u) => {                           // banda de 8 a 12 m/s de descenso
    const y1 = u.valToPos(-8, "y", true), y2 = u.valToPos(-12, "y", true), { ctx } = u;
    const top = Math.min(y1, y2), alto = Math.abs(y2 - y1);
    ctx.save();
    if (isPlano()) {                                // rayas finas de tinta a 45° y filetes
      const dpr = window.devicePixelRatio || 1, t = Math.max(6, Math.round(7 * dpr));
      const tile = document.createElement("canvas"); tile.width = tile.height = t;
      const g = tile.getContext("2d"); g.strokeStyle = INK; g.globalAlpha = 0.55; g.lineWidth = Math.max(1, dpr * 0.8);
      g.beginPath(); g.moveTo(0, t); g.lineTo(t, 0); g.moveTo(-t / 2, t / 2); g.lineTo(t / 2, -t / 2);
      g.moveTo(t / 2, t * 1.5); g.lineTo(t * 1.5, t / 2); g.stroke();
      ctx.fillStyle = ctx.createPattern(tile, "repeat");
      ctx.fillRect(u.bbox.left, top, u.bbox.width, alto);
      ctx.strokeStyle = INK; ctx.lineWidth = Math.max(1, dpr); ctx.globalAlpha = 1;
      ctx.strokeRect(u.bbox.left, top, u.bbox.width, alto);
    } else {
      ctx.fillStyle = "rgba(52,211,153,.14)";
      ctx.fillRect(u.bbox.left, top, u.bbox.width, alto);
    }
    ctx.restore();
  };
  makeChart("ch-h", [{ key: "h", label: "Altura (m)", color: "#38bdf8", ink: INK, ancho: 2.4 }]);
  makeChart("ch-vz", [{ key: "vz", label: "vz (m/s)", color: "#fb923c", ink: INK, ancho: 2.2 }], { hooks: { drawClear: [vzBand] } });
  makeChart("ch-a", [{ key: "ax", label: "ax (g)", color: "#f87171", dec: 2, ink: INK },
                     { key: "ay", label: "ay (g)", color: "#34d399", dec: 2, ink: INK2, dash: [9, 4] },
                     { key: "az", label: "az (g)", color: "#60a5fa", dec: 2, ink: "#6c7893", dash: [2, 4] }]);
  makeChart("ch-att", [{ key: "ro", label: "roll (°)", color: "#f472b6", ink: INK },
                       { key: "pi", label: "pitch (°)", color: "#fbbf24", ink: INK2, dash: [9, 4] },
                       { key: "ya", label: "yaw (°)", color: "#a78bfa", dec: 0, ink: "#6c7893", dash: [2, 4] }]);
  charts.forEach(fit);
}
function renderCharts() {
  const d = S.d, n = d.m.length;
  if (!n) { charts.forEach((c) => c.u.setData([[], ...c.keys.map(() => [])])); return; }
  const start = S.winS > 0 ? lowerBound(d.m, d.m[n - 1] - S.winS) : 0;
  charts.forEach((c) => c.u.setData([d.m.slice(start), ...c.keys.map((k) => d[k].slice(start))]));
}
setInterval(() => {
  if (S.dirty && !S.paused) { S.dirty = false; renderCharts(); }
}, REDRAW_MS);

/* ---------------- indicador de actitud (canvas) ---------------- */
const att = { c: null, ctx: null };
function drawAttitude() {
  const c = att.c, ctx = att.ctx;
  const dpr = window.devicePixelRatio || 1;
  const w = Math.round(c.clientWidth * dpr), h = Math.round(c.clientHeight * dpr);
  if (!w || !h) return;
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  const p = S.last, roll = p ? p.ro : 0, pitch = p ? p.pi : 0, yaw = p ? p.ya : 0;
  const cx = w / 2, cy = h / 2, Ro = Math.min(w, h) / 2 * 0.97, Ri = Ro * 0.76;
  const rad = (d) => (d * Math.PI) / 180;
  ctx.clearRect(0, 0, w, h);
  ctx.textAlign = "center"; ctx.textBaseline = "middle";
  const pl = isPlano();
  // Paleta: plano = papel y tinta (sin cielo azul ni tierra café); clásico = la anterior
  const P = pl ? {
    anillo: PAPER, tickM: INK, tickm: INK2, rotulo: INK, cero: INK, marca: INK,
    cielo: PAPER, tierra: INK, horizonte: PAPER, escalaCielo: INK, escalaTierra: PAPER,
    aro: INK, cuerpo: INK, halo: PAPER, fuente: "'IBM Plex Mono', 'DejaVu Sans Mono', monospace",
  } : {
    anillo: "#0d1621", tickM: "#9fb0c3", tickm: "#4a5b70", rotulo: "#c9d4e0", cero: "#fbbf24", marca: "#fbbf24",
    cielo: "#2b78b8", tierra: "#7a5230", horizonte: "#ffffff", escalaCielo: "#ffffff", escalaTierra: "#ffffff",
    aro: "#e9eff6", cuerpo: "#fbbf24", halo: null, fuente: "Segoe UI, Arial",
  };

  // Anillo de rumbo (yaw relativo: 0 = rumbo al arrancar)
  ctx.save(); ctx.translate(cx, cy);
  ctx.fillStyle = P.anillo; ctx.beginPath(); ctx.arc(0, 0, Ro, 0, 7); ctx.fill();
  if (pl) { ctx.strokeStyle = INK; ctx.lineWidth = 1.5 * dpr; ctx.beginPath(); ctx.arc(0, 0, Ro - dpr, 0, 7); ctx.stroke(); }
  ctx.rotate(rad(-yaw));
  for (let a = 0; a < 360; a += 10) {
    const major = a % 30 === 0, r1 = Ro, r2 = Ro * (major ? 0.9 : 0.94);
    ctx.strokeStyle = major ? P.tickM : P.tickm; ctx.lineWidth = major ? 2 * dpr : 1 * dpr;
    ctx.beginPath(); ctx.moveTo(Math.sin(rad(a)) * r1, -Math.cos(rad(a)) * r1);
    ctx.lineTo(Math.sin(rad(a)) * r2, -Math.cos(rad(a)) * r2); ctx.stroke();
    if (major) {
      const lab = { 0: "0", 90: "90", 180: "180", 270: "270" }[a] || String(a);
      ctx.save(); ctx.translate(Math.sin(rad(a)) * Ro * 0.83, -Math.cos(rad(a)) * Ro * 0.83);
      ctx.rotate(rad(a)); ctx.fillStyle = a === 0 ? P.cero : P.rotulo;
      ctx.font = `${pl ? "700 " : ""}${Ro * 0.085}px ${P.fuente}`; ctx.fillText(lab, 0, 0); ctx.restore();
    }
  }
  ctx.restore();
  // Marca fija del rumbo
  ctx.fillStyle = P.marca; ctx.beginPath();
  ctx.moveTo(cx, cy - Ro * 0.99); ctx.lineTo(cx - Ro * 0.04, cy - Ro * 1.0 + Ro * 0.07);
  ctx.lineTo(cx + Ro * 0.04, cy - Ro * 1.0 + Ro * 0.07); ctx.fill();

  // Horizonte artificial: gira con -roll y se desplaza con el pitch (cabeceo arriba = horizonte abajo)
  ctx.save(); ctx.translate(cx, cy);
  ctx.beginPath(); ctx.arc(0, 0, Ri, 0, 7); ctx.clip();
  ctx.rotate(rad(-roll));
  const k = Ri / 45, hy = pitch * k;
  ctx.fillStyle = P.cielo; ctx.fillRect(-Ro * 2, -Ro * 2, Ro * 4, Ro * 2 + hy);
  ctx.fillStyle = P.tierra; ctx.fillRect(-Ro * 2, hy, Ro * 4, Ro * 2);
  ctx.strokeStyle = P.horizonte; ctx.lineWidth = 2.5 * dpr;
  ctx.beginPath(); ctx.moveTo(-Ro * 2, hy); ctx.lineTo(Ro * 2, hy); ctx.stroke();
  ctx.lineWidth = 1.5 * dpr; ctx.font = `${pl ? "700 " : ""}${Ri * 0.09}px ${P.fuente}`;
  for (let a = -40; a <= 40; a += 10) {
    if (!a) continue;
    const y = hy - a * k, half = Ri * (a % 20 ? 0.16 : 0.28);
    // plano: la escala va en tinta sobre el cielo (papel) y en papel sobre la tierra (tinta)
    ctx.strokeStyle = ctx.fillStyle = a > 0 ? P.escalaCielo : P.escalaTierra;
    ctx.beginPath(); ctx.moveTo(-half, y); ctx.lineTo(half, y); ctx.stroke();
    if (a % 20 === 0) { ctx.fillText(Math.abs(a), half + Ri * 0.1, y); ctx.fillText(Math.abs(a), -half - Ri * 0.1, y); }
  }
  ctx.restore();

  // Aro del instrumento y símbolo fijo del cuerpo
  ctx.strokeStyle = P.aro; ctx.lineWidth = 3 * dpr;
  ctx.beginPath(); ctx.arc(cx, cy, Ri, 0, 7); ctx.stroke();
  const simbolo = () => {
    ctx.beginPath(); ctx.moveTo(cx - Ri * 0.55, cy); ctx.lineTo(cx - Ri * 0.2, cy);
    ctx.moveTo(cx + Ri * 0.2, cy); ctx.lineTo(cx + Ri * 0.55, cy); ctx.stroke();
  };
  ctx.lineCap = pl ? "butt" : "round";
  if (P.halo) {                                    // plano: halo de papel para verse sobre la tierra de tinta
    ctx.strokeStyle = P.halo; ctx.lineWidth = 8 * dpr; simbolo();
    ctx.fillStyle = P.halo; ctx.beginPath(); ctx.arc(cx, cy, Ri * 0.035 + 2.5 * dpr, 0, 7); ctx.fill();
  }
  ctx.strokeStyle = P.cuerpo; ctx.lineWidth = 4 * dpr; simbolo();
  ctx.fillStyle = P.cuerpo; ctx.beginPath(); ctx.arc(cx, cy, Ri * 0.035, 0, 7); ctx.fill();
}

/* ---------------- WebSocket ---------------- */
function connect() {
  const proto = location.protocol === "https:" ? "wss://" : "ws://";
  const ws = new WebSocket(proto + location.host + "/ws");
  S.ws = ws;
  ws.onopen = () => { S.wsOk = true; updateBanner(); };
  ws.onclose = () => { S.wsOk = false; updateBanner(); setTimeout(connect, 1000); };  // se recupera sola
  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => { try { onMessage(JSON.parse(ev.data)); } catch (e) { console.error(e); } };
}
function onMessage(m) {
  switch (m.type) {
    case "hello":
      clearData();
      $("events").innerHTML = "";
      buildFlags(m.flags);
      m.history.forEach(pushPoint);
      m.events.forEach((e) => addEvent(e, true));
      S.last = m.history.length ? m.history[m.history.length - 1] : null;
      applyStatus(m.status);
      if (S.last) { updateCards(S.last); updateFlags(S.last.f); }
      S.dirty = true;
      break;
    case "packet":
      S.last = m.p;
      pushPoint(m.p);
      updateCards(m.p);
      updateFlags(m.p.f);
      applyStats(m.s);
      break;
    case "event": addEvent(m.e); break;
    case "status": applyStatus(m); break;
    case "reset":
      clearData(); $("events").innerHTML = ""; S.last = null; S.estado = null; S.stateTimes = {};
      updateTimeline(); updateFlags(0);
      break;
  }
}
// Cronómetro y estado de grabación fluidos entre paquetes
setInterval(() => {
  const c = S.clock;
  const t = c.mission + (c.running ? ((performance.now() - c.at) / 1000) * S.speed : 0);
  $("clock").textContent = fmtT(t).replace("T+", "T+ ");
  updateRec();
  if (S.stats && S.stats.link === "perdido") updateBanner();
}, 250);
function drawLoop() { drawAttitude(); requestAnimationFrame(drawLoop); }

/* ---------------- controles ---------------- */
async function post(url) { const r = await fetch(url, { method: "POST" }); return r.json(); }
$("recbtn").onclick = async () => {
  if (S.rec && S.rec.active) {
    if (!confirm("¿Finalizar la toma de datos? Se cierran el CSV y el archivo crudo.")) return;
    S.rec = await post("/api/record/stop");
  } else {
    S.rec = await post("/api/record/start");
  }
  updateRec();
};
$("exportbtn").onclick = async () => {
  const r = await fetch("/api/export.csv");
  if (!r.ok) { alert((await r.json()).error || "No hay sesión para exportar"); return; }
  const blob = await r.blob();
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  const cd = r.headers.get("content-disposition") || "";
  const m = /filename="?([^";]+)"?/.exec(cd);
  a.download = m ? m[1] : "sesion.csv";
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 2000);
};
$("reportbtn").onclick = async () => {
  const btn = $("reportbtn");
  if (btn.disabled) return;
  btn.disabled = true;
  const label = btn.textContent;
  btn.textContent = "Generando…";
  try {
    const r = await fetch("/api/report", { method: "POST" });
    const j = await r.json().catch(() => ({}));
    if (!r.ok) alert(j.error || "No se pudo generar el reporte");
    else {
      btn.title = "Último reporte: " + j.carpeta;
      if (j.aviso) alert("Reporte generado en:\n" + j.carpeta + "\n\nNo se pudo abrir la carpeta: " + j.aviso);
    }   // el registro de eventos muestra la carpeta
  } catch (e) {
    alert("No se pudo generar el reporte: " + e);
  } finally {
    btn.disabled = false;
    btn.textContent = label;
  }
};
$("resetbtn").onclick = async () => {
  if (confirm("¿Reiniciar estadísticas, gráficas y eventos? (No borra los archivos grabados)")) await post("/api/reset");
};
function setPaused(p) {
  S.paused = p;
  $("pausebtn").textContent = p ? "▶ Reanudar" : "⏸ Pausar";
  $("pausebtn").classList.toggle("active", p);
  $("pausetag").hidden = !p;
  if (!p) { S.dirty = true; }
}
$("pausebtn").onclick = () => setPaused(!S.paused);
document.querySelectorAll(".win").forEach((b) => (b.onclick = () => {
  S.winS = +b.dataset.win;
  document.querySelectorAll(".win").forEach((x) => x.classList.toggle("active", x === b));
  S.dirty = true;
  if (S.paused) renderCharts();
}));
function setBandText() {                           // la banda es rayada en el plano
  const small = document.querySelector("#ch-vz .ct small");
  if (small) small.textContent = isPlano() ? "m/s · banda rayada: 8 a 12" : "m/s · banda verde: 8 a 12";
}
function rebuildCharts() {                         // al cambiar de tema se recrean las gráficas
  charts.forEach((c) => { if (c.ro) c.ro.disconnect(); c.u.destroy(); c.host.innerHTML = ""; });
  charts.length = 0;
  buildCharts();
  setBandText();
  S.dirty = true;
  renderCharts();
}
function setTheme(name, save) {
  document.documentElement.classList.toggle("t-plano", name === "plano");
  if (save) lsSet("hellmary_tema", name);
  rebuildCharts();
}
function setVideo(on) {
  document.body.classList.toggle("video", on);
  lsSet("hellmary_video", on ? "1" : "0");
  setTimeout(() => charts.forEach(fit), 50);
}
$("videobtn").onclick = () => setVideo(!document.body.classList.contains("video"));
document.addEventListener("keydown", (e) => {
  if (e.ctrlKey || e.altKey || e.metaKey) return;
  if (e.key === "v" || e.key === "V") setVideo(!document.body.classList.contains("video"));
  else if (e.key === "p" || e.key === "P") setPaused(!S.paused);
  else if (e.key === "t" || e.key === "T") setTheme(isPlano() ? "clasico" : "plano", true);
});
window.addEventListener("resize", () => setTimeout(() => charts.forEach(fit), 50));

/* ---------------- arranque ---------------- */
buildTimeline();
buildCharts();
att.c = $("att"); att.ctx = att.c.getContext("2d");
const qs = new URLSearchParams(location.search);   // ?video=1 abre en modo pantalla de video
if (qs.get("video") === "1" || (qs.get("video") === null && lsGet("hellmary_video") === "1")) setVideo(true);
setBandText();
updateTimeline();
connect();
drawLoop();
