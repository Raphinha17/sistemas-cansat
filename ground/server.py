"""Estación terrena Project Hellmary: backend FastAPI + WebSocket + interfaz web local.

Uso (desde la raíz del repositorio, ver ground/README.md):
  py ground/server.py --sim
  py ground/server.py --port COM5
  py ground/server.py --replay ground/data/sesion_XXXX.csv --speed 4
y abrir http://127.0.0.1:8000 en el navegador.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import subprocess
import sys
import time
from collections import deque
from contextlib import asynccontextmanager

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

from fastapi import FastAPI, WebSocket, WebSocketDisconnect  # noqa: E402
from fastapi.responses import FileResponse, JSONResponse, Response  # noqa: E402
from fastapi.staticfiles import StaticFiles  # noqa: E402

import gs_core as C  # noqa: E402
import gs_sources as SRC  # noqa: E402
import telemetry as T  # noqa: E402

STATIC_DIR = os.path.join(HERE, "static")
REPORT_SCRIPT = os.path.abspath(os.path.join(HERE, "..", "tools", "reporte_vuelo.py"))
REPORT_TIMEOUT_S = 300
DEFAULT_DATA_DIR = os.path.join(HERE, "data")
EVENTS_MAX = 500
CLIENT_QUEUE_MAX = 2000
TICK_S = 0.5

_LOGO_PLACEHOLDER = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 320 120">
<rect x="2" y="2" width="316" height="116" rx="10" fill="#1b2430" stroke="#fbbf24"
 stroke-width="3" stroke-dasharray="10 6"/>
<text x="160" y="52" font-family="Segoe UI,Arial,sans-serif" font-size="26" font-weight="700"
 fill="#fbbf24" text-anchor="middle">LOGO DEL EQUIPO</text>
<text x="160" y="80" font-family="Segoe UI,Arial,sans-serif" font-size="14" fill="#c9d4e0"
 text-anchor="middle">poner el archivo en</text>
<text x="160" y="100" font-family="Consolas,monospace" font-size="15" fill="#e8eef5"
 text-anchor="middle">ground/static/logo.png</text></svg>"""


class Station:
    """Estado de la estación: fuente -> parser -> estadísticas -> grabación -> WebSocket."""

    def __init__(self, source, data_dir: str = DEFAULT_DATA_DIR, auto_record: bool = False,
                 clock=time.monotonic, wall=time.time, open_reports: bool = True):
        self.source = source
        self.stats = C.Stats()
        self.recorder = C.Recorder(data_dir)
        self.auto_record = auto_record
        self.clock, self.wall = clock, wall
        self.history: deque = deque(maxlen=C.HISTORY_MAX)
        self.events: deque = deque(maxlen=EVENTS_MAX)
        self.clients: set = set()
        self.open_reports = open_reports     # abrir la carpeta del reporte al terminar
        self.report_busy = False

    # --- línea recibida de la fuente ---
    async def handle_line(self, line: str):
        now, wall = self.clock(), self.wall()
        kind, data = C.parse_line(line)
        if kind != "empty":
            self.recorder.write_raw(line)
        if kind == "packet":
            events = self.stats.on_packet(data, now, wall)
            self.recorder.write_packet(data, wall)
            pd = C.packet_dict(data, self.stats.mission_s, wall)
            self.history.append(pd)
            await self.broadcast({"type": "packet", "p": pd, "s": self.stats.snapshot(now)})
            await self.add_events(events)
        elif kind == "invalid":
            self.stats.on_invalid()
        elif kind == "radio_error":
            self.stats.on_radio_error()

    async def add_events(self, events: list):
        for ev in events:
            self.events.append(ev)
            await self.broadcast({"type": "event", "e": ev})

    # --- difusión por WebSocket ---
    async def broadcast(self, msg: dict):
        text = json.dumps(msg, separators=(",", ":"))
        for q in list(self.clients):
            try:
                q.put_nowait(text)
            except asyncio.QueueFull:
                pass  # cliente lento: se descartan mensajes, el servidor nunca espera

    def status(self) -> dict:
        now = self.clock()
        return {"type": "status", "s": self.stats.snapshot(now),
                "source": self.source.info(), "rec": self.recorder.info(),
                "clients": len(self.clients), "now": self.wall()}

    def hello(self) -> dict:
        return {"type": "hello", "status": self.status(), "history": list(self.history),
                "events": list(self.events), "states": T.STATE_NAMES,
                "flags": [{"key": k, "bit": b, "text": t, "level": lv}
                          for k, b, t, lv in C.FLAG_DEFS]}

    async def tick_loop(self):
        while True:
            await asyncio.sleep(TICK_S)
            await self.add_events(self.stats.tick(self.clock(), self.wall()))
            await self.broadcast(self.status())

    # --- grabación ---
    async def start_recording(self) -> dict:
        info = self.recorder.start(self.wall())
        await self.add_events([{"kind": "grabacion", "level": "ok", "w": self.wall(),
                                "m": self.stats.mission_s,
                                "text": f"Toma de datos iniciada ({info['csv']})"}])
        return info

    async def stop_recording(self) -> dict:
        was = self.recorder.active
        info = self.recorder.stop()
        if was:
            await self.add_events([{"kind": "grabacion", "level": "warn", "w": self.wall(),
                                    "m": self.stats.mission_s,
                                    "text": f"Toma de datos finalizada ({info['packets']} paquetes)"}])
        return info

    # --- reporte de vuelo (tools/reporte_vuelo.py) ---
    async def make_report(self) -> tuple:
        """Genera el reporte de la sesión actual (o la última grabada) en otro proceso, para
        no frenar la recepción. Devuelve (código HTTP, respuesta)."""
        csv_path = self.recorder.csv_path
        if not csv_path or not os.path.isfile(csv_path):
            return 404, {"error": "no hay sesión grabada: inicia la toma de datos"}
        if self.report_busy:
            return 409, {"error": "ya se está generando un reporte"}
        self.report_busy = True
        try:
            cmd = [sys.executable, REPORT_SCRIPT, csv_path, "--json"]
            if self.open_reports:
                cmd.append("--abrir")
            try:
                res = await asyncio.to_thread(
                    subprocess.run, cmd, capture_output=True, text=True, encoding="utf-8",
                    errors="replace", timeout=REPORT_TIMEOUT_S)
            except (OSError, subprocess.TimeoutExpired) as e:
                return 500, {"error": f"no se pudo generar el reporte: {e}"}
            info = None
            for line in reversed(res.stdout.splitlines()):
                if line.startswith("{"):
                    try:
                        info = json.loads(line)
                    except ValueError:
                        pass
                    break
            if res.returncode != 0 or not info:
                err = (res.stderr.strip().splitlines() or ["error desconocido"])[-1]
                return 500, {"error": f"no se pudo generar el reporte: {err}"}
        finally:
            self.report_busy = False
        parcial = " (sesión en curso: datos hasta ahora)" if self.recorder.active else ""
        await self.add_events([{"kind": "reporte", "level": "ok", "w": self.wall(),
                                "m": self.stats.mission_s,
                                "text": f"Reporte generado{parcial}: {info['carpeta']}"}])
        return 200, dict(info, ok=True, csv=os.path.basename(csv_path))

    async def reset(self):
        self.stats.reset()
        self.history.clear()
        self.events.clear()
        await self.broadcast({"type": "reset"})


def create_app(station: Station) -> FastAPI:
    @asynccontextmanager
    async def lifespan(app: FastAPI):
        tasks = [asyncio.create_task(station.tick_loop()),
                 asyncio.create_task(_run_source(station))]
        if station.auto_record:
            await station.start_recording()
        yield
        for t in tasks:
            t.cancel()
        station.recorder.stop()

    app = FastAPI(title="Project Hellmary - estación terrena", lifespan=lifespan)
    app.state.station = station

    @app.get("/")
    async def index():
        return FileResponse(os.path.join(STATIC_DIR, "index.html"),
                            headers={"Cache-Control": "no-store"})

    @app.get("/logo")
    async def logo():
        path = os.path.join(STATIC_DIR, "logo.png")
        if os.path.isfile(path):
            return FileResponse(path, headers={"Cache-Control": "no-cache"})
        # Sin logotipo: marcador de posición que dice dónde poner el archivo.
        return Response(_LOGO_PLACEHOLDER, media_type="image/svg+xml",
                        headers={"Cache-Control": "no-cache"})

    @app.get("/api/status")
    async def api_status():
        return station.status()

    @app.post("/api/record/start")
    async def record_start():
        return await station.start_recording()

    @app.post("/api/record/stop")
    async def record_stop():
        return await station.stop_recording()

    @app.post("/api/report")
    async def api_report():
        code, body = await station.make_report()
        return JSONResponse(body, status_code=code)

    @app.post("/api/reset")
    async def api_reset():
        await station.reset()
        return {"ok": True}

    @app.get("/api/export.csv")
    async def export_csv():
        r = station.recorder
        if not r.csv_path or not os.path.isfile(r.csv_path):
            return JSONResponse({"error": "no hay sesión grabada: inicia la toma de datos"},
                                status_code=404)
        return FileResponse(r.csv_path, media_type="text/csv",
                            filename=os.path.basename(r.csv_path))

    @app.websocket("/ws")
    async def ws(sock: WebSocket):
        await sock.accept()
        q: asyncio.Queue = asyncio.Queue(maxsize=CLIENT_QUEUE_MAX)
        await sock.send_text(json.dumps(station.hello(), separators=(",", ":")))
        station.clients.add(q)

        async def sender():
            while True:
                await sock.send_text(await q.get())

        task = asyncio.create_task(sender())
        try:
            while True:
                await sock.receive_text()   # la interfaz no manda nada; detecta el cierre
        except (WebSocketDisconnect, RuntimeError):
            pass
        finally:
            task.cancel()
            station.clients.discard(q)

    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
    return app


async def _run_source(station: Station):
    """Corre la fuente; si falla, lo reporta en el estado y no tumba el servidor."""
    try:
        await station.source.run(station.handle_line)
    except asyncio.CancelledError:
        raise
    except Exception as e:  # noqa: BLE001
        station.source.ok, station.source.message = False, f"fuente detenida: {e}"
        print(f"[fuente] error: {e}", file=sys.stderr)


def _blackout(s: str):
    try:
        a, b = (float(x) for x in s.split(","))
        return a, b
    except ValueError:
        raise argparse.ArgumentTypeError("usa INICIO,DURACION en segundos (p. ej. 60,8)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Estación terrena Project Hellmary")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--port", help="puerto serie de la Heltec receptora (p. ej. COM5)")
    g.add_argument("--sim", action="store_true", help="vuelo simulado (tools/flight_sim.py)")
    g.add_argument("--replay", metavar="CSV", help="repite una grabación (CSV de ground/data)")
    g.add_argument("--list-ports", action="store_true", help="lista los puertos serie y sale")
    ap.add_argument("--baud", type=int, default=115200, help="baudios (rx_station: 115200)")
    ap.add_argument("--speed", type=float, default=1.0, help="velocidad de --sim/--replay")
    ap.add_argument("--sim-seed", type=int, default=1)
    ap.add_argument("--sim-loss", type=float, default=0.0, help="%% de paquetes perdidos en --sim")
    ap.add_argument("--sim-blackout", type=_blackout, metavar="INICIO,DUR",
                    help="--sim sin paquetes en esa ventana (s), para probar el enlace perdido")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--web-port", type=int, default=8000, help="puerto HTTP de la interfaz")
    ap.add_argument("--data-dir", default=DEFAULT_DATA_DIR)
    ap.add_argument("--auto-record", action="store_true",
                    help="empieza a grabar al arrancar (sin pulsar el botón)")
    ap.add_argument("--no-abrir-reporte", action="store_true",
                    help="«Generar reporte» no abre la carpeta (solo la crea)")
    a = ap.parse_args(argv)

    if a.list_ports:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device}\t{p.description}")
        return 0
    if a.speed <= 0:
        ap.error("--speed debe ser > 0")
    if a.sim:
        source = SRC.SimSource(a.speed, a.sim_seed, a.sim_loss, a.sim_blackout)
    elif a.replay:
        if not os.path.isfile(a.replay):
            ap.error(f"no existe {a.replay}")
        source = SRC.ReplaySource(a.replay, a.speed)
    elif a.port:
        source = SRC.SerialSource(a.port, a.baud)
    else:
        ap.error("elige una fuente: --port COMx, --sim o --replay archivo.csv")

    import uvicorn
    station = Station(source, a.data_dir, a.auto_record,
                      open_reports=not a.no_abrir_reporte)
    print(f"Fuente: {source.name}\nDatos en: {os.path.abspath(a.data_dir)}\n"
          f"Abre http://{a.host}:{a.web_port} en el navegador (Ctrl+C para salir)")
    uvicorn.run(create_app(station), host=a.host, port=a.web_port, log_level="warning")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
