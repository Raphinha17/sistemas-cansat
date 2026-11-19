"""Fuentes de datos intercambiables de la estación terrena.

Todas entregan *líneas de texto* con el formato de rx_station ("HEX,RSSI,SNR"), así que
--sim y --replay pasan por el mismo parser que el puerto serie real.

  SerialSource  puerto serie real (rx_station, 115200 baudios: Serial.begin(115200))
  SimSource     tools/flight_sim.py en tiempo real (con pérdidas y apagones opcionales)
  ReplaySource  repite un CSV grabado a velocidad real o con --speed
"""
from __future__ import annotations

import asyncio
import math
import random
import threading
import time
from typing import Callable, Iterator, Optional, Tuple

import gs_core as C
import telemetry as T


class Source:
    name = "?"
    speed = 1.0          # factor de velocidad (1 = tiempo real); lo usa la interfaz
    ok = True            # False = la fuente tiene un problema (p. ej. puerto desconectado)
    message = ""

    async def run(self, on_line: Callable[[str], "asyncio.Future"]):
        raise NotImplementedError

    def info(self) -> dict:
        return {"name": self.name, "ok": self.ok, "message": self.message,
                "speed": self.speed}


# ---------------------------------------------------------------------------
# Puerto serie
# ---------------------------------------------------------------------------
class SerialSource(Source):
    RETRY_S = 2.0

    def __init__(self, port: str, baud: int = 115200):
        self.port, self.baud = port, baud
        self.name = f"{port} @ {baud}"
        self.ok = False
        self.message = "abriendo puerto..."

    async def run(self, on_line):
        loop = asyncio.get_running_loop()
        q: asyncio.Queue = asyncio.Queue()
        stop = threading.Event()
        t = threading.Thread(target=self._reader, args=(loop, q, stop), daemon=True)
        t.start()
        try:
            while True:
                await on_line(await q.get())
        finally:
            stop.set()

    def _reader(self, loop, q, stop):
        import serial  # pyserial

        ser = None
        buf = b""
        while not stop.is_set():
            try:
                if ser is None:
                    ser = serial.Serial(self.port, self.baud, timeout=0.2)
                    self.ok, self.message = True, ""
                    buf = b""
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                buf += data
                *lines, buf = buf.split(b"\n")
                if len(buf) > 4096:       # sin salto de línea: basura, se descarta
                    buf = b""
                for raw in lines:
                    loop.call_soon_threadsafe(q.put_nowait, raw.decode("utf-8", "replace"))
            except (serial.SerialException, OSError) as e:
                # Puerto desconectado u ocupado: se avisa y se reintenta solo.
                self.ok, self.message = False, f"puerto {self.port} no disponible ({e})"
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                stop.wait(self.RETRY_S)
        if ser is not None:
            ser.close()


# ---------------------------------------------------------------------------
# Simulador (tools/flight_sim.py)
# ---------------------------------------------------------------------------
def sim_lines(seed: int = 1, loss_pct: float = 0.0,
              blackout: Optional[Tuple[float, float]] = None,
              **sim_kwargs) -> Iterator[Tuple[float, str]]:
    """Genera (t_simulado, línea) con la lógica de tools/flight_sim.py.

    loss_pct: % de paquetes que no llegan. blackout: (inicio_s, duración_s) sin paquetes
    (para probar el aviso de enlace perdido). RSSI/SNR sintéticos: bajan con la altura.
    """
    import flight_sim as S

    rnd = random.Random(seed + 4242)
    for t, pkt in S.simulate(seed=seed, **sim_kwargs):
        if blackout and blackout[0] <= t < blackout[0] + blackout[1]:
            continue
        if loss_pct and rnd.random() * 100 < loss_pct:
            continue
        rssi = -58.0 - 0.012 * max(0.0, pkt.altura_m) + rnd.gauss(0, 1.5)
        snr = 9.5 - 0.004 * max(0.0, pkt.altura_m) + rnd.gauss(0, 0.6)
        yield t, C.make_line(pkt, rssi, snr)


class SimSource(Source):
    def __init__(self, speed: float = 1.0, seed: int = 1, loss_pct: float = 0.0,
                 blackout: Optional[Tuple[float, float]] = None):
        self.speed, self.seed, self.loss_pct, self.blackout = speed, seed, loss_pct, blackout
        self.name = f"SIMULADOR (semilla {seed}, x{speed:g})"

    async def run(self, on_line):
        t0 = time.monotonic()
        for t, line in sim_lines(self.seed, self.loss_pct, self.blackout):
            delay = t / self.speed - (time.monotonic() - t0)
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                await asyncio.sleep(0)       # deja respirar al servidor si va atrasado
            await on_line(line)
        self.message = "simulación terminada"
        await asyncio.Event().wait()         # se queda quieto (el enlace se marcará perdido)


# ---------------------------------------------------------------------------
# Repetición de una grabación
# ---------------------------------------------------------------------------
def replay_lines(path: str) -> Iterator[Tuple[float, str]]:
    """Genera (t_s desde el primer paquete, línea) desde un CSV grabado.

    El tiempo sale del reloj de la placa (t_ms), así que se conservan los huecos por
    paquetes perdidos. Si t_ms retrocede (reinicio de la placa) se avanza 0.2 s."""
    rows = C.load_csv(path)
    if not rows:
        raise ValueError(f"{path}: no tiene filas de paquetes válidas")
    t0 = rows[0][0].t_ms
    t = 0.0
    prev = t0
    for pkt, rssi, snr in rows:
        dt = (pkt.t_ms - prev) / 1000
        t += dt if dt >= 0 else 0.2
        prev = pkt.t_ms
        rssi = rssi if math.isfinite(rssi) else 0.0
        snr = snr if math.isfinite(snr) else 0.0
        yield t, C.make_line(pkt, rssi, snr)


class ReplaySource(Source):
    def __init__(self, path: str, speed: float = 1.0):
        self.path, self.speed = path, speed
        self.name = f"REPLAY {path} (x{speed:g})"

    async def run(self, on_line):
        t0 = time.monotonic()
        for t, line in replay_lines(self.path):
            delay = t / self.speed - (time.monotonic() - t0)
            if delay > 0:
                await asyncio.sleep(delay)
            else:
                await asyncio.sleep(0)
            await on_line(line)
        self.message = "reproducción terminada"
        await asyncio.Event().wait()
