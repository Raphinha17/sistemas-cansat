"""Núcleo de la estación terrena (sin red ni puerto serie, probado con pytest).

- parse_line: convierte una línea de rx_station ("HEX,RSSI,SNR") en un paquete.
- Stats: altura máxima, tiempo de misión, pérdida por secuencia, tasa, vz promedio de 1 s,
  enlace perdido y eventos.
- Recorder: un CSV y un archivo crudo por sesión de grabación en ground/data/.

Formato de línea: protocol/packet.md. Paquete: protocol/telemetry.py.
"""
from __future__ import annotations

import csv
import math
import os
import re
import sys
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from typing import Optional

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
for _d in ("protocol", "tools"):
    _p = os.path.abspath(os.path.join(_ROOT, _d))
    if _p not in sys.path:
        sys.path.insert(0, _p)

import telemetry as T  # noqa: E402

LINK_TIMEOUT_S = 3.0        # sin paquetes más de esto -> enlace perdido
RATE_WINDOW_S = 5.0         # ventana de la tasa de paquetes por segundo
SEQ_RESTART_GAP = 30000     # salto de secuencia mayor que esto = reinicio de la placa, no pérdida
HISTORY_MAX = 6000          # paquetes que se guardan para la interfaz (~20 min a 5 Hz)

_HEX_RE = re.compile(r"^[0-9A-Fa-f]{%d}$" % (2 * T.PACKET_SIZE))

# Banderas: (clave, bit, texto del evento, nivel)
FLAG_DEFS = [
    ("base_lista", T.F_BASE_LISTA, "Base lista", "info"),
    ("liberacion", T.F_LIBERACION, "Liberación detectada", "ok"),
    ("disparo", T.F_DISPARO, "Disparo del servo", "ok"),
    ("respaldo", T.F_DISPARO_RESPALDO, "Disparo por respaldo", "warn"),
    ("aterrizado", T.F_ATERRIZADO, "Aterrizaje detectado", "ok"),
    ("fallo_baro", T.F_FALLO_BARO, "Fallo de barómetro", "bad"),
    ("fallo_imu", T.F_FALLO_IMU, "Fallo de IMU", "bad"),
    ("no_go", T.F_NO_GO, "NO-GO", "bad"),
]

CSV_FIELDS = (["hora_pc", "epoch_pc"] + T.field_names()
              + ["rssi_dbm", "snr_db", "estado_nombre"])


# ---------------------------------------------------------------------------
# Parser de líneas
# ---------------------------------------------------------------------------
@dataclass
class RxPacket:
    pkt: T.Packet
    rssi: float
    snr: float


def parse_line(line: str):
    """Clasifica una línea del receptor. Devuelve (tipo, dato):

    ("packet", RxPacket)        línea válida HEX,RSSI,SNR
    ("info", texto)             línea informativa que empieza con '#'
    ("radio_error", (cod, rssi, snr))   línea ERR,... (paquete con CRC inválido, etc.)
    ("empty", None)             línea vacía
    ("invalid", motivo)         cualquier otra cosa (se cuenta y se descarta)
    Nunca lanza excepciones.
    """
    s = (line or "").strip().lstrip("﻿")
    if not s:
        return "empty", None
    if s.startswith("#"):
        return "info", s[1:].strip()
    parts = s.split(",")
    try:
        if parts[0] == "ERR":
            if len(parts) != 4:
                return "invalid", "ERR con campos incorrectos"
            return "radio_error", (int(parts[1]), float(parts[2]), float(parts[3]))
        if len(parts) != 3:
            return "invalid", f"se esperaban 3 campos, hay {len(parts)}"
        if not _HEX_RE.match(parts[0]):
            return "invalid", "hex de longitud o caracteres incorrectos"
        rssi, snr = float(parts[1]), float(parts[2])
        if not (math.isfinite(rssi) and math.isfinite(snr)):
            return "invalid", "RSSI/SNR no finitos"
        pkt = T.decode(bytes.fromhex(parts[0]))
    except (ValueError, TypeError) as e:
        return "invalid", f"no se pudo interpretar: {e}"
    # El paquete no lleva CRC propio: se validan versión y estado como filtro básico.
    if pkt.version != T.VERSION:
        return "invalid", f"versión de paquete {pkt.version}"
    if pkt.estado >= len(T.STATE_NAMES):
        return "invalid", f"estado {pkt.estado} fuera de rango"
    return "packet", RxPacket(pkt, rssi, snr)


def packet_dict(rx: RxPacket, mission_s: float, wall: float) -> dict:
    """Paquete compacto para la interfaz web (claves cortas)."""
    p = rx.pkt
    return {"m": round(mission_s, 3), "w": round(wall, 3), "t": p.t_ms, "q": p.secuencia,
            "st": p.estado, "h": p.altura_m, "vz": p.vz_ms, "p": p.presion_pa,
            "T": p.temp_c, "ax": p.ax_g, "ay": p.ay_g, "az": p.az_g,
            "ro": p.roll_deg, "pi": p.pitch_deg, "ya": p.yaw_deg, "f": p.banderas,
            "b": p.bateria_v, "rssi": rx.rssi, "snr": rx.snr}


def make_line(pkt: T.Packet, rssi: float, snr: float) -> str:
    """Línea tal como la imprime rx_station (para --sim y --replay)."""
    return f"{T.encode(pkt).hex().upper()},{rssi:.1f},{snr:.2f}"


# ---------------------------------------------------------------------------
# Estadísticas y eventos
# ---------------------------------------------------------------------------
class Stats:
    def __init__(self, link_timeout_s: float = LINK_TIMEOUT_S):
        self.link_timeout_s = link_timeout_s
        self.reset()

    def reset(self):
        self.received = 0
        self.invalid = 0
        self.radio_errors = 0
        self.lost = 0
        self.dups = 0
        self.seq_restarts = 0
        self.h_max: Optional[float] = None
        self.last_seq: Optional[int] = None
        self.last_rx: Optional[float] = None     # reloj monotónico del último paquete
        self.link = "esperando"                  # "esperando" | "ok" | "perdido"
        self._lost_since: Optional[float] = None
        self._arrivals: deque = deque()
        self._vz: deque = deque()                # (t_ms, vz) de ~1 s
        self._last_t: Optional[int] = None
        self._t_base = 0
        self._mission_ms = 0
        self._flags = 0
        self.estado: Optional[int] = None
        self.state_times: dict = {}
        self.last_rssi: Optional[float] = None
        self.last_snr: Optional[float] = None

    # --- eventos de línea que no son paquete ---
    def on_invalid(self):
        self.invalid += 1

    def on_radio_error(self):
        self.radio_errors += 1

    # --- paquete válido ---
    def on_packet(self, rx: RxPacket, now: float, wall: float) -> list:
        p = rx.pkt
        ev: list = []

        def event(kind, text, level):
            ev.append({"kind": kind, "text": text, "level": level,
                       "m": round(self._mission_ms / 1000, 3), "w": round(wall, 3)})

        # Tiempo de misión: reloj de la placa desde el primer paquete. Si la placa
        # reinicia (t_ms retrocede) el tiempo de misión continúa.
        reboot = False
        if self._last_t is None:
            self._t_base = p.t_ms
        elif p.t_ms + 1000 < self._last_t:
            reboot = True
            self._t_base = p.t_ms - self._mission_ms
            self._vz.clear()
        self._last_t = p.t_ms
        self._mission_ms = max(0, p.t_ms - self._t_base)

        # Enlace
        if self.link == "perdido":
            dur = now - (self._lost_since or now)
            event("enlace", f"Enlace recuperado (silencio de {dur:.1f} s)", "ok")
        elif self.link == "esperando":
            event("enlace", "Primer paquete recibido", "ok")
        self.link = "ok"
        self._lost_since = None
        self.last_rx = now
        self.last_rssi, self.last_snr = rx.rssi, rx.snr
        if reboot:
            event("placa", "Reinicio de la placa detectado (t_ms retrocedió)", "warn")

        # Paquetes perdidos por secuencia (u16 que da la vuelta)
        if self.last_seq is not None:
            if p.secuencia == self.last_seq:
                self.dups += 1
            else:
                gap = (p.secuencia - self.last_seq - 1) & 0xFFFF
                if gap > SEQ_RESTART_GAP:
                    self.seq_restarts += 1   # secuencia hacia atrás: reinicio, no pérdida
                else:
                    self.lost += gap
        self.last_seq = p.secuencia
        self.received += 1

        # Tasa y velocidad vertical promedio
        self._arrivals.append(now)
        while self._arrivals and now - self._arrivals[0] > RATE_WINDOW_S:
            self._arrivals.popleft()
        self._vz.append((p.t_ms, p.vz_ms))
        while self._vz and self._vz[0][0] < p.t_ms - 1000:
            self._vz.popleft()

        if self.h_max is None or p.altura_m > self.h_max:
            self.h_max = p.altura_m

        # Cambio de estado y banderas que se encienden
        if p.estado != self.estado:
            self.state_times[p.estado] = {"m": round(self._mission_ms / 1000, 3),
                                          "w": round(wall, 3)}
            event("estado", f"Estado: {T.STATE_NAMES[p.estado]}", "info")
            self.estado = p.estado
        rising = p.banderas & ~self._flags
        for _key, bit, text, level in FLAG_DEFS:
            if rising & bit:
                event("bandera", text, level)
        self._flags = p.banderas
        return ev

    def tick(self, now: float, wall: float) -> list:
        """Detecta el enlace perdido (más de link_timeout_s sin paquetes)."""
        if self.link == "ok" and self.last_rx is not None \
                and now - self.last_rx > self.link_timeout_s:
            self.link = "perdido"
            self._lost_since = self.last_rx
            return [{"kind": "enlace", "level": "bad", "w": round(wall, 3),
                     "m": round(self._mission_ms / 1000, 3),
                     "text": f"Enlace perdido (sin paquetes más de {self.link_timeout_s:g} s)"}]
        return []

    # --- cálculos ---
    @property
    def mission_s(self) -> float:
        return self._mission_ms / 1000

    @property
    def loss_pct(self) -> float:
        total = self.received + self.lost
        return 100.0 * self.lost / total if total else 0.0

    def vz_avg(self) -> Optional[float]:
        if not self._vz:
            return None
        return sum(v for _, v in self._vz) / len(self._vz)

    def pps(self, now: float) -> float:
        """Paquetes por segundo en la ventana reciente (0 si el enlace está callado)."""
        while self._arrivals and now - self._arrivals[0] > RATE_WINDOW_S:
            self._arrivals.popleft()
        n = len(self._arrivals)
        if n < 2:
            return 0.0
        span = self._arrivals[-1] - self._arrivals[0]
        return (n - 1) / span if span > 0 else 0.0

    def snapshot(self, now: float) -> dict:
        silence = None if self.last_rx is None else max(0.0, now - self.last_rx)
        return {
            "received": self.received, "invalid": self.invalid,
            "radio_errors": self.radio_errors, "lost": self.lost,
            "loss_pct": round(self.loss_pct, 2), "dups": self.dups,
            "seq_restarts": self.seq_restarts,
            "pps": round(self.pps(now), 2), "h_max": self.h_max,
            "mission_s": round(self.mission_s, 3), "vz_avg": self.vz_avg(),
            "link": self.link, "silence_s": None if silence is None else round(silence, 2),
            "rssi": self.last_rssi, "snr": self.last_snr, "estado": self.estado,
            "state_times": {str(k): v for k, v in self.state_times.items()},
        }


# ---------------------------------------------------------------------------
# Grabación de sesiones
# ---------------------------------------------------------------------------
class Recorder:
    """Un CSV (variables decodificadas + RSSI/SNR + hora) y un crudo (líneas recibidas)
    por sesión, en data_dir. Se escribe y se vacía el búfer en cada línea: si se corta
    la corriente de la laptop se pierde a lo sumo la última línea."""

    def __init__(self, data_dir: str):
        self.data_dir = data_dir
        self.active = False
        self.csv_path: Optional[str] = None
        self.raw_path: Optional[str] = None
        self.started_wall: Optional[float] = None
        self.packets = 0
        self.raw_lines = 0
        self._csv = None
        self._csv_w = None
        self._raw = None

    def start(self, wall: Optional[float] = None) -> dict:
        if self.active:
            return self.info()
        wall = time.time() if wall is None else wall
        os.makedirs(self.data_dir, exist_ok=True)
        stamp = datetime.fromtimestamp(wall).strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.data_dir, f"sesion_{stamp}")
        n = 1
        while os.path.exists(base + ".csv") or os.path.exists(base + ".raw.txt"):
            n += 1
            base = os.path.join(self.data_dir, f"sesion_{stamp}_{n}")
        self.csv_path, self.raw_path = base + ".csv", base + ".raw.txt"
        self._csv = open(self.csv_path, "w", newline="", encoding="utf-8")
        self._csv_w = csv.writer(self._csv)
        self._csv_w.writerow(CSV_FIELDS)
        self._raw = open(self.raw_path, "w", newline="", encoding="utf-8")
        self._csv.flush()
        self.started_wall = wall
        self.packets = self.raw_lines = 0
        self.active = True
        return self.info()

    def stop(self) -> dict:
        self.active = False
        for f in (self._csv, self._raw):
            if f:
                f.close()
        self._csv = self._csv_w = self._raw = None
        return self.info()

    def write_raw(self, line: str):
        if self.active and self._raw:
            self._raw.write(line.rstrip("\r\n") + "\n")
            self._raw.flush()
            self.raw_lines += 1

    def write_packet(self, rx: RxPacket, wall: float):
        if not (self.active and self._csv_w):
            return
        p = rx.pkt
        row = [datetime.fromtimestamp(wall).isoformat(timespec="milliseconds"),
               f"{wall:.3f}"]
        row += [getattr(p, name) for name in T.field_names()]
        row += [rx.rssi, rx.snr, T.STATE_NAMES[p.estado]]
        self._csv_w.writerow(row)
        self._csv.flush()
        self.packets += 1

    def info(self) -> dict:
        return {"active": self.active, "csv": self.csv_path and os.path.basename(self.csv_path),
                "raw": self.raw_path and os.path.basename(self.raw_path),
                "started": self.started_wall, "packets": self.packets,
                "raw_lines": self.raw_lines}


# ---------------------------------------------------------------------------
# Lectura de una grabación (--replay)
# ---------------------------------------------------------------------------
_INT_FIELDS = {"version", "estado", "secuencia", "t_ms", "presion_pa", "banderas"}


def load_csv(path: str) -> list:
    """Lee un CSV grabado y devuelve [(Packet, rssi, snr)]. Salta filas dañadas."""
    out = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            try:
                kw = {n: (int(float(row[n])) if n in _INT_FIELDS else float(row[n]))
                      for n in T.field_names()}
                out.append((T.Packet(**kw), float(row.get("rssi_dbm", "nan")),
                            float(row.get("snr_db", "nan"))))
            except (KeyError, ValueError, TypeError):
                continue
    return out
