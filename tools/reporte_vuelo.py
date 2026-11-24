#!/usr/bin/env python3
"""Reporte de vuelo de Project Hellmary a partir de una sesión de la estación terrena.

Lee un CSV de sesión (ground/data/sesion_*.csv; por defecto el más reciente con paquetes)
y escribe en <carpeta del CSV>/reporte_<fecha>/:

  01_altura.png ... 06_enlace.png   gráficas (200 dpi) con las fases sombreadas por estado
  reporte.md, reporte.html          tabla de números (el HTML lleva las imágenes adentro)
  resumen.json                      los mismos números, para otras herramientas
  reporte.pdf                       con --pdf (dos hojas)

  py tools/reporte_vuelo.py                                    # sesión más reciente
  py tools/reporte_vuelo.py ground/data/sesion_20261119_172700.csv --pdf
  py tools/reporte_vuelo.py sesion.csv --log vuelo_dump.csv    # superpone el log de la placa

--log acepta el log interno bajado con DUMP: un CSV con las columnas del paquete (al menos
t_ms y altura_m) o el texto tal como lo imprime DUMP (una línea hex de 32 B por registro).

Criterios de cada número: docs/PREGUNTAS.md (P50 a P56).
"""
from __future__ import annotations

import argparse
import base64
import csv
import glob
import html
import json
import math
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, ".."))
_PROTO = os.path.join(ROOT, "protocol")
if _PROTO not in sys.path:
    sys.path.insert(0, _PROTO)
import telemetry as T  # noqa: E402

EQUIPO = "Project Hellmary"
CONCURSO = "CanSat PEUUM 2026"
DATA_DIR = os.path.join(ROOT, "ground", "data")
LOGO = os.path.join(ROOT, "ground", "static", "logo.png")

BANDA_DESCENSO = (8.0, 12.0)    # m/s, velocidad de descenso pedida con el autogiro
T_POST_ATERRIZAJE = 10.0        # s que hay que seguir transmitiendo tras aterrizar
HUECO_S = 3.0                   # s sin paquetes = hueco de enlace (como "enlace perdido")
SEQ_RESTART_GAP = 30000         # igual que la estación: salto mayor = reinicio de la placa
H_SOBRE_SUELO = 2.0             # m sobre el suelo para considerar que tocó tierra
IMPACTO_MIN_G = 2.0             # pico que se toma como el instante del impacto
VZ_FISICA = 80.0                # m/s: más que esto es el estimador corrompido, no el vuelo
SALTO_V = 80.0                  # m/s y margen (m) del filtro de saltos de altura
SALTO_MARGEN = 15.0
HAMPEL_M = 30.0                 # m: pico aislado de altura (ventana de 5 muestras)
BIN_PERDIDAS_S = 5.0

CAMPOS_NUM = ["presion_pa", "temp_c", "altura_m", "vz_ms", "ax_g", "ay_g", "az_g",
              "roll_deg", "pitch_deg", "yaw_deg", "bateria_v", "rssi_dbm", "snr_db",
              "epoch_pc"]
_HEX64 = re.compile(r"^[0-9A-Fa-f]{%d}$" % (2 * T.PACKET_SIZE))

ESTADOS = ["AUTOPRUEBA", "TIERRA", "ASCENSO", "CAÍDA", "DESPLIEGUE", "DESCENSO", "ATERRIZADO"]
COLOR_FASE = ["#ecebe8", "#e4e2db", "#dce9f8", "#fadcd8", "#fbedc6", "#d6f0e4", "#e6ece0"]
# Paleta categórica de referencia (validada para daltonismo en el orden 1-2-3).
C1, C2, C3, C8 = "#2a78d6", "#eb6834", "#1baf7a", "#e34948"
TINTA, TINTA2, TINTA3 = "#0b0b0b", "#52514e", "#8a8984"

ARCHIVOS_PNG = ["01_altura.png", "02_velocidad_vertical.png", "03_presion_temperatura.png",
                "04_aceleracion.png", "05_actitud.png", "06_enlace.png"]
ARCHIVOS = ARCHIVOS_PNG + ["reporte.md", "reporte.html", "resumen.json", "reporte.pdf"]


# ---------------------------------------------------------------------------
# Lectura
# ---------------------------------------------------------------------------
def _num(s) -> float:
    try:
        v = float(s)
    except (TypeError, ValueError):
        return math.nan
    return v if math.isfinite(v) else math.nan


def _int(s) -> Optional[int]:
    v = _num(s)
    return None if math.isnan(v) else int(v)


def leer_registros(path: str) -> Tuple[List[dict], int]:
    """Lee un CSV de sesión/log o un volcado DUMP (hex). Devuelve (registros, descartados).

    Cada registro es un dict con t_ms, estado, secuencia, banderas y los campos numéricos
    (NaN si faltan). Las filas que no se pueden interpretar se cuentan y se saltan."""
    with open(path, encoding="utf-8-sig", errors="replace", newline="") as f:
        lines = f.read().splitlines()
    first = next((ln for ln in lines if ln.strip()), "")
    head = [c.strip().lower() for c in first.split(",")]
    if "t_ms" in head:
        return _leer_csv(lines)
    return _leer_hex(lines)


def _leer_csv(lines: List[str]) -> Tuple[List[dict], int]:
    out, bad = [], 0
    rd = csv.DictReader(lines)
    rd.fieldnames = [c.strip().lower() for c in (rd.fieldnames or [])]
    for row in rd:
        t_ms, est = _int(row.get("t_ms")), _int(row.get("estado"))
        if t_ms is None or t_ms < 0 or (est is not None and not 0 <= est < len(ESTADOS)):
            bad += 1
            continue
        r = {"t_ms": t_ms, "estado": est, "secuencia": _int(row.get("secuencia")),
             "banderas": _int(row.get("banderas")) or 0,
             "hora_pc": (row.get("hora_pc") or "").strip()}
        for k in CAMPOS_NUM:
            r[k] = _num(row.get(k))
        out.append(r)
    return out, bad


def _leer_hex(lines: List[str]) -> Tuple[List[dict], int]:
    """Volcado DUMP (una línea hex por registro) o crudo de la estación (HEX,RSSI,SNR)."""
    out, bad = [], 0
    for ln in lines:
        s = ln.strip()
        if not s or s.startswith("#") or s.startswith("LOG_") or s.startswith("OK"):
            continue
        parts = s.split(",")
        if not _HEX64.match(parts[0]):
            # Hex de otra longitud = registro dañado; texto del monitor serial = se ignora.
            bad += bool(re.fullmatch(r"[0-9A-Fa-f]{8,}", parts[0]))
            continue
        data = bytes.fromhex(parts[0])
        if data[0] == T.LOG_EVENT_MARK:          # registro de evento: no es un paquete
            continue
        p = T.decode(data)
        if p.version != T.VERSION or p.estado >= len(ESTADOS):
            bad += 1
            continue
        r = {"t_ms": p.t_ms, "estado": p.estado, "secuencia": p.secuencia,
             "banderas": p.banderas, "hora_pc": ""}
        for k in CAMPOS_NUM:
            r[k] = float(getattr(p, k)) if hasattr(p, k) else math.nan
        if len(parts) == 3:
            r["rssi_dbm"], r["snr_db"] = _num(parts[1]), _num(parts[2])
        out.append(r)
    return out, bad


# ---------------------------------------------------------------------------
# Datos limpios
# ---------------------------------------------------------------------------
@dataclass
class Datos:
    t: np.ndarray                     # s de misión (reloj de la placa, continúa si reinicia)
    t_ms: np.ndarray
    estado: np.ndarray                # -1 = desconocido
    banderas: np.ndarray
    secuencia: List[Optional[int]]
    v: Dict[str, np.ndarray]          # campos numéricos ya limpios (NaN = sin dato)
    hora: List[str]
    calidad: Dict[str, int] = field(default_factory=dict)
    saltos: np.ndarray = None         # índices de altura descartada (para la gráfica)

    @property
    def n(self) -> int:
        return len(self.t)

    def __getitem__(self, k: str) -> np.ndarray:
        return self.v[k]


def _hampel(x: np.ndarray, umbral: float, medio: int = 2) -> np.ndarray:
    """Máscara de picos aislados: se aparta más de umbral de la mediana de su ventana."""
    bad = np.zeros(len(x), bool)
    for i in range(len(x)):
        if not math.isfinite(x[i]):
            continue
        w = x[max(0, i - medio):i + medio + 1]
        w = w[np.isfinite(w)]
        if len(w) >= 3 and abs(x[i] - np.median(w)) > umbral:
            bad[i] = True
    return bad


def _saltos(t: np.ndarray, h: np.ndarray, vmax: float = SALTO_V, margen: float = SALTO_MARGEN,
            n_reanclar: int = 3) -> np.ndarray:
    """Máscara de alturas que se alejan de la última buena más de lo físicamente posible.

    Si aparecen n_reanclar muestras seguidas coherentes entre sí, la equivocada era la
    referencia: se aceptan y se sigue desde ellas (p. ej. tras un hueco largo)."""
    bad = np.zeros(len(h), bool)
    ancla = None
    racha: List[int] = []

    def cerca(i, j):
        return abs(h[i] - h[j]) <= vmax * max(0.0, t[i] - t[j]) + margen

    for i in np.flatnonzero(np.isfinite(h)):
        if ancla is None or cerca(i, ancla):
            ancla, racha = i, []
            continue
        racha = racha + [i] if racha and cerca(i, racha[-1]) else [i]
        bad[i] = True
        if len(racha) >= n_reanclar:
            bad[racha] = False
            ancla, racha = racha[-1], []
    return bad


def construir(registros: List[dict]) -> Datos:
    """Convierte registros en arreglos, quita duplicados y marca como NaN lo que no sirve."""
    regs, dups, prev = [], 0, None
    for r in registros:
        key = (r.get("secuencia"), r["t_ms"])
        if prev is not None and key == prev:
            dups += 1
            continue
        prev = key
        regs.append(r)

    # Tiempo de misión como la estación: si t_ms retrocede (reinicio) el tiempo continúa.
    t, base, last, mis, reinicios = [], None, None, 0, 0
    for r in regs:
        tm = r["t_ms"]
        if base is None:
            base = tm
        elif tm + 1000 < last:
            base, reinicios = tm - mis, reinicios + 1
        last = tm
        mis = max(0, tm - base)
        t.append(mis / 1000)

    def col(k):
        return np.array([r.get(k, math.nan) for r in regs], float)

    v = {k: col(k) for k in CAMPOS_NUM}
    est = np.array([-1 if r.get("estado") is None else r["estado"] for r in regs], int)
    fl = np.array([r.get("banderas") or 0 for r in regs], int)
    t = np.array(t, float)
    q = {"filas": len(regs), "duplicados": dups, "reinicios_placa": reinicios}

    def anular(mask, keys, motivo):
        mask = mask & np.isfinite(v[keys[0]])
        if mask.any():
            q[motivo] = q.get(motivo, 0) + int(mask.sum())
            for k in keys:
                v[k][mask] = math.nan

    baro, imu = ["presion_pa", "temp_c"], ["ax_g", "ay_g", "az_g", "roll_deg", "pitch_deg",
                                           "yaw_deg"]
    with np.errstate(invalid="ignore"):
        anular((fl & T.F_FALLO_BARO) != 0, baro, "baro_con_falla")
        anular((v["presion_pa"] < 30000) | (v["presion_pa"] > 110000), baro, "presion_invalida")
        anular((v["temp_c"] < -40) | (v["temp_c"] > 85), ["temp_c"], "temp_invalida")
        zeros = (v["ax_g"] == 0) & (v["ay_g"] == 0) & (v["az_g"] == 0)
        anular(((fl & T.F_FALLO_IMU) != 0) | zeros, imu, "imu_sin_dato")
        for k in ("ax_g", "ay_g", "az_g"):
            anular(np.abs(v[k]) > 32, [k], "fuera_de_rango")
        for k in ("roll_deg", "pitch_deg", "yaw_deg"):
            anular(np.abs(v[k]) > 180.5, [k], "fuera_de_rango")
        anular(np.abs(v["vz_ms"]) > VZ_FISICA, ["vz_ms", "altura_m"], "estimador_fuera_de_rango")
        anular((v["altura_m"] < -20) | (v["altura_m"] > 3200), ["altura_m"], "fuera_de_rango")
        anular((v["rssi_dbm"] >= 0) | (v["rssi_dbm"] < -160), ["rssi_dbm"], "rssi_invalido")
        anular(np.abs(v["snr_db"]) > 40, ["snr_db"], "snr_invalido")
        v["bateria_v"][v["bateria_v"] <= 0] = math.nan      # 0 = la placa no la mide
        v["epoch_pc"][v["epoch_pc"] <= 0] = math.nan
    h = v["altura_m"]
    bad = _hampel(h, HAMPEL_M)
    h2 = h.copy()
    h2[bad] = math.nan
    bad |= _saltos(t, h2)
    saltos = np.flatnonzero(bad)
    if len(saltos):
        q["saltos_de_altura"] = len(saltos)
        h_crudo = h.copy()
        h[bad] = math.nan
        v["altura_cruda"] = h_crudo
    else:
        v["altura_cruda"] = h.copy()
    return Datos(t=t, t_ms=np.array([r["t_ms"] for r in regs], float), estado=est,
                 banderas=fl, secuencia=[r.get("secuencia") for r in regs], v=v,
                 hora=[r.get("hora_pc", "") for r in regs], calidad=q, saltos=saltos)


def cargar(path: str) -> Datos:
    regs, bad = leer_registros(path)
    d = construir(regs)
    d.calidad["filas_ilegibles"] = bad
    return d


# ---------------------------------------------------------------------------
# Cálculos
# ---------------------------------------------------------------------------
def enlace(secuencias: List[Optional[int]]) -> Optional[dict]:
    """Recibidos/esperados por secuencia (u16 con vuelta), como tools/resumen_enlace.py.

    Devuelve también perdidos_antes[i]: paquetes que faltaron justo antes del i-ésimo."""
    idx = [i for i, s in enumerate(secuencias) if s is not None]
    if not idx:
        return None
    perd = [0] * len(secuencias)
    rx = esperados = 1
    for a, b in zip(idx, idx[1:]):
        gap = (secuencias[b] - secuencias[a]) & 0xFFFF
        if gap == 0:
            continue
        rx += 1
        if gap > SEQ_RESTART_GAP:
            esperados += 1
        else:
            esperados += gap
            perd[b] = gap - 1
    return {"recibidos": rx, "esperados": esperados, "perdidos": esperados - rx,
            "perdida_pct": 100.0 * (esperados - rx) / esperados, "perdidos_antes": perd}


def huecos(t: np.ndarray, minimo: float = HUECO_S) -> List[Tuple[float, float]]:
    """Intervalos (inicio, fin) sin paquetes de más de `minimo` segundos."""
    if len(t) < 2:
        return []
    dt = np.diff(t)
    return [(float(t[i]), float(t[i + 1])) for i in np.flatnonzero(dt > minimo)]


def _primero(cond: np.ndarray, desde: int = 0) -> Optional[int]:
    """Primer índice >= desde donde cond se cumple y también en la muestra siguiente
    (o es la última): un paquete suelto con un estado raro no cuenta."""
    n = len(cond)
    for i in range(desde, n):
        if cond[i] and (i == n - 1 or cond[i + 1]):
            return i
    return None


def _ventana(d: Datos, a: float, b: float, k: str) -> Tuple[np.ndarray, np.ndarray]:
    m = (d.t >= a) & (d.t <= b) & np.isfinite(d[k])
    return d.t[m], d[k][m]


def _modulo(d: Datos) -> np.ndarray:
    return np.sqrt(d["ax_g"] ** 2 + d["ay_g"] ** 2 + d["az_g"] ** 2)


def _evento(d: Datos, i: Optional[int]) -> Optional[dict]:
    if i is None:
        return None
    h = d["altura_m"][i]
    if not math.isfinite(h):                      # altura descartada: la más cercana buena
        ok = np.flatnonzero(np.isfinite(d["altura_m"]))
        h = float(d["altura_m"][ok[np.argmin(np.abs(ok - i))]]) if len(ok) else math.nan
    return {"t": float(d.t[i]), "altura_m": _f(h), "i": int(i)}


def _f(x) -> Optional[float]:
    return None if x is None or not math.isfinite(float(x)) else float(x)


def analizar(d: Datos) -> dict:
    """Todos los números del reporte. Lo que no se puede calcular queda en None."""
    r: dict = {"n": d.n, "calidad": dict(d.calidad), "notas": []}
    if d.n == 0:
        r["notas"].append("La sesión no tiene paquetes.")
        return r
    h, vz, est, fl = d["altura_m"], d["vz_ms"], d.estado, d.banderas
    r["sesion_s"] = float(d.t[-1] - d.t[0])

    # --- altura máxima ---
    ok = np.isfinite(h)
    if ok.any():
        i = int(np.flatnonzero(ok)[np.argmax(h[ok])])
        r["h_max"] = {"t": float(d.t[i]), "altura_m": float(h[i]), "i": i}

    # --- eventos por estado y banderas ---
    i_asc = _primero(est >= T.ASCENSO)
    i_lib = _primero((est >= T.CAIDA) | ((fl & T.F_LIBERACION) != 0))
    i_disp = _primero((est >= T.DESPLIEGUE) | ((fl & T.F_DISPARO) != 0))
    i_desc = _primero(est >= T.DESCENSO)
    i_ater = _primero((est >= T.ATERRIZADO) | ((fl & T.F_ATERRIZADO) != 0))

    if i_asc is not None:      # despegue: última muestra en el suelo antes de ASCENSO o en él
        j = i_asc
        while j >= 0 and not (math.isfinite(h[j]) and h[j] <= H_SOBRE_SUELO):
            j -= 1
        r["despegue"] = _evento(d, j if j >= 0 else i_asc)
    r["liberacion"] = _evento(d, i_lib)
    r["despliegue"] = _evento(d, i_disp)
    if i_desc is not None:
        r["fin_despliegue"] = _evento(d, i_desc)
    elif i_disp is not None:                      # sin paquete de DESCENSO: 1 s después
        k = np.flatnonzero(d.t >= d.t[i_disp] + 1.0)
        if len(k):
            r["fin_despliegue"] = _evento(d, int(k[0]))
            r["notas"].append("No llegó ningún paquete de DESCENSO: el fin del despliegue "
                              "se toma 1 s después del disparo.")
    r["estado_aterrizado"] = _evento(d, i_ater)

    # --- aterrizaje (toque) ---
    if i_ater is not None:
        hs = h[(est == T.ATERRIZADO) & np.isfinite(h)][:50]
        suelo = float(np.median(hs)) if len(hs) else 0.0
    else:
        suelo = 0.0
    r["suelo_m"] = suelo
    # Referencia para buscar el toque: el descenso o, sin eventos, la altura máxima (vuelo
    # sin liberación detectada, p. ej. soltado por debajo de la altura mínima).
    ref = next((x for x in (i_desc, i_disp, i_lib) if x is not None), None)
    if ref is None and "h_max" in r and r["h_max"]["altura_m"] > 10:
        ref = r["h_max"]["i"]
    if ref is not None:
        cand = np.flatnonzero(np.isfinite(h) & (h <= suelo + H_SOBRE_SUELO)
                              & (np.arange(d.n) > ref))
        if len(cand):
            ic, metodo = int(cand[0]), "altura a menos de 2 m del suelo"
        elif i_ater is not None:
            ic, metodo = i_ater, "estado ATERRIZADO"
        else:
            ic, metodo = None, None
        if ic is not None:
            tc = d.t[ic]
            a = _modulo(d)
            m = (d.t >= max(tc - 3.0, d.t[ref])) & (d.t <= tc + 1.5) & np.isfinite(a)
            if m.any() and a[m].max() >= IMPACTO_MIN_G:
                ic, metodo = int(np.flatnonzero(m)[np.argmax(a[m])]), "pico de aceleración"
            r["aterrizaje"] = dict(_evento(d, ic), metodo=metodo)
    if ref is not None and "aterrizaje" not in r:
        r["notas"].append("No se encontró el aterrizaje (la sesión termina antes o el "
                          "enlace se perdió).")

    t_toque = r["aterrizaje"]["t"] if r.get("aterrizaje") else None
    lib, fin = r.get("liberacion"), r.get("fin_despliegue")
    if lib and "h_max" in r and r["h_max"]["t"] > lib["t"] + 5:
        r["notas"].append("La altura máxima aparece después de la liberación: la altura "
                          "estimada por la placa no es coherente en esta sesión; revisar los datos.")

    # --- duraciones ---
    if t_toque is not None and r.get("despegue"):
        r["duracion_mision_s"] = t_toque - r["despegue"]["t"]
    if t_toque is not None and fin:
        r["tiempo_descenso_s"] = t_toque - fin["t"]
    if t_toque is not None and lib:
        r["tiempo_caida_total_s"] = t_toque - lib["t"]

    # --- velocidad vertical máxima en caída ---
    a0 = lib["t"] if lib else (r["h_max"]["t"] if "h_max" in r and r["h_max"]["altura_m"] > 10
                               else None)
    b0 = fin["t"] if fin else t_toque
    if a0 is not None:
        tv, vv = _ventana(d, a0, b0 if b0 is not None else d.t[-1], "vz_ms")
        if len(vv) and vv.min() < 0:
            k = int(np.argmin(vv))
            r["vz_max_caida"] = {"t": float(tv[k]), "vz_ms": float(-vv[k])}

    # --- descenso con autogiro ---
    if fin and t_toque is not None and t_toque > fin["t"]:
        m = (d.t >= fin["t"]) & (d.t < t_toque) & np.isfinite(vz)
        vel = -vz[m]
        if len(vel) >= 3:
            lo, hi = BANDA_DESCENSO
            media, mediana = float(np.mean(vel)), float(np.median(vel))
            dh = (fin["altura_m"] - suelo) if fin["altura_m"] is not None else None
            r["descenso"] = {
                "media_ms": media, "mediana_ms": mediana, "n": int(len(vel)),
                "min_ms": float(vel.min()), "max_ms": float(vel.max()),
                "pct_en_banda": 100.0 * float(np.mean((vel >= lo) & (vel <= hi))),
                "media_altura_tiempo_ms": (dh / (t_toque - fin["t"])) if dh is not None else None,
                "en_banda": bool(lo <= media <= hi and lo <= mediana <= hi),
            }
        else:
            r["notas"].append("Muy pocas muestras de vz en el descenso con autogiro.")

    # --- impacto ---
    if t_toque is not None:
        ti, ai = d.t, _modulo(d)
        m = (ti >= t_toque - 1.0) & (ti <= t_toque + 2.0) & np.isfinite(ai)
        if m.any():
            k = int(np.flatnonzero(m)[np.argmax(ai[m])])
            r["impacto"] = {"t": float(ti[k]), "g": float(ai[k])}

    # --- transmisión tras aterrizar ---
    if t_toque is not None:
        post = float(d.t[-1] - t_toque)
        gaps = [g for g in huecos(d.t) if g[1] > t_toque]
        r["post_aterrizaje"] = {"s": post, "cumple": post >= T_POST_ATERRIZAJE,
                                "hueco_max_s": max((b - a for a, b in gaps), default=0.0)}

    # --- enlace ---
    e = enlace(d.secuencia)
    if e is not None:
        r["enlace"] = {k: v for k, v in e.items() if k != "perdidos_antes"}
    for k, nombre in (("rssi_dbm", "rssi"), ("snr_db", "snr")):
        x = d[k][np.isfinite(d[k])]
        if len(x):
            r[nombre] = {"prom": float(np.mean(x)), "min": float(np.min(x)),
                         "max": float(np.max(x))}
    g = huecos(d.t)
    r["huecos"] = {"n": len(g), "max_s": max((b - a for a, b in g), default=0.0),
                   "total_s": sum(b - a for a, b in g)}
    return r


# ---------------------------------------------------------------------------
# Log interno (DUMP)
# ---------------------------------------------------------------------------
def alinear_log(est: Datos, log: Datos) -> float:
    """Desfase (s) que se suma al tiempo del log para llevarlo al de la estación.

    Usa los t_ms que aparecen en ambos (el log guarda los mismos paquetes que se
    transmiten); si no hay ninguno, supone el mismo arranque de la placa."""
    if est.n == 0 or log.n == 0:
        return 0.0
    pos = {}
    for i, tm in enumerate(est.t_ms):
        pos.setdefault(tm, i)
    dif = [est.t[pos[tm]] - log.t[j] for j, tm in enumerate(log.t_ms) if tm in pos]
    if dif:
        return float(np.median(dif))
    return float((log.t_ms[0] - est.t_ms[0]) / 1000.0)


def no_recibido(est: Datos, t_log: np.ndarray) -> np.ndarray:
    """Máscara de registros del log que caen donde la estación no recibió nada: antes del
    primer paquete, después del último, en un hueco o entre dos paquetes con pérdidas."""
    if est.n == 0:
        return np.ones(len(t_log), bool)
    m = (t_log < est.t[0]) | (t_log > est.t[-1])
    e = enlace(est.secuencia)
    perd = e["perdidos_antes"] if e else [0] * est.n
    for i in range(1, est.n):
        if est.t[i] - est.t[i - 1] > HUECO_S or perd[i] > 0:
            m |= (t_log > est.t[i - 1]) & (t_log < est.t[i])
    return m


def tramos(t: np.ndarray, mask: np.ndarray, unir: float = 0.25) -> List[Tuple[float, float]]:
    """Intervalos contiguos donde mask es verdadera (une los separados menos de `unir` s,
    p. ej. por un solo paquete recibido en medio)."""
    out, a = [], None
    for i in range(len(t)):
        if mask[i] and a is None:
            a = t[i]
        if a is not None and (not mask[i] or i == len(t) - 1):
            b = float(t[i] if mask[i] else t[i - 1])
            if out and a - out[-1][1] <= unir:
                out[-1] = (out[-1][0], b)
            else:
                out.append((float(a), b))
            a = None
    return out


# ---------------------------------------------------------------------------
# Formato
# ---------------------------------------------------------------------------
class Reloj:
    """Hora de la PC para un tiempo de misión: la del paquete más cercano más la diferencia."""

    def __init__(self, d: Datos):
        ok = np.isfinite(d["epoch_pc"])
        self.t, self.e = d.t[ok], d["epoch_pc"][ok]
        self.hora = [h for h, k in zip(d.hora, ok) if k]

    def __call__(self, t: Optional[float], dec: int = 1) -> str:
        if t is None or not len(self.t):
            return "—"
        k = int(np.argmin(np.abs(self.t - t)))
        s = datetime.fromtimestamp(self.e[k] + (t - self.t[k])).strftime("%H:%M:%S.%f")
        return s[: 9 + dec] if dec else s[:8]

    def fecha(self) -> str:
        if not len(self.e):
            return ""
        return datetime.fromtimestamp(self.e[0]).strftime("%d/%m/%Y")


def _n(x, dec=1, unidad="") -> str:
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "—"
    s = f"{x:.{dec}f}"
    if re.fullmatch(r"-0\.?0*", s):
        s = s[1:]
    return s + (f" {unidad}" if unidad else "")


def _dur(s) -> str:
    if s is None:
        return "—"
    m, r = divmod(max(0.0, s), 60)
    return f"{s:.1f} s ({int(m)} min {r:04.1f} s)" if s >= 60 else f"{s:.1f} s"


def filas_tabla(r: dict, reloj: Reloj) -> List[Tuple[str, str]]:
    """(concepto, valor) de la tabla principal."""
    def ev(e):
        if not e:
            return "no detectada"
        return f"{reloj(e['t'])} · {_n(e['altura_m'], 1, 'm')} (T+{e['t']:.1f} s)"

    hm = r.get("h_max")
    d = r.get("descenso")
    en = r.get("enlace")
    post = r.get("post_aterrizaje")
    rs, sn = r.get("rssi"), r.get("snr")
    lo, hi = BANDA_DESCENSO
    if d:
        banda = ("Sí" if d["en_banda"] else "No") + f" ({d['pct_en_banda']:.0f} % de las muestras en {lo:g} a {hi:g} m/s)"
    else:
        banda = "—"
    imp = r.get("impacto")
    at = r.get("aterrizaje")
    filas = [
        ("Duración de la misión (despegue → aterrizaje)", _dur(r.get("duracion_mision_s"))),
        ("Altura máxima", f"{_n(hm['altura_m'], 1, 'm')} a las {reloj(hm['t'])} (T+{hm['t']:.1f} s)" if hm else "—"),
        ("Liberación: hora · altura", ev(r.get("liberacion"))),
        ("Despliegue del autogiro: hora · altura", ev(r.get("despliegue"))),
        ("Velocidad vertical máxima en caída", _n(r["vz_max_caida"]["vz_ms"], 1, "m/s") + f" (T+{r['vz_max_caida']['t']:.1f} s)" if r.get("vz_max_caida") else "—"),
        ("Descenso con autogiro: velocidad media", _n(d["media_ms"], 2, "m/s") if d else "—"),
        ("Descenso con autogiro: velocidad mediana", _n(d["mediana_ms"], 2, "m/s") if d else "—"),
        (f"¿Descenso dentro de {lo:g} a {hi:g} m/s?", banda),
        ("Tiempo de descenso con autogiro (fin del despliegue → aterrizaje)", _dur(r.get("tiempo_descenso_s"))),
        ("Tiempo desde la liberación hasta el aterrizaje", _dur(r.get("tiempo_caida_total_s"))),
        ("Aterrizaje: hora", f"{reloj(at['t'])} (T+{at['t']:.1f} s, por {at['metodo']})" if at else "no detectado"),
        ("Impacto (aceleración máxima al aterrizar)", _n(imp["g"], 2, "g") if imp else "—"),
        ("Paquetes recibidos / esperados", f"{en['recibidos']} / {en['esperados']}" if en else "—"),
        ("Pérdida de paquetes", _n(en["perdida_pct"], 1, "%") + f" ({en['perdidos']} perdidos)" if en else "—"),
        ("RSSI promedio (mínimo)", f"{_n(rs['prom'], 1, 'dBm')} ({_n(rs['min'], 1)})" if rs else "—"),
        ("SNR promedio (mínimo)", f"{_n(sn['prom'], 2, 'dB')} ({_n(sn['min'], 2)})" if sn else "—"),
        (f"¿Siguió transmitiendo ≥ {T_POST_ATERRIZAJE:g} s después de aterrizar?",
         (("Sí" if post["cumple"] else "No") + f", {post['s']:.1f} s") if post else "—"),
    ]
    return filas


def filas_log(r: dict, reloj: Reloj, desfase: float) -> List[str]:
    """La columna del log interno, alineada con filas_tabla (sin datos de enlace)."""
    def ev(e):
        return f"{reloj(e['t'] + desfase)} · {_n(e['altura_m'], 1, 'm')}" if e else "no detectada"

    hm, d, imp = r.get("h_max"), r.get("descenso"), r.get("impacto")
    at = r.get("aterrizaje")
    return [
        _dur(r.get("duracion_mision_s")),
        _n(hm["altura_m"], 1, "m") if hm else "—",
        ev(r.get("liberacion")), ev(r.get("despliegue")),
        _n(r["vz_max_caida"]["vz_ms"], 1, "m/s") if r.get("vz_max_caida") else "—",
        _n(d["media_ms"], 2, "m/s") if d else "—",
        _n(d["mediana_ms"], 2, "m/s") if d else "—",
        ("Sí" if d["en_banda"] else "No") if d else "—",
        _dur(r.get("tiempo_descenso_s")), _dur(r.get("tiempo_caida_total_s")),
        f"{reloj(at['t'] + desfase)}" if at else "no detectado",
        _n(imp["g"], 2, "g") if imp else "—",
        "", "", "", "", "",
    ]


def notas_calidad(d: Datos, r: dict) -> List[str]:
    q = d.calidad
    txt = {
        "filas_ilegibles": "filas ilegibles descartadas",
        "duplicados": "paquetes duplicados descartados",
        "reinicios_placa": "reinicios de la placa (t_ms retrocedió; el tiempo continúa)",
        "baro_con_falla": "muestras con bandera de fallo de barómetro (presión y temperatura no se grafican)",
        "presion_invalida": "presiones fuera de 300 a 1100 hPa (p. ej. 0 sin sensor)",
        "temp_invalida": "temperaturas fuera de -40 a 85 °C",
        "imu_sin_dato": "muestras sin IMU (bandera de fallo o aceleración 0,0,0)",
        "fuera_de_rango": "valores fuera del rango físico",
        "estimador_fuera_de_rango": f"muestras con |vz| > {VZ_FISICA:g} m/s (estimador corrompido; se descartan altura y vz)",
        "saltos_de_altura": "alturas descartadas por saltos imposibles (pico o salto > 80 m/s)",
        "rssi_invalido": "RSSI inválidos", "snr_invalido": "SNR inválidos",
    }
    out = [f"{q[k]} {t}" for k, t in txt.items() if q.get(k)]
    hu = r.get("huecos")
    if hu and hu["n"]:
        out.append(f"Huecos de enlace de más de {HUECO_S:g} s: {hu['n']} "
                   f"(el mayor de {hu['max_s']:.1f} s, {hu['total_s']:.1f} s en total); "
                   "sombreados con rayas en las gráficas")
    if r.get("impacto"):
        out.append("El impacto sale de la telemetría (2 a 5 paquetes por segundo): el pico "
                   "real puede ser mayor. El log interno (--log) lo mide mejor.")
    return out + r.get("notas", [])


# ---------------------------------------------------------------------------
# Gráficas
# ---------------------------------------------------------------------------
def _mpl():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 10, "axes.titlesize": 12,
        "axes.edgecolor": TINTA3, "axes.labelcolor": TINTA2, "xtick.color": TINTA2,
        "ytick.color": TINTA2, "axes.grid": True, "grid.color": "#e6e5e1",
        "grid.linewidth": 0.7, "axes.spines.top": False, "axes.spines.right": False,
        "legend.frameon": False, "figure.facecolor": "white", "axes.facecolor": "white",
        "lines.linewidth": 1.6, "savefig.facecolor": "white",
    })
    return plt


def _segmentos(d: Datos) -> List[Tuple[float, float, int]]:
    """(inicio, fin, estado) contiguos; cada muestra vale hasta la siguiente."""
    seg = []
    for i in range(d.n):
        a = d.t[i]
        b = d.t[i + 1] if i + 1 < d.n else d.t[i]
        if seg and seg[-1][2] == d.estado[i]:
            seg[-1][1] = b
        else:
            seg.append([a, b, int(d.estado[i])])
    return [tuple(s) for s in seg]


def _fondo(ax, d: Datos, gaps, etiquetas=True):
    for a, b, e in _segmentos(d):
        if e < 0 or b <= a:
            continue
        ax.axvspan(a, b, color=COLOR_FASE[e], lw=0, zorder=0)
    for a, b in gaps:
        ax.axvspan(a, b, facecolor="none", edgecolor="#b9b8b2", hatch="////", lw=0, zorder=0.5)
    if etiquetas and d.n > 1:
        span = d.t[-1] - d.t[0] or 1
        for a, b, e in _segmentos(d):
            if e >= 0 and (b - a) / span > 0.045:
                ax.text((a + b) / 2, 0.985, ESTADOS[e], transform=ax.get_xaxis_transform(),
                        ha="center", va="top", fontsize=7.5, color=TINTA2, zorder=5)
    ax.set_xlim(d.t[0], d.t[-1] if d.t[-1] > d.t[0] else d.t[0] + 1)


def _leyenda_fases(fig, d: Datos, gaps, extra=None):
    from matplotlib.patches import Patch
    vistos = sorted({e for e in d.estado if e >= 0})
    hs = [Patch(facecolor=COLOR_FASE[e], label=ESTADOS[e]) for e in vistos]
    if gaps:
        hs.append(Patch(facecolor="white", edgecolor="#b9b8b2", hatch="////",
                        label="sin paquetes (hueco)"))
    hs += extra or []
    fig.legend(handles=hs, loc="lower center", ncol=min(len(hs), 11), fontsize=8,
               bbox_to_anchor=(0.5, 0.005), handlelength=1.6, columnspacing=1.2)


def _encabezado(fig, titulo, sesion, logo):
    h = fig.get_figheight()
    fig.text(0.012, 1 - 0.12 / h, EQUIPO.upper(), fontsize=13, fontweight="bold", color=TINTA,
             va="top")
    fig.text(0.012, 1 - 0.36 / h, f"{titulo}  ·  {sesion}", fontsize=10, color=TINTA2, va="top")
    if logo is not None:
        ax = fig.add_axes([0.93, 1 - 0.62 / h, 0.06, 0.56 / h])
        ax.imshow(logo)
        ax.axis("off")


def _lineas_t(t, y, salto=None):
    """Inserta NaN donde hay un hueco (o un salto de yaw) para no unir con una recta."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    if len(t) < 2:
        return t, y
    corte = np.diff(t) > HUECO_S
    if salto is not None:
        corte |= np.abs(np.diff(y)) > salto
    idx = np.flatnonzero(corte) + 1
    return np.insert(t, idx, np.nan), np.insert(y, idx, np.nan)


def _anotar(ax, x, y, texto, dy, color=TINTA, marker="o", dx=8, nivel=None):
    """Marca un punto con una etiqueta; dx < 0 la pone a la izquierda. Con nivel (0 a 1)
    la etiqueta va a esa altura de la gráfica, un poco a la derecha del punto: así no se
    enciman eventos cercanos en el tiempo."""
    if x is None or y is None:
        return
    ax.plot([x], [y], marker=marker, ms=8, color=color, mec="white", mew=1.5, zorder=6)
    if nivel is not None:
        x0, x1 = ax.get_xlim()
        ax.annotate(texto, (x, y), xytext=(x + 0.015 * (x1 - x0), nivel),
                    textcoords=("data", "axes fraction"), fontsize=8.5, color=TINTA, zorder=7,
                    ha="left", va="center",
                    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#d5d4cf", lw=0.6),
                    arrowprops=dict(arrowstyle="-", color=TINTA3, lw=0.7))
        return
    ax.annotate(texto, (x, y), xytext=(dx, dy), textcoords="offset points", fontsize=8.5,
                color=TINTA, zorder=7, ha="left" if dx >= 0 else "right",
                va="center" if abs(dy) < 12 else "bottom" if dy > 0 else "top",
                bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#d5d4cf", lw=0.6),
                arrowprops=dict(arrowstyle="-", color=TINTA3, lw=0.7))


def graficas(d: Datos, r: dict, carpeta: str, sesion: str, reloj: Reloj, dpi: int = 200,
             log: Optional[Datos] = None, desfase: float = 0.0,
             log_nr: Optional[np.ndarray] = None) -> List[str]:
    plt = _mpl()
    from matplotlib.lines import Line2D
    try:
        logo = plt.imread(LOGO) if os.path.isfile(LOGO) else None
    except (OSError, ValueError, SyntaxError):
        logo = None
    gaps = huecos(d.t)
    t = d.t
    tl = log.t + desfase if log is not None else None
    hechos = []

    def figura(titulo, filas=1, alturas=None):
        alto = 3.0 + 2.6 * filas
        fig, axs = plt.subplots(filas, 1, figsize=(13, alto), sharex=True,
                                squeeze=False, gridspec_kw={"height_ratios": alturas} if alturas else None)
        fig.subplots_adjust(left=0.07, right=0.985, top=1 - 0.78 / alto, bottom=0.78 / alto,
                            hspace=0.12)
        _encabezado(fig, titulo, sesion, logo)
        for ax in axs[:, 0]:
            _fondo(ax, d, gaps, etiquetas=ax is axs[0, 0])
        axs[-1, 0].set_xlabel("Tiempo de misión (s, reloj de la placa desde el primer paquete)")
        return fig, [a for a in axs[:, 0]]

    def guardar(fig, nombre, extra=None):
        _leyenda_fases(fig, d, gaps, extra)
        path = os.path.join(carpeta, nombre)
        fig.savefig(path, dpi=dpi)
        plt.close(fig)
        hechos.append(path)

    lib, disp, at, hm = r.get("liberacion"), r.get("despliegue"), r.get("aterrizaje"), r.get("h_max")
    extra_log = []
    if log is not None:
        extra_log = [Line2D([], [], color=TINTA3, lw=1.0, label="log interno"),
                     Line2D([], [], color=C8, lw=3.2, alpha=0.75,
                            label="log: no recibido en la estación")]

    def capa_log(ax, k):
        if log is None:
            return
        y = log[k]
        ax.plot(*_lineas_t(tl, y), color=TINTA3, lw=1.0, zorder=2)
        if log_nr is not None and log_nr.any():
            ax.plot(tl, np.where(log_nr, y, np.nan), color=C8, lw=3.2, alpha=0.75, zorder=3,
                    solid_capstyle="butt")

    # 1. Altura
    fig, (ax,) = figura("Altura vs tiempo")
    capa_log(ax, "altura_m")
    ax.plot(*_lineas_t(t, d["altura_m"]), color=C1, lw=2, zorder=4, label="altura")
    if len(d.saltos):
        ax.plot(t[d.saltos], np.clip(d["altura_cruda"][d.saltos], *_lims(d["altura_m"])),
                "x", color=TINTA3, ms=5, zorder=3)
    ax.set_ylabel("Altura sobre el suelo (m)")
    ax.margins(y=0.1)
    if hm:
        _anotar(ax, hm["t"], hm["altura_m"], f"Altura máxima {hm['altura_m']:.1f} m\n{reloj(hm['t'])}",
                -6, C1, "^", dx=-14)
    junto = lib and disp and lib["i"] == disp["i"]
    if lib:
        ax.axvline(lib["t"], color=TINTA2, ls="--", lw=1, zorder=3)
        nom = "Liberación y despliegue" if junto else "Liberación"
        _anotar(ax, lib["t"], lib["altura_m"], f"{nom} {_n(lib['altura_m'], 1, 'm')}\n{reloj(lib['t'])}",
                0, TINTA2, "v", nivel=0.80)
    if disp and not junto:
        ax.axvline(disp["t"], color=C2, ls="--", lw=1, zorder=3)
        _anotar(ax, disp["t"], disp["altura_m"], f"Despliegue {_n(disp['altura_m'], 1, 'm')}\n{reloj(disp['t'])}",
                0, C2, "D", nivel=0.60)
    if at:
        _anotar(ax, at["t"], at["altura_m"], f"Aterrizaje\n{reloj(at['t'])}", 0, C3, "s", nivel=0.32)
    extra = [Line2D([], [], color=C1, lw=2, label="altura (estación)")]
    if len(d.saltos):
        extra.append(Line2D([], [], color=TINTA3, marker="x", lw=0, label="dato descartado"))
    guardar(fig, ARCHIVOS_PNG[0], extra + extra_log)

    # 2. Velocidad vertical
    fig, (ax,) = figura("Velocidad vertical vs tiempo")
    fin = r.get("fin_despliegue")
    lo, hi = BANDA_DESCENSO
    if fin and at:
        ax.fill_between([fin["t"], at["t"]], -hi, -lo, color=C3, alpha=0.22, lw=0, zorder=1)
        for y in (-lo, -hi):
            ax.plot([fin["t"], at["t"]], [y, y], color=C3, lw=0.8, ls=":", zorder=1)
    capa_log(ax, "vz_ms")
    ax.plot(*_lineas_t(t, d["vz_ms"]), color=C2, lw=1.8, zorder=4)
    ax.axhline(0, color=TINTA3, lw=0.8, zorder=1)
    ax.margins(y=0.1)
    ax.set_ylabel("Velocidad vertical (m/s, + = sube)")
    vm = r.get("vz_max_caida")
    if vm:
        _anotar(ax, vm["t"], -vm["vz_ms"], f"Máx. en caída {vm['vz_ms']:.1f} m/s", 0, C2, "v", dx=-14)
    ds = r.get("descenso")
    if ds and fin and at:
        ax.annotate(f"descenso con autogiro: media {ds['media_ms']:.2f} m/s · mediana {ds['mediana_ms']:.2f} m/s",
                    ((fin["t"] + at["t"]) / 2, -hi), xytext=(0, -16), textcoords="offset points",
                    ha="center", va="top", fontsize=8.5, color=TINTA, zorder=7,
                    bbox=dict(boxstyle="round,pad=0.25", fc="white", ec="#d5d4cf", lw=0.6))
    from matplotlib.patches import Patch
    guardar(fig, ARCHIVOS_PNG[1], [Line2D([], [], color=C2, lw=2, label="vz (estación)"),
                                   Patch(facecolor=C3, alpha=0.3, label=f"banda {lo:g} a {hi:g} m/s (descenso)")]
            + extra_log)

    # 3. Presión y temperatura (dos gráficas: unidades distintas)
    fig, (a1, a2) = figura("Presión y temperatura", filas=2)
    a1.plot(*_lineas_t(t, d["presion_pa"] / 100), color=C1, lw=1.8, zorder=4)
    a1.set_ylabel("Presión (hPa)")
    a2.plot(*_lineas_t(t, d["temp_c"]), color=C2, lw=1.8, zorder=4)
    a2.set_ylabel("Temperatura (°C)")
    for ax, k in ((a1, "presion_pa"), (a2, "temp_c")):
        if not np.isfinite(d[k]).any():
            ax.text(0.5, 0.5, "sin datos válidos", transform=ax.transAxes, ha="center",
                    color=TINTA3, fontsize=11)
    if log is not None:
        a1.plot(*_lineas_t(tl, log["presion_pa"] / 100), color=TINTA3, lw=1.0, zorder=2)
        a2.plot(*_lineas_t(tl, log["temp_c"]), color=TINTA3, lw=1.0, zorder=2)
    guardar(fig, ARCHIVOS_PNG[2], [Line2D([], [], color=C1, lw=2, label="presión"),
                                   Line2D([], [], color=C2, lw=2, label="temperatura")]
            + extra_log[:1])

    # 4. Aceleración
    fig, (ax,) = figura("Aceleración (3 ejes y módulo)")
    mod = _modulo(d)
    if log is not None:
        ax.plot(*_lineas_t(tl, _modulo(log)), color=TINTA3, lw=0.9, zorder=2)
    for k, c, lab in (("ax_g", C1, "ax"), ("ay_g", C2, "ay"), ("az_g", C3, "az")):
        ax.plot(*_lineas_t(t, d[k]), color=c, lw=1.3, zorder=3, label=lab)
    ax.plot(*_lineas_t(t, mod), color=TINTA, lw=1.8, zorder=4, label="|a|")
    ax.set_ylabel("Aceleración (g)")
    imp = r.get("impacto")
    if imp:
        _anotar(ax, imp["t"], imp["g"], f"Impacto {imp['g']:.2f} g", 10, TINTA, "o")
    if not np.isfinite(mod).any():
        ax.text(0.5, 0.5, "sin datos de IMU", transform=ax.transAxes, ha="center", color=TINTA3)
    guardar(fig, ARCHIVOS_PNG[3], [Line2D([], [], color=c, lw=2, label=lab) for c, lab in
                                   ((C1, "ax"), (C2, "ay"), (C3, "az"), (TINTA, "|a| módulo"))]
            + ([Line2D([], [], color=TINTA3, lw=1, label="|a| log interno")] if log is not None else []))

    # 5. Actitud
    fig, (ax,) = figura("Actitud (yaw relativo al arranque)")
    for k, c, lab in (("roll_deg", C1, "roll"), ("pitch_deg", C2, "pitch"), ("yaw_deg", C3, "yaw")):
        ax.plot(*_lineas_t(t, d[k], salto=180 if k == "yaw_deg" else None), color=c, lw=1.4,
                zorder=4)
    ax.set_ylabel("Ángulo (°)")
    ax.set_ylim(-185, 185)
    ax.set_yticks(range(-180, 181, 45))
    guardar(fig, ARCHIVOS_PNG[4], [Line2D([], [], color=c, lw=2, label=lab) for c, lab in
                                   ((C1, "roll"), (C2, "pitch"), (C3, "yaw"))])

    # 6. Enlace
    fig, (a1, a2, a3) = figura("Calidad del enlace", filas=3)
    a1.plot(*_lineas_t(t, d["rssi_dbm"]), color=C1, lw=1.4, zorder=4)
    a1.set_ylabel("RSSI (dBm)")
    a2.plot(*_lineas_t(t, d["snr_db"]), color=C2, lw=1.4, zorder=4)
    a2.set_ylabel("SNR (dB)")
    e = enlace(d.secuencia)
    if e and d.n:
        bins = np.arange(d.t[0], d.t[-1] + BIN_PERDIDAS_S, BIN_PERDIDAS_S)
        cnt, _ = np.histogram(d.t, bins=bins, weights=e["perdidos_antes"]) if len(bins) > 1 else (np.array([]), None)
        if len(cnt):
            a3.bar(bins[:-1], cnt, width=BIN_PERDIDAS_S * 0.85, align="edge", color=C8, zorder=4)
        a3.text(0.995, 0.92, f"{e['perdidos']} perdidos de {e['esperados']} ({e['perdida_pct']:.1f} %)",
                transform=a3.transAxes, ha="right", va="top", fontsize=9, color=TINTA)
    a3.set_ylabel(f"Perdidos / {BIN_PERDIDAS_S:g} s")
    from matplotlib.ticker import MaxNLocator
    a3.yaxis.set_major_locator(MaxNLocator(integer=True))
    top = a3.get_ylim()[1]
    a3.set_ylim(0, max(1.0, top * 1.15))
    guardar(fig, ARCHIVOS_PNG[5], [Line2D([], [], color=C1, lw=2, label="RSSI"),
                                   Line2D([], [], color=C2, lw=2, label="SNR"),
                                   Patch(facecolor=C8, label="paquetes perdidos")])
    return hechos


def _lims(x):
    x = x[np.isfinite(x)]
    if not len(x):
        return (-10, 10)
    pad = max(5.0, 0.08 * (x.max() - x.min()))
    return (x.min() - pad, x.max() + pad)


def pdf(carpeta: str, titulo: str, filas: List[Tuple[str, ...]], cab: List[str],
        notas: List[str]) -> str:
    """Dos hojas A4: tabla + altura + vz; luego las otras cuatro gráficas."""
    plt = _mpl()
    from matplotlib.backends.backend_pdf import PdfPages
    path = os.path.join(carpeta, "reporte.pdf")
    imgs = [os.path.join(carpeta, f) for f in ARCHIVOS_PNG]
    with PdfPages(path) as pp:
        fig = plt.figure(figsize=(8.27, 11.69))
        fig.text(0.06, 0.965, f"{EQUIPO.upper()} — Reporte de vuelo", fontsize=15,
                 fontweight="bold", color=TINTA, va="top")
        fig.text(0.06, 0.94, titulo, fontsize=9, color=TINTA2, va="top")
        ax = fig.add_axes([0.06, 0.50, 0.88, 0.42])
        ax.axis("off")
        import textwrap
        anchos = [50, 44] if len(cab) == 2 else [44, 30, 30]
        celdas = [[textwrap.fill(str(c), anchos[j]) for j, c in enumerate(f)] for f in filas]
        lineas = [max(c.count("\n") + 1 for c in f) for f in celdas]
        tb = ax.table(cellText=celdas, colLabels=cab, loc="upper center",
                      cellLoc="left", colLoc="left",
                      colWidths=[0.46] + [0.54 / (len(cab) - 1)] * (len(cab) - 1))
        tb.auto_set_font_size(False)
        tb.set_fontsize(7.2 if len(cab) == 2 else 6.4)
        alto = 0.95 / (len(filas) + 1 + sum(n - 1 for n in lineas) * 0.6)
        for (row, _c), cell in tb.get_celld().items():
            cell.set_edgecolor("#d5d4cf")
            cell.set_height(alto * (1 + 0.6 * (lineas[row - 1] - 1)) if row else alto)
            if row == 0:
                cell.set_text_props(fontweight="bold")
                cell.set_facecolor("#f1f0ec")
        for k, img in enumerate(imgs[:2]):
            a = fig.add_axes([0.03, 0.255 - 0.235 * k, 0.94, 0.235])
            a.imshow(plt.imread(img))
            a.axis("off")
        pp.savefig(fig)
        plt.close(fig)
        fig = plt.figure(figsize=(8.27, 11.69))
        for k, img in enumerate(imgs[2:]):
            a = fig.add_axes([0.03, 0.75 - 0.24 * k, 0.94, 0.235])
            a.imshow(plt.imread(img))
            a.axis("off")
        if notas:
            fig.text(0.06, 0.012, "Notas: " + " · ".join(notas)[:600], fontsize=6.5,
                     color=TINTA2, wrap=True)
        pp.savefig(fig)
        plt.close(fig)
    return path


# ---------------------------------------------------------------------------
# Reporte
# ---------------------------------------------------------------------------
def _fecha_sesion(path: str, d: Datos) -> str:
    m = re.search(r"(\d{8}_\d{6}(?:_\d+)?)", os.path.basename(path))
    if m:
        return m.group(1)
    if d.n and math.isfinite(d["epoch_pc"][0]):
        return datetime.fromtimestamp(d["epoch_pc"][0]).strftime("%Y%m%d_%H%M%S")
    return datetime.fromtimestamp(os.path.getmtime(path)).strftime("%Y%m%d_%H%M%S")


def carpeta_reporte(csv_path: str, salida: Optional[str] = None,
                    d: Optional[Datos] = None) -> str:
    """Carpeta del reporte: <salida o carpeta del CSV>/reporte_<fecha de la sesión>."""
    if d is None:
        m = re.search(r"(\d{8}_\d{6}(?:_\d+)?)", os.path.basename(csv_path))
        fecha = m.group(1) if m else _fecha_sesion(csv_path, cargar(csv_path))
    else:
        fecha = _fecha_sesion(csv_path, d)
    base = salida or os.path.dirname(os.path.abspath(csv_path))
    return os.path.join(base, f"reporte_{fecha}")


def sesion_reciente(data_dir: str = DATA_DIR) -> Optional[str]:
    """CSV de sesión más reciente que tenga al menos un paquete."""
    files = sorted(glob.glob(os.path.join(data_dir, "sesion_*.csv")),
                   key=lambda p: (os.path.getmtime(p), p), reverse=True)
    for p in files:
        regs, _ = leer_registros(p)
        if regs:
            return p
    return None


def _md_tabla(cab: List[str], filas) -> str:
    esc = lambda s: str(s).replace("|", "\\|")
    out = ["| " + " | ".join(cab) + " |", "|" + "---|" * len(cab)]
    out += ["| " + " | ".join(esc(c) for c in f) + " |" for f in filas]
    return "\n".join(out)


def _eventos(r: dict, reloj: Reloj, desfase=0.0) -> List[Tuple[str, str, str, str]]:
    nombres = [("despegue", "Despegue (última muestra en el suelo)"),
               ("liberacion", "Liberación (CAÍDA)"), ("despliegue", "Disparo del autogiro"),
               ("fin_despliegue", "Fin del despliegue (DESCENSO)"),
               ("aterrizaje", "Aterrizaje (toque)"),
               ("estado_aterrizado", "Estado ATERRIZADO")]
    out = []
    for k, nom in nombres:
        e = r.get(k)
        if e:
            out.append((nom, reloj(e["t"] + desfase), f"{e['t']:.2f}", _n(e["altura_m"], 1, "m")))
    return out


HTML_CSS = """
:root{--t1:#0b0b0b;--t2:#52514e;--t3:#8a8984;--ln:#e3e2dd;--bg:#fcfcfb;--ac:#2a78d6;--ok:#008300;--bad:#c62f2e}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--t1);
font:15px/1.5 "Segoe UI",system-ui,-apple-system,Roboto,Arial,sans-serif}
main{max-width:1180px;margin:0 auto;padding:28px 28px 60px}
header{display:flex;align-items:center;gap:20px;border-bottom:2px solid var(--t1);padding-bottom:14px}
header img{height:72px}header h1{margin:0;font-size:26px;letter-spacing:.04em}
header p{margin:2px 0 0;color:var(--t2);font-size:13px}
h2{font-size:17px;margin:34px 0 10px;letter-spacing:.02em}
table{border-collapse:collapse;width:100%;font-variant-numeric:tabular-nums}
th,td{text-align:left;padding:7px 10px;border-bottom:1px solid var(--ln);vertical-align:top}
th{font-size:12px;text-transform:uppercase;letter-spacing:.06em;color:var(--t2);background:#f1f0ec}
td:first-child{color:var(--t2);width:42%}
.si{color:var(--ok);font-weight:600}.no{color:var(--bad);font-weight:600}
ul{padding-left:20px;color:var(--t2)}figure{margin:18px 0}figure img{width:100%;border:1px solid var(--ln)}
figcaption{font-size:12px;color:var(--t3)}footer{margin-top:40px;font-size:12px;color:var(--t3)}
@media print{main{padding:0}figure{break-inside:avoid}}
"""


def _html(titulo, sub, cab, filas, eventos, notas, imgs, extra_log) -> str:
    e = html.escape

    def celda(s):
        s = e(str(s))
        if s.startswith("Sí"):
            return f'<span class="si">{s}</span>'
        if s.startswith("No") and not s.startswith("No se"):
            return f'<span class="no">{s}</span>'
        return s

    logo = ""
    if os.path.isfile(LOGO):
        with open(LOGO, "rb") as f:
            logo = f'<img alt="Logotipo" src="data:image/png;base64,{base64.b64encode(f.read()).decode()}">'
    t = ["<table><thead><tr>" + "".join(f"<th>{e(c)}</th>" for c in cab) + "</tr></thead><tbody>"]
    t += ["<tr>" + "".join(f"<td>{celda(c)}</td>" for c in f) + "</tr>" for f in filas]
    t.append("</tbody></table>")
    ev = ["<table><thead><tr><th>Evento</th><th>Hora (PC)</th><th>T+ (s)</th><th>Altura</th>"
          "</tr></thead><tbody>"]
    ev += ["<tr>" + "".join(f"<td>{e(c)}</td>" for c in f) + "</tr>" for f in eventos]
    ev.append("</tbody></table>")
    figs = []
    for path, cap in imgs:
        with open(path, "rb") as f:
            b = base64.b64encode(f.read()).decode()
        figs.append(f'<figure><img alt="{e(cap)}" src="data:image/png;base64,{b}">'
                    f"<figcaption>{e(cap)}</figcaption></figure>")
    return f"""<!doctype html>
<html lang="es"><head><meta charset="utf-8"><title>{e(EQUIPO)} — Reporte de vuelo</title>
<meta name="viewport" content="width=device-width,initial-scale=1"><style>{HTML_CSS}</style></head>
<body><main>
<header>{logo}<div><h1>{e(EQUIPO.upper())} — Reporte de vuelo</h1><p>{e(sub)}</p></div></header>
<h2>Resumen</h2>{''.join(t)}
<h2>Eventos</h2>{''.join(ev)}
{extra_log}
<h2>Calidad de los datos</h2><ul>{''.join(f'<li>{e(n)}</li>' for n in notas) or '<li>Sin observaciones.</li>'}</ul>
<h2>Gráficas</h2>{''.join(figs)}
<footer>{e(titulo)} · generado por tools/reporte_vuelo.py</footer>
</main></body></html>
"""


def generar_reporte(csv_path: str, log_path: Optional[str] = None,
                    salida: Optional[str] = None, con_pdf: bool = False,
                    dpi: int = 200) -> Tuple[str, dict]:
    """Genera el reporte completo. Devuelve (carpeta, resumen)."""
    d = cargar(csv_path)
    if d.n == 0:
        raise ValueError(f"{os.path.basename(csv_path)} no tiene paquetes válidos")
    r = analizar(d)
    reloj = Reloj(d)
    carpeta = carpeta_reporte(csv_path, salida, d)
    os.makedirs(carpeta, exist_ok=True)
    for f in ARCHIVOS:                             # regenerar: quita lo de la vez anterior
        p = os.path.join(carpeta, f)
        if os.path.isfile(p):
            os.remove(p)
    sesion = os.path.basename(csv_path)

    log = log_r = log_nr = None
    desfase = 0.0
    log_txt: List[str] = []
    if log_path:
        log = cargar(log_path)
        if log.n == 0:
            log_txt.append(f"El log {os.path.basename(log_path)} no tiene registros válidos.")
            log = None
        else:
            desfase = alinear_log(d, log)
            log_r = analizar(log)
            log_nr = no_recibido(d, log.t + desfase)
            tr = tramos(log.t + desfase, log_nr)
            dur = sum(b - a for a, b in tr)
            log_txt.append(f"Log interno {os.path.basename(log_path)}: {log.n} registros; "
                           f"{int(log_nr.sum())} caen donde la estación no recibió paquetes "
                           f"({len(tr)} tramos, {dur:.1f} s en total; en rojo en las gráficas).")
            for k, nom in (("liberacion", "La liberación"), ("despliegue", "El disparo"),
                           ("aterrizaje", "El aterrizaje")):
                e = log_r.get(k)
                if e and any(a <= e["t"] + desfase <= b for a, b in tr):
                    log_txt.append(f"{nom} ocurrió en un tramo que la estación no recibió: "
                                   f"el log lo ubica a las {reloj(e['t'] + desfase)} "
                                   f"({_n(e['altura_m'], 1, 'm')}).")
            # Primero los tramos del vuelo (despegue a aterrizaje), que son los que importan.
            v0 = (r.get("despegue") or {}).get("t", -math.inf)
            v1 = (r.get("aterrizaje") or {}).get("t", math.inf)
            vuelo = [x for x in tr if x[1] >= v0 and x[0] <= v1]
            for a, b in vuelo[:12]:
                log_txt.append(f"Sin recepción en vuelo: T+{a:.1f} a T+{b:.1f} s "
                               f"({reloj(a)} a {reloj(b)}, {b - a:.1f} s).")
            if len(vuelo) > 12:
                log_txt.append(f"... y {len(vuelo) - 12} tramos más en vuelo.")
            if len(tr) > len(vuelo):
                log_txt.append(f"{len(tr) - len(vuelo)} tramos sin recepción antes del "
                               "despegue o después del aterrizaje.")

    imgs = graficas(d, r, carpeta, sesion, reloj, dpi, log, desfase, log_nr)
    filas = filas_tabla(r, reloj)
    cab = ["Concepto", "Estación terrena"]
    if log_r is not None:
        cab.append("Log interno")
        col = filas_log(log_r, reloj, desfase)
        filas_t = [(a, b, c) for (a, b), c in zip(filas, col)]
    else:
        filas_t = filas
    notas = notas_calidad(d, r)
    eventos = _eventos(r, reloj)
    fecha = reloj.fecha()
    titulo = f"{EQUIPO} · {CONCURSO} · sesión {sesion}" + (f" ({fecha})" if fecha else "")
    sub = (f"Sesión {sesion}" + (f" · {fecha}" if fecha else "") +
           f" · {r['n']} paquetes · {_dur(r.get('sesion_s'))} de sesión"
           + (f" · log interno {os.path.basename(log_path)}" if log_path else ""))
    caps = ["Altura vs tiempo", "Velocidad vertical vs tiempo (banda 8 a 12 m/s en el descenso)",
            "Presión y temperatura", "Aceleración (3 ejes y módulo)", "Actitud (roll, pitch, yaw)",
            "Calidad del enlace (RSSI, SNR y paquetes perdidos)"]

    md = [f"# {EQUIPO} — Reporte de vuelo", "", sub, "", "## Resumen", "",
          _md_tabla(cab, filas_t), "", "## Eventos", "",
          _md_tabla(["Evento", "Hora (PC)", "T+ (s)", "Altura"], eventos) if eventos else "Sin eventos de vuelo.",
          ""]
    if log_txt:
        md += ["## Log interno vs estación", ""] + [f"- {x}" for x in log_txt] + [""]
    md += ["## Calidad de los datos", ""] + ([f"- {x}" for x in notas] or ["- Sin observaciones."])
    md += ["", "## Gráficas", ""] + [f"![{c}]({os.path.basename(p)})" for p, c in zip(imgs, caps)]
    md += ["", f"_{titulo} · generado por tools/reporte_vuelo.py_", ""]
    with open(os.path.join(carpeta, "reporte.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(md))
    extra = ("<h2>Log interno vs estación</h2><ul>" + "".join(f"<li>{html.escape(x)}</li>" for x in log_txt)
             + "</ul>") if log_txt else ""
    with open(os.path.join(carpeta, "reporte.html"), "w", encoding="utf-8") as f:
        f.write(_html(titulo, sub, cab, filas_t, eventos, notas, list(zip(imgs, caps)), extra))
    res = {"sesion": sesion, "estacion": _limpio(r), "log": _limpio(log_r) if log_r else None,
           "desfase_log_s": desfase if log_r else None, "tabla": filas_t}
    with open(os.path.join(carpeta, "resumen.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, ensure_ascii=False, indent=1)
    if con_pdf:
        pdf(carpeta, titulo, filas_t, cab, notas + log_txt[:3])
    return carpeta, r


def _limpio(x):
    if isinstance(x, dict):
        return {k: _limpio(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_limpio(v) for v in x]
    if isinstance(x, float) and not math.isfinite(x):
        return None
    if isinstance(x, (np.integer,)):
        return int(x)
    if isinstance(x, (np.floating,)):
        return _limpio(float(x))
    return x


def abrir_carpeta(path: str) -> bool:
    """Abre la carpeta en el explorador de archivos. False si no se pudo."""
    try:
        if sys.platform.startswith("win"):
            os.startfile(path)  # type: ignore[attr-defined]
        elif sys.platform == "darwin":
            subprocess.Popen(["open", path])
        else:
            subprocess.Popen(["xdg-open", path], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
        return True
    except (OSError, AttributeError):
        return False


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Reporte de vuelo (gráficas + tabla) de una sesión")
    ap.add_argument("csv", nargs="?", help="CSV de sesión (def. el más reciente de ground/data)")
    ap.add_argument("--log", help="log interno bajado con DUMP (CSV o texto hex) para superponer")
    ap.add_argument("--pdf", action="store_true", help="también reporte.pdf (dos hojas)")
    ap.add_argument("--salida", help="carpeta donde crear reporte_<fecha>/ (def. la del CSV)")
    ap.add_argument("--dpi", type=int, default=200, help="resolución de los PNG (def. 200)")
    ap.add_argument("--abrir", action="store_true", help="abre la carpeta al terminar")
    ap.add_argument("--json", action="store_true",
                    help="imprime al final una línea JSON con la carpeta (para la estación)")
    a = ap.parse_args(argv)
    for stream in (sys.stdout, sys.stderr):        # Windows: la tabla lleva "→" y "≥"
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, ValueError):
            pass
    path = a.csv or sesion_reciente()
    if not path:
        ap.error(f"no hay sesiones con paquetes en {DATA_DIR}")
    if not os.path.isfile(path):
        ap.error(f"no existe {path}")
    if a.log and not os.path.isfile(a.log):
        ap.error(f"no existe {a.log}")
    try:
        carpeta, r = generar_reporte(path, a.log, a.salida, a.pdf, a.dpi)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    reloj = Reloj(cargar(path))
    print(f"Sesión: {path}")
    for k, v in filas_tabla(r, reloj):
        print(f"  {k:66} {v}")
    print(f"Reporte en: {carpeta}")
    if a.abrir:
        abrir_carpeta(carpeta)
    if a.json:
        print(json.dumps({"carpeta": carpeta,
                          "archivos": sorted(os.listdir(carpeta))}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
