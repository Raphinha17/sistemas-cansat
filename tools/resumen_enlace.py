#!/usr/bin/env python3
"""Resume la calidad del enlace de una o varias sesiones grabadas por la estación terrena.

Para la prueba de alcance (docs/PRUEBAS_SISTEMAS.md, prueba j): una sesión (CSV de
ground/data/) por distancia. Imprime por archivo: paquetes recibidos, esperados (por
secuencia), % de pérdida y RSSI/SNR promedio y mínimo.

  py tools/resumen_enlace.py ground/data/sesion_*.csv
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys
from typing import Dict, List, Optional


def resumen(path: str) -> Optional[Dict[str, float]]:
    """Estadísticas de un CSV de la estación; None si no tiene paquetes."""
    seq: List[int] = []
    rssi: List[float] = []
    snr: List[float] = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        for row in csv.DictReader(f):
            try:
                s, r, n = int(float(row["secuencia"])), float(row["rssi_dbm"]), float(row["snr_db"])
            except (KeyError, ValueError, TypeError):
                continue
            seq.append(s)
            rssi.append(r)
            snr.append(n)
    if not seq:
        return None
    # Esperados por secuencia (u16 con vuelta): suma de saltos hacia adelante.
    esperados = 1
    rx = 1
    for a, b in zip(seq, seq[1:]):
        gap = (b - a) & 0xFFFF
        if gap == 0:                      # duplicado: no cuenta
            continue
        rx += 1
        esperados += 1 if gap > 30000 else gap   # reinicio de la placa: empieza otra racha

    return {
        "recibidos": rx, "esperados": esperados,
        "perdida_pct": 100.0 * max(0, esperados - rx) / esperados,
        "rssi_prom": statistics.fmean(rssi), "rssi_min": min(rssi),
        "snr_prom": statistics.fmean(snr), "snr_min": min(snr),
    }


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(description="Resumen de enlace por sesión (CSV de ground/data)")
    ap.add_argument("csv", nargs="+", help="uno o más CSV de sesión")
    a = ap.parse_args(argv)
    print(f"{'archivo':34} {'rx':>5} {'esp':>5} {'pérd %':>7} {'RSSI prom':>10} {'RSSI mín':>9} "
          f"{'SNR prom':>9} {'SNR mín':>8}")
    for p in a.csv:
        r = resumen(p)
        name = p.replace("\\", "/").rsplit("/", 1)[-1]
        if r is None:
            print(f"{name:34} (sin paquetes)")
            continue
        print(f"{name:34} {r['recibidos']:5d} {r['esperados']:5d} {r['perdida_pct']:7.1f} "
              f"{r['rssi_prom']:10.1f} {r['rssi_min']:9.1f} {r['snr_prom']:9.2f} {r['snr_min']:8.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
