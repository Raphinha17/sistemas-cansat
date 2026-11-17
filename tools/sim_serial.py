#!/usr/bin/env python3
"""Envía un vuelo simulado a la placa por serial (modo simulación, docs/sim_input.md).

Lee el CSV crudo de tools/flight_sim.py (--raw-csv, 100 Hz) y manda una línea
SIM,... por fila al ritmo del reloj (o más rápido con --speed). La placa usa esas
lecturas en lugar de sus sensores: sirve para la prueba en banco con el servo real.
Lo que imprime la placa se muestra en pantalla.

Ejemplos:
  py tools/flight_sim.py --raw-csv vuelo.csv --seed 1
  py tools/sim_serial.py --csv vuelo.csv --port COM5
  py tools/sim_serial.py --csv vuelo.csv --dry-run --out lineas.txt   # sin placa
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from typing import Dict, Iterable, Iterator, Optional

IMU_COLS = ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")


def _num(x: float, nd: int) -> str:
    s = f"{x:.{nd}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def format_line(row: Dict[str, str]) -> str:
    """Una fila del CSV crudo -> línea SIM,t_ms,ax,ay,az,gx,gy,gz,p_pa,temp_c.

    flight_sim.py marca el IMU muerto con ceros en las seis columnas: se mandan vacías
    (lectura fallida), igual que haría la placa con un error de I2C.
    """
    t_ms = int(round(float(row["t"]) * 1000))
    imu = [float(row[c]) for c in IMU_COLS]
    if all(v == 0.0 for v in imu):
        imu_txt = [""] * 6
    else:
        imu_txt = [_num(v, 4) for v in imu[:3]] + [_num(v, 3) for v in imu[3:]]
    p = _num(float(row["p_pa"]), 1)
    temp = _num(float(row["temp_c"]), 2)
    return ",".join(["SIM", str(t_ms)] + imu_txt + [p, temp])


def lines_from_csv(path: str) -> Iterator[str]:
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            yield format_line(row)


def send(lines: Iterable[str], ser, speed: float = 1.0, period_s: float = 0.01,
         echo=None) -> int:
    """Envía las líneas a ritmo de period_s/speed. Devuelve cuántas envió."""
    n = 0
    t0 = time.perf_counter()
    for line in lines:
        target = t0 + n * period_s / speed
        while True:
            now = time.perf_counter()
            if ser is not None and ser.in_waiting:
                msg = ser.readline().decode(errors="replace").rstrip()
                if msg and echo:
                    echo(msg)
            if now >= target:
                break
            time.sleep(min(0.002, target - now))
        data = (line + "\n").encode()
        if ser is not None:
            ser.write(data)
        n += 1
    return n


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(description="Vuelo simulado -> placa (líneas SIM por serial)")
    ap.add_argument("--csv", required=True, help="CSV crudo de flight_sim.py --raw-csv")
    ap.add_argument("--port", help="puerto serial de la placa (p. ej. COM5)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=float, default=1.0,
                    help="factor de velocidad (1 = tiempo real; el enlace a 115200 admite ~1.3)")
    ap.add_argument("--dry-run", action="store_true", help="no abre el puerto; escribe las líneas")
    ap.add_argument("--out", help="con --dry-run: archivo de salida (por defecto, stdout)")
    ap.add_argument("--fin", action="store_true", help="al terminar, manda SIM_FIN")
    a = ap.parse_args(argv)
    if a.speed <= 0:
        ap.error("--speed debe ser > 0")

    if a.dry_run:
        out = open(a.out, "w") if a.out else sys.stdout
        n = 0
        for line in lines_from_csv(a.csv):
            out.write(line + "\n")
            n += 1
        if out is not sys.stdout:
            out.close()
        print(f"lineas={n}", file=sys.stderr)
        return 0

    if not a.port:
        ap.error("falta --port (o --dry-run)")
    import serial  # pyserial, solo si se usa el puerto

    with serial.Serial(a.port, a.baud, timeout=0.05) as ser:
        time.sleep(0.5)
        n = send(lines_from_csv(a.csv), ser, a.speed, echo=lambda m: print("<", m))
        if a.fin:
            ser.write(b"SIM_FIN\n")
        t_end = time.time() + 1.0
        while time.time() < t_end:
            msg = ser.readline().decode(errors="replace").rstrip()
            if msg:
                print("<", msg)
    print(f"lineas enviadas={n}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
