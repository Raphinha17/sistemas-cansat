#!/usr/bin/env python3
"""Envía un vuelo simulado a la placa por serial (modo simulación, docs/sim_input.md).

Lee el CSV crudo de tools/flight_sim.py (--raw-csv, 100 Hz) y manda una línea
SIM,... por fila al ritmo del reloj (o más rápido con --speed). La placa usa esas
lecturas en lugar de sus sensores: sirve para la prueba en banco con el servo real.
Lo que imprime la placa se muestra en pantalla.

Ejemplos:
  py tools/flight_sim.py --raw-csv vuelo.csv --seed 1
  py tools/sim_serial.py --csv vuelo.csv --port COM5
  py tools/sim_serial.py --csv vuelo.csv --port COM5 --modo B   # MODO B antes del vuelo
  py tools/sim_serial.py --csv vuelo.csv --dry-run --out lineas.txt   # sin placa

Antes de las líneas manda SIM_INICIO (la placa entra al modo simulación y reinicia de cero,
aunque no tenga sensores o venga de otro vuelo simulado); --no-inicio lo omite.

Integridad (docs/sim_input.md): cada línea lleva "*HHHH" (CRC-16/CCITT-FALSE). La placa
descarta las dañadas o fuera de orden y pide "SIM_REPITE t_ms"; este programa retrocede a
esa línea y sigue a tiempo real desde ahí. Al final pregunta SIM_PROXIMA y reenvía lo que
falte, así la placa procesa todas las líneas del CSV, en orden y sin huecos. Termina con
código 0 solo si la placa lo confirma. --sin-crc manda el formato anterior (firmware
viejo; sin reenvío ni confirmación).
"""
from __future__ import annotations

import argparse
import binascii
import csv
import re
import sys
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, Iterator, List, Optional

IMU_COLS = ("ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")

REPITE_RE = re.compile(r"^SIM_REPITE (\d+)\s*$")
PROXIMA_RE = re.compile(r"^SIM_PROXIMA (-?\d+)\s*$")
REWIND_DEDUP_S = 0.15  # ignora el mismo SIM_REPITE si ya se retrocedió hace menos que esto


def _num(x: float, nd: int) -> str:
    s = f"{x:.{nd}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def format_line(row: Dict[str, str]) -> str:
    """Una fila del CSV crudo -> línea SIM,t_ms,ax,ay,az,gx,gy,gz,p_pa,temp_c (sin CRC).

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


def add_crc(body: str) -> str:
    """Agrega *HHHH: CRC-16/CCITT-FALSE de todo el texto (igual que simCrc16 en la placa)."""
    return f"{body}*{binascii.crc_hqx(body.encode('ascii'), 0xFFFF):04X}"


def line_t_ms(line: str) -> int:
    return int(line.split(",", 2)[1])


def lines_from_csv(path: str, crc: bool = True) -> Iterator[str]:
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            body = format_line(row)
            yield add_crc(body) if crc else body


class LineReader:
    """Lee lo que manda la placa sin bloquear.

    (readline() con timeout esperaba hasta 50 ms a que terminara una línea y luego se
    mandaban varias líneas SIM de golpe.)"""

    def __init__(self, ser):
        self.ser = ser
        self.buf = b""

    def poll(self) -> List[str]:
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        if b"\n" not in self.buf:
            return []
        *done, self.buf = self.buf.split(b"\n")
        return [d.decode(errors="replace").rstrip("\r") for d in done]


@dataclass
class SendResult:
    enviadas: int = 0          # líneas escritas, con los reenvíos
    pedidos: int = 0           # SIM_REPITE recibidos
    retrocesos: int = 0        # veces que se volvió atrás (SIM_REPITE o SIM_PROXIMA)
    proxima: Optional[int] = None   # última respuesta a SIM_PROXIMA
    confirmado: bool = False   # la placa recibió hasta la última línea


def send(lines: Iterable[str], ser, speed: float = 1.0, period_s: float = 0.01,
         echo: Optional[Callable[[str], None]] = None, verify: bool = True,
         wait_s: float = 1.0, max_consultas: int = 5,
         clock=time.perf_counter, sleep=time.sleep) -> SendResult:
    """Envía las líneas a ritmo de period_s/speed atendiendo los pedidos de reenvío.

    Tras un SIM_REPITE se retrocede a esa línea y el ritmo sigue desde ahí (sin ráfagas:
    el vuelo tarda un poco más que el tiempo real). Con verify, al final pregunta
    SIM_PROXIMA hasta que la placa confirma la última (máx. max_consultas sin respuesta).
    """
    lines = list(lines)
    res = SendResult()
    if not lines:
        return res
    t_of = [line_t_ms(x) for x in lines]
    idx = {t: i for i, t in enumerate(t_of)}
    reader = LineReader(ser) if ser is not None else None
    step = period_s / speed
    i = 0
    nxt = clock()
    last_rw = (None, -1.0)

    def rewind(j: int) -> None:
        nonlocal i, nxt
        i = j
        nxt = clock()
        res.retrocesos += 1

    def pump() -> Optional[int]:
        """Atiende los mensajes de la placa; devuelve la respuesta a SIM_PROXIMA si llegó."""
        nonlocal last_rw
        prox = None
        if reader is None:
            return None
        for msg in reader.poll():
            m = REPITE_RE.match(msg)
            if m:
                res.pedidos += 1
                t = int(m.group(1))
                j = idx.get(t)
                now = clock()
                if (j is not None and j < i
                        and not (last_rw[0] == t and now - last_rw[1] < REWIND_DEDUP_S)):
                    last_rw = (t, now)
                    rewind(j)
                continue
            m = PROXIMA_RE.match(msg)
            if m:
                prox = int(m.group(1))
                continue
            if msg and echo:
                echo(msg)
        return prox

    consultas = 0
    while True:
        while i < len(lines):
            while True:
                pump()
                now = clock()
                if now >= nxt:
                    break
                sleep(min(0.002, nxt - now))
            if ser is not None:
                ser.write((lines[i] + "\n").encode())
            i += 1
            res.enviadas += 1
            nxt += step
            now = clock()
            if now - nxt > 3 * step:
                nxt = now  # la PC se atrasó: sigue desde ahora, sin mandar de golpe
        if not verify or ser is None:
            break
        # Fin del CSV: ¿la placa recibió hasta la última línea?
        ser.write(b"SIM_PROXIMA\n")
        t_end = clock() + wait_s
        prox = None
        while prox is None and i >= len(lines) and clock() < t_end:
            prox = pump()
            if prox is None:
                sleep(0.005)
        if i < len(lines):
            continue  # llegó un SIM_REPITE: reenviar desde ahí
        if prox is None:
            consultas += 1
            if consultas >= max_consultas:
                break
            continue
        res.proxima = prox
        if prox > t_of[-1]:
            res.confirmado = True
            break
        j = idx.get(prox, 0 if prox < t_of[0] else None)
        if j is None:
            break  # la placa espera una línea que no está en el CSV
        rewind(j)
    return res


def prepare(ser, modo: Optional[str] = None, wait_s: float = 0.4, echo=None) -> list:
    """Manda SIM_INICIO (y MODO A|B|C si se pide) y devuelve los comandos enviados.

    SIM_INICIO va primero: deja la placa en el estado 0 y así acepta MODO aunque venga de
    un vuelo simulado que terminó en ATERRIZADO."""
    cmds = ["SIM_INICIO"] + ([f"MODO {modo.upper()}"] if modo else [])
    for c in cmds:
        ser.write((c + "\n").encode())
        t_end = time.time() + wait_s
        while time.time() < t_end:
            msg = ser.readline().decode(errors="replace").rstrip()
            if msg and echo:
                echo(msg)
    return cmds


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
    ap.add_argument("--modo", choices=["A", "B", "C"],
                    help="manda MODO A|B|C antes del vuelo (el CSV debe ser del mismo modo)")
    ap.add_argument("--espera", type=float, default=2.0,
                    help="s de espera tras abrir el puerto (por si la placa se reinicia; def. 2)")
    ap.add_argument("--no-inicio", action="store_true",
                    help="no manda SIM_INICIO (la primera línea SIM activa el modo)")
    ap.add_argument("--sin-crc", action="store_true",
                    help="formato anterior, sin CRC ni reenvío (firmware viejo)")
    a = ap.parse_args(argv)
    if a.speed <= 0:
        ap.error("--speed debe ser > 0")
    crc = not a.sin_crc

    if a.dry_run:
        out = open(a.out, "w") if a.out else sys.stdout
        n = 0
        for line in lines_from_csv(a.csv, crc):
            out.write(line + "\n")
            n += 1
        if out is not sys.stdout:
            out.close()
        print(f"lineas={n}", file=sys.stderr)
        return 0

    if not a.port:
        ap.error("falta --port (o --dry-run)")
    import serial  # pyserial, solo si se usa el puerto

    lines = list(lines_from_csv(a.csv, crc))
    # DTR/RTS en bajo antes de abrir: así abrir el puerto no reinicia la placa (CP210x).
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = a.port, a.baud, 0.05
    ser.dtr = False
    ser.rts = False
    ser.open()
    t0 = time.time()

    def echo(m):                       # lo que dice la placa, con segundos desde el inicio
        print(f"[{time.time() - t0:7.2f} s] < {m}", flush=True)

    with ser:
        time.sleep(a.espera)
        if not a.no_inicio:
            prepare(ser, a.modo, echo=echo)
        elif a.modo:
            ap.error("--modo necesita SIM_INICIO (quita --no-inicio)")
        t_send = time.time()
        res = send(lines, ser, a.speed, echo=echo, verify=crc)
        dur = time.time() - t_send
        ser.write(b"STAT\n")           # resumen final: disparo, intentos del servo, etc.
        t_end = time.time() + 1.5
        while time.time() < t_end:
            msg = ser.readline().decode(errors="replace").rstrip()
            if msg:
                echo(msg)
        if a.fin:
            ser.write(b"SIM_FIN\n")
            time.sleep(0.3)
    sim_s = len(lines) * 0.01 / a.speed
    print(f"lineas del CSV={len(lines)} enviadas={res.enviadas} "
          f"(reenviadas={res.enviadas - len(lines)}) pedidos_de_reenvio={res.pedidos} "
          f"retrocesos={res.retrocesos} duracion={dur:.1f} s (tiempo real {sim_s:.1f} s)",
          file=sys.stderr)
    if not crc:
        return 0
    if res.confirmado:
        print(f"OK: la placa recibio las {len(lines)} lineas en orden "
              f"(SIM_PROXIMA={res.proxima})", file=sys.stderr)
        return 0
    print(f"ERROR: la placa no confirmo todas las lineas (SIM_PROXIMA={res.proxima}). "
          "Si el firmware es anterior al CRC, usa --sin-crc o actualizalo.", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
