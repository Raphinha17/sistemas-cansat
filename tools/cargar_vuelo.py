#!/usr/bin/env python3
"""Prepara por serial la placa de vuelo y verifica su estado (lo usa tools/cargar_vuelo.sh).

Manda MISION_BORRAR (espera 5 s: la placa reinicia), BORRAR, MODO B y STAT, interpreta lo
recibido e imprime un resumen. Código de salida 0 solo si todo está bien.

  python tools/cargar_vuelo.py --port /dev/ttyUSB1
"""
from __future__ import annotations

import argparse
import re
import sys
import time
from typing import List, Tuple

ROJO, VERDE, FIN = "\033[31m", "\033[32m", "\033[0m"
ESPERA_REINICIO_S = 5.0
ESPERA_CMD_S = 1.5


def analizar(arranque: str, stat: str) -> List[Tuple[str, bool, str]]:
    """Revisiones (nombre, ok, detalle) a partir del texto del arranque y de la respuesta
    de STAT."""
    r: List[Tuple[str, bool, str]] = []
    m = re.search(r"despliegue=([ABC])\b", stat)
    r.append(("despliegue=B confirmado", bool(m and m.group(1) == "B"),
              f"STAT dice despliegue={m.group(1)}" if m else "STAT no mostró el modo"))
    for campo in ("imu_err", "baro_err"):
        m = re.search(rf"{campo}=(\d+)", stat)
        r.append((f"{campo} en 0", bool(m and int(m.group(1)) == 0),
                  f"{campo}={m.group(1)}" if m else f"STAT no mostró {campo}"))
    if not re.search(r"IMU \w+", arranque):
        r.append(("arranque sin IMU FALLA ni BARO FALLA", False,
                  "no se vio el arranque de la placa (¿reinició?)"))
    else:
        mal = [x for x in ("IMU FALLA", "BARO FALLA") if x in arranque]
        r.append(("arranque sin IMU FALLA ni BARO FALLA", not mal,
                  ", ".join(mal) if mal else "IMU OK, BARO OK"))
    return r


def resumen(revisiones, color: bool = True) -> Tuple[str, bool]:
    ok_total = all(ok for _, ok, _ in revisiones)
    rojo, verde, fin = (ROJO, VERDE, FIN) if color else ("", "", "")
    lineas = []
    for nombre, ok, det in revisiones:
        lineas.append(f"  {verde + 'OK   ' if ok else rojo + 'FALLA'}{fin}  {nombre}  ({det})")
    fallas = [n for n, ok, _ in revisiones if not ok]
    if ok_total:
        lineas.append(f"{verde}Placa lista para vuelo en MODO B{fin}")
    else:
        lineas.append(f"{rojo}Placa NO lista para vuelo: falló {'; '.join(fallas)}{fin}")
    return "\n".join(lineas), ok_total


def _leer(ser, segundos: float) -> str:
    fin, buf = time.monotonic() + segundos, b""
    while time.monotonic() < fin:
        buf += ser.read(ser.in_waiting or 1)
    return buf.decode("utf-8", "replace")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args(argv)
    import serial
    try:
        ser = serial.Serial()
        ser.port, ser.baudrate, ser.timeout = a.port, a.baud, 0.2
        ser.dtr = ser.rts = False                      # abrir no reinicia la placa
        ser.open()
    except (serial.SerialException, OSError) as e:
        print(f"{ROJO}No se pudo abrir {a.port}: {e}{FIN}", file=sys.stderr)
        return 2
    try:
        ser.reset_input_buffer()
        texto = {}
        for cmd, espera in (("MISION_BORRAR", ESPERA_REINICIO_S), ("BORRAR", ESPERA_CMD_S),
                            ("MODO B", ESPERA_CMD_S), ("STAT", ESPERA_CMD_S)):
            ser.write(cmd.encode() + b"\n")
            ser.flush()
            texto[cmd] = _leer(ser, espera)
            print(f">>> {cmd}\n{texto[cmd].strip()}\n")
    except (serial.SerialException, OSError) as e:
        print(f"{ROJO}Se perdió la conexión: {e}{FIN}", file=sys.stderr)
        return 2
    finally:
        ser.close()
    txt, ok = resumen(analizar(texto["MISION_BORRAR"], texto["STAT"]))
    print(txt)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
