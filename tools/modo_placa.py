#!/usr/bin/env python3
"""Cambia el modo de despliegue de la placa por serial y confirma con STAT (sin red).

Lo usan tools/modo_dron.sh (B), tools/tinaco_separacion.sh (C) y tools/tinaco_primaria.sh (D).

  python tools/modo_placa.py --modo D [--port /dev/ttyUSB1]

Puerto: el argumento --port; si no, se prueba cada /dev/ttyUSB* mandando STAT y se elige la
que responde con "estado="; si ninguna o varias responden, avisa y sale sin hacer nada.
Códigos de salida: 0 = modo confirmado; 1 = el STAT no confirma el modo; 2 = no se pudo
abrir el puerto o se perdió; 3 = no se pudo elegir un puerto.
"""
from __future__ import annotations

import argparse
import glob
import re
import sys
import time
from typing import Callable, List, Optional, Tuple

ROJO, VERDE, NEGRITA, FIN = "\033[31m", "\033[32m", "\033[1m", "\033[0m"
PATRON = "/dev/ttyUSB*"

MENSAJES = {
    "B": ("MODO B (DRON): lanzamiento real, suelta ~100 m, autogiro a 80 m",
          "PLACA EN MODO B: lista para el dron"),
    "C": ("MODO C (TINACO CON SEPARACIÓN)",
          "TINACO CON SEPARACIÓN: el servo SÍ disparará"),
    "D": ("MODO D (TINACO SOLO CARGA PRIMARIA)",
          "TINACO SOLO CARGA PRIMARIA: el servo NO disparará"),
}


def abrir(port: str, baud: int = 115200):
    """Abre el puerto sin reiniciar la placa (DTR y RTS apagados antes de abrir)."""
    import serial
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.2
    ser.dtr = ser.rts = False
    ser.open()
    return ser


def leer(ser, segundos: float) -> str:
    fin, buf = time.monotonic() + segundos, b""
    while time.monotonic() < fin:
        buf += ser.read(ser.in_waiting or 1)
    return buf.decode("utf-8", "replace")


def responde_stat(port: str, espera: float = 2.0) -> bool:
    """True si la placa del puerto contesta STAT con 'estado='."""
    try:
        ser = abrir(port)
    except Exception:  # noqa: BLE001  (ocupado, sin permiso, desconectado)
        return False
    try:
        ser.reset_input_buffer()
        ser.write(b"STAT\n")
        ser.flush()
        return "estado=" in leer(ser, espera)
    except Exception:  # noqa: BLE001
        return False
    finally:
        ser.close()


def elegir_puerto(explicito: Optional[str], candidatos: Optional[List[str]] = None,
                  prueba: Callable[[str], bool] = responde_stat) -> Tuple[Optional[str], str]:
    """(puerto, mensaje de error). Con puerto explícito se usa tal cual; si no, el único
    candidato que responde STAT con 'estado='."""
    if explicito:
        return explicito, ""
    cand = sorted(glob.glob(PATRON)) if candidatos is None else candidatos
    if not cand:
        return None, f"No hay ningún puerto {PATRON}. Conecta la placa o indica el puerto."
    vivos = [p for p in cand if prueba(p)]
    if not vivos:
        return None, f"Ningún puerto ({', '.join(cand)}) respondió STAT con 'estado='."
    if len(vivos) > 1:
        return None, f"Varias placas respondieron ({', '.join(vivos)}). Indica cuál usar."
    return vivos[0], ""


def confirma(stat: str, modo: str) -> bool:
    """True si la respuesta de STAT dice despliegue=<modo>."""
    m = re.search(r"despliegue=([ABCD])\b", stat)
    return bool(m and m.group(1) == modo)


def banner(texto: str, color: str) -> str:
    ancho = len(texto) + 8
    barra = "#" * ancho
    vacio = "#" + " " * (ancho - 2) + "#"
    return "\n".join([color + NEGRITA, barra, vacio, f"#   {texto}   #", vacio, barra, FIN])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--modo", required=True, choices=sorted(MENSAJES))
    ap.add_argument("--port", help="puerto serie (si falta, se detecta)")
    a = ap.parse_args(argv)
    port, err = elegir_puerto(a.port)
    if not port:
        print(f"{ROJO}{err}{FIN}", file=sys.stderr)
        return 3
    print(f"Puerto: {port}")
    try:
        ser = abrir(port)
    except Exception as e:  # noqa: BLE001
        print(f"{ROJO}No se pudo abrir {port}: {e}{FIN}", file=sys.stderr)
        return 2
    try:
        ser.reset_input_buffer()
        ser.write(f"MODO {a.modo}\n".encode())
        ser.flush()
        resp = leer(ser, 2.0)
        print(f">>> MODO {a.modo}\n{resp.strip()}\n")
        ser.write(b"STAT\n")
        ser.flush()
        stat = leer(ser, 2.0)
        print(f">>> STAT\n{stat.strip()}\n")
    except Exception as e:  # noqa: BLE001
        print(f"{ROJO}Se perdió la conexión: {e}{FIN}", file=sys.stderr)
        return 2
    finally:
        ser.close()
    if confirma(stat, a.modo):
        print(banner(MENSAJES[a.modo][1], VERDE))
        return 0
    print(banner(f"FALLA: el STAT NO confirma despliegue={a.modo}", ROJO), file=sys.stderr)
    if "ERR" in resp:
        print(f"{ROJO}La placa respondió: {resp.strip()}{FIN}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
