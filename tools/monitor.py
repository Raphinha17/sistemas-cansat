#!/usr/bin/env python3
"""Monitor serial interactivo para las placas (pyserial).

  python tools/monitor.py --port $TX

Un hilo imprime lo que llega del puerto. Lo que escribes solo se envía al pulsar Enter, y
siempre como línea completa (en mayúsculas, sin espacios sobrantes, con "\\n"): nunca se
mandan caracteres sueltos. Una línea vacía no envía nada. Salir: /salir o Ctrl+C.
Abrir el puerto no reinicia la placa (DTR y RTS apagados antes de abrir).
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
from typing import Optional

DEFAULT_PORT = "/dev/ttyUSB1"


def normalizar(linea: str) -> Optional[str]:
    """Línea lista para enviar: sin espacios en los extremos y en mayúsculas (los espacios
    internos se respetan, p. ej. "modo b" -> "MODO B"). None si queda vacía."""
    s = (linea or "").strip().upper()
    return s or None


def es_salida(linea: str) -> bool:
    return (linea or "").strip().lower() == "/salir"


def abrir(port: str, baud: int):
    """Abre el puerto con DTR y RTS apagados antes de abrir, para no reiniciar la placa."""
    import serial
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, baud, 0.2
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def _lector(ser, parar: threading.Event, perdido: threading.Event):
    import serial
    buf = b""
    while not parar.is_set():
        try:
            datos = ser.read(ser.in_waiting or 1)
        except (serial.SerialException, OSError):
            perdido.set()
            return
        if not datos:
            continue
        buf += datos
        *lineas, buf = buf.split(b"\n")
        for ln in lineas:
            print(ln.decode("utf-8", "replace").rstrip("\r"), flush=True)
        if len(buf) > 4096:
            print(buf.decode("utf-8", "replace"), flush=True)
            buf = b""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Monitor serial interactivo (envía líneas completas)")
    ap.add_argument("--port", default=os.environ.get("TX", DEFAULT_PORT),
                    help="puerto serie (def. variable TX, si no %s)" % DEFAULT_PORT)
    ap.add_argument("--baud", type=int, default=115200)
    a = ap.parse_args(argv)
    try:
        import serial
        ser = abrir(a.port, a.baud)
    except (ImportError, OSError, ValueError) as e:
        print(f"No se pudo abrir {a.port}: {e}", file=sys.stderr)
        return 1
    print(f"Conectado a {a.port} @ {a.baud}. Escribe un comando y Enter; /salir o Ctrl+C para salir.")
    parar, perdido = threading.Event(), threading.Event()
    hilo = threading.Thread(target=_lector, args=(ser, parar, perdido), daemon=True)
    hilo.start()
    codigo = 0
    try:
        while True:
            if perdido.is_set():
                raise OSError("puerto desconectado")
            try:
                linea = input("> ")
            except EOFError:
                break
            if perdido.is_set():
                raise OSError("puerto desconectado")
            if es_salida(linea):
                break
            cmd = normalizar(linea)
            if cmd is None:
                continue
            ser.write(cmd.encode("utf-8") + b"\n")
            ser.flush()
    except KeyboardInterrupt:
        print()
    except (serial.SerialException, OSError):
        print("\nSe perdió la conexión con el puerto (¿desconectaron la placa?). Saliendo.",
              file=sys.stderr)
        codigo = 1
    finally:
        parar.set()
        hilo.join(timeout=1)
        try:
            ser.close()
        except Exception:  # noqa: BLE001
            pass
    return codigo


if __name__ == "__main__":
    raise SystemExit(main())
