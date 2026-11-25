#!/usr/bin/env bash
# Tinaco SOLO carga primaria: pone la placa en MODO D (el servo NO dispara).
#
#   tools/tinaco_primaria.sh [PUERTO]     sin argumento detecta la placa (la /dev/ttyUSB* que responde STAT)
#
# Sin red. Falla en rojo con código distinto de 0 si el STAT no confirma el modo.
set -u
RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-$RAIZ/.venv/bin/python}"
if [ ! -x "$PYTHON" ]; then
    printf '\033[31mNo se encontró %s (crea el .venv: ver README.md)\033[0m\n' "$PYTHON" >&2
    exit 1
fi
if [ $# -ge 1 ]; then
    exec "$PYTHON" "$RAIZ/tools/modo_placa.py" --modo D --port "$1"
fi
exec "$PYTHON" "$RAIZ/tools/modo_placa.py" --modo D
