#!/usr/bin/env bash
# Prepara la placa de vuelo con un solo comando: detecta el puerto, compila y carga
# heltec_wifi_lora_32_V3, y manda MISION_BORRAR, BORRAR, MODO B y STAT verificando el resultado.
#
#   tools/cargar_vuelo.sh [PUERTO]       p. ej. tools/cargar_vuelo.sh /dev/ttyUSB1
#
# Sin argumento usa /dev/ttyUSB* solo si hay exactamente uno. Código de salida 0 = lista.
set -u

RAIZ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-$RAIZ/.venv/bin/python}"
PIO="${PIO:-$RAIZ/.venv/bin/pio}"
PATRON="${CARGAR_VUELO_PATRON:-/dev/ttyUSB*}"
ROJO=$'\033[31m'; FIN=$'\033[0m'

falla() { echo "${ROJO}$*${FIN}" >&2; exit "${2:-1}"; }

if [ $# -ge 1 ]; then
    PUERTO="$1"
else
    shopt -s nullglob
    # shellcheck disable=SC2206  # el patrón debe expandirse
    puertos=($PATRON)
    shopt -u nullglob
    case ${#puertos[@]} in
        0) falla "No hay ningún puerto $PATRON. Conecta la placa o indica el puerto: $0 /dev/ttyUSB1" ;;
        1) PUERTO="${puertos[0]}" ;;
        *) falla "Hay varios puertos (${puertos[*]}). Indica cuál: $0 ${puertos[0]}" ;;
    esac
fi
[ -e "$PUERTO" ] || falla "No existe el puerto $PUERTO"
[ -x "$PYTHON" ] || falla "No se encontró $PYTHON (crea el .venv: ver README.md)"
[ -x "$PIO" ] || falla "No se encontró $PIO (instala requirements.txt en el .venv)"

echo "Puerto: $PUERTO"
echo "== Compilando y cargando heltec_wifi_lora_32_V3 =="
"$PIO" run -d "$RAIZ/firmware" -e heltec_wifi_lora_32_V3 -t upload --upload-port "$PUERTO" \
    || falla "Falló la compilación o la carga: la placa NO está lista para vuelo" 3

echo "== Esperando a que la placa reinicie =="
sleep 4
"$PYTHON" "$RAIZ/tools/cargar_vuelo.py" --port "$PUERTO"
