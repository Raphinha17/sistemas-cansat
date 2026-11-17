# Firmware CanSat Hellmary

PlatformIO, placa `heltec_wifi_lora_32_V3` (ESP32-S3 + SX1262), framework Arduino, RadioLib.
Fuente de verdad: `docs/SPEC.md`. Dudas y suposiciones: `docs/PREGUNTAS.md`.

Etapa actual: sensores, filtros, telemetría y log. **Sin máquina de estados de vuelo.**

## Entornos
| Entorno | Qué es | Comando (desde `firmware/`) |
|---|---|---|
| `heltec_wifi_lora_32_V3` | vuelo | `pio run -e heltec_wifi_lora_32_V3` |
| `rx_station` | receptor de la estación terrena | `pio run -e rx_station -t upload` |
| `native` | pruebas en PC | `pio test -e native` |

En Windows, `pio` está en el `.venv` del proyecto: `..\.venv\Scripts\pio.exe`.
El entorno `native` usa `g++` si existe; si no, el compilador de `zig` (`pip install ziglang`
en el `.venv`). Las pruebas generan sus datos con `tools/flight_sim.py --raw-csv`.

El entorno de vuelo **no compila** mientras haya pines `TODO_CONFIRMAR` en
`include/config.h` (#error intencional). Para comprobar que el resto compila:
`set PLATFORMIO_BUILD_FLAGS=-DPERMITIR_PINES_PENDIENTES` (cmd) antes de `pio run`.
Nunca subir a la placa un binario compilado así.

## Estructura
```
include/  config.h (parámetros y pines), packet.h, lora_setup.h (radio compartido)
lib/      attitude (Mahony + sesgo del giroscopio), estimator (Kalman h/vz + p0),
          telemetry (armado del paquete)  -> portables, probados en native
src/flight/  main.cpp (tareas), sensors (MPU6500 propio + BMP280), telemetry (LoRa),
             logger (LittleFS), power (batería, servo apagado)
src/rx/      receptor: imprime HEX,RSSI,SNR por línea
src/native/  repetición de un CSV del simulador por los filtros
test/        pruebas Unity (native)
scripts/     scripts de PlatformIO
```

## Tareas
- `control` (núcleo 1, 100 Hz): IMU cada ciclo, BMP280 cada 4 (25 Hz), Mahony, Kalman,
  paquete. Envía a colas sin bloqueo (`xQueueOverwrite` para radio, `xQueueSend` con
  espera 0 para el log).
- `comms` (núcleo 0): LoRa sin bloqueo (`startTransmit` + interrupción DIO1), log en
  LittleFS, comandos serial.

## Comandos serial (115200)
`TX1` (1 Hz, por defecto), `TX5` (5 Hz, pruebas), `STAT`, `DUMP` (log en hex), `BORRAR`.
