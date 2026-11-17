# Firmware CanSat Hellmary

PlatformIO, placa `heltec_wifi_lora_32_V3` (ESP32-S3 + SX1262), framework Arduino, RadioLib.
Fuente de verdad: `docs/SPEC.md`. Dudas y suposiciones: `docs/PREGUNTAS.md`.

Etapa actual: sensores, filtros, telemetría, log, **máquina de estados (0 a 6)**, servo de
liberación, NVS (reinicio en vuelo), modo simulación por serial y modo `DEMO_SEPARACION`.

## Entornos
| Entorno | Qué es | Comando (desde `firmware/`) |
|---|---|---|
| `heltec_wifi_lora_32_V3` | vuelo | `pio run -e heltec_wifi_lora_32_V3` |
| `heltec_demo` | vuelo + `DEMO_SEPARACION` (video de aceptación) | `pio run -e heltec_demo` |
| `rx_station` | receptor de la estación terrena | `pio run -e rx_station -t upload` |
| `native` | pruebas en PC | `pio test -e native` |

En Windows, `pio` está en el `.venv` del proyecto: `..\.venv\Scripts\pio.exe`.
El entorno `native` usa `g++` si existe; si no, el compilador de `zig` (`pip install ziglang`
en el `.venv`). Las pruebas generan sus datos con `tools/flight_sim.py --raw-csv`.

Los entornos de vuelo y demo **no compilan** mientras haya pines (o el nivel de encendido
del servo) `TODO_CONFIRMAR` en `include/config.h` (#error intencional). Para comprobar que el resto compila:
`set PLATFORMIO_BUILD_FLAGS=-DPERMITIR_PINES_PENDIENTES` (cmd) antes de `pio run`.
Nunca subir a la placa un binario compilado así.

## Estructura
```
include/  config.h (parámetros, umbrales SM_* y pines), packet.h, lora_setup.h
lib/      attitude (Mahony + sesgo del giroscopio), estimator (Kalman h/vz + p0),
          telemetry (armado del paquete), state_machine (estados 0-6 + modo demo),
          servo_release (secuencia -10° -> -80° con reintentos),
          flight_core (lazo de control completo + formato del modo simulación)
          -> portables, probados en native
src/flight/  main.cpp (tareas, modos, comandos), sensors (MPU6500 propio + BMP280),
             telemetry (LoRa), logger (LittleFS), power (batería, reed),
             servo_hw (energía + PWM + pull-down), mission_store (NVS)
src/rx/      receptor: imprime HEX,RSSI,SNR por línea
src/native/  repetición de un CSV del simulador por los filtros
test/        pruebas Unity (native): test_flight = escenarios de vuelo con
             tools/flight_sim.py; test_state_machine = casos sintéticos
scripts/     scripts de PlatformIO
```

## Tareas
- `control` (núcleo 1, 100 Hz): IMU cada ciclo, BMP280 cada 4 (25 Hz) y
  `FlightCore::step` (Mahony, Kalman, fallas, máquina de estados); aplica la orden del
  servo y arma el paquete. Envía a colas sin bloqueo (`xQueueOverwrite` para radio y
  NVS, `xQueueSend` con espera 0 para el log). En modo simulación avanza un ciclo por
  línea `SIM,...` recibida (ver `docs/sim_input.md`).
- `comms` (núcleo 0): LoRa sin bloqueo (`startTransmit` + interrupción DIO1), log en
  LittleFS, escritura de NVS, comandos serial.

## Tasas por estado
| Estado | TX | Log | Servo |
|---|---|---|---|
| 0 AUTOPRUEBA, 1 TIERRA | 1 Hz (5 Hz con `TX5`) | 1 Hz | sin energía |
| 2 ASCENSO | 2 Hz | 50 Hz | sin energía |
| 3 CAIDA | 5 Hz | 50 Hz | alimentado en -10° |
| 4 DESPLIEGUE | 5 Hz | 50 Hz | -80°, reintentos |
| 5 DESCENSO | 5 Hz | 50 Hz | sin energía |
| 6 ATERRIZADO | 2 Hz 60 s, luego cada 5 s | 1 Hz | sin energía |

## Comandos serial (115200)
`TX1` (según el estado, por defecto), `TX5` (5 Hz en tierra, pruebas), `STAT`, `DUMP`
(log en hex), `BORRAR`, `MISION` (registro de NVS), `MISION_BORRAR` (borra la misión y
reinicia), `SIM_FIN`, y en `heltec_demo`: `ARMAR`, `DESARMAR`. Las líneas `SIM,...` activan
el modo simulación (solo en estados 0 y 1).

Mensajes de la placa: `# ESTADO n banderas=0x..` en cada cambio de estado, `# DISPARO`
(o `# SEPARACION` en demo) con su `t_ms`, y `# REINICIO EN VUELO` al recuperar una misión.
