# Firmware CanSat Hellmary

PlatformIO, placa `heltec_wifi_lora_32_V3` (ESP32-S3 + SX1262), framework Arduino, RadioLib.
Fuente de verdad: `docs/SPEC.md`. Dudas y suposiciones: `docs/PREGUNTAS.md`.

Etapa actual: sensores, filtros, telemetría, log, **máquina de estados (0 a 6)**, servo de
liberación, NVS (reinicio en vuelo), **modos de despliegue B (lanzamiento real, por
defecto)/A/C**, modo simulación por
serial y modo `DEMO_SEPARACION` (armado por serial o botón PRG). Procedimiento para el
video de aceptación: `docs/PRUEBA_VIDEO.md`.

Encendido: interruptor deslizable en la línea de la pila de 9 V. Al encender, la placa
arranca y entra sola a AUTOPRUEBA -> TIERRA (sin armado); no hay reed ni enclavamiento.

## Entornos
| Entorno | Qué es | Comando (desde `firmware/`) |
|---|---|---|
| `heltec_wifi_lora_32_V3` | vuelo | `pio run -e heltec_wifi_lora_32_V3` |
| `heltec_demo` | vuelo + `DEMO_SEPARACION` (video de aceptación) | `pio run -e heltec_demo` |
| `heltec_banco` | banco: modo demo y sin medición de batería (**NUNCA PARA VUELO**) | `pio run -e heltec_banco -t upload` |
| `rx_station` | receptor de la estación terrena | `pio run -e rx_station -t upload` |
| `native` | pruebas en PC | `pio test -e native` |

`pio` está en el `.venv` del proyecto: en Linux `../.venv/bin/pio` (o `source
../.venv/bin/activate` y luego `pio`), en Windows `..\.venv\Scripts\pio.exe`. Puertos: Linux
`/dev/ttyUSB0`, `/dev/ttyUSB1`... (`--upload-port /dev/ttyUSB1`); Windows `COM5`, `COM6`...
El entorno `native` usa `g++` si existe; si no, el compilador de `zig` (`pip install ziglang`
en el `.venv`). Las pruebas generan sus datos con `tools/flight_sim.py --raw-csv`.

Pines (`include/config.h`): I2C SDA 41 / SCL 42 y señal del servo 7, confirmados. El
servo se alimenta desde el riel de 5 V del regulador (MP1584), no desde el pin 5V ni el
USB de la Heltec, con GND común (por confirmar con electrónica), y no tiene pin de energía
(`SERVO_POWER_ENABLED 0`): solo se genera PWM desde el
estado 3 (riesgo documentado en `docs/PREGUNTAS.md` P30). El ADC de la pila es opcional
(`BATT_ADC_ENABLED 0` por defecto: batería 0 en el paquete). Los entornos de la placa
tienen un `#error` intencional si falta el pin de una función activa (p. ej. el ADC con
`BATT_ADC_ENABLED 1` y `PIN_BATT_ADC` en `TODO_CONFIRMAR`); con la configuración actual
compilan los tres.

> **AVISO: `heltec_banco` NUNCA debe usarse para vuelo.** Es el firmware del modo demo
> (`DEMO_SEPARACION`: el servo se dispara con `ARMAR` o el botón PRG) y nunca mide la
> pila (el paquete reporta batería 0 aunque se active `BATT_ADC_ENABLED`).

## Estructura
```
include/  config.h (parámetros, umbrales SM_* y pines), packet.h, lora_setup.h
lib/      attitude (Mahony + sesgo del giroscopio), estimator (Kalman h/vz + p0),
          telemetry (armado del paquete), state_machine (estados 0-6 + modo demo),
          servo_release (secuencia -10° -> -80° con reintentos),
          flight_core (lazo de control completo + formato del modo simulación)
          -> portables, probados en native
src/flight/  main.cpp (tareas, modos, comandos), sensors (MPU6500 propio + BMP280),
             telemetry (LoRa), logger (LittleFS), power (batería opcional),
             servo_hw (energía + PWM + pull-down), mission_store (NVS: misión y modo)
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

## Modos de despliegue
| Modo | Liberación válida | Detector | Disparo | Respaldos |
|---|---|---|---|---|
| **B** (por defecto, lanzamiento real: suelta ~100 m) | h > 85 m | baja g 100 ms + vz < -5 m/s en 0.6 s; o vz < -8 m/s 0.2 s; o 8 m bajo h_max bajando a > 5 m/s | h ≤ 80 + \|vz\|·0.5 y compuerta 45°; inmediato si la liberación llega tarde | compuerta 0.3 s, 70 m; sin baro 1.0 s |
| **A** (reglamento anterior) | h > 240 m (~450 m) | baja g 150 ms + vz < -8 m/s en 1 s; o vz < -12 m/s 0.4 s; o 25 m bajo h_max | h ≤ 200 + \|vz\|·0.5 y compuerta 45° | compuerta 1.5 s, 160 m; sin baro 7 s |
| **C** (tinaco) | caída libre 150 ms, sin altura (también en TIERRA) | — | inmediato, sin compuerta | — |

`MODO A|B|C` lo cambia (solo en estados 0 y 1), se guarda en NVS y queda en el log
(registro de evento en cada arranque y cambio). `MODO`, `STAT` y el arranque muestran el
modo activo y sus umbrales; el arranque avisa (`# AVISO: modo A activo...`) si el modo
guardado no es el B. **Una placa que ya tenía `MODO A` guardado sigue en A hasta mandar
`MODO B`.** Parámetros en `config.h` (`DEPLOY_*`); dudas en `docs/PREGUNTAS.md` P31-P33 y
P57-P59.

p0 en TIERRA: el promedio móvil solo toma muestras con el CanSat quieto en el suelo
(|h| < 3 m y |vz| < 0.5 m/s); al levantarlo se pausa (hasta 300 s) para que un ascenso
lento de dron no lo arrastre (`STAT`: `p0_pausa=1`). P57.

## Comandos serial (115200)
`TX1` (según el estado, por defecto), `TX5` (5 Hz en tierra, pruebas), `STAT`, `DUMP`
(log en hex), `BORRAR`, `MISION` (registro de NVS), `MISION_BORRAR` (borra la misión y
reinicia), `MISION_PRUEBA` (guarda una misión FALSA en curso, para probar el reinicio en
vuelo sin volar), `MODO [A|B|C]`, `SIM_INICIO` (entra al modo simulación, o lo reinicia de
cero, sin depender de los sensores), `SIM_FIN`, `SIM_PROXIMA` (próxima línea SIM esperada),
y en `heltec_demo`/`heltec_banco`: `ARMAR`, `DESARMAR`. Las líneas `SIM,...` también activan
el modo simulación (solo en estados 0 y 1). Con CRC (`SIM,...*HHHH`, como las manda
`tools/sim_serial.py`) la placa descarta las dañadas o fuera de orden y pide `SIM_REPITE t_ms`
(`docs/sim_input.md`, "Integridad").

**Placa sin sensores** (banco con solo las Heltec): IMU y barómetro fallan en la
autoprueba, la placa llega a TIERRA con NO-GO y fallos de ambos sensores (banderas 0xE0) y
se queda ahí transmitiendo a 1 Hz; no pasa a ASCENSO ni guarda misión. El modo simulación
funciona igual (guía: `docs/PRUEBAS_SISTEMAS.md`).

**`MISION_BORRAR` antes de cada vuelo o prueba**: un corte de corriente (o apagar con el
interruptor) a mitad de una misión deja el registro en NVS y el siguiente arranque lo
toma como reinicio en vuelo.

Modo demo: además de `ARMAR`, mantener presionado el botón PRG 1 s arma la cuenta
regresiva (LED de la placa parpadeando rápido); otro toque largo antes de terminar
cancela; tras el disparo el LED queda fijo. No presionar PRG al encender (GPIO0 es pin
de arranque). El firmware de vuelo normal no usa el botón ni el LED.

Mensajes de la placa: `# ESTADO n banderas=0x.. t_ms=..` en cada cambio de estado (el
`t_ms` es el del ciclo en que lo imprime la tarea de comunicaciones, ±15 ms),
`# DISPARO t_ms=.. h=.. vz=.. h_despliegue=..` (o `# SEPARACION` en demo; h y vz estimados
en el ciclo del disparo y h_despliegue = h - |vz|*0.5, la altura donde actúa el autogiro),
`# DEMO ARMADO t_ms=..` / `# DEMO CANCELADO`,
`OK modo X` al cambiar de modo y `# REINICIO EN VUELO` al recuperar una misión.
