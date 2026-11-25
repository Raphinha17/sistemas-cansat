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

Las librerías están fijadas en `platformio.ini`; compilar una vez con internet las deja en
`.pio/libdeps` para usarlas sin red.

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

## Señales de vida (LED y OLED)
Solo `heltec_wifi_lora_32_V3` y `rx_station` (en `heltec_demo`/`heltec_banco` el LED conserva
su patrón de la cuenta regresiva y no hay pantalla). La lógica es pura y está en
`lib/indicadores` (probada en native: `test_indicadores`); el manejo de la pantalla en
`src/common/oled_hw.*` (Adafruit SSD1306 + GFX sobre `Wire1`; pines `SDA_OLED` 17, `SCL_OLED` 18, `RST_OLED` 21 y
`Vext` 36 del variant, verificados con `static_assert`; Vext en LOW alimenta la OLED). El
panel es el de 0.49" de la Wireless Stick V3: **64x32 píxeles visibles** centrados en las
128 columnas del SSD1306 (`OLED_ANCHO`, `OLED_ALTO` y `OLED_DESPLAZ_X` = 32 en `config.h`;
la librería aplica el desplazamiento al usar un panel de 64 de ancho). Antes se dibujaba
para 128x64 y el texto se cortaba por la izquierda. El LED y la pantalla se
manejan en la tarea `comms` (núcleo 0); el lazo de control de 100 Hz no participa.

| Estado de la placa | LED (GPIO35) |
|---|---|
| 0 AUTOPRUEBA | parpadeo muy rápido (10 Hz) |
| 1 TIERRA sin fallos (banderas 5, 6 y 7 en 0) | doble destello cada 2 s |
| 1 TIERRA con fallo de barómetro, de IMU o NO-GO | parpadeo rápido (5 Hz) |
| 2 a 5 (ASCENSO, CAIDA, DESPLIEGUE, DESCENSO) | apagado |
| 6 ATERRIZADO | un destello por segundo |

Texto de las pantallas (fuente 6x8: 10 caracteres por línea, 4 líneas; probado en native
con un framebuffer de 64x32 que ningún píxel sale del área visible):
- Vuelo (≤ 2 Hz, solo estados 0, 1 y 6; en 2 a 5 se manda un único `DISPLAYOFF` al entrar y no
  se vuelve a escribir): `GO` / `NO-GO` en tamaño 2 (NO-GO si las banderas 5, 6 o 7 están en 1,
  en vídeo inverso), `MODO B  S1` e `I:OK B:OK` (`NO` si falla).
- `rx_station` (2 Hz): `RX n`, `RSSI/SNRdB` (p. ej. `-45/12dB`), `ST1 H0.0m` (altura con un
  decimal si |h| < 10 m) y `ULT 2s`, o `SIN ENLACE` (> 5 s sin paquetes o aún sin ninguno).
Sin pantalla, `oled::begin()` devuelve false (no responde el 0x3C), el arranque lo imprime y
todo sigue sin ella.

**Librerías con versión fija** (sin `^`, para poder compilar sin internet una vez
descargadas en `.pio/libdeps`): RadioLib 7.8.1, Adafruit BMP280 2.6.8, Unified Sensor 1.1.15,
BusIO 1.17.4, SSD1306 2.5.17 y GFX 1.12.6. Tamaño (vuelo): RAM 6.6 % (21.7 kB de 327 kB),
flash 12.9 % (431 kB de 3.3 MB); `rx_station`: RAM 6.2 %, flash 10.0 %.

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
| **D** (tinaco, solo carga primaria) | igual que C | — | **ninguno**: servo inhibido (sin energía, sin intentos, sin bandera de disparo) | — |

`MODO A|B|C|D` lo cambia (solo en estados 0 y 1), se guarda en NVS y queda en el log
(registro de evento en cada arranque y cambio). `MODO`, `STAT` y el arranque muestran el
modo activo y sus umbrales; el arranque avisa (`# AVISO: modo A activo...`) si el modo
guardado no es el B. **Una placa que ya tenía `MODO A` guardado sigue en A hasta mandar
`MODO B`.** Parámetros en `config.h` (`DEPLOY_*`); dudas en `docs/PREGUNTAS.md` P31-P33 y
P57-P59.

**Modo D** (tinaco solo carga primaria): recorre CAIDA → DESPLIEGUE → DESCENSO → ATERRIZADO
con la misma detección que C y los mismos eventos de estado en la telemetría y el log, pero
`ServoCmd` queda siempre sin energía, no se arranca la secuencia del servo (0 intentos, sin
confirmación), `FLAG_DISPARO` y `FLAG_DISPARO_RESPALDO` nunca se encienden y el demo
(`ARMAR`, botón PRG) no arma. `STAT` imprime `despliegue=D [tinaco: solo carga primaria,
SIN disparo]`. Prueba nativa: `test_modo_d_nunca_dispara` (`test_state_machine`). Scripts
para ponerlo y confirmarlo: `tools/tinaco_primaria.sh` (y `tinaco_separacion.sh` para C,
`modo_dron.sh` para B).

p0 en TIERRA: el promedio móvil solo toma muestras con el CanSat quieto en el suelo
(|h| < 3 m y |vz| < 0.5 m/s); al levantarlo se pausa (hasta 300 s) para que un ascenso
lento de dron no lo arrastre (`STAT`: `p0_pausa=1`). P57.

## Comandos serial (115200)
`TX1` (según el estado, por defecto), `TX5` (5 Hz en tierra, pruebas), `STAT`, `I2C` (diagnóstico del bus de sensores, solo estados 0 y 1: nivel en reposo de SDA 41 / SCL 42 con pull-up, escaneo 0x08–0x77 normal y con SDA/SCL cruzados, y un veredicto «OK: IMU y BARO detectados» o la causa probable; el control reutiliza la última muestra durante el escaneo, no cambia el estado, y el bus se restaura al terminar), `DUMP`
(log en hex), `BORRAR`, `MISION` (registro de NVS), `MISION_BORRAR` (borra la misión y
reinicia), `MISION_PRUEBA` (guarda una misión FALSA en curso, para probar el reinicio en
vuelo sin volar), `MODO [A|B|C|D]`, `SIM_INICIO` (entra al modo simulación, o lo reinicia de
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
