# Project Hellmary — sistemas del CanSat

CanSat de dos etapas con recuperación por autogiro, desarrollado para el concurso
**PEUUM CanSat 2026** por el equipo de la **Universidad de Montemorelos**. Este repositorio
contiene únicamente los **sistemas**: firmware de la placa, protocolo de telemetría,
estación terrena y herramientas de apoyo. El CAD y la electrónica se manejan aparte.

| Componente | Descripción |
|---|---|
| Placa | Heltec WiFi LoRa 32 V3 (ESP32-S3 + radio SX1262) |
| IMU | MPU6500 (6 ejes, sin magnetómetro) |
| Barómetro | BMP280 (presión, temperatura y altura) |
| Servo | MG90S (libera el autogiro) |
| Radio | LoRa a 915 MHz (SF7, ancho de banda 250 kHz) |

Alimentación: pila cuadrada de 9 V con interruptor deslizable y regulador a 5 V. No hay GPS.

## Qué hace el software

- **Máquina de estados de 7 estados:** 0 AUTOPRUEBA, 1 TIERRA, 2 ASCENSO, 3 CAIDA,
  4 DESPLIEGUE, 5 DESCENSO y 6 ATERRIZADO. Detecta la liberación, dispara el servo a la
  altura de despliegue y confirma el aterrizaje (`docs/SPEC.md`).
- **Telemetría LoRa de 32 bytes:** estado, secuencia, tiempo, presión, temperatura, altura,
  velocidad vertical, aceleración, actitud, banderas y batería (`protocol/packet.md`).
- **Estación terrena web:** recibe los paquetes de una segunda Heltec, los muestra en
  tiempo real y graba cada sesión en CSV (`ground/README.md`).
- **Reporte de vuelo:** gráficas y tabla de resultados de una sesión grabada
  (`tools/reporte_vuelo.py`).

## Estructura del repositorio

```
docs/       especificación, pruebas, preguntas abiertas y formato del modo simulación
protocol/   formato del paquete de telemetría (packet.md) y su codificación en Python
firmware/   código de la placa (PlatformIO): lib/ portable, src/ de la placa y del receptor
ground/     estación terrena (servidor FastAPI + interfaz web) y sus pruebas
tools/      simulador de vuelo, monitor serial, reporte de vuelo y utilidades
sim_csv/    vuelos simulados para el modo simulación (ignorada por git; ver docs/PRUEBAS_SISTEMAS.md §1.6)
```

## Instalación

Se necesita Python 3 y, para el firmware, PlatformIO (viene en `requirements.txt`).

**Linux (bash), desde la raíz del repositorio:**
```bash
python -m venv .venv                          # solo la primera vez
.venv/bin/python -m pip install -r requirements.txt
source .venv/bin/activate                     # deja python y pio del proyecto en esta terminal
```

**Windows (PowerShell):**
```powershell
py -m venv .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
```
En Windows se usan `.\.venv\Scripts\python` y `.\.venv\Scripts\pio.exe` en lugar de
`python` y `pio`.

### Puertos y `ambiente.sh` (Linux)
Cada placa aparece como un puerto serie: `/dev/ttyUSB0`, `/dev/ttyUSB1`... en Linux y
`COM5`, `COM6`... en Windows. Para verlos: `python ground/server.py --list-ports` (o el
Administrador de dispositivos en Windows). Conviene conectar una placa a la vez para saber
cuál es cuál.

Para no repetir el puerto en cada comando, cada integrante crea su propio `ambiente.sh` en
la raíz, con sus puertos:
```bash
source .venv/bin/activate
export RX=/dev/ttyUSB0   # receptora (rx_station)
export TX=/dev/ttyUSB1   # emisora (la placa que va en el CanSat o en el banco)
```
y lo carga con `source ambiente.sh`. Ese archivo es personal y **está en `.gitignore`**:
no se sube al repositorio.

## Uso rápido

Todos los comandos se escriben desde la raíz del repositorio.

**Pruebas** (conviene correrlas antes de cada commit):
```bash
python -m pytest ground/tests tools/tests protocol/tests -q    # Python
pio test -d firmware -e native                                 # lógica del firmware en la PC
```

**Compilar y cargar el firmware** (`-d firmware` indica la carpeta del proyecto PlatformIO):
```bash
pio run -d firmware -e heltec_wifi_lora_32_V3 -t upload --upload-port $TX   # VUELO
pio run -d firmware -e heltec_banco -t upload --upload-port $TX             # banco (NUNCA para vuelo)
pio run -d firmware -e heltec_demo -t upload --upload-port $TX              # video de aceptación
pio run -d firmware -e rx_station -t upload --upload-port $RX               # receptora
```
Sin `-t upload` solo compila. Descripción de cada entorno: `firmware/README.md`.

**Preparar la placa de vuelo con un solo comando** (`tools/cargar_vuelo.sh`):
```bash
tools/cargar_vuelo.sh              # usa /dev/ttyUSB* solo si hay exactamente uno
tools/cargar_vuelo.sh /dev/ttyUSB1 # o indica el puerto de la placa de vuelo
```
Compila y carga `heltec_wifi_lora_32_V3`, espera el reinicio y manda `MISION_BORRAR` (espera
5 s), `BORRAR`, `MODO B` y `STAT`. Verifica que `STAT` diga `despliegue=B`, que `imu_err` y
`baro_err` sean 0 y que el arranque no diga `IMU FALLA` ni `BARO FALLA`; imprime cada
revisión como OK o FALLA (en rojo) y termina con «Placa lista para vuelo en MODO B» o el
motivo, con código de salida distinto de 0 si algo falla. Con cero o varios puertos avisa y
sale sin tocar nada. Requiere la placa **con sensores conectados** (sin ellos falla a propósito).

**Estación terrena** (abrir luego `http://127.0.0.1:8000`, o el puerto elegido):
```bash
python ground/server.py --port $RX --auto-record --web-port 8000   # placa receptora real
python ground/server.py --sim                                       # vuelo simulado, sin placa
```
`--auto-record` empieza a grabar al arrancar; sin él se graba con el botón *INICIAR TOMA DE
DATOS*. Las sesiones quedan en `ground/data/`.

**Monitor serial** (`tools/monitor.py`) y **comandos sueltos** (`tools/cmd_serial.py`):
```bash
python tools/monitor.py --port $TX          # interactivo; solo envía la línea completa al pulsar Enter
python tools/cmd_serial.py STAT "MODO B"    # envía cada argumento como comando y muestra la respuesta
```
Salir del monitor: `/salir` o Ctrl+C. Un solo programa puede tener abierto un puerto: se
cierra el monitor antes de abrir la estación terrena o `sim_serial.py` en el mismo puerto.

**Reporte de vuelo:**
```bash
python tools/reporte_vuelo.py                                     # sesión más reciente de ground/data
python tools/reporte_vuelo.py ground/data/sesion_XXXX.csv --pdf   # una sesión concreta, con PDF
python tools/reporte_vuelo.py ground/data/sesion_XXXX.csv --log volcado.txt   # con el log de la placa (DUMP)
```
Crea una carpeta `reporte_<fecha>/` con gráficas, `reporte.md` y `reporte.html`. También se
genera con el botón *Generar reporte* de la estación terrena.

## Modos de despliegue

| Modo | Uso | Altura de suelta → autogiro |
|---|---|---|
| **B** | **Vuelo real** (por defecto) | ~100 m → 80 m |
| **A** | Reglamento anterior | ~450 m → 200 m |
| **C** | Pruebas con tinaco: dispara al detectar la caída libre, sin altura mínima | — |

El modo se cambia con `MODO A`, `MODO B` o `MODO C` (solo en los estados 0 y 1), se guarda
en la memoria de la placa y **persiste entre reinicios**: una placa que quedó en A sigue en
A hasta mandar `MODO B`. Para cambiarlo y confirmarlo:
```bash
python tools/cmd_serial.py "MODO B" STAT      # STAT muestra «despliegue=B ...» con sus umbrales
```

## Comandos de la placa (serial, 115200 baudios)

| Comando | Efecto |
|---|---|
| `MODO [A\|B\|C]` | muestra el modo de despliegue, o lo cambia |
| `STAT` | estadísticas y estado actual |
| `BORRAR` | vacía el log interno |
| `DUMP` | vuelca el log interno en hexadecimal |
| `MISION` / `MISION_BORRAR` | muestra / borra el registro de misión guardado (borrar reinicia la placa) |
| `TX1` / `TX5` | frecuencia de transmisión según el estado (por defecto) / 5 Hz en tierra, para pruebas |
| `ARMAR` / `DESARMAR` | solo modo demo: inicia / cancela la cuenta regresiva del servo (10 s) |
| `SIM_INICIO`, `SIM_FIN` | entran / salen del modo simulación (`docs/sim_input.md`) |

Otros comandos de prueba (`MISION_PRUEBA`, `SIM_PROXIMA`) están en `firmware/README.md`.
En el modo demo, mantener presionado el botón PRG 1 s equivale a `ARMAR`. **Antes de cada
vuelo o prueba se ejecutan `MISION_BORRAR`, `BORRAR`, `MODO B` y `STAT`** (checklist en
`docs/PRUEBAS_SISTEMAS.md`, sección 1.4b).

## El dashboard de la estación terrena

- **Indicador de enlace, RSSI, SNR, pérdida y paquetes:** calidad de la señal; *ENLACE
  PERDIDO* aparece tras 3 s sin paquetes.
- **Tiempo de misión:** cronómetro con el reloj de la placa.
- **Línea de estados:** los 7 estados con la hora en que se entró a cada uno.
- **Tarjetas:** altura, velocidad vertical (verde si el descenso está entre 8 y 12 m/s),
  altura máxima, presión, temperatura y batería.
- **Gráficas:** altura, velocidad vertical, aceleración y actitud; ventanas de 30 s, 2 min,
  5 min o todo, y *Pausar* (solo congela la vista; los datos siguen llegando y grabándose).
- **Actitud del cuerpo:** horizonte artificial con roll, pitch, yaw (relativo) e inclinación.
- **Banderas y registro de eventos:** base lista, liberación, disparo, aterrizaje, fallos de
  sensores y NO-GO; cada evento con su hora.
- **Botones del encabezado:** iniciar/finalizar toma de datos, exportar CSV, generar
  reporte y modo video (`V`).

## Advertencias

- **No usar `heltec_banco` ni `heltec_demo` para vuelo.** Son del modo demo (el servo se
  dispara con `ARMAR` o el botón PRG) y `heltec_banco` no mide la pila. Para vuelo, solo
  `heltec_wifi_lora_32_V3`.
- **No presionar el botón PRG al encender la placa:** GPIO0 es un pin de arranque.
- **El servo se alimenta desde el riel de 5 V del regulador (MP1584), no desde el pin 5V ni
  el USB de la Heltec, y todos los GND van unidos (GND común).** Esto debe confirmarse con
  electrónica. Hay un riesgo documentado de giro al arrancar (`docs/PREGUNTAS.md`, P30).
- **No subir `ambiente.sh` ni los datos de `ground/data/`**: están en `.gitignore`.
- Este repositorio es solo de sistemas: el CAD y la electrónica se manejan aparte.

## Documentación (`docs/`)

| Archivo | Contenido |
|---|---|
| [SPEC.md](docs/SPEC.md) | especificación: reglas del concurso, estados, fallas y modos |
| [PRUEBAS_SISTEMAS.md](docs/PRUEBAS_SISTEMAS.md) | guía de pruebas con dos Heltec, sin sensores ni servo |
| [PRUEBA_VIDEO.md](docs/PRUEBA_VIDEO.md) | procedimiento del video de aceptación de la separación |
| [sim_input.md](docs/sim_input.md) | formato del modo simulación por serial |
| [PREGUNTAS.md](docs/PREGUNTAS.md) | dudas, suposiciones y decisiones tomadas |

Otros documentos: [protocol/packet.md](protocol/packet.md) (paquete de 32 bytes),
[firmware/README.md](firmware/README.md) y [ground/README.md](ground/README.md).
