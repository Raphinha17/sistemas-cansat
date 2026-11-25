# Pruebas de sistemas con dos Heltec V3 (sin sensores ni servo)

Guía paso a paso de las pruebas de sistema que se pueden hacer con solo **dos placas
Heltec WiFi LoRa 32 V3**, sin MPU6500, BMP280 ni servo, para **Linux** (bash; probado en
Arch) y **Windows** (PowerShell). El firmware de vuelo se alimenta con el **modo
simulación por serial** (`docs/sim_input.md`): una PC manda vuelos de `tools/flight_sim.py`
y la placa los procesa como si vinieran de los sensores. Dudas y supuestos:
`docs/PREGUNTAS.md` (P44 a P49 y P57 a P59).

**Reglamento del lanzamiento (vigente):** el dron sube el CanSat a **~100 m** sobre el suelo
de despegue y lo suelta; autogiro y separación a **80 m**. Es el **modo B**, el de por
defecto del firmware. El modo A (450 m → 200 m) queda como reglamento anterior y el C para
el tinaco. Las pruebas (b) a (g) usan el modo B.

## 0. Qué se prueba y qué no

| Con esta guía SÍ se prueba | NO se prueba (hace falta hardware) |
|---|---|
| Máquina de estados real en la placa: liberación, disparo, descenso, aterrizaje | Lectura real del MPU6500 y del BMP280 (I2C) |
| Modos B, A y C; sueltas a 90-105 m; fallas de baro e IMU; liberación falsa | Calibración del giroscopio con sensor real, ruido y vibración reales del dron |
| Telemetría LoRa real entre las dos placas y la estación terrena | Servo: giro de -10° a -80°, confirmación por pico de 2 g |
| Armado por ARMAR y por el botón PRG, cuenta de 10 s, LED | Mecanismo de liberación del autogiro |
| Reinicio en vuelo (NVS) y MISION_BORRAR | Batería de 9 V y su medición |
| Alcance del radio (RSSI, SNR, pérdida) | Vuelo real (viento, vibración, antena en el aire) |

**Placa sin sensores.** Al arrancar, IMU y barómetro fallan en la autoprueba. La placa llega
a TIERRA con las banderas **NO-GO + fallo baro + fallo IMU (0xE0)**, transmite a 1 Hz y
**se queda en TIERRA** (no pasa a ASCENSO ni guarda una misión en NVS). El modo simulación
se activa igual que con sensores (comando `SIM_INICIO` o la primera línea `SIM,...`).

> **No se probó en una placa real**: el comportamiento sin sensores y todos los números de
> esta guía salen de las pruebas native (`pio test -e native`, mismo código que la placa;
> los de las pruebas (b) a (g) los imprime `test_sim_link`, `test_guia_pruebas_sistemas`,
> con el mismo texto con CRC que manda `sim_serial.py`) y de leer el firmware. Si algo no
> coincide en la placa, anótalo en la tabla de la sección 3 y avisa: puede ser un error de
> esta guía.

## 1. Preparación (una sola vez)

Todos los comandos se escriben **desde la raíz del repositorio** (`sistemas-cansat/`).
Llamaremos **placa TX** a la que lleva el firmware de vuelo/banco (transmite) y **placa
RX** a la receptora (`rx_station`). (Antes se llamaban "placa A" y "placa B"; se
renombraron para no confundirlas con los modos A y B.)

| | Linux (bash) | Windows (PowerShell) |
|---|---|---|
| Python del proyecto | `python` tras `source .venv/bin/activate` (o `.venv/bin/python`) | `& $PY` (`$PY = ".\.venv\Scripts\python.exe"`) |
| PlatformIO | `pio` tras activar el `.venv` (o `.venv/bin/pio`) | `& $PIO` (`$PIO = ".\.venv\Scripts\pio.exe"`) |
| Puerto placa TX | `$TX` (p. ej. `/dev/ttyUSB1`) | `COM5` |
| Puerto placa RX | `$RX` (p. ej. `/dev/ttyUSB0`) | `COM6` |
| Rutas | `sim_csv/B_nominal.csv` | `sim_csv\B_nominal.csv` |

### 1.1 Entorno de Python
```bash
# Linux
python -m venv .venv                              # solo si no existe .venv
.venv/bin/python -m pip install -r requirements.txt
source .venv/bin/activate                         # python y pio del proyecto en esta terminal
export TX=/dev/ttyUSB1 RX=/dev/ttyUSB0            # cambia por tus puertos (§1.2)
```
```powershell
# Windows
py -m venv .venv                                  # solo si no existe .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
$PY  = ".\.venv\Scripts\python.exe"               # atajos para esta ventana de PowerShell
$PIO = ".\.venv\Scripts\pio.exe"
```
Los atajos (`source`, `export`, `$PY`, `$PIO`) valen solo en la terminal donde los
escribes; si abres otra, vuelve a escribirlos. (En Linux, un `ambiente.sh` local con esas
líneas sirve: `source ambiente.sh`.)

### 1.2 Ver el puerto de cada placa
1. Conecta **una sola** placa por USB.
2. **Linux:** `ls /dev/ttyUSB*` (o `ls -l /dev/serial/by-id/`, que muestra "Silicon_Labs_CP2102..."
   apuntando a `ttyUSB0`). **Windows:** Administrador de dispositivos (Windows + X) →
   **Puertos (COM y LPT)** → "Silicon Labs CP210x USB to UART Bridge (COM5)".
3. Desconecta esa placa, conecta la otra y repite. Anota: **TX = ____, RX = ____**.
   (En esta guía: TX = `/dev/ttyUSB1` o `COM5`, RX = `/dev/ttyUSB0` o `COM6`.) En Linux el
   número depende del orden de conexión: conéctalas siempre en el mismo orden o usa las
   rutas de `/dev/serial/by-id/`.

También: `python ground/server.py --list-ports` (Windows: `& $PY ground\server.py
--list-ports` o `Get-PnpDevice -Class Ports`). Si no aparece ningún puerto: cable USB solo
de carga (usa uno de datos); en Windows, falta el driver CP210x. **Linux, "Permission
denied":** `sudo usermod -aG uucp $USER` (Arch; en Debian/Ubuntu el grupo es `dialout`) y
vuelve a iniciar sesión.

### 1.3 Subir el firmware
Antenas LoRa **conectadas** en las dos placas antes de energizarlas (transmitir sin antena
puede dañar el radio).
```bash
# Linux
pio run -d firmware -e heltec_banco -t upload --upload-port $TX   # TX: banco (NUNCA para vuelo)
pio run -d firmware -e rx_station -t upload --upload-port $RX     # RX: receptor
```
```powershell
# Windows
& $PIO run -d firmware -e heltec_banco -t upload --upload-port COM5
& $PIO run -d firmware -e rx_station -t upload --upload-port COM6
```
La primera compilación tarda varios minutos. Si el *upload* dice "Failed to connect": mantén
presionado **PRG**, toca **RST**, suelta PRG y repite el comando.

### 1.4 Monitor serie
```bash
pio device monitor -p $TX -b 115200 --filter time --echo           # Linux, placa TX
```
```powershell
& $PIO device monitor -p COM5 -b 115200 --filter time --echo       # Windows, placa TX
```
`--filter time` pone la hora de la PC en cada línea y `--echo` muestra lo que escribes.
Escribe un comando y pulsa Enter. Salir: **Ctrl + C**. Un solo programa puede tener abierto
un puerto: **cierra el monitor antes de correr `sim_serial.py` o la estación** en ese mismo
puerto. Si la placa se reinicia al abrir el monitor, espera 3 s.

**Monitor propio (`tools/monitor.py`, Linux):** alternativa que solo envía líneas
completas. Lo que llega se imprime en pantalla; lo que escribes **no se manda hasta pulsar
Enter** y entonces va todo junto, en mayúsculas y sin espacios sobrantes (`modo b` →
`MODO B`). Una línea vacía no envía nada; nunca se mandan caracteres sueltos. No reinicia
la placa al abrir (DTR y RTS apagados). Salir: `/salir` o Ctrl+C; si se desconecta la placa
avisa y sale sin traceback.
```bash
.venv/bin/python tools/monitor.py --port $TX      # --port por defecto: $TX o /dev/ttyUSB1
.venv/bin/python tools/monitor.py --port $RX --baud 115200
```
(`tools/cmd_serial.py` manda comandos sueltos y muestra la respuesta:
`.venv/bin/python tools/cmd_serial.py STAT "MODO B"`.)

Arranque esperado de la placa TX (sin sensores):
```
CanSat Hellmary - MODO DEMO_SEPARACION (disparo 10 s tras ARMAR o toque largo de PRG) - BANCO: ...
IMU FALLA (WHO_AM_I=0x00)  BARO FALLA
LittleFS OK
Log .../... B (.. %)
Radio OK (codigo 0)
NVS OK
Modo por defecto: despliegue=B [lanzamiento real: suelta ~100 m] liberacion valida > 85 m, autogiro a 80 m (h_disp = 80 + |vz|*0.5), respaldo 70 m, compuerta 45 grados max 0.3 s
  deteccion: baja g 100 ms + vz < -5 m/s en 0.6 s | vz < -8 m/s 0.2 s | 8 m bajo h_max con vz < -5 m/s | sin baro: disparo 1.0 s tras liberar | liberacion tardia: disparo inmediato
# ESTADO 1 banderas=0xE0 t_ms=...
```
(`IMU FALLA` y `BARO FALLA` son lo esperado sin sensores; puede aparecer antes un
`# ESTADO 0` de un ciclo.) Si la placa ya tenía un modo guardado en NVS dice `Modo
guardado: despliegue=A ...` y luego **`# AVISO: modo A activo; el lanzamiento real es el
modo B (manda MODO B)`**: una placa que antes estaba en A **sigue en A** hasta mandar
`MODO B`. Y de la placa RX:
```
# CanSat Hellmary - receptor rx_station
# radio OK, escuchando (0)
0100...,-31.5,9.75        <- una línea HEX,RSSI,SNR por paquete recibido
```

### 1.4a Preparar la placa de vuelo de una vez (`tools/cargar_vuelo.sh`, Linux)
```bash
tools/cargar_vuelo.sh [/dev/ttyUSB1]    # sin argumento: solo si hay un único /dev/ttyUSB*
```
Carga `heltec_wifi_lora_32_V3` y hace el checklist de 1.4b (`MISION_BORRAR` con 5 s de
espera, `BORRAR`, `MODO B`, `STAT`), verificando `despliegue=B`, `imu_err=0`, `baro_err=0` y
un arranque sin `IMU FALLA` ni `BARO FALLA`. Sale con código 0 y «Placa lista para vuelo en
MODO B» solo si todo está bien. **Es para la placa de vuelo con sensores**: en el banco sin
sensores (§0) siempre falla en IMU y BARO, que ahí es lo esperado; usa el checklist manual.

### 1.4b Antes de cada prueba (checklist: `MISION_BORRAR`, `BORRAR`, `MODO B`, `STAT`)
```
MISION_BORRAR     # borra el registro de misión de NVS y reinicia la placa
BORRAR            # vacía el log: "OK log vaciado (X B liberados)"; si no, se llena (100 %)
MODO B            # lanzamiento real; el modo se guarda en NVS: déjalo en B
STAT              # despliegue=B [...] > 85 m ... 80 m, estado=1, log=0/... (0 %)
```
(En el monitor de la placa TX. `sim_serial.py` ya manda `SIM_INICIO` y, con `--modo`,
`MODO`; no manda `MISION_BORRAR`.) Si el arranque dice `# REINICIO EN VUELO`, hay una
misión guardada: `MISION_BORRAR`.

### 1.5 Estación terrena
En **otra** terminal (con el `.venv` activado o `$PY` definido):
```bash
python ground/server.py --port $RX                 # Linux
```
```powershell
& $PY ground\server.py --port COM6                 # Windows
```
Abre **http://127.0.0.1:8000** en el navegador. Debe decir "ENLACE OK" en cuanto la placa
TX transmita. Detalles: `ground/README.md`. Para grabar una prueba: botón **INICIAR TOMA DE
DATOS** (y **FINALIZAR** al terminar); el CSV queda en `ground/data/`.

### 1.6 Vuelos simulados (CSV)
```bash
# Linux
mkdir -p sim_csv
python tools/flight_sim.py --raw-csv sim_csv/B_nominal.csv --seed 1 --mode B
python tools/flight_sim.py --raw-csv sim_csv/B_90.csv --seed 1 --mode B --release-alt 90
python tools/flight_sim.py --raw-csv sim_csv/B_95.csv --seed 1 --mode B --release-alt 95
python tools/flight_sim.py --raw-csv sim_csv/B_105.csv --seed 1 --mode B --release-alt 105
python tools/flight_sim.py --raw-csv sim_csv/B_inclinada.csv --seed 1 --mode B --fall-tilt 70
python tools/flight_sim.py --raw-csv sim_csv/B_lento.csv --seed 1 --mode B --v-climb 1.5 --t-hover 120 --vib 0.3
python tools/flight_sim.py --raw-csv sim_csv/B_falsa_liberacion.csv --seed 1 --mode B --false-release 70
python tools/flight_sim.py --raw-csv sim_csv/B_falla_baro.csv --seed 1 --mode B --fail baro --fail-at 45
python tools/flight_sim.py --raw-csv sim_csv/B_falla_imu.csv --seed 1 --mode B --fail imu --fail-at 45
python tools/flight_sim.py --raw-csv sim_csv/A_nominal.csv --seed 1 --mode A
python tools/flight_sim.py --raw-csv sim_csv/C_tinaco.csv --seed 1 --mode C
```
```powershell
# Windows: lo mismo con & $PY y "\" (ejemplo con los dos primeros)
New-Item -ItemType Directory -Force sim_csv | Out-Null
& $PY tools\flight_sim.py --raw-csv sim_csv\B_nominal.csv --seed 1 --mode B
& $PY tools\flight_sim.py --raw-csv sim_csv\B_90.csv --seed 1 --mode B --release-alt 90
```
Son los mismos argumentos que los casos de `firmware/scripts/gen_sim_data.py` (pruebas
native), así que los CSV son idénticos byte a byte. (`sim_csv/` está en `.gitignore`.)

| CSV | Líneas | Duración | Perfil |
|---|---:|---:|---|
| `B_nominal` | 14 887 | 2 min 29 s | 20 s en tierra, ascenso de dron a 3 m/s, 20 s estacionado a 100 m (±0.5 m), suelta a T+78.3 s |
| `B_90` / `B_95` / `B_105` | 14 511 / 14 699 / 15 073 | 2:25 / 2:27 / 2:31 | igual, suelta a 90 / 95 / 105 m |
| `B_inclinada` | 14 850 | 2:29 | B nominal cayendo inclinado ~70° |
| `B_lento` | 28 122 | 4:41 | ascenso a 1.5 m/s, **120 s** estacionado, vibración 0.3 g por eje |
| `B_falsa_liberacion` | 14 887 | 2:29 | B nominal + 0.25 s de caída libre del dron estacionado a T+70 s |
| `B_falla_baro` / `B_falla_imu` | 14 899 / 14 862 | 2:29 | sensor muerto a T+45 s (ascenso, ~65 m) |
| `A_nominal` | 19 391 | 3:14 | 450 m → 200 m (reglamento anterior) |
| `C_tinaco` | 9 196 | 1:32 | sube a 8 m y suelta |

### 1.7 Cómo se corre un vuelo simulado
Con la estación abierta y el monitor de la placa TX **cerrado**:
```bash
python tools/sim_serial.py --csv sim_csv/B_nominal.csv --port $TX --modo B        # Linux
```
```powershell
& $PY tools\sim_serial.py --csv sim_csv\B_nominal.csv --port COM5 --modo B        # Windows
```
`sim_serial.py` manda `SIM_INICIO`, `MODO B` y las líneas a tiempo real; imprime lo que
dice la placa con los segundos **de la PC** desde que abrió el puerto (`[  83.20 s] <
# DISPARO ...`, unos 3 s más que el tiempo de simulación **más** lo que retrasen los
reenvíos) y al final pide `STAT`. Los tiempos de la simulación (`t_ms` de los mensajes de
la placa y T+ de la estación) empiezan en 0 con la primera línea y **no** cambian con los
reenvíos. **No** cierres ni toques la placa durante el vuelo.

**Integridad del enlace (desde el 19-nov-2026).** El puerto serie de la placa pierde bloques
de bytes (~1 por segundo, mientras la flash guarda el log). Cada línea lleva CRC; la placa
descarta las dañadas o fuera de orden y pide `SIM_REPITE`, y `sim_serial.py` las reenvía
(detalles: `docs/sim_input.md`, "Integridad"). Al final de **cada** vuelo simulado,
`sim_serial.py` debe imprimir (con `B_nominal`):
```
lineas del CSV=14887 enviadas=14xxx (reenviadas=...) pedidos_de_reenvio=... retrocesos=... duracion=...
OK: la placa recibio las 14887 lineas en orden (SIM_PROXIMA=148880)
```
y el `STAT` final, en su última línea, `sim_rx=` **igual** a las líneas del CSV,
`sim_descartadas=0` y `sim_crc=1`. `sim_invalidas`, `sim_huecos`, `sim_repetidas` y
`sim_repite` pueden ser > 0 (pérdidas detectadas y reenviadas, ~1 por segundo). Si sale
`ERROR: la placa no confirmo...` o `sim_rx` no coincide, **la corrida no vale** (no la
cuentes como falla del vuelo): repítela. Con eso la placa procesa exactamente las mismas
lecturas que las pruebas native, y los números de las tablas deben coincidir:
**tiempos ±0.02 s** (`t_ms` de `# ESTADO 3` y de `# DISPARO`) y **h, vz y h_despliegue
del `# DISPARO` ±0.5 m**; el margen solo cubre diferencias de redondeo de punto flotante
entre la PC y el ESP32-S3. Si el enlace se confirmó pero no se cumple, es una diferencia
real entre la placa y native: anótala y avisa. (`# ESTADO` lo imprime la tarea de
comunicaciones: su `t_ms` puede ir hasta ~15 ms después del ciclo del cambio.)

Cada prueba pide leer, en `sim_serial`: los mensajes `# ESTADO`, `# DISPARO ...` y el
`STAT` final; y en la estación: la línea de tiempo, los eventos, las banderas y las
tarjetas. Los T+ de la estación se redondean hacia abajo al segundo.

---

## 2. Pruebas

"h_despliegue" es la altura **estimada a bordo** donde actúa el autogiro (`h - |vz|·0.5`,
la que imprime la placa en `# DISPARO`); el disparo ocurre cuando cruza 80 m, así que en
un disparo por altura sale ≈ 79.9. La altura **real** del vuelo simulado al actuar el
autogiro se verifica en `pio test -e native` (`test_flight`): 79.7 a 80.0 m en las sueltas
de 90 a 105 m.

### (a) Enlace entre las dos placas (rx_station)
**Objetivo:** comprobar que la telemetría LoRa llega de la placa TX a la RX y la estación la
muestra, sin sensores ni simulación.

1. Placa TX (banco) y placa RX (rx_station) cargadas, antenas puestas, separadas **1 a 2 m**.
2. Estación (§1.5); abre http://127.0.0.1:8000.
3. En la estación pulsa **INICIAR TOMA DE DATOS**, espera **2 minutos** y pulsa
   **FINALIZAR**.
4. Resumen: `python tools/resumen_enlace.py ground/data/sesion_*.csv` (Windows:
   `& $PY tools\resumen_enlace.py ground\data\sesion_*.csv`); la última fila es esta.

**Resultado esperado:**

| Dato | Esperado |
|---|---|
| Estado | TIERRA (1) resaltado, T+ avanza |
| Banderas | Fallo baro, Fallo IMU y NO-GO encendidas (0xE0); resto apagadas |
| Tarjetas | Altura 0.0 m, batería "N/D" |
| Paquetes en 2 min | 120 ±2 (1 Hz) |
| Pérdida | ≤ 2 % |
| Líneas inválidas (`techstats`) | 0 |
| RSSI / SNR a 1-2 m | RSSI > -90 dBm, SNR > 5 dB |

**Pasa** si se cumplen paquetes, pérdida, 0 inválidas y RSSI/SNR. **Falla** si no llega
nada (revisa antenas, `# radio OK` en la RX, que ambas usen el mismo `lora_setup.h`) o
si la pérdida > 2 %.

### (b) Vuelo simulado nominal, modo B (lanzamiento real: 100 m → 80 m)
**Objetivo:** vuelo completo: ascenso de dron, 20 s estacionado a 100 m, suelta, autogiro a
80 m, con la estación mostrando estados y disparo.
```bash
python tools/sim_serial.py --csv sim_csv/B_nominal.csv --port $TX --modo B
```
```powershell
& $PY tools\sim_serial.py --csv sim_csv\B_nominal.csv --port COM5 --modo B
```
Antes: estación abierta, **INICIAR TOMA DE DATOS** pulsado.

**Resultado esperado (semilla 1):**

| Evento | Esperado (tiempo de simulación) |
|---|---|
| `STAT` (primera línea) | `despliegue=B [lanzamiento real: suelta ~100 m] liberacion valida > 85 m, autogiro a 80 m ...` |
| Estados en orden | 0 → 1 → 2 → 3 → 4 → 5 → 6, sin retrocesos |
| ASCENSO | T+33.7 s (`t_ms` ≈ 33 670; el dron va por ~29 m: 20 m + 3 s a 3 m/s) |
| Estacionamiento a 100 m | T+58 s a T+78 s en ASCENSO (la estación muestra ~100 m y vz ≈ 0); **ningún** `# ESTADO 3` |
| CAÍDA (liberación) | T+78.80 s (la suelta es a 78.31 s: detectada **0.49 s** después) (estación T+01:18) |
| `# DISPARO` | `t_ms` = 79 930, `h=87.5 vz=-15.1` (1.13 s después de la liberación) |
| **h_despliegue** | **80 ±3 m** (≈ **79.9**) |
| ATERRIZADO | T+92.05 s |
| Altura máxima | 100.5 m (tarjeta: ~100 m) |
| vz en DESCENSO | ≈ -10 m/s, tarjeta **verde** |
| Banderas al final | 0x17: base lista, liberación, disparo, aterrizaje; **sin** respaldo ni fallos |
| `STAT` final | `estado=6 ... disparos=1 ... intentos=1 confirmado=1`, `liberaciones_descartadas=0` |
| Enlace serie | `OK: la placa recibio las 14887 lineas en orden (SIM_PROXIMA=148880)`; `sim_rx=14887 sim_descartadas=0 sim_crc=1`; `ciclos=14887` |
| Duración en la PC | 2 min 29 s a ~2 min 55 s (cada reenvío retrasa el resto) |
| Pérdida de paquetes (estación) | ≤ 2 % |

**Pasa** si el enlace se confirmó, estados en orden, banderas 0x17 y **h_despliegue ∈ [77,
83] m** (y dentro de ±0.5 m de 79.9; tiempos ±0.02 s). **Falla** si dispara por respaldo
(bit 3), si aparece CAÍDA antes de T+78.3 s (falsa liberación en el ascenso o en el
estacionamiento) o si h_despliegue sale del rango. **No vale** (repetir) si `sim_serial`
no imprime `OK: la placa recibio ...`.

#### (b2) Sueltas a 90, 95 y 105 m
El mismo vuelo con el dron estacionado a otra altura. Las tres son liberaciones válidas
(> 85 m) y despliegan a 80 m.
```bash
python tools/sim_serial.py --csv sim_csv/B_90.csv --port $TX --modo B      # y B_95, B_105
```
```powershell
& $PY tools\sim_serial.py --csv sim_csv\B_90.csv --port COM5 --modo B
```
| CSV | Suelta | CAÍDA | `# DISPARO` (`t_ms`, h, vz) | h_despliegue | ATERRIZADO | Altura máx. | `STAT` servo |
|---|---|---|---|---|---|---|---|
| `B_90` | 74.97 s, 90.3 m | T+75.45 s | 75 990, 85.0, -10.1 | **79.9** | T+88.37 s | 90.6 m | `intentos=3 confirmado=0` |
| `B_95` | 76.64 s, 95.3 m | T+77.12 s | 77 980, 86.3, -12.9 | **79.9** | T+90.13 s | 95.5 m | `intentos=3 confirmado=0` |
| `B_105` | 79.97 s, 105.3 m | T+80.46 s | 81 850, 88.3, -17.1 | **79.8** | T+93.93 s | 105.5 m | `intentos=1 confirmado=1` |

Líneas: 14 511 / 14 699 / 15 073 (`SIM_PROXIMA=` 145120 / 147000 / 150740). Banderas al
final 0x17 en las tres. Con 90 y 95 m el CanSat va a ≤ 13 m/s al desplegar y el autogiro
simulado no da el pico de 2 g: el servo hace los 3 intentos (3.6 s; es lo esperado, P59).
**Pasa** si las tres dan h_despliegue ∈ [77, 83] m, 0x17 y estados en orden.

#### (b3) Caída inclinada
```bash
python tools/sim_serial.py --csv sim_csv/B_inclinada.csv --port $TX --modo B
```
Cae inclinado ~70°: al cruzar h_disp la compuerta de 45° no se cumple y en B se espera
como máximo **0.3 s** (en A, 1.5 s).

| Dato | Esperado |
|---|---|
| CAÍDA | T+78.79 s |
| `# DISPARO` | `t_ms` = 80 230 (0.30 s después de cruzar h_disp), `h=82.6 vz=-17.5` |
| h_despliegue | **73.8 m** (±0.5): ~6 m más abajo por la espera de la compuerta |
| Banderas al final | **0x1F** (con **respaldo**, bit 3) |

**Pasa** si el disparo tiene el bit 3, h_despliegue ∈ [72, 77] m y es por encima del
respaldo de 70 m.

#### (b4) Ascenso lento y estacionamiento largo (opcional, 4 min 41 s)
```bash
python tools/sim_serial.py --csv sim_csv/B_lento.csv --port $TX --modo B
```
El dron sube a 1.5 m/s con vibración fuerte (0.3 g por eje) y se queda **120 s**
estacionado a 100 m. Comprueba el ajuste del p0 en TIERRA (P57): antes, con este ascenso
la placa **nunca** pasaba a ASCENSO y no desplegaba.

| Dato | Esperado |
|---|---|
| ASCENSO | T+39.71 s (dron a ~24 m: 20 m + 3 s a 1.5 m/s) |
| T+90 s a T+210 s | estacionado a 100 m en ASCENSO; ningún `# ESTADO 3`, `liberaciones_descartadas=0` |
| CAÍDA / `# DISPARO` | T+211.17 s / `t_ms` = 212 270, `h=87.4 vz=-14.9` |
| h_despliegue | **79.9** (80 ±3 m) |
| ATERRIZADO / banderas | T+224.49 s / 0x17 |
| Enlace | `OK: la placa recibio las 28122 lineas en orden (SIM_PROXIMA=281230)` |

### (c) Modo A (reglamento anterior: 450 m → 200 m)
**Objetivo:** el modo A sigue igual que antes.
```bash
python tools/sim_serial.py --csv sim_csv/A_nominal.csv --port $TX --modo A
```
```powershell
& $PY tools\sim_serial.py --csv sim_csv\A_nominal.csv --port COM5 --modo A
```
`--modo A` cambia el modo (se guarda en NVS). **Al terminar, vuelve a B** (`MODO B`; el
arranque avisa `# AVISO: modo A activo...` mientras siga en A).

| Evento | Esperado |
|---|---|
| `STAT` | `despliegue=A [reglamento anterior: suelta ~450 m] liberacion valida > 240 m, autogiro a 200 m ..., respaldo 160 m, compuerta 45 grados max 1.5 s` |
| Estados | 0 → 1 → 2 → 3 → 4 → 5 → 6 |
| ASCENSO / CAÍDA | T+31.34 s / T+104.79 s |
| `# DISPARO` | `t_ms` = 113 110, `h=215.9 vz=-34.5` |
| **h_despliegue** | **200 ±3 m** (≈ 198.6) |
| ATERRIZADO | T+137.21 s |
| Altura máxima | ≈ 450 m |
| Banderas al final | 0x17 |
| Enlace | `OK: la placa recibio las 19391 lineas en orden (SIM_PROXIMA=193920)` |

**Pasa** si h_despliegue ∈ [197, 203] m, estados en orden y sin bandera de respaldo.
Contraprueba opcional: `B_nominal.csv` con `--modo A` **no** libera (100 m < 240 m): la
placa se queda en ASCENSO (`liberaciones_descartadas=2`) y no dispara.

> **19-nov-2026, primera corrida de la antigua prueba (b) en modo A (firmware sin CRC): no
> valió.** `sim_rx=17318 sim_invalidas=137` de 19 391 enviadas; disparo a T+45 s con
> `h=238.5 vz=+130.0`. Causa: bytes perdidos en el puerto serie y líneas pegadas (P49).

### (d) Modo C (tinaco)
**Objetivo:** disparo inmediato al detectar la caída libre, sin altura mínima ni compuerta.
```bash
python tools/sim_serial.py --csv sim_csv/C_tinaco.csv --port $TX --modo C
```
```powershell
& $PY tools\sim_serial.py --csv sim_csv\C_tinaco.csv --port COM5 --modo C
```
(El CSV sube a 8 m y suelta; **vuelve a `MODO B` al terminar**.)

| Evento | Esperado |
|---|---|
| `STAT` | `despliegue=C [tinaco] disparo al detectar la caida libre (baja g 150 ms), sin altura minima ni compuerta` |
| Estados | 0 → 1 → **(sin 2)** → 3 → 4 → 5 → 6. CAÍDA dura un ciclo (10 ms) y puede no verse |
| Eventos de la estación | "Liberación detectada" y "Disparo del servo" en el mismo segundo, T+00:29 |
| `# DISPARO` | `t_ms` = 29 750 (la caída empieza a 29.59 s): **0.15 a 0.20 s después de soltarse** |
| Altura al disparo | ≈ 8 m (`h≈7.9`; antes 6.9: el p0 ya no se arrastra al subir los 8 m, P57) |
| ATERRIZADO | T+39.13 s |
| `STAT` | `intentos=3 confirmado=0` (el impacto simulado no da el pico de 2 g dentro del intento) |
| Banderas al final | 0x17 |
| Enlace | `OK: la placa recibio las 9196 lineas en orden (SIM_PROXIMA=91970)` |

**Pasa** si **no** se entra a ASCENSO (en la estación el cuadro "2 · ASCENSO" no tiene hora),
el disparo ocurre entre `t_ms` 29 600 y 29 900 y la bandera de respaldo está apagada.
**Falla** si espera una altura o pasa por ASCENSO.

### (e) Liberación falsa descartada (dron estacionado a 100 m)
**Objetivo:** 0.25 s de caída libre del dron **estacionado a 100 m** (por encima del mínimo
de 85 m) a T+70 s no debe contar como liberación: la baja g dura más de 100 ms, pero vz
solo llega a ≈ -2.5 m/s (el detector pide -5 m/s en 0.6 s).
```bash
python tools/sim_serial.py --csv sim_csv/B_falsa_liberacion.csv --port $TX --modo B
```
```powershell
& $PY tools\sim_serial.py --csv sim_csv\B_falsa_liberacion.csv --port COM5 --modo B
```
| Dato | Esperado |
|---|---|
| Hasta T+78 s | Estado ASCENSO; **ningún** `# ESTADO 3` ni evento "Liberación detectada" |
| `STAT` final | **`liberaciones_descartadas=1`** |
| CAÍDA (liberación real) | T+78.80 s |
| `# DISPARO` | `t_ms` = 79 930, `h=87.5 vz=-15.1` |
| **h_despliegue** | **80 ±3 m** (≈ 79.9) |
| Banderas al final | 0x17 |

**Pasa** si `liberaciones_descartadas=1`, la primera liberación es la de T+78.8 s y
h_despliegue ∈ [77, 83]. **Falla** si aparece CAÍDA cerca de T+70 s.
(En native, `test_flight` también prueba la misma caída breve durante el ascenso a ~50 m.)

### (f) Falla de barómetro (modo B)
**Objetivo:** el barómetro se congela a los 45 s (ascenso, ~65 m); la liberación se detecta
solo por IMU y el disparo es **por tiempo** (1.0 s después de la liberación).
```bash
python tools/sim_serial.py --csv sim_csv/B_falla_baro.csv --port $TX --modo B
```
```powershell
& $PY tools\sim_serial.py --csv sim_csv\B_falla_baro.csv --port COM5 --modo B
```
| Dato | Esperado |
|---|---|
| Evento "Fallo de barómetro" | T+46 s (1 s después de congelarse), bandera bit 5 |
| Bandera NO-GO | **No** (el IMU sigue vivo) |
| CAÍDA | T+78.83 s (solo por baja g + confirmación; 0.52 s tras la suelta) |
| `# DISPARO` | `t_ms` = 79 830 |
| **Liberación → disparo** | **1.00 s ±0.02 s** (`t_ms` de `# ESTADO 3` y de `# DISPARO`; `# ESTADO` puede ir ~15 ms tarde) |
| Banderas al final | 0x3F: base, liberación, disparo, **respaldo**, aterrizaje, fallo baro |
| h del `# DISPARO` y altura máxima | **no son confiables** (baro muerto a ~65 m); se ignoran |
| ATERRIZADO | T+89.25 s |

**Pasa** si Fallo baro (bit 5) y respaldo (bit 3) están encendidos y la diferencia es
1.0 ±0.1 s. **Falla** si no dispara, o dispara por altura. (En native el autogiro actúa a
81.7 m reales: sin barómetro el disparo por tiempo queda un poco por encima de 80 m.)

### (g) Falla de IMU (modo B)
**Objetivo:** el IMU deja de responder a los 45 s; liberación solo por barómetro, disparo
**sin compuerta de ángulo**.
```bash
python tools/sim_serial.py --csv sim_csv/B_falla_imu.csv --port $TX --modo B
```
```powershell
& $PY tools\sim_serial.py --csv sim_csv\B_falla_imu.csv --port COM5 --modo B
```
| Dato | Esperado |
|---|---|
| Evento "Fallo de IMU" | T+46 s, bandera bit 6 |
| CAÍDA | T+79.73 s (solo barómetro: 1.42 s tras la suelta) |
| `# DISPARO` | `t_ms` = 80 210, `h=85.3 vz=-10.9` |
| **h_despliegue** | **80 ±3 m** (≈ 79.9, estimado a bordo) |
| Respaldo (bit 3) | apagado |
| `STAT` | `intentos=3 confirmado=0` (sin IMU no hay confirmación) |
| Banderas al final | 0x57: base, liberación, disparo, aterrizaje, fallo IMU |
| ATERRIZADO | T+94.01 s |

**Pasa** si h_despliegue ∈ [77, 83], bit 6 encendido, bit 3 apagado, `intentos=3`.
Ojo: sin acelerómetro la vz estimada va retrasada (-10.9 contra -17.3 m/s reales): en
native el autogiro actúa a **74.2 m reales** (P59).

### (h) Armado por botón PRG y por ARMAR, cuenta de 10 s
**Objetivo:** el servo se dispara `DEMO_DELAY_S = 10` s después de armar; el instante queda
en serial, en el paquete (bit 2) y el LED lo indica. Sin sensores y sin simulación.

Placa TX en TIERRA (`# ESTADO 1`), monitor abierto (§1.4), estación con INICIAR pulsado.
**No** presiones PRG al encender (GPIO0 es pin de arranque).

**h1. Por comando**
1. Escribe `ARMAR`. Aparece `OK armado: disparo en 10 s` y `# DEMO ARMADO t_ms=XXXX ...`.
2. El LED integrado de la placa parpadea rápido (≈ 5 Hz) y la estación sube a **5 paquetes/s**.
3. Escribe `STAT` a los ~3 s: `servo=ON` (con PWM a -10°; sin servo conectado no pasa nada).
4. A los 10 s: `# SEPARACION t_ms=YYYY h=0.0 vz=0.0 h_despliegue=0.0` y el LED queda **fijo**.
   En la estación aparece "Disparo del servo" y la bandera 2 queda encendida.
5. `STAT` a los ~2 s del disparo: `servo=ON`; a los ≥ 5 s: **`servo=OFF`** (la secuencia
   dura 3.6 s: sin IMU no hay confirmación y se hacen los 3 intentos).

**h2. Por botón**: mantén **presionado PRG 1 s** (placa ya encendida y en TIERRA). Debe
aparecer `# DEMO ARMADO` y repetirse los pasos 2 a 5.

**h3. Cancelar**: arma (ARMAR o PRG) y, antes de 10 s, otro toque largo de PRG o `DESARMAR`.
Debe aparecer `# DEMO CANCELADO`, el LED se apaga y **no** hay `# SEPARACION` en los 15 s
siguientes.

| Dato | Esperado |
|---|---|
| **Armado → separación** | **10.000 s ±0.05 s** (`t_ms` de `# SEPARACION` menos el de `# DEMO ARMADO`) |
| Tiempo según el monitor (`--filter time`) | 10 s ±0.3 s |
| LED | parpadeo rápido durante la cuenta; fijo después; apagado al cancelar |
| TX durante la cuenta y después | 5 Hz (la estación muestra ≈ 5/s) |
| Bandera de disparo en paquetes | solo desde la separación |
| Servo | ON durante la cuenta y 3.6 s tras el disparo; OFF después |

**Pasa** si ARMAR y PRG dan 10 s ±0.05 s, el LED se comporta como se indica y la
cancelación funciona. Opcional con osciloscopio en GPIO7: PWM de 50 Hz con pulso de
1389 µs (-10°) durante la cuenta y 611 µs (-80°) tras el disparo.

### (i) Reinicio en vuelo y MISION_BORRAR
**Objetivo:** comprobar el registro de misión en NVS: un corte a mitad de misión se
recupera, y `MISION_BORRAR` lo limpia. Como la placa sin sensores no despega, se usa el
comando `MISION_PRUEBA`, que guarda una misión **falsa** en ASCENSO.

En el monitor de la placa TX (§1.4), con la placa en TIERRA:

| Paso | Escribes / haces | Esperado |
|---|---|---|
| 1 | `MISION_BORRAR` | `OK mision borrada; reiniciando`; la placa reinicia y arranca en `# ESTADO 1` |
| 2 | `MISION` | `sin registro de mision` |
| 3 | `MISION_PRUEBA` | `OK mision de PRUEBA guardada (ASCENSO)...` |
| 4 | `MISION` | `mision=1 estado=2 banderas=0x01 ...` |
| 5 | Botón **RST** de la placa (simula el corte de energía) | arranque con `# REINICIO EN VUELO: mision=1 estado=2 ...`; luego `# ESTADO 2` |
| 6 | `STAT` | `estado=2`; la estación muestra **ASCENSO** y TX a **2 Hz** |
| 7 | `SIM_INICIO` | `ERR hay una mision en curso...` (el modo simulación se bloquea con misión) |
| 8 | `MISION_BORRAR` | `OK mision borrada; reiniciando` → arranque normal `# ESTADO 1`, sin `REINICIO EN VUELO` |
| 9 | `MISION` | `sin registro de mision` |
| 10 | `SIM_INICIO` | `OK SIM_INICIO ...` (ya se puede simular otra vez) |

**Pasa** si los diez pasos dan lo indicado. **Falla** si tras el paso 5 la placa arranca en
TIERRA (no recuperó la misión) o si tras el paso 8 sigue en `REINICIO EN VUELO`.
Recomendación: **`MISION_BORRAR` antes de cada vuelo o prueba**.

### (j) Prueba de alcance a 250 m
**Objetivo:** medir RSSI, SNR y pérdida de paquetes a 10, 50, 100, 150, 200 y 250 m
(la aceptación pide transmitir a 250 m).

**Montaje:** placa TX (banco, firmware sin cambios) conectada a un **power bank USB** con
su antena **vertical y puesta**; arranca sola en TIERRA y transmite a **1 Hz**, sin PC.
Placa RX (rx_station) en la laptop con la estación: `python ground/server.py --port $RX`
(Windows: `& $PY ground\server.py --port COM6`). La laptop y la placa RX se quedan fijas;
la placa TX se lleva a cada distancia **en línea
recta con visibilidad directa**, a la misma altura (de preferencia ≥ 1.5 m del suelo,
en la mano o sobre un tripié). Mide las distancias con cinta, rueda o un mapa/GPS del
teléfono.

**Procedimiento por distancia:**
1. Coloca la placa TX a la distancia indicada y espera a ver "ENLACE OK".
2. Estación: **INICIAR TOMA DE DATOS**; espera **2 minutos** (120 paquetes); **FINALIZAR**.
   Cada distancia queda en su propio CSV.
3. Pasa a la siguiente distancia. Al final:
```bash
python tools/resumen_enlace.py ground/data/sesion_*.csv        # Linux
```
```powershell
& $PY tools\resumen_enlace.py ground\data\sesion_*.csv            # Windows
```
imprime por sesión: recibidos, esperados, % de pérdida, RSSI y SNR promedio y mínimo.
Copia los valores a la tabla.

**Tabla de registro** (fecha: ____ lugar: ____ antenas: ____ altura de la placa TX: ____ m)

| Distancia (m) | Archivo de sesión | Paquetes recibidos | Esperados | Pérdida (%) | RSSI prom (dBm) | RSSI mín (dBm) | SNR prom (dB) | SNR mín (dB) | ¿Pasa? |
|---:|---|---:|---:|---:|---:|---:|---:|---:|:--:|
| 10 | | | | | | | | | |
| 50 | | | | | | | | | |
| 100 | | | | | | | | | |
| 150 | | | | | | | | | |
| 200 | | | | | | | | | |
| **250** | | | | | | | | | |
| 300 (opcional, margen) | | | | | | | | | |

**Criterio de pasa/falla (suposición, P47):** a **250 m**, con 2 min de datos:
**pérdida ≤ 5 %**, **RSSI promedio ≥ -100 dBm** y **SNR promedio ≥ 0 dB**. Con SF7 y 250 kHz
la sensibilidad del SX1262 es ≈ -118 dBm y el límite de demodulación SNR ≈ -7.5 dB, así
que ese criterio deja un margen de unos 18 dB. **Falla** si no se cumple alguno; en ese
caso prueba con la antena más alta o repite en otro lugar antes de concluir.
Una prueba a nivel de suelo es más exigente que el vuelo (el suelo atenúa la señal).

---

## 3. Registro de resultados

| Prueba | Fecha | Firmware / commit | Resultado (pasa/falla) | Observaciones |
|---|---|---|---|---|
| (a) Enlace | | | | |
| (b) Modo B nominal (100 m) | | | | h_despliegue = ___ m |
| (b2) B sueltas 90/95/105 m | | | | h_despliegue = ___ / ___ / ___ m |
| (b3) B caída inclinada | | | | h_despliegue = ___ m |
| (b4) B ascenso lento | | | | ASCENSO a T+___ s |
| (c) Modo A | | | | h_despliegue = ___ m |
| (d) Modo C | | | | disparo a ___ s de la caída |
| (e) Liberación falsa (B) | | | | descartadas = ___ |
| (f) Falla de baro (B) | | | | liberación→disparo = ___ s |
| (g) Falla de IMU (B) | | | | h_despliegue = ___ m |
| (h) ARMAR / PRG | | | | armado→separación = ___ s |
| (i) MISION_BORRAR | | | | |
| (j) Alcance 250 m | | | | ver tabla |

## 4. Problemas frecuentes

| Síntoma | Qué hacer |
|---|---|
| `sim_serial` no encuentra el puerto / "Acceso denegado" / "Device or resource busy" | Otro programa lo tiene abierto (monitor, estación): ciérralo. |
| Linux: "Permission denied" en `/dev/ttyUSB*` | Tu usuario no está en el grupo del puerto: `sudo usermod -aG uucp $USER` (Arch; en Debian/Ubuntu `dialout`) y vuelve a iniciar sesión. |
| La placa no responde a `SIM_INICIO` | Espera 3 s tras abrir el puerto (puede reiniciarse); usa `--espera 4`. |
| `ERR hay una mision en curso` | `MISION_BORRAR` (la placa reinicia) y repite. |
| La placa no pasa a ASCENSO en un ascenso real | Ver `STAT`: `p0_pausa=1` al levantarla es lo normal. Debe subir > 20 m a > 1 m/s durante 3 s (P57). |
| La estación dice "ESPERANDO DATOS" en la simulación | En modo simulación la placa **no transmite** hasta la primera línea `SIM`; empieza `sim_serial`. |
| Estados que no coinciden con esta guía | Confirma el modo (`MODO`; se guarda en NVS) y que el CSV es del mismo modo (`--modo` de `sim_serial.py`). |
| `sim_descartadas` > 0 en `STAT` | El PC mandó más rápido de lo que la placa consume: usa `--speed 1`. |
| `ERROR: la placa no confirmo todas las lineas` | Firmware sin CRC (sube `heltec_banco` de nuevo) o la placa se reinició durante el vuelo. Repite la corrida. |
| `sim_crc=0` en `STAT` | La placa no recibió líneas con CRC: `sim_serial.py` viejo o `--sin-crc`. |
| Vuelo con disparo muy temprano, h_max de cientos de metros de más | Líneas dañadas aceptadas (firmware o `sim_serial.py` sin CRC). Ver "Integridad del enlace" en §1.7. |
| Pérdida alta con las placas a 1 m | Revisa antenas; separa las placas ≥ 1 m (saturación del receptor). |
| Quedó el modo A o C guardado (el arranque dice `# AVISO: modo A activo...`) | `MODO B` en el monitor. |
| `# AVISO: log al X %` al arrancar | El log pasó del 50 %: `BORRAR`. Lleno (100 %) deja de guardar; al despegar o armar la placa lo vacía sola si quedan < 600 KB. |
