# Pruebas de sistemas con dos Heltec V3 (sin sensores ni servo)

Guía paso a paso, para **Windows PowerShell**, de las pruebas de sistema que se pueden hacer
con solo **dos placas Heltec WiFi LoRa 32 V3**, sin MPU6500, BMP280 ni servo. El firmware
de vuelo se alimenta con el **modo simulación por serial** (`docs/sim_input.md`): una PC
manda vuelos de `tools/flight_sim.py` y la placa los procesa como si vinieran de los
sensores. Dudas y supuestos: `docs/PREGUNTAS.md` (P44 a P49).

## 0. Qué se prueba y qué no

| Con esta guía SÍ se prueba | NO se prueba (hace falta hardware) |
|---|---|
| Máquina de estados real en la placa: liberación, disparo, descenso, aterrizaje | Lectura real del MPU6500 y del BMP280 (I2C) |
| Modos A, B y C; fallas de baro e IMU; liberación falsa | Calibración del giroscopio con sensor real, ruido real |
| Telemetría LoRa real entre las dos placas y la estación terrena | Servo: giro de -10° a -80°, confirmación por pico de 2 g |
| Armado por ARMAR y por el botón PRG, cuenta de 10 s, LED | Mecanismo de liberación del autogiro |
| Reinicio en vuelo (NVS) y MISION_BORRAR | Batería de 9 V y su medición |
| Alcance del radio (RSSI, SNR, pérdida) | Vuelo real (viento, vibración, antena en el aire) |

**Placa sin sensores.** Al arrancar, IMU y barómetro fallan en la autoprueba. La placa llega
a TIERRA con las banderas **NO-GO + fallo baro + fallo IMU (0xE0)**, transmite a 1 Hz y
**se queda en TIERRA** (no pasa a ASCENSO ni guarda una misión en NVS). El modo simulación
se activa igual que con sensores (comando `SIM_INICIO` o la primera línea `SIM,...`).
Antes de este ajuste (P44) la placa saltaba a ASCENSO y bloqueaba el modo simulación.

> **No se probó en una placa real**: el comportamiento sin sensores y todos los números de
> esta guía salen de las pruebas native (`pio test -e native`, mismo código que la placa) y
> de leer el firmware. Si algo no coincide en la placa, anótalo en la tabla de la sección 3
> y avisa: puede ser un error de esta guía.

## 1. Preparación (una sola vez)

Todos los comandos se escriben en PowerShell **desde la raíz del repositorio**
(`C:\Users\leiru\Hiram\sistemas-cansat`).

### 1.1 Entorno de Python
```powershell
py -m venv .venv                                  # solo si no existe .venv
.\.venv\Scripts\python -m pip install -r requirements.txt
$PY  = ".\.venv\Scripts\python.exe"               # atajos para esta ventana de PowerShell
$PIO = ".\.venv\Scripts\pio.exe"
```
Los atajos `$PY` y `$PIO` valen solo en la ventana donde los escribes; si abres otra,
vuelve a escribirlos.

### 1.2 Ver el puerto COM de cada placa
Llamaremos **placa A** a la que lleva el firmware de vuelo/banco (transmite) y **placa B** a
la receptora (`rx_station`).

1. Conecta **una sola** placa por USB.
2. Abre el **Administrador de dispositivos** (tecla Windows + X → *Administrador de
   dispositivos*) → **Puertos (COM y LPT)**: aparece "Silicon Labs CP210x USB to UART
   Bridge (COM5)" o similar. Ese número es el puerto.
3. Desconecta esa placa, conecta la otra y repite. Anota: **A = COM__, B = COM__**.
   (En esta guía: A = `COM5`, B = `COM6`; cambia los números por los tuyos.)

También se puede ver con comandos:
```powershell
Get-PnpDevice -Class Ports | Select-Object FriendlyName, Status
& $PY ground\server.py --list-ports
```
Si no aparece ningún puerto: cable USB solo de carga (usa uno de datos) o falta el driver
CP210x de Silicon Labs.

### 1.3 Subir el firmware
Antenas LoRa **conectadas** en las dos placas antes de energizarlas (transmitir sin antena
puede dañar el radio).
```powershell
# Placa A: firmware de banco (modo simulación + modo demo; NUNCA para vuelo)
& $PIO run -d firmware -e heltec_banco -t upload --upload-port COM5
# Placa B: receptor de la estación terrena
& $PIO run -d firmware -e rx_station -t upload --upload-port COM6
```
La primera compilación tarda varios minutos. Si el *upload* dice "Failed to connect": mantén
presionado **PRG**, toca **RST**, suelta PRG y repite el comando.

### 1.4 Monitor serie
```powershell
& $PIO device monitor -p COM5 -b 115200 --filter time --echo     # placa A
```
`--filter time` pone la hora de la PC en cada línea y `--echo` muestra lo que escribes.
Escribe un comando y pulsa Enter. Salir: **Ctrl + C**. Un solo programa puede tener abierto
un puerto: **cierra el monitor antes de correr `sim_serial.py` o la estación** en ese mismo
puerto. Si la placa se reinicia al abrir el monitor, espera 3 s.

Arranque esperado de la placa A (sin sensores):
```
CanSat Hellmary - MODO DEMO_SEPARACION (disparo 10 s tras ARMAR o toque largo de PRG) - BANCO: ...
IMU FALLA (WHO_AM_I=0x00)  BARO FALLA
LittleFS OK
Radio OK (codigo 0)
NVS OK
Modo por defecto: despliegue=A (liberacion valida > 240 m, autogiro a 200 m, respaldo 160 m)
# ESTADO 1 banderas=0xE0 t_ms=...
```
(`IMU FALLA` y `BARO FALLA` son lo esperado sin sensores; puede aparecer antes un
`# ESTADO 0` de un ciclo.) Y de la placa B:
```
# CanSat Hellmary - receptor rx_station
# radio OK, escuchando (0)
0100...,-31.5,9.75        <- una línea HEX,RSSI,SNR por paquete recibido
```

### 1.4b Antes de cada prueba (checklist: `MISION_BORRAR`, `BORRAR`, `MODO A`, `STAT`)
```powershell
# En el monitor de la placa A:
MISION_BORRAR     # borra el registro de misión de NVS y reinicia la placa
BORRAR            # vacía el log: "OK log vaciado (X B liberados)"; si no, se llena (100 %)
MODO A            # el modo de despliegue se guarda en NVS: déjalo en A
STAT              # log=0/... (0 %), estado=1, despliegue=A
# (sim_serial.py ya manda SIM_INICIO y, con --modo, MODO; no manda MISION_BORRAR)
```
Si el arranque dice `# REINICIO EN VUELO`, hay una misión guardada: `MISION_BORRAR`.

### 1.5 Estación terrena
En **otra** ventana de PowerShell (con `$PY` definido):
```powershell
& $PY ground\server.py --port COM6
```
Abre **http://127.0.0.1:8000** en Edge o Chrome. Debe decir "ENLACE OK" en cuanto la placa
A transmita. Detalles: `ground/README.md`. Para grabar una prueba: botón **INICIAR TOMA DE
DATOS** (y **FINALIZAR** al terminar); el CSV queda en `ground\data\`.

### 1.6 Vuelos simulados (CSV)
```powershell
New-Item -ItemType Directory -Force sim_csv | Out-Null
& $PY tools\flight_sim.py --raw-csv sim_csv\A_nominal.csv --seed 1
& $PY tools\flight_sim.py --raw-csv sim_csv\B_nominal.csv --seed 1 --mode B
& $PY tools\flight_sim.py --raw-csv sim_csv\C_tinaco.csv --seed 1 --mode C
& $PY tools\flight_sim.py --raw-csv sim_csv\A_falsa_liberacion.csv --seed 1 --false-release 70
& $PY tools\flight_sim.py --raw-csv sim_csv\A_falla_baro.csv --seed 1 --fail baro --fail-at 40
& $PY tools\flight_sim.py --raw-csv sim_csv\A_falla_imu.csv --seed 1 --fail imu --fail-at 40
```
(`sim_csv\` está en `.gitignore`.) Duraciones: A ≈ 3 min 14 s, B ≈ 1 min 55 s, C ≈ 1 min
32 s, falsa liberación ≈ 3 min 14 s, falla de baro ≈ 3 min 17 s, falla de IMU ≈ 3 min 14 s.

### 1.7 Cómo se corre un vuelo simulado
Con la estación abierta y el monitor de la placa A **cerrado**:
```powershell
& $PY tools\sim_serial.py --csv sim_csv\A_nominal.csv --port COM5 --modo A
```
`sim_serial.py` manda `SIM_INICIO`, `MODO A` y las ~19 400 líneas a tiempo real; imprime lo
que dice la placa con los segundos **de la PC** desde que abrió el puerto (`[ 116.20 s] <
# DISPARO ...`, unos 3 s más que el tiempo de simulación **más** lo que retrasen los
reenvíos) y al final pide `STAT`. Los tiempos de la simulación (`t_ms` de los mensajes de
la placa y T+ de la estación) empiezan en 0 con la primera línea y **no** cambian con los
reenvíos. **No** cierres ni toques la placa durante el vuelo.

**Integridad del enlace (desde el 19-nov-2026).** El puerto serie de la placa pierde bloques
de bytes (~1 por segundo, mientras la flash guarda el log): en la primera corrida de la
prueba (b) se perdieron ~1 900 líneas y algunas llegaron "pegadas" con valores falsos, lo
que produjo un disparo a T+45 s. Ahora cada línea lleva CRC; la placa descarta las dañadas
o fuera de orden y pide `SIM_REPITE`, y `sim_serial.py` las reenvía (detalles:
`docs/sim_input.md`, "Integridad"). **Requiere el firmware nuevo** (sube otra vez
`heltec_banco`). Al final de **cada** vuelo simulado, `sim_serial.py` debe imprimir:
```
lineas del CSV=19391 enviadas=19xxx (reenviadas=...) pedidos_de_reenvio=... retrocesos=... duracion=...
OK: la placa recibio las 19391 lineas en orden (SIM_PROXIMA=193920)
```
y el `STAT` final, en su última línea, `sim_rx=` **igual** a las líneas del CSV,
`sim_descartadas=0` y `sim_crc=1`. `sim_invalidas`, `sim_huecos`, `sim_repetidas` y
`sim_repite` pueden ser > 0 (son las pérdidas detectadas y reenviadas; del orden de 1 por
segundo). Si sale `ERROR: la placa no confirmo...` o `sim_rx` no coincide, **la corrida
no vale** (no la cuentes como falla del vuelo): repítela. Con eso la placa procesa
exactamente las mismas lecturas que `pio test -e native`, y los números de las tablas
deben coincidir con la tolerancia indicada en cada prueba.

Cada prueba pide leer, en `sim_serial`: los mensajes `# ESTADO`, `# DISPARO ...` y el
`STAT` final; y en la estación: la línea de tiempo, los eventos, las banderas y las
tarjetas. Los T+ de la estación se redondean hacia abajo al segundo.

---

## 2. Pruebas

Tolerancias: las de la tabla de cada prueba. "h_despliegue" es la altura **estimada** donde
actúa el autogiro (`h - |vz|·0.5`, la que imprime la placa en `# DISPARO`). Es la del
estimador a bordo, que dispara en el umbral; la altura real del vuelo simulado (198.3 m y
199.4 m con las semillas 1 y 7) se verifica en `pio test -e native`.

### (a) Enlace entre las dos placas (rx_station)
**Objetivo:** comprobar que la telemetría LoRa llega de la placa A a la B y la estación la
muestra, sin sensores ni simulación.

1. Placa A (banco) y placa B (rx_station) cargadas, antenas puestas, separadas **1 a 2 m**.
2. Estación: `& $PY ground\server.py --port COM6`; abre http://127.0.0.1:8000.
3. En la estación pulsa **INICIAR TOMA DE DATOS**, espera **2 minutos** y pulsa
   **FINALIZAR**.
4. Resumen: `& $PY tools\resumen_enlace.py ground\data\sesion_*.csv` (la última fila es esta).

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
nada (revisa antenas, `# radio OK` en la B, que ambas usen el mismo `lora_setup.h`) o
si la pérdida > 2 %.

### (b) Vuelo simulado nominal, modo A
**Objetivo:** vuelo completo 450 m → 200 m, con la estación mostrando estados y disparo.

```powershell
& $PY tools\sim_serial.py --csv sim_csv\A_nominal.csv --port COM5 --modo A
```
Antes: estación abierta, **INICIAR TOMA DE DATOS** pulsado.

**Resultado esperado (semilla 1):**

| Evento | Esperado (tiempo de simulación) |
|---|---|
| Estados en orden | 0 → 1 → 2 → 3 → 4 → 5 → 6, sin retrocesos |
| CAÍDA (liberación) | T+104.8 s (en la estación T+01:44) |
| `# DISPARO t_ms=` | ≈ 113 110, `h≈215.9 vz≈-34.6` |
| **h_despliegue** | **200 ±3 m** (≈ 198.6 con la semilla 1) |
| DESCENSO (≈ 1 s tras el disparo) / ATERRIZADO | ≈ T+114 s / T+137.2 s |
| Altura máxima | 445.9 m (tarjeta) |
| vz en DESCENSO | ≈ -10 m/s, tarjeta **verde** |
| Banderas al final | 0x17: base lista, liberación, disparo, aterrizaje; **sin** respaldo ni fallos |
| `STAT` final | `estado=6 ... disparos=1 ... intentos=1 confirmado=1`, `liberaciones_descartadas=0` |
| Enlace serie (`sim_serial` y `STAT`) | `OK: la placa recibio las 19391 lineas en orden`; `sim_rx=19391 sim_descartadas=0 sim_crc=1`; `ciclos=19391` |
| Duración en la PC | 3 min 14 s a ~3 min 50 s (cada reenvío retrasa el resto) |
| Pérdida de paquetes (estación) | ≤ 2 % |

**Tolerancia nueva.** Con el enlace verificado la placa procesa las mismas lecturas que la
prueba native `test_sim_link` (mismo formato de texto que `sim_serial.py`), así que los
tiempos de liberación y disparo deben salir **iguales a ±0.02 s** (`t_ms` de `# ESTADO 3` =
104 790 y de `# DISPARO` = 113 110) y `h`, `vz` y `h_despliegue` del `# DISPARO` **iguales
a ±0.5 m** (215.9, -34.5, 198.6); el margen solo cubre diferencias de redondeo de punto
flotante entre la PC y el ESP32-S3. Si se cumple la condición del enlace pero no esta, es
una diferencia real entre la placa y native: anótala y avisa.

**Pasa** si el enlace se confirmó y se cumplen las filas; la condición decisiva:
**h_despliegue ∈ [197, 203] m** (y dentro de ±0.5 m de 198.6), estados en orden y banderas
0x17. **Falla** si dispara por respaldo (bit 3), si falta un estado o si h_despliegue sale
del rango. **No vale** (repetir) si `sim_serial` no imprime `OK: la placa recibio ...`.
(Con `--seed 7`: h_despliegue ≈ 199.7.)

> **19-nov-2026, primera corrida (firmware sin CRC): no valió.** `sim_rx=17318
> sim_invalidas=137` de 19 391 enviadas; disparo a T+45 s con `h=238.5 vz=+130.0`,
> h_max de la estación 1061 m. Causa: bytes perdidos en el puerto serie y líneas pegadas
> (ver arriba y PREGUNTAS P49). Repetir con el firmware y el `sim_serial.py` nuevos.

### (c) Modo B (liberación a ~100 m, autogiro a 50 m)
**Objetivo:** el mismo vuelo a baja altura con `DEPLOY_MODE = B`.
```powershell
& $PY tools\sim_serial.py --csv sim_csv\B_nominal.csv --port COM5 --modo B
```
`--modo B` cambia el modo (se guarda en NVS). **Al terminar, vuelve a A** (siguiente
sección: `--modo A` o `MODO A`).

| Evento | Esperado |
|---|---|
| `STAT` | `despliegue=B (liberacion valida > 90 m, autogiro a 50 m, respaldo 40 m)` |
| Estados | 0 → 1 → 2 → 3 → 4 → 5 → 6 |
| CAÍDA | T+46.5 s |
| `# DISPARO` | t_ms ≈ 48 580, `h≈61.7 vz≈-23.7` |
| **h_despliegue** | **50 ±3 m** (≈ 49.9) |
| ATERRIZADO | T+57.9 s |
| Altura máxima | 100 m |
| Banderas al final | 0x17 |

**Pasa** si h_despliegue ∈ [47, 53] m, estados en orden y sin bandera de respaldo.
Contraprueba opcional: con el mismo CSV en modo **A** (`--modo A`) **no** hay liberación
(100 m < 240 m): la placa se queda en ASCENSO y no dispara.

### (d) Modo C (tinaco)
**Objetivo:** disparo inmediato al detectar la caída libre, sin altura mínima ni compuerta.
```powershell
& $PY tools\sim_serial.py --csv sim_csv\C_tinaco.csv --port COM5 --modo C
```
(El CSV sube a 8 m y suelta; **vuelve a `MODO A` al terminar**.)

| Evento | Esperado |
|---|---|
| `STAT` | `despliegue=C (tinaco: ...)` |
| Estados | 0 → 1 → **(sin 2)** → 3 → 4 → 5 → 6. CAÍDA dura un ciclo (10 ms) y puede no verse |
| Eventos de la estación | "Liberación detectada" y "Disparo del servo" en el mismo segundo, T+00:29 |
| `# DISPARO` | t_ms ≈ 29 750 (la caída empieza a 29.59 s): **0.15 a 0.20 s después de soltarse** |
| Altura al disparo | ≈ 7 m (`h≈6.9`) |
| ATERRIZADO | T+39.0 s |
| `STAT` | `intentos=3 confirmado=0` (el impacto simulado no da el pico de 2 g dentro del intento) |
| Banderas al final | 0x17 |

**Pasa** si **no** se entra a ASCENSO (en la estación el cuadro "2 · ASCENSO" no tiene hora),
el disparo ocurre entre t_ms 29 600 y 29 900 y la bandera de respaldo está apagada.
**Falla** si espera una altura o pasa por ASCENSO.

### (e) Liberación falsa descartada
**Objetivo:** 0.25 s de caída libre a ~270 m en el ascenso (T+70 s) no debe contar como
liberación.
```powershell
& $PY tools\sim_serial.py --csv sim_csv\A_falsa_liberacion.csv --port COM5 --modo A
```
| Dato | Esperado |
|---|---|
| Hasta T+104 s | Estado ASCENSO; **ningún** `# ESTADO 3` ni evento "Liberación detectada" |
| `STAT` final | **`liberaciones_descartadas=1`** |
| CAÍDA (liberación real) | T+105.2 s (en la estación T+01:45) |
| `# DISPARO` | t_ms ≈ 113 460, `h≈217.0 vz≈-34.6` |
| **h_despliegue** | **200 ±3 m** (≈ 199.7) |
| Banderas al final | 0x17 |

**Pasa** si `liberaciones_descartadas=1`, la primera liberación es la de T+105 s y
h_despliegue ∈ [197, 203]. **Falla** si aparece CAÍDA cerca de T+70 s.

### (f) Falla de barómetro
**Objetivo:** el barómetro se congela a los 40 s; la liberación se detecta solo por IMU y
el disparo es **por tiempo** (7 s después de la liberación).
```powershell
& $PY tools\sim_serial.py --csv sim_csv\A_falla_baro.csv --port COM5 --modo A
```
| Dato | Esperado |
|---|---|
| Evento "Fallo de barómetro" | T+41 s (1 s después de congelarse), bandera bit 5 |
| Bandera NO-GO | **No** (el IMU sigue vivo) |
| CAÍDA | T+104.8 s (solo por baja g + confirmación) |
| `# DISPARO` | t_ms ≈ 111 800 |
| **Liberación → disparo** | **7.0 s ±0.1 s** (`t_ms` de `# ESTADO 3` y de `# DISPARO`) |
| Banderas al final | 0x3F: base, liberación, disparo, **respaldo**, aterrizaje, fallo baro |
| h del `# DISPARO` | **no es confiable** (baro muerto); se ignora |

**Pasa** si Fallo baro (bit 5) y respaldo (bit 3) están encendidos y la diferencia es
7.0 ±0.1 s. **Falla** si no dispara, o dispara por altura.

### (g) Falla de IMU
**Objetivo:** el IMU deja de responder a los 40 s; liberación solo por barómetro, disparo
**sin compuerta de ángulo**.
```powershell
& $PY tools\sim_serial.py --csv sim_csv\A_falla_imu.csv --port COM5 --modo A
```
| Dato | Esperado |
|---|---|
| Evento "Fallo de IMU" | T+41 s, bandera bit 6 |
| CAÍDA | T+106.4 s (por vz < -12 m/s durante 0.4 s) |
| `# DISPARO` | t_ms ≈ 113 080, `h≈216.9` |
| **h_despliegue** | **200 ±10 m** (≈ 199.6; sin IMU vz va retrasada: tolerancia mayor) |
| Respaldo (bit 3) | apagado |
| `STAT` | `intentos=3 confirmado=0` (sin IMU no hay confirmación) |
| Banderas al final | 0x57: base, liberación, disparo, aterrizaje, fallo IMU |

**Pasa** si h_despliegue ∈ [190, 210], bit 6 encendido, bit 3 apagado, `intentos=3`.

### (h) Armado por botón PRG y por ARMAR, cuenta de 10 s
**Objetivo:** el servo se dispara `DEMO_DELAY_S = 10` s después de armar; el instante queda
en serial, en el paquete (bit 2) y el LED lo indica. Sin sensores y sin simulación.

Placa A en TIERRA (`# ESTADO 1`), monitor abierto (§1.4), estación con INICIAR pulsado.
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

En el monitor de la placa A (§1.4), con la placa en TIERRA:

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

**Montaje:** placa A (banco, firmware sin cambios) conectada a un **power bank USB** con
su antena **vertical y puesta**; arranca sola en TIERRA y transmite a **1 Hz**, sin PC.
Placa B (rx_station) en la laptop con la estación: `& $PY ground\server.py --port COM6`.
La laptop y la placa B se quedan fijas; la placa A se lleva a cada distancia **en línea
recta con visibilidad directa**, a la misma altura (de preferencia ≥ 1.5 m del suelo,
en la mano o sobre un tripié). Mide las distancias con cinta, rueda o un mapa/GPS del
teléfono.

**Procedimiento por distancia:**
1. Coloca la placa A a la distancia indicada y espera a ver "ENLACE OK".
2. Estación: **INICIAR TOMA DE DATOS**; espera **2 minutos** (120 paquetes); **FINALIZAR**.
   Cada distancia queda en su propio CSV.
3. Pasa a la siguiente distancia. Al final:
```powershell
& $PY tools\resumen_enlace.py ground\data\sesion_*.csv
```
imprime por sesión: recibidos, esperados, % de pérdida, RSSI y SNR promedio y mínimo.
Copia los valores a la tabla.

**Tabla de registro** (fecha: ____ lugar: ____ antenas: ____ altura de la placa A: ____ m)

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
| (b) Modo A nominal | | | | h_despliegue = ___ m |
| (c) Modo B | | | | h_despliegue = ___ m |
| (d) Modo C | | | | disparo a ___ s de la caída |
| (e) Liberación falsa | | | | descartadas = ___ |
| (f) Falla de baro | | | | liberación→disparo = ___ s |
| (g) Falla de IMU | | | | h_despliegue = ___ m |
| (h) ARMAR / PRG | | | | armado→separación = ___ s |
| (i) MISION_BORRAR | | | | |
| (j) Alcance 250 m | | | | ver tabla |

## 4. Problemas frecuentes

| Síntoma | Qué hacer |
|---|---|
| `sim_serial` no encuentra el puerto / "Acceso denegado" | Otro programa lo tiene abierto (monitor, estación): ciérralo. |
| La placa no responde a `SIM_INICIO` | Espera 3 s tras abrir el puerto (puede reiniciarse); usa `--espera 4`. |
| `ERR hay una mision en curso` | `MISION_BORRAR` (la placa reinicia) y repite. |
| La estación dice "ESPERANDO DATOS" en la simulación | En modo simulación la placa **no transmite** hasta la primera línea `SIM`; empieza `sim_serial`. |
| Estados que no coinciden con esta guía | Confirma `MODO A` (el modo se guarda en NVS) y que el CSV es del mismo modo. |
| `sim_descartadas` > 0 en `STAT` | El PC mandó más rápido de lo que la placa consume: usa `--speed 1`. |
| `ERROR: la placa no confirmo todas las lineas` | Firmware sin CRC (sube `heltec_banco` de nuevo) o la placa se reinició durante el vuelo. Repite la corrida. |
| `sim_crc=0` en `STAT` | La placa no recibió líneas con CRC: `sim_serial.py` viejo o `--sin-crc`. |
| Vuelo con disparo muy temprano, h_max de cientos de metros de más | Líneas dañadas aceptadas (firmware o `sim_serial.py` sin CRC). Ver "Integridad del enlace" en §1.7. |
| Pérdida alta con las placas a 1 m | Revisa antenas; separa las placas ≥ 1 m (saturación del receptor). |
| Quedó el modo B o C guardado | `MODO A` en el monitor. |
| `# AVISO: log al X %` al arrancar | El log pasó del 50 %: `BORRAR`. Lleno (100 %) deja de guardar; al despegar o armar la placa lo vacía sola si quedan < 600 KB. |
