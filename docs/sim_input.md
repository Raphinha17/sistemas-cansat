# Modo simulación por serial

Permite correr el firmware de vuelo completo en la placa (filtros, máquina de estados,
servo, telemetría y log) con lecturas sintéticas en lugar de los sensores. Sirve para la
prueba en banco (B5): se manda un vuelo de `tools/flight_sim.py` por USB y se observa que
el servo dispara en el momento correcto y que la estación terrena recibe los paquetes.

Implementación: `firmware/lib/flight_core/sim_input.cpp` (formato, probado en native) y
`firmware/src/flight/main.cpp` (manejo del modo).

## Formato de línea

Una línea de texto ASCII por ciclo del lazo de control (10 ms a 100 Hz), terminada en
`\n` (se acepta `\r\n`), a 115200 baudios por el puerto USB de la placa (UART0 a través
del puente CP2102):

```
SIM,t_ms,ax,ay,az,gx,gy,gz,p_pa,temp_c*CRC
```

`*CRC` es opcional (ver **Integridad**): `*` y 4 dígitos hexadecimales del
CRC-16/CCITT-FALSE de todo lo anterior al `*` (polinomio 0x1021, valor inicial 0xFFFF; en
Python `binascii.crc_hqx(texto, 0xFFFF)`). `tools/sim_serial.py` siempre lo manda.

| Campo | Unidad | Notas |
|---|---|---|
| `SIM` | — | prefijo exacto, en mayúsculas |
| `t_ms` | ms | reloj de la simulación (entero ≥ 0). Lo usan la máquina de estados y el campo `t_ms` del paquete |
| `ax, ay, az` | g | fuerza específica en ejes del sensor (en reposo nivelado: 0, 0, 1) |
| `gx, gy, gz` | °/s | giroscopio crudo, **con** sesgo (el firmware lo calibra en la autoprueba) |
| `p_pa` | Pa | presión |
| `temp_c` | °C | temperatura |

Exactamente 9 campos después de `SIM`. Máximo 160 caracteres por línea; una línea más
larga se descarta completa.

**Lecturas fallidas:** un campo vacío o `nan` significa que esa lectura falló en ese
ciclo, como un error de I2C:
- si falta cualquiera de los seis campos del IMU, falla el IMU en ese ciclo
  (más de 1 s seguido de fallas → IMU muerto, bandera bit6);
- si falta `p_pa`, falla el barómetro (más de 1 s seguido → bandera bit5).
- Un barómetro **congelado** se simula repitiendo el mismo `p_pa` durante más de 1 s.

Ejemplos:
```
SIM,1230,0.01,-0.02,1.003,0.5,-1.25,0.1,100912.5,24.87
SIM,40000,,,,,,,96340.0,22.10        <- IMU fallido
SIM,40010,0.02,0.01,1.0,0.4,-1.2,0.1,nan,22.1   <- barómetro fallido
```

## Comportamiento

- **Entrada al modo:** el comando `SIM_INICIO` o la primera línea `SIM,...` válida, solo
  si la placa está en AUTOPRUEBA (0) o TIERRA (1). En cualquier otro estado se ignora (nunca
  se activa con la misión en curso). La placa responde `# SIM: modo simulacion activado ...`
  o `OK SIM_INICIO ...`.
- **`SIM_INICIO`** deja la placa en el estado 0 y reinicia el núcleo con la primera línea: sirve
  para repetir vuelos simulados seguidos (sin él, tras ATERRIZADO las líneas siguen en el
  estado 6) y no depende de los sensores. Una placa **sin sensores** (banco con solo las
  Heltec) queda en TIERRA con NO-GO y acepta el modo simulación sin ajustes. En modo
  simulación la placa no transmite hasta recibir la primera línea.
- Al entrar, el núcleo arranca de cero con los datos sintéticos: autoprueba, sesgo del
  giroscopio (las primeras 200 líneas deben ser en reposo) y p0 nuevos.
- **Un ciclo por línea:** el lazo avanza solo cuando llega una línea, así que el ritmo lo
  pone la PC. Los filtros suponen 10 ms entre líneas (`t_ms` debe avanzar de 10 en 10).
  El barómetro se lee en una de cada 4 líneas (25 Hz), igual que el sensor real.
- La placa guarda hasta 32 líneas en cola; si la PC manda más rápido de lo que se
  consume, las sobrantes se descartan (`STAT` → `sim_descartadas`).
- Ancho de banda: una línea de `sim_serial.py` mide 64 caracteres en promedio (70 como
  máximo) más 5 del CRC: 100 líneas/s ≈ 6.9 kB/s, el 60 % de los 11.52 kB/s de 115200
  baudios (8N1). El vuelo cabe a tiempo real; no conviene ir a más de ~1.3× (`--speed`).
  El búfer de recepción de la placa es de 8 KB (`SERIAL_RX_BUF`, ~1.2 s de líneas).
- **No se escribe la NVS** en este modo: un ensayo en banco no deja una misión "en curso"
  para el siguiente arranque. Por lo mismo, el reinicio en vuelo no se prueba así (está
  cubierto por las pruebas native).
- La batería se reporta como 0.0 V. Telemetría LoRa, log y servo funcionan normal.
- **Modo de despliegue:** se usa el que tenga la placa (`MODO`, guardado en NVS). Para
  probar el modo B o C, mandar `MODO B` (o `MODO C`) antes de las líneas y generar el
  vuelo con el mismo modo (`flight_sim.py --mode B`).
- **Salida del modo:** comando `SIM_FIN` (vuelve a los sensores reales y recalibra) o
  reinicio de la placa.
- Mensajes de la placa durante la simulación: `# ESTADO n banderas=0x.. t_ms=..` en cada
  cambio de estado y `# DISPARO t_ms=.. h=.. vz=.. h_despliegue=..` al disparar. Un estado que
  dura un solo ciclo (CAÍDA en el modo C) puede no llegar a imprimirse.

## Integridad: CRC, huecos y reenvío

**Por qué.** En la prueba (b) de `docs/PRUEBAS_SISTEMAS.md` (19-nov-2026) la placa recibió
17 318 de 19 391 líneas, 137 inválidas, y disparó a T+45 s. El puerto serie pierde bloques
de bytes. Causa más probable (deducida del código, no medida en la placa; PREGUNTAS P49):
la tarea comms escribe el log en LittleFS (cada segundo, y cada escritura tras
un `flush` copia el último bloque de 4 KB a uno nuevo) y mientras la flash está ocupada
la caché está apagada y la interrupción de la UART no corre: solo caben 128 B en la FIFO
(~18 ms de líneas). Además el búfer de recepción del núcleo Arduino era de 256 B. Un
bloque perdido une la **cabeza de una línea con la cola de otra**; si el corte cae en el
mismo campo de las dos, la línea pegada tiene 9 campos y pasa el formato con valores
falsos (p. ej. `...,10` + `104,24.00` → 10 104 Pa ≈ 16 km). El Kalman salta, la liberación
se detecta por "h bajo h_max − 25 m" y el disparo es inmediato (vz = +130 m/s da
h_disp = 265 m). Los huecos solos (líneas que faltan) también rompen la suposición de
10 ms entre líneas, pero sin valores falsos producen errores pequeños (con 10 % de
líneas faltantes, 1.2 m en h_despliegue y los estados en orden). `pio test -e native`
(`test_sim_link`) reproduce las dos cosas con un enlace que pierde bloques de bytes.

**Qué hace la placa** (`SimLink` en `firmware/lib/flight_core/sim_input.cpp`):
- Desde la primera línea con CRC correcto, la sesión es **estricta**: solo pasan al lazo
  las líneas con CRC correcto **y** `t_ms` igual al esperado (anterior + 10 ms).
- Línea dañada (CRC, formato, más de 160 caracteres, fragmento sin `SIM,` o sin CRC) o
  adelantada (faltan líneas antes): se descarta y la placa responde
  `SIM_REPITE <t_ms esperado>`. Mientras no llegue, no lo repite antes de 250 ms.
- Línea repetida (`t_ms` ya procesado, tras un reenvío): se descarta sin aviso.
- `SIM_PROXIMA` responde `SIM_PROXIMA <t_ms esperado>` (`-1` si aún no llegó ninguna).
- Así el lazo procesa **exactamente** las líneas del CSV, en orden, y dt = 10 ms es exacto
  (no hace falta tocar los filtros ni usar el tiempo de llegada). El tiempo de la
  máquina de estados sigue siendo el `t_ms` de la línea.
- Sin CRC (líneas escritas a mano, `--sin-crc`): como antes, pasan todas las de formato
  válido; los huecos solo se cuentan.
- `STAT`: `sim_rx` aceptadas, `sim_invalidas`, `sim_huecos` (líneas adelantadas),
  `sim_repetidas`, `sim_repite` (pedidos de reenvío), `sim_crc=1` si la sesión es estricta.

**Qué hace la PC** (`sim_serial.py`): al recibir `SIM_REPITE t` vuelve a esa línea y sigue
a tiempo real desde ahí (sin ráfagas); al terminar pregunta `SIM_PROXIMA` y reenvía lo que
falte. Cada pérdida retrasa el resto del envío: con ~1 pérdida por segundo el vuelo de
3 min 14 s tarda ~15 % más en la PC. Los tiempos de la simulación (`t_ms`, T+ de la
estación) no cambian.

**Baud rate.** No se sube: el caudal es el 60 % de 115200 y el problema son las pausas de
la flash, que con más baudios serían iguales (a mayor velocidad, una ráfaga llena antes
la FIFO de 128 B). Además `monitor_speed`, el monitor y `rx_station` siguen a 115200.

## Prueba en banco con `tools/sim_serial.py`

```
py tools/flight_sim.py --raw-csv vuelo.csv --seed 1
py tools/sim_serial.py --csv vuelo.csv --port COM5 --modo A --fin
```

`sim_serial.py` manda `SIM_INICIO` (y `MODO` con `--modo`) y luego convierte cada fila del CSV crudo (100 Hz) en una línea `SIM,...*CRC`, la
envía a tiempo real, atiende los `SIM_REPITE` y muestra lo que imprime la placa. Al final
pide `SIM_PROXIMA` y `STAT`, e imprime `lineas del CSV=... enviadas=... (reenviadas=...)`
y `OK: la placa recibio las N lineas en orden` (código de salida 0) o `ERROR: ...`
(código 2). Con `--sin-crc` manda el formato anterior (para un firmware viejo; sin
reenvío ni confirmación). Las filas con el IMU en ceros
(`flight_sim.py --fail imu`) se mandan con los campos del IMU vacíos. Con `--dry-run`
solo escribe las líneas, sin placa. Escenarios útiles: ver el encabezado de
`tools/flight_sim.py` (`--mode B`, `--mode C`, `--fail baro`, `--fall-tilt 70`,
`--false-release 70`, ...).

Durante un vuelo simulado completo (~3 min) el servo se alimenta al entrar en CAIDA y
dispara al cruzar h_disp; **el mecanismo debe estar montado o el servo libre**.
