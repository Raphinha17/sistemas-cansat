# Modo simulación por serial

Permite correr el firmware de vuelo completo en la placa (filtros, máquina de estados,
servo, telemetría y log) con lecturas sintéticas en lugar de los sensores. Sirve para la
prueba en banco (B5): se manda un vuelo de `tools/flight_sim.py` por USB y se observa que
el servo dispara en el momento correcto y que la estación terrena recibe los paquetes.

Implementación: `firmware/lib/flight_core/sim_input.cpp` (formato, probado en native) y
`firmware/src/flight/main.cpp` (manejo del modo).

## Formato de línea

Una línea de texto ASCII por ciclo del lazo de control (10 ms a 100 Hz), terminada en
`\n` (se acepta `\r\n`), a 115200 baudios por el puerto USB de la placa:

```
SIM,t_ms,ax,ay,az,gx,gy,gz,p_pa,temp_c
```

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
- Ancho de banda: una línea típica mide ~70 caracteres; 100 líneas/s ≈ 7 kB/s, dentro de
  los ~11.5 kB/s de 115200 baudios. No conviene ir a más de ~1.3× tiempo real.
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

## Prueba en banco con `tools/sim_serial.py`

```
py tools/flight_sim.py --raw-csv vuelo.csv --seed 1
py tools/sim_serial.py --csv vuelo.csv --port COM5 --modo A --fin
```

`sim_serial.py` manda `SIM_INICIO` (y `MODO` con `--modo`) y luego convierte cada fila del CSV crudo (100 Hz) en una línea `SIM,...`, la
envía a tiempo real y muestra lo que imprime la placa. Las filas con el IMU en ceros
(`flight_sim.py --fail imu`) se mandan con los campos del IMU vacíos. Con `--dry-run`
solo escribe las líneas, sin placa. Escenarios útiles: ver el encabezado de
`tools/flight_sim.py` (`--mode B`, `--mode C`, `--fail baro`, `--fall-tilt 70`,
`--false-release 70`, ...).

Durante un vuelo simulado completo (~3 min) el servo se alimenta al entrar en CAIDA y
dispara al cruzar h_disp; **el mecanismo debe estar montado o el servo libre**.
