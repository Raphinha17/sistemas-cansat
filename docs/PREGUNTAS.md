# Preguntas abiertas y suposiciones

Cada dato dudoso del firmware se lista aquí. **SUPOSICIÓN** marca el valor que usa el
código mientras no haya respuesta. Al confirmar, actualizar `firmware/include/config.h`
(o el archivo indicado) y mover la pregunta a "Resueltas".

## Hardware y pines

**P1. Pines externos.** SDA/SCL del bus de sensores, señal del servo, energía del servo,
reed, enclavamiento y ADC de la pila.
- SUPOSICIÓN: ninguna. Están como `TODO_CONFIRMAR` en `config.h` y el entorno
  `heltec_wifi_lora_32_V3` no compila (#error) hasta que se den.
- Nota: en la Heltec V3, GPIO17/18 son el I2C de la pantalla OLED y GPIO36 es `Vext`
  (alimentación conmutada de la OLED y del conector). ¿Los sensores van en el mismo bus
  que la OLED? ¿Se alimentan de `Vext` (entonces el firmware debe encenderlo)?

**P2. Enclavamiento y reed.** ¿Qué nivel lógico mantiene encendida la placa? ¿El firmware
debe escribir el pin al arrancar, y cuándo puede soltarlo? ¿El reed se lee como entrada
(con qué pull) o solo actúa sobre el enclavamiento?
- SUPOSICIÓN: el firmware no escribe `PIN_LATCH`. El reed solo se lee en el modo
  `DEMO_SEPARACION` (para armar): activo en bajo con pull-up interno
  (`REED_ACTIVE_LEVEL 0`, `REED_PULLUP 1` en `config.h`).

**P2b. Energía del servo.** ¿Qué nivel enciende el servo (`SERVO_POWER_ON_LEVEL`)?
- SUPOSICIÓN: ninguna. Está como `TODO_CONFIRMAR` en `config.h` y forma parte del mismo
  `#error` que los pines. Aun así, el hardware debe dejar el servo apagado por defecto
  mientras el ESP32 arranca (antes de que el firmware configure el pin).

**P3. Direcciones I2C.** BMP280 0x76/0x77 (pin SDO) y MPU6500 0x68/0x69 (pin AD0).
- SUPOSICIÓN: BMP280 = 0x76, MPU6500 = 0x68.

**P4. Radio SX1262 de la Heltec V3.** Pines: salen de la variante de la placa
(`SS`, `SCK`, `MISO`, `MOSI`, `RST_LoRa`, `BUSY_LoRa`, `DIO0`), no se definen aquí.
- SUPOSICIÓN: TCXO de 1.8 V en DIO3 (`LORA_TCXO_V`) y conmutador de antena por DIO2
  (`setDio2AsRfSwitch(true)`). Así lo configuran proyectos conocidos para esta placa,
  pero no está verificado con el esquemático. Si `radio.begin` falla al arrancar
  (el código de error se imprime por serial), probar 1.6 V.

**P5. Sync word.** La SPEC pide "sync word propio" sin valor.
- SUPOSICIÓN: `0x2B` (distinto de 0x12 privado por defecto y de 0x34 LoRaWAN).
  Debe ser igual en vuelo y en `rx_station` (ambos usan `config.h`).

**P6. Orientación del sensor y eje de la compuerta.** Se supone que los ejes del MPU6500
coinciden con los del cuerpo, con +z hacia arriba cuando el CanSat está en reposo
vertical. ¿Cómo va montada la placa? ¿Qué eje y signo usa la compuerta de 45°?
- SUPOSICIÓN: ejes del sensor = ejes del cuerpo; compuerta con +z
  (`BODY_AXIS_INDEX 2`, `BODY_AXIS_SIGN 1`). La compuerta ya se usa en CAIDA (estado 3):
  si el eje está mal, el disparo saldría siempre por el respaldo de 1.5 s.

**P7. Rangos del MPU6500.**
- SUPOSICIÓN: ±16 g (impacto y despliegue pueden pasar de 8 g) y ±2000 °/s (giro en
  caída libre). Filtros internos ~92 Hz (giroscopio) y ~99 Hz (acelerómetro).
  Con ±16 g la resolución es 0.5 mg, suficiente.

**P8. Configuración del BMP280.**
- SUPOSICIÓN: modo normal, presión x4, temperatura x1, **sin filtro IIR** (el IIR
  retrasa la altura a 35 m/s), standby 1 ms; se lee a 25 Hz. El ruido sin IIR
  (~1–2 Pa ≈ 0.1–0.2 m) lo filtra el Kalman.

**P8b. Batería.** Divisor resistivo de la pila de 9 V al ADC: relación
`BAT_DIVIDER_RATIO`.
- SUPOSICIÓN: 0 (desconocido) → el paquete reporta batería 0.0 V. La lectura de batería
  propia de la Heltec (GPIO1 + ADC_Ctrl) mide la LiPo de la placa, no la pila de 9 V,
  por eso no se usa.

## Software

**P9. Calibración del giroscopio con movimiento.** Si en el arranque el CanSat se mueve,
la ventana de 2 s se rechaza (desviación > 0.5 °/s) y se reintenta.
- SUPOSICIÓN: tras 5 intentos fallidos se usa la última media, sin bandera. ¿Debe
  marcarse NO-GO o fallo IMU en ese caso?

**P10. Tolerancias de las pruebas native** (con `tools/flight_sim.py`, que es un modelo
simplificado, no datos reales):
- Actitud: roll/pitch RMS < 1.5° en tierra/ascenso; error máx. < 5° en caída libre
  (bastante menos que la compuerta de 45°); RMS < 3° en descenso.
- Altura/velocidad: h RMS < 1.5 m, vz RMS < 1 m/s en todas las fases; altura máxima
  ±2 m; altura estimada al cruzar 200 m reales: ±3 m.
- Resultado actual (semillas 1 y 7): actitud RMS ≤ 1.2°, máx ≤ 2.7°; h RMS ≤ 0.3 m,
  vz RMS ≤ 0.22 m/s; altura al cruzar 200 m: 200.1 m. Son **optimistas**: el simulador
  no modela desalineación del sensor, deriva del giroscopio con la temperatura, errores
  de escala ni ráfagas de viento. Hay que revisarlas con datos reales de banco/vuelo.
- El impacto no se evalúa: el simulador pasa de -10 m/s a 0 en un paso y su acelerómetro
  solo registra un pico de 8 g durante 30 ms (≈2.4 m/s de cambio, no 10), así que el
  Kalman tarda ~3 s en volver a vz ≈ 0. Por eso, en las pruebas, ATERRIZADO llega
  ~4.3 s después del impacto (tolerancia 6 s); con un impacto real debería ser ~1–2 s.
- Máquina de estados (test_flight, semillas 1 y 7): despliegue a 198.3 m y 199.4 m
  (h real al disparar − |vz|·t_lat); liberación detectada ~0.8 s después de la real (baja g 150 ms + vz < -8 m/s).

**P11. Ruido de proceso/medición del Kalman.** `KF_ACC_SIGMA = 0.5 m/s²`,
`KF_BARO_SIGMA = 0.4 m`. Ajustados con el simulador; ajustar con datos reales.
Ganancias de Mahony: `Kp = 2.0`, `Ki = 0.02`, compuerta del acelerómetro ±0.15 g.

**P12. p0 y NVS.** p0 = promedio móvil de 30 s (medias de 1 s). Se congela al pasar a
ASCENSO y se guarda en NVS. La bandera "base lista" (bit0) se activa cuando la ventana
de 30 s está completa (y queda activa durante la misión).
- SUPOSICIÓN: el p0 que se congela es el último promedio con el CanSat quieto en el
  suelo (|vz| < 0.5 m/s y |h| < 3 m), no el del instante de la detección: la detección
  ocurre 3 s después de pasar 20 m, y para entonces el promedio de 30 s ya incluye
  ~6 s de subida (≈3 m de error en todas las alturas). Al congelarlo, la altura medida
  salta unos decímetros a metros y el Kalman lo absorbe en <1 s.

**P13. Log.** Se registran paquetes de 32 B en `/vuelo.bin` de LittleFS: 50 Hz
(`LOG_HZ`) en los estados 2 a 5 y 1 Hz (`LOG_HZ_TIERRA`) en 0, 1 y 6, más un registro
extra en cada cambio de estado y en el disparo (instante exacto). Se hace `flush` cada
1 s (y al aterrizar) y se llena hasta el 90 % de la partición. Con la tabla de
particiones por defecto (~1.4 MB de datos) caben ~15 min a 50 Hz; la espera en tierra a
1 Hz casi no ocupa.
- ¿Basta con 15 min? Si no, usar una tabla de particiones con más espacio para datos.
- El log persiste entre reinicios (se agrega al final); se borra con el comando `BORRAR`.

**P14. Formato de salida del receptor.** La SPEC dice "hex más RSSI y SNR".
- SUPOSICIÓN: `HEX,RSSI,SNR` por línea (ver `protocol/packet.md`); errores como
  `ERR,codigo,RSSI,SNR`; líneas informativas empiezan con `#`.

**P15. Fin de AUTOPRUEBA.** El estado 0 dura mientras se calibra el giroscopio y no hay
primera lectura de presión; luego pasa a TIERRA (1). Con los dos sensores muertos se
activa NO-GO (bit7) y pasa a TIERRA igual (sigue transmitiendo, pero nunca despega).

## Máquina de estados, servo y modos (etapa 2)

**P16. Mapeo del servo.** `-10°` (bloqueo) y `-80°` (liberación) con 1500 us = 0° y
11.1 us/° (500–2500 us = ±90°): -10° = 1389 us, -80° = 611 us. PWM 50 Hz, LEDC canal 0.
- ¿Qué convención de ángulo usa el mecanismo? ¿El MG90S llega a 611 us sin forzar? Probar
  primero en banco sin el mecanismo (`SERVO_US_CENTRO`, `SERVO_US_POR_GRADO`).

**P17. Confirmación del despliegue.** La SPEC dice "pico de aceleración (por validar)".
- SUPOSICIÓN: |a| > 2 g (`SERVO_CONFIRMA_G`) dentro de 1 s tras girar a -80°. En el
  simulador la apertura del autogiro llega a ~12 g. Sin IMU no hay confirmación y se
  hacen los 3 intentos. Validar con una prueba de caída real.

**P18. Energía del servo y reintentos.**
- SUPOSICIÓN: "sin energía hasta el estado 3" = el servo se alimenta al entrar en CAIDA
  (3), sostenido en -10°, para que esté firme y sin el tirón del encendido al disparar.
  En DESPLIEGUE (4) gira a -80°; cada reintento vuelve a -10° durante 0.3 s y otra vez a
  -80°. Al terminar la secuencia (confirmada o tras 3 intentos) se le quita la energía.
- La señal queda como entrada con pull-down interno mientras no hay energía (además del
  pull-down externo). Al encender: primero PWM y luego energía; al apagar, al revés.

**P19. Liberación con el barómetro muerto.** Solo por IMU (SPEC).
- SUPOSICIÓN: la confirmación "vz < -8 m/s" se evalúa como *cambio* de vz desde el inicio
  de la ventana de baja g, porque sin barómetro vz solo sale de integrar el acelerómetro
  y deriva (con 0.01 g de sesgo, ~6 m/s en un minuto). No se puede verificar la altura
  mínima de 240 m. Los criterios por vz < -12 m/s y por 25 m bajo el máximo se desactivan.

**P20. Barómetro muerto antes del despegue.** Sin barómetro el despegue no se puede
detectar (h y vz por integración derivan).
- SUPOSICIÓN: se pasa de TIERRA a ASCENSO de inmediato, con la detección de liberación
  por IMU armada (baja g 150 ms + caída de 8 m/s en 1 s ≈ caer 3 m; un manejo normal en
  tierra no la activa). ¿Preferirían NO-GO y no disparar nunca?

**P21. IMU muerto.** Liberación solo por barómetro, disparo sin compuerta. Sin
acelerómetro el Kalman usa velocidad casi constante y vz va retrasada.
- En el simulador el despliegue quedó a 199.4 m; la prueba tolera ±10 m.

**P22. Altura mínima de liberación (240 m).**
- SUPOSICIÓN: se evalúa al *inicio* de cada evidencia: al empezar la baja g (criterio
  a), al empezar vz < -12 m/s (b) y con la altura máxima (c). Si se evaluara al confirmar,
  una liberación a ~245 m se rechazaría porque al confirmar ya cayó varios metros.

**P23. Disparo por tiempo sin barómetro.** `SM_T_DISPARO_SIN_BARO_S = 7 s` (mismo valor
que el simulador).
- Con liberación a 450 m y terminal 35 m/s, 7 s ≈ 290 m; 9 s ≈ 205 m. Pero sin barómetro
  no se sabe a qué altura fue la liberación: si fuera a 300 m, 9 s sería ~55 m.
  SUPOSICIÓN: 7 s (desplegar alto es más seguro que bajo). Tras un reinicio en CAIDA sin
  barómetro se dispara de inmediato (no se sabe cuánto tiempo pasó).

**P24. Aterrizaje.** La SPEC dice "impacto y reposo".
- SUPOSICIÓN: impacto = |a| > 2.5 g, ignorado los primeros 2 s de DESCENSO (choque de la
  apertura). Reposo = | |a| − 1 g | < 0.15 g, |ω| < 10 °/s y |vz| < 1 m/s (con lo que haya
  disponible). ATERRIZADO tras 1 s de reposo después del impacto, o 5 s de reposo sin
  impacto detectado. El |ω| evita confundir el descenso estable bajo el autogiro (|a| ≈ 1 g
  pero girando) con el reposo. Sin barómetro y sin impacto detectado se usa solo el IMU.

**P25. Bandera "vz de descenso fuera de 8–12 m/s".** La SPEC la pide, pero los 8 bits de
banderas del paquete ya están ocupados.
- SUPOSICIÓN: no se transmite; se calcula (promedio de 1 s) y se muestra en `STAT`. La
  estación terrena puede calcularla con vz. ¿Agregarla en una versión 2 del paquete?

**P26. Reinicio en vuelo.** Al pasar a ASCENSO se guarda en NVS: misión en curso, estado,
p0, altura máxima, banderas y sesgo del giroscopio; se actualiza en cada cambio de
estado y cada 10 m de altura máxima nueva (máx. 1 vez/s). Al arrancar con una misión en
curso: sin recalibrar (sesgo y p0 guardados), el Kalman empieza con vz desconocida y,
tras 1 s, se decide el estado:
- guardado ASCENSO: vz < -8 m/s → CAIDA (la liberación ocurrió durante el reinicio); si
  no, ASCENSO. Guardado CAIDA → CAIDA. Guardado DESPLIEGUE → repite la secuencia del
  servo. Guardado DESCENSO → DESCENSO. Durante esa espera en CAIDA el respaldo de 160 m
  sigue activo.
- La misión se da por terminada (se borra la bandera) al entrar en ATERRIZADO.
- Riesgo: si se apaga la placa a mitad de una misión en tierra (p. ej. tras un ensayo
  en que subió > 20 m), el siguiente arranque cree que es un reinicio en vuelo. La placa
  lo avisa por serial; se borra con `MISION_BORRAR`. ¿Hace falta otra protección?
- Escribir en flash detiene la caché de ambos núcleos unos milisegundos; las escrituras
  de NVS son pocas (ya pasa lo mismo con el log).

**P27. Sensores a baja frecuencia en TIERRA.** La SPEC pide "sensores a baja frecuencia
(ahorro de pila)".
- SUPOSICIÓN: no implementado; el lazo sigue a 100 Hz en tierra. El consumo lo dominan el
  ESP32 y el radio (el MPU6500 gasta ~3 mA); en tierra solo bajan la TX (1 Hz) y el log
  (1 Hz). Un ahorro real requeriría bajar la frecuencia de la CPU o light-sleep.

**P28. Modo DEMO_SEPARACION.** Entorno `heltec_demo` (mismo `#error` de pines).
- SUPOSICIÓN: al armar (comando `ARMAR` o flanco del reed hacia "imán presente" después
  del arranque, con antirrebote de 50 ms) el servo se alimenta en -10° y, a los
  `DEMO_DELAY_S` s, hace la misma secuencia del vuelo (con reintentos si no hay pico de
  aceleración). Mientras está armado: TX 5 Hz y log 50 Hz. El instante de separación
  queda en el log (registro forzado ese ciclo, con bit2) y por serial
  (`# SEPARACION t_ms=...`). Se puede re-armar para repetir la toma; `DESARMAR` cancela.
- Si el reed es el propio interruptor de encendido, ¿"armar por imán" significa armar al
  encender? Hoy hace falta un segundo paso del imán después de arrancar.

**P29. Modo simulación por serial (prueba B5).** Formato en `docs/sim_input.md`.
- El repositorio no define la prueba B5. SUPOSICIÓN: prueba en banco del firmware
  completo con un vuelo de `flight_sim.py` enviado por USB (`tools/sim_serial.py`), con
  el servo y el radio reales.
- SUPOSICIÓN: solo se puede entrar en los estados 0 y 1, no escribe NVS, y una línea es
  un ciclo de control (el ritmo lo pone la PC).

## Resueltas
(ninguna todavía)
