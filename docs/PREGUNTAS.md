# Preguntas abiertas y suposiciones

Cada dato dudoso del firmware se lista aquí. **SUPOSICIÓN** marca el valor que usa el
código mientras no haya respuesta. Al confirmar, actualizar `firmware/include/config.h`
(o el archivo indicado) y mover la pregunta a "Resueltas".

## Hardware y pines

**P1. Pines externos.** SDA/SCL del bus de sensores, señal del servo, energía del servo y
ADC de la pila.
- RESUELTO en parte (electrónica): SDA = GPIO41, SCL = GPIO42 (BMP280 y MPU6500 en el
  mismo bus), señal del servo = GPIO7, sin pin de energía del servo (ver P30).
  Verificado en `pins_arduino.h` de la variante: no chocan con el SX1262 (8-14), la OLED
  (17, 18, 21), `Vext` (36) ni el LED (35). GPIO41/42 son además el `SDA`/`SCL` por
  defecto de la variante, y GPIO7 no es pin de arranque (strapping: 0, 3, 45, 46).
- Pendiente: el pin del ADC de la pila (`PIN_BATT_ADC`, `TODO_CONFIRMAR`). Es opcional:
  con `BATT_ADC_ENABLED 0` (por defecto) el paquete reporta batería 0 y el `#error` no
  lo exige; los tres entornos de la placa compilan sin `PERMITIR_PINES_PENDIENTES`.
  Ver P8b.
- Ya no hay reed ni enclavamiento (ver "Resueltas", P2).
- Nota: en la Heltec V3, GPIO17/18 son el I2C de la pantalla OLED y GPIO36 es `Vext`
  (alimentación conmutada de la OLED y del conector). ¿Los sensores van en el mismo bus
  que la OLED? ¿Se alimentan de `Vext` (entonces el firmware debe encenderlo)?

**P2b. Energía del servo.** ¿Qué nivel enciende el servo (`SERVO_POWER_ON_LEVEL`)?
- RESUELTO: no hay pin de energía (`SERVO_POWER_ENABLED 0`). Ver P30.

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

**P8b. Batería.** Medición opcional de la pila de 9 V con un divisor resistivo al ADC:
`BATT_ADC_ENABLED`, `PIN_BATT_ADC` y `BATT_DIVIDER_RATIO` en `config.h`.
- SUPOSICIÓN: desactivada (`BATT_ADC_ENABLED 0`) → el paquete reporta batería 0.0 V y no
  se exige pin. Para activarla: poner 1, el pin (si falta, `#error` en los entornos de la
  placa) y la relación del divisor (si es 0, `static_assert` en `power.cpp`).
  ¿Electrónica va a poner el divisor? ¿En qué GPIO (con ADC1) y con qué resistencias?
- La lectura de batería propia de la Heltec (GPIO1 + ADC_Ctrl) mide la LiPo de la
  placa, no la pila de 9 V, por eso no se usa.

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
- Además de los paquetes, el log tiene registros de evento de 32 B (byte 0 = `0xEE`): en
  cada arranque y en cada cambio de modo de despliegue, con el modo (ver
  `protocol/packet.md`, "Log", y `decode_log_event` en `protocol/telemetry.py`).

**P14. Formato de salida del receptor.** La SPEC dice "hex más RSSI y SNR".
- SUPOSICIÓN: `HEX,RSSI,SNR` por línea (ver `protocol/packet.md`); errores como
  `ERR,codigo,RSSI,SNR`; líneas informativas empiezan con `#`.

**P15. Fin de AUTOPRUEBA.** El estado 0 dura mientras se calibra el giroscopio y no hay
primera lectura de presión; luego pasa a TIERRA (1). Con los dos sensores muertos se
activa NO-GO (bit7) y pasa a TIERRA igual (sigue transmitiendo, pero nunca despega). Esto
último se cumple desde P44 (antes, sin barómetro la placa saltaba a ASCENSO aunque no
hubiera ningún sensor).

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

**P22. Altura mínima de liberación (DEPLOY_ALT + 40 m: 240 m en A, 90 m en B).**
- SUPOSICIÓN: se evalúa al *inicio* de cada evidencia: al empezar la baja g (criterio
  a), al empezar vz < -12 m/s (b) y con la altura máxima (c). Si se evaluara al confirmar,
  una liberación a ~245 m se rechazaría porque al confirmar ya cayó varios metros.

**P23. Disparo por tiempo sin barómetro (modo A).** `DEPLOY_T_SIN_BARO_A_S = 7 s` (mismo
valor que el simulador). Para el modo B, ver P32.
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
  Con el interruptor deslizable en la línea de la pila, apagar la placa a mitad de una
  prueba (o un corte de corriente) deja el registro en NVS. Recomendación: mandar
  `MISION_BORRAR` antes de cada vuelo o prueba (ver `docs/PRUEBA_VIDEO.md`).
- Escribir en flash detiene la caché de ambos núcleos unos milisegundos; las escrituras
  de NVS son pocas (ya pasa lo mismo con el log).

**P27. Sensores a baja frecuencia en TIERRA.** La SPEC pide "sensores a baja frecuencia
(ahorro de pila)".
- SUPOSICIÓN: no implementado; el lazo sigue a 100 Hz en tierra. El consumo lo dominan el
  ESP32 y el radio (el MPU6500 gasta ~3 mA); en tierra solo bajan la TX (1 Hz) y el log
  (1 Hz). Un ahorro real requeriría bajar la frecuencia de la CPU o light-sleep.

**P28. Modo DEMO_SEPARACION.** Entornos `heltec_demo` y `heltec_banco`.
- SUPOSICIÓN: al armar (comando `ARMAR` o toque largo del botón PRG, ver P34) el servo se
  alimenta en -10° y, a los `DEMO_DELAY_S` s, hace la misma secuencia del vuelo (con
  reintentos si no hay pico de aceleración). Mientras cuenta y después del disparo: TX
  5 Hz y log 50 Hz. El instante de separación queda en el log (registro forzado ese
  ciclo, con bit2) y por serial (`# SEPARACION t_ms=...`). Se puede re-armar para repetir
  la toma; `DESARMAR` o un segundo toque largo durante la cuenta cancelan.

**P29. Modo simulación por serial (prueba B5).** Formato en `docs/sim_input.md`.
- El repositorio no define la prueba B5. SUPOSICIÓN: prueba en banco del firmware
  completo con un vuelo de `flight_sim.py` enviado por USB (`tools/sim_serial.py`), con
  el servo y el radio reales.
- SUPOSICIÓN: solo se puede entrar en los estados 0 y 1, no escribe NVS, y una línea es
  un ciclo de control (el ritmo lo pone la PC).

**P30. Servo sin pin de energía (riesgo de giro al arrancar).** El MG90S se alimenta
directo del regulador de 5 V, así que tiene energía desde que se enciende la placa.
- Riesgo: un servo analógico con energía y una señal flotante o con ruido puede girar.
  Esto puede pasar durante el arranque del ESP32 (antes de que el firmware configure
  GPIO7), al reiniciarse en vuelo, o si un pulso espurio en la línea se interpreta como
  orden. Un giro hacia -80° en tierra o en el cohete liberaría el mecanismo antes de
  tiempo. Además, al conectar la pila el servo puede dar un tirón y una caída de tensión
  del regulador puede reiniciar la placa.
- Qué hace el firmware: GPIO7 queda en entrada con pull-down desde `servo_hw::begin()`
  (lo primero de `power::begin()`), y solo se genera PWM cuando la máquina de estados lo
  pide, es decir, desde el estado 3 (CAIDA, -10°) en adelante. En `heltec_demo` y
  `heltec_banco` también hay PWM a -10° mientras el demo está armado. Sin PWM, el MG90S
  no hace fuerza: el mecanismo queda retenido solo por fricción y por su geometría.
- SUPOSICIÓN: sin pulsos el MG90S se queda quieto, y en reset el GPIO7 del ESP32-S3 está
  en alta impedancia (no es pin de arranque). Se recomienda a electrónica una
  resistencia de pull-down externa de 10 kΩ en la señal (el pull-down interno solo actúa
  después de que corre el firmware), que el mecanismo no se pueda abrir sin fuerza del
  servo, y probar en banco varios arranques y reinicios con el mecanismo montado.
  Alternativa más segura: un MOSFET de energía (`SERVO_POWER_ENABLED 1` con
  `PIN_SERVO_POWER` y `SERVO_POWER_ON_LEVEL`; el código ya lo soporta).

## Modos de despliegue, botón PRG y entornos (etapa 3)

**P31. Modo de despliegue (`DEPLOY_MODE`).** A (450 m -> 200 m), B (100 m -> 50 m) y C
(tinaco), por defecto A. Comando `MODO A|B|C`; `MODO` solo muestra el actual.
- SUPOSICIÓN: el modo solo se puede cambiar en AUTOPRUEBA o TIERRA (con la misión en
  curso responde `ERR`), para que un comando por error no cambie las alturas en el aire.
  Se guarda en NVS en una clave aparte del registro de misión ("modo"), así que se
  conserva al apagar y en un reinicio en vuelo. Se aplica también en el modo simulación
  por serial (y se guarda en NVS: es configuración, no misión).
- "Visible en el log": registro de evento en `/vuelo.bin` en cada arranque (con el modo
  vigente) y en cada cambio (ver P13). El paquete de telemetría no lleva el modo (no hay
  bytes libres); la estación terrena no lo ve. ¿Hace falta transmitirlo?

**P32. Parámetros del modo B que la tarea no fija.** Altura mínima de liberación 90 m y
h_disp = 50 + |vz| * t_lat, como se pidió. Lo demás:
- SUPOSICIÓN: respaldo por altura `DEPLOY_RESPALDO_B_M = 40 m` (en A es 200 - 40 =
  160 m; en B, 50 - 40 = 10 m sería demasiado tarde). A ~25 m/s, de h_disp (~62 m) a
  40 m pasan ~0.9 s, así que en B el respaldo de altura suele llegar antes que el de la
  compuerta (1.5 s, el mismo que en A).
- SUPOSICIÓN: disparo por tiempo sin barómetro `DEPLOY_T_SIN_BARO_B_S = 1.5 s` desde la
  liberación detectada: con liberación a 100 m despliega a ~65 m (a 2 s serían ~53 m, y
  con una liberación a 90 m, ~43 m). Más alto es más seguro, como en A (P23).
- En el simulador (`flight_sim.py --mode B`) el despliegue queda a 49.6 m y 49.4 m
  (semillas 1 y 7).

**P33. Modo C (tinaco).** "Al detectar liberación (caída libre, sin condición de altura),
dispara de inmediato".
- SUPOSICIÓN: la caída libre es el criterio (a) sin la confirmación por vz: |a| < 0.35 g
  durante 150 ms basta. Esperar vz < -8 m/s tardaría ~0.8 s (≈3 m de caída), más que
  la caída desde un tinaco. También vale el criterio (b) (vz < -12 m/s durante 0.4 s, por
  si el IMU murió); el (c) (25 m bajo el máximo) no aplica.
- SUPOSICIÓN: la detección está activa ya en TIERRA (el CanSat no sube los 20 m del
  despegue). Pasa de TIERRA a CAIDA (congela p0 y marca la misión en NVS) y dispara en el
  ciclo siguiente: el log registra el estado 3 un ciclo y luego el 4.
- Riesgo: en C, cualquier caída de más de ~11 cm (150 ms) dispara el servo, también por
  accidente en la mesa, y la falsa liberación de un dron también dispararía. Por eso C
  no es para el concurso: volver a `MODO A` antes de volar.
- En el simulador, la caída desde 8 m dura ~1.3 s: el impacto llega mientras el servo
  está en el retorno a -10° del primer reintento, así que no confirma y hace los 3
  intentos (en el suelo). El pico de 2 g del impacto podría tomarse como confirmación si
  cae dentro de una ventana de -80°; en C eso no cambia nada.

**P34. Botón PRG y LED (modo demo).** GPIO0 (PRG/BOOT) y LED_BUILTIN (GPIO35), de
`pins_arduino.h` de la variante.
- GPIO0 es pin de arranque: si se mantiene presionado PRG al encender, el ESP32-S3 entra
  en modo de descarga y el firmware no corre. Presionarlo solo con la placa ya
  encendida. El modo de vuelo normal no configura ni lee el botón ni el LED.
- SUPOSICIÓN: el LED parpadea rápido durante la cuenta (100 ms encendido / 100 ms
  apagado), queda encendido fijo después del disparo y se apaga al cancelar.
- SUPOSICIÓN: toque largo = 1 s presionado; se detecta al cumplir el segundo (sin esperar
  a soltar) y una sola vez por pulsación. Durante la cuenta, un toque largo cancela;
  después del disparo, re-arma (otra toma). `ARMAR` por serial siempre (re)arma y
  reinicia la cuenta si ya estaba contando.
- SUPOSICIÓN: "con la bandera de disparo activa" se aplica desde el disparo: durante la
  cuenta el paquete va a 5 Hz sin el bit 2, para que el primer paquete con el bit marque
  el instante de la separación (`protocol/packet.md`). ¿La querían encendida también
  durante la cuenta?

**P35. Entorno `heltec_banco`.** Ahora es `heltec_demo` con `BATT_ADC_ENABLED` forzado a
0. Con la configuración por defecto (ADC de la pila desactivado) los dos binarios hacen
lo mismo; la diferencia aparece si se activa la medición de la pila.
- SUPOSICIÓN: se queda (se pidió). Aviso: NUNCA para vuelo (modo demo y sin batería).

## Estación terrena (etapa 4, `ground/`)

**P36. Baudios y formato.** `rx_station` usa `Serial.begin(115200)` (verificado en
`firmware/src/rx/main.cpp`).
- SUPOSICIÓN: la línea `HEX,RSSI,SNR` (P14) es la única que se interpreta; `# ...` son
  mensajes y se ignoran, `ERR,...` cuenta como error de radio, y cualquier otra cosa se
  cuenta como línea inválida y se descarta. El paquete no lleva CRC, así que además se
  exige versión = 1 y estado 0 a 6 para aceptarlo.

**P37. Tiempo de misión.** La SPEC no dice cuándo empieza.
- SUPOSICIÓN: reloj de la placa (`t_ms`) desde el *primer paquete recibido* de la sesión
  (no desde el despegue); si la placa reinicia (`t_ms` retrocede) el tiempo continúa. La
  línea de tiempo muestra la hora de la PC y el T+ de cada cambio de estado. ¿Prefieren
  que el cronómetro arranque en ASCENSO o al pulsar INICIAR TOMA DE DATOS?

**P38. Pérdida de paquetes.** Por salto de la secuencia (u16 con vuelta).
- SUPOSICIÓN: % = perdidos / (recibidos + perdidos). Un salto hacia atrás o mayor de
  30 000 se toma como reinicio de la placa y no cuenta como pérdida; los duplicados se
  cuentan aparte. No se detectan los paquetes perdidos antes del primero ni después del
  último. Los paquetes con error de CRC de LoRa (`ERR,...`) los descarta la radio y
  aparecen como pérdida por secuencia. La tasa de paquetes es la de los últimos 5 s.

**P39. Enlace perdido y velocidad vertical.** Más de 3 s sin paquetes válidos = enlace
perdido (reloj de la PC; con `--speed` mayor que 1 en `--sim` o `--replay` esos 3 s
siguen siendo reales). La velocidad vertical de la tarjeta es el promedio de 1 s (por
`t_ms` de la placa).
- SUPOSICIÓN: verde si el promedio está entre 8 y 12 m/s de descenso y ámbar fuera de
  rango, solo en el estado DESCENSO (5); en los demás estados la tarjeta no se colorea.
  Batería 0 se muestra como "N/D" sin alarma (la placa no mide la pila, P8b).

**P40. Grabación.** Un CSV y un crudo por sesión en `ground/data/`.
- SUPOSICIÓN: la grabación no empieza sola (salvo `--auto-record`): lo recibido antes de
  pulsar INICIAR no se guarda (se ve en pantalla). FINALIZAR pide confirmación para
  evitar un clic accidental. Exportar descarga el CSV de la sesión actual o, si ya
  terminó, el de la última sesión. El CSV tiene `hora_pc`, `epoch_pc`, todas las
  variables del paquete, `rssi_dbm`, `snr_db` y `estado_nombre`; `--replay` lee ese
  formato. Cada línea se vacía a disco al llegar. ¿El reglamento pide algún formato o
  campo específico?

**P41. Interfaz.**
- SUPOSICIÓN: se proyecta en 1920x1080 con el zoom del navegador al 100 %; todo escala
  con la altura de la ventana (rem), pero solo se revisó en Edge (Chromium) a 1920x1080,
  no en Firefox ni en pantallas 4:3. El servidor escucha en 127.0.0.1 sin contraseña; si
  se necesita verlo desde otra laptop, usar `--host 0.0.0.0` en una red de confianza.
- El indicador de actitud es un horizonte artificial con anillo de rumbo (yaw relativo,
  0 = rumbo al arrancar). Con los signos de roll/pitch del firmware (P6) es aproximado:
  validar el sentido de giro con el CanSat en la mano. En la gráfica el yaw se corta al
  dar la vuelta en ±180° (el autogiro gira unos 90°/s).
- Al recargar la página se recuperan los últimos ~6000 paquetes (~20 min a 5 Hz) y los
  eventos; el CSV tiene todo desde que se pulsó INICIAR. El modo de despliegue (P31) no
  viaja en el paquete, así que la estación no lo muestra.

**P42. Logotipo.** `ground/static/logo.png` (el equipo debe poner el archivo). Sin él se
muestra un marcador amarillo con la ruta.

**P43. Pruebas sin la placa.** El puerto serie real no se pudo probar sin la placa
receptora: se probó con un puerto inexistente (el servidor avisa y reintenta), con
`--sim`, con `--replay` y con pruebas automáticas. Falta probar con la Heltec real: la
reconexión al desenchufar y enchufar el USB y que el parser tolere el arranque de la
placa (mensajes `#` y posible basura en la primera línea).

## Pruebas de sistema sin sensores (etapa 5)

**P44. Placa sin sensores (banco con solo dos Heltec).** Revisando el firmware: sin IMU ni
barómetro la placa llegaba a TIERRA en un ciclo y, por la regla de P20 (baro muerto →
ASCENSO de inmediato), pasaba a ASCENSO con NO-GO, **guardaba una misión en NVS** (el
siguiente arranque decía `REINICIO EN VUELO`) y bloqueaba el modo simulación (solo entra
en los estados 0 y 1). No se reiniciaba ni se quedaba colgada.
- Ajuste (único cambio al comportamiento de la máquina de estados): con IMU **y** barómetro
  muertos, TIERRA no pasa a ASCENSO (se queda con NO-GO, como decía P15). Con un solo
  sensor vivo todo sigue igual (P20); la prueba `test_sin_sensores_se_queda_en_tierra` lo
  verifica. El vuelo normal (sensores sanos) no cambia.
- Comando nuevo `SIM_INICIO`: entra al modo simulación (o lo reinicia de cero) en los
  estados 0 y 1 o ya en simulación; con una misión real en curso responde `ERR`. Sirve
  para vuelos simulados seguidos (sin él, tras ATERRIZADO las líneas siguen en el estado 6)
  y no depende de los sensores. `sim_serial.py` lo manda solo (y `MODO` con `--modo`).
- SUPOSICIÓN: no se pudo probar en una placa (solo native). Falta confirmar que
  `Adafruit_BMP280::begin` y las lecturas I2C sin dispositivo no tardan ni cuelgan el arranque
  (por el código de las librerías no debería: el sondeo falla de inmediato).

**P45. Comandos y mensajes agregados para las pruebas.**
- `MISION_PRUEBA` guarda una misión *falsa* en ASCENSO para probar el reinicio en vuelo y
  `MISION_BORRAR` sin despegar. Solo en estados 0 y 1 y fuera de la simulación. Si se
  usa y se olvida, el siguiente arranque cree que es un reinicio en vuelo: borrar con
  `MISION_BORRAR`.
- `# DISPARO` (y `# SEPARACION`) ahora imprime `h`, `vz` (estimados en el ciclo del
  disparo) y `h_despliegue = h - |vz|·0.5`; `# ESTADO` y `# DEMO ARMADO` imprimen `t_ms`;
  `STAT` agrega `liberaciones_descartadas`, `disparo_h`, `disparo_vz` y `h_despliegue`.
  Solo es información por serial: no cambia el control.
- `h_despliegue` es del estimador a bordo y dispara en el umbral (por construcción da
  ≈ 200 m); la altura real de despliegue (198.3 y 199.4 m con las semillas 1 y 7) la mide
  `pio test -e native` con la verdad del simulador.

**P46. Abrir el puerto puede reiniciar la placa.** Con el CP210x de la Heltec V3, abrir el
puerto serie puede reiniciar la placa (DTR/RTS).
- SUPOSICIÓN: `sim_serial.py` abre el puerto con DTR y RTS en bajo y espera `--espera 2`
  segundos antes de mandar `SIM_INICIO`. Si aun así se reinicia (se perdería `SIM_INICIO`),
  subir `--espera`. Un reinicio durante el vuelo simulado lo estropea (la simulación no se
  guarda en NVS).

**P47. Criterio de la prueba de alcance a 250 m.** El reglamento pide transmitir a 250 m;
no define cuántos paquetes ni qué calidad.
- SUPOSICIÓN: pasa con pérdida ≤ 5 %, RSSI promedio ≥ -100 dBm y SNR promedio ≥ 0 dB en
  2 minutos a 1 Hz (≈ 120 paquetes). Con SF7/250 kHz la sensibilidad es ≈ -118 dBm y el
  límite de SNR ≈ -7.5 dB (valores aproximados), o sea ≈ 18 dB de margen. La prueba se hace
  en tierra, con la placa transmitiendo a 1 Hz en TIERRA (sin PC), sin la placa en el
  aire: es más exigente que el vuelo en visibilidad directa, pero no incluye el efecto
  de la antena montada en el CanSat ni del movimiento. ¿El jurado exige otro número?

**P48. Alcance de las pruebas sin sensores.** Con la simulación no se prueba el I2C real,
el servo (giro y confirmación por pico de 2 g), la batería, el ruido real de los sensores
ni el vuelo real. Para el servo sin conectar, la secuencia sin confirmación dura 3.6 s
(1 s + 2 × (0.3 s + 1 s)): es lo esperado en las pruebas (h) y (g). La guía es
`docs/PRUEBAS_SISTEMAS.md`.

## Resueltas

**P2. Reed y enclavamiento (DECISIÓN DE HARDWARE).** Ya no se usa interruptor reed ni
enclavamiento por GPIO. El interruptor es un deslizable (Steren SCM-122) que corta la
línea de la pila de 9 V: al encender, la placa arranca y entra sola a AUTOPRUEBA ->
TIERRA, sin armado. El firmware no lee ningún pin de encendido. Se quitaron del firmware,
`config.h`, la documentación y las pruebas los pines del reed y del enclavamiento, su
polaridad y el armado del modo demo por imán (ahora es con el botón PRG, P34).
