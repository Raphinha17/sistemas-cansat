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
- SUPOSICIÓN: el firmware no escribe `PIN_LATCH`; el reed se configura como `INPUT`
  sin pull y no se usa todavía.

**P2b. Energía del servo.** ¿Qué nivel enciende el servo (`SERVO_POWER_ON_LEVEL`)?
- SUPOSICIÓN: no definido; el firmware no maneja ese pin, así que el hardware debe dejar
  el servo apagado por defecto (SPEC: "Servo sin energía hasta el estado 3").

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
  (`BODY_AXIS_INDEX 2`, `BODY_AXIS_SIGN 1`).

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
  solo registra un pico de 8 g durante 30 ms, así que el Kalman tarda ~1 s en volver a
  vz ≈ 0.

**P11. Ruido de proceso/medición del Kalman.** `KF_ACC_SIGMA = 0.5 m/s²`,
`KF_BARO_SIGMA = 0.4 m`. Ajustados con el simulador; ajustar con datos reales.
Ganancias de Mahony: `Kp = 2.0`, `Ki = 0.02`, compuerta del acelerómetro ±0.15 g.

**P12. p0 y NVS.** p0 = promedio móvil de 30 s (medias de 1 s). En esta etapa no hay
máquina de estados: p0 nunca se congela ni se guarda en NVS. La bandera "base lista"
(bit0) se activa cuando la ventana de 30 s está completa.

**P13. Log.** Se registran paquetes de 32 B a 50 Hz (`LOG_HZ`) en `/vuelo.bin` de
LittleFS, se hace `flush` cada 1 s y se llena hasta el 90 % de la partición. Con la tabla
de particiones por defecto de la placa (~1.4 MB de datos) caben ~15 min a 50 Hz.
- ¿Basta con 15 min? Si no, usar una tabla de particiones con más espacio para datos.
- El log persiste entre reinicios (se agrega al final); se borra con el comando `BORRAR`.

**P14. Formato de salida del receptor.** La SPEC dice "hex más RSSI y SNR".
- SUPOSICIÓN: `HEX,RSSI,SNR` por línea (ver `protocol/packet.md`); errores como
  `ERR,codigo,RSSI,SNR`; líneas informativas empiezan con `#`.

**P15. Estado reportado sin máquina de estados.** AUTOPRUEBA (0) mientras se calibra el
giroscopio y no hay primera lectura de presión; TIERRA (1) después.

## Resueltas
(ninguna todavía)
