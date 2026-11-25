// Configuración del firmware CanSat Hellmary. Ver docs/SPEC.md y docs/PREGUNTAS.md.
//
// Pines externos: NO inventar. Los no confirmados quedan como TODO_CONFIRMAR hasta que
// el equipo de electrónica los confirme. Confirmados por electrónica: I2C (41/42) y
// señal del servo (7); verificados contra pins_arduino.h de la variante: no los usan
// el SX1262 (8-14), la OLED (17, 18, 21), Vext (36) ni el LED (35). Los pines del radio
// SX1262 NO están aquí: salen de la variante de la placa (pins_arduino.h de
// heltec_wifi_lora_32_V3).
//
// Encendido: interruptor deslizable (Steren SCM-122) en serie con la pila de 9 V. No pasa
// por ningún GPIO: al encender, la placa arranca y entra sola a AUTOPRUEBA -> TIERRA.
#pragma once

// Valor centinela para pines no confirmados.
#define TODO_CONFIRMAR (-1)

// ---------------------------------------------------------------------------
// Pines externos
// ---------------------------------------------------------------------------
#define PIN_I2C_SDA 41                  // SDA del bus de BMP280 + MPU6500 (confirmado)
#define PIN_I2C_SCL 42                  // SCL del bus de BMP280 + MPU6500 (confirmado)
#define PIN_SERVO_SIGNAL 7              // señal PWM del MG90S (confirmado)

// Energía del servo: NO hay pin de habilitación; el MG90S se alimenta directo del
// regulador de 5 V (PREGUNTAS P30). El firmware solo garantiza que no haya PWM fuera del
// estado 3 en adelante (señal en entrada con pull-down mientras tanto).
#define SERVO_POWER_ENABLED 0
#define PIN_SERVO_POWER (-1)            // no existe (SERVO_POWER_ENABLED = 0)
// Nivel lógico de PIN_SERVO_POWER que ENCIENDE el servo. Sin uso mientras
// SERVO_POWER_ENABLED = 0.
#define SERVO_POWER_ON_LEVEL (-1)

// ADC de la pila de 9 V (opcional). Con BATT_ADC_ENABLED = 0 el paquete reporta batería 0
// y no se exige pin. Para activarlo hacen falta el pin y el divisor (PREGUNTAS P8b).
#ifndef BATT_ADC_ENABLED
#define BATT_ADC_ENABLED 0
#endif
#define PIN_BATT_ADC TODO_CONFIRMAR     // ADC de la pila de 9 V (con divisor)
// Relación del divisor (V_pila / V_adc). Debe ser > 0 si BATT_ADC_ENABLED = 1.
#define BATT_DIVIDER_RATIO 0.0f

// Botón PRG y LED integrado de la Heltec V3 (de pins_arduino.h de la variante: el botón
// PRG/BOOT es GPIO0 y LED_BUILTIN es GPIO35). Solo los usa el modo DEMO_SEPARACION
// (entornos heltec_demo y heltec_banco); el modo de vuelo normal no los toca.
#define PIN_BOTON_PRG 0                 // activo en bajo (pull-up en la placa)
#define PIN_LED_PLACA 35

// ENTORNO DE BANCO (heltec_banco, define CANSAT_BANCO): modo DEMO_SEPARACION y sin
// medición de batería (BATT_ADC_ENABLED forzado a 0; el paquete reporta batería 0).
// *** NUNCA USAR heltec_banco PARA VUELO: es el firmware del modo demo y no mide la pila.
#ifdef CANSAT_BANCO
#undef BATT_ADC_ENABLED
#define BATT_ADC_ENABLED 0
#endif

// La verificación se activa SOLO en los entornos de la placa (vuelo, demo y banco), que
// definen CANSAT_FLIGHT_BOARD. Nunca en native ni en rx_station. Solo se exigen los pines
// de las funciones activas: el ADC de la pila si BATT_ADC_ENABLED = 1 y la energía del
// servo si SERVO_POWER_ENABLED = 1.
// Para compilar a pesar de los pines pendientes (solo pruebas de compilación):
//   -D PERMITIR_PINES_PENDIENTES
#if defined(CANSAT_FLIGHT_BOARD) && !defined(PERMITIR_PINES_PENDIENTES)
#if (PIN_I2C_SDA == TODO_CONFIRMAR) || (PIN_I2C_SCL == TODO_CONFIRMAR) ||           \
    (PIN_SERVO_SIGNAL == TODO_CONFIRMAR) ||                                         \
    (BATT_ADC_ENABLED && (PIN_BATT_ADC == TODO_CONFIRMAR)) ||                       \
    (SERVO_POWER_ENABLED &&                                                         \
     ((PIN_SERVO_POWER == TODO_CONFIRMAR) || (SERVO_POWER_ON_LEVEL == TODO_CONFIRMAR)))
#error "Pines externos pendientes (TODO_CONFIRMAR) en include/config.h; ver docs/PREGUNTAS.md"
#endif
#endif

// ---------------------------------------------------------------------------
// Direcciones I2C
// ---------------------------------------------------------------------------
// BMP280: 0x76 si SDO a GND, 0x77 si SDO a VCC. Suposición: 0x76 (PREGUNTAS P3).
#define BMP280_I2C_ADDR 0x76
// MPU6500: 0x68 si AD0 a GND, 0x69 si AD0 a VCC. Suposición: 0x68 (PREGUNTAS P3).
#define MPU6500_I2C_ADDR 0x68
#define MPU6500_WHO_AM_I_EXPECTED 0x70
#define I2C_CLOCK_HZ 400000

// ---------------------------------------------------------------------------
// Radio LoRa (SPEC: 915 MHz, SF7, BW 250 kHz, CR 4/5, 14 dBm, sync word propio)
// ---------------------------------------------------------------------------
#define LORA_FREQ_MHZ 915.0f
#define LORA_BW_KHZ 250.0f
#define LORA_SF 7
#define LORA_CR 5               // denominador: 4/5
#define LORA_TX_DBM 14
#define LORA_PREAMBLE 8
// Sync word "propio": valor elegido por nosotros, distinto de 0x12 (privado por
// defecto) y 0x34 (LoRaWAN). Suposición, ver PREGUNTAS P5.
#define LORA_SYNC_WORD 0x2B
// TCXO del SX1262 de la Heltec V3 controlado por DIO3. Suposición 1.8 V (PREGUNTAS P4).
#define LORA_TCXO_V 1.8f

// ---------------------------------------------------------------------------
// Lazo de control y tareas
// ---------------------------------------------------------------------------
#define CONTROL_HZ 100
#define CONTROL_CORE 1          // núcleo del lazo de control
#define COMMS_CORE 0            // núcleo de radio + log
#define BARO_DECIM 4            // BMP280 leído cada 4 ciclos = 25 Hz
#define LOG_HZ 50               // registros al log por segundo
#define TX_HZ_TIERRA 1          // TX por defecto (modo Tierra)
#define TX_HZ_PRUEBA 5          // TX con el comando serial "TX5" (solo estados 0 y 1)
#define TX_HZ_ASCENSO 2
#define TX_HZ_CAIDA 5           // estados 3, 4 y 5
#define TX_HZ_ATERRIZADO 2      // durante SM_ATERRIZADO_TX_S, luego baliza
#define LOG_HZ_TIERRA 1         // log en estados 0, 1 y 6 (ahorra flash en la espera)
#define LOG_QUEUE_LEN 64

// ---------------------------------------------------------------------------
// Calibración
// ---------------------------------------------------------------------------
#define P0_WINDOW_S 30          // promedio móvil de p0 (~30 s)
#define GYRO_CAL_SAMPLES 200    // 2 s a 100 Hz
#define GYRO_CAL_MAX_STD_DPS 0.5f  // si la desviación es mayor, se considera en movimiento
#define GYRO_CAL_MAX_TRIES 5

// ---------------------------------------------------------------------------
// Filtros
// ---------------------------------------------------------------------------
#define MAHONY_KP 2.0f
#define MAHONY_KI 0.02f
// La corrección por acelerómetro solo se aplica si | |a| - 1 g | < este valor.
#define MAHONY_ACC_GATE_G 0.15f
// Ruido de proceso (aceleración vertical, m/s^2) y de medición del barómetro (m).
#define KF_ACC_SIGMA 0.5f
#define KF_BARO_SIGMA 0.4f
#define KF_ACC_SIGMA_SIN_IMU 3.0f  // si el IMU falla (solo barómetro)

// Eje del cuerpo usado para la compuerta de ángulo (SPEC estado 3): índice 0=x,1=y,2=z
// y signo. Suposición: +z (PREGUNTAS P6).
#define BODY_AXIS_INDEX 2
#define BODY_AXIS_SIGN 1
// Reinicio en vuelo: varianza inicial de vz en el Kalman ((m/s)^2), para que converja
// rápido a la velocidad real (la placa reinicia sin saber vz).
#define KF_P_VZ_REINICIO 400.0f

// ---------------------------------------------------------------------------
// Máquina de estados (docs/SPEC.md, "Estados" y "Fallas"). Alturas sobre p0.
// ---------------------------------------------------------------------------
// 1 TIERRA -> 2 ASCENSO
#define SM_DESPEGUE_H_M 20.0f
#define SM_DESPEGUE_VZ_MS 1.0f
#define SM_DESPEGUE_T_S 3.0f
// p0 que se congela: el último promedio con el CanSat quieto en el suelo (|vz| y |h|
// menores que esto), no el del instante de la detección, que ya incluye ~6 s de subida.
#define SM_P0_QUIETO_VZ_MS 0.5f
#define SM_P0_QUIETO_H_M 3.0f
// Pausa del promedio de p0 en TIERRA: mientras el CanSat NO está quieto en el suelo (ya
// subió más de SM_P0_QUIETO_H_M o se mueve), el promedio móvil deja de recibir muestras.
// Sin la pausa, en un ascenso lento de dron el promedio "persigue" a la subida: la altura
// relativa se satura en ~15 s x v (22 m a 1.5 m/s) y vz relativa tiende a 0, y nunca se
// cumplía la condición de despegue (PREGUNTAS P57). Si la pausa dura más que esto sin
// volver a estar quieto (p. ej. lo subieron a pie a una loma), el promedio se reanuda.
#define SM_P0_PAUSA_MAX_S 300.0f
// 2 ASCENSO -> 3 CAIDA (liberación), detector del modo A (y C). La altura mínima y el
// detector de cada modo están en "Modos de despliegue" (DEPLOY_*).
#define SM_LIB_LOWG_G 0.35f          // |a| menor que esto ... (todos los modos)
#define SM_LIB_LOWG_T_S 0.15f        // ... durante este tiempo
#define SM_LIB_CONFIRMA_VZ_MS -8.0f  // confirmada por vz menor que esto ...
#define SM_LIB_CONFIRMA_T_S 1.0f     // ... dentro de este tiempo
#define SM_LIB_VZ_RAPIDA_MS -12.0f   // o vz menor que esto ...
#define SM_LIB_VZ_RAPIDA_T_S 0.4f    // ... durante este tiempo
#define SM_LIB_BAJA_DESDE_MAX_M 25.0f  // o altura esta distancia por debajo del máximo
// 3 CAIDA -> 4 DESPLIEGUE (disparo). Alturas según el modo (DEPLOY_*).
#define SM_T_LATENCIA_S 0.5f         // latencia servo + apertura (inicial, ajustar)
#define SM_COMPUERTA_DEG 45.0f       // eje del cuerpo vs vertical
#define SM_COMPUERTA_RESPALDO_S 1.5f // modo A: si la compuerta no se cumple en este tiempo, disparar
// 5 DESCENSO
#define SM_DESC_VZ_MIN_MS 8.0f       // rango de la SPEC para el descenso con autogiro
#define SM_DESC_VZ_MAX_MS 12.0f
#define SM_DESC_PROM_S 1.0f          // ventana del promedio de vz
// 5 DESCENSO -> 6 ATERRIZADO (impacto y reposo)
#define SM_IMPACTO_G 2.5f            // pico de |a| que cuenta como impacto
#define SM_IMPACTO_ARMADO_S 2.0f     // se ignora el impacto al inicio (choque de la apertura)
#define SM_REPOSO_ACC_TOL_G 0.15f    // reposo: | |a| - 1 g | menor que esto,
#define SM_REPOSO_GIRO_DPS 10.0f     //   giro menor que esto (el autogiro gira),
#define SM_REPOSO_VZ_MS 1.0f         //   |vz| menor que esto
#define SM_REPOSO_T_S 1.0f           // reposo tras impacto
#define SM_REPOSO_SIN_IMPACTO_T_S 5.0f  // reposo sin impacto detectado
// 6 ATERRIZADO
#define SM_ATERRIZADO_TX_S 60        // TX a TX_HZ_ATERRIZADO durante este tiempo
#define SM_BALIZA_PERIODO_MS 5000    // luego una baliza cada 5 s
// Reinicio en vuelo
#define SM_REINICIO_ESPERA_S 1.0f    // espera a que el Kalman converja antes de decidir
#define SM_REINICIO_VZ_CAIDA_MS -8.0f  // vz menor que esto tras reiniciar en ASCENSO -> CAIDA
#define SM_NVS_DH_MAX_M 10.0f        // guardar h_max en NVS cada vez que sube esto ...
#define SM_NVS_MIN_PERIODO_S 1.0f    // ... como máximo una vez por este periodo

// ---------------------------------------------------------------------------
// Modos de despliegue (DEPLOY_MODE). Se elige con el comando serial "MODO A|B|C|D", se
// guarda en NVS y se ve en STAT, en el arranque y en el log. Solo se puede cambiar en los
// estados 0 y 1.
//  B: LANZAMIENTO REAL (reglamento vigente, por defecto): el dron sube a ~100 m sobre el
//     suelo de despegue y suelta; autogiro y separación a 80 m. Detector de liberación
//     rápido (la caída hasta 80 m dura ~2 s). Ver PREGUNTAS P57 a P59.
//  A: reglamento anterior: liberación a ~450 m, autogiro a 200 m.
//  C: tinaco: dispara en cuanto detecta la caída libre (sin altura mínima, sin h_disp
//     ni compuerta de ángulo). Ver PREGUNTAS P33.
//  D: tinaco solo carga primaria: igual que C (misma detección y mismos eventos) pero con
//     la salida del servo inhibida: ni energía, ni disparo, ni bandera de disparo.
// En A y B: liberación válida solo si h > DEPLOY_ALT + DEPLOY_LIB_MARGEN_x_M (A = 240 m,
// B = 85 m) y h_disp = DEPLOY_ALT + |vz| * SM_T_LATENCIA_S.
// ---------------------------------------------------------------------------
#define DEPLOY_MODE_DEFAULT 'B'

// --- Modo A (450 m -> 200 m): detector y compuerta con los valores SM_* de arriba ---
#define DEPLOY_ALT_A_M 200.0f
#define DEPLOY_LIB_MARGEN_A_M 40.0f  // liberación válida > 240 m
#define DEPLOY_RESPALDO_A_M 160.0f   // por debajo de esta altura, disparar siempre
#define DEPLOY_T_SIN_BARO_A_S 7.0f   // baro muerto: disparo por tiempo desde la liberación

// --- Modo B (lanzamiento real: suelta ~100 m -> autogiro a 80 m) ---
// Suelta a 100 m: h cruza h_disp (~88 m) ~1.6 s después de soltarse. El detector del modo
// A tarda ~0.85 s (baja g 0.15 s + vz < -8 m/s); el de B ~0.5 s, y deja ~1.1 s / ~11 m de
// margen (PREGUNTAS P58).
#define DEPLOY_ALT_B_M 80.0f
// Liberación válida > 80 + 5 = 85 m. Con +10 m (90 m) una suelta a 90 m quedaría justo en
// el umbral y la mitad de las veces se descartaría por el ruido del barómetro (P58).
#define DEPLOY_LIB_MARGEN_B_M 5.0f
#define DEPLOY_RESPALDO_B_M 70.0f    // por debajo de esta altura, disparar siempre (P58)
// Baro muerto: disparo por tiempo desde la liberación detectada. Suelta a 100 m ->
// autogiro a ~83 m (más alto es más seguro: sin barómetro no se sabe la altura).
#define DEPLOY_T_SIN_BARO_B_S 1.0f
// Detector de liberación del modo B
#define DEPLOY_B_LOWG_T_S 0.10f        // |a| < SM_LIB_LOWG_G durante 100 ms ...
#define DEPLOY_B_CONFIRMA_VZ_MS -5.0f  // ... confirmada por vz < -5 m/s ...
#define DEPLOY_B_CONFIRMA_T_S 0.6f     // ... en los 0.6 s siguientes
#define DEPLOY_B_VZ_RAPIDA_MS -8.0f    // o vz < -8 m/s ...
#define DEPLOY_B_VZ_RAPIDA_T_S 0.2f    // ... durante 0.2 s
#define DEPLOY_B_BAJA_DESDE_MAX_M 8.0f // o 8 m por debajo del máximo ...
#define DEPLOY_B_BAJA_VZ_MS -5.0f      // ... bajando a más de 5 m/s (un dron no baja así)
// Compuerta de 45°: esperar como máximo 0.3 s tras cruzar h_disp. Cada 0.1 s de espera
// baja el despliegue ~2 m (a ~15-17 m/s): con 1.5 s (modo A) se llegaría al respaldo de
// 70 m; con 0.3 s el peor caso (cae inclinado) despliega a ~74 m (PREGUNTAS P58).
#define DEPLOY_B_COMPUERTA_RESPALDO_S 0.3f
// Liberación detectada con h ya por debajo de h_disp: disparo inmediato, sin esperar la
// compuerta (en B siempre; en A no, como hasta ahora).
#define DEPLOY_B_DISPARO_TARDIO_INMEDIATO 1

// ---------------------------------------------------------------------------
// Servo de liberación del autogiro (MG90S). Ver servo_release.h.
// ---------------------------------------------------------------------------
#define SERVO_ANG_BLOQUEO_DEG -10.0f // posición de espera (autogiro retenido)
#define SERVO_ANG_LIBERA_DEG -80.0f  // posición de liberación
#define SERVO_REINTENTOS 2           // reintentos si no hay confirmación
#define SERVO_CONFIRMA_G 2.0f        // confirmación: pico de |a| (por validar, PREGUNTAS)
#define SERVO_CONFIRMA_T_S 1.0f      // ventana de confirmación por intento
#define SERVO_T_RETORNO_S 0.3f       // en cada reintento: tiempo en -10° antes de volver a -80°
// Pulso: SERVO_US_CENTRO en 0° y SERVO_US_POR_GRADO (suposición: 500–2500 us = ±90°).
#define SERVO_US_CENTRO 1500.0f
#define SERVO_US_POR_GRADO (1000.0f / 90.0f)
#define SERVO_PWM_HZ 50
#define SERVO_PWM_BITS 14
#define SERVO_LEDC_CANAL 0

// ---------------------------------------------------------------------------
// Modo DEMO_SEPARACION (entornos heltec_demo y heltec_banco): el servo se dispara
// DEMO_DELAY_S segundos después de armar (comando serial ARMAR o botón PRG presionado
// DEMO_BOTON_MS). Un segundo toque largo antes de terminar cancela.
// ---------------------------------------------------------------------------
#ifndef DEMO_DELAY_S
#define DEMO_DELAY_S 10
#endif
#define DEMO_BOTON_MS 1000           // toque largo del botón PRG
#define DEMO_LED_PARPADEO_MS 100     // LED: medio periodo del parpadeo durante la cuenta

// ---------------------------------------------------------------------------
// Modo simulación por serial (docs/sim_input.md)
// ---------------------------------------------------------------------------
#define SIM_QUEUE_LEN 32
#define SIM_LINEA_MAX 160
// Búfer de recepción del puerto serie (UART0 por el CP2102, 115200 baudios). 8 KB = ~1.2 s
// de líneas SIM (~6.9 kB/s): cubre las pausas de la tarea comms (LittleFS, radio). Los
// bytes que llegan mientras la flash tiene la caché apagada dependen de la FIFO de 128 B
// y pueden perderse: eso lo resuelven el CRC y el reenvío (docs/sim_input.md).
#define SERIAL_RX_BUF 8192
