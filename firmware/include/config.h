// Configuración del firmware CanSat Hellmary. Ver docs/SPEC.md y docs/PREGUNTAS.md.
//
// Pines externos: NO inventar. Quedan como TODO_CONFIRMAR hasta que el equipo de
// electrónica los confirme. Los pines del radio SX1262 NO están aquí: salen de la
// variante de la placa (pins_arduino.h de heltec_wifi_lora_32_V3).
#pragma once

// Valor centinela para pines no confirmados.
#define TODO_CONFIRMAR (-1)

// ---------------------------------------------------------------------------
// Pines externos (pendientes)
// ---------------------------------------------------------------------------
#define PIN_I2C_SDA TODO_CONFIRMAR      // SDA del bus de BMP280 + MPU6500
#define PIN_I2C_SCL TODO_CONFIRMAR      // SCL del bus de BMP280 + MPU6500
#define PIN_SERVO_SIGNAL TODO_CONFIRMAR // señal PWM del MG90S (con pull-down externo)
#define PIN_SERVO_POWER TODO_CONFIRMAR  // habilitación de energía del servo
#define PIN_REED TODO_CONFIRMAR         // lectura del reed
#define PIN_LATCH TODO_CONFIRMAR        // enclavamiento de encendido (GPIO)
#define PIN_BAT_ADC TODO_CONFIRMAR      // ADC de la pila de 9 V (con divisor)

// Nivel lógico de PIN_SERVO_POWER que ENCIENDE el servo (HIGH=1 o LOW=0). Pendiente
// (PREGUNTAS P2b): depende del transistor/MOSFET que diseñe electrónica.
#define SERVO_POWER_ON_LEVEL TODO_CONFIRMAR
// Nivel de PIN_REED con el imán presente (solo lo usa el modo DEMO_SEPARACION para
// armar). Suposición: LOW, con pull-up interno (PREGUNTAS P2).
#define REED_ACTIVE_LEVEL 0
#define REED_PULLUP 1

// Relación del divisor de la batería (V_pila / V_adc). Pendiente: depende del divisor
// que diseñe electrónica. 0 = desconocido -> no se reporta batería (bateria = 0).
#define BAT_DIVIDER_RATIO 0.0f

// La verificación se activa SOLO en el entorno de vuelo (heltec_wifi_lora_32_V3),
// que define CANSAT_FLIGHT_BOARD. Nunca en native ni en rx_station.
// Para compilar a pesar de los pines pendientes (solo pruebas de compilación):
//   -D PERMITIR_PINES_PENDIENTES
#if defined(CANSAT_FLIGHT_BOARD) && !defined(PERMITIR_PINES_PENDIENTES)
#if (PIN_I2C_SDA == TODO_CONFIRMAR) || (PIN_I2C_SCL == TODO_CONFIRMAR) ||           \
    (PIN_SERVO_SIGNAL == TODO_CONFIRMAR) || (PIN_SERVO_POWER == TODO_CONFIRMAR) || \
    (PIN_REED == TODO_CONFIRMAR) || (PIN_LATCH == TODO_CONFIRMAR) ||               \
    (PIN_BAT_ADC == TODO_CONFIRMAR) || (SERVO_POWER_ON_LEVEL == TODO_CONFIRMAR)
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
// 2 ASCENSO -> 3 CAIDA (liberación)
#define SM_LIB_H_MIN_M 240.0f        // liberación válida solo por encima de esta altura
#define SM_LIB_LOWG_G 0.35f          // |a| menor que esto ...
#define SM_LIB_LOWG_T_S 0.15f        // ... durante este tiempo
#define SM_LIB_CONFIRMA_VZ_MS -8.0f  // confirmada por vz menor que esto ...
#define SM_LIB_CONFIRMA_T_S 1.0f     // ... dentro de este tiempo
#define SM_LIB_VZ_RAPIDA_MS -12.0f   // o vz menor que esto ...
#define SM_LIB_VZ_RAPIDA_T_S 0.4f    // ... durante este tiempo
#define SM_LIB_BAJA_DESDE_MAX_M 25.0f  // o altura esta distancia por debajo del máximo
// 3 CAIDA -> 4 DESPLIEGUE (disparo)
#define SM_H_DISPARO_M 200.0f        // h_disp = SM_H_DISPARO_M + |vz| * SM_T_LATENCIA_S
#define SM_T_LATENCIA_S 0.5f         // latencia servo + apertura (inicial, ajustar)
#define SM_COMPUERTA_DEG 45.0f       // eje del cuerpo vs vertical
#define SM_COMPUERTA_RESPALDO_S 1.5f // si la compuerta no se cumple en este tiempo, disparar
#define SM_H_RESPALDO_M 160.0f       // por debajo de esta altura, disparar siempre
#define SM_T_DISPARO_SIN_BARO_S 7.0f // baro muerto: disparo por tiempo desde la liberación
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
// Modo DEMO_SEPARACION (entorno heltec_demo): el servo se dispara DEMO_DELAY_S
// segundos después de armar (comando serial ARMAR o reed).
// ---------------------------------------------------------------------------
#ifndef DEMO_DELAY_S
#define DEMO_DELAY_S 10
#endif
#define REED_ANTIRREBOTE_MS 50

// ---------------------------------------------------------------------------
// Modo simulación por serial (docs/sim_input.md)
// ---------------------------------------------------------------------------
#define SIM_QUEUE_LEN 32
#define SIM_LINEA_MAX 160
