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
    (PIN_BAT_ADC == TODO_CONFIRMAR)
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
#define TX_HZ_PRUEBA 5          // TX con el comando serial "TX5"
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
// y signo. Suposición: +z (PREGUNTAS P6). La compuerta aún no se usa.
#define BODY_AXIS_INDEX 2
#define BODY_AXIS_SIGN 1
