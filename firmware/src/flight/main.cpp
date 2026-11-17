// Firmware de vuelo CanSat Hellmary (etapa 1: sensores, filtros, telemetría y log).
// AÚN NO hay máquina de estados de vuelo: el estado reportado es AUTOPRUEBA durante la
// calibración y TIERRA después. El servo permanece sin energía.
//
// Tareas:
//  - control (núcleo CONTROL_CORE, prioridad alta, 100 Hz): sensores, Mahony, Kalman,
//    armado de paquetes. Nunca espera al radio ni al log: usa colas sin bloqueo.
//  - comms (núcleo COMMS_CORE): radio LoRa, log en LittleFS y comandos por serial.
//
// Comandos por serial (115200, terminados en salto de línea):
//   TX1     telemetría a 1 Hz (por defecto, modo Tierra)
//   TX5     telemetría a 5 Hz (pruebas)
//   STAT    estadísticas
//   DUMP    vuelca el log en hex
//   BORRAR  borra el log
#include <Arduino.h>

#include "attitude.h"
#include "config.h"
#include "estimator.h"
#include "logger.h"
#include "packet.h"
#include "power.h"
#include "sensors.h"
#include "telemetry.h"
#include "telemetry_pack.h"

static Sensors sensors;
static Mahony ahrs(MAHONY_KP, MAHONY_KI, MAHONY_ACC_GATE_G);
static GyroBiasCal gyro_cal(GYRO_CAL_SAMPLES, GYRO_CAL_MAX_STD_DPS);
static VerticalKF kf(KF_ACC_SIGMA, KF_BARO_SIGMA);
static BaseCal base(P0_WINDOW_S, (float)CONTROL_HZ / BARO_DECIM);

static QueueHandle_t tx_queue;   // longitud 1, se sobrescribe (siempre el más reciente)
static QueueHandle_t log_queue;  // registros para LittleFS

static volatile uint8_t tx_hz = TX_HZ_TIERRA;

// Estadísticas (escritas por control, leídas por comms solo para diagnóstico)
static volatile uint32_t st_cycles = 0, st_overruns = 0, st_max_us = 0;
static volatile uint32_t st_log_drops = 0, st_imu_err = 0, st_baro_err = 0;

// ---------------------------------------------------------------------------
// Tarea de control
// ---------------------------------------------------------------------------
static void controlTask(void *) {
  const float dt = 1.0f / CONTROL_HZ;
  const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
  TickType_t last_wake = xTaskGetTickCount();

  ImuSample imu{};
  BaroSample baro{};
  bool imu_dead = !sensors.imuOk();
  bool baro_dead = !sensors.baroOk();
  bool gyro_ready = imu_dead;  // sin IMU no hay nada que calibrar
  int gyro_tries = 0;
  float gyro_bias[3] = {0, 0, 0};
  bool kf_init = false;
  float last_p = -1;
  uint32_t last_p_change_ms = millis();
  float acc_now[3] = {0, 0, 0};
  float bat_v = power::batteryVolts();
  uint16_t seq = 0;
  int imu_fails = 0, baro_fails = 0;
  uint32_t cycle = 0;

  for (;;) {
    vTaskDelayUntil(&last_wake, period);
    const uint32_t t0 = micros();
    cycle++;

    // --- IMU ---
    if (!imu_dead) {
      if (sensors.readImu(imu)) {
        for (int i = 0; i < 3; i++) acc_now[i] = imu.acc_g[i];
        if (!gyro_ready) {
          // Sesgo del giroscopio en reposo al arrancar
          if (gyro_cal.add(imu.gyro_dps[0], imu.gyro_dps[1], imu.gyro_dps[2])) {
            if (gyro_cal.ok() || ++gyro_tries >= GYRO_CAL_MAX_TRIES) {
              // Tras GYRO_CAL_MAX_TRIES intentos con movimiento se usa la última media
              // (PREGUNTAS P9).
              for (int i = 0; i < 3; i++) gyro_bias[i] = gyro_cal.bias[i];
              ahrs.initFromAccel(imu.acc_g[0], imu.acc_g[1], imu.acc_g[2]);
              gyro_ready = true;
            } else {
              gyro_cal.reset();
            }
          }
        } else {
          ahrs.update(imu.gyro_dps[0] - gyro_bias[0], imu.gyro_dps[1] - gyro_bias[1],
                      imu.gyro_dps[2] - gyro_bias[2], imu.acc_g[0], imu.acc_g[1],
                      imu.acc_g[2], dt);
        }
        imu_fails = 0;
      } else {
        st_imu_err++;
        // Más de 1 s sin lecturas válidas: IMU muerto
        if (++imu_fails > CONTROL_HZ) imu_dead = true;
      }
    }

    // --- Kalman: predicción con la aceleración vertical ---
    float a_up = 0.0f;
    if (!imu_dead && gyro_ready) {
      a_up = (ahrs.verticalSpecificForce(acc_now[0], acc_now[1], acc_now[2]) - 1.0f) * kG;
    } else if (imu_dead) {
      // Sin IMU: modelo de velocidad casi constante, con más ruido de proceso.
      kf.setAccSigma(KF_ACC_SIGMA_SIN_IMU);
      for (float &a : acc_now) a = 0.0f;  // no reportar la última lectura como vigente
    }
    if (kf_init) kf.predict(a_up, dt);

    // --- Barómetro (25 Hz) ---
    if (!baro_dead && cycle % BARO_DECIM == 0) {
      if (sensors.readBaro(baro)) {
        // Congelado: valor idéntico durante más de 1 s (SPEC, Fallas)
        if (baro.p_pa != last_p) {
          last_p = baro.p_pa;
          last_p_change_ms = millis();
        } else if (millis() - last_p_change_ms > 1000) {
          baro_dead = true;
        }
        if (!baro_dead) {
          base.add(baro.p_pa);
          const float h_baro = baroAltitude(baro.p_pa, base.p0());
          if (!kf_init) {
            kf.reset(h_baro, 0.0f);
            kf_init = true;
          } else {
            kf.updateBaro(h_baro);
          }
        }
        baro_fails = 0;
      } else {
        st_baro_err++;
        if (++baro_fails > CONTROL_HZ / BARO_DECIM) baro_dead = true;  // >1 s
      }
    }
    if (baro_dead && !kf_init) {
      kf.reset(0.0f, 0.0f);  // solo IMU
      kf_init = true;
    }

    if (cycle % CONTROL_HZ == 0) bat_v = power::batteryVolts();

    // --- Paquete ---
    const bool calibrando = !gyro_ready || (!baro_dead && !base.hasValue());
    uint8_t flags = 0;
    if (base.ready()) flags |= FLAG_BASE_LISTA;
    if (baro_dead) flags |= FLAG_FALLO_BARO;
    if (imu_dead) flags |= FLAG_FALLO_IMU;
    if (baro_dead && imu_dead) flags |= FLAG_NO_GO;

    const uint32_t log_div = CONTROL_HZ / LOG_HZ;
    const uint8_t hz = tx_hz;
    const uint32_t tx_div = CONTROL_HZ / (hz ? hz : 1);
    const bool do_log = (cycle % log_div) == 0;
    const bool do_tx = (cycle % tx_div) == 0;
    if (do_log || do_tx) {
      TelemetryValues v;
      v.estado = calibrando ? ST_AUTOPRUEBA : ST_TIERRA;
      v.secuencia = seq;
      v.t_ms = millis();
      v.presion_pa = baro.p_pa;
      v.temp_c = baro.temp_c;
      v.altura_m = kf.h();
      v.vz_ms = kf.vz();
      for (int i = 0; i < 3; i++) v.acc_g[i] = acc_now[i];
      v.roll_deg = ahrs.roll();
      v.pitch_deg = ahrs.pitch();
      v.yaw_deg = ahrs.yaw();
      v.banderas = flags;
      v.bateria_v = bat_v;
      const TelemetryPacket pkt = packTelemetry(v);
      if (do_tx) {
        xQueueOverwrite(tx_queue, &pkt);
        seq++;
      }
      if (do_log && xQueueSend(log_queue, &pkt, 0) != pdTRUE) st_log_drops++;
    }

    const uint32_t us = micros() - t0;
    if (us > st_max_us) st_max_us = us;
    if (us > 1000000u / CONTROL_HZ) st_overruns++;
    st_cycles = cycle;
  }
}

// ---------------------------------------------------------------------------
// Tarea de comunicaciones: radio, log y serial
// ---------------------------------------------------------------------------
static void handleCommand(const String &cmd) {
  if (cmd == "TX1") {
    tx_hz = TX_HZ_TIERRA;
    Serial.println("OK TX 1 Hz");
  } else if (cmd == "TX5") {
    tx_hz = TX_HZ_PRUEBA;
    Serial.println("OK TX 5 Hz");
  } else if (cmd == "STAT") {
    Serial.printf("ciclos=%lu sobretiempos=%lu max_us=%lu imu_err=%lu baro_err=%lu\n",
                  (unsigned long)st_cycles, (unsigned long)st_overruns,
                  (unsigned long)st_max_us, (unsigned long)st_imu_err,
                  (unsigned long)st_baro_err);
    Serial.printf("tx_hz=%u tx_ok=%lu tx_omitidos=%lu tx_err=%lu\n", tx_hz,
                  (unsigned long)telemetry::sent(), (unsigned long)telemetry::skipped(),
                  (unsigned long)telemetry::errors());
    Serial.printf("log=%u/%u B descartados=%lu\n", (unsigned)logger::sizeBytes(),
                  (unsigned)logger::capacityBytes(), (unsigned long)st_log_drops);
  } else if (cmd == "DUMP") {
    logger::dump(Serial);
  } else if (cmd == "BORRAR") {
    logger::erase();
    Serial.println("OK log borrado");
  } else if (cmd.length()) {
    Serial.println("Comandos: TX1 TX5 STAT DUMP BORRAR");
  }
}

static void commsTask(void *) {
  String line;
  uint32_t last_flush = millis();
  TelemetryPacket pkt;
  for (;;) {
    // Espera corta por registros del log; despierta al menos cada 5 ms.
    if (xQueueReceive(log_queue, &pkt, pdMS_TO_TICKS(5)) == pdTRUE) {
      logger::append(pkt);
      while (xQueueReceive(log_queue, &pkt, 0) == pdTRUE) logger::append(pkt);
    }
    if (xQueueReceive(tx_queue, &pkt, 0) == pdTRUE) telemetry::send(pkt);
    telemetry::poll();

    if (millis() - last_flush >= 1000) {
      logger::flush();
      last_flush = millis();
    }

    while (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        line.trim();
        line.toUpperCase();
        handleCommand(line);
        line = "";
      } else if (line.length() < 32) {
        line += c;
      }
    }
  }
}

void setup() {
  power::begin();  // primero: servo sin energía
  Serial.begin(115200);
  delay(200);
  Serial.println("CanSat Hellmary - firmware de vuelo (etapa 1)");

  sensors.begin();
  Serial.printf("IMU %s (WHO_AM_I=0x%02X)  BARO %s\n", sensors.imuOk() ? "OK" : "FALLA",
                sensors.imuWhoAmI(), sensors.baroOk() ? "OK" : "FALLA");
  Serial.printf("LittleFS %s\n", logger::begin() ? "OK" : "FALLA");
  const int16_t rs = telemetry::begin();
  Serial.printf("Radio %s (codigo %d)\n", rs == 0 ? "OK" : "FALLA", rs);

  tx_queue = xQueueCreate(1, sizeof(TelemetryPacket));
  log_queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(TelemetryPacket));

  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, configMAX_PRIORITIES - 2,
                          nullptr, CONTROL_CORE);
  xTaskCreatePinnedToCore(commsTask, "comms", 8192, nullptr, 2, nullptr, COMMS_CORE);
}

void loop() {
  // Todo el trabajo está en las tareas; loop() no se usa.
  vTaskDelete(nullptr);
}
