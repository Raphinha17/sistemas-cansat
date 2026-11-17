// Firmware de vuelo CanSat Hellmary: sensores, filtros, máquina de estados (0 a 6),
// servo de liberación, telemetría LoRa, log y NVS. Ver docs/SPEC.md.
//
// La lógica del lazo de control (calibración, Mahony, Kalman, fallas, máquina de
// estados) está en lib/flight_core y es la misma que se prueba en native.
//
// Tareas:
//  - control (núcleo CONTROL_CORE, prioridad alta, 100 Hz): sensores -> FlightCore ->
//    servo y paquetes. Nunca espera al radio, al log ni a la NVS: usa colas sin bloqueo.
//  - comms (núcleo COMMS_CORE): radio LoRa, log en LittleFS, NVS y comandos por serial.
//
// Modos:
//  - Normal (vuelo).
//  - Simulación por serial (docs/sim_input.md): al recibir una línea "SIM,..." en los
//    estados 0 o 1, el lazo usa esos valores en lugar de los sensores (prueba B5).
//  - DEMO_SEPARACION (entorno heltec_demo): ARMAR o el reed disparan el servo
//    DEMO_DELAY_S segundos después. El modo de vuelo no cambia.
//
// Comandos por serial (115200, terminados en salto de línea):
//   TX1           telemetría según el estado (por defecto)
//   TX5           telemetría a 5 Hz en los estados 0 y 1 (pruebas)
//   STAT          estadísticas y estado
//   DUMP          vuelca el log en hex
//   BORRAR        borra el log
//   MISION        muestra el registro de misión guardado en NVS
//   MISION_BORRAR borra el registro de misión y reinicia la placa
//   SIM_FIN       sale del modo simulación (vuelve a los sensores y recalibra)
//   ARMAR         (solo DEMO_SEPARACION) arma la cuenta regresiva del servo
//   DESARMAR      (solo DEMO_SEPARACION) cancela la cuenta regresiva
#include <Arduino.h>

#include "config.h"
#include "flight_core.h"
#include "logger.h"
#include "mission_store.h"
#include "packet.h"
#include "power.h"
#include "sensors.h"
#include "servo_hw.h"
#include "sim_input.h"
#include "telemetry.h"
#include "telemetry_pack.h"

static Sensors sensors;
static FlightCore core;

static QueueHandle_t tx_queue;   // longitud 1, se sobrescribe (siempre el más reciente)
static QueueHandle_t log_queue;  // registros para LittleFS
static QueueHandle_t nvs_queue;  // longitud 1: último registro de misión a guardar
static QueueHandle_t sim_queue;  // entradas del modo simulación

static volatile bool tx_test = false;  // comando TX5

// Peticiones de comms a control (las atiende control al inicio del ciclo)
static volatile bool req_sim_on = false, req_sim_off = false;
static volatile bool req_demo_arm = false, req_demo_disarm = false;
static volatile bool sim_mode = false;
static volatile bool req_flush = false;

// Publicado por control para comms (diagnóstico y avisos por serial)
static volatile uint8_t pub_state = ST_AUTOPRUEBA, pub_flags = 0;
static volatile uint32_t pub_fire_ms = 0, pub_fire_count = 0;
static volatile float pub_h = 0, pub_vz = 0, pub_hmax = 0, pub_tilt = 0, pub_demo_rest = 0;
static volatile uint8_t pub_servo_int = 0;
static volatile bool pub_servo_conf = false, pub_servo_on = false, pub_vz_fuera = false;

// Estadísticas (escritas por control, leídas por comms solo para diagnóstico)
static volatile uint32_t st_cycles = 0, st_overruns = 0, st_max_us = 0;
static volatile uint32_t st_log_drops = 0, st_imu_err = 0, st_baro_err = 0;
static volatile uint32_t st_sim_rx = 0, st_sim_drops = 0, st_sim_bad = 0;

static bool boot_recovered = false;
static MissionRecord boot_rec;

// ---------------------------------------------------------------------------
// Tarea de control
// ---------------------------------------------------------------------------
static void controlTask(void *) {
  const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
  TickType_t last_wake = xTaskGetTickCount();

  ImuSample imu{};
  BaroSample baro{};
  float bat_v = power::batteryVolts();
  uint16_t seq = 0;
  uint32_t cycle = 0, last_tx = 0, last_log = 0;
  uint32_t rec_seq = core.recordSeq();
  bool sim_fresh = false;
#if defined(DEMO_SEPARACION) && (PIN_REED >= 0)
  bool reed_stable = digitalRead(PIN_REED) == REED_ACTIVE_LEVEL;  // sin flanco al arrancar
  bool reed_last = reed_stable;
  uint32_t reed_change_ms = millis();
#endif

  for (;;) {
    // --- Cambios de modo pedidos por serial ---
    if (req_sim_on) {
      req_sim_on = false;
      sim_mode = true;
      sim_fresh = true;
    }
    if (req_sim_off) {
      req_sim_off = false;
      if (sim_mode) {
        sim_mode = false;
        xQueueReset(sim_queue);
        core.begin(millis(), sensors.imuOk(), sensors.baroOk());  // recalibra con sensores
        cycle = last_tx = last_log = 0;
        rec_seq = core.recordSeq();
        last_wake = xTaskGetTickCount();
      }
    }

    // --- Entrada del ciclo: sensores o línea de simulación ---
    CoreInput in;
    uint32_t t0;
    if (sim_mode) {
      // Un ciclo por línea recibida (el reloj es el t_ms de la línea).
      if (xQueueReceive(sim_queue, &in, pdMS_TO_TICKS(100)) != pdTRUE) continue;
      t0 = micros();
      if (sim_fresh) {
        // Arranque limpio con los datos sintéticos: autoprueba, calibración y p0 nuevos.
        // No se escribe NVS en este modo (ver abajo).
        sim_fresh = false;
        core.begin(in.t_ms, true, true);
        cycle = last_tx = last_log = 0;
      }
      in.baro_sampled = (cycle % BARO_DECIM) == 0;
    } else {
      vTaskDelayUntil(&last_wake, period);
      t0 = micros();
      in.t_ms = millis();
      if (!core.imuDead()) {
        in.imu_valid = sensors.readImu(imu);
        if (in.imu_valid) {
          for (int i = 0; i < 3; i++) {
            in.acc_g[i] = imu.acc_g[i];
            in.gyro_dps[i] = imu.gyro_dps[i];
          }
        } else {
          st_imu_err++;
        }
      }
      in.baro_sampled = !core.baroDead() && (cycle % BARO_DECIM) == 0;
      if (in.baro_sampled) {
        in.baro_valid = sensors.readBaro(baro);
        if (in.baro_valid) {
          in.p_pa = baro.p_pa;
          in.temp_c = baro.temp_c;
        } else {
          st_baro_err++;
        }
      }
    }

    // --- Demo: armado por comando o por el reed (flanco hacia "imán presente") ---
    if (req_demo_arm) {
      req_demo_arm = false;
      core.demoArm(in.t_ms);
    }
    if (req_demo_disarm) {
      req_demo_disarm = false;
      core.demoDisarm();
    }
#if defined(DEMO_SEPARACION) && (PIN_REED >= 0)
    {
      const bool r = digitalRead(PIN_REED) == REED_ACTIVE_LEVEL;
      if (r != reed_last) {
        reed_last = r;
        reed_change_ms = millis();
      } else if (r != reed_stable && millis() - reed_change_ms >= REED_ANTIRREBOTE_MS) {
        reed_stable = r;
        if (r) core.demoArm(in.t_ms);
      }
    }
#endif

    // --- Núcleo: filtros + máquina de estados ---
    core.step(in);
    cycle++;
    servo_hw::apply(core.servo());

    if (core.evFlushLog()) req_flush = true;
    if (core.recordSeq() != rec_seq) {
      rec_seq = core.recordSeq();
      // En modo simulación no se toca la NVS: un ensayo en banco no debe dejar una
      // misión "en curso" para el siguiente arranque.
      if (!sim_mode) {
        const MissionRecord r = core.record();
        xQueueOverwrite(nvs_queue, &r);
      }
    }
    if (core.evFire()) {
      pub_fire_ms = in.t_ms;
      pub_fire_count = pub_fire_count + 1;
    }

    if (!sim_mode && cycle % CONTROL_HZ == 0) bat_v = power::batteryVolts();

    // --- Paquete: TX y log según el estado ---
    uint32_t tx_ms = core.txPeriodMs();
    if (tx_test && core.state() <= ST_TIERRA && tx_ms > 1000u / TX_HZ_PRUEBA)
      tx_ms = 1000u / TX_HZ_PRUEBA;
    const uint32_t tx_div = tx_ms * CONTROL_HZ / 1000 ? tx_ms * CONTROL_HZ / 1000 : 1;
    const uint32_t log_hz = core.logHz();
    const uint32_t log_div = CONTROL_HZ / (log_hz ? log_hz : 1);
    const bool do_tx = cycle - last_tx >= tx_div;
    // El ciclo del disparo y de cada cambio de estado siempre queda en el log, para
    // registrar el instante exacto (p. ej. la separación del modo demo).
    const bool do_log = cycle - last_log >= log_div || core.evFire() || core.evStateChange();
    if (do_tx || do_log) {
      TelemetryValues v = core.telemetry();
      v.secuencia = seq;
      v.bateria_v = sim_mode ? 0.0f : bat_v;
      const TelemetryPacket pkt = packTelemetry(v);
      if (do_tx) {
        xQueueOverwrite(tx_queue, &pkt);
        seq++;
        last_tx = cycle;
      }
      if (do_log) {
        last_log = cycle;
        if (xQueueSend(log_queue, &pkt, 0) != pdTRUE) st_log_drops++;
      }
    }

    // --- Publicación para diagnóstico ---
    const SmOutput &so = core.sm().out();
    pub_state = core.state();
    pub_flags = core.flags();
    pub_h = core.kf().h();
    pub_vz = core.kf().vz();
    pub_hmax = so.h_max;
    pub_tilt = core.tiltDeg();
    pub_servo_on = servo_hw::powered();
    pub_servo_int = so.servo_intentos;
    pub_servo_conf = so.servo_confirmado;
    pub_vz_fuera = so.vz_fuera_rango;
    pub_demo_rest = core.demo().remainingS(in.t_ms);

    const uint32_t us = micros() - t0;
    if (us > st_max_us) st_max_us = us;
    if (us > 1000000u / CONTROL_HZ) st_overruns++;
    st_cycles = cycle;
  }
}

// ---------------------------------------------------------------------------
// Tarea de comunicaciones: radio, log, NVS y serial
// ---------------------------------------------------------------------------
static void printRecord(const MissionRecord &r) {
  Serial.printf("mision=%u estado=%u banderas=0x%02X p0=%.1f Pa (valido=%u) h_max=%.1f m "
                "sesgo_giro=%.3f,%.3f,%.3f\n",
                r.mission, r.state, r.flags, r.p0_pa, r.p0_valid, r.h_max_m, r.gyro_bias[0],
                r.gyro_bias[1], r.gyro_bias[2]);
}

static void handleSim(const char *line) {
  CoreInput in;
  if (!parseSimLine(line, in)) {
    st_sim_bad++;
    return;
  }
  if (!sim_mode && !req_sim_on) {
    // Solo en tierra: nunca se entra en este modo con la misión en curso.
    if (pub_state > ST_TIERRA) {
      st_sim_bad++;
      return;
    }
    req_sim_on = true;
    Serial.println("# SIM: modo simulacion activado (sensores ignorados, sin NVS)");
  }
  st_sim_rx++;
  if (xQueueSend(sim_queue, &in, 0) != pdTRUE) st_sim_drops++;
}

static void handleCommand(const String &cmd) {
  if (cmd == "TX1") {
    tx_test = false;
    Serial.println("OK TX segun el estado");
  } else if (cmd == "TX5") {
    tx_test = true;
    Serial.println("OK TX 5 Hz en estados 0 y 1");
  } else if (cmd == "STAT") {
    Serial.printf("estado=%u banderas=0x%02X h=%.1f vz=%.1f h_max=%.1f incl=%.0f modo=%s%s\n",
                  pub_state, pub_flags, pub_h, pub_vz, pub_hmax, pub_tilt,
                  sim_mode ? "SIM" : "sensores", core.demoEnabled() ? " DEMO" : "");
    Serial.printf("servo=%s intentos=%u confirmado=%u disparos=%lu ultimo_disparo_ms=%lu "
                  "vz_fuera_rango=%u\n",
                  pub_servo_on ? "ON" : "OFF", pub_servo_int, pub_servo_conf,
                  (unsigned long)pub_fire_count, (unsigned long)pub_fire_ms, pub_vz_fuera);
    Serial.printf("ciclos=%lu sobretiempos=%lu max_us=%lu imu_err=%lu baro_err=%lu\n",
                  (unsigned long)st_cycles, (unsigned long)st_overruns,
                  (unsigned long)st_max_us, (unsigned long)st_imu_err,
                  (unsigned long)st_baro_err);
    Serial.printf("tx_ok=%lu tx_omitidos=%lu tx_err=%lu\n", (unsigned long)telemetry::sent(),
                  (unsigned long)telemetry::skipped(), (unsigned long)telemetry::errors());
    Serial.printf("log=%u/%u B descartados=%lu nvs_escrituras=%lu\n",
                  (unsigned)logger::sizeBytes(), (unsigned)logger::capacityBytes(),
                  (unsigned long)st_log_drops, (unsigned long)mission_store::writes());
    Serial.printf("sim_rx=%lu sim_descartadas=%lu sim_invalidas=%lu\n",
                  (unsigned long)st_sim_rx, (unsigned long)st_sim_drops,
                  (unsigned long)st_sim_bad);
  } else if (cmd == "DUMP") {
    logger::dump(Serial);
  } else if (cmd == "BORRAR") {
    logger::erase();
    Serial.println("OK log borrado");
  } else if (cmd == "MISION") {
    MissionRecord r;
    if (mission_store::load(r))
      printRecord(r);
    else
      Serial.println("sin registro de mision");
  } else if (cmd == "MISION_BORRAR") {
    mission_store::clear();
    Serial.println("OK mision borrada; reiniciando");
    Serial.flush();
    delay(100);
    ESP.restart();
  } else if (cmd == "SIM_FIN") {
    req_sim_off = true;
    Serial.println("OK fin del modo simulacion (recalibrando con sensores)");
  } else if (cmd == "ARMAR" || cmd == "DESARMAR") {
    if (!core.demoEnabled()) {
      Serial.println("ERR solo en el modo DEMO_SEPARACION (entorno heltec_demo)");
    } else if (cmd == "ARMAR") {
      req_demo_arm = true;
      Serial.printf("OK armado: disparo en %d s\n", DEMO_DELAY_S);
    } else {
      req_demo_disarm = true;
      Serial.println("OK desarmado");
    }
  } else if (cmd.length()) {
    Serial.println("Comandos: TX1 TX5 STAT DUMP BORRAR MISION MISION_BORRAR SIM_FIN"
                   " ARMAR DESARMAR | lineas SIM,... (docs/sim_input.md)");
  }
}

static void commsTask(void *) {
  char line[SIM_LINEA_MAX + 1];
  size_t len = 0;
  bool overflow = false;
  uint32_t last_flush = millis();
  uint8_t last_state = 0xFF;
  uint32_t last_fire_count = 0;
  TelemetryPacket pkt;
  MissionRecord rec;
  for (;;) {
    // Espera corta por registros del log; despierta al menos cada 5 ms.
    if (xQueueReceive(log_queue, &pkt, pdMS_TO_TICKS(5)) == pdTRUE) {
      logger::append(pkt);
      while (xQueueReceive(log_queue, &pkt, 0) == pdTRUE) logger::append(pkt);
    }
    if (xQueueReceive(tx_queue, &pkt, 0) == pdTRUE) telemetry::send(pkt);
    telemetry::poll();

    if (xQueueReceive(nvs_queue, &rec, 0) == pdTRUE) mission_store::save(rec);

    if (req_flush || millis() - last_flush >= 1000) {
      req_flush = false;
      logger::flush();
      last_flush = millis();
    }

    // Avisos por serial
    if (pub_state != last_state) {
      last_state = pub_state;
      Serial.printf("# ESTADO %u banderas=0x%02X\n", last_state, pub_flags);
    }
    if (pub_fire_count != last_fire_count) {
      last_fire_count = pub_fire_count;
      Serial.printf("# %s t_ms=%lu\n", core.demoEnabled() ? "SEPARACION" : "DISPARO",
                    (unsigned long)pub_fire_ms);
    }

    while (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        if (len && !overflow) {
          line[len] = 0;
          if (strncmp(line, "SIM,", 4) == 0) {
            handleSim(line);
          } else {
            String cmd(line);
            cmd.trim();
            cmd.toUpperCase();
            handleCommand(cmd);
          }
        }
        len = 0;
        overflow = false;
      } else if (len < SIM_LINEA_MAX) {
        line[len++] = c;
      } else {
        overflow = true;  // línea demasiado larga: se descarta completa
      }
    }
  }
}

void setup() {
  power::begin();  // primero: servo sin energía y señal con pull-down
  Serial.begin(115200);
  delay(200);
#ifdef DEMO_SEPARACION
  Serial.printf("CanSat Hellmary - MODO DEMO_SEPARACION (disparo %d s tras ARMAR o reed)\n",
                DEMO_DELAY_S);
#else
  Serial.println("CanSat Hellmary - firmware de vuelo");
#endif

  sensors.begin();
  Serial.printf("IMU %s (WHO_AM_I=0x%02X)  BARO %s\n", sensors.imuOk() ? "OK" : "FALLA",
                sensors.imuWhoAmI(), sensors.baroOk() ? "OK" : "FALLA");
  Serial.printf("LittleFS %s\n", logger::begin() ? "OK" : "FALLA");
  const int16_t rs = telemetry::begin();
  Serial.printf("Radio %s (codigo %d)\n", rs == 0 ? "OK" : "FALLA", rs);
  Serial.printf("NVS %s\n", mission_store::begin() ? "OK" : "FALLA");

  // Reinicio en vuelo: si hay una misión en curso en NVS, se recupera sin recalibrar.
  boot_recovered = mission_store::load(boot_rec) && boot_rec.mission;
  if (boot_recovered) {
    Serial.print("# REINICIO EN VUELO: ");
    printRecord(boot_rec);
    Serial.println("# (si la placa esta en tierra: comando MISION_BORRAR)");
    core.beginRecovered(millis(), sensors.imuOk(), sensors.baroOk(), boot_rec);
    pub_state = boot_rec.state;  // antes del primer ciclo: bloquea el modo simulación
  } else {
    core.begin(millis(), sensors.imuOk(), sensors.baroOk());
  }
#ifdef DEMO_SEPARACION
  core.setDemo(true);
#endif

  tx_queue = xQueueCreate(1, sizeof(TelemetryPacket));
  log_queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(TelemetryPacket));
  nvs_queue = xQueueCreate(1, sizeof(MissionRecord));
  sim_queue = xQueueCreate(SIM_QUEUE_LEN, sizeof(CoreInput));

  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, configMAX_PRIORITIES - 2,
                          nullptr, CONTROL_CORE);
  xTaskCreatePinnedToCore(commsTask, "comms", 8192, nullptr, 2, nullptr, COMMS_CORE);
}

void loop() {
  // Todo el trabajo está en las tareas; loop() no se usa.
  vTaskDelete(nullptr);
}
