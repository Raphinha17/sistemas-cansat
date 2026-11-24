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
//  - DEMO_SEPARACION (entornos heltec_demo y heltec_banco): ARMAR o un toque largo
//    (1 s) del botón PRG arman la cuenta regresiva; el servo se dispara DEMO_DELAY_S
//    segundos después. Otro toque largo durante la cuenta cancela. LED de la placa:
//    parpadeo rápido durante la cuenta, fijo tras el disparo. El modo de vuelo no cambia
//    y no usa el botón ni el LED.
//  - Modo de despliegue B (lanzamiento real, por defecto), A o C (comando MODO, guardado en
//    NVS; ver config.h). STAT y el arranque muestran el modo activo y sus umbrales.
//
// Encendido: interruptor deslizable en la línea de la pila; al encender, la placa
// arranca y entra sola a AUTOPRUEBA -> TIERRA (sin armado).
//
// Comandos por serial (115200, terminados en salto de línea):
//   TX1           telemetría según el estado (por defecto)
//   TX5           telemetría a 5 Hz en los estados 0 y 1 (pruebas)
//   STAT          estadísticas y estado
//   DUMP          vuelca el log en hex
//   BORRAR        borra el log
//   MISION        muestra el registro de misión guardado en NVS
//   MISION_BORRAR borra el registro de misión y reinicia la placa
//   MODO          muestra el modo de despliegue; MODO A|B|C lo cambia (estados 0 y 1)
//   SIM_INICIO    entra al modo simulación (o lo reinicia de cero) sin esperar sensores
//   SIM_FIN       sale del modo simulación (vuelve a los sensores y recalibra)
//   SIM_PROXIMA   responde "SIM_PROXIMA t_ms": próxima línea SIM esperada (sim_serial.py)
//   MISION_PRUEBA guarda en NVS una misión en curso FALSA (para probar el reinicio en vuelo)
//   ARMAR         (solo DEMO_SEPARACION) arma (o re-arma) la cuenta regresiva del servo
//   DESARMAR      (solo DEMO_SEPARACION) cancela la cuenta regresiva
#include <Arduino.h>

#include "config.h"
#include "flight_core.h"
#include "log_usage.h"
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
static volatile int8_t req_mode = -1;  // DeployMode pedido por el comando MODO (-1: nada)
static volatile bool sim_mode = false;
static volatile bool req_flush = false;

// Publicado por control para comms (diagnóstico y avisos por serial)
static volatile uint8_t pub_state = ST_AUTOPRUEBA, pub_flags = 0;
static volatile uint32_t pub_fire_ms = 0, pub_fire_count = 0;
static volatile float pub_fire_h = 0, pub_fire_vz = 0;  // estimados en el ciclo del disparo
static volatile uint16_t pub_lowg = 0;                  // liberaciones falsas descartadas
static volatile uint32_t pub_t_ms = 0;                  // reloj del último ciclo de control
static volatile float pub_h = 0, pub_vz = 0, pub_hmax = 0, pub_tilt = 0, pub_demo_rest = 0;
static volatile uint8_t pub_servo_int = 0;
static volatile bool pub_servo_conf = false, pub_servo_on = false, pub_vz_fuera = false;
static volatile bool pub_p0_pausa = false;          // promedio de p0 en pausa (TIERRA)
static volatile float pub_p0 = 0;
static volatile uint8_t pub_mode = DEPLOY_MODE_B;  // modo de despliegue aplicado (setup)
static volatile uint8_t pub_demo = 0;              // 0 sin armar, 1 cuenta, 2 disparado

// Estadísticas (escritas por control, leídas por comms solo para diagnóstico)
static volatile uint32_t st_cycles = 0, st_overruns = 0, st_max_us = 0;
static volatile uint32_t st_log_drops = 0, st_imu_err = 0, st_baro_err = 0;
static uint32_t st_log_lost = 0;  // registros sin guardar por log lleno (solo comms)
static volatile uint32_t st_sim_drops = 0;
static SimLink sim_link;  // integridad de las líneas SIM (solo la usa comms)

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
#ifdef DEMO_SEPARACION
  LongPress boton(DEMO_BOTON_MS);
#endif

  for (;;) {
    // --- Cambios de modo pedidos por serial ---
    if (req_sim_on) {
      req_sim_on = false;
      sim_mode = true;
      sim_fresh = true;                 // la primera línea reinicia el núcleo de cero
      xQueueReset(sim_queue);
      pub_state = ST_AUTOPRUEBA;        // como un arranque nuevo (permite MODO, SIM_INICIO...)
      pub_flags = 0;
    }
    // Modo de despliegue: solo en AUTOPRUEBA o TIERRA (nunca con la misión en curso).
    if (req_mode >= 0) {
      const int8_t m = req_mode;
      req_mode = -1;
      if (core.state() <= ST_TIERRA) core.setDeployMode((DeployMode)m);
      pub_mode = core.deployMode();
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

    // --- Demo: armado por comando o por toque largo del botón PRG ---
    if (req_demo_arm) {
      req_demo_arm = false;
      core.demoArm(in.t_ms);
    }
    if (req_demo_disarm) {
      req_demo_disarm = false;
      core.demoDisarm();
    }
#ifdef DEMO_SEPARACION
    // Activo en bajo. El tiempo del toque es el reloj real (también en modo simulación).
    if (boton.step(digitalRead(PIN_BOTON_PRG) == LOW, millis())) core.demoToggle(in.t_ms);
#endif

    // --- Núcleo: filtros + máquina de estados ---
    core.step(in);
    cycle++;
    servo_hw::apply(core.servo());
#ifdef DEMO_SEPARACION
    digitalWrite(PIN_LED_PLACA, core.demo().ledOn(in.t_ms) ? HIGH : LOW);
#endif

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
      pub_fire_h = core.kf().h();
      pub_fire_vz = core.kf().vz();
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
    pub_t_ms = in.t_ms;
    pub_flags = core.flags();
    pub_h = core.kf().h();
    pub_vz = core.kf().vz();
    pub_hmax = so.h_max;
    pub_tilt = core.tiltDeg();
    pub_servo_on = servo_hw::powered();
    pub_servo_int = so.servo_intentos;
    pub_servo_conf = so.servo_confirmado;
    pub_vz_fuera = so.vz_fuera_rango;
    pub_lowg = so.lowg_descartadas;
    pub_p0_pausa = core.p0Paused();
    pub_p0 = core.base().p0();
    pub_demo_rest = core.demo().remainingS(in.t_ms);
    pub_demo = core.demo().counting() ? 1 : core.demo().fired() ? 2 : 0;

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

// Modo activo y sus umbrales (STAT, MODO y arranque). Dos líneas en A y B.
static void printMode(DeployMode m) {
  const DeployParams p = deployParams(m);
  if (p.immediate) {
    Serial.printf("despliegue=%c [%s] disparo al detectar la caida libre (baja g %.0f ms), "
                  "sin altura minima ni compuerta\n",
                  deployModeChar(m), p.name, p.lowg_t_s * 1000);
    return;
  }
  Serial.printf("despliegue=%c [%s] liberacion valida > %.0f m, autogiro a %.0f m "
                "(h_disp = %.0f + |vz|*%.1f), respaldo %.0f m, compuerta %.0f grados max "
                "%.1f s\n",
                deployModeChar(m), p.name, p.release_min_m, p.deploy_alt_m, p.deploy_alt_m,
                SM_T_LATENCIA_S, p.backup_alt_m, SM_COMPUERTA_DEG, p.gate_backup_s);
  Serial.printf("  deteccion: baja g %.0f ms + vz < %.0f m/s en %.1f s | vz < %.0f m/s %.1f s "
                "| %.0f m bajo h_max",
                p.lowg_t_s * 1000, p.confirm_vz_ms, p.confirm_t_s, p.fast_vz_ms, p.fast_t_s,
                p.drop_from_max_m);
  if (p.drop_vz_ms < 1e3f) Serial.printf(" con vz < %.0f m/s", p.drop_vz_ms);
  Serial.printf(" | sin baro: disparo %.1f s tras liberar%s\n", p.t_fire_no_baro_s,
                p.late_fire_now ? " | liberacion tardia: disparo inmediato" : "");
}

static void sendResendRequest() {
  uint32_t t;
  if (sim_link.takeResend(t)) Serial.printf("SIM_REPITE %lu\n", (unsigned long)t);
}

static void handleSim(const char *line) {
  const uint32_t now = millis();
  if (!sim_mode && !req_sim_on) {
    // Solo en tierra: nunca se entra en este modo con la misión en curso.
    CoreInput tmp;
    if (pub_state > ST_TIERRA || !parseSimLine(line, tmp)) {
      sim_link.reject(now);  // fuera del modo simulación solo se cuenta (sin reenvío)
      return;
    }
    req_sim_on = true;
    sim_link.reset();
    Serial.println("# SIM: modo simulacion activado (sensores ignorados, sin NVS)");
  }
  if (uxQueueSpacesAvailable(sim_queue) == 0) {
    // Cola llena (la PC manda más rápido de lo que se procesa): la línea se pierde y, en
    // modo estricto, la siguiente pide el reenvío.
    st_sim_drops++;
    return;
  }
  CoreInput in;
  if (sim_link.feed(line, now, in) == SimLink::ACEPTADA) xQueueSend(sim_queue, &in, 0);
  sendResendRequest();
}

// Devuelve false si el texto no es un comando conocido.
static bool handleCommand(const String &cmd) {
  if (cmd == "TX1") {
    tx_test = false;
    Serial.println("OK TX segun el estado");
  } else if (cmd == "TX5") {
    tx_test = true;
    Serial.println("OK TX 5 Hz en estados 0 y 1");
  } else if (cmd == "STAT") {
    printMode((DeployMode)pub_mode);
    Serial.printf("estado=%u banderas=0x%02X h=%.1f vz=%.1f h_max=%.1f incl=%.0f modo=%s%s\n",
                  pub_state, pub_flags, pub_h, pub_vz, pub_hmax, pub_tilt,
                  sim_mode ? "SIM" : "sensores", core.demoEnabled() ? " DEMO" : "");
    Serial.printf("servo=%s intentos=%u confirmado=%u disparos=%lu ultimo_disparo_ms=%lu "
                  "vz_fuera_rango=%u\n",
                  pub_servo_on ? "ON" : "OFF", pub_servo_int, pub_servo_conf,
                  (unsigned long)pub_fire_count, (unsigned long)pub_fire_ms, pub_vz_fuera);
    Serial.printf("liberaciones_descartadas=%u disparo_h=%.1f disparo_vz=%.1f "
                  "h_despliegue=%.1f p0=%.1f Pa p0_pausa=%u\n",
                  pub_lowg, pub_fire_h, pub_fire_vz,
                  pub_fire_h - fabsf(pub_fire_vz) * SM_T_LATENCIA_S, pub_p0,
                  pub_p0_pausa ? 1u : 0u);
    Serial.printf("ciclos=%lu sobretiempos=%lu max_us=%lu imu_err=%lu baro_err=%lu\n",
                  (unsigned long)st_cycles, (unsigned long)st_overruns,
                  (unsigned long)st_max_us, (unsigned long)st_imu_err,
                  (unsigned long)st_baro_err);
    Serial.printf("tx_ok=%lu tx_omitidos=%lu tx_err=%lu\n", (unsigned long)telemetry::sent(),
                  (unsigned long)telemetry::skipped(), (unsigned long)telemetry::errors());
    Serial.printf("log=%u/%u B (%u %%) descartados=%lu sin_espacio=%lu nvs_escrituras=%lu\n",
                  (unsigned)logger::sizeBytes(), (unsigned)logger::capacityBytes(),
                  logPercent(logger::sizeBytes(), logger::capacityBytes()),
                  (unsigned long)st_log_drops, (unsigned long)st_log_lost, (unsigned long)mission_store::writes());
    Serial.printf("sim_rx=%lu sim_descartadas=%lu sim_invalidas=%lu sim_huecos=%lu "
                  "sim_repetidas=%lu sim_repite=%lu sim_crc=%u\n",
                  (unsigned long)sim_link.accepted(), (unsigned long)st_sim_drops,
                  (unsigned long)sim_link.invalid(), (unsigned long)sim_link.gaps(),
                  (unsigned long)sim_link.duplicates(), (unsigned long)sim_link.resends(),
                  sim_link.strict() ? 1u : 0u);
  } else if (cmd == "DUMP") {
    logger::dump(Serial);
  } else if (cmd == "BORRAR") {
    const size_t antes = logger::sizeBytes();
    logger::erase();
    st_log_lost = 0;
    Serial.printf("OK log vaciado (%u B liberados): log=%u/%u B (%u %%)\n", (unsigned)antes,
                  (unsigned)logger::sizeBytes(), (unsigned)logger::capacityBytes(),
                  logPercent(logger::sizeBytes(), logger::capacityBytes()));
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
  } else if (cmd == "MODO") {
    printMode((DeployMode)pub_mode);
  } else if (cmd.startsWith("MODO ")) {
    DeployMode m;
    String arg = cmd.substring(5);
    arg.trim();
    if (arg.length() != 1 || !deployModeFromChar(arg[0], m)) {
      Serial.println("ERR uso: MODO A|B|C");
    } else if (pub_state > ST_TIERRA) {
      Serial.println("ERR el modo solo se cambia en los estados 0 y 1 (en tierra)");
    } else if ((uint8_t)m == pub_mode) {
      Serial.printf("OK modo %c (sin cambios)\n", deployModeChar(m));
    } else {
      req_mode = (int8_t)m;  // lo aplica control; comms lo guarda en NVS y en el log
    }
  } else if (cmd == "SIM_INICIO") {
    // Entra al modo simulación sin depender de los sensores (banco sin MPU6500/BMP280), o
    // lo reinicia de cero tras un vuelo simulado. Nunca con una misión real en curso.
    if (!sim_mode && pub_state > ST_TIERRA) {
      Serial.println("ERR hay una mision en curso; MISION_BORRAR y reiniciar la placa");
    } else {
      req_sim_on = true;
      sim_link.reset();
      pub_state = ST_AUTOPRUEBA;
      Serial.println("OK SIM_INICIO: modo simulacion (sensores ignorados, sin NVS); "
                     "manda lineas SIM,...");
    }
  } else if (cmd == "MISION_PRUEBA") {
    // Solo para probar el reinicio en vuelo sin volar: guarda una misión FALSA en curso.
    if (sim_mode || pub_state > ST_TIERRA) {
      Serial.println("ERR solo en los estados 0 y 1 y fuera del modo simulacion");
    } else {
      MissionRecord r;
      r.mission = 1;
      r.state = ST_ASCENSO;
      r.flags = FLAG_BASE_LISTA;
      Serial.println(mission_store::save(r)
                         ? "OK mision de PRUEBA guardada (ASCENSO). Reinicia con el boton RST: "
                           "debe decir # REINICIO EN VUELO. Para quitarla: MISION_BORRAR"
                         : "ERR no se pudo guardar en NVS");
    }
  } else if (cmd == "SIM_PROXIMA") {
    // Para sim_serial.py al terminar: si faltan las últimas líneas, las reenvía.
    if (sim_link.hasNext())
      Serial.printf("SIM_PROXIMA %lu\n", (unsigned long)sim_link.next());
    else
      Serial.println("SIM_PROXIMA -1");
  } else if (cmd == "SIM_FIN") {
    req_sim_off = true;
    sim_link.reset();
    Serial.println("OK fin del modo simulacion (recalibrando con sensores)");
  } else if (cmd == "ARMAR" || cmd == "DESARMAR") {
    if (!core.demoEnabled()) {
      Serial.println("ERR solo en el modo DEMO_SEPARACION (heltec_demo o heltec_banco)");
    } else if (cmd == "ARMAR") {
      req_demo_arm = true;
      Serial.printf("OK armado: disparo en %d s\n", DEMO_DELAY_S);
    } else {
      req_demo_disarm = true;
      Serial.println("OK desarmado");
    }
  } else {
    return false;
  }
  return true;
}

static void printHelp() {
  Serial.println("Comandos: TX1 TX5 STAT DUMP BORRAR MISION MISION_BORRAR MISION_PRUEBA "
                 "MODO [A|B|C] SIM_INICIO SIM_FIN SIM_PROXIMA ARMAR DESARMAR | lineas SIM,... "
                 "(docs/sim_input.md)");
}

static void commsTask(void *) {
  char line[SIM_LINEA_MAX + 1];
  size_t len = 0;
  bool overflow = false;
  uint32_t last_flush = millis();
  uint8_t last_state = 0xFF;
  uint32_t last_fire_count = 0;
  uint8_t saved_mode = pub_mode;  // el modo con que arrancó (ya está en NVS o es el default)
  uint8_t last_demo = 0;
  TelemetryPacket pkt;
  MissionRecord rec;
  for (;;) {
    // Espera corta por registros del log; despierta al menos cada 5 ms.
    if (xQueueReceive(log_queue, &pkt, pdMS_TO_TICKS(5)) == pdTRUE) {
      if (!logger::append(pkt)) st_log_lost++;
      while (xQueueReceive(log_queue, &pkt, 0) == pdTRUE)
        if (!logger::append(pkt)) st_log_lost++;
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
    // Al despegar (o al armar el demo) el log debe tener sitio para todo el vuelo: si no,
    // se vacía el log de arranque y pruebas (los datos de vuelo nunca se pierden).
    if ((pub_state == ST_ASCENSO && last_state <= ST_TIERRA) ||
        (pub_demo == 1 && last_demo == 0)) {
      if (logNeedsRoom(logger::sizeBytes(), logger::capacityBytes())) {
        const size_t antes = logger::sizeBytes();
        logger::erase();
        logger::appendEvent(LOG_EV_LOG_VACIADO, 0, millis());
        Serial.printf("# AVISO: log vaciado al despegar/armar (%u B de arranque y pruebas "
                      "descartados) para guardar el vuelo\n", (unsigned)antes);
      }
    }
    if (pub_state != last_state) {
      last_state = pub_state;
      Serial.printf("# ESTADO %u banderas=0x%02X t_ms=%lu\n", last_state, pub_flags,
                    (unsigned long)pub_t_ms);
    }
    if (pub_mode != saved_mode) {
      // Modo nuevo aplicado por control: se guarda en NVS y queda en el log.
      saved_mode = pub_mode;
      const bool ok = mission_store::saveMode((DeployMode)saved_mode);
      logger::appendEvent(LOG_EV_MODO, (uint8_t)deployModeChar((DeployMode)saved_mode),
                          millis());
      Serial.printf("OK modo %c%s\n", deployModeChar((DeployMode)saved_mode),
                    ok ? " (guardado en NVS)" : " (ERROR al guardar en NVS)");
      printMode((DeployMode)saved_mode);
    }
    if (pub_demo != last_demo) {
      if (pub_demo == 1)
        Serial.printf("# DEMO ARMADO t_ms=%lu: disparo en %d s (toque largo de PRG o DESARMAR "
                      "cancela)\n",
                      (unsigned long)pub_t_ms, DEMO_DELAY_S);
      else if (last_demo == 1 && pub_demo == 0)
        Serial.println("# DEMO CANCELADO");
      last_demo = pub_demo;
    }
    if (pub_fire_count != last_fire_count) {
      last_fire_count = pub_fire_count;
      Serial.printf("# %s t_ms=%lu h=%.1f vz=%.1f h_despliegue=%.1f\n",
                    core.demoEnabled() ? "SEPARACION" : "DISPARO", (unsigned long)pub_fire_ms,
                    pub_fire_h, pub_fire_vz,
                    pub_fire_h - fabsf(pub_fire_vz) * SM_T_LATENCIA_S);
    }

    while (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == '\n' || c == '\r') {
        const bool sim = sim_mode || req_sim_on;
        if (len && overflow) {
          // Línea demasiado larga (en simulación: varias líneas pegadas por bytes perdidos).
          if (sim) {
            sim_link.reject(millis());
            sendResendRequest();
          }
        } else if (len) {
          line[len] = 0;
          if (strncmp(line, "SIM,", 4) == 0) {
            handleSim(line);
          } else {
            String cmd(line);
            cmd.trim();
            cmd.toUpperCase();
            if (cmd.length() && !handleCommand(cmd)) {
              if (sim) {
                // Fragmento de una línea SIM que perdió el inicio: inválida, sin la ayuda.
                sim_link.reject(millis());
                sendResendRequest();
              } else {
                Serial.printf("ERR comando desconocido: \"%s\" (%u car.)\n", cmd.c_str(),
                              (unsigned)cmd.length());
                printHelp();
              }
            }
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
#ifdef DEMO_SEPARACION
  static_assert(PIN_LED_PLACA == LED_BUILTIN, "PIN_LED_PLACA debe ser el LED de la variante");
  pinMode(PIN_BOTON_PRG, INPUT_PULLUP);
  pinMode(PIN_LED_PLACA, OUTPUT);
  digitalWrite(PIN_LED_PLACA, LOW);
#endif
  // Búfer de recepción grande (antes de begin): en el modo simulación llegan ~7 kB/s y
  // comms puede estar ocupada decenas de ms (LittleFS); con los 256 B por defecto del
  // núcleo Arduino se perdían bytes. No cambia nada del vuelo.
  Serial.setRxBufferSize(SERIAL_RX_BUF);
  Serial.begin(115200);
  delay(200);
#ifdef DEMO_SEPARACION
  Serial.printf("CanSat Hellmary - MODO DEMO_SEPARACION (disparo %d s tras ARMAR o toque "
                "largo de PRG)%s\n",
                DEMO_DELAY_S,
#ifdef CANSAT_BANCO
                " - BANCO: sin medicion de bateria. NO USAR PARA VUELO"
#else
                ""
#endif
  );
#else
  Serial.println("CanSat Hellmary - firmware de vuelo");
#endif

  sensors.begin();
  Serial.printf("IMU %s (WHO_AM_I=0x%02X)  BARO %s\n", sensors.imuOk() ? "OK" : "FALLA",
                sensors.imuWhoAmI(), sensors.baroOk() ? "OK" : "FALLA");
  Serial.printf("LittleFS %s\n", logger::begin() ? "OK" : "FALLA");
  Serial.printf("Log %u/%u B (%u %%)\n", (unsigned)logger::sizeBytes(),
                (unsigned)logger::capacityBytes(),
                logPercent(logger::sizeBytes(), logger::capacityBytes()));
  if (logNeedsWarning(logger::sizeBytes(), logger::capacityBytes()))
    Serial.printf("# AVISO: log al %u %%, usa BORRAR antes del vuelo\n",
                  logPercent(logger::sizeBytes(), logger::capacityBytes()));
  const int16_t rs = telemetry::begin();
  Serial.printf("Radio %s (codigo %d)\n", rs == 0 ? "OK" : "FALLA", rs);
  Serial.printf("NVS %s\n", mission_store::begin() ? "OK" : "FALLA");

  // Modo de despliegue guardado (o el de config.h). Queda en el log en cada arranque.
  DeployMode mode = deployModeDefault();
  const bool mode_saved = mission_store::loadMode(mode);
  core.setDeployMode(mode);
  pub_mode = mode;
  Serial.print(mode_saved ? "Modo guardado: " : "Modo por defecto: ");
  printMode(mode);
  if (mode != deployModeDefault())
    Serial.printf("# AVISO: modo %c activo; el lanzamiento real es el modo %c (manda MODO %c)\n",
                  deployModeChar(mode), deployModeChar(deployModeDefault()),
                  deployModeChar(deployModeDefault()));
  logger::appendEvent(LOG_EV_ARRANQUE, (uint8_t)deployModeChar(mode), millis());

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
