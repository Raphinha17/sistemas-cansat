// Pruebas de la máquina de estados con vuelos de tools/flight_sim.py, pasando por el
// mismo núcleo del lazo de control que la placa (FlightCore: calibración, Mahony,
// Kalman, fallas y máquina de estados). Los escenarios los genera
// scripts/gen_sim_data.py (ver CASES).
//
// Altura de despliegue: h_real(t_disparo) - |vz_real(t_disparo)| * SM_T_LATENCIA_S, es
// decir, dónde queda el CanSat cuando el autogiro actúa tras la latencia supuesta. Se usa
// la verdad del simulador en el instante del disparo del firmware (la física del
// simulador despliega según su propia réplica de la lógica, no según este firmware).
#include <math.h>
#include <stdio.h>
#include <unity.h>

#include <map>
#include <string>
#include <vector>

#include "../sim_csv.h"
#include "config.h"
#include "flight_core.h"
#include "sim_input.h"

void setUp() {}
void tearDown() {}

static const std::vector<SimRow> &rowsOf(const char *name) {
  static std::map<std::string, std::vector<SimRow>> cache;
  auto it = cache.find(name);
  if (it == cache.end()) it = cache.emplace(name, loadSim(name)).first;
  TEST_ASSERT_TRUE_MESSAGE(it->second.size() > 10000, "no se pudo leer el CSV del simulador");
  return it->second;
}

static float firstPhaseT(const std::vector<SimRow> &rows, const char *fase) {
  for (const SimRow &r : rows)
    if (r.fase == fase) return r.t;
  return -1;
}

struct Transition {
  float t;
  int state;
};

struct RunResult {
  std::vector<Transition> trans;
  uint8_t flags = 0;  // banderas finales del paquete
  uint8_t flags_ever = 0;
  float t_release = -1, t_hdisp = -1, t_fire = -1, t_land = -1;
  float h_est_fire = 0, h_true_fire = 0, vz_true_fire = 0, tilt_fire = 0, tilt_hdisp = 0;
  bool servo_power_before_caida = false;
  bool servo_power_ever = false;
  bool servo_confirmed = false;
  int servo_attempts = 0;
  int lowg_descartadas = 0;
  float h_est_end = 0;
  bool vz_fuera_mid_descent = true;
  // Reinicio
  bool rebooted = false;
  int state_at_reboot = -1, state_after_reboot = -1;
  float p0_saved = 0, p0_after_reboot = 0;
  bool gyro_ready_after_reboot = false;

  float deployAlt() const { return h_true_fire - fabsf(vz_true_fire) * SM_T_LATENCIA_S; }
  std::string states() const {
    std::string s;
    for (const Transition &t : trans) s += char('0' + t.state);
    return s;
  }
};

struct RunOpts {
  float reboot_at = -1;    // s: simula un reinicio de la placa en este instante
  float reboot_gap = 0.8f; // s sin datos durante el reinicio (arranque + setup)
};

static RunResult runSim(const char *name, RunOpts o = RunOpts()) {
  const std::vector<SimRow> &rows = rowsOf(name);
  const float t_impact = firstPhaseT(rows, "impact");
  RunResult R;
  FlightCore core;
  core.begin(0, true, true);
  MissionRecord nvs;  // "NVS": se copia el registro cada vez que cambia su contador
  uint32_t seq_seen = 0;
  int last_state = -1;
  uint32_t cycle = 0;

  for (size_t i = 0; i < rows.size(); i++) {
    if (o.reboot_at > 0 && !R.rebooted && rows[i].t >= o.reboot_at) {
      R.rebooted = true;
      R.state_at_reboot = core.state();
      i += (size_t)(o.reboot_gap * CONTROL_HZ);
      if (i >= rows.size()) break;
      R.p0_saved = nvs.p0_pa;
      const uint32_t t_ms = (uint32_t)lroundf(rows[i].t * 1000);
      if (nvs.mission)
        core.beginRecovered(t_ms, true, true, nvs);
      else
        core.begin(t_ms, true, true);
      R.state_after_reboot = core.sm().state();
      R.gyro_ready_after_reboot = core.gyroReady();
      R.p0_after_reboot = core.base().p0();
      seq_seen = core.recordSeq();
      cycle = 0;
    }
    const SimRow &r = rows[i];
    CoreInput in;
    in.t_ms = (uint32_t)lroundf(r.t * 1000);
    // El simulador marca el IMU muerto con ceros: en la placa sería una lectura fallida.
    bool any = false;
    for (int k = 0; k < 3; k++) {
      in.acc_g[k] = r.acc[k];
      in.gyro_dps[k] = r.gyro[k];
      any |= r.acc[k] != 0 || r.gyro[k] != 0;
    }
    in.imu_valid = any;
    in.baro_sampled = (cycle % BARO_DECIM) == 0;  // como el lazo de la placa (25 Hz)
    in.baro_valid = true;
    in.p_pa = r.p_pa;
    in.temp_c = r.temp_c;
    core.step(in);
    cycle++;

    if (core.recordSeq() != seq_seen) {
      nvs = core.record();
      seq_seen = core.recordSeq();
    }
    const int st = core.state();
    if (st != last_state) {
      R.trans.push_back({r.t, st});
      last_state = st;
    }
    const ServoCmd sc = core.servo();
    if (sc.power) R.servo_power_ever = true;
    if (sc.power && st < ST_CAIDA) R.servo_power_before_caida = true;
    if (st == ST_CAIDA && R.t_release < 0) R.t_release = r.t;
    if (core.sm().tHdisp() && R.t_hdisp < 0) {
      R.t_hdisp = r.t;
      R.tilt_hdisp = core.tiltDeg();
    }
    if (core.evFire() && R.t_fire < 0) {
      R.t_fire = r.t;
      R.h_est_fire = core.kf().h();
      R.h_true_fire = r.h;
      R.vz_true_fire = r.vz;
      R.tilt_fire = core.tiltDeg();
    }
    if (st == ST_ATERRIZADO && R.t_land < 0) R.t_land = r.t;
    if (t_impact > 0 && fabsf(r.t - (t_impact - 3.0f)) < 0.005f && st == ST_DESCENSO)
      R.vz_fuera_mid_descent = core.sm().out().vz_fuera_rango;
    R.flags_ever |= core.flags();
  }
  R.flags = core.flags();
  R.servo_confirmed = core.sm().out().servo_confirmado;
  R.servo_attempts = core.sm().out().servo_intentos;
  R.lowg_descartadas = core.sm().out().lowg_descartadas;
  R.h_est_end = core.kf().h();

  char msg[300];
  snprintf(msg, sizeof msg,
           "%s: estados=%s lib=%.2f hdisp=%.2f disp=%.2f (h_est=%.1f h_real=%.1f vz=%.1f "
           "incl=%.0f/%.0f) despl=%.1f m aterr=%.2f flags=0x%02X servo=%d/%d",
           name, R.states().c_str(), R.t_release, R.t_hdisp, R.t_fire, R.h_est_fire,
           R.h_true_fire, R.vz_true_fire, R.tilt_hdisp, R.tilt_fire, R.deployAlt(), R.t_land,
           R.flags, R.servo_attempts, (int)R.servo_confirmed);
  TEST_MESSAGE(msg);
  return R;
}

// ---------------------------------------------------------------------------
// Vuelo nominal completo
// ---------------------------------------------------------------------------
static void check_nominal(const char *name) {
  const std::vector<SimRow> &rows = rowsOf(name);
  const float t_fall = firstPhaseT(rows, "fall");
  const float t_impact = firstPhaseT(rows, "impact");
  const RunResult R = runSim(name);

  TEST_ASSERT_EQUAL_STRING("0123456", R.states().c_str());  // en orden, sin retrocesos
  TEST_ASSERT_FALSE(R.servo_power_before_caida);
  // Liberación detectada poco después de la real
  TEST_ASSERT_TRUE(R.t_release >= t_fall);
  TEST_ASSERT_TRUE(R.t_release - t_fall < 1.5f);
  // Disparo por altura con la compuerta cumplida, despliegue a 200 m ±3 m
  TEST_ASSERT_BITS(FLAG_LIBERACION | FLAG_DISPARO, FLAG_LIBERACION | FLAG_DISPARO, R.flags);
  TEST_ASSERT_EQUAL(0, R.flags & FLAG_DISPARO_RESPALDO);
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 200.0f, R.deployAlt());
  TEST_ASSERT_TRUE(R.servo_confirmed);
  // Descenso dentro de 8..12 m/s y aterrizaje poco después del impacto
  TEST_ASSERT_FALSE(R.vz_fuera_mid_descent);
  // El impacto del simulador (8 g durante 30 ms) no frena de -10 a 0 m/s, así que el
  // Kalman tarda ~3 s en llevar vz a ~0 (PREGUNTAS P10): por eso la tolerancia de 6 s.
  TEST_ASSERT_TRUE(R.t_land > t_impact);
  TEST_ASSERT_TRUE(R.t_land - t_impact < 6.0f);
  TEST_ASSERT_BITS(FLAG_ATERRIZADO | FLAG_BASE_LISTA, FLAG_ATERRIZADO | FLAG_BASE_LISTA, R.flags);
  TEST_ASSERT_EQUAL(0, R.flags_ever & (FLAG_FALLO_BARO | FLAG_FALLO_IMU | FLAG_NO_GO));
  // p0 congelado sin el sesgo de la subida: al aterrizar la altura vuelve a ~0
  TEST_ASSERT_FLOAT_WITHIN(1.5f, 0.0f, R.h_est_end);
}

static void test_nominal_seed1() { check_nominal("flight_seed1"); }
static void test_nominal_seed7() { check_nominal("flight_seed7"); }

// ---------------------------------------------------------------------------
// Liberación válida y falsas liberaciones
// ---------------------------------------------------------------------------
static void test_liberacion_valida() {
  // En el vuelo nominal la liberación ocurre a 450 m y se detecta por baja g confirmada
  // por vz (criterio a), antes que por vz < -12 m/s durante 0.4 s (criterio b).
  const std::vector<SimRow> &rows = rowsOf("flight_seed1");
  const float t_fall = firstPhaseT(rows, "fall");
  float t_vz12 = -1;  // cuándo la vz real pasa de -12 m/s (+0.4 s del criterio b)
  for (const SimRow &r : rows)
    if (r.t > t_fall && r.vz < -12.0f) {
      t_vz12 = r.t + SM_LIB_VZ_RAPIDA_T_S;
      break;
    }
  const RunResult R = runSim("flight_seed1");
  TEST_ASSERT_TRUE(R.t_release > t_fall + SM_LIB_LOWG_T_S);
  TEST_ASSERT_TRUE(R.t_release < t_vz12);
  TEST_ASSERT_EQUAL(0, R.lowg_descartadas);
}

static void test_falsa_liberacion_descartada() {
  // 0.25 s de caída libre a ~270 m durante el ascenso: hay baja g pero vz no llega a
  // -8 m/s, así que se descarta; la liberación real (450 m) sí se detecta.
  const std::vector<SimRow> &rows = rowsOf("false_release");
  const float t_fall = firstPhaseT(rows, "fall");
  const RunResult R = runSim("false_release");
  TEST_ASSERT_EQUAL(1, R.lowg_descartadas);
  TEST_ASSERT_TRUE(R.t_release >= t_fall);
  TEST_ASSERT_EQUAL_STRING("0123456", R.states().c_str());
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 200.0f, R.deployAlt());
}

static void test_liberacion_bajo_240_descartada() {
  // El dron suelta el CanSat a 200 m (< SM_LIB_H_MIN_M): nunca se declara liberación,
  // el servo nunca se alimenta.
  const RunResult R = runSim("low_release");
  TEST_ASSERT_EQUAL_STRING("012", R.states().c_str());
  TEST_ASSERT_EQUAL(0, R.flags_ever & (FLAG_LIBERACION | FLAG_DISPARO));
  TEST_ASSERT_FALSE(R.servo_power_ever);
}

// ---------------------------------------------------------------------------
// Disparo: compuerta y respaldos
// ---------------------------------------------------------------------------
static void test_disparo_compuerta_45() {
  // Cae inclinado ~60° hasta 214 m y luego se endereza: al cruzar h_disp la compuerta no
  // se cumple, el disparo espera y ocurre cuando el ángulo baja de 45° (sin respaldo).
  const RunResult R = runSim("gate_delay");
  TEST_ASSERT_TRUE(R.t_hdisp > 0 && R.t_fire > 0);
  TEST_ASSERT_TRUE(R.tilt_hdisp > SM_COMPUERTA_DEG);
  TEST_ASSERT_TRUE(R.tilt_fire <= SM_COMPUERTA_DEG);
  TEST_ASSERT_TRUE(R.t_fire - R.t_hdisp > 0.1f);
  TEST_ASSERT_TRUE(R.t_fire - R.t_hdisp < SM_COMPUERTA_RESPALDO_S);
  TEST_ASSERT_BITS(FLAG_DISPARO | FLAG_DISPARO_RESPALDO, FLAG_DISPARO, R.flags);
  TEST_ASSERT_EQUAL_STRING("0123456", R.states().c_str());
}

static void test_disparo_respaldo_1_5s() {
  // Cae inclinado ~70° todo el tiempo: la compuerta nunca se cumple y se dispara
  // 1.5 s después de cruzar h_disp, todavía por encima de 160 m.
  const RunResult R = runSim("tilt_backup");
  TEST_ASSERT_TRUE(R.t_hdisp > 0 && R.t_fire > 0);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, SM_COMPUERTA_RESPALDO_S, R.t_fire - R.t_hdisp);
  TEST_ASSERT_TRUE(R.h_est_fire > SM_H_RESPALDO_M);
  TEST_ASSERT_TRUE(R.tilt_fire > SM_COMPUERTA_DEG);
  TEST_ASSERT_BITS(FLAG_DISPARO | FLAG_DISPARO_RESPALDO, FLAG_DISPARO | FLAG_DISPARO_RESPALDO,
                   R.flags);
  TEST_ASSERT_EQUAL_STRING("0123456", R.states().c_str());
}

static void test_disparo_respaldo_160m() {
  // Inclinado ~70° y con caída más rápida (terminal 60 m/s): llega a 160 m antes de que
  // pasen los 1.5 s de la compuerta, y dispara por altura mínima.
  const RunResult R = runSim("fast_fall");
  TEST_ASSERT_TRUE(R.t_hdisp > 0 && R.t_fire > 0);
  TEST_ASSERT_TRUE(R.t_fire - R.t_hdisp < SM_COMPUERTA_RESPALDO_S);
  TEST_ASSERT_TRUE(R.h_est_fire <= SM_H_RESPALDO_M);
  TEST_ASSERT_TRUE(R.h_est_fire > SM_H_RESPALDO_M - 1.0f);  // en el primer ciclo bajo 160 m
  TEST_ASSERT_BITS(FLAG_DISPARO | FLAG_DISPARO_RESPALDO, FLAG_DISPARO | FLAG_DISPARO_RESPALDO,
                   R.flags);
}

// ---------------------------------------------------------------------------
// Fallas
// ---------------------------------------------------------------------------
static void test_falla_baro() {
  // El barómetro se congela a los 40 s (ascenso): se detecta tras 1 s, la liberación se
  // detecta solo por IMU y el disparo es por tiempo desde la liberación.
  const std::vector<SimRow> &rows = rowsOf("fail_baro");
  const float t_fall = firstPhaseT(rows, "fall");
  const RunResult R = runSim("fail_baro");
  TEST_ASSERT_TRUE(R.flags & FLAG_FALLO_BARO);
  TEST_ASSERT_EQUAL(0, R.flags & (FLAG_FALLO_IMU | FLAG_NO_GO));
  TEST_ASSERT_TRUE(R.t_release >= t_fall && R.t_release - t_fall < 1.5f);
  TEST_ASSERT_FLOAT_WITHIN(0.02f, SM_T_DISPARO_SIN_BARO_S, R.t_fire - R.t_release);
  TEST_ASSERT_BITS(FLAG_DISPARO | FLAG_DISPARO_RESPALDO, FLAG_DISPARO | FLAG_DISPARO_RESPALDO,
                   R.flags);
  TEST_ASSERT_FALSE(R.servo_power_before_caida);
  TEST_ASSERT_EQUAL_STRING("0123456", R.states().c_str());  // aterriza por impacto + reposo
}

static void test_falla_imu() {
  // El IMU deja de responder a los 40 s: liberación solo por barómetro y disparo a h_disp
  // sin compuerta de ángulo.
  const std::vector<SimRow> &rows = rowsOf("fail_imu");
  const float t_fall = firstPhaseT(rows, "fall");
  const RunResult R = runSim("fail_imu");
  TEST_ASSERT_TRUE(R.flags & FLAG_FALLO_IMU);
  TEST_ASSERT_EQUAL(0, R.flags & (FLAG_FALLO_BARO | FLAG_NO_GO));
  TEST_ASSERT_TRUE(R.t_release >= t_fall && R.t_release - t_fall < 3.0f);
  TEST_ASSERT_BITS(FLAG_DISPARO | FLAG_DISPARO_RESPALDO, FLAG_DISPARO, R.flags);
  // Sin acelerómetro vz va retrasada: tolerancia mayor que en el nominal (PREGUNTAS P21)
  TEST_ASSERT_FLOAT_WITHIN(10.0f, 200.0f, R.deployAlt());
  TEST_ASSERT_EQUAL(3, R.servo_attempts);  // sin IMU no hay confirmación: todos los intentos
  TEST_ASSERT_EQUAL_STRING("0123456", R.states().c_str());
}

// ---------------------------------------------------------------------------
// Reinicio en vuelo (estado en NVS)
// ---------------------------------------------------------------------------
static void test_reinicio_en_ascenso() {
  // Reinicio a los 60 s (~210 m subiendo): recupera ASCENSO y p0 desde NVS sin
  // recalibrar, y el resto del vuelo sigue igual (despliegue a 200 m ±3 m).
  RunOpts o;
  o.reboot_at = 60.0f;
  const RunResult R = runSim("flight_seed1", o);
  TEST_ASSERT_TRUE(R.rebooted);
  TEST_ASSERT_EQUAL(ST_ASCENSO, R.state_at_reboot);
  TEST_ASSERT_EQUAL(ST_ASCENSO, R.state_after_reboot);
  TEST_ASSERT_TRUE(R.gyro_ready_after_reboot);           // sin recalibrar el giroscopio
  TEST_ASSERT_EQUAL_FLOAT(R.p0_saved, R.p0_after_reboot);  // p0 recargado, no recalibrado
  TEST_ASSERT_TRUE(R.p0_saved > 90000.0f);
  TEST_ASSERT_EQUAL_STRING("0123456", R.states().c_str());  // sigue en 2 tras reiniciar
  TEST_ASSERT_FLOAT_WITHIN(3.0f, 200.0f, R.deployAlt());
  TEST_ASSERT_EQUAL(0, R.flags & FLAG_DISPARO_RESPALDO);
}

static void test_reinicio_en_caida() {
  // Reinicio 3 s después de la liberación (~410 m, cayendo a ~25 m/s): recupera CAIDA,
  // espera a que el Kalman converja y dispara igual.
  const std::vector<SimRow> &rows = rowsOf("flight_seed1");
  RunOpts o;
  o.reboot_at = firstPhaseT(rows, "fall") + 3.0f;
  const RunResult R = runSim("flight_seed1", o);
  TEST_ASSERT_EQUAL(ST_CAIDA, R.state_at_reboot);
  TEST_ASSERT_EQUAL(ST_CAIDA, R.state_after_reboot);
  TEST_ASSERT_TRUE(R.flags & FLAG_LIBERACION);
  TEST_ASSERT_TRUE(R.flags & FLAG_DISPARO);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 200.0f, R.deployAlt());
  TEST_ASSERT_EQUAL(ST_ATERRIZADO, R.trans.back().state);
}

// ---------------------------------------------------------------------------
// Formato del modo simulación por serial (docs/sim_input.md)
// ---------------------------------------------------------------------------
static void test_sim_line() {
  CoreInput c;
  TEST_ASSERT_TRUE(parseSimLine("SIM,1230,0.01,-0.02,1.003,0.5,-1.25,0.1,100912.5,24.87", c));
  TEST_ASSERT_EQUAL_UINT32(1230, c.t_ms);
  TEST_ASSERT_TRUE(c.imu_valid);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.003f, c.acc_g[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, -1.25f, c.gyro_dps[1]);
  TEST_ASSERT_TRUE(c.baro_valid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100912.5f, c.p_pa);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 24.87f, c.temp_c);
  // IMU fallido (campos vacíos) y barómetro fallido (nan)
  TEST_ASSERT_TRUE(parseSimLine("SIM,10,,,,,,,100000,25\r\n", c));
  TEST_ASSERT_FALSE(c.imu_valid);
  TEST_ASSERT_TRUE(c.baro_valid);
  TEST_ASSERT_TRUE(parseSimLine("SIM,20,0,0,1,0,0,0,nan,25", c));
  TEST_ASSERT_TRUE(c.imu_valid);
  TEST_ASSERT_FALSE(c.baro_valid);
  // Formato inválido
  TEST_ASSERT_FALSE(parseSimLine("SIM,20,0,0,1,0,0,0,100000", c));       // faltan campos
  TEST_ASSERT_FALSE(parseSimLine("SIM,20,0,0,1,0,0,0,100000,25,9", c));  // sobran
  TEST_ASSERT_FALSE(parseSimLine("SIM,,0,0,1,0,0,0,100000,25", c));      // sin t_ms
  TEST_ASSERT_FALSE(parseSimLine("STAT", c));
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_nominal_seed1);
  RUN_TEST(test_nominal_seed7);
  RUN_TEST(test_liberacion_valida);
  RUN_TEST(test_falsa_liberacion_descartada);
  RUN_TEST(test_liberacion_bajo_240_descartada);
  RUN_TEST(test_disparo_compuerta_45);
  RUN_TEST(test_disparo_respaldo_1_5s);
  RUN_TEST(test_disparo_respaldo_160m);
  RUN_TEST(test_falla_baro);
  RUN_TEST(test_falla_imu);
  RUN_TEST(test_reinicio_en_ascenso);
  RUN_TEST(test_reinicio_en_caida);
  RUN_TEST(test_sim_line);
  return UNITY_END();
}
