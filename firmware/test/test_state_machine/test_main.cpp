// Pruebas unitarias de la máquina de estados, la secuencia del servo y el modo demo
// con entradas sintéticas (casos de borde que el simulador no produce con precisión).
// Los escenarios de vuelo completos están en test_flight.
#include <math.h>
#include <unity.h>

#include "config.h"
#include "servo_release.h"
#include "state_machine.h"

void setUp() {}
void tearDown() {}

// Entrada base: sensores bien, calibrado, en reposo en el suelo.
static SmInput base(uint32_t t) {
  SmInput in;
  in.t_ms = t;
  in.calib_done = true;
  in.base_ready = true;
  in.p0_pa = 100000;
  in.h = 0;
  in.vz = 0;
  in.tilt_deg = 5;
  in.acc_g = 1;
  return in;
}

// Lleva la máquina hasta CAIDA con una liberación a 450 m. Devuelve el tiempo actual.
static uint32_t toCaida(StateMachine &sm) {
  uint32_t t = 0;
  sm.begin(t);
  for (; t < 2000; t += 10) sm.step(base(t));  // 0 -> 1
  TEST_ASSERT_EQUAL(ST_TIERRA, sm.state());
  // Ascenso a 6 m/s desde 30 m
  float h = 30;
  for (; h < 450; t += 10, h += 0.06f) {
    SmInput in = base(t);
    in.h = h;
    in.vz = 6;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_ASCENSO, sm.state());
  // Liberación: baja g y vz cayendo
  float vz = 0;
  for (int i = 0; i < 200 && sm.state() == ST_ASCENSO; i++, t += 10) {
    SmInput in = base(t);
    vz -= 0.098f;
    h += vz * 0.01f;
    in.h = h;
    in.vz = vz;
    in.acc_g = 0.05f;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_CAIDA, sm.state());
  return t;
}

static void test_servo_sin_energia_hasta_caida() {
  StateMachine sm;
  sm.begin(0);
  sm.step(base(0));
  TEST_ASSERT_FALSE(sm.out().servo.power);  // AUTOPRUEBA
  uint32_t t = toCaida(sm);
  SmInput in = base(t);
  in.h = 400;
  in.vz = -20;
  in.acc_g = 0.5f;
  sm.step(in);
  TEST_ASSERT_TRUE(sm.out().servo.power);  // CAIDA: alimentado en bloqueo
  TEST_ASSERT_EQUAL_FLOAT(SERVO_ANG_BLOQUEO_DEG, sm.out().servo.angle_deg);
  TEST_ASSERT_EQUAL(1000 / TX_HZ_CAIDA, sm.out().tx_period_ms);
}

static void test_despegue_congela_p0_quieto() {
  StateMachine sm;
  sm.begin(0);
  uint32_t t = 0;
  for (; t < 1000; t += 10) sm.step(base(t));
  // El promedio de p0 empieza a bajar con la subida: se congela el valor "quieto".
  float h = 0;
  bool froze = false;
  float p0_frozen = 0;
  for (; t < 20000 && !froze; t += 10) {
    SmInput in = base(t);
    h += 0.06f;
    in.h = h;
    in.vz = 6;
    in.p0_pa = 100000 - h * 0.5f;  // p0 "contaminado" por la subida
    sm.step(in);
    if (sm.out().ev_freeze_p0) {
      froze = true;
      p0_frozen = sm.out().p0_freeze_pa;
    }
  }
  TEST_ASSERT_TRUE(froze);
  TEST_ASSERT_EQUAL(ST_ASCENSO, sm.state());
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 100000.0f, p0_frozen);
  // 20 m + 3 s a 6 m/s
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 20.0f + 3.0f * 6.0f, h);
  TEST_ASSERT_EQUAL(1, sm.record().mission);
  TEST_ASSERT_EQUAL_FLOAT(p0_frozen, sm.record().p0_pa);
}

static void test_despegue_requiere_3s() {
  StateMachine sm;
  sm.begin(0);
  uint32_t t = 0;
  for (; t < 1000; t += 10) sm.step(base(t));
  // 2.5 s cumpliendo, una muestra que no cumple, y otra vez 2.5 s: no despega
  for (int rep = 0; rep < 2; rep++) {
    for (int i = 0; i < 250; i++, t += 10) {
      SmInput in = base(t);
      in.h = 25;
      in.vz = 2;
      sm.step(in);
    }
    SmInput in = base(t);
    in.h = 25;
    in.vz = 0.5f;
    sm.step(in);
    t += 10;
  }
  TEST_ASSERT_EQUAL(ST_TIERRA, sm.state());
}

static void test_compuerta_espera_y_dispara() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  // Bajo h_disp con 60°: espera; a los 0.8 s el ángulo baja a 30°: dispara sin respaldo.
  float h = 215;
  uint32_t t_cross = 0;
  for (int i = 0; i < 300 && sm.state() == ST_CAIDA; i++, t += 10) {
    SmInput in = base(t);
    in.h = h;
    in.vz = -35;
    in.acc_g = 1.0f;
    in.tilt_deg = (t_cross && t - t_cross >= 800) ? 30 : 60;
    sm.step(in);
    if (!t_cross && sm.tHdisp()) t_cross = t;
    h -= 0.35f;
  }
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  TEST_ASSERT_EQUAL_UINT32(t_cross + 800, sm.tFire());
  TEST_ASSERT_EQUAL(FLAG_DISPARO, sm.out().flags & (FLAG_DISPARO | FLAG_DISPARO_RESPALDO));
}

static void test_respaldo_160m_sintetico() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  // Caída a 60 m/s siempre inclinado: llega a 160 m antes de 1.5 s
  float h = 240;
  for (int i = 0; i < 300 && sm.state() == ST_CAIDA; i++, t += 10) {
    SmInput in = base(t);
    in.h = h;
    in.vz = -60;
    in.tilt_deg = 80;
    sm.step(in);
    h -= 0.6f;
  }
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_DISPARO_RESPALDO);
  TEST_ASSERT_TRUE((t - 10) - sm.tHdisp() < 1500);
}

static void test_sin_imu_no_hay_compuerta() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  SmInput in = base(t);
  in.imu_ok = false;
  in.h = 210;
  in.vz = -30;
  in.tilt_deg = 90;  // ignorado
  sm.step(in);
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  TEST_ASSERT_EQUAL(0, sm.out().flags & FLAG_DISPARO_RESPALDO);
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_FALLO_IMU);
}

static void test_no_go() {
  StateMachine sm;
  sm.begin(0);
  SmInput in = base(0);
  in.imu_ok = in.baro_ok = false;
  sm.step(in);
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_NO_GO);
}

static void test_reinicio_decide_por_vz() {
  MissionRecord rec;
  rec.mission = 1;
  rec.state = ST_ASCENSO;
  rec.p0_pa = 100000;
  rec.p0_valid = 1;
  rec.h_max_m = 440;
  // Reinició en ASCENSO pero ya cae a -25 m/s: pasa a CAIDA tras la espera.
  StateMachine sm;
  sm.beginRecovered(rec, 5000);
  uint32_t t = 5000;
  for (; t < 5000 + 900; t += 10) {
    SmInput in = base(t);
    in.h = 400;
    in.vz = -25;
    sm.step(in);
    TEST_ASSERT_EQUAL(ST_ASCENSO, sm.state());
    TEST_ASSERT_TRUE(sm.out().recuperando);
  }
  for (; t < 5000 + 1100; t += 10) {
    SmInput in = base(t);
    in.h = 395;
    in.vz = -25;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_CAIDA, sm.state());
  TEST_ASSERT_TRUE(sm.out().flags & FLAG_LIBERACION);
  // Y si sube (cerca del máximo guardado), sigue en ASCENSO
  StateMachine sm2;
  sm2.beginRecovered(rec, 0);
  for (t = 0; t < 1100; t += 10) {
    SmInput in = base(t);
    in.h = 430;
    in.vz = 5;
    sm2.step(in);
  }
  TEST_ASSERT_EQUAL(ST_ASCENSO, sm2.state());
  TEST_ASSERT_FALSE(sm2.out().recuperando);
}

static void test_aterrizado_tasas_y_mision() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  SmInput in = base(t);
  in.h = 210;
  in.vz = -30;
  sm.step(in);  // dispara (compuerta 5°)
  TEST_ASSERT_EQUAL(ST_DESPLIEGUE, sm.state());
  // Confirmación por pico de aceleración
  t += 10;
  in = base(t);
  in.acc_g = 5;
  in.h = 200;
  in.vz = -20;
  sm.step(in);
  TEST_ASSERT_EQUAL(ST_DESCENSO, sm.state());
  TEST_ASSERT_TRUE(sm.out().servo_confirmado);
  TEST_ASSERT_FALSE(sm.out().servo.power);
  // Descenso a 10 m/s 5 s, impacto y reposo
  for (int i = 0; i < 500; i++) {
    t += 10;
    in = base(t);
    in.h = 100;
    in.vz = -10;
    in.rate_dps = 90;
    sm.step(in);
  }
  TEST_ASSERT_FALSE(sm.out().vz_fuera_rango);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -10.0f, sm.out().vz_prom);
  t += 10;
  in = base(t);
  in.acc_g = 8;
  sm.step(in);
  for (int i = 0; i < 110; i++) {
    t += 10;
    sm.step(base(t));
  }
  TEST_ASSERT_EQUAL(ST_ATERRIZADO, sm.state());
  TEST_ASSERT_EQUAL(0, sm.record().mission);
  TEST_ASSERT_EQUAL(1000 / TX_HZ_ATERRIZADO, sm.out().tx_period_ms);
  for (int i = 0; i < SM_ATERRIZADO_TX_S * 100 + 10; i++) {
    t += 10;
    sm.step(base(t));
  }
  TEST_ASSERT_EQUAL(SM_BALIZA_PERIODO_MS, sm.out().tx_period_ms);
}

static void test_descenso_fuera_de_rango() {
  StateMachine sm;
  uint32_t t = toCaida(sm);
  SmInput in = base(t);
  in.h = 210;
  in.vz = -30;
  sm.step(in);
  // Sin confirmación: la secuencia dura 1 + 2 * (0.3 + 1) s
  for (int i = 0; i < 400 && sm.state() == ST_DESPLIEGUE; i++) {
    t += 10;
    in = base(t);
    in.h = 150;
    in.vz = -15;
    sm.step(in);
  }
  TEST_ASSERT_EQUAL(ST_DESCENSO, sm.state());
  TEST_ASSERT_FALSE(sm.out().servo_confirmado);
  TEST_ASSERT_EQUAL(1 + SERVO_REINTENTOS, sm.out().servo_intentos);
  for (int i = 0; i < 150; i++) {
    t += 10;
    in = base(t);
    in.h = 120;
    in.vz = -15;  // demasiado rápido
    in.rate_dps = 90;
    sm.step(in);
  }
  TEST_ASSERT_TRUE(sm.out().vz_fuera_rango);
}

// ---------------------------------------------------------------------------
// Secuencia del servo
// ---------------------------------------------------------------------------
static void test_servo_reintentos() {
  ServoSequencer s;
  s.start(0);
  TEST_ASSERT_TRUE(s.cmd().power);
  TEST_ASSERT_EQUAL_FLOAT(SERVO_ANG_LIBERA_DEG, s.cmd().angle_deg);
  // Sin confirmación: -80 (1 s), -10 (0.3 s), -80 (1 s), -10 (0.3 s), -80 (1 s), fin
  struct {
    uint32_t t;
    bool power;
    float ang;
    int attempt;
  } exp[] = {{990, true, SERVO_ANG_LIBERA_DEG, 1},   {1000, true, SERVO_ANG_BLOQUEO_DEG, 2},
             {1290, true, SERVO_ANG_BLOQUEO_DEG, 2}, {1300, true, SERVO_ANG_LIBERA_DEG, 2},
             {2300, true, SERVO_ANG_BLOQUEO_DEG, 3}, {2600, true, SERVO_ANG_LIBERA_DEG, 3},
             {3590, true, SERVO_ANG_LIBERA_DEG, 3},  {3600, false, SERVO_ANG_LIBERA_DEG, 3}};
  size_t k = 0;
  for (uint32_t t = 10; t <= 3700; t += 10) {
    s.step(t, false);
    if (k < sizeof exp / sizeof exp[0] && t == exp[k].t) {
      TEST_ASSERT_EQUAL_MESSAGE(exp[k].power, s.cmd().power, "energía");
      TEST_ASSERT_EQUAL_FLOAT(exp[k].ang, s.cmd().angle_deg);
      TEST_ASSERT_EQUAL(exp[k].attempt, s.attempts());
      k++;
    }
  }
  TEST_ASSERT_EQUAL(sizeof exp / sizeof exp[0], k);
  TEST_ASSERT_TRUE(s.done());
  TEST_ASSERT_FALSE(s.confirmed());
  // Con confirmación en el primer intento
  ServoSequencer c;
  c.start(0);
  c.step(400, true);
  TEST_ASSERT_TRUE(c.done());
  TEST_ASSERT_TRUE(c.confirmed());
  TEST_ASSERT_EQUAL(1, c.attempts());
  TEST_ASSERT_FALSE(c.cmd().power);
}

static void test_servo_pulso() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f, servoPulseUs(0));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f - 10 * 1000.0f / 90, servoPulseUs(-10));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1500.0f - 80 * 1000.0f / 90, servoPulseUs(-80));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 500.0f, servoPulseUs(-200));  // saturación
}

// ---------------------------------------------------------------------------
// Modo demo
// ---------------------------------------------------------------------------
static void test_demo() {
  DemoSeparation d;
  d.step(0, false);
  TEST_ASSERT_FALSE(d.cmd().power);  // sin armar: sin energía
  d.arm(1000);
  uint32_t t_fire = 0;
  for (uint32_t t = 1000; t <= 1000 + DEMO_DELAY_S * 1000 + 100; t += 10) {
    d.step(t, false);
    if (!d.fired()) {
      TEST_ASSERT_TRUE(d.cmd().power);
      TEST_ASSERT_EQUAL_FLOAT(SERVO_ANG_BLOQUEO_DEG, d.cmd().angle_deg);
    }
    if (d.evFire()) t_fire = t;
  }
  TEST_ASSERT_EQUAL_UINT32(1000 + DEMO_DELAY_S * 1000, t_fire);
  TEST_ASSERT_EQUAL_UINT32(t_fire, d.tFire());
  TEST_ASSERT_EQUAL_FLOAT(SERVO_ANG_LIBERA_DEG, d.cmd().angle_deg);
  // Confirmación y fin
  d.step(t_fire + 300, true);
  TEST_ASSERT_TRUE(d.seq().done());
  TEST_ASSERT_FALSE(d.cmd().power);
  TEST_ASSERT_TRUE(d.fired());
  // Desarmar antes de tiempo cancela
  DemoSeparation e;
  e.arm(0);
  e.step(5000, false);
  e.disarm();
  for (uint32_t t = 5000; t < 20000; t += 10) e.step(t, false);
  TEST_ASSERT_FALSE(e.fired());
  TEST_ASSERT_FALSE(e.cmd().power);
}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_servo_sin_energia_hasta_caida);
  RUN_TEST(test_despegue_congela_p0_quieto);
  RUN_TEST(test_despegue_requiere_3s);
  RUN_TEST(test_compuerta_espera_y_dispara);
  RUN_TEST(test_respaldo_160m_sintetico);
  RUN_TEST(test_sin_imu_no_hay_compuerta);
  RUN_TEST(test_no_go);
  RUN_TEST(test_reinicio_decide_por_vz);
  RUN_TEST(test_aterrizado_tasas_y_mision);
  RUN_TEST(test_descenso_fuera_de_rango);
  RUN_TEST(test_servo_reintentos);
  RUN_TEST(test_servo_pulso);
  RUN_TEST(test_demo);
  return UNITY_END();
}
