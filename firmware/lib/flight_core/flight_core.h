// Núcleo del lazo de control, portable (sin Arduino): calibración del giroscopio,
// Mahony, Kalman vertical, p0, detección de fallas de sensores y máquina de estados.
//
// El firmware de vuelo (src/flight/main.cpp) lo llama a CONTROL_HZ con las lecturas de
// los sensores (o con las líneas del modo simulación por serial), y las pruebas native
// lo llaman con los datos de tools/flight_sim.py: así se prueba el mismo código.
#pragma once

#include <stdint.h>

#include "attitude.h"
#include "config.h"
#include "estimator.h"
#include "servo_release.h"
#include "state_machine.h"
#include "telemetry_pack.h"

// Entrada de un ciclo (periodo 1/CONTROL_HZ).
struct CoreInput {
  uint32_t t_ms = 0;
  bool imu_valid = false;  // false = la lectura del IMU falló en este ciclo
  float acc_g[3] = {0, 0, 0};
  float gyro_dps[3] = {0, 0, 0};
  bool baro_sampled = false;  // en este ciclo se intentó leer el barómetro (25 Hz)
  bool baro_valid = false;    // la lectura fue válida
  float p_pa = 0;
  float temp_c = 0;
};

class FlightCore {
 public:
  FlightCore();
  // Arranque normal. imu/baro_present: resultado de la autoprueba (IDs de sensores).
  void begin(uint32_t t_ms, bool imu_present, bool baro_present);
  // Reinicio en vuelo (rec.mission == 1): sin recalibrar el giroscopio ni p0.
  void beginRecovered(uint32_t t_ms, bool imu_present, bool baro_present,
                      const MissionRecord &rec);
  void step(const CoreInput &in);

  // Modo de despliegue (A, B o C). Se conserva en begin()/beginRecovered().
  void setDeployMode(DeployMode m) { sm_.setDeployMode(m); }
  DeployMode deployMode() const { return sm_.deployMode(); }
  // Solo pruebas (ver StateMachine::setDeployParams).
  void setDeployParams(DeployMode m, const DeployParams &p) { sm_.setDeployParams(m, p); }

  // Modo DEMO_SEPARACION (solo lo activan los entornos heltec_demo y heltec_banco)
  void setDemo(bool on) { demo_on_ = on; }
  bool demoEnabled() const { return demo_on_; }
  // En el modo D (sin servo) el demo no arma: tampoco debe haber disparo por ahí.
  void demoArm(uint32_t t_ms) { if (demo_on_ && !sm_.deployParamsNow().no_servo) demo_.arm(t_ms); }
  void demoDisarm() { demo_.disarm(); }
  void demoToggle(uint32_t t_ms) {
    if (demo_on_ && !sm_.deployParamsNow().no_servo) demo_.toggle(t_ms);
  }
  const DemoSeparation &demo() const { return demo_; }

  // --- Salidas del último ciclo ---
  uint8_t state() const { return sm_.out().state; }
  uint8_t flags() const;
  ServoCmd servo() const;
  uint16_t txPeriodMs() const;
  uint16_t logHz() const;
  bool evFire() const { return sm_.out().ev_fire || demo_.evFire(); }
  bool evFlushLog() const { return sm_.out().ev_flush_log; }
  bool evStateChange() const { return sm_.out().ev_state_change; }
  // Paquete en unidades físicas (sin secuencia ni batería, que pone quien llama).
  TelemetryValues telemetry() const;
  // Registro para NVS (incluye el sesgo del giroscopio) y su contador de cambios.
  MissionRecord record() const;
  uint32_t recordSeq() const { return sm_.recordSeq(); }

  const StateMachine &sm() const { return sm_; }
  const Mahony &ahrs() const { return ahrs_; }
  const VerticalKF &kf() const { return kf_; }
  const BaseCal &base() const { return base_; }
  bool imuDead() const { return imu_dead_; }
  bool baroDead() const { return baro_dead_; }
  bool gyroReady() const { return gyro_ready_; }
  float tiltDeg() const { return tilt_; }
  float accMagG() const { return acc_mag_; }
  float rateDps() const { return rate_; }
  // Promedio de p0 en pausa (TIERRA con el CanSat fuera del suelo o moviéndose).
  bool p0Paused() const { return p0_paused_; }

 private:
  void reset(uint32_t t_ms, bool imu_present, bool baro_present);

  Mahony ahrs_;
  GyroBiasCal gyro_cal_;
  VerticalKF kf_;
  BaseCal base_;
  StateMachine sm_;
  DemoSeparation demo_;
  bool demo_on_ = false;

  bool imu_dead_ = false, baro_dead_ = false;
  bool gyro_ready_ = false, kf_init_ = false;
  int gyro_tries_ = 0, imu_fails_ = 0, baro_fails_ = 0;
  float gyro_bias_[3] = {0, 0, 0};
  float acc_now_[3] = {0, 0, 0};
  float last_p_ = -1;
  uint32_t last_p_change_ms_ = 0;
  float p_pa_ = 0, temp_c_ = 0;
  float tilt_ = 0, acc_mag_ = 1, rate_ = 0;
  uint32_t t_ms_ = 0;
  // Reinicio en vuelo
  bool recovered_ = false, att_init_pending_ = false;
  // Pausa del promedio de p0 en TIERRA
  bool p0_paused_ = false;
  uint32_t t_p0_quiet_ = 0;
};
