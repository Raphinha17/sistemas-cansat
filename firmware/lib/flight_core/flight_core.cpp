#include "flight_core.h"

#include <math.h>

FlightCore::FlightCore()
    : ahrs_(MAHONY_KP, MAHONY_KI, MAHONY_ACC_GATE_G),
      gyro_cal_(GYRO_CAL_SAMPLES, GYRO_CAL_MAX_STD_DPS),
      kf_(KF_ACC_SIGMA, KF_BARO_SIGMA),
      base_(P0_WINDOW_S, (float)CONTROL_HZ / BARO_DECIM) {}

void FlightCore::reset(uint32_t t_ms, bool imu_present, bool baro_present) {
  ahrs_.reset();
  gyro_cal_.reset();
  kf_ = VerticalKF(KF_ACC_SIGMA, KF_BARO_SIGMA);
  base_ = BaseCal(P0_WINDOW_S, (float)CONTROL_HZ / BARO_DECIM);
  demo_.disarm();
  imu_dead_ = !imu_present;
  baro_dead_ = !baro_present;
  gyro_ready_ = imu_dead_;  // sin IMU no hay nada que calibrar
  kf_init_ = false;
  gyro_tries_ = imu_fails_ = baro_fails_ = 0;
  for (int i = 0; i < 3; i++) gyro_bias_[i] = acc_now_[i] = 0;
  last_p_ = -1;
  last_p_change_ms_ = t_ms;
  p_pa_ = temp_c_ = 0;
  tilt_ = rate_ = 0;
  acc_mag_ = 1;
  t_ms_ = t_ms;
  recovered_ = att_init_pending_ = false;
}

void FlightCore::begin(uint32_t t_ms, bool imu_present, bool baro_present) {
  reset(t_ms, imu_present, baro_present);
  sm_.begin(t_ms);
}

void FlightCore::beginRecovered(uint32_t t_ms, bool imu_present, bool baro_present,
                                const MissionRecord &rec) {
  reset(t_ms, imu_present, baro_present);
  recovered_ = true;
  // Sin recalibrar: el CanSat puede estar en movimiento. Se usa el sesgo guardado.
  if (!imu_dead_) {
    for (int i = 0; i < 3; i++) gyro_bias_[i] = rec.gyro_bias[i];
    gyro_ready_ = true;
    att_init_pending_ = true;
  }
  if (rec.p0_valid) base_.setFrozen(rec.p0_pa);
  sm_.beginRecovered(rec, t_ms);
}

void FlightCore::step(const CoreInput &in) {
  const float dt = 1.0f / CONTROL_HZ;
  t_ms_ = in.t_ms;

  // --- IMU ---
  if (!imu_dead_) {
    if (in.imu_valid) {
      for (int i = 0; i < 3; i++) acc_now_[i] = in.acc_g[i];
      acc_mag_ = sqrtf(acc_now_[0] * acc_now_[0] + acc_now_[1] * acc_now_[1] +
                       acc_now_[2] * acc_now_[2]);
      if (!gyro_ready_) {
        // Sesgo del giroscopio en reposo al arrancar
        if (gyro_cal_.add(in.gyro_dps[0], in.gyro_dps[1], in.gyro_dps[2])) {
          if (gyro_cal_.ok() || ++gyro_tries_ >= GYRO_CAL_MAX_TRIES) {
            // Tras GYRO_CAL_MAX_TRIES intentos con movimiento se usa la última media
            // (PREGUNTAS P9).
            for (int i = 0; i < 3; i++) gyro_bias_[i] = gyro_cal_.bias[i];
            ahrs_.initFromAccel(in.acc_g[0], in.acc_g[1], in.acc_g[2]);
            gyro_ready_ = true;
          } else {
            gyro_cal_.reset();
          }
        }
      } else {
        // Tras un reinicio en vuelo, roll/pitch se inicializan con el acelerómetro en
        // cuanto mide ~1 g (en ascenso o a velocidad terminal); mientras, solo giroscopio.
        if (att_init_pending_ && fabsf(acc_mag_ - 1.0f) < 0.3f) {
          ahrs_.initFromAccel(in.acc_g[0], in.acc_g[1], in.acc_g[2]);
          att_init_pending_ = false;
        }
        const float gx = in.gyro_dps[0] - gyro_bias_[0];
        const float gy = in.gyro_dps[1] - gyro_bias_[1];
        const float gz = in.gyro_dps[2] - gyro_bias_[2];
        ahrs_.update(gx, gy, gz, in.acc_g[0], in.acc_g[1], in.acc_g[2], dt);
        rate_ = sqrtf(gx * gx + gy * gy + gz * gz);
      }
      imu_fails_ = 0;
    } else if (++imu_fails_ > CONTROL_HZ) {
      imu_dead_ = true;  // más de 1 s sin lecturas válidas
    }
  }

  // --- Kalman: predicción con la aceleración vertical ---
  float a_up = 0.0f;
  if (!imu_dead_ && gyro_ready_) {
    a_up = (ahrs_.verticalSpecificForce(acc_now_[0], acc_now_[1], acc_now_[2]) - 1.0f) * kG;
  } else if (imu_dead_) {
    // Sin IMU: modelo de velocidad casi constante, con más ruido de proceso.
    kf_.setAccSigma(KF_ACC_SIGMA_SIN_IMU);
    for (float &a : acc_now_) a = 0.0f;  // no reportar la última lectura como vigente
    acc_mag_ = 0;
    rate_ = 0;
  }
  if (kf_init_) kf_.predict(a_up, dt);

  // --- Barómetro (25 Hz) ---
  if (!baro_dead_ && in.baro_sampled) {
    if (in.baro_valid) {
      p_pa_ = in.p_pa;
      temp_c_ = in.temp_c;
      // Congelado: valor idéntico durante más de 1 s (SPEC, Fallas)
      if (in.p_pa != last_p_) {
        last_p_ = in.p_pa;
        last_p_change_ms_ = in.t_ms;
      } else if ((uint32_t)(in.t_ms - last_p_change_ms_) > 1000) {
        baro_dead_ = true;
      }
      if (!baro_dead_) {
        base_.add(in.p_pa);  // no cambia p0 si está congelado
        const float h_baro = baroAltitude(in.p_pa, base_.p0());
        if (!kf_init_) {
          kf_.reset(h_baro, 0.0f);
          if (recovered_) kf_.P[1][1] = KF_P_VZ_REINICIO;  // vz desconocida
          kf_init_ = true;
        } else {
          kf_.updateBaro(h_baro);
        }
      }
      baro_fails_ = 0;
    } else if (++baro_fails_ > CONTROL_HZ / BARO_DECIM) {
      baro_dead_ = true;  // más de 1 s sin lecturas válidas
    }
  }
  if (baro_dead_ && !kf_init_) {
    kf_.reset(0.0f, 0.0f);  // solo IMU
    kf_init_ = true;
  }

  // --- Máquina de estados ---
  tilt_ = ahrs_.axisTiltDeg(BODY_AXIS_INDEX, BODY_AXIS_SIGN);
  SmInput si;
  si.t_ms = in.t_ms;
  si.calib_done = gyro_ready_ && (baro_dead_ || base_.hasValue());
  si.imu_ok = !imu_dead_;
  si.baro_ok = !baro_dead_;
  si.base_ready = base_.ready();
  si.p0_pa = base_.p0();
  si.h = kf_.h();
  si.vz = kf_.vz();
  si.tilt_deg = tilt_;
  si.acc_g = acc_mag_;
  si.rate_dps = rate_;
  sm_.step(si);
  if (sm_.out().ev_freeze_p0 && !baro_dead_) base_.setFrozen(sm_.out().p0_freeze_pa);

  if (demo_on_) demo_.step(in.t_ms, !imu_dead_ && acc_mag_ > SERVO_CONFIRMA_G);
}

// ---------------------------------------------------------------------------
// Salidas
// ---------------------------------------------------------------------------
static bool demoActive(const DemoSeparation &d) { return d.armed() || d.fired(); }

uint8_t FlightCore::flags() const {
  uint8_t f = sm_.out().flags;
  if (demo_on_ && demo_.fired()) f |= FLAG_DISPARO;  // instante de separación del demo
  return f;
}

ServoCmd FlightCore::servo() const {
  const ServoCmd s = sm_.out().servo;
  if (s.power || !demo_on_) return s;  // la máquina de vuelo tiene prioridad
  return demo_.cmd();
}

uint16_t FlightCore::txPeriodMs() const {
  uint16_t p = sm_.out().tx_period_ms;
  if (demo_on_ && demoActive(demo_)) {
    const uint16_t d = 1000 / TX_HZ_CAIDA;
    if (d < p) p = d;
  }
  return p;
}

uint16_t FlightCore::logHz() const {
  uint16_t h = sm_.out().log_hz;
  if (demo_on_ && demoActive(demo_) && h < LOG_HZ) h = LOG_HZ;
  return h;
}

TelemetryValues FlightCore::telemetry() const {
  TelemetryValues v;
  v.estado = state();
  v.t_ms = t_ms_;
  v.presion_pa = p_pa_;
  v.temp_c = temp_c_;
  v.altura_m = kf_.h();
  v.vz_ms = kf_.vz();
  for (int i = 0; i < 3; i++) v.acc_g[i] = acc_now_[i];
  v.roll_deg = ahrs_.roll();
  v.pitch_deg = ahrs_.pitch();
  v.yaw_deg = ahrs_.yaw();
  v.banderas = flags();
  return v;
}

MissionRecord FlightCore::record() const {
  MissionRecord r = sm_.record();
  for (int i = 0; i < 3; i++) r.gyro_bias[i] = gyro_bias_[i];
  return r;
}
