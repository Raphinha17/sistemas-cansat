#include "state_machine.h"

#include <math.h>

static uint16_t hzToMs(int hz) { return (uint16_t)(1000 / (hz > 0 ? hz : 1)); }

// ---------------------------------------------------------------------------
// Arranque
// ---------------------------------------------------------------------------
void StateMachine::begin(uint32_t t_ms) {
  *this = StateMachine();
  t_ = t_state_ = t_ms;
  state_ = ST_AUTOPRUEBA;
}

void StateMachine::beginRecovered(const MissionRecord &rec, uint32_t t_ms) {
  begin(t_ms);
  rec_ = rec;
  latched_ = rec.flags & (FLAG_LIBERACION | FLAG_DISPARO | FLAG_DISPARO_RESPALDO);
  h_max_ = rec.h_max_m;
  state_ = rec.state;
  if (state_ < ST_ASCENSO || state_ > ST_DESCENSO) state_ = ST_ASCENSO;  // registro raro
  recovering_ = true;
  t_recover_ = t_ms;
}

// ---------------------------------------------------------------------------
// Transiciones
// ---------------------------------------------------------------------------
void StateMachine::enter(uint8_t s, uint32_t t) {
  state_ = s;
  t_state_ = t;
  out_.ev_state_change = true;
  switch (s) {
    case ST_CAIDA:
      latched_ |= FLAG_LIBERACION;
      t_rel_ = t;
      t_hdisp_ = 0;
      break;
    case ST_DESCENSO:
      vz_sum_ = 0;
      vz_n_ = 0;
      t_blk_ = t;
      vz_fuera_ = false;
      impact_ = rest_ = false;
      break;
    case ST_ATERRIZADO:
      latched_ |= FLAG_ATERRIZADO;
      out_.ev_flush_log = true;
      rec_.mission = 0;  // misión terminada: un reinicio después arranca normal
      break;
    default:
      break;
  }
  if (s >= ST_ASCENSO) saveRecord(t);
}

void StateMachine::fire(uint32_t t, bool backup) {
  latched_ |= FLAG_DISPARO;
  if (backup) latched_ |= FLAG_DISPARO_RESPALDO;
  t_fire_ = t;
  out_.ev_fire = true;
  seq_.start(t);
  enter(ST_DESPLIEGUE, t);
}

void StateMachine::saveRecord(uint32_t t) {
  rec_.magic = kMissionMagic;
  rec_.state = state_;
  rec_.flags = latched_;
  rec_.h_max_m = h_max_;
  rec_seq_++;
  t_rec_save_ = t;
}

// ---------------------------------------------------------------------------
// Ciclo
// ---------------------------------------------------------------------------
void StateMachine::step(const SmInput &in) {
  t_ = in.t_ms;
  imu_ok_ = in.imu_ok;
  baro_ok_ = in.baro_ok;
  out_.ev_state_change = out_.ev_freeze_p0 = out_.ev_fire = out_.ev_flush_log = false;

  if (recovering_) {
    // Tras un reinicio en vuelo: el estado guardado se mantiene hasta que el Kalman
    // converge. En CAIDA el respaldo de altura mínima sigue activo.
    if (state_ == ST_CAIDA && baro_ok_ && in.h <= SM_H_RESPALDO_M) {
      recovering_ = false;
      fire(t_, true);
    } else if (since(t_recover_) >= SM_REINICIO_ESPERA_S) {
      recovering_ = false;
      decideRecovered(in);
    }
  } else {
    switch (state_) {
      case ST_AUTOPRUEBA:
        if (in.calib_done) enter(ST_TIERRA, t_);
        break;
      case ST_TIERRA:
        stepTierra(in);
        break;
      case ST_ASCENSO:
        stepAscenso(in);
        break;
      case ST_CAIDA:
        stepCaida(in);
        break;
      case ST_DESPLIEGUE:
        seq_.step(t_, imu_ok_ && in.acc_g > SERVO_CONFIRMA_G);
        if (seq_.done()) enter(ST_DESCENSO, t_);
        break;
      case ST_DESCENSO:
        stepDescenso(in);
        break;
      default:  // ATERRIZADO: estado final
        break;
    }
  }

  // --- Salidas ---
  out_.state = state_;
  uint8_t f = latched_;
  if (in.base_ready || rec_.mission) f |= FLAG_BASE_LISTA;
  if (!baro_ok_) f |= FLAG_FALLO_BARO;
  if (!imu_ok_) f |= FLAG_FALLO_IMU;
  if (!baro_ok_ && !imu_ok_) f |= FLAG_NO_GO;  // sin sensores no hay misión posible
  out_.flags = f;

  // Servo: sin energía hasta CAIDA; en CAIDA alimentado en bloqueo; en DESPLIEGUE la
  // secuencia; después, sin energía.
  ServoCmd sc;
  if (state_ == ST_CAIDA) {
    sc.power = true;
    sc.angle_deg = SERVO_ANG_BLOQUEO_DEG;
  } else if (state_ == ST_DESPLIEGUE) {
    sc = seq_.cmd();
  }
  out_.servo = sc;

  switch (state_) {
    case ST_ASCENSO:
      out_.tx_period_ms = hzToMs(TX_HZ_ASCENSO);
      break;
    case ST_CAIDA:
    case ST_DESPLIEGUE:
    case ST_DESCENSO:
      out_.tx_period_ms = hzToMs(TX_HZ_CAIDA);
      break;
    case ST_ATERRIZADO:
      out_.tx_period_ms = since(t_state_) < SM_ATERRIZADO_TX_S ? hzToMs(TX_HZ_ATERRIZADO)
                                                                : SM_BALIZA_PERIODO_MS;
      break;
    default:
      out_.tx_period_ms = hzToMs(TX_HZ_TIERRA);
      break;
  }
  out_.log_hz = (state_ >= ST_ASCENSO && state_ <= ST_DESCENSO) ? LOG_HZ : LOG_HZ_TIERRA;

  out_.h_max = h_max_;
  out_.h_disp = h_disp_;
  out_.vz_prom = vz_prom_;
  out_.vz_fuera_rango = vz_fuera_;
  out_.servo_confirmado = seq_.confirmed();
  out_.servo_intentos = (uint8_t)seq_.attempts();
  out_.lowg_descartadas = lowg_desc_;
  out_.recuperando = recovering_;
}

// ---------------------------------------------------------------------------
// 1 TIERRA: espera el despegue
// ---------------------------------------------------------------------------
void StateMachine::stepTierra(const SmInput &in) {
  // p0 de referencia: el último promedio con el CanSat quieto en el suelo.
  if (in.baro_ok && fabsf(in.vz) < SM_P0_QUIETO_VZ_MS && fabsf(in.h) < SM_P0_QUIETO_H_M) {
    p0_ref_ = in.p0_pa;
    has_p0_ref_ = true;
  }

  bool go = false;
  if (!in.baro_ok) {
    // Sin barómetro el despegue no se puede detectar (h y vz solo por integración del
    // acelerómetro derivan). Se pasa a ASCENSO de inmediato para dejar armada la
    // detección de liberación por IMU (SPEC, Fallas). Ver PREGUNTAS P20.
    go = true;
  } else if (in.h > SM_DESPEGUE_H_M && in.vz > SM_DESPEGUE_VZ_MS) {
    if (!climb_) {
      climb_ = true;
      t_climb_ = t_;
    }
    go = since(t_climb_) >= SM_DESPEGUE_T_S;
  } else {
    climb_ = false;
  }
  if (!go) return;

  out_.ev_freeze_p0 = true;
  out_.p0_freeze_pa = has_p0_ref_ ? p0_ref_ : in.p0_pa;
  rec_.mission = 1;
  rec_.p0_pa = out_.p0_freeze_pa;
  rec_.p0_valid = in.baro_ok ? 1 : 0;
  h_max_ = in.baro_ok ? in.h : 0.0f;
  lowg_ = pend_ = fast_ = false;
  enter(ST_ASCENSO, t_);
}

// ---------------------------------------------------------------------------
// 2 ASCENSO: altura máxima y detección de liberación
// ---------------------------------------------------------------------------
void StateMachine::stepAscenso(const SmInput &in) {
  if (in.baro_ok && in.h > h_max_) h_max_ = in.h;
  if (in.baro_ok && h_max_ > rec_.h_max_m + SM_NVS_DH_MAX_M &&
      since(t_rec_save_) >= SM_NVS_MIN_PERIODO_S)
    saveRecord(t_);

  bool rel = false;

  // (a) Baja g: |a| < 0.35 g durante 150 ms, confirmada por vz < -8 m/s en el siguiente
  // 1 s. La altura mínima se evalúa al inicio de la ventana de baja g.
  // Con barómetro muerto vz solo viene de integrar el acelerómetro (deriva): se usa el
  // cambio de vz desde el inicio de la ventana (PREGUNTAS P19).
  if (in.imu_ok) {
    if (in.acc_g < SM_LIB_LOWG_G) {
      if (!lowg_) {
        lowg_ = true;
        t_lowg_ = t_;
        h_lowg_ = in.h;
        vz_lowg_ = in.vz;
      }
      if (!pend_ && since(t_lowg_) >= SM_LIB_LOWG_T_S) {
        pend_ = true;
        t_pend_ = t_;
        h_pend_ = h_lowg_;
        vz_pend_ = vz_lowg_;
      }
    } else {
      lowg_ = false;
    }
    if (pend_) {
      const bool conf = in.baro_ok ? (in.vz < SM_LIB_CONFIRMA_VZ_MS)
                                   : (in.vz - vz_pend_ < SM_LIB_CONFIRMA_VZ_MS);
      const bool h_ok = !in.baro_ok || h_pend_ > SM_LIB_H_MIN_M;
      if (conf && h_ok) {
        rel = true;
      } else if (since(t_pend_) > SM_LIB_CONFIRMA_T_S) {
        pend_ = false;
        lowg_ = false;  // si la baja g sigue, se rearma con un nuevo inicio
        lowg_desc_++;
      }
    }
  } else {
    lowg_ = pend_ = false;
  }

  // (b) y (c) solo con barómetro (SPEC: baro muerto -> liberación solo por IMU).
  if (in.baro_ok) {
    if (in.vz < SM_LIB_VZ_RAPIDA_MS) {
      if (!fast_) {
        fast_ = true;
        t_fast_ = t_;
        h_fast_ = in.h;
      }
      if (since(t_fast_) >= SM_LIB_VZ_RAPIDA_T_S && h_fast_ > SM_LIB_H_MIN_M) rel = true;
    } else {
      fast_ = false;
    }
    if (h_max_ > SM_LIB_H_MIN_M && in.h < h_max_ - SM_LIB_BAJA_DESDE_MAX_M) rel = true;
  } else {
    fast_ = false;
  }

  if (rel) enter(ST_CAIDA, t_);
}

// ---------------------------------------------------------------------------
// 3 CAIDA: altura de disparo, compuerta de ángulo y respaldos
// ---------------------------------------------------------------------------
void StateMachine::stepCaida(const SmInput &in) {
  if (!in.baro_ok) {
    // Sin barómetro: disparo por tiempo desde la liberación.
    if (since(t_rel_) >= SM_T_DISPARO_SIN_BARO_S) fire(t_, true);
    return;
  }
  h_disp_ = SM_H_DISPARO_M + fabsf(in.vz) * SM_T_LATENCIA_S;
  if (in.h <= h_disp_) {
    if (!t_hdisp_) t_hdisp_ = t_ ? t_ : 1;
    // Compuerta de ángulo; sin IMU no hay actitud y no se aplica.
    const bool gate = !in.imu_ok || in.tilt_deg <= SM_COMPUERTA_DEG;
    if (gate) {
      fire(t_, false);
      return;
    }
    if ((uint32_t)(t_ - t_hdisp_) * 0.001f >= SM_COMPUERTA_RESPALDO_S) {
      fire(t_, true);
      return;
    }
  }
  if (in.h <= SM_H_RESPALDO_M) fire(t_, true);
}

// ---------------------------------------------------------------------------
// 5 DESCENSO: vz promedio y detección de aterrizaje
// ---------------------------------------------------------------------------
void StateMachine::stepDescenso(const SmInput &in) {
  vz_sum_ += in.vz;
  vz_n_++;
  if (since(t_blk_) >= SM_DESC_PROM_S) {
    vz_prom_ = (float)(vz_sum_ / vz_n_);
    const float v = -vz_prom_;
    vz_fuera_ = in.baro_ok && (v < SM_DESC_VZ_MIN_MS || v > SM_DESC_VZ_MAX_MS);
    vz_sum_ = 0;
    vz_n_ = 0;
    t_blk_ = t_;
  }

  // Impacto: pico de |a|, ignorando los primeros segundos (choque de la apertura).
  if (in.imu_ok && since(t_state_) >= SM_IMPACTO_ARMADO_S && in.acc_g > SM_IMPACTO_G)
    impact_ = true;

  // Reposo con lo que haya disponible. Con IMU: |a| ~ 1 g y sin giro (bajo el autogiro
  // |a| también es ~1 g, pero el cuerpo gira). Con barómetro: |vz| ~ 0.
  bool rest = false;
  if (in.imu_ok || in.baro_ok) {
    rest = true;
    if (in.imu_ok)
      rest = fabsf(in.acc_g - 1.0f) < SM_REPOSO_ACC_TOL_G && in.rate_dps < SM_REPOSO_GIRO_DPS;
    if (in.baro_ok) rest = rest && fabsf(in.vz) < SM_REPOSO_VZ_MS;
  }
  if (rest) {
    if (!rest_) {
      rest_ = true;
      t_rest_ = t_;
    }
  } else {
    rest_ = false;
  }
  const float need = impact_ ? SM_REPOSO_T_S : SM_REPOSO_SIN_IMPACTO_T_S;
  if (rest_ && since(t_rest_) >= need) enter(ST_ATERRIZADO, t_);
}

// ---------------------------------------------------------------------------
// Reinicio en vuelo: estado según lo guardado y vz actual
// ---------------------------------------------------------------------------
void StateMachine::decideRecovered(const SmInput &in) {
  switch (state_) {
    case ST_ASCENSO:
      if (in.baro_ok && in.h > h_max_) h_max_ = in.h;
      if (in.vz < SM_REINICIO_VZ_CAIDA_MS) {
        enter(ST_CAIDA, t_);  // la liberación ocurrió durante el reinicio
      } else {
        lowg_ = pend_ = fast_ = false;
        enter(ST_ASCENSO, t_);
      }
      break;
    case ST_CAIDA:
      // Se reinicia el reloj de la liberación (el anterior se perdió). Sin barómetro no
      // se sabe cuánto tiempo pasó: se dispara ya (más alto es más seguro).
      enter(ST_CAIDA, t_);
      if (!in.baro_ok) fire(t_, true);
      break;
    case ST_DESPLIEGUE:
      // No se sabe si el servo terminó: se repite la secuencia (girar a -80° otra vez
      // no hace daño).
      fire(t_, (latched_ & FLAG_DISPARO_RESPALDO) != 0);
      break;
    default:
      enter(ST_DESCENSO, t_);
      break;
  }
}

// ---------------------------------------------------------------------------
// Modo demo
// ---------------------------------------------------------------------------
void DemoSeparation::arm(uint32_t t_ms) {
  armed_ = true;
  fired_ = false;
  t_arm_ = t_ms;
  seq_.reset();
}

void DemoSeparation::disarm() {
  armed_ = false;
  seq_.reset();
}

void DemoSeparation::step(uint32_t t_ms, bool confirm) {
  ev_fire_ = false;
  if (!armed_) return;
  if (!fired_) {
    if ((uint32_t)(t_ms - t_arm_) >= (uint32_t)DEMO_DELAY_S * 1000u) {
      fired_ = true;
      ev_fire_ = true;
      t_fire_ = t_ms;
      seq_.start(t_ms);
    }
    return;
  }
  seq_.step(t_ms, confirm);
  if (seq_.done()) armed_ = false;  // terminado; queda fired_ hasta el próximo ARMAR
}

float DemoSeparation::remainingS(uint32_t t_ms) const {
  if (!armed_ || fired_) return 0;
  const float r = DEMO_DELAY_S - (uint32_t)(t_ms - t_arm_) * 0.001f;
  return r > 0 ? r : 0;
}

ServoCmd DemoSeparation::cmd() const {
  ServoCmd c;
  if (armed_ && !fired_) {
    c.power = true;  // armado: equivale al estado 3 (alimentado en bloqueo)
    c.angle_deg = SERVO_ANG_BLOQUEO_DEG;
  } else if (fired_) {
    c = seq_.cmd();
  }
  return c;
}
