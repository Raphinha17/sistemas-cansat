#include "state_machine.h"

#include <math.h>

static uint16_t hzToMs(int hz) { return (uint16_t)(1000 / (hz > 0 ? hz : 1)); }

// ---------------------------------------------------------------------------
// Modos de despliegue
// ---------------------------------------------------------------------------
DeployParams deployParams(DeployMode m) {
  DeployParams p;
  switch (m) {
    case DEPLOY_MODE_B:
      p.deploy_alt_m = DEPLOY_ALT_B_M;
      p.release_min_m = DEPLOY_ALT_B_M + DEPLOY_LIB_MARGEN_B_M;
      p.backup_alt_m = DEPLOY_RESPALDO_B_M;
      p.t_fire_no_baro_s = DEPLOY_T_SIN_BARO_B_S;
      p.lowg_t_s = DEPLOY_B_LOWG_T_S;
      p.confirm_vz_ms = DEPLOY_B_CONFIRMA_VZ_MS;
      p.confirm_t_s = DEPLOY_B_CONFIRMA_T_S;
      p.fast_vz_ms = DEPLOY_B_VZ_RAPIDA_MS;
      p.fast_t_s = DEPLOY_B_VZ_RAPIDA_T_S;
      p.drop_from_max_m = DEPLOY_B_BAJA_DESDE_MAX_M;
      p.drop_vz_ms = DEPLOY_B_BAJA_VZ_MS;
      p.gate_backup_s = DEPLOY_B_COMPUERTA_RESPALDO_S;
      p.late_fire_now = DEPLOY_B_DISPARO_TARDIO_INMEDIATO != 0;
      p.name = "lanzamiento real: suelta ~100 m";
      break;
    case DEPLOY_MODE_C:
      // Sin altura mínima, sin h_disp ni compuerta: no se usan las alturas.
      p.immediate = true;
      p.name = "tinaco";
      break;
    case DEPLOY_MODE_D:
      // Igual que C (misma detección de caída libre y mismos eventos), pero sin servo.
      p.immediate = true;
      p.no_servo = true;
      p.name = "tinaco: solo carga primaria, SIN disparo";
      break;
    default:  // A
      p.deploy_alt_m = DEPLOY_ALT_A_M;
      p.release_min_m = DEPLOY_ALT_A_M + DEPLOY_LIB_MARGEN_A_M;
      p.backup_alt_m = DEPLOY_RESPALDO_A_M;
      p.t_fire_no_baro_s = DEPLOY_T_SIN_BARO_A_S;
      p.name = "reglamento anterior: suelta ~450 m";
      break;
  }
  return p;
}

char deployModeChar(DeployMode m) {
  return m == DEPLOY_MODE_B ? 'B' : m == DEPLOY_MODE_C ? 'C' : m == DEPLOY_MODE_D ? 'D' : 'A';
}

bool deployModeFromChar(char c, DeployMode &out) {
  switch (c) {
    case 'A': case 'a': out = DEPLOY_MODE_A; return true;
    case 'B': case 'b': out = DEPLOY_MODE_B; return true;
    case 'C': case 'c': out = DEPLOY_MODE_C; return true;
    case 'D': case 'd': out = DEPLOY_MODE_D; return true;
    default: return false;
  }
}

DeployMode deployModeDefault() {
  DeployMode m = DEPLOY_MODE_B;
  deployModeFromChar(DEPLOY_MODE_DEFAULT, m);
  return m;
}

// ---------------------------------------------------------------------------
// Arranque
// ---------------------------------------------------------------------------
void StateMachine::begin(uint32_t t_ms) {
  // El modo (y sus umbrales) es configuración: sobrevive al reinicio lógico.
  const DeployMode m = mode_;
  const DeployParams p = prm_;
  *this = StateMachine();
  setDeployParams(m, p);
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
      caida_first_ = true;
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
  if (prm_.no_servo) {
    // Modo D: la máquina recorre CAIDA -> DESPLIEGUE -> DESCENSO como en C, pero sin
    // bandera de disparo, sin evento de disparo y sin arrancar la secuencia del servo.
    t_fire_ = t;
    enter(ST_DESPLIEGUE, t);
    return;
  }
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
    // converge. En CAIDA el respaldo de altura mínima sigue activo (y en el modo C, que
    // dispara sin condiciones, se dispara ya).
    if (state_ == ST_CAIDA && prm_.immediate) {
      recovering_ = false;
      fire(t_, false);
    } else if (state_ == ST_CAIDA && baro_ok_ && in.h <= prm_.backup_alt_m) {
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
        if (prm_.no_servo) {  // modo D: no hay secuencia que esperar
          enter(ST_DESCENSO, t_);
          break;
        }
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
  // Modo D: nunca hay energía ni orden al servo.
  ServoCmd sc;
  if (prm_.no_servo) {
    // sc queda sin energía
  } else if (state_ == ST_CAIDA) {
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
// 1 TIERRA: espera el despegue (en el modo C también la caída libre)
// ---------------------------------------------------------------------------
void StateMachine::stepTierra(const SmInput &in) {
  // p0 de referencia: el último promedio con el CanSat quieto en el suelo.
  if (in.baro_ok && fabsf(in.vz) < SM_P0_QUIETO_VZ_MS && fabsf(in.h) < SM_P0_QUIETO_H_M) {
    p0_ref_ = in.p0_pa;
    has_p0_ref_ = true;
  }

  // NO-GO: sin ningún sensor (IMU y barómetro muertos) no hay misión posible. Se queda en
  // TIERRA transmitiendo, sin pasar a ASCENSO ni guardar una misión en NVS (PREGUNTAS P44).
  // Con un solo sensor vivo todo sigue igual.
  if (!in.baro_ok && !in.imu_ok) {
    climb_ = false;
    return;
  }

  // Modo C (tinaco): la caída libre se detecta ya en TIERRA (el CanSat no sube los 20 m
  // del despegue). Pasa directo a CAIDA, que dispara en el ciclo siguiente.
  if (prm_.immediate && detectRelease(in)) {
    startMission(in);
    enter(ST_CAIDA, t_);
    return;
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

  startMission(in);
  enter(ST_ASCENSO, t_);
}

// Inicio de la misión (al salir de TIERRA): congela p0 y marca la misión en curso.
void StateMachine::startMission(const SmInput &in) {
  out_.ev_freeze_p0 = true;
  out_.p0_freeze_pa = has_p0_ref_ ? p0_ref_ : in.p0_pa;
  rec_.mission = 1;
  rec_.p0_pa = out_.p0_freeze_pa;
  rec_.p0_valid = in.baro_ok ? 1 : 0;
  h_max_ = in.baro_ok ? in.h : 0.0f;
  lowg_ = pend_ = fast_ = false;
}

// ---------------------------------------------------------------------------
// 2 ASCENSO: altura máxima y detección de liberación
// ---------------------------------------------------------------------------
void StateMachine::stepAscenso(const SmInput &in) {
  if (in.baro_ok && in.h > h_max_) h_max_ = in.h;
  if (in.baro_ok && h_max_ > rec_.h_max_m + SM_NVS_DH_MAX_M &&
      since(t_rec_save_) >= SM_NVS_MIN_PERIODO_S)
    saveRecord(t_);

  if (detectRelease(in)) enter(ST_CAIDA, t_);
}

// Criterios de liberación (SPEC, estado 3). La altura mínima y los tiempos son los del modo
// (DeployParams; A: DEPLOY_ALT + 40 m y los SM_LIB_*, B: DEPLOY_ALT + 5 m y DEPLOY_B_*).
// En el modo C no hay altura mínima y la caída libre basta.
bool StateMachine::detectRelease(const SmInput &in) {
  const float h_min = prm_.release_min_m;
  bool rel = false;

  // (a) Baja g: |a| < 0.35 g durante lowg_t_s (A: 150 ms, B: 100 ms), confirmada por
  // vz < confirm_vz_ms (A: -8, B: -5 m/s) en los confirm_t_s siguientes (A: 1 s, B: 0.6 s).
  // La altura mínima se evalúa al inicio de la ventana de baja g.
  // Con barómetro muerto vz solo viene de integrar el acelerómetro (deriva): se usa el
  // cambio de vz desde el inicio de la ventana (PREGUNTAS P19).
  // Modo C: los 150 ms de baja g bastan, sin confirmación por vz (PREGUNTAS P33).
  if (in.imu_ok) {
    if (in.acc_g < SM_LIB_LOWG_G) {
      if (!lowg_) {
        lowg_ = true;
        t_lowg_ = t_;
        h_lowg_ = in.h;
        vz_lowg_ = in.vz;
      }
      if (!pend_ && since(t_lowg_) >= prm_.lowg_t_s) {
        pend_ = true;
        t_pend_ = t_;
        h_pend_ = h_lowg_;
        vz_pend_ = vz_lowg_;
      }
    } else {
      lowg_ = false;
    }
    if (pend_) {
      const bool conf = in.baro_ok ? (in.vz < prm_.confirm_vz_ms)
                                   : (in.vz - vz_pend_ < prm_.confirm_vz_ms);
      const bool h_ok = !in.baro_ok || h_pend_ > h_min;
      if (prm_.immediate || (conf && h_ok)) {
        rel = true;
      } else if (since(t_pend_) > prm_.confirm_t_s) {
        pend_ = false;
        lowg_ = false;  // si la baja g sigue, se rearma con un nuevo inicio
        lowg_desc_++;
      }
    }
  } else {
    lowg_ = pend_ = false;
  }

  // (b) y (c) solo con barómetro (SPEC: baro muerto -> liberación solo por IMU).
  // (b) vz < fast_vz_ms durante fast_t_s (A: -12 m/s 0.4 s, B: -8 m/s 0.2 s).
  // (c) h < h_max - drop_from_max_m (A: 25 m; B: 8 m y además vz < -5 m/s, para que un
  //     dron que baja unos metros en estacionario no cuente).
  // En el modo C, (b) sin altura mínima (respaldo si el IMU murió) y (c) no aplica.
  if (in.baro_ok) {
    if (in.vz < prm_.fast_vz_ms) {
      if (!fast_) {
        fast_ = true;
        t_fast_ = t_;
        h_fast_ = in.h;
      }
      if (since(t_fast_) >= prm_.fast_t_s && (prm_.immediate || h_fast_ > h_min))
        rel = true;
    } else {
      fast_ = false;
    }
    if (!prm_.immediate && h_max_ > h_min && in.h < h_max_ - prm_.drop_from_max_m &&
        in.vz < prm_.drop_vz_ms)
      rel = true;
  } else {
    fast_ = false;
  }
  return rel;
}

// ---------------------------------------------------------------------------
// 3 CAIDA: altura de disparo, compuerta de ángulo y respaldos
// ---------------------------------------------------------------------------
void StateMachine::stepCaida(const SmInput &in) {
  if (prm_.immediate) {
    // Modo C: disparo inmediato, sin h_disp ni compuerta de ángulo.
    fire(t_, false);
    return;
  }
  if (!in.baro_ok) {
    // Sin barómetro: disparo por tiempo desde la liberación.
    if (since(t_rel_) >= prm_.t_fire_no_baro_s) fire(t_, true);
    return;
  }
  h_disp_ = prm_.deploy_alt_m + fabsf(in.vz) * SM_T_LATENCIA_S;
  const bool first = caida_first_;
  caida_first_ = false;
  if (in.h <= h_disp_) {
    if (!t_hdisp_) t_hdisp_ = t_ ? t_ : 1;
    // Liberación detectada tarde (ya por debajo de h_disp): cada ciclo cuenta, se dispara
    // sin esperar la compuerta (modo B).
    if (first && prm_.late_fire_now) {
      late_fired_ = true;
      fire(t_, false);
      return;
    }
    // Compuerta de ángulo; sin IMU no hay actitud y no se aplica.
    const bool gate = !in.imu_ok || in.tilt_deg <= SM_COMPUERTA_DEG;
    if (gate) {
      fire(t_, false);
      return;
    }
    if ((uint32_t)(t_ - t_hdisp_) * 0.001f >= prm_.gate_backup_s) {
      fire(t_, true);
      return;
    }
  }
  if (in.h <= prm_.backup_alt_m) fire(t_, true);
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

void DemoSeparation::toggle(uint32_t t_ms) {
  if (counting())
    disarm();  // segundo toque largo antes de terminar: cancela
  else
    arm(t_ms);
}

bool DemoSeparation::ledOn(uint32_t t_ms) const {
  if (counting()) return ((uint32_t)(t_ms - t_arm_) / DEMO_LED_PARPADEO_MS) % 2 == 0;
  return fired_;
}

// ---------------------------------------------------------------------------
// Toque largo del botón
// ---------------------------------------------------------------------------
bool LongPress::step(bool pressed, uint32_t t_ms) {
  if (!pressed) {
    down_ = done_ = false;
    return false;
  }
  if (!down_) {
    down_ = true;
    t_down_ = t_ms;
  }
  if (!done_ && (uint32_t)(t_ms - t_down_) >= hold_ms_) {
    done_ = true;  // una vez por pulsación
    return true;
  }
  return false;
}
