// Máquina de estados de vuelo (estados 0 a 6 de docs/SPEC.md). Código puro: sin
// hardware ni Arduino, compila en el entorno native. Todos los umbrales están en
// include/config.h (prefijo SM_).
//
// Entradas (SmInput): lo que ya producen el estimador y la detección de fallas
// (altura y vz del Kalman, inclinación de Mahony, |a|, |ω|, banderas de falla, p0).
// Salidas (SmOutput): estado, banderas del paquete, orden al servo, tasas de TX y log,
// y eventos de un ciclo (congelar p0, disparo, vaciar log).
//
// Persistencia (reinicio en vuelo): record() devuelve el registro para NVS y
// recordSeq() cambia cada vez que hay que guardarlo. Quien llama hace la escritura
// (fuera del lazo de control). Tras un reinicio, beginRecovered() recibe lo guardado.
#pragma once

#include <stdint.h>

#include "config.h"
#include "packet.h"
#include "servo_release.h"

static constexpr uint32_t kMissionMagic = 0x314E534D;  // "MSN1"

// Registro persistente de la misión (NVS). Plano y de tamaño fijo.
struct MissionRecord {
  uint32_t magic = kMissionMagic;
  uint8_t mission = 0;   // 1 = misión en curso (desde ASCENSO hasta ATERRIZADO)
  uint8_t state = ST_TIERRA;
  uint8_t flags = 0;     // banderas latcheadas (liberación, disparo, respaldo)
  uint8_t p0_valid = 0;  // p0 congelado con barómetro vivo
  float p0_pa = 0;
  float h_max_m = 0;
  float gyro_bias[3] = {0, 0, 0};  // lo llena FlightCore (no lo usa la máquina)
};

struct SmInput {
  uint32_t t_ms = 0;
  bool calib_done = false;  // autoprueba terminada (sesgo del giroscopio y 1.ª presión)
  bool imu_ok = true;       // false = IMU muerto
  bool baro_ok = true;      // false = barómetro muerto o congelado
  bool base_ready = false;  // ventana de p0 completa (bandera "base lista")
  float p0_pa = 0;          // p0 actual del promedio móvil
  float h = 0;              // m sobre p0 (Kalman)
  float vz = 0;             // m/s, positiva hacia arriba (Kalman)
  float tilt_deg = 0;       // ángulo entre el eje del cuerpo y la vertical
  float acc_g = 1;          // |a| en g
  float rate_dps = 0;       // |ω| en °/s (sin sesgo)
};

struct SmOutput {
  uint8_t state = ST_AUTOPRUEBA;
  uint8_t flags = 0;
  ServoCmd servo;
  uint16_t tx_period_ms = 1000;
  uint16_t log_hz = LOG_HZ_TIERRA;
  // Eventos (solo durante el ciclo en que ocurren)
  bool ev_state_change = false;
  bool ev_freeze_p0 = false;  // congelar p0 en p0_freeze_pa (al pasar a ASCENSO)
  float p0_freeze_pa = 0;
  bool ev_fire = false;       // disparo del servo (entrada a DESPLIEGUE)
  bool ev_flush_log = false;  // vaciar el log (entrada a ATERRIZADO)
  // Diagnóstico
  float h_max = 0;
  float h_disp = 0;             // altura de disparo calculada en CAIDA
  float vz_prom = 0;            // promedio de vz (1 s) en DESCENSO
  bool vz_fuera_rango = false;  // promedio fuera de 8..12 m/s (no cabe en el paquete)
  bool servo_confirmado = false;
  uint8_t servo_intentos = 0;
  uint16_t lowg_descartadas = 0;  // ventanas de baja g sin confirmación (falsas liberaciones)
  bool recuperando = false;       // esperando tras un reinicio en vuelo
};

class StateMachine {
 public:
  // Arranque normal: estado 0 (AUTOPRUEBA).
  void begin(uint32_t t_ms);
  // Reinicio en vuelo con la misión guardada en NVS (rec.mission == 1). Se queda en el
  // estado guardado SM_REINICIO_ESPERA_S y luego decide según vz (ver .cpp).
  void beginRecovered(const MissionRecord &rec, uint32_t t_ms);
  void step(const SmInput &in);

  const SmOutput &out() const { return out_; }
  uint8_t state() const { return state_; }
  const MissionRecord &record() const { return rec_; }
  uint32_t recordSeq() const { return rec_seq_; }
  // Instantes (ms) de eventos, 0 si no ocurrieron (para pruebas y log).
  uint32_t tRelease() const { return t_rel_; }
  uint32_t tHdisp() const { return t_hdisp_; }
  uint32_t tFire() const { return t_fire_; }

 private:
  void enter(uint8_t s, uint32_t t);
  void fire(uint32_t t, bool backup);
  void saveRecord(uint32_t t);
  void stepTierra(const SmInput &in);
  void stepAscenso(const SmInput &in);
  void stepCaida(const SmInput &in);
  void stepDescenso(const SmInput &in);
  void decideRecovered(const SmInput &in);
  float since(uint32_t t0) const { return (uint32_t)(t_ - t0) * 0.001f; }

  uint8_t state_ = ST_AUTOPRUEBA;
  uint8_t latched_ = 0;  // banderas latcheadas
  uint32_t t_ = 0, t_state_ = 0;
  SmOutput out_;
  MissionRecord rec_;
  uint32_t rec_seq_ = 0, t_rec_save_ = 0;
  bool imu_ok_ = true, baro_ok_ = true;

  // Reinicio en vuelo
  bool recovering_ = false;
  uint32_t t_recover_ = 0;

  // TIERRA
  bool climb_ = false;
  uint32_t t_climb_ = 0;
  bool has_p0_ref_ = false;
  float p0_ref_ = 0;

  // ASCENSO
  float h_max_ = 0;
  bool lowg_ = false, pend_ = false, fast_ = false;
  uint32_t t_lowg_ = 0, t_pend_ = 0, t_fast_ = 0;
  float h_lowg_ = 0, vz_lowg_ = 0, h_pend_ = 0, vz_pend_ = 0, h_fast_ = 0;
  uint16_t lowg_desc_ = 0;

  // CAIDA
  uint32_t t_rel_ = 0, t_hdisp_ = 0, t_fire_ = 0;
  float h_disp_ = 0;

  // DESPLIEGUE
  ServoSequencer seq_;

  // DESCENSO
  double vz_sum_ = 0;
  int vz_n_ = 0;
  uint32_t t_blk_ = 0;
  float vz_prom_ = 0;
  bool vz_fuera_ = false;
  bool impact_ = false, rest_ = false;
  uint32_t t_rest_ = 0;
};

// Modo DEMO_SEPARACION (video de aceptación): al armar, el servo se alimenta en -10° y
// se dispara DEMO_DELAY_S segundos después con la misma secuencia del vuelo.
// No toca la máquina de vuelo: quien la usa combina sus salidas (ver FlightCore).
class DemoSeparation {
 public:
  // Arma la cuenta regresiva (también re-arma tras un disparo, para repetir la toma).
  void arm(uint32_t t_ms);
  void disarm();
  void step(uint32_t t_ms, bool confirm);

  bool armed() const { return armed_; }
  bool fired() const { return fired_; }
  bool evFire() const { return ev_fire_; }
  uint32_t tFire() const { return t_fire_; }
  float remainingS(uint32_t t_ms) const;
  ServoCmd cmd() const;
  const ServoSequencer &seq() const { return seq_; }

 private:
  bool armed_ = false, fired_ = false, ev_fire_ = false;
  uint32_t t_arm_ = 0, t_fire_ = 0;
  ServoSequencer seq_;
};
