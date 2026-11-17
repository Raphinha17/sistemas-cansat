// Secuencia de liberación del autogiro con el servo MG90S. Código portable (sin
// Arduino): decide energía y ángulo; el manejo del pin lo hace src/flight/servo_hw.
//
// Reglas (docs/SPEC.md):
//  - Servo SIN energía hasta el estado 3 (CAIDA). En CAIDA se alimenta en la posición
//    de bloqueo (-10°) para que esté firme y listo; la máquina de estados decide eso.
//  - Disparo (estado 4): girar de -10° a -80°. Se espera confirmación (pico de
//    aceleración de la apertura) durante SERVO_CONFIRMA_T_S; si no llega, se reintenta
//    hasta SERVO_REINTENTOS veces: vuelve a -10° SERVO_T_RETORNO_S y de nuevo a -80°.
//  - Al terminar (confirmado o sin más reintentos) el servo queda sin energía.
//  - La señal tiene pull-down (externo + interno) mientras no hay energía, para que el
//    servo no reciba pulsos espurios ni se alimente por la señal.
#pragma once

#include <stdint.h>

#include "config.h"

// Orden al servo para un ciclo.
struct ServoCmd {
  bool power = false;                      // energía del servo
  float angle_deg = SERVO_ANG_BLOQUEO_DEG;  // ángulo pedido (solo vale con energía)
};

// Ancho de pulso (us) para un ángulo, con saturación a ±90°.
float servoPulseUs(float angle_deg);

class ServoSequencer {
 public:
  // Inicia el disparo: alimenta y gira a -80° (primer intento).
  void start(uint32_t t_ms);
  // Avanza la secuencia. confirm = pico de aceleración visto en este ciclo.
  void step(uint32_t t_ms, bool confirm);
  void reset() { phase_ = kIdle; }

  bool active() const { return phase_ == kRetorno || phase_ == kLibera; }
  bool done() const { return phase_ == kFin; }
  bool confirmed() const { return confirmed_; }
  int attempts() const { return attempt_; }  // 1 = primer intento, hasta 1 + REINTENTOS
  ServoCmd cmd() const;

 private:
  enum Phase : uint8_t { kIdle, kRetorno, kLibera, kFin };
  Phase phase_ = kIdle;
  uint32_t t_phase_ = 0;
  int attempt_ = 0;
  bool confirmed_ = false;
};
