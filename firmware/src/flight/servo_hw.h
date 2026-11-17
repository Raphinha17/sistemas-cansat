// Manejo físico del servo MG90S: energía (PIN_SERVO_POWER) y señal PWM (LEDC).
// La secuencia (cuándo y a qué ángulo) la decide servo_release / la máquina de estados.
//
// - Sin energía, la señal queda como entrada con pull-down interno (además del
//   pull-down externo): sin pulsos espurios y sin alimentar el servo por la señal.
// - Al encender: primero la señal PWM en el ángulo pedido y luego la energía, para que el
//   servo reciba pulsos válidos desde el primer instante. Al apagar, al revés.
// Solo se llama desde la tarea de control.
#pragma once

#include <Arduino.h>

#include "servo_release.h"

namespace servo_hw {
// Estado seguro: sin energía y señal con pull-down. Llamar lo antes posible en setup().
void begin();
// Aplica la orden del ciclo (energía y ángulo).
void apply(const ServoCmd &c);
void off();
bool powered();
}  // namespace servo_hw
