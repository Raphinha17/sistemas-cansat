// Energía: medición de la pila de 9 V, energía del servo (apagada) y enclavamiento.
// Sin máquina de estados todavía: el servo se mantiene SIN energía siempre (SPEC).
#pragma once

#include <Arduino.h>

namespace power {
void begin();
// Voltaje de la pila en V; 0 si el pin o el divisor no están confirmados.
float batteryVolts();
// Corta la energía del servo (estado seguro).
void servoPowerOff();
}  // namespace power
