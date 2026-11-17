// Energía: medición de la pila de 9 V, estado seguro del servo al arrancar, reed y
// enclavamiento. El servo lo maneja servo_hw según la máquina de estados.
#pragma once

#include <Arduino.h>

namespace power {
void begin();
// Voltaje de la pila en V; 0 si el pin o el divisor no están confirmados.
float batteryVolts();
// Corta la energía del servo (estado seguro).
void servoPowerOff();
}  // namespace power
