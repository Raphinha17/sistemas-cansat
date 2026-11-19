// Energía: medición de la pila de 9 V (opcional, BATT_ADC_ENABLED) y estado seguro del
// servo al arrancar. El servo lo maneja servo_hw según la máquina de estados.
// El encendido lo hace un interruptor deslizable en la línea de la pila: el firmware no
// lo lee.
#pragma once

#include <Arduino.h>

namespace power {
void begin();
// Voltaje de la pila en V; 0 si BATT_ADC_ENABLED = 0 (o sin pin/divisor).
float batteryVolts();
// Corta la energía del servo (estado seguro).
void servoPowerOff();
}  // namespace power
