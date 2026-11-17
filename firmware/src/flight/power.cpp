#include "power.h"

#include "config.h"
#include "servo_hw.h"

namespace power {

void begin() {
  // Servo sin energía y señal con pull-down (servo_hw).
  servo_hw::begin();
#if PIN_REED >= 0
  // El reed solo se lee en el modo DEMO_SEPARACION (para armar). Pull según config.h.
  pinMode(PIN_REED, REED_PULLUP ? INPUT_PULLUP : INPUT);
#endif
  // El enclavamiento (PIN_LATCH) no se maneja todavía: su lógica y polaridad las define
  // electrónica (PREGUNTAS P2). No se escribe nada en ese pin.
#if PIN_BAT_ADC >= 0
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BAT_ADC, ADC_11db);
#endif
}

void servoPowerOff() { servo_hw::off(); }

float batteryVolts() {
#if PIN_BAT_ADC >= 0
  if (BAT_DIVIDER_RATIO <= 0.0f) return 0.0f;
  return analogReadMilliVolts(PIN_BAT_ADC) * 0.001f * BAT_DIVIDER_RATIO;
#else
  return 0.0f;
#endif
}

}  // namespace power
