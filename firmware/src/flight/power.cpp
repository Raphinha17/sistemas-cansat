#include "power.h"

#include "config.h"

namespace power {

void begin() {
  servoPowerOff();
#if PIN_SERVO_SIGNAL >= 0
  // Señal del servo en bajo; el pull-down externo la mantiene así en el arranque.
  pinMode(PIN_SERVO_SIGNAL, OUTPUT);
  digitalWrite(PIN_SERVO_SIGNAL, LOW);
#endif
#if PIN_REED >= 0
  pinMode(PIN_REED, INPUT);
#endif
  // El enclavamiento (PIN_LATCH) no se maneja todavía: su lógica y polaridad las define
  // electrónica (PREGUNTAS P2). No se escribe nada en ese pin.
#if PIN_BAT_ADC >= 0
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BAT_ADC, ADC_11db);
#endif
}

void servoPowerOff() {
#if (PIN_SERVO_POWER >= 0) && defined(SERVO_POWER_ON_LEVEL)
  pinMode(PIN_SERVO_POWER, OUTPUT);
  digitalWrite(PIN_SERVO_POWER, SERVO_POWER_ON_LEVEL ? LOW : HIGH);
#endif
}

float batteryVolts() {
#if PIN_BAT_ADC >= 0
  if (BAT_DIVIDER_RATIO <= 0.0f) return 0.0f;
  return analogReadMilliVolts(PIN_BAT_ADC) * 0.001f * BAT_DIVIDER_RATIO;
#else
  return 0.0f;
#endif
}

}  // namespace power
