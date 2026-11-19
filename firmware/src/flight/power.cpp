#include "power.h"

#include "config.h"
#include "servo_hw.h"

// El ADC de la pila solo se usa si está activado y con pin confirmado (el #error de
// config.h exige el pin si BATT_ADC_ENABLED = 1, salvo con PERMITIR_PINES_PENDIENTES).
#if BATT_ADC_ENABLED && (PIN_BATT_ADC >= 0)
#define BATT_ADC 1
static_assert(BATT_DIVIDER_RATIO > 0.0f, "BATT_ADC_ENABLED = 1 requiere BATT_DIVIDER_RATIO > 0");
#else
#define BATT_ADC 0
#endif

namespace power {

void begin() {
  // Servo sin energía y señal con pull-down (servo_hw).
  servo_hw::begin();
#if BATT_ADC
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BATT_ADC, ADC_11db);
#endif
}

void servoPowerOff() { servo_hw::off(); }

float batteryVolts() {
#if BATT_ADC
  return analogReadMilliVolts(PIN_BATT_ADC) * 0.001f * BATT_DIVIDER_RATIO;
#else
  return 0.0f;
#endif
}

}  // namespace power
