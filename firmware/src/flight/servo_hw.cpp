#include "servo_hw.h"

#include "config.h"

// Con pines pendientes (solo posible con PERMITIR_PINES_PENDIENTES) no se toca ningún pin.
// Con SERVO_POWER_ENABLED = 0 el servo tiene energía siempre: "encender" solo significa
// generar PWM, y "apagar" quitar el PWM y dejar la señal en entrada con pull-down.
#if (PIN_SERVO_SIGNAL >= 0) && \
    (!SERVO_POWER_ENABLED || ((PIN_SERVO_POWER >= 0) && (SERVO_POWER_ON_LEVEL >= 0)))
#define SERVO_HW 1
#else
#define SERVO_HW 0
#endif

namespace servo_hw {

static bool powered_ = false;
static float angle_ = 1000.0f;  // último ángulo escrito (valor imposible al inicio)

#if SERVO_HW
static uint32_t dutyFor(float angle_deg) {
  const float period_us = 1e6f / SERVO_PWM_HZ;
  return (uint32_t)(servoPulseUs(angle_deg) / period_us * ((1u << SERVO_PWM_BITS) - 1) + 0.5f);
}

static void pwmAttach() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(PIN_SERVO_SIGNAL, SERVO_PWM_HZ, SERVO_PWM_BITS, SERVO_LEDC_CANAL);
#else
  ledcSetup(SERVO_LEDC_CANAL, SERVO_PWM_HZ, SERVO_PWM_BITS);
  ledcAttachPin(PIN_SERVO_SIGNAL, SERVO_LEDC_CANAL);
#endif
}

static void pwmWrite(uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO_SIGNAL, duty);
#else
  ledcWrite(SERVO_LEDC_CANAL, duty);
#endif
}

static void pwmDetach() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcDetach(PIN_SERVO_SIGNAL);
#else
  ledcDetachPin(PIN_SERVO_SIGNAL);
#endif
  pinMode(PIN_SERVO_SIGNAL, INPUT_PULLDOWN);
}

static void powerPin(bool on) {
#if SERVO_POWER_ENABLED
  digitalWrite(PIN_SERVO_POWER, on ? SERVO_POWER_ON_LEVEL : !SERVO_POWER_ON_LEVEL);
#else
  (void)on;
#endif
}
#endif

void begin() {
#if SERVO_HW
  // Señal primero: sin PWM y con pull-down desde el arranque.
  pinMode(PIN_SERVO_SIGNAL, INPUT_PULLDOWN);
#if SERVO_POWER_ENABLED
  // Primero el nivel y luego el modo, para no dar un pulso de encendido.
  digitalWrite(PIN_SERVO_POWER, !SERVO_POWER_ON_LEVEL);
  pinMode(PIN_SERVO_POWER, OUTPUT);
  powerPin(false);
#endif
#endif
  powered_ = false;
  angle_ = 1000.0f;
}

void apply(const ServoCmd &c) {
  if (!c.power) {
    off();
    return;
  }
#if SERVO_HW
  if (!powered_) {
    pwmAttach();
    pwmWrite(dutyFor(c.angle_deg));
    powerPin(true);
  } else if (c.angle_deg != angle_) {
    pwmWrite(dutyFor(c.angle_deg));
  }
#endif
  angle_ = c.angle_deg;
  powered_ = true;
}

void off() {
  if (!powered_) return;
#if SERVO_HW
  powerPin(false);
  pwmDetach();
#endif
  powered_ = false;
  angle_ = 1000.0f;
}

bool powered() { return powered_; }

}  // namespace servo_hw
