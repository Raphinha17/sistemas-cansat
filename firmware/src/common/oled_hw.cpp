#include "oled_hw.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

// Pines del variant (heltec_wifi_lora_32_V3 = los de la Wireless Stick V3)
static_assert(SDA_OLED == 17 && SCL_OLED == 18 && RST_OLED == 21 && Vext == 36,
              "pines de la OLED distintos de los esperados");
// La librería, con un panel de 64 de ancho, empieza en la columna 0x20 (= desplazamiento 32)
static_assert(OLED_ANCHO == 64 && OLED_ALTO == 32 && OLED_DESPLAZ_X == 32,
              "Adafruit SSD1306 solo aplica el desplazamiento de 32 columnas en un panel 64x32");
static Adafruit_SSD1306 disp(OLED_ANCHO, OLED_ALTO, &Wire1, RST_OLED, 400000UL, 400000UL);
static bool ok = false;

namespace oled {

bool begin() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  // LOW = alimenta la OLED
  delay(20);
  Wire1.begin(SDA_OLED, SCL_OLED, 400000UL);
  Wire1.setTimeOut(10);  // ms: una pantalla colgada no debe trabar el firmware
  // periphBegin = false: Wire1 ya está iniciado. begin() falla si el 0x3C no responde.
  ok = disp.begin(SSD1306_SWITCHCAPVCC, 0x3C, true, false);
  if (ok) {
    disp.clearDisplay();
    disp.display();
  }
  return ok;
}

bool present() { return ok; }

void off() {
  if (!ok) return;
  disp.clearDisplay();
  disp.display();
  disp.ssd1306_command(SSD1306_DISPLAYOFF);
}

void on() {
  if (!ok) return;
  disp.ssd1306_command(SSD1306_DISPLAYON);
}

void draw(const Pantalla &p) {
  if (!ok) return;
  disp.clearDisplay();
  for (uint8_t i = 0; i < p.n; i++) {
    const Texto &t = p.t[i];
    const int w = (int)strlen(t.s) * OLED_FUENTE_ANCHO * t.tam;
    if (t.invertido) {
      // Fondo blanco del ancho del texto (en SIN ENLACE / NO-GO: bien visible)
      const int x0 = (t.tam == 1 && t.x == 0) ? 0 : t.x - 1;
      disp.fillRect(x0 < 0 ? 0 : x0, t.y, w + 2 > OLED_ANCHO ? OLED_ANCHO : w + 2,
                    OLED_FUENTE_ALTO * t.tam, SSD1306_WHITE);
    }
    disp.setTextColor(t.invertido ? SSD1306_BLACK : SSD1306_WHITE);
    disp.setTextSize(t.tam);
    disp.setCursor(t.x, t.y);
    disp.print(t.s);
  }
  disp.display();
}

}  // namespace oled
