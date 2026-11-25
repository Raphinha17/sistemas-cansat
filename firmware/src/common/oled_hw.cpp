#include "oled_hw.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <Wire.h>

static constexpr int W = 128, H = 64;
static Adafruit_SSD1306 disp(W, H, &Wire1, RST_OLED, 400000UL, 400000UL);
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

static void texto(int x, int y, const char *s, int tam = 1) {
  disp.setTextSize(tam);
  disp.setCursor(x, y);
  disp.print(s);
}

void drawFlight(const PantallaVuelo &p) {
  if (!ok) return;
  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  texto(0, 0, p.titulo);
  texto(0, 10, p.estado);
  // GO / NO-GO en grande; NO-GO en vídeo inverso para que se note
  const int w = (int)strlen(p.go) * 18;
  if (!p.go_ok) {
    disp.fillRect(0, 20, W, 26, SSD1306_WHITE);
    disp.setTextColor(SSD1306_BLACK);
  }
  texto((W - w) / 2, 22, p.go, 3);
  disp.setTextColor(SSD1306_WHITE);
  texto(0, 48, p.sensores);
  texto(0, 57, p.tx);
  disp.display();
}

void drawRx(const PantallaRx &p) {
  if (!ok) return;
  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  texto(0, 0, p.titulo);
  texto(0, 12, p.enlace);
  texto(0, 24, p.estado);
  texto(0, 36, p.altura);
  if (p.sin_enlace) {
    disp.fillRect(0, 50, W, 14, SSD1306_WHITE);
    disp.setTextColor(SSD1306_BLACK);
  }
  texto(2, 53, p.ultimo, 1);
  disp.display();
}

}  // namespace oled
