// Pantalla OLED SSD1306 128x64 de la Heltec V3 en un bus I2C propio (Wire1), separado del
// bus de sensores (pines 41 y 42). Pines y Vext salen del variant de la placa (SDA_OLED,
// SCL_OLED, RST_OLED, Vext). Si la pantalla no responde, begin() devuelve false y el resto
// de las funciones no hacen nada: el firmware sigue normal.
#pragma once

#include "indicadores.h"

namespace oled {
bool begin();                 // enciende Vext (LOW), inicia Wire1 y la pantalla. No bloquea.
bool present();
void off();                   // apaga el panel (comando DISPLAYOFF); no vuelve a escribir
void on();
void drawFlight(const PantallaVuelo &p);
void drawRx(const PantallaRx &p);
}  // namespace oled
