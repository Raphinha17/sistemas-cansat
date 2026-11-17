// Enlace de telemetría LoRa (SX1262 vía RadioLib), sin bloqueo.
// Solo se usa desde la tarea de comunicaciones (núcleo 0).
#pragma once

#include <Arduino.h>

#include "packet.h"

namespace telemetry {
// Devuelve el código de RadioLib (0 = bien).
int16_t begin();
// Inicia la transmisión si el radio está libre. Si sigue ocupado con el paquete
// anterior, descarta este (cuenta en skipped) y regresa de inmediato.
bool send(const TelemetryPacket &p);
// Atiende el fin de transmisión; llamar seguido.
void poll();
uint32_t sent();
uint32_t skipped();
uint32_t errors();
}  // namespace telemetry
