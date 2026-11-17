// Log binario en LittleFS: registros de 32 B (mismo formato que la telemetría).
// Solo se usa desde la tarea de comunicaciones (núcleo 0); no es reentrante.
#pragma once

#include <Arduino.h>

#include "packet.h"

namespace logger {
bool begin();
// Agrega un registro (al búfer de LittleFS). Devuelve false si el log está lleno o falló.
bool append(const TelemetryPacket &p);
// Fuerza la escritura a flash (llamar ~1 vez por segundo).
void flush();
size_t sizeBytes();
size_t capacityBytes();
// Imprime el log por Serial en hex, un registro por línea, entre marcas LOG_INICIO/LOG_FIN.
void dump(Stream &out);
void erase();
}  // namespace logger
