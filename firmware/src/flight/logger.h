// Log binario en LittleFS: registros de 32 B (mismo formato que la telemetría), más
// registros de evento del mismo tamaño que empiezan con LOG_EVENTO_MARCA en lugar de la
// versión del paquete (ver protocol/packet.md, "Log").
// Solo se usa desde la tarea de comunicaciones (núcleo 0); no es reentrante.
#pragma once

#include <Arduino.h>

#include "packet.h"

#define LOG_EVENTO_MARCA 0xEE  // byte 0 de un registro de evento (version del paquete = 1)
enum : uint8_t {
  LOG_EV_ARRANQUE = 1,  // valor = modo de despliegue al arrancar ('A', 'B' o 'C')
  LOG_EV_MODO = 2,      // valor = nuevo modo de despliegue (comando MODO)
};

typedef struct __attribute__((packed)) {
  uint8_t marca;  // LOG_EVENTO_MARCA
  uint8_t tipo;   // LOG_EV_*
  uint8_t valor;
  uint8_t reservado;
  uint32_t t_ms;  // ms desde el arranque
  uint8_t relleno[24];
} LogEvento;
static_assert(sizeof(LogEvento) == sizeof(TelemetryPacket), "evento del log de 32 B");

namespace logger {
bool begin();
// Agrega un registro (al búfer de LittleFS). Devuelve false si el log está lleno o falló.
bool append(const TelemetryPacket &p);
// Agrega un registro de evento (p. ej. el modo de despliegue al arrancar o al cambiarlo).
bool appendEvent(uint8_t tipo, uint8_t valor, uint32_t t_ms);
// Fuerza la escritura a flash (llamar ~1 vez por segundo).
void flush();
size_t sizeBytes();
size_t capacityBytes();
// Imprime el log por Serial en hex, un registro por línea, entre marcas LOG_INICIO/LOG_FIN.
void dump(Stream &out);
void erase();
}  // namespace logger
