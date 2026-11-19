#include "logger.h"

#include <LittleFS.h>

namespace logger {

static const char *kPath = "/vuelo.bin";
static File f;
static bool ok = false;
static size_t written = 0;
static size_t capacity = 0;

bool begin() {
  // formatOnFail = true: si la partición no tiene sistema de archivos, se formatea.
  ok = LittleFS.begin(true);
  if (!ok) return false;
  // Deja ~10 % libre para metadatos de LittleFS.
  capacity = (LittleFS.totalBytes() * 9) / 10;
  f = LittleFS.open(kPath, FILE_APPEND);
  if (!f) {
    ok = false;
    return false;
  }
  written = f.size();
  return true;
}

bool append(const TelemetryPacket &p) {
  if (!ok || written + sizeof p > capacity) return false;
  const size_t n = f.write(reinterpret_cast<const uint8_t *>(&p), sizeof p);
  written += n;
  return n == sizeof p;
}

bool appendEvent(uint8_t tipo, uint8_t valor, uint32_t t_ms) {
  LogEvento e;
  memset(&e, 0, sizeof e);
  e.marca = LOG_EVENTO_MARCA;
  e.tipo = tipo;
  e.valor = valor;
  e.t_ms = t_ms;
  TelemetryPacket p;
  memcpy(&p, &e, sizeof p);
  return append(p);
}

void flush() {
  if (ok) f.flush();
}

size_t sizeBytes() { return written; }
size_t capacityBytes() { return capacity; }

void dump(Stream &out) {
  if (!ok) {
    out.println("LOG_ERROR sin LittleFS");
    return;
  }
  f.flush();
  File r = LittleFS.open(kPath, FILE_READ);
  out.println("LOG_INICIO");
  uint8_t b[sizeof(TelemetryPacket)];
  char hex[2 * sizeof b + 1];
  while (r && r.read(b, sizeof b) == sizeof b) {
    for (size_t i = 0; i < sizeof b; i++) snprintf(&hex[2 * i], 3, "%02X", b[i]);
    out.println(hex);
  }
  if (r) r.close();
  out.println("LOG_FIN");
}

void erase() {
  if (!ok) return;
  f.close();
  LittleFS.remove(kPath);
  f = LittleFS.open(kPath, FILE_APPEND);
  written = 0;
  ok = (bool)f;
}

}  // namespace logger
