#include "telemetry.h"

#include "lora_setup.h"

namespace telemetry {

static SX1262 *radio = nullptr;
static volatile bool tx_done = false;
static bool busy = false;
static bool ready = false;
static uint32_t t_start = 0;
static uint32_t n_sent = 0, n_skipped = 0, n_err = 0;
static uint8_t buf[sizeof(TelemetryPacket)];

// El tiempo en aire de 32 B a SF7/BW250 es de ~30 ms; si no llega la interrupción
// en este plazo se da por perdida la transmisión.
static constexpr uint32_t kTxTimeoutMs = 500;

static void IRAM_ATTR onDio1() { tx_done = true; }

int16_t begin() {
  static SX1262 r(loraModule());
  radio = &r;
  const int16_t st = loraBegin(r);
  if (st == RADIOLIB_ERR_NONE) {
    r.setDio1Action(onDio1);
    ready = true;
  }
  return st;
}

bool send(const TelemetryPacket &p) {
  if (!ready) return false;
  poll();
  if (busy) {
    n_skipped++;
    return false;
  }
  memcpy(buf, &p, sizeof buf);
  tx_done = false;
  const int16_t st = radio->startTransmit(buf, sizeof buf);
  if (st != RADIOLIB_ERR_NONE) {
    n_err++;
    return false;
  }
  busy = true;
  t_start = millis();
  return true;
}

void poll() {
  if (!busy) return;
  if (tx_done) {
    tx_done = false;
    radio->finishTransmit();
    busy = false;
    n_sent++;
  } else if (millis() - t_start > kTxTimeoutMs) {
    radio->finishTransmit();
    busy = false;
    n_err++;
  }
}

uint32_t sent() { return n_sent; }
uint32_t skipped() { return n_skipped; }
uint32_t errors() { return n_err; }

}  // namespace telemetry
