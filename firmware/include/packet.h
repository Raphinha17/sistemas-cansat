// Paquete de telemetría de 32 B, little-endian. Ver docs/SPEC.md y protocol/packet.md.
// El ESP32-S3 es little-endian, por lo que el struct se envía tal cual.
#pragma once

#include <stddef.h>
#include <stdint.h>

#define PACKET_VERSION 1

// Estados
enum {
  ST_AUTOPRUEBA = 0,
  ST_TIERRA = 1,
  ST_ASCENSO = 2,
  ST_CAIDA = 3,
  ST_DESPLIEGUE = 4,
  ST_DESCENSO = 5,
  ST_ATERRIZADO = 6,
};

// Banderas
enum {
  FLAG_BASE_LISTA = 1 << 0,
  FLAG_LIBERACION = 1 << 1,
  FLAG_DISPARO = 1 << 2,
  FLAG_DISPARO_RESPALDO = 1 << 3,
  FLAG_ATERRIZADO = 1 << 4,
  FLAG_FALLO_BARO = 1 << 5,
  FLAG_FALLO_IMU = 1 << 6,
  FLAG_NO_GO = 1 << 7,
};

typedef struct __attribute__((packed)) {
  uint8_t version;      //  0
  uint8_t estado;       //  1
  uint16_t secuencia;   //  2
  uint32_t t_ms;        //  4
  uint32_t presion_Pa;  //  8
  int16_t temp_centi;   // 12  0.01 C
  int16_t altura_dm;    // 14  0.1 m
  int16_t vz_cms;       // 16  cm/s
  int16_t ax_mg;        // 18
  int16_t ay_mg;        // 20
  int16_t az_mg;        // 22
  int16_t roll_centi;   // 24  0.01 deg
  int16_t pitch_centi;  // 26
  int16_t yaw_centi;    // 28
  uint8_t banderas;     // 30
  uint8_t bateria_dv;   // 31  V*10
} TelemetryPacket;

static_assert(sizeof(TelemetryPacket) == 32, "TelemetryPacket debe medir 32 bytes");
static_assert(offsetof(TelemetryPacket, version) == 0, "offset version");
static_assert(offsetof(TelemetryPacket, estado) == 1, "offset estado");
static_assert(offsetof(TelemetryPacket, secuencia) == 2, "offset secuencia");
static_assert(offsetof(TelemetryPacket, t_ms) == 4, "offset t_ms");
static_assert(offsetof(TelemetryPacket, presion_Pa) == 8, "offset presion_Pa");
static_assert(offsetof(TelemetryPacket, temp_centi) == 12, "offset temp");
static_assert(offsetof(TelemetryPacket, altura_dm) == 14, "offset altura");
static_assert(offsetof(TelemetryPacket, vz_cms) == 16, "offset vz");
static_assert(offsetof(TelemetryPacket, ax_mg) == 18, "offset ax");
static_assert(offsetof(TelemetryPacket, ay_mg) == 20, "offset ay");
static_assert(offsetof(TelemetryPacket, az_mg) == 22, "offset az");
static_assert(offsetof(TelemetryPacket, roll_centi) == 24, "offset roll");
static_assert(offsetof(TelemetryPacket, pitch_centi) == 26, "offset pitch");
static_assert(offsetof(TelemetryPacket, yaw_centi) == 28, "offset yaw");
static_assert(offsetof(TelemetryPacket, banderas) == 30, "offset banderas");
static_assert(offsetof(TelemetryPacket, bateria_dv) == 31, "offset bateria");
