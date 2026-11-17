"""Codificación/decodificación del paquete de telemetría de 32 B (little-endian).

Ver docs/SPEC.md y protocol/packet.md.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, fields

PACKET_SIZE = 32
VERSION = 1
_FMT = "<BBHIIhhhhhhhhhBB"
assert struct.calcsize(_FMT) == PACKET_SIZE

# Estados
AUTOPRUEBA, TIERRA, ASCENSO, CAIDA, DESPLIEGUE, DESCENSO, ATERRIZADO = range(7)
STATE_NAMES = ["AUTOPRUEBA", "TIERRA", "ASCENSO", "CAIDA", "DESPLIEGUE", "DESCENSO", "ATERRIZADO"]

# Banderas
F_BASE_LISTA = 1 << 0
F_LIBERACION = 1 << 1
F_DISPARO = 1 << 2
F_DISPARO_RESPALDO = 1 << 3
F_ATERRIZADO = 1 << 4
F_FALLO_BARO = 1 << 5
F_FALLO_IMU = 1 << 6
F_NO_GO = 1 << 7


def _sat(v: float, lo: int, hi: int) -> int:
    return max(lo, min(hi, int(round(v))))


@dataclass
class Packet:
    """Valores en unidades físicas; la escala se aplica al codificar."""
    estado: int = 0
    secuencia: int = 0
    t_ms: int = 0
    presion_pa: int = 101325
    temp_c: float = 0.0
    altura_m: float = 0.0
    vz_ms: float = 0.0
    ax_g: float = 0.0
    ay_g: float = 0.0
    az_g: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    banderas: int = 0
    bateria_v: float = 9.0
    version: int = VERSION


def encode(p: Packet) -> bytes:
    return struct.pack(
        _FMT,
        p.version & 0xFF,
        p.estado & 0xFF,
        p.secuencia & 0xFFFF,
        p.t_ms & 0xFFFFFFFF,
        _sat(p.presion_pa, 0, 0xFFFFFFFF),
        _sat(p.temp_c * 100, -32768, 32767),
        _sat(p.altura_m * 10, -32768, 32767),
        _sat(p.vz_ms * 100, -32768, 32767),
        _sat(p.ax_g * 1000, -32768, 32767),
        _sat(p.ay_g * 1000, -32768, 32767),
        _sat(p.az_g * 1000, -32768, 32767),
        _sat(p.roll_deg * 100, -32768, 32767),
        _sat(p.pitch_deg * 100, -32768, 32767),
        _sat(p.yaw_deg * 100, -32768, 32767),
        p.banderas & 0xFF,
        _sat(p.bateria_v * 10, 0, 255),
    )


def decode(data: bytes) -> Packet:
    if len(data) != PACKET_SIZE:
        raise ValueError(f"se esperaban {PACKET_SIZE} bytes, llegaron {len(data)}")
    (ver, est, seq, t_ms, pres, temp, alt, vz, ax, ay, az,
     roll, pitch, yaw, flags, bat) = struct.unpack(_FMT, data)
    return Packet(
        version=ver, estado=est, secuencia=seq, t_ms=t_ms, presion_pa=pres,
        temp_c=temp / 100, altura_m=alt / 10, vz_ms=vz / 100,
        ax_g=ax / 1000, ay_g=ay / 1000, az_g=az / 1000,
        roll_deg=roll / 100, pitch_deg=pitch / 100, yaw_deg=yaw / 100,
        banderas=flags, bateria_v=bat / 10,
    )


def field_names() -> list[str]:
    return [f.name for f in fields(Packet)]
