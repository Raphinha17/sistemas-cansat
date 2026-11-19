import struct

import pytest

import telemetry as T


def test_size_is_32():
    assert T.PACKET_SIZE == 32
    assert len(T.encode(T.Packet())) == 32


def test_roundtrip():
    p = T.Packet(estado=3, secuencia=65535, t_ms=123456, presion_pa=95500, temp_c=-12.34,
                 altura_m=245.7, vz_ms=-35.12, ax_g=0.123, ay_g=-0.456, az_g=1.0,
                 roll_deg=-45.67, pitch_deg=12.34, yaw_deg=179.99,
                 banderas=T.F_LIBERACION | T.F_NO_GO, bateria_v=8.7)
    q = T.decode(T.encode(p))
    assert q == p


def test_offsets():
    raw = T.encode(T.Packet(estado=5, secuencia=0x1234, t_ms=0xAABBCCDD, presion_pa=0x01020304,
                            banderas=0x81, bateria_v=9.0))
    assert raw[0] == T.VERSION and raw[1] == 5
    assert struct.unpack_from("<H", raw, 2)[0] == 0x1234
    assert struct.unpack_from("<I", raw, 4)[0] == 0xAABBCCDD
    assert struct.unpack_from("<I", raw, 8)[0] == 0x01020304
    assert raw[30] == 0x81 and raw[31] == 90


def test_saturation():
    q = T.decode(T.encode(T.Packet(altura_m=1e6, vz_ms=-1e6)))
    assert q.altura_m == 3276.7 and q.vz_ms == -327.68


def test_bad_length():
    with pytest.raises(ValueError):
        T.decode(b"\x00" * 31)


def test_log_event():
    raw = struct.pack("<BBBBI24x", T.LOG_EVENT_MARK, T.LOG_EV_MODO, ord("B"), 0, 123456)
    ev = T.decode_log_event(raw)
    assert ev == T.LogEvent(tipo=T.LOG_EV_MODO, valor=ord("B"), t_ms=123456)
    assert ev.modo == "B"
    # Un paquete normal no es evento
    assert T.decode_log_event(T.encode(T.Packet())) is None
    with pytest.raises(ValueError):
        T.decode_log_event(raw[:31])
