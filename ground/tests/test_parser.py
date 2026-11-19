import gs_core as C
import telemetry as T


def _line(**kw):
    p = T.Packet(estado=5, secuencia=7, t_ms=12345, presion_pa=95000, temp_c=21.5,
                 altura_m=123.4, vz_ms=-10.2, banderas=0x07, bateria_v=0.0, **kw)
    return C.make_line(p, -48.5, 9.75), p


def test_linea_valida():
    line, p = _line()
    kind, rx = C.parse_line(line)
    assert kind == "packet"
    assert rx.pkt == T.decode(T.encode(p))
    assert (rx.rssi, rx.snr) == (-48.5, 9.75)


def test_tolera_crlf_espacios_y_minusculas():
    line, p = _line()
    for variante in (line + "\r\n", "  " + line + " \n", line.lower()):
        kind, rx = C.parse_line(variante)
        assert kind == "packet" and rx.pkt.altura_m == p.altura_m


def test_lineas_que_no_son_paquete():
    assert C.parse_line(None) == ("empty", None)
    assert C.parse_line("") == ("empty", None)
    assert C.parse_line("\r\n") == ("empty", None)
    assert C.parse_line("# radio OK, escuchando (0)") == ("info", "radio OK, escuchando (0)")
    assert C.parse_line("ERR,-7,-90.0,-3.50") == ("radio_error", (-7, -90.0, -3.5))


def test_invalidas_se_descartan_sin_excepcion():
    line, _ = _line()
    hexa = line.split(",")[0]
    malas = [
        "basura", "a,b", "1,2,3,4", hexa, hexa + ",x,1", hexa + ",1,", hexa + ",nan,1",
        hexa + ",inf,1", hexa[:-2] + ",-50,5", hexa + "00,-50,5", "ZZ" * 32 + ",-50,5",
        "ERR,x,1,2", "ERR,1", "\x00\xff", "," * 5,
    ]
    for m in malas:
        kind, _ = C.parse_line(m)
        assert kind == "invalid", m


def test_version_y_estado_invalidos():
    p = T.Packet(version=9, estado=1)
    assert C.parse_line(C.make_line(p, -50, 5))[0] == "invalid"
    p = T.Packet(estado=7)
    assert C.parse_line(C.make_line(p, -50, 5))[0] == "invalid"
