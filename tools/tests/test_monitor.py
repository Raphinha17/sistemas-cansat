import monitor as M


def test_normalizar_mayusculas_y_espacios():
    assert M.normalizar("stat") == "STAT"
    assert M.normalizar("  modo b \r\n") == "MODO B"      # espacios internos tal cual
    assert M.normalizar("sim_repite  120") == "SIM_REPITE  120"


def test_normalizar_vacia_no_envia():
    for x in ("", "   ", "\n", "\t \r\n", None):
        assert M.normalizar(x) is None


def test_comando_salir():
    assert M.es_salida("/salir") and M.es_salida("  /SALIR ") and not M.es_salida("salir")
