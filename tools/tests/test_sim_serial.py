import csv

import flight_sim as S
import sim_serial as SS


def _row(**kw):
    r = {"t": "1.23", "ax_g": "0.01", "ay_g": "-0.02", "az_g": "1.003", "gx_dps": "0.5",
         "gy_dps": "-1.25", "gz_dps": "0.1", "p_pa": "100912.5", "temp_c": "24.87"}
    r.update(kw)
    return r


def test_format_line():
    assert SS.format_line(_row()) == "SIM,1230,0.01,-0.02,1.003,0.5,-1.25,0.1,100912.5,24.87"


def test_imu_muerto_campos_vacios():
    z = {c: "0" for c in SS.IMU_COLS}
    line = SS.format_line(_row(**z))
    assert line == "SIM,1230,,,,,,,100912.5,24.87"
    assert len(line.split(",")) == 10


def test_dry_run_vuelo(tmp_path):
    raw = tmp_path / "raw.csv"
    assert S.main(["--raw-csv", str(raw), "--seed", "1", "--fail", "imu", "--fail-at", "40"]) == 0
    out = tmp_path / "lineas.txt"
    assert SS.main(["--csv", str(raw), "--dry-run", "--out", str(out)]) == 0
    lines = out.read_text().splitlines()
    rows = list(csv.DictReader(open(raw)))
    assert len(lines) == len(rows)
    assert all(len(x.split(",")) == 10 and x.startswith("SIM,") for x in lines)
    # 10 ms por línea y longitud que cabe en el búfer de la placa (SIM_LINEA_MAX = 160)
    t = [int(x.split(",")[1]) for x in lines[:100]]
    assert all(b - a == 10 for a, b in zip(t, t[1:]))
    assert max(len(x) for x in lines) < 100
    # IMU muerto desde los 40 s: campos vacíos
    assert lines[-1].split(",")[2:8] == [""] * 6


class _FakeSerial:
    """Puerto falso: guarda lo escrito y no devuelve nada."""

    def __init__(self):
        self.out = []

    def write(self, b):
        self.out.append(b.decode().strip())

    def readline(self):
        return b""


def test_prepare_manda_sim_inicio_primero():
    ser = _FakeSerial()
    assert SS.prepare(ser, "b", wait_s=0.0) == ["SIM_INICIO", "MODO B"]
    assert ser.out == ["SIM_INICIO", "MODO B"]
    ser = _FakeSerial()
    assert SS.prepare(ser, None, wait_s=0.0) == ["SIM_INICIO"]
