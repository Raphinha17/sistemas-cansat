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


def test_crc_igual_al_de_la_placa():
    # Vector de referencia de CRC-16/CCITT-FALSE (simCrc16 en firmware/lib/flight_core)
    assert SS.add_crc("123456789") == "123456789*29B1"
    line = SS.add_crc(SS.format_line(_row()))
    body, crc = line.split("*")
    assert body == "SIM,1230,0.01,-0.02,1.003,0.5,-1.25,0.1,100912.5,24.87"
    assert len(crc) == 4 and int(crc, 16) >= 0
    assert SS.line_t_ms(line) == 1230


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
    # Cada línea con su CRC correcto
    assert all(SS.add_crc(x.split("*")[0]) == x for x in lines)
    # 10 ms por línea y longitud que cabe en el búfer de la placa (SIM_LINEA_MAX = 160)
    t = [int(x.split(",")[1]) for x in lines[:100]]
    assert all(b - a == 10 for a, b in zip(t, t[1:]))
    assert max(len(x) for x in lines) < 100
    # IMU muerto desde los 40 s: campos vacíos
    assert lines[-1].split(",")[2:8] == [""] * 6
    # --sin-crc: formato anterior
    out2 = tmp_path / "lineas_sin_crc.txt"
    assert SS.main(["--csv", str(raw), "--dry-run", "--sin-crc", "--out", str(out2)]) == 0
    assert out2.read_text().splitlines() == [x.split("*")[0] for x in lines]


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


class _LossyBoard:
    """Placa falsa con la lógica de SimLink (firmware/lib/flight_core/sim_input.cpp):
    acepta solo la línea esperada con CRC correcto, pide SIM_REPITE ante un hueco o una
    línea dañada y responde SIM_PROXIMA. `lose` son los t_ms que se pierden (o se dañan,
    si están en `damage`) la primera vez que se mandan."""

    def __init__(self, lose=(), damage=(), answer_proxima=True):
        self.lose, self.damage = set(lose), set(damage)
        self.answer = answer_proxima
        self.accepted, self.next, self.asked = [], None, None
        self.out = b""
        self.sent_lines = 0

    @property
    def in_waiting(self):
        return len(self.out)

    def read(self, n):
        b, self.out = self.out[:n], self.out[n:]
        return b

    def _ask(self):
        if self.next is not None and self.asked != self.next:
            self.asked = self.next
            self.out += f"SIM_REPITE {self.next}\n".encode()

    def write(self, data):
        for line in data.decode().splitlines():
            if line == "SIM_PROXIMA":
                if self.answer:
                    nxt = -1 if self.next is None else self.next
                    self.out += f"SIM_PROXIMA {nxt}\n".encode()
                continue
            self.sent_lines += 1
            t = SS.line_t_ms(line)
            if t in self.lose:
                self.lose.discard(t)
                if t in self.damage:
                    line = line.replace(",", ";", 1)  # llega dañada
                else:
                    continue                          # no llega
            body, _, crc = line.partition("*")
            if SS.add_crc(body) != line or not line.startswith("SIM,"):
                self._ask()
            elif self.next is None or t == self.next:
                self.accepted.append(t)
                self.next = t + 10
            elif t > self.next:
                self._ask()
            # t < next: repetida, se descarta


def _lines(n=300):
    return [SS.add_crc(f"SIM,{10 * (k + 1)},0,0,1,0,0,0,100000,25") for k in range(n)]


def test_send_reenvia_huecos_dañadas_y_la_ultima():
    lines = _lines()
    ts = [SS.line_t_ms(x) for x in lines]
    # Huecos sueltos y en bloque, una línea dañada y la última (solo la salva SIM_PROXIMA)
    lose = [500, 1200, 1210, 1220, 2000, ts[-1]]
    board = _LossyBoard(lose=lose, damage=[2000])
    res = SS.send(lines, board, speed=1000.0, wait_s=0.2)
    assert board.accepted == ts            # todas, en orden, una vez
    assert res.confirmado and res.proxima == ts[-1] + 10
    assert res.pedidos == 3 and res.retrocesos == 4   # 3 SIM_REPITE + 1 por SIM_PROXIMA
    assert res.enviadas == board.sent_lines > len(lines)


def test_send_sin_respuesta_no_confirma():
    lines = _lines(20)
    board = _LossyBoard(answer_proxima=False)  # firmware viejo: no conoce SIM_PROXIMA
    res = SS.send(lines, board, speed=1000.0, wait_s=0.01, max_consultas=3)
    assert not res.confirmado and res.proxima is None
    assert board.accepted == [SS.line_t_ms(x) for x in lines]


def test_send_sin_verificar():
    board = _LossyBoard(lose=[50])
    res = SS.send(_lines(10), board, speed=1000.0, verify=False)
    assert res.enviadas >= 10 and not res.confirmado
