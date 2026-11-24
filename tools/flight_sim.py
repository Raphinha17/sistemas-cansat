#!/usr/bin/env python3
r"""Simulador de vuelo sintético del CanSat: emite paquetes de 32 B (ver docs/SPEC.md).

Perfil (modo B, el lanzamiento real y el de por defecto): 3 s de autoprueba + 20 s de
espera en tierra, ascenso de dron a 3 m/s hasta 100 m, 20 s de estacionamiento a 100 m
(con ±0.5 m de oscilación de altura), suelta, caída libre con arrastre (velocidad
terminal ~35 m/s), despliegue del autogiro a ~80 m, descenso a ~10 m/s, aterrizaje y 60 s
de transmisión posterior. El modo A (reglamento anterior) sube a 6 m/s hasta 450 m,
espera 2 s y despliega a ~200 m.

La lógica de estados es una réplica simplificada de la SPEC que corre sobre las
señales *medidas* (con ruido), no sobre la verdad física, para que la salida se
parezca a lo que verá la estación terrena.

Ejemplos (Linux: python o .venv/bin/python; Windows: py o .\.venv\Scripts\python):
  python tools/flight_sim.py --hex                       # hex por stdout (modo B)
  python tools/flight_sim.py --out vuelo.bin --seed 3
  python tools/flight_sim.py --port /dev/ttyUSB1 --realtime     # Windows: --port COM5
  python tools/flight_sim.py --fail baro --fail-at 45 --loss 10 --hex

Modos de despliegue (DEPLOY_MODE del firmware; por defecto B, el lanzamiento real):
  --mode B                    suelta a 100 m tras 20 s de estacionamiento, autogiro a 80 m
                              (liberación válida > 85 m; detector rápido, ver MODES)
  --mode A                    liberación a 450 m, autogiro a 200 m (válida > 240 m)
  --mode C                    tinaco: sube a 8 m, se suelta y dispara al detectar la
                              caída libre (sin altura mínima, h_disp ni compuerta)
  --release-alt H             altura de liberación (sustituye la del modo; alias --h-top)
  --deploy-alt H              altura del autogiro (sustituye la del modo). La liberación
                              es válida por encima de deploy-alt + margen del modo
                              (A: 40 m, B: 5 m).
  --v-climb V                 velocidad de ascenso del dron (A: 6 m/s, B: 3 m/s)
  --t-hover T                 estacionamiento antes de soltar (A: 2 s, B: 20 s)
  --vib G                     vibración del acelerómetro en ascenso y estacionamiento
                              (g por eje; por defecto 0.06 en ascenso y 0.03 quieto)

Escenarios para las pruebas de la máquina de estados (los valores por defecto no
cambian el vuelo nominal):
  --mode B --release-alt 95|90|105   suelta a otra altura (todas válidas en B)
  --mode B --release-alt 80   suelta bajo el mínimo de 85 m: no hay liberación
  --mode B --false-release 70 0.25 s de caída libre del dron estacionado a 100 m
  --mode B --false-release 40 lo mismo durante el ascenso (~50 m)
  --mode B --v-climb 1.5 --t-hover 120 --vib 0.3   ascenso lento, estacionamiento largo
                              y vibración fuerte del dron
  --release-alt 200           modo A: el dron libera a 200 m (debajo del mínimo de 240 m)
  --false-release 70          modo A: 0.25 s de caída libre en el ascenso
  --fall-tilt 70              caída libre inclinada ~70° (compuerta de 45° no se cumple)
  --fall-tilt 60 --fall-tilt-until-h 214   inclinada hasta 214 m, luego se endereza
  --v-terminal 60             caída más rápida (respaldo de 160 m del modo A)
"""
from __future__ import annotations

import argparse
import math
import os
import random
import sys
import time
from typing import Callable, Iterator, Optional, Tuple

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "protocol"))
import telemetry as T  # noqa: E402

G = 9.80665
DT = 0.005                  # paso físico (200 Hz)
BARO_DT = 0.04              # barómetro/temperatura a 25 Hz

# Perfil
T_SELFTEST = 3.0
T_GROUND = 20.0
H_TOP = 450.0
V_CLIMB = 6.0
A_CLIMB = 1.5
T_HOVER = 2.0
V_TERMINAL = 35.0
V_CHUTE = 10.0
T_SERVO_DELAY = 0.3         # el servo empieza a mover el autogiro
T_OPEN = 0.6                # tiempo de apertura del autogiro
T_REST_LAND = 1.0           # reposo tras impacto antes de ATERRIZADO
T_POST_LAND = 60.0

# Parámetros de la SPEC (ver docs/SPEC.md). Las alturas y el detector son los del modo A;
# en general H_RELEASE_MIN = deploy_alt + release_margin y el respaldo es backup_alt (o
# _backup_alt() si no se da). Los del modo B están en MODES.
H_RELEASE_MIN = 240.0
H_DEPLOY = 200.0
RELEASE_MARGIN = 40.0
T_LAT = 0.5
H_BACKUP = 160.0
T_GATE_BACKUP = 1.5
ANGLE_GATE = 45.0
DROP_FROM_MAX = 25.0
T_FIRE_NO_BARO = 7.0        # disparo por tiempo desde la liberación (baro muerto)
T_DEPLOY_STATE = 1.0        # duración del estado 4 (sin validar el pico de aceleración)

T_LOWG = 0.15               # s de baja g (|a| < 0.35 g) para detectar la caída libre

# Modos de despliegue: valores por defecto de simulate() para --mode. Los umbrales del
# detector replican los de firmware/include/config.h (DEPLOY_B_*).
MODES = {
    "A": dict(h_top=H_TOP, deploy_alt=H_DEPLOY, t_fire_no_baro=T_FIRE_NO_BARO),
    "B": dict(h_top=100.0, deploy_alt=80.0, t_fire_no_baro=1.0,
              v_climb=3.0, t_hover=20.0, hover_wander=0.5, p0_hold=True,
              release_margin=5.0, backup_alt=70.0,
              lowg_t=0.10, confirm_vz=-5.0, confirm_t=0.6, fast_vz=-8.0, fast_t=0.2,
              drop_from_max=8.0, drop_vz=-5.0, gate_backup=0.3, late_fire_now=True),
    "C": dict(h_top=8.0, immediate=True),
}
DEFAULT_MODE = "B"

T_WANDER = 8.0              # s, periodo de la oscilación de altura en el estacionamiento
HOVER_WN = 1.5              # rad/s, control de altura del dron (estacionamiento dinámico)
HOVER_ZETA = 0.8
HOVER_A_MAX = 3.0           # m/s^2

FALSE_RELEASE_T = 0.25       # s de caída libre de la falsa liberación (--false-release)
FALL_TILT_SIG = 5.0         # ° de oscilación cuando se fuerza la inclinación (--fall-tilt)

TAU_W = 0.3                 # s, correlación de la velocidad angular simulada
RAW_DECIM = 2               # salida cruda a 200/2 = 100 Hz (frecuencia del lazo de control)
RAW_COLUMNS = ["t", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "p_pa", "temp_c",
               "baro_new", "h_true", "vz_true", "roll_true", "pitch_true", "yaw_true", "fase"]

RATES_HZ = {T.AUTOPRUEBA: 1, T.TIERRA: 1, T.ASCENSO: 2, T.CAIDA: 5,
            T.DESPLIEGUE: 5, T.DESCENSO: 5, T.ATERRIZADO: 2}


def _pressure(h: float, p0: float) -> float:
    return p0 * (1.0 - h / 44330.0) ** 5.255


def _alt_from_p(p: float, p0: float) -> float:
    return 44330.0 * (1.0 - (p / p0) ** (1.0 / 5.255))


def _backup_alt(deploy_alt: float) -> float:
    """Respaldo por altura si no se da backup_alt: 160 m en A (200 - 40)."""
    return deploy_alt - min(40.0, 0.2 * deploy_alt)


def simulate(seed: int = 1, fail: Optional[str] = None, fail_at: float = 30.0,
             rate: Optional[float] = None,
             raw_cb: Optional[Callable[[tuple], None]] = None,
             h_top: float = H_TOP, v_terminal: float = V_TERMINAL,
             false_release_at: Optional[float] = None,
             fall_tilt: Optional[float] = None,
             fall_tilt_until_h: float = 0.0,
             deploy_alt: float = H_DEPLOY,
             t_fire_no_baro: float = T_FIRE_NO_BARO,
             immediate: bool = False,
             v_climb: float = V_CLIMB, t_hover: float = T_HOVER,
             hover_wander: float = 0.0, vib_g: Optional[float] = None,
             p0_hold: bool = False,
             release_margin: float = RELEASE_MARGIN,
             backup_alt: Optional[float] = None,
             lowg_t: float = T_LOWG, confirm_vz: float = -8.0, confirm_t: float = 1.0,
             fast_vz: float = -12.0, fast_t: float = 0.4,
             drop_from_max: float = DROP_FROM_MAX, drop_vz: float = math.inf,
             gate_backup: float = T_GATE_BACKUP,
             late_fire_now: bool = False) -> Iterator[Tuple[float, T.Packet]]:
    """Genera (t, Packet) en el instante de cada transmisión (antes de aplicar pérdidas).

    Escenarios (por defecto, vuelo nominal):
      h_top: altura de liberación del dron.  v_terminal: velocidad terminal en caída libre.
      false_release_at: instante (s) de una caída libre breve (FALSE_RELEASE_T) en el ascenso.
      fall_tilt: inclinación (°, en roll) a la que tiende el cuerpo en caída libre, con
      oscilación reducida (FALL_TILT_SIG); se mantiene mientras h > fall_tilt_until_h.
      deploy_alt: altura del autogiro (liberación válida por encima de deploy_alt +
      release_margin).
      t_fire_no_baro: disparo por tiempo desde la liberación con el barómetro muerto.
      immediate: modo C (tinaco): disparo al detectar la caída libre, también en TIERRA.
      v_climb, t_hover: velocidad de ascenso y duración del estacionamiento antes de soltar.
      hover_wander: amplitud (m) de la oscilación de altura del dron estacionado; con > 0
      (o con una falsa liberación en el estacionamiento) el estacionamiento es dinámico
      (control de altura del dron).
      vib_g: vibración del acelerómetro (g por eje) en ascenso y estacionamiento.
      p0_hold: la réplica pausa el promedio de p0 con el CanSat fuera del suelo, como el
      firmware (SM_P0_PAUSA_MAX_S). El modo A conserva la réplica anterior para que sus
      datos no cambien; el firmware aplica la pausa en todos los modos.
      release_margin, backup_alt, lowg_t, confirm_vz, confirm_t, fast_vz, fast_t,
      drop_from_max, drop_vz, gate_backup, late_fire_now: detector y disparo de la
      réplica (los del modo B, en MODES, replican config.h).

    Si se da raw_cb, se llama a 100 Hz con una tupla en el orden de RAW_COLUMNS
    (sensores crudos + verdad física), útil para probar filtros del firmware.
    """
    rnd = random.Random(seed)
    gauss = rnd.gauss

    p0_true = 101325.0 + rnd.uniform(-600, 600)
    acc_bias = [gauss(0, 0.01) for _ in range(3)]
    h_release_min = deploy_alt + release_margin
    h_backup = _backup_alt(deploy_alt) if backup_alt is None else backup_alt
    k_fall = G / v_terminal ** 2
    k_chute = G / V_CHUTE ** 2
    launch_t = T_SELFTEST + T_GROUND

    # --- verdad física ---
    h = 0.0
    vz = 0.0
    phys = "ground"
    t_hover0 = t_rel = t_fire = t_impact = 0.0
    fired = False
    roll = pitch = yaw = 0.0
    tilt_rest = (rnd.uniform(-2, 2), rnd.uniform(-2, 2))
    roll, pitch = tilt_rest
    w_r = w_p = 0.0
    prev_att = (roll, pitch, yaw)
    gyro_bias = [gauss(0, 1.0) for _ in range(3)]  # °/s, lo estima el firmware al arrancar
    gyro = [0.0, 0.0, 0.0]
    baro_new = False
    f_spec = 1.0  # fuerza específica vertical en g

    # --- sensores / estimador ---
    p_meas = _pressure(0, p0_true)
    temp_meas = 25.0
    p0_est = None
    p0_n = 0
    p0_frozen = False
    est_h = est_vz = 0.0
    est_roll = est_pitch = est_yaw = 0.0
    a_meas = [0.0, 0.0, 1.0]
    baro_frozen_val = None
    baro_dead = imu_dead = False
    next_baro = 0.0

    # --- máquina de estados del firmware ---
    state = T.AUTOPRUEBA
    flags = 0
    t_state = 0.0
    h_max = 0.0
    climb_t0 = None
    lowg_t0 = None
    confirm_deadline = None
    fastfall_t0 = None
    ff_t0 = None  # inicio de la baja g del modo C
    t_rel_det = None
    hdisp_t0 = None
    fire_by_backup = False
    first_caida = False
    hover_dyn = False
    t_land_state = None
    seq = 0
    next_tx = 0.0

    t = 0.0
    n = 0
    while True:
        n += 1
        t = n * DT

        # ---------- fallas inyectadas ----------
        if fail == "baro" and t >= fail_at and not baro_dead:
            baro_dead = True
            baro_frozen_val = p_meas
            t_baro_dead = t
        if fail == "imu" and t >= fail_at:
            imu_dead = True
        if baro_dead and t >= t_baro_dead + 1.0:
            flags |= T.F_FALLO_BARO
        if imu_dead:
            flags |= T.F_FALLO_IMU

        # ---------- física ----------
        sig, tau = 1.0, 2.0
        yaw_rate = 0.0
        tilt_target = 0.0
        if phys == "ground":
            if t >= launch_t:
                phys = "ascent"
        if phys == "ascent":
            v_target = min(v_climb, math.sqrt(max(0.0, 2 * A_CLIMB * (h_top - h))))
            dv = max(-A_CLIMB * DT, min(A_CLIMB * DT, v_target - vz))
            if false_release_at is not None and false_release_at <= t < false_release_at + FALSE_RELEASE_T:
                dv = -G * DT  # caída libre breve (el dron cae y se recupera)
            a_up = dv / DT
            vz += dv
            h += vz * DT
            f_spec = 1.0 + a_up / G
            sig, tau, yaw_rate = 4.0, 1.5, 0.3
            if h >= h_top - 0.05 and vz < 0.05:
                h = h_top
                vz = 0.0
                phys = "hover"
                t_hover0 = t
        elif phys == "hover":
            in_false = (false_release_at is not None
                        and false_release_at <= t < false_release_at + FALSE_RELEASE_T)
            if in_false or hover_wander > 0:
                hover_dyn = True
            if hover_dyn:
                # Dron estacionado con control de altura: sigue h_top + oscilación; durante
                # una falsa liberación cae libre y luego se recupera.
                target = h_top + hover_wander * math.sin(2 * math.pi * (t - t_hover0) / T_WANDER)
                if in_false:
                    a_up = -G
                else:
                    a_up = HOVER_WN ** 2 * (target - h) - 2 * HOVER_ZETA * HOVER_WN * vz
                    a_up = max(-HOVER_A_MAX, min(HOVER_A_MAX, a_up))
                vz += a_up * DT
                h += vz * DT
                f_spec = 1.0 + a_up / G
            else:
                f_spec = 1.0
            sig, tau = 3.0, 1.5
            if t - t_hover0 >= t_hover:
                phys = "fall"
                t_rel = t
        elif phys == "fall":
            if fired and t >= t_fire + T_SERVO_DELAY:
                k = k_fall + (k_chute - k_fall) * min(1.0, (t - t_fire - T_SERVO_DELAY) / T_OPEN)
            else:
                k = k_fall
            a_true = -G - k * vz * abs(vz)
            f_spec = -k * vz * abs(vz) / G
            vz += a_true * DT
            h += vz * DT
            if fired and t >= t_fire + T_SERVO_DELAY:
                sig, tau, yaw_rate = 6.0, 1.0, 90.0
            else:
                sig, tau, yaw_rate = 30.0, 1.2, 40.0
                if fall_tilt is not None and h > fall_tilt_until_h:
                    sig, tilt_target = FALL_TILT_SIG, fall_tilt
            if h <= 0.0:
                h = 0.0
                vz = 0.0
                phys = "impact"
                t_impact = t
                tilt_rest = (rnd.uniform(-25, 25), rnd.uniform(-25, 25))
        elif phys == "impact":
            f_spec = 8.0 if t - t_impact < 0.03 else 1.0
            sig, tau = 0.0, 1.0
            roll += (tilt_rest[0] - roll) * 0.01
            pitch += (tilt_rest[1] - pitch) * 0.01

        if phys not in ("impact", "ground"):
            # Velocidad angular como proceso Ornstein-Uhlenbeck (tau_w) y ángulo que
            # regresa a 0 con tau: da actitud suave y un giroscopio físicamente coherente.
            # Desviación estándar del ángulo ~ sig.
            sw = sig / math.sqrt(TAU_W * tau)
            s = sw * math.sqrt(2 * DT / TAU_W)
            w_r += -w_r / TAU_W * DT + s * gauss(0, 1)
            w_p += -w_p / TAU_W * DT + s * gauss(0, 1)
            roll += (-(roll - tilt_target) / tau + w_r) * DT
            pitch += (-pitch / tau + w_p) * DT
        if phys != "ground":
            yaw += (yaw_rate + gauss(0, 0.3)) * DT
            yaw = (yaw + 180.0) % 360.0 - 180.0

        # ---------- sensores ----------
        r_, p_ = math.radians(roll), math.radians(pitch)
        vib = 0.01 if phys in ("ground", "impact") else (0.06 if phys == "ascent" else 0.03)
        if vib_g is not None and phys in ("ascent", "hover"):
            vib = vib_g
        f_body = f_spec
        a_true_b = [-math.sin(p_) * f_body,
                    math.sin(r_) * math.cos(p_) * f_body,
                    math.cos(r_) * math.cos(p_) * f_body]
        a_meas = [max(-16.0, min(16.0, a_true_b[i] + acc_bias[i] + gauss(0, vib + 0.004)))
                  for i in range(3)]
        est_roll = roll + gauss(0, 0.15)
        est_pitch = pitch + gauss(0, 0.15)
        est_yaw = yaw
        if imu_dead:
            a_meas = [0.0, 0.0, 0.0]
            est_roll = est_pitch = est_yaw = 0.0
        raw_tick = raw_cb is not None and n % RAW_DECIM == 0
        if raw_tick:
            # Giroscopio: velocidades de cuerpo a partir de las derivadas de Euler (ZYX)
            # en el intervalo desde la muestra cruda anterior.
            dtr = RAW_DECIM * DT
            d_r = (roll - prev_att[0]) / dtr
            d_p = (pitch - prev_att[1]) / dtr
            d_y = ((yaw - prev_att[2] + 180.0) % 360.0 - 180.0) / dtr
            mr = math.radians((roll + prev_att[0]) / 2)
            mp = math.radians((pitch + prev_att[1]) / 2)
            gyro = [d_r - d_y * math.sin(mp),
                    d_p * math.cos(mr) + d_y * math.cos(mp) * math.sin(mr),
                    -d_p * math.sin(mr) + d_y * math.cos(mp) * math.cos(mr)]
            gyro = [max(-2000.0, min(2000.0, gyro[i] + gyro_bias[i] + gauss(0, 0.1)))
                    for i in range(3)]
            if imu_dead:
                gyro = [0.0, 0.0, 0.0]
            prev_att = (roll, pitch, yaw)

        baro_tick = t >= next_baro
        if baro_tick:
            next_baro += BARO_DT
            temp_true = 25.0 - 6.5e-3 * h
            temp_meas = temp_true + gauss(0, 0.05)
            if baro_dead:
                p_meas = baro_frozen_val
            else:
                p_meas = round(_pressure(h, p0_true) + gauss(0, 1.5))
            baro_new = True
        if raw_tick:
            raw_cb((t, a_meas[0], a_meas[1], a_meas[2], gyro[0], gyro[1], gyro[2],
                    p_meas, temp_meas, int(baro_new), h, vz, roll, pitch, yaw, phys))
            baro_new = False

        # ---------- estimador (alfa-beta con aceleración) ----------
        if imu_dead:
            a_v = 0.0
        else:
            er, ep = math.radians(est_roll), math.radians(est_pitch)
            f_w = (-math.sin(ep) * a_meas[0] + math.sin(er) * math.cos(ep) * a_meas[1]
                   + math.cos(er) * math.cos(ep) * a_meas[2])
            a_v = (f_w - 1.0) * G
        est_h += est_vz * DT + 0.5 * a_v * DT * DT
        est_vz += a_v * DT
        if baro_tick and not baro_dead:
            if p0_est is None:
                p0_est = p_meas
            hold = (p0_hold and state == T.TIERRA and p0_n > 0
                    and not (abs(est_vz) < 0.5 and abs(est_h) < 3.0))
            if not p0_frozen and not hold:
                p0_n += 1
                p0_est += (p_meas - p0_est) / min(p0_n, 30.0 / BARO_DT)  # ~30 s de media móvil
            h_meas = _alt_from_p(p_meas, p0_est)
            if p0_n <= 1:
                est_h = h_meas
            r = h_meas - est_h
            est_h += 0.15 * r
            est_vz += 0.25 * r
        elif phys == "ground" and p0_est is None:
            pass

        # ---------- lógica de estados ----------
        t_state_age = t - t_state
        mag = math.sqrt(sum(a * a for a in a_meas))
        angle = math.degrees(math.acos(max(-1.0, min(1.0,
                math.cos(math.radians(est_roll)) * math.cos(math.radians(est_pitch))))))
        new_state = state

        # Modo C: la caída libre basta (sin altura mínima), en TIERRA o en ASCENSO.
        if immediate and state in (T.TIERRA, T.ASCENSO):
            if not imu_dead and mag < 0.35:
                ff_t0 = ff_t0 if ff_t0 is not None else t
                if t - ff_t0 >= T_LOWG:
                    new_state = T.CAIDA
                    flags |= T.F_LIBERACION
                    t_rel_det = t
                    p0_frozen = True
            else:
                ff_t0 = None

        if new_state != state:
            pass
        elif state == T.AUTOPRUEBA:
            if t >= T_SELFTEST:
                new_state = T.TIERRA
                flags |= T.F_BASE_LISTA
        elif state == T.TIERRA:
            if est_h > 20.0 and est_vz > 1.0:
                climb_t0 = climb_t0 if climb_t0 is not None else t
                if t - climb_t0 >= 3.0:
                    new_state = T.ASCENSO
                    p0_frozen = True
            else:
                climb_t0 = None
        elif state == T.ASCENSO:
            h_max = max(h_max, est_h)
            released = False
            if (est_h > h_release_min or baro_dead) and not immediate:
                if not imu_dead:
                    if mag < 0.35:
                        lowg_t0 = lowg_t0 if lowg_t0 is not None else t
                        if t - lowg_t0 >= lowg_t and confirm_deadline is None:
                            confirm_deadline = t + confirm_t
                    else:
                        lowg_t0 = None
                    if confirm_deadline is not None:
                        if est_vz < confirm_vz:
                            released = True
                        elif t > confirm_deadline:
                            confirm_deadline = None
                if est_vz < fast_vz:
                    fastfall_t0 = fastfall_t0 if fastfall_t0 is not None else t
                    if t - fastfall_t0 >= fast_t:
                        released = True
                else:
                    fastfall_t0 = None
                if not baro_dead and est_h < h_max - drop_from_max and est_vz < drop_vz:
                    released = True
            if released and t - launch_t > 5.0:
                new_state = T.CAIDA
                flags |= T.F_LIBERACION
                t_rel_det = t
                first_caida = True
        elif state == T.CAIDA:
            fire = False
            if immediate:
                fire = True
            elif baro_dead:
                if t - t_rel_det >= t_fire_no_baro:
                    fire, fire_by_backup = True, True
            else:
                h_disp = deploy_alt + abs(est_vz) * T_LAT
                if est_h <= h_disp:
                    hdisp_t0 = hdisp_t0 if hdisp_t0 is not None else t
                    gate = imu_dead or angle <= ANGLE_GATE or (first_caida and late_fire_now)
                    if gate:
                        fire = True
                    elif t - hdisp_t0 >= gate_backup:
                        fire, fire_by_backup = True, True
                first_caida = False
                if est_h <= h_backup and not fire:
                    fire, fire_by_backup = True, True
            if fire:
                new_state = T.DESPLIEGUE
                flags |= T.F_DISPARO | (T.F_DISPARO_RESPALDO if fire_by_backup else 0)
                fired = True
                t_fire = t
        elif state == T.DESPLIEGUE:
            if t_state_age >= T_DEPLOY_STATE:
                new_state = T.DESCENSO
        elif state == T.DESCENSO:
            if phys == "impact" and t - t_impact >= T_REST_LAND:
                new_state = T.ATERRIZADO
                flags |= T.F_ATERRIZADO
                t_land_state = t

        if new_state != state:
            state = new_state
            t_state = t

        # ---------- transmisión ----------
        if t >= next_tx:
            hz = rate if rate else RATES_HZ[state]
            next_tx = t + 1.0 / hz
            bat = 8.95 - 0.0004 * t + gauss(0, 0.02) - (0.08 if phys == "ascent" else 0.0)
            yield t, T.Packet(
                estado=state, secuencia=seq & 0xFFFF, t_ms=int(t * 1000),
                presion_pa=int(p_meas), temp_c=temp_meas,
                altura_m=est_h, vz_ms=est_vz,
                ax_g=a_meas[0], ay_g=a_meas[1], az_g=a_meas[2],
                roll_deg=est_roll, pitch_deg=est_pitch, yaw_deg=est_yaw,
                banderas=flags, bateria_v=round(bat, 1),
            )
            seq += 1

        if t_land_state is not None and t - t_land_state >= T_POST_LAND:
            return
        if t > 900:  # salvaguarda
            return


class _Sinks:
    def __init__(self, out, port, baud, hexmode, to_stdout):
        self.hex = hexmode
        self.files = []
        self.ser = None
        if out:
            self.files.append(open(out, "w" if hexmode else "wb"))
        if to_stdout:
            self.files.append(sys.stdout if hexmode else sys.stdout.buffer)
        if port:
            import serial  # pyserial, solo si se usa --port
            self.ser = serial.Serial(port, baud)

    def send(self, raw: bytes):
        for f in self.files:
            if self.hex:
                f.write(raw.hex().upper() + "\n")
            else:
                f.write(raw)
        if self.ser:
            self.ser.write(raw.hex().upper().encode() + b"\n" if self.hex else raw)

    def close(self):
        for f in self.files:
            f.flush()
            if f is not sys.stdout and f is not getattr(sys.stdout, "buffer", None):
                f.close()
        if self.ser:
            self.ser.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Simulador de vuelo CanSat (paquetes de 32 B)")
    ap.add_argument("--rate", type=float, help="Hz fijos; sin esto usa la frecuencia por estado")
    ap.add_argument("--loss", type=float, default=0.0, help="%% de paquetes perdidos")
    ap.add_argument("--fail", choices=["baro", "imu"], help="sensor que falla")
    ap.add_argument("--fail-at", type=float, default=30.0, help="s desde el inicio (def. 30)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--out", help="archivo de salida")
    ap.add_argument("--port", help="puerto serial (p. ej. /dev/ttyUSB1 o COM5)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--hex", action="store_true", help="una línea hex por paquete en vez de binario")
    ap.add_argument("--realtime", action="store_true", help="emite a ritmo de reloj real")
    ap.add_argument("--raw-csv", help="escribe sensores crudos + verdad a 100 Hz (CSV) para "
                    "probar los filtros del firmware; si no hay --out/--port/--hex, no emite paquetes")
    ap.add_argument("--mode", choices=sorted(MODES), default=DEFAULT_MODE,
                    help="modo de despliegue (def. B: 100 m -> 80 m, lanzamiento real; "
                         "A: 450 m -> 200 m; C: tinaco)")
    ap.add_argument("--release-alt", "--h-top", dest="h_top", type=float, default=None,
                    help="altura de liberación (def. la del modo: A 450 m, B 100 m, C 8 m)")
    ap.add_argument("--deploy-alt", type=float, default=None,
                    help="altura del autogiro (def. la del modo: A 200 m, B 80 m)")
    ap.add_argument("--v-climb", type=float, default=None,
                    help="velocidad de ascenso del dron (def. A 6 m/s, B 3 m/s)")
    ap.add_argument("--t-hover", type=float, default=None,
                    help="s de estacionamiento antes de soltar (def. A 2 s, B 20 s)")
    ap.add_argument("--vib", type=float, default=None,
                    help="vibración del acelerómetro en ascenso/estacionamiento (g por eje)")
    ap.add_argument("--v-terminal", type=float, default=V_TERMINAL,
                    help="velocidad terminal en caída libre (def. 35 m/s)")
    ap.add_argument("--false-release", type=float, metavar="T",
                    help="s: caída libre de %.2f s en el ascenso (falsa liberación)" % FALSE_RELEASE_T)
    ap.add_argument("--fall-tilt", type=float, metavar="DEG",
                    help="inclinación forzada en caída libre (roll, °)")
    ap.add_argument("--fall-tilt-until-h", type=float, default=0.0, metavar="H",
                    help="la inclinación forzada se mantiene mientras h > H (def. 0)")
    a = ap.parse_args(argv)

    if not 0 <= a.loss < 100:
        ap.error("--loss debe estar en [0, 100)")
    if a.rate is not None and a.rate <= 0:
        ap.error("--rate debe ser > 0")

    loss_rnd = random.Random(a.seed + 7919)
    raw_f = open(a.raw_csv, "w", newline="") if a.raw_csv else None
    raw_cb = None
    if raw_f:
        raw_f.write(",".join(RAW_COLUMNS) + "\n")

        def raw_cb(row):
            raw_f.write(",".join(x if isinstance(x, str) else f"{x:.6g}" if isinstance(x, float)
                                 else str(x) for x in row) + "\n")
    to_stdout = not (a.out or a.port) and (not a.raw_csv or a.hex)
    sinks = _Sinks(a.out, a.port, a.baud, a.hex, to_stdout=to_stdout)
    sent = dropped = 0
    t0 = time.monotonic()
    try:
        kw = dict(MODES[a.mode])
        if a.h_top is not None:
            kw["h_top"] = a.h_top
        if a.deploy_alt is not None:
            kw["deploy_alt"] = a.deploy_alt
        if a.v_climb is not None:
            kw["v_climb"] = a.v_climb
        if a.t_hover is not None:
            kw["t_hover"] = a.t_hover
        if a.vib is not None:
            kw["vib_g"] = a.vib
        for t, pkt in simulate(a.seed, a.fail, a.fail_at, a.rate, raw_cb,
                               v_terminal=a.v_terminal, false_release_at=a.false_release,
                               fall_tilt=a.fall_tilt, fall_tilt_until_h=a.fall_tilt_until_h,
                               **kw):
            if loss_rnd.random() * 100 < a.loss:
                dropped += 1
                continue
            if a.realtime:
                delay = t - (time.monotonic() - t0)
                if delay > 0:
                    time.sleep(delay)
            sinks.send(T.encode(pkt))
            sent += 1
    except BrokenPipeError:
        pass
    except OSError as e:
        if e.errno != 22:  # EINVAL: tubería cerrada (p. ej. `| head`) en Windows
            raise
    finally:
        try:
            sinks.close()
        except OSError:
            pass
        if raw_f:
            raw_f.close()
    print(f"paquetes enviados={sent} perdidos={dropped}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
