# Script previo de PlatformIO (entorno native): genera con tools/flight_sim.py los datos
# crudos de sensores a 100 Hz que usan las pruebas native (attitude, estimator, flight), y pasa la
# ruta al código C++ como SIM_DATA_DIR.
import os
import subprocess

Import("env")  # noqa: F821  (lo inyecta PlatformIO/SCons)

project_dir = env.subst("$PROJECT_DIR")  # noqa: F821
sim = os.path.normpath(os.path.join(project_dir, "..", "tools", "flight_sim.py"))
out_dir = os.path.join(env.subst("$PROJECT_WORKSPACE_DIR"), "simdata")  # noqa: F821
os.makedirs(out_dir, exist_ok=True)

# (nombre, argumentos extra del simulador). Los escenarios de la máquina de estados
# usan las opciones de flight_sim.py descritas en su encabezado.
CASES = [
    # Modo A (450 m -> 200 m, reglamento anterior)
    ("flight_seed1", ["--seed", "1", "--mode", "A"]),
    ("flight_seed7", ["--seed", "7", "--mode", "A"]),
    ("fail_baro", ["--seed", "1", "--mode", "A", "--fail", "baro", "--fail-at", "40"]),
    ("fail_imu", ["--seed", "1", "--mode", "A", "--fail", "imu", "--fail-at", "40"]),
    ("false_release", ["--seed", "1", "--mode", "A", "--false-release", "70"]),
    ("low_release", ["--seed", "1", "--mode", "A", "--release-alt", "200"]),
    ("gate_delay", ["--seed", "1", "--mode", "A", "--fall-tilt", "60", "--fall-tilt-until-h", "214"]),
    ("tilt_backup", ["--seed", "1", "--mode", "A", "--fall-tilt", "70"]),
    ("fast_fall", ["--seed", "1", "--mode", "A", "--fall-tilt", "70", "--v-terminal", "60"]),
    # Modo B (lanzamiento real: ascenso de dron a 3 m/s, 20 s estacionado a 100 m, suelta,
    # autogiro a 80 m)
    ("mode_b_seed1", ["--seed", "1", "--mode", "B"]),
    ("mode_b_seed7", ["--seed", "7", "--mode", "B"]),
    ("mode_b_95", ["--seed", "1", "--mode", "B", "--release-alt", "95"]),
    ("mode_b_90", ["--seed", "1", "--mode", "B", "--release-alt", "90"]),
    ("mode_b_105", ["--seed", "1", "--mode", "B", "--release-alt", "105"]),
    ("mode_b_low", ["--seed", "1", "--mode", "B", "--release-alt", "80"]),
    ("mode_b_tilt", ["--seed", "1", "--mode", "B", "--fall-tilt", "70"]),
    ("mode_b_false_hover", ["--seed", "1", "--mode", "B", "--false-release", "70"]),
    ("mode_b_false_climb", ["--seed", "1", "--mode", "B", "--false-release", "40"]),
    ("mode_b_slow", ["--seed", "1", "--mode", "B", "--v-climb", "1.5", "--t-hover", "120",
                     "--vib", "0.3"]),
    ("mode_b_fail_baro", ["--seed", "1", "--mode", "B", "--fail", "baro", "--fail-at", "45"]),
    ("mode_b_fail_imu", ["--seed", "1", "--mode", "B", "--fail", "imu", "--fail-at", "45"]),
    # Modo C (tinaco, 8 m)
    ("tinaco", ["--seed", "1", "--mode", "C"]),
]

for name, extra in CASES:
    out = os.path.join(out_dir, name + ".csv")
    args_file = out + ".args"
    args_txt = " ".join(extra)
    old_args = open(args_file).read() if os.path.exists(args_file) else None
    # Se regenera si falta, si el simulador cambió después de generarlo o si cambiaron
    # los argumentos del caso. (Las mismas órdenes generan los CSV de
    # docs/PRUEBAS_SISTEMAS.md: allí y aquí deben coincidir.)
    if (not os.path.exists(out) or os.path.getmtime(out) < os.path.getmtime(sim)
            or old_args != args_txt):
        print("gen_sim_data: generando", out)
        subprocess.check_call([env.subst("$PYTHONEXE"), sim, "--raw-csv", out] + extra)  # noqa: F821
        with open(args_file, "w") as f:
            f.write(args_txt)

env.Append(CPPDEFINES=[("SIM_DATA_DIR", '\\"%s/\\"' % out_dir.replace("\\", "/"))])  # noqa: F821
