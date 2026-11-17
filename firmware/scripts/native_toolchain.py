# Script previo de PlatformIO (entorno native): si no hay gcc/g++ en el PATH, usa el
# compilador de zig (paquete pip "ziglang", instalado en el .venv del proyecto) como
# compilador de C/C++ para las pruebas en PC. En Linux/macOS con gcc no cambia nada.
import os
import shutil
import subprocess
import sys

Import("env")  # noqa: F821  (lo inyecta PlatformIO/SCons)

NL = chr(10)
CRLF = chr(13) + chr(10)


def _find_zig():
    # Se pregunta al intérprete de PlatformIO (el del .venv), porque dentro de SCons
    # sys.path no incluye necesariamente site-packages.
    py = env.subst("$PYTHONEXE")  # noqa: F821
    try:
        out = subprocess.check_output(
            [py, "-c", "import ziglang, os; print(os.path.dirname(ziglang.__file__))"],
            text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return None
    exe = os.path.join(out, "zig.exe" if os.name == "nt" else "zig")
    return exe if os.path.isfile(exe) else None


if shutil.which("g++") is None:
    zig = _find_zig()
    if zig is None:
        sys.stderr.write("native_toolchain: no hay g++ ni ziglang; instala: pip install ziglang" + NL)
    else:
        zig = zig.replace(chr(92), "/")
        # El builder de la plataforma native fija CC/CXX = gcc/g++ después de este script,
        # así que en vez de reemplazar variables se crean "shims" gcc/g++/ar/ranlib que
        # llaman a zig y se ponen primero en el PATH de la compilación.
        ws = env.subst("$PROJECT_WORKSPACE_DIR")  # noqa: F821
        shim_dir = os.path.join(ws, "zig-shims")
        os.makedirs(shim_dir, exist_ok=True)
        for name, sub in (("gcc", "cc"), ("g++", "c++"), ("cc", "cc"), ("c++", "c++"),
                          ("ar", "ar"), ("ranlib", "ranlib")):
            if os.name == "nt":
                path = os.path.join(shim_dir, name + ".cmd")
                body = '@"%s" %s %%*' % (zig, sub) + CRLF
            else:
                path = os.path.join(shim_dir, name)
                body = "#!/bin/sh" + NL + 'exec "%s" %s "$@"' % (zig, sub) + NL
            if not os.path.exists(path) or open(path, newline="").read() != body:
                with open(path, "w", newline="") as f:
                    f.write(body)
                if os.name != "nt":
                    os.chmod(path, 0o755)
        env.PrependENVPath("PATH", shim_dir)  # noqa: F821
        # Caché de zig dentro del proyecto (.pio, ignorado por git)
        cache = os.path.join(ws, "zig-cache")
        env["ENV"]["ZIG_GLOBAL_CACHE_DIR"] = cache  # noqa: F821
        env["ENV"]["ZIG_LOCAL_CACHE_DIR"] = cache  # noqa: F821
        sys.stderr.write("native_toolchain: usando zig de %s%s" % (zig, NL))
