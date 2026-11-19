"""Hace que las pruebas encuentren ground/, protocol/ y tools/."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
for d in (os.path.join(ROOT, "ground"), os.path.join(ROOT, "protocol"),
          os.path.join(ROOT, "tools")):
    if d not in sys.path:
        sys.path.insert(0, d)
