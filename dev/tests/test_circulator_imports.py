#!/usr/bin/env python3
"""Importing tools.circulator must not import pymodbus or pyserial.

Two reasons this is a test and not a convention:

1. The tree has to import cleanly on a machine with no instrument attached --
   the lint run, the test suite, and ``--help`` all import every package.
2. **pyserial's presence is not harmless here.** The circulator sits behind an
   FTDI FT232R whose DTR line is capacitively coupled to /RESET, so opening its
   port resets the microcontroller. Keeping the vendor import inside the
   function that actually opens the port is what makes "import the package" and
   "touch the hardware" two separate, visible events.

The control case at the bottom is what makes this check real: it proves the
subprocess CAN see pymodbus, so a green result above means the package left it
alone rather than that the package was simply unavailable.

    python dev/tests/test_circulator_imports.py
"""
from __future__ import annotations

import ast
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

fails = 0

#: Anything whose top-level package is one of these must stay out of sys.modules.
VENDOR = ("serial", "pymodbus")

MODULES = [
    "tools.circulator",
    "tools.circulator.api",
    "tools.circulator.codec",
    "tools.circulator.safety",
    "tools.circulator.link",
    "tools.circulator.circulator",
    "tools.circulator.node",
]

PROBE = (
    "import sys\n"
    "before = sorted(m for m in sys.modules if m.split('.')[0] in %r)\n"
    "import importlib\n"
    "mod = importlib.import_module(%r)\n"
    "after = sorted(m for m in sys.modules if m.split('.')[0] in %r)\n"
    "print('NAME=' + mod.__name__)\n"
    "print('BEFORE=' + ','.join(before))\n"
    "print('AFTER=' + ','.join(after))\n"
)


def ok(condition: bool, message: str, detail: str = "") -> None:
    global fails
    print("[test] %s: %s%s" % ("PASS" if condition else "FAIL", message,
                               f"  ({detail})" if detail else ""))
    if not condition:
        fails += 1


def probe(module: str) -> tuple[bool, str]:
    code = PROBE % (VENDOR, module, VENDOR)
    result = subprocess.run([sys.executable, "-c", code], cwd=str(REPO),
                            capture_output=True, text=True, timeout=120)
    if result.returncode != 0:
        return False, (result.stderr or result.stdout).strip().splitlines()[-1][:90]
    leaked = ""
    for line in result.stdout.splitlines():
        if line.startswith("AFTER="):
            leaked = line[len("AFTER="):].strip()
    return (leaked == ""), leaked or "clean"


print("--- no vendor module is imported at module level ---")
for module in MODULES:
    clean, detail = probe(module)
    ok(clean, "import %s leaves pymodbus/pyserial unimported" % module, detail)

print("\n--- control: the subprocess really can see pymodbus ---")
control = subprocess.run(
    [sys.executable, "-c",
     "import sys, pymodbus\n"
     "print('SEEN=' + ','.join(sorted(m for m in sys.modules "
     "if m.split('.')[0] in ('pymodbus',))[:1]))\n"],
    cwd=str(REPO), capture_output=True, text=True, timeout=120)
seen = "SEEN=pymodbus" in control.stdout
ok(seen, "pymodbus IS installed and would show up if it were imported",
   (control.stdout or control.stderr).strip()[:70])
if not seen:
    print("      NOTE: without this control the checks above prove nothing.")

print("\n--- the deferred import sits INSIDE the function that opens the port ---")
# Parsed, not grepped: a docstring may name the vendor package (this one does,
# at length), so a text search would either fail on prose or pass on a real
# top-level import hidden further down.
LINK = REPO / "tools" / "circulator" / "link.py"
tree = ast.parse(LINK.read_text(encoding="utf-8"))
module_level: list[str] = []
nested: list[str] = []
for node in ast.walk(tree):
    if not isinstance(node, (ast.Import, ast.ImportFrom)):
        continue
    if isinstance(node, ast.Import):
        roots = [alias.name.split(".")[0] for alias in node.names]
    else:
        roots = [(node.module or "").split(".")[0]]
    for root in roots:
        if root in VENDOR:
            nested.append(root)
for node in tree.body:  # top level only
    if isinstance(node, ast.Import):
        module_level += [alias.name.split(".")[0] for alias in node.names]
    elif isinstance(node, ast.ImportFrom):
        module_level.append((node.module or "").split(".")[0])
leaked = sorted(set(module_level) & set(VENDOR))
ok(not leaked, "link.py has NO vendor import at module level", ",".join(leaked) or "clean")
ok(bool(nested), "and the real client IS reachable, from inside a function",
   ",".join(sorted(set(nested))))

print("\n%s" % ("ALL PASS" if fails == 0 else "%d FAILURE(S)" % fails))
sys.exit(1 if fails else 0)
