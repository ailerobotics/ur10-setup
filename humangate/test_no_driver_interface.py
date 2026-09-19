#!/usr/bin/env python3
"""
Proof that this package is plan-only: it imports NONE of the robot-driver
interfaces (task brief item 5, report §10 Phase 1).

Two independent checks:
  1. STATIC (ast): no module in the package imports a driver message/service
     package (control_msgs, ur_msgs, ur_dashboard_msgs) or anything named like a
     dashboard client, and no code uses the FollowJointTrajectory /
     SetSpeedSliderFraction interfaces. Docstrings and comments are ignored
     (they legitimately NAME these interfaces to say the package avoids them).
  2. RUNTIME: importing every package module (with rclpy/std_msgs stubbed so no
     ROS stack is needed) never pulls a forbidden driver package into
     sys.modules.

Run standalone:  python3 test_no_driver_interface.py
Or under pytest: pytest test_no_driver_interface.py
"""
import ast
import glob
import importlib
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# Driver interfaces this task must never touch (report §2 mechanism table, §5).
FORBIDDEN_IMPORT_ROOTS = {"control_msgs", "ur_msgs", "ur_dashboard_msgs"}
FORBIDDEN_IDENTIFIERS = {"FollowJointTrajectory", "SetSpeedSliderFraction"}
DASHBOARD_HINT = "dashboard"

# The package's own modules (everything that is NOT a test).
PACKAGE_MODULES = sorted(
    p for p in glob.glob(os.path.join(HERE, "*.py"))
    if not os.path.basename(p).startswith("test_")
)


def _imported_names(tree):
    roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for a in node.names:
                roots.add(a.name.split(".")[0])
                roots.add(a.name)
        elif isinstance(node, ast.ImportFrom):
            if node.module:
                roots.add(node.module.split(".")[0])
                roots.add(node.module)
    return roots


def _used_identifiers(tree):
    ids = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Name):
            ids.add(node.id)
        elif isinstance(node, ast.Attribute):
            ids.add(node.attr)
    return ids


def test_no_driver_package_imports():
    assert PACKAGE_MODULES, "no package modules found to scan"
    for path in PACKAGE_MODULES:
        with open(path) as f:
            tree = ast.parse(f.read(), filename=path)
        imports = _imported_names(tree)
        bad = imports & FORBIDDEN_IMPORT_ROOTS
        assert not bad, f"{os.path.basename(path)} imports driver package(s): {bad}"
        dash = {m for m in imports if DASHBOARD_HINT in m.lower()}
        assert not dash, f"{os.path.basename(path)} imports a dashboard client: {dash}"


def test_no_driver_interface_identifiers_used():
    for path in PACKAGE_MODULES:
        with open(path) as f:
            tree = ast.parse(f.read(), filename=path)
        used = _used_identifiers(tree)
        bad = used & FORBIDDEN_IDENTIFIERS
        assert not bad, f"{os.path.basename(path)} uses driver interface(s): {bad}"


def _stub_ros():
    """Install minimal rclpy/std_msgs stand-ins so node import needs no ROS."""
    ros_pkgs = ["rclpy", "rclpy.node", "rclpy.qos", "std_msgs", "std_msgs.msg"]
    for name in ros_pkgs:
        mod = types.ModuleType(name)
        mod.__getattr__ = lambda attr: type(attr, (object,), {})
        sys.modules[name] = mod


def test_importing_package_pulls_no_driver_package():
    _stub_ros()
    for root in FORBIDDEN_IMPORT_ROOTS:
        sys.modules.pop(root, None)
    for path in PACKAGE_MODULES:
        name = os.path.splitext(os.path.basename(path))[0]
        if name.endswith("launch"):
            continue  # launch files import the launch DSL, not the gate runtime
        sys.modules.pop(name, None)
        importlib.import_module(name)
    leaked = FORBIDDEN_IMPORT_ROOTS & set(sys.modules)
    assert not leaked, f"importing the package pulled driver package(s): {leaked}"


def _run_all():
    fns = [v for k, v in sorted(globals().items())
           if k.startswith("test_") and callable(v)]
    failed = 0
    for fn in fns:
        try:
            fn()
            print(f"PASS {fn.__name__}")
        except AssertionError as e:
            failed += 1
            print(f"FAIL {fn.__name__}: {e}")
        except Exception as e:                       # noqa: BLE001
            failed += 1
            print(f"ERROR {fn.__name__}: {type(e).__name__}: {e}")
    print(f"\n{len(fns) - failed}/{len(fns)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(_run_all())
