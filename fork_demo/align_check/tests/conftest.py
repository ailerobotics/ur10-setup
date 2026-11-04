"""Put the repo root on sys.path so ``import fork_demo.align_check`` works
without an install (this repo is script-based)."""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[3]  # repo root
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

FIXTURES = pathlib.Path(__file__).resolve().parent / "fixtures"
