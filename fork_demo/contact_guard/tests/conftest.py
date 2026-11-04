"""Put the repo root on sys.path so ``import fork_demo.contact_guard`` works
without an install (this repo is script-based), and share the committed
envelope + params as fixtures."""
import pathlib
import sys

import pytest
import yaml

ROOT = pathlib.Path(__file__).resolve().parents[3]   # tests -> contact_guard -> fork_demo -> repo
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fork_demo.contact_guard import envelope, params  # noqa: E402

ENV_PATH = (pathlib.Path(__file__).resolve().parents[1] / "envelopes"
            / "teach-002.yaml")


@pytest.fixture(scope="session")
def p():
    return params.load()


@pytest.fixture(scope="session")
def env():
    return envelope.Envelope.from_dict(yaml.safe_load(ENV_PATH.read_text()))
