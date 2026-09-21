# SPDX-License-Identifier: Apache-2.0
"""The harness must refuse to run on the shared/live ROS domains (0, 42, 77) and
default to the SIM-only domain 88. Tested against the real domain_guard.sh that
isaac_env.sh sources first — no conda / Isaac needed."""
import os
import subprocess

_HERE = os.path.dirname(os.path.abspath(__file__))
_HARNESS = os.path.dirname(_HERE)
_GUARD = os.path.join(_HARNESS, "domain_guard.sh")


def _source_guard(domain):
    """Source the guard with ROS_DOMAIN_ID=domain (None => unset). Returns the
    completed process; stdout carries 'OK:<resulting domain>' when it proceeds."""
    env = os.environ.copy()
    if domain is None:
        env.pop("ROS_DOMAIN_ID", None)
    else:
        env["ROS_DOMAIN_ID"] = str(domain)
    cmd = f'source "{_GUARD}" && echo "OK:${{ROS_DOMAIN_ID}}"'
    return subprocess.run(["bash", "-c", cmd], env=env,
                          capture_output=True, text=True)


def test_defaults_to_88_when_unset():
    r = _source_guard(None)
    assert r.returncode == 0, r.stderr
    assert "OK:88" in r.stdout


def test_allows_a_permitted_domain():
    r = _source_guard(85)
    assert r.returncode == 0, r.stderr
    assert "OK:85" in r.stdout


def test_default_88_is_allowed_explicitly():
    r = _source_guard(88)
    assert r.returncode == 0, r.stderr
    assert "OK:88" in r.stdout


def test_refuses_forbidden_domains():
    for dom in (0, 42, 77):
        r = _source_guard(dom)
        assert r.returncode == 3, f"domain {dom} not refused: {r.stdout!r}"
        assert "OK:" not in r.stdout, f"domain {dom} proceeded"
        assert "REFUSING" in r.stderr
