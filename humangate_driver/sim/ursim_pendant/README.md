# URSim CB3 pendant-mode assets (External Control program set)

These files let `sim/investigate_pause_ursim.sh MODE=pendant` bring up URSim CB3
with the **External Control URCap installed and an External Control program LOADED
and PLAYING on the (simulated) pendant** — the non-headless setup this cell does
*not* run today, needed to test whether pausing a loaded program freezes-and-continues
the planned motion (see `../../PENDANT_PAUSE_FINDING.md`).

## What is here

`programs/` — the CB3 program folder mounted read-then-copied into the URSim
container at `/ursim/programs`:

| file | role |
|---|---|
| `external_control.urp` | a PolyScope program whose MainProgram is a single External Control node (`com.fzi.externalcontrol.ExternalControlNode`). Created in PolyScope 3.15.8 — the exact version this cell runs. Loaded + played via the dashboard; the driver then streams the `scaled_joint_trajectory_controller` motion over the reverse interface. |
| `default.installation` | the installation the program references (CB3 3.12.1 format, forward-compatible with 3.15.8). It has NO External Control URCap installation node, so the URCap's built-in default host IP applies. |
| `default.variables` | the installation's variables file (required alongside the installation). |
| `wait_program.urp` | a trivial wait program (unused by the pause test; kept as the upstream set ships it, useful for a "program loaded but not External Control" cross-check). |

The **External Control URCap `.jar` is NOT committed** — the harness copies it at
run time from the installed driver
(`/opt/ros/*/share/ur_robot_driver/resources/externalcontrol-*.urcap`, a JAR), so it
always matches the `ur_robot_driver` in use.

## The host IP the loaded program connects back to

The External Control URCap 1.0.5 has a compiled-in default host IP `192.168.56.1`
and port `50002` (`ExternalControlInstallationNodeContribution`: `DEFAULT_IP`,
`DEFAULT_PORT`). `default.installation` does not override it. So the harness runs
URSim on a docker network whose **gateway is `192.168.56.1`** (subnet
`192.168.56.0/24`) and launches the driver with `reverse_ip:=192.168.56.1`; the
loaded program then reaches the driver with **no installation edit**. On the real
pendant the owner sets this host IP to the workstation's address on the robot
network (`192.168.0.2` in this cell) — see `../../LIVE_TEST.md`.

## Provenance

`external_control.urp`, `wait_program.urp`, `default.installation`,
`default.variables` are taken verbatim from the Universal Robots **client library**
test fixtures (`Universal_Robots_Client_Library`,
`tests/resources/dockerursim/programs/cb3/`), the same CB3 URSim assets that
project's own CI loads. That library is **BSD-3-Clause** (© Universal Robots A/S).
They are unmodified. SHA-256 of the committed copies is recorded in
`../../PENDANT_PAUSE_FINDING.md` so drift is detectable.
