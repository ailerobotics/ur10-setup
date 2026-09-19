# Does slider zero fully halt the arm? — measured on URSim CB3 (task brief item 4)

The brief asks to "measure and report ... whether slider zero fully halts or a
floor applies." Measured on URSim `universalrobots/ursim_cb3:3.15.8` (matching the
cell's PolyScope 3.15.8) with the real `ur_robot_driver`, sweeping the speed
slider while ONE long trajectory executed and sampling both the applied scaling
(`/speed_scaling_state_broadcaster/speed_scaling`) and the actual joint motion.
Repeatable via `sim/measure_slider_ursim.sh`; raw output in
`../data/human-gate-sim-v1/ursim/slider_measurement.txt`.

| commanded slider | applied scaling | shoulder_pan motion (rad / 2.5 s) |
|---|---|---|
| 1.0            | 100 %           | 0.013 (moving)                    |
| 0.5            | 50 %            | 0.017                             |
| 0.1            | 10 %            | 0.004 (creeping)                  |
| **0.0**        | **held at 10 %**| **0.0045 — STILL CREEPING**       |
| 1.0 (resume)   | 100 %           | 0.064 (SAME trajectory continues) |

## Conclusion

- **The slider IS the primary lever for SLOW and RESUME.** Lowering it lowers the
  applied scaling proportionally (1.0->100 %, 0.5->50 %, 0.1->10 %) and the arm
  slows; raising it back to 1.0 resumes the SAME trajectory from where it was
  (pan kept advancing, no replan). This is the behaviour the design chose and it
  works.
- **Slider zero does NOT fully halt the arm on CB3.** Commanding 0.0 leaves the
  applied scaling at its previous non-zero value (a floor ~10 %, and in an idle
  cross-check it simply held the last commanded value), and the arm keeps creeping.
  So "STOP by driving the slider to zero" is NOT a reliable full stop on this CB3
  driver/URSim.

This is exactly **residual risk R3** in the design report (§9): "if the driver's
minimum slider fraction is >0, 'STOP via slider' doesn't fully halt." It is now
empirically confirmed, not assumed.

## Why this vindicates the design rather than breaking it

The design anticipated this and made the independent **goal-cancel backstop** the
real stop, with the primary lever falling through to it (report §2, §5 #15):

> STOP -> command slider -> 0 (or the driver's minimum; MEASURE whether 0 fully
> halts or a floor applies — if a floor applies, fall through to the backstop).

That is exactly what this implementation does. When the gate commands STOP, the
slider does not drive scaling to ~0 within the confirmation deadline, so:
1. the gate's driver-stopped feedback never confirms a stop (scaling stays > the
   stop threshold), so the gate's own primary->backstop fall-through fires; and
2. the independent watcher sees a hazard STOP that the slider did not effect
   within its deadline and cancels the goal too.

Either way the trajectory is cancelled and the arm decelerates to a hold — the
backstop, not the slider, delivers the full STOP. The slider still does the
useful graded SLOW and the smooth RESUME.

## Command-to-effect delay

In the URSim end-to-end run (`sim/run_ursim_e2e.sh`) the applied scaling reached
the commanded SLOW fraction (0.1) within **~50 ms** of the gate's slider write —
one gate cycle at the 20 Hz decide rate; the RTDE path itself is ~8 ms/cycle at
125 Hz (report §3). The coarse sweep above samples at 2.5 s so cannot resolve
below that, but the e2e measurement (`assert_e2e.py` "command-to-effect delay")
pins it at one gate cycle. The safety-relevant number — the STOP that the slider
CANNOT deliver — is handled by the backstop and does not depend on this delay.

## Implication for the live test

On the real CB3 the same is expected but must be re-measured with the owner
present (`LIVE_TEST.md`): whether slider zero halts, and the real stopping time and
distance. If the real arm's slider also floors, the goal-cancel backstop is the
commissioned STOP and a resume after STOP is a fresh plan (not the same
trajectory); the graded SLOW + resume-via-slider still apply above the floor.
