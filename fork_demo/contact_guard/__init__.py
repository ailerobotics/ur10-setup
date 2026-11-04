"""Early-failure contact-safety layer for the real UR10 fork pick-and-place.

Pure, offline-testable cores (no rclpy, no torch): signal extraction
(:mod:`signals`), per-leg/per-progress envelopes (:mod:`envelope`), the runtime
detector (:mod:`detector`), reaction policy (:mod:`reaction`), incremental
contact-leg segmenting (:mod:`segmenting`), the pre-contact gate (:mod:`gate`),
post-action verification (:mod:`verify`), the external-alignment veto interface
(:mod:`veto`) and fault injection (:mod:`faults`).

The one module that reads a recorded mcap bag (:mod:`bagsource`) imports ``mcap``
lazily; the ROS live watcher (:mod:`node`) imports ``rclpy`` lazily. Everything
else is importable with only the Python standard library + numpy + PyYAML.

Nothing in this package commands the real robot. The runtime watcher only ever
*cancels* a running goal and (on URSim/mock) sends a short in-axis back-off; on
the real cell the existing multi-gate execute path and the powered-off cell are
the real stops. This layer is a supplementary early-failure guard, NOT a
certified safety function.
"""

__all__ = [
    "params",
    "signals",
    "envelope",
    "detector",
    "reaction",
    "segmenting",
    "gate",
    "verify",
    "veto",
    "faults",
]
