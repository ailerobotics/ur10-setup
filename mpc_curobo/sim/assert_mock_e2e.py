#!/usr/bin/env python3
"""Assert the mock-hardware e2e recordings show the required gate behaviour.

The bridge publishes a monotonic ``forwarded`` counter that increments ONLY on a
tick where the shim FORWARDED a command (i.e. the gate was RUN). We classify each
inter-status interval as "forwarding" (counter rose) or "held" (counter flat) and
attribute the joint travel in that interval accordingly. This is robust to status
sampling rate and ties motion directly to actual forwarding.

GATING: joint travel while FORWARDING must dominate; travel while HELD must be
~0 (a command arriving during STOP/SLOW/BACKSTOP does not move the arm); and
there must be >=2 forwarding episodes separated by a hold (moves -> holds ->
RESUMES toward the freshest target when RUN returns).

STARVATION: after the bridge is killed (its status stream stops), the arm must
hold — post-kill joint travel ~0 — proving forward_position_controller holds its
last setpoint when the command stream starves (the arm ends stopped).
"""
import argparse
import json
import sys


def load(path):
    js, st = [], []
    with open(path) as fh:
        for line in fh:
            d = json.loads(line)
            if d.get("kind") == "joint_states":
                js.append((d["t"], d["pos"]))
            elif d.get("kind") == "status":
                st.append((d["t"], d.get("gate"), d.get("forwarded", 0)))
    js.sort(key=lambda x: x[0]); st.sort(key=lambda x: x[0])
    return js, st


def travel(a, b):
    return sum(abs(x - y) for x, y in zip(a, b))


def _sum_travel(js, t0, t1):
    tot = 0.0
    prev = None
    for t, p in js:
        if t0 <= t <= t1:
            if prev is not None:
                tot += travel(prev, p)
            prev = p
    return tot


def check_gating(path):
    js, st = load(path)
    if len(js) < 5 or len(st) < 3:
        return False, f"gating: too few samples (js={len(js)}, st={len(st)})"
    fwd_travel = held_travel = 0.0
    episodes = 0            # number of distinct forwarding runs
    in_episode = False
    for i in range(1, len(st)):
        t0, _, f0 = st[i - 1]
        t1, _, f1 = st[i]
        seg = _sum_travel(js, t0, t1)
        if f1 > f0:                       # forwarded during this interval => RUN
            fwd_travel += seg
            if not in_episode:
                episodes += 1
                in_episode = True
        else:                             # counter flat => held (not RUN)
            held_travel += seg
            in_episode = False
    ok = fwd_travel > 0.1 and held_travel < 0.05 and episodes >= 2
    return ok, (f"gating: forwarding_travel={fwd_travel:.4f} held_travel={held_travel:.4f} "
                f"episodes={episodes} (need fwd>0.1, held<0.05, episodes>=2 for resume)")


def check_starvation(path):
    js, st = load(path)
    if len(js) < 5 or not st:
        return False, f"starvation: too few samples (js={len(js)}, st={len(st)})"
    kill_t = max(t for t, _, _ in st)          # last bridge status ~ kill time
    pre = [p for (t, p) in js if t <= kill_t]
    post = [p for (t, p) in js if t > kill_t + 0.5]  # allow 0.5 s settle
    if len(pre) < 2 or len(post) < 2:
        return False, f"starvation: not enough pre/post samples (pre={len(pre)}, post={len(post)})"
    pre_travel = sum(travel(pre[i - 1], pre[i]) for i in range(1, len(pre)))
    post_travel = sum(travel(post[i - 1], post[i]) for i in range(1, len(post)))
    ok = pre_travel > 0.1 and post_travel < 0.02
    return ok, (f"starvation: kill@{kill_t:.1f}s pre_travel={pre_travel:.4f} "
                f"post_travel={post_travel:.4f} (need pre>0.1 moving, post<0.02 held)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gating", required=True)
    ap.add_argument("--starvation", required=True)
    args = ap.parse_args()
    results = [("GATING", *check_gating(args.gating)),
               ("STARVATION", *check_starvation(args.starvation))]
    allok = True
    for name, ok, msg in results:
        print(f"[{'PASS' if ok else 'FAIL'}] {name}: {msg}")
        allok = allok and ok
    print("E2E RESULT:", "PASS" if allok else "FAIL")
    sys.exit(0 if allok else 1)


if __name__ == "__main__":
    main()
