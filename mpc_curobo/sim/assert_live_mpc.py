#!/usr/bin/env python3
"""Assert the LIVE cuRobo-MPC e2e: the MPC-driven arm moves while the gate is RUN
(forwarded counter rising) and holds while STOP (counter flat, ~0 travel)."""
import argparse
import json
import sys


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rec", required=True)
    args = ap.parse_args()
    js, st = [], []
    for line in open(args.rec):
        d = json.loads(line)
        if d.get("kind") == "joint_states":
            js.append((d["t"], d["pos"]))
        elif d.get("kind") == "status":
            st.append((d["t"], d.get("forwarded", 0)))
    js.sort(); st.sort()

    def trav(a, b):
        return sum(abs(x - y) for x, y in zip(a, b))

    def seg(t0, t1):
        tot, prev = 0.0, None
        for t, p in js:
            if t0 <= t <= t1:
                if prev is not None:
                    tot += trav(prev, p)
                prev = p
        return tot

    fwd = held = 0.0
    for i in range(1, len(st)):
        s = seg(st[i - 1][0], st[i][0])
        if st[i][1] > st[i - 1][1]:
            fwd += s
        else:
            held += s
    ok = fwd > 0.1 and held < 0.05 and len(js) > 10 and len(st) > 3
    print(f"[{'PASS' if ok else 'FAIL'}] LIVE-MPC: forwarding_travel={fwd:.4f} "
          f"held_travel={held:.4f} js={len(js)} st={len(st)} "
          f"(need fwd>0.1 moving under MPC while RUN, held<0.05 while STOP)")
    print("LIVE-MPC RESULT:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
