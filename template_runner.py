\
"""
template_runner.py

Run JSON motion templates using AxisServo.
Designed for simplicity: one Python process owns the serial port.

Usage:
  python template_runner.py --scan
  python template_runner.py --template templates/demo_sequence.json
  python template_runner.py --template templates/demo_abs.json --duration 1.2

"""
from __future__ import annotations

import argparse
import json
import time
from typing import Dict, List

from servo_controller import load_config, AxisServo


def _parse_targets(obj: Dict, ids: List[int]) -> Dict[int, float]:
    """
    Supports:
      {"all": 120} -> all motors 120
      {"1": 90, "3": -45} -> subset
      {"all": 10, "2": 30} -> 2 overrides all
    """
    out: Dict[int, float] = {}
    if "all" in obj:
        v = float(obj["all"])
        for i in ids:
            out[i] = v
    for k, v in obj.items():
        if k == "all":
            continue
        out[int(k)] = float(v)
    return out


def run_template(servo: AxisServo, template_path: str, override_duration: float | None = None):
    with open(template_path, "r", encoding="utf-8") as f:
        tpl = json.load(f)

    steps = tpl.get("steps", [])
    print(f"== Running template: {tpl.get('name', template_path)} ==")

    last_result = None
    for idx, step in enumerate(steps, start=1):
        op = step.get("op")
        print(f"[{idx}/{len(steps)}] op={op}")

        if op == "soft_reset":
            mode = step.get("mode", "to_present")
            servo.soft_reset(mode)
            print("  soft_reset:", mode)

        elif op == "set_abs_deg":
            targets = _parse_targets(step["targets"], servo.cfg.ids)
            servo.set_targets_abs_deg(targets)
            print(f"  set_abs_deg: {len(targets)} motors")

        elif op == "nudge_rel_deg":
            targets = _parse_targets(step["targets"], servo.cfg.ids)
            servo.nudge_rel_deg(targets)
            print(f"  nudge_rel_deg: {len(targets)} motors")

        elif op == "execute":
            duration_s = float(step.get("duration_s", 1.0))
            if override_duration is not None:
                duration_s = override_duration

            poll_hz = int(step.get("poll_hz", 50))
            eps_deg = float(step.get("eps_deg", 2.0))
            settle_n = int(step.get("settle_n", 3))

            # optional current guard in template:
            guard = step.get("current_guard")  # {"min": -300, "max": 300}
            guard_tuple = None
            if isinstance(guard, dict) and "min" in guard and "max" in guard:
                guard_tuple = (int(guard["min"]), int(guard["max"]))

            last_result = servo.execute_time_synced(
                duration_s=duration_s,
                poll_hz=poll_hz,
                eps_deg=eps_deg,
                settle_n=settle_n,
                current_guard=guard_tuple,
                log=True,
            )
            print(f"  execute done. settled={last_result['settled']} max_err_deg={last_result['max_err_deg']:.2f}")

        elif op == "wait":
            sec = float(step.get("seconds", 0.1))
            time.sleep(sec)
            print(f"  wait {sec}s")

        else:
            raise ValueError(f"Unknown op: {op}")

    return last_result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.json")
    ap.add_argument("--scan", action="store_true", help="scan IDs and exit")
    ap.add_argument("--template", default=None, help="path to a template json")
    ap.add_argument("--duration", type=float, default=None, help="override execute.duration_s")
    ap.add_argument("--no_mode_set", action="store_true", help="skip ensure_extended_mode")
    ap.add_argument("--soft_reset", default=None, choices=["to_present", "to_nearest_turn"], help="run soft_reset then continue")
    args = ap.parse_args()

    cfg = load_config(args.config)
    servo = AxisServo(cfg)

    servo.connect()
    try:
        if args.scan:
            print("Online IDs:", servo.scan_ids())
            return

        if not args.no_mode_set:
            servo.ensure_extended_mode(cfg.ids)

        if args.soft_reset:
            servo.soft_reset(args.soft_reset)

        if args.template:
            run_template(servo, args.template, override_duration=args.duration)
        else:
            print("No --template given. Use --scan or --template <path>.")
    finally:
        servo.close()


if __name__ == "__main__":
    main()
