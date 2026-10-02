#!/usr/bin/env python3
"""A bounded typed provider request; exit 0 requires the executed model ACK."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile


def response_json(response):
    return {
        "accepted": bool(response.accepted),
        "enabled": bool(response.enabled),
        "reason": int(response.reason),
        "generation": str(response.generation),
        "message": response.message,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--service", required=True, help="exact absolute SetProvider service name")
    parser.add_argument("--action", required=True, choices=("observe", "start", "stop"))
    parser.add_argument("--expected-generation", type=int, help="explicit CAS generation; otherwise use observe ACK")
    parser.add_argument("--timeout", type=float, default=5.0, help="total wall-time budget in seconds (0, 30]")
    parser.add_argument("--receipt", type=Path, help="write the same JSON receipt atomically to this path")
    args = parser.parse_args()
    if not args.service.startswith("/") or args.service == "/":
        parser.error("--service must name an absolute endpoint")
    if not math.isfinite(args.timeout) or not 0 < args.timeout <= 30:
        parser.error("--timeout must be finite and in (0, 30]")
    if args.expected_generation is not None and not 0 <= args.expected_generation < 2**64:
        parser.error("--expected-generation must be uint64")
    receipt = {
        "schema": "xgc.sim-provider-command/1",
        "service": args.service,
        "action": args.action,
        "calls": [],
        "completed": False,
    }

    def expired(_signum, _frame):
        raise TimeoutError("provider command exceeded its total wall-time budget")

    signal.signal(signal.SIGALRM, expired)
    signal.setitimer(signal.ITIMER_REAL, args.timeout)
    exit_code = 1
    try:
        import rospy
        from xgc2_lightweight_sim_msgs.srv import SetProvider

        rospy.init_node("xgc_provider_once", anonymous=True, disable_signals=True)
        rospy.wait_for_service(args.service, timeout=args.timeout)
        service = rospy.ServiceProxy(args.service, SetProvider, persistent=False)

        def call(action, generation):
            response = service(action=action, generation=generation)
            receipt["calls"].append({
                "request": {"action": action, "generation": str(generation)},
                "response": response_json(response),
            })
            if int(response.reason) not in (0, 1, 2):
                raise RuntimeError("invalid provider reason in executed result")
            return response

        observed = call(0, 0)
        if not observed.accepted or observed.reason != 0:
            raise RuntimeError("provider observe was not accepted")
        if args.action == "observe":
            receipt["completed"] = True
        else:
            generation = observed.generation if args.expected_generation is None else args.expected_generation
            action = 1 if args.action == "start" else 2
            result = call(action, generation)
            enabled = action == 1
            expected = (observed.generation if observed.enabled else generation + 1) if enabled else generation
            receipt["completed"] = (
                bool(result.accepted) and result.reason == 0 and
                bool(result.enabled) == enabled and result.generation == expected
            )
            if not receipt["completed"]:
                raise RuntimeError("provider did not ACK the requested CAS state")
        exit_code = 0
    except Exception as error:
        receipt["error"] = str(error)
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
    payload = json.dumps(receipt, sort_keys=True) + "\n"
    if args.receipt:
        args.receipt.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=args.receipt.name + ".", dir=args.receipt.parent)
        try:
            with os.fdopen(fd, "w") as output:
                output.write(payload)
            os.replace(temporary, args.receipt)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
    sys.stdout.write(payload)
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
