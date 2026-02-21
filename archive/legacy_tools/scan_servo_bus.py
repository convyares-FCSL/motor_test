#!/usr/bin/env python3
import argparse
import sys
from pathlib import Path


def add_local_library_path() -> None:
    project_root = Path(__file__).resolve().parent
    lib_root = project_root / "stservo-env"
    if not lib_root.exists():
        raise FileNotFoundError(f"Library folder not found: {lib_root}")
    sys.path.insert(0, str(lib_root))


add_local_library_path()

from scservo_sdk import COMM_SUCCESS  # noqa: E402


def parse_bauds(text: str) -> list[int]:
    vals = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        vals.append(int(part))
    if not vals:
        raise ValueError("No baud rates provided.")
    return vals


def scan_with_stack(
    stack_name: str,
    port: str,
    bauds: list[int],
    id_min: int,
    id_max: int,
) -> int:
    if stack_name == "sms":
        from scservo_sdk import PortHandler, sms_sts as HandlerClass  # noqa: E402
    elif stack_name == "sts":
        from STservo_sdk import PortHandler, sts as HandlerClass  # noqa: E402
    else:
        raise ValueError(f"Unknown stack: {stack_name}")

    total_hits = 0
    print(f"\n=== Stack: {stack_name} ===", flush=True)

    for baud in bauds:
        print(f"[STEP] port={port} baud={baud} ids={id_min}..{id_max}", flush=True)
        ph = PortHandler(port)
        handler = HandlerClass(ph)
        hits = []
        try:
            if not ph.openPort():
                print("  [ERR] openPort failed", flush=True)
                continue
            if not ph.setBaudRate(baud):
                print("  [ERR] setBaudRate failed", flush=True)
                continue

            for sid in range(id_min, id_max + 1):
                model, comm, err = handler.ping(sid)
                if comm == COMM_SUCCESS and err == 0:
                    hits.append((sid, model))
        except Exception as exc:
            print(f"  [ERR] exception: {exc}", flush=True)
        finally:
            try:
                ph.closePort()
            except Exception:
                pass

        if hits:
            total_hits += len(hits)
            hit_text = ", ".join([f"ID={sid} model={model}" for sid, model in hits])
            print(f"  [HIT] {hit_text}", flush=True)
        else:
            print("  [MISS] no responses", flush=True)

    print(f"[DONE] stack={stack_name} total_hits={total_hits}", flush=True)
    return total_hits


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan serial bus servos by port/baud/ID range.")
    parser.add_argument("--port", required=True, help="Serial port (COM13, /dev/ttyUSB0, etc.)")
    parser.add_argument(
        "--baud-list",
        default="1000000,500000,250000,115200,57600,38400",
        help="Comma-separated baud list",
    )
    parser.add_argument("--id-min", type=int, default=0, help="Start ID (default: 0)")
    parser.add_argument("--id-max", type=int, default=20, help="End ID (default: 20)")
    parser.add_argument(
        "--stack",
        choices=["sms", "sts", "both"],
        default="both",
        help="Protocol stack to test",
    )
    args = parser.parse_args()

    if args.id_min < 0 or args.id_max > 253 or args.id_min > args.id_max:
        print("[ERR] Invalid ID range; must be 0..253 and id-min <= id-max.", flush=True)
        return 2

    bauds = parse_bauds(args.baud_list)
    stacks = ["sms", "sts"] if args.stack == "both" else [args.stack]

    grand_total = 0
    for stack_name in stacks:
        grand_total += scan_with_stack(stack_name, args.port, bauds, args.id_min, args.id_max)

    if grand_total == 0:
        print("\n[RESULT] No devices detected.", flush=True)
        return 1

    print(f"\n[RESULT] Total detected responses: {grand_total}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
