"""Setup checks, gamepad teleop, passive diagnostics and policy benchmarks for the LWA 4P."""

import argparse
import json
import logging
import time

from .bridge_client import ArmFault, BridgeClient, BridgeError


def _check(args) -> None:
    import torch

    gpu = torch.cuda.get_device_name(0) if torch.cuda.is_available() else "none"
    print(f"torch {torch.__version__}, CUDA GPU: {gpu}")
    try:
        bridge = BridgeClient(args.host, args.port)
        bridge.connect()
        state = bridge.call("state")
        print(f"bridge: q={state['q']} gripper={state['gripper']} error={state['error']!r}")
        bridge.close()
    except BridgeError as e:
        print(f"bridge: {e}")
    try:
        import pyrealsense2 as rs

        devices = [
            f"{d.get_info(rs.camera_info.name)} #{d.get_info(rs.camera_info.serial_number)}"
            for d in rs.context().query_devices()
        ]
        hint = "none found (udev rule installed? scripts/system_setup.sh up)"
        print(f"RealSense: {devices or hint}")
    except RuntimeError as e:
        print(f"RealSense: {e}")


def _gamepad(args) -> None:
    from .teleop import GamepadReader

    pad = GamepadReader()
    print(f"{pad.open()}: press controls to see what teleop would command (Ctrl+C to stop)")
    last = None
    try:
        while True:
            r = pad.read()
            line = "  ".join(
                [*(k for k in ("deadman", "precision", "close", "open") if r[k])]
                + [
                    f"{n}={v:+.1f}"
                    for n, v in zip(("x", "y", "z", "roll", "pitch", "yaw"), r["twist"])
                    if v
                ]
            )
            if line != last:
                print(line or "(idle)", flush=True)
                last = line
            time.sleep(0.05)
    except KeyboardInterrupt:
        pad.close()


def _bench(args) -> None:
    import torch
    from lerobot.configs.policies import PreTrainedConfig
    from lerobot.configs.types import FeatureType
    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    cfg = PreTrainedConfig.from_pretrained(args.policy)
    cfg.device = "cuda"
    if args.dtype and hasattr(cfg, "dtype"):
        cfg.dtype = args.dtype
    policy = get_policy_class(cfg.type).from_pretrained(args.policy, config=cfg).eval()
    pre, _ = make_pre_post_processors(cfg, pretrained_path=args.policy)
    obs = {"task": args.task}
    for key, ft in cfg.input_features.items():
        obs[key] = torch.rand(ft.shape) if ft.type == FeatureType.VISUAL else torch.zeros(ft.shape)
    batch = pre(obs)
    torch.cuda.reset_peak_memory_stats()
    times = []
    with torch.inference_mode():
        for _ in range(args.runs + 1):
            torch.cuda.synchronize()
            t0 = time.perf_counter()
            chunk = policy.predict_action_chunk(batch)
            torch.cuda.synchronize()
            times.append(time.perf_counter() - t0)
    lat = sorted(times[1:])
    print(
        f"{args.policy} ({cfg.type}): chunk {tuple(chunk.shape)}, latency median "
        f"{lat[len(lat) // 2] * 1e3:.0f} ms (min {lat[0] * 1e3:.0f}), peak VRAM "
        f"{torch.cuda.max_memory_allocated() / 2**30:.2f} GiB"
    )


def _teleop(args) -> None:
    """Run gamepad control with a clean, explicit exit on a drive fault."""
    from .robot import Lwa4p, Lwa4pConfig
    from .teleop import Lwa4pGamepad, Lwa4pGamepadConfig

    robot = Lwa4p(
        Lwa4pConfig(
            host=args.host,
            port=args.port,
            cameras={},
            max_velocity_rad_s=args.joint_speed,
            max_acceleration_rad_s2=args.acceleration,
            max_jerk_rad_s3=args.jerk,
        )
    )
    teleop = Lwa4pGamepad(
        Lwa4pGamepadConfig(
            host=args.host,
            port=args.port,
            linear_speed_m_s=args.speed,
            grasp_gap_m=args.grasp_gap,
        )
    )
    try:
        teleop.connect()
        robot.connect()
        while True:
            started = time.monotonic()
            robot.send_action(teleop.get_action())
            time.sleep(max(0.0, 0.05 - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    except (ArmFault, ConnectionError) as error:
        print(f"Teleop stopped: {error}. Drives will not be re-enabled automatically.", flush=True)
        raise SystemExit(2) from None
    finally:
        try:
            robot.disconnect()
        except BridgeError as error:
            logging.warning("disconnect: %s", error)
        teleop.disconnect()


def _capture(args) -> None:
    """Record passive bus feedback and read-only bridge state for a later authorized test."""
    from .capture import capture

    print(f"Passive capture to {args.output}; no drive commands are sent", flush=True)
    try:
        print(capture(args.interface, args.host, args.port, args.output, args.duration))
    except (OSError, ValueError) as error:
        print(f"Capture stopped: {error}", flush=True)
        raise SystemExit(2) from None


def _analyze(args) -> None:
    """Print a compact summary from a saved capture without connecting to hardware."""
    from .capture import analyze

    print(json.dumps(analyze(args.input), indent=2))


def main() -> None:
    """Entry point."""
    ap = argparse.ArgumentParser(prog="lwa4p-vla", description=__doc__)
    sub = ap.add_subparsers(dest="cmd", required=True)
    check = sub.add_parser("check", help="GPU, bridge and RealSense status")
    check.add_argument("--host", default="127.0.0.1")
    check.add_argument("--port", type=int, default=5555)
    check.set_defaults(func=_check)
    sub.add_parser("gamepad", help="show what the gamepad teleop reads").set_defaults(func=_gamepad)
    bench = sub.add_parser("bench", help="policy inference latency and peak VRAM")
    bench.add_argument("--policy", default="lerobot/smolvla_base")
    bench.add_argument("--task", default="pick up the red cube")
    bench.add_argument("--runs", type=int, default=10)
    bench.add_argument("--dtype", help="override the policy dtype, e.g. bfloat16 for pi05")
    bench.set_defaults(func=_bench)
    teleop = sub.add_parser("teleop", help="gamepad control with clean drive-fault reporting")
    teleop.add_argument("--host", default="127.0.0.1")
    teleop.add_argument("--port", type=int, default=5555)
    teleop.add_argument("--speed", type=float, default=0.03)
    teleop.add_argument("--joint-speed", type=float, default=0.3)
    teleop.add_argument("--acceleration", type=float, default=0.3)
    teleop.add_argument("--jerk", type=float, default=2.0)
    teleop.add_argument("--grasp-gap", type=float, default=0.017)
    teleop.set_defaults(func=_teleop)
    capture = sub.add_parser("capture", help="passive CAN/bridge recording; no motion or enable")
    capture.add_argument("--interface", default="can0")
    capture.add_argument("--host", default="127.0.0.1")
    capture.add_argument("--port", type=int, default=5555)
    capture.add_argument("--output", required=True)
    capture.add_argument("--duration", type=float, default=120.0)
    capture.set_defaults(func=_capture)
    analyze = sub.add_parser("analyze", help="summarize a saved capture offline")
    analyze.add_argument("input")
    analyze.set_defaults(func=_analyze)
    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
