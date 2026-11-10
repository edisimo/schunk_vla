"""`lwa4p-vla` CLI: check the setup, and measure a policy's chunk latency and VRAM on this GPU."""

import argparse
import time

from .bridge_client import BridgeClient, BridgeError


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
    args = ap.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
