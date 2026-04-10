#!/usr/bin/env python3
"""Record a web dashboard demo video by capturing headless Chrome frames."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import time
from pathlib import Path

import imageio.v2 as imageio


def run(cmd: list[str]) -> None:
    subprocess.run(cmd, check=True)


def submit_order(container_name: str, pickup: str, delivery: str, priority: int) -> None:
    shell_cmd = (
        "set +u; "
        "source /opt/ros/humble/setup.bash; "
        "source /ros2_ws/install/setup.bash; "
        "set -u; "
        "ros2 service call /submit_order warehouse_interfaces/srv/SubmitOrder "
        f"\"{{pickup_station: '{pickup}', delivery_station: '{delivery}', priority: {priority}}}\""
    )
    run(["sudo", "-n", "docker", "exec", container_name, "bash", "-lc", shell_cmd])


def capture_frame(chrome_bin: str, url: str, output_path: Path, settle_ms: int) -> None:
    run(
        [
            chrome_bin,
            "--headless=new",
            "--disable-gpu",
            "--hide-scrollbars",
            "--window-size=1366,768",
            f"--virtual-time-budget={settle_ms}",
            f"--screenshot={str(output_path)}",
            url,
        ]
    )


def encode_video(frame_paths: list[Path], output_mp4: Path, fps: int) -> None:
    output_mp4.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(
        str(output_mp4),
        fps=fps,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
    )
    try:
        for frame_path in frame_paths:
            writer.append_data(imageio.imread(frame_path))
    finally:
        writer.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Record dashboard demo video.")
    parser.add_argument("--url", default="http://localhost:8080/")
    parser.add_argument("--container", default="docker-ros2-1")
    parser.add_argument("--pickup", default="A")
    parser.add_argument("--delivery", default="D1")
    parser.add_argument("--priority", type=int, default=1)
    parser.add_argument("--fps", type=int, default=2)
    parser.add_argument("--frames", type=int, default=40)
    parser.add_argument("--settle-ms", type=int, default=2200)
    parser.add_argument("--start-order-at", type=int, default=2)
    parser.add_argument("--output", default="docs/demo/dashboard-demo.mp4")
    parser.add_argument("--keep-frames", action="store_true")
    args = parser.parse_args()

    chrome_bin = shutil.which("google-chrome") or shutil.which("chromium") or shutil.which("chromium-browser")
    if not chrome_bin:
        raise SystemExit("No Chrome/Chromium binary found on host.")

    repo_root = Path(__file__).resolve().parents[1]
    frames_dir = repo_root / "demo_capture" / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    # Clear old frames.
    for old in frames_dir.glob("frame_*.png"):
        old.unlink()

    print(f"Capturing {args.frames} frames from {args.url} ...")
    frame_paths: list[Path] = []
    order_submitted = False
    for i in range(args.frames):
        if not order_submitted and i >= args.start_order_at:
            print("Submitting demo order ...")
            submit_order(args.container, args.pickup, args.delivery, args.priority)
            order_submitted = True
            # Allow order state to begin transitioning before next screenshot.
            time.sleep(1.0)

        frame_path = frames_dir / f"frame_{i:04d}.png"
        capture_frame(chrome_bin, args.url, frame_path, args.settle_ms)
        frame_paths.append(frame_path)
        print(f"Captured {i + 1}/{args.frames}: {frame_path.name}")

    output_mp4 = repo_root / args.output
    print(f"Encoding video: {output_mp4}")
    encode_video(frame_paths, output_mp4, args.fps)
    print(f"Done: {output_mp4}")

    if not args.keep_frames:
        for frame_path in frame_paths:
            frame_path.unlink(missing_ok=True)
        print(f"Cleaned frames from {frames_dir}")


if __name__ == "__main__":
    main()
