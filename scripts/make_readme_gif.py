#!/usr/bin/env python3
"""Create the README demo GIF from a frame range in the teleoperation video."""

from __future__ import annotations

import argparse
import subprocess
import tempfile
from pathlib import Path


def run(command: list[str]) -> None:
    subprocess.run(command, check=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="Input video path")
    parser.add_argument("output", type=Path, help="Output GIF path")
    parser.add_argument("--start-frame", type=int, default=3000)
    parser.add_argument("--end-frame", type=int, default=3180)
    parser.add_argument("--source-fps", type=float, default=30.0)
    parser.add_argument("--gif-fps", type=int, default=10)
    parser.add_argument("--width", type=int, default=720)
    args = parser.parse_args()

    if args.end_frame <= args.start_frame:
        raise SystemExit("--end-frame must be greater than --start-frame")

    start = args.start_frame / args.source_fps
    duration = (args.end_frame - args.start_frame) / args.source_fps
    args.output.parent.mkdir(parents=True, exist_ok=True)

    filters = (
        f"fps={args.gif_fps},"
        f"scale={args.width}:-1:flags=lanczos,"
        "split[s0][s1];"
        "[s0]palettegen=max_colors=96:stats_mode=diff[p];"
        "[s1][p]paletteuse=dither=bayer:bayer_scale=3"
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        temp_output = Path(tmpdir) / args.output.name
        run(
            [
                "ffmpeg",
                "-y",
                "-ss",
                f"{start:.3f}",
                "-t",
                f"{duration:.3f}",
                "-i",
                str(args.source),
                "-vf",
                filters,
                "-loop",
                "0",
                str(temp_output),
            ]
        )
        temp_output.replace(args.output)


if __name__ == "__main__":
    main()
