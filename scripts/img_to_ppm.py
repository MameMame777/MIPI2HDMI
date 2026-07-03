"""Convert an arbitrary image file to binary PPM (P6) for the cocotb img_file_uvm block.

Runs under the repo-root CPython venv (.venv, which has Pillow) as a SUBPROCESS of the
cocotb pytest wrapper -- the MinGW cocotb venv cannot import Pillow, so all real-format
decoding is isolated here. Keep this script Pillow-only (no numpy).

Usage:
    python scripts/img_to_ppm.py INPUT OUTPUT.ppm [--max-width N] [--max-height N]

Prints "WIDTH HEIGHT" on success.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--max-width", type=int, default=640)
    ap.add_argument("--max-height", type=int, default=480)
    args = ap.parse_args()

    try:
        from PIL import Image
    except ImportError:
        print("img_to_ppm: Pillow is not installed in this interpreter "
              f"({sys.executable}). Install Pillow or pass a .ppm/.pgm directly.",
              file=sys.stderr)
        return 2

    img = Image.open(args.input).convert("RGB")
    img.thumbnail((args.max_width, args.max_height), Image.LANCZOS)
    w, h = img.size
    raw = img.tobytes()  # RGB byte triplets, row-major

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(b"P6\n%d %d\n255\n" % (w, h) + raw)
    print(f"{w} {h}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
