"""
BINgo - nudge the saved floor frame after the known-point test.

The known-point report said, for example, "Height error ... (mean -3.8)": the ball centre was reported
3.8 cm too LOW at every point. A constant height error like that is a floor-plane offset (board
thickness, grazing-angle pose, ball size) and can be removed exactly:

    python3 adjust_world_frame.py --raise-cm 3.8

--raise-cm  : raise every reported height by this many cm (use the NEGATIVE of the report's mean height
              error: mean -3.8 -> --raise-cm 3.8). Positive mean error -> negative --raise-cm.
The original file is kept as <name>.before_adjust.npz. Run check_known_points_split.py again afterwards:
the height error should now be about 0.

Not corrected here on purpose: scale (a 0.8% scale error is inside the noise) and rotation about the
vertical (a 1-2 deg turn of the floor axes does not matter for catching, because the robot will be
located in this same frame; it only means your tape lines are not exactly the world axes).
"""
import argparse
import shutil
from pathlib import Path

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument("--world", default="bingo_world_frame_split.npz")
ap.add_argument("--raise-cm", type=float, required=True)
args = ap.parse_args()

path = Path(args.world)
if not path.exists():
    raise SystemExit(f"Cannot find {path}")
w = dict(np.load(path, allow_pickle=False))
if "R_cam0_from_world" not in w:
    raise SystemExit("Not a split-pipeline world file.")
backup = path.with_suffix(".before_adjust.npz")
if not backup.exists():
    shutil.copy(path, backup)
d = args.raise_cm / 100.0
R = w["R_cam0_from_world"].astype(float)
t = w["t_cam0_from_world"].astype(float).reshape(3)
w["t_cam0_from_world"] = (t + d * R[:, 2]).reshape(3, 1)      # moves the floor plane; height = -Z
w["height_correction_cm_total"] = np.array([float(w.get("height_correction_cm_total", 0.0)) + args.raise_cm])
np.savez(path, **w)
print(f"Heights raised by {args.raise_cm:+.1f} cm (total applied so far: {float(w['height_correction_cm_total'][0]):+.1f} cm).")
print(f"Original saved as {backup}.  Re-run check_known_points_split.py to confirm the height error is ~0.")