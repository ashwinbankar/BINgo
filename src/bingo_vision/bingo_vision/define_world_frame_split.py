"""
BINgo - Define the floor / world frame from the OBS side-by-side feed (Phase 4a)

Put the ChArUco board FLAT on the floor, taped down, CLOSE to the phones: about
0.5-0.7 m in front of the line joining them, roughly between them. A 53x35 cm board
lying on the floor is seen at a grazing angle; farther away its markers become too
small to detect (the live corner counter tells you). Press S.
Output: bingo_world_frame_split.npz

Why this version is better than the old one
  * It measures the board pose in BOTH phones independently, then compares them.
    If the calibration, the phones' positions, or the board are wrong, the two
    estimates disagree, and you see by how much BEFORE you trust any XYZ.
  * It uses both estimates (average), not just cam0.
  * It reads the OBS feed without forcing a size, and refuses a size mismatch.
  * It uses robust averaging, so a few bad frames do not spoil the result, and the
    thresholds fit a hand-printed board seen from 2-3 m (the old 0.5 deg / 5 mm
    limit was tighter than the noise you will actually have).

Controls:  S = collect and save     Q = quit

World frame (same convention as your old scripts)
    origin = the board's outer corner (the pattern corner where X = Y = 0)
    X along the 12-square side (0 .. 0.528 m), Y along the 8-square side (0 .. 0.352 m)
    Z = board normal pointing INTO the floor, so height above floor = -Z
The preview draws the X (red), Y (green), Z (blue) axes on the board so you can SEE this.

Do NOT move the phones after calibration. They must be in their final positions now.

Usage
    python3 define_world_frame_split.py --cam 1
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bingo_split_common as bc

MIN_CORNERS = 30             # per phone; a floor board is small and foreshortened
SAMPLES = 60
TIMEOUT_S = 60.0
WARN_TRANS_MM, WARN_ROT_DEG = 30.0, 3.0      # my guesses for "check things"
REFUSE_TRANS_MM, REFUSE_ROT_DEG = 80.0, 6.0  # my guesses for "something is wrong"


def poses_in_cam0(det0, det1, calib, board_pts):
    """Board poses in the cam0 frame, from cam0 directly and from cam1 via stereo."""
    p0 = bc.board_pose(det0[0], det0[1], board_pts, calib.K0, calib.d0)
    p1 = bc.board_pose(det1[0], det1[1], board_pts, calib.K1, calib.d1)
    if p0 is None or p1 is None:
        return None
    R1, t1 = bc.pose_cam1_to_cam0(p1[0], p1[1], calib)
    return (p0[0], p0[1]), (R1, t1), (p0[2], p1[2])


def pose_single(det, cam, calib, board_pts):
    """Board pose in the cam0 frame from ONE phone (cam1 goes through the stereo calibration)."""
    K, d = (calib.K0, calib.d0) if cam == 0 else (calib.K1, calib.d1)
    p = bc.board_pose(det[0], det[1], board_pts, K, d)
    if p is None:
        return None
    if cam == 0:
        return p[0], p[1]
    return bc.pose_cam1_to_cam0(p[0], p[1], calib)


def combine(estimates):
    """Average the two estimates (cam0-direct, cam1-via-stereo)."""
    (Ra, ta), (Rb, tb) = estimates
    return bc.mean_rotation([Ra, Rb]), 0.5 * (ta + tb)


def draw(cam_imgs, dets, calib, pose0, text_lines, color):
    views = [cam_imgs[0].copy(), cam_imgs[1].copy()]
    for v, (c, ids) in zip(views, dets):
        for p in c:
            cv2.circle(v, (int(p[0]), int(p[1])), 4, (0, 255, 0), -1)
    if pose0 is not None:
        rvec, _ = cv2.Rodrigues(pose0[0])
        cv2.drawFrameAxes(views[0], calib.K0, calib.d0, rvec, pose0[1].reshape(3, 1),
                          4 * calib.square_m, 3)
    for i, line in enumerate(text_lines):
        cv2.putText(views[0], line, (15, 35 + 30 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    return cv2.resize(cv2.hconcat(views), None, fx=0.55, fy=0.55, interpolation=cv2.INTER_AREA)


def collect(cam, calib, detector, board_pts, n, min_corners, single=False):
    est0, est1 = [], []          # cam0-direct and cam1-via-stereo estimates (may differ in count)
    start = time.time()
    n_ok = 0
    while n_ok < n:
        ok, left, right, _ = cam.read()
        if not ok:
            print("Camera stopped delivering frames.")
            return None
        dets = [bc.detect_board(detector, left), bc.detect_board(detector, right)]
        ok0, ok1 = len(dets[0][1]) >= min_corners, len(dets[1][1]) >= min_corners
        if ok0 and ok1:
            got = poses_in_cam0(dets[0], dets[1], calib, board_pts)
            if got is not None:
                est0.append(got[0])
                est1.append(got[1])
                n_ok += 1
        elif single and (ok0 or ok1):
            e = pose_single(dets[0], 0, calib, board_pts) if ok0 else pose_single(dets[1], 1, calib, board_pts)
            if e is not None:
                (est0 if ok0 else est1).append(e)
                n_ok += 1
        cv2.imshow("BINgo world frame", draw(
            (left, right), dets, calib, None,
            [f"Collecting {n_ok}/{n}  (do not touch the board or phones)"], (0, 255, 0)))
        cv2.waitKey(1)
        if time.time() - start > TIMEOUT_S:
            print("\nTimed out: the board was not seen well by BOTH phones.")
            return None
    return est0, est1


def report_single(est, which, calib, args):
    R, t, sp = bc.robust_pose_average([e[0] for e in est], [e[1] for e in est])
    wf = bc.WorldFrame(R, t)
    c0, c1 = wf.cam0_position_world(), wf.cam1_position_world(calib)
    print("\n============ FLOOR BOARD, seen by ONE phone (" + which + ") ============")
    print(f"{len(est)} frames. Board distance from cam0: {np.linalg.norm(t):.2f} m. "
          f"Frame-to-frame noise: {sp['rot_spread_deg']:.2f} deg / {sp['trans_spread_mm']:.1f} mm")
    print("NO CROSS-CHECK between the phones was possible, so errors of this floor frame are NOT visible here. "
          "A board seen at a grazing angle by one camera has an uncertain TILT. Verify with the tape:")
    print("\nPhone positions in the floor frame (X, Y in m; |Z| = lens height above floor):")
    print(f"  cam0 (left phone):  X={c0[0]:+.3f}  Y={c0[1]:+.3f}  height={-c0[2]:.3f} m")
    print(f"  cam1 (right phone): X={c1[0]:+.3f}  Y={c1[1]:+.3f}  height={-c1[2]:.3f} m")
    print("  CHECK 1: measure both lens heights with a tape; they should match to ~3-5 cm.")
    print("  CHECK 2: the known-point test (check_known_points_split.py) is now the real verification: if it "
          "shows a HEIGHT TILT or ROTATION note, redo this step with the board closer/larger in the image.")
    wf.save(args.out, calib, extra=dict(
        phone_disagreement_mm=float("nan"), phone_disagreement_deg=float("nan"), single_phone=which,
        cam0_height_m=float(-c0[2]), cam1_height_m=float(-c1[2])))
    print(f"\nSaved: {args.out}")
    return True


def report_and_save(est0, est1, calib, args):
    if min(len(est0), len(est1)) < 5:
        if max(len(est0), len(est1)) < 5:
            print("Too few usable frames.")
            return False
        return report_single(est0 if len(est0) >= len(est1) else est1,
                             "cam0" if len(est0) >= len(est1) else "cam1", calib, args)
    R0, t0, s0 = bc.robust_pose_average([e[0] for e in est0], [e[1] for e in est0])
    R1, t1, s1 = bc.robust_pose_average([e[0] for e in est1], [e[1] for e in est1])
    d_rot = bc.rotation_angle_deg(R0, R1)
    dt = t1 - t0
    d_mm = float(np.linalg.norm(dt) * 1000.0)
    depth_dir = t0 / np.linalg.norm(t0)
    d_depth = float(dt @ depth_dir * 1000.0)
    d_lat = float(np.linalg.norm(dt - (dt @ depth_dir) * depth_dir) * 1000.0)

    Rc, tc = bc.mean_rotation([R0, R1]), 0.5 * (t0 + t1)
    wf = bc.WorldFrame(Rc, tc)
    c0, c1 = wf.cam0_position_world(), wf.cam1_position_world(calib)

    print("\n================ FLOOR BOARD, seen by each phone ================")
    print(f"Board distance from cam0: {np.linalg.norm(t0):.2f} m   from cam1 (via stereo): "
          f"{np.linalg.norm(t1):.2f} m")
    print(f"Frame-to-frame noise (random):  cam0 {s0['rot_spread_deg']:.2f} deg / "
          f"{s0['trans_spread_mm']:.1f} mm    cam1 {s1['rot_spread_deg']:.2f} deg / "
          f"{s1['trans_spread_mm']:.1f} mm")
    print(f"DISAGREEMENT between the two phones: {d_rot:.2f} deg, {d_mm:.1f} mm "
          f"({d_depth:+.1f} mm along the viewing direction, {d_lat:.1f} mm sideways)")
    print("  Treat this as a LOWER BOUND on the floor-frame error. Averaging frames cannot "
          "reduce it, and errors both phones share (a wrong --square-mm, a warped board) "
          "do not show up here at all. The tape-measure checks below catch those.")
    print("\nPhone positions in the floor frame (X, Y in m; |Z| = lens height above floor):")
    print(f"  cam0 (left phone):  X={c0[0]:+.3f}  Y={c0[1]:+.3f}  height={-c0[2]:.3f} m")
    print(f"  cam1 (right phone): X={c1[0]:+.3f}  Y={c1[1]:+.3f}  height={-c1[2]:.3f} m")
    print("  CHECK: measure both lens heights with a tape. If they differ from these by more "
          "than ~3-5 cm, the floor plane is tilted or the calibration scale is off.")

    bad = d_mm > REFUSE_TRANS_MM or d_rot > REFUSE_ROT_DEG
    warn = d_mm > WARN_TRANS_MM or d_rot > WARN_ROT_DEG
    if bad and not args.force:
        print("\nNOT SAVED: the two phones disagree far too much. Likely causes: a phone was "
              "moved after calibration, the board is not flat or has moved, a wrong "
              "calibration file, or the board pose flipped. Fix that and press S again "
              "(or use --force to save anyway).")
        return False
    if warn:
        print("\nWARNING: disagreement is larger than I would like. It is usable for a first "
              "test, but expect the floor frame to be off by roughly this much.")

    wf.save(args.out, calib, extra=dict(
        phone_disagreement_mm=d_mm, phone_disagreement_deg=d_rot,
        cam0_height_m=float(-c0[2]), cam1_height_m=float(-c1[2])))
    print(f"\nSaved: {args.out}")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=1, help="OBS Virtual Camera index")
    ap.add_argument("--calib", default=bc.DEFAULT_CALIBRATION)
    ap.add_argument("--out", default=bc.DEFAULT_WORLD)
    ap.add_argument("--samples", type=int, default=SAMPLES)
    ap.add_argument("--min-corners", type=int, default=MIN_CORNERS,
                    help="corners each phone must see (default %d). A floor board is small and "
                         "foreshortened: if you cannot reach 30, lower this, but accuracy drops" % MIN_CORNERS)
    ap.add_argument("--force", action="store_true", help="save even if the phones disagree a lot")
    ap.add_argument("--single", action="store_true",
                    help="accept the board when only ONE phone sees it well (the other pose follows from the "
                         "stereo calibration). Use this when the two phones are toed in so much that no floor "
                         "spot is seen well by both. No phone-to-phone cross-check then; use the tape checks.")
    args = ap.parse_args()

    calib = bc.load_calibration(args.calib)
    board = bc.make_board(calib.square_m)
    detector = cv2.aruco.CharucoDetector(board)
    board_pts = bc.board_corner_points(board)
    cam = bc.SplitCamera(args.cam, calib.size)
    print(f"Calibration: {args.calib}  (baseline {calib.baseline_m:.3f} m, "
          f"each phone {calib.size[0]}x{calib.size[1]})")
    print("Board flat on the floor, taped down, visible to BOTH phones. S = save, Q = quit.\n")

    saved = False
    try:
        while not saved:
            ok, left, right, _ = cam.read()
            if not ok:
                print("Camera stopped delivering frames.")
                break
            dets = [bc.detect_board(detector, left), bc.detect_board(detector, right)]
            n0, n1 = len(dets[0][1]), len(dets[1][1])
            ok0, ok1 = n0 >= args.min_corners, n1 >= args.min_corners
            ready = (ok0 and ok1) or (args.single and (ok0 or ok1))
            need = "one phone is enough (--single)" if args.single else "each"
            lines, color, pose0 = [f"cam0 corners {n0}   cam1 corners {n1}   (need {args.min_corners}, {need})"], \
                (0, 165, 255), None
            if ready and not (ok0 and ok1):
                e = pose_single(dets[0], 0, calib, board_pts) if ok0 else pose_single(dets[1], 1, calib, board_pts)
                if e is not None:
                    pose0 = e if ok0 else None
                    lines += ["READY - single-phone mode (" + ("cam0" if ok0 else "cam1") + "), no cross-check",
                              "press S to collect and save"]
                    color = (0, 255, 255)
            elif ready:
                got = poses_in_cam0(dets[0], dets[1], calib, board_pts)
                if got is not None:
                    pose0 = got[0]
                    dt = np.linalg.norm(got[1][1] - got[0][1]) * 1000.0
                    dr = bc.rotation_angle_deg(got[0][0], got[1][0])
                    lines += [f"READY - single-frame phone disagreement {dt:.0f} mm / {dr:.1f} deg",
                              "press S to collect and save"]
                    color = (0, 255, 0)
            else:
                lines.append("move the board so BOTH phones see enough corners")
            cv2.imshow("BINgo world frame", draw((left, right), dets, calib, pose0, lines, color))
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s") and ready:
                res = collect(cam, calib, detector, board_pts, args.samples, args.min_corners, args.single)
                if res is not None:
                    saved = report_and_save(res[0], res[1], calib, args)
                if not saved:
                    print("Adjust the setup, then press S to try again.\n")
    finally:
        cam.release()
        cv2.destroyAllWindows()
    if not saved:
        print("World frame was not saved.")


if __name__ == "__main__":
    main()