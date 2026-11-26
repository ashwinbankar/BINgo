"""
BINgo - live tracker (roadmap step 7): OBS side-by-side feed -> YOLO -> 3D floor position ->
Kalman filter -> landing prediction, with a CSV log for evaluate_throws.py.

Needs (all from the earlier steps, in the same folder):
    stereo_charuco_calibration_split.npz   from calibrate_split_charuco.py
    bingo_world_frame_split.npz            from define_world_frame_split.py
    timing_offset.json                     from check_timing_split.py flash  (optional but recommended)
    yolov8_model/weights/best.pt           your ball detector

What you see
    Left window : both phone images, the YOLO detection (green circle), the filtered position (yellow),
                  the predicted landing point (magenta cross) and the catch point (cyan cross).
    Map window  : top-down floor view: phones (triangles), floor-frame origin (O), the track, the
                  predicted landing. Grid lines every 0.5 m.
    Console     : one line per detected throw and its final predicted landing.
Keys: Q = quit and save, R = reset the tracker

Do a few throws, then:  python3 evaluate_throws.py track_log.csv --sweep-drag

Usage
    python3 live_tracker_split.py --cam 1 --device mps
"""

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import bingo_split_common as bc
import bingo_tracking as bt
from check_timing_split import LatestReader


# =====================================================================
# detection helpers
# =====================================================================

def candidates(result, top=3):
    """YOLO result -> up to `top` (x, y, conf) box centres, best confidence first."""
    if result.boxes is None or len(result.boxes) == 0:
        return []
    conf = result.boxes.conf.cpu().numpy()
    xyxy = result.boxes.xyxy.cpu().numpy()
    order = np.argsort(-conf)[:top]
    return [((xyxy[i, 0] + xyxy[i, 2]) / 2.0, (xyxy[i, 1] + xyxy[i, 3]) / 2.0, float(conf[i])) for i in order]


def pick_pair(calib, cand_l, cand_r):
    """Choose the left/right candidate pair whose rays agree best (smallest reprojection error).
    This rejects most false detections without any extra model. -> (pl, pr, conf) or None"""
    if not cand_l or not cand_r:
        return None
    pairs = [(a, b) for a in cand_l for b in cand_r]
    pl = np.array([[a[0], a[1]] for a, _ in pairs])
    pr = np.array([[b[0], b[1]] for _, b in pairs])
    _, err = bc.triangulate(calib, pl, pr)
    j = int(np.argmin(err))
    (a, b) = pairs[j]
    return (a[0], a[1]), (b[0], b[1]), min(a[2], b[2])


# =====================================================================
# drawing
# =====================================================================

class Projector:
    def __init__(self, calib, world):
        self.calib, self.world = calib, world
        self.rv1, _ = cv2.Rodrigues(calib.R)

    def to_pixels(self, X, Y, H):
        pw = np.array([X, Y, -H], float)
        pc0 = self.world.R_c0w @ pw + self.world.t_c0w
        a = cv2.projectPoints(pc0.reshape(1, 1, 3), np.zeros(3), np.zeros(3), self.calib.K0, self.calib.d0)[0].ravel()
        b = cv2.projectPoints(pc0.reshape(1, 1, 3), self.rv1, self.calib.T.reshape(3, 1), self.calib.K1,
                              self.calib.d1)[0].ravel()
        ok = pc0[2] > 0.1
        return (a, b) if ok else (None, None)


def cross(img, p, color, size=14, thick=2):
    if p is None or not np.all(np.isfinite(p)):
        return
    x, y = int(p[0]), int(p[1])
    cv2.line(img, (x - size, y), (x + size, y), color, thick)
    cv2.line(img, (x, y - size), (x, y + size), color, thick)


def draw_views(left, right, det, row, proj, r):
    views = [left.copy(), right.copy()]
    if det is not None:
        for v, p in zip(views, det):
            cv2.circle(v, (int(p[0]), int(p[1])), 14, (0, 255, 0), 2)
    show_kf = (row is not None and not np.isnan(row["kf_X"]) and row["state"] == "tracking"
               and (row["throw_id"] or row["accepted"]))
    if show_kf:
        pa, pb = proj.to_pixels(row["kf_X"], row["kf_Y"], row["kf_H"])
        for v, p in zip(views, (pa, pb)):
            if p is not None:
                cv2.circle(v, (int(p[0]), int(p[1])), 8, (0, 255, 255), -1)
        if row["throw_id"] and not np.isnan(row["kf_land_X"]):
            pa, pb = proj.to_pixels(row["kf_land_X"], row["kf_land_Y"], r)
            for v, p in zip(views, (pa, pb)):
                cross(v, p, (255, 0, 255))
        if row["throw_id"] and not np.isnan(row["kf_catch_X"]):
            pa, pb = proj.to_pixels(row["kf_catch_X"], row["kf_catch_Y"], 0.15)
            for v, p in zip(views, (pa, pb)):
                cross(v, p, (255, 255, 0), 10, 2)
    txt = "idle"
    if row is not None:
        txt = row["state"]
        if row["throw_id"]:
            txt += f"  throw {int(row['throw_id'])}"
            if not np.isnan(row["kf_land_in"]):
                txt += f"  lands in {row['kf_land_in']:.2f}s at ({row['kf_land_X']*100:.0f},{row['kf_land_Y']*100:.0f}) cm"
        elif row["reason"]:
            txt += f"  ({row['reason']})"
    cv2.putText(views[0], txt, (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(views[0], "green=YOLO  yellow=filter  magenta=predicted landing (only during a throw)  cyan=catch point",
                (15, views[0].shape[0] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return cv2.resize(cv2.hconcat(views), None, fx=0.55, fy=0.55, interpolation=cv2.INTER_AREA)


class TopDown:
    def __init__(self, world, calib, size=620, half_extent=3.0, center=None):
        c0 = world.cam0_position_world()
        c1 = world.cam1_position_world(calib)
        self.cams = [c0[:2], c1[:2]]
        mid = (c0[:2] + c1[:2]) / 2 if center is None else np.array(center, float)
        self.cx, self.cy, self.ext, self.size = mid[0], mid[1], half_extent, size
        self.trail = []
        self.land = None
        self.land_t = 0.0

    def px(self, X, Y):
        s = self.size / (2 * self.ext)
        return int(self.size / 2 + (X - self.cx) * s), int(self.size / 2 + (Y - self.cy) * s)

    def draw(self, row):
        img = np.full((self.size, self.size, 3), 25, np.uint8)
        g0 = np.floor((self.cx - self.ext) / 0.5) * 0.5
        for g in np.arange(g0, self.cx + self.ext + 0.5, 0.5):
            cv2.line(img, self.px(g, self.cy - self.ext), self.px(g, self.cy + self.ext), (45, 45, 45), 1)
        g0 = np.floor((self.cy - self.ext) / 0.5) * 0.5
        for g in np.arange(g0, self.cy + self.ext + 0.5, 0.5):
            cv2.line(img, self.px(self.cx - self.ext, g), self.px(self.cx + self.ext, g), (45, 45, 45), 1)
        cv2.putText(img, "O", self.px(0, 0), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.circle(img, self.px(0, 0), 4, (255, 255, 255), -1)
        for i, c in enumerate(self.cams):
            x, y = self.px(*c)
            cv2.drawMarker(img, (x, y), (0, 200, 255), cv2.MARKER_TRIANGLE_UP, 18, 2)
            cv2.putText(img, f"cam{i}", (x + 10, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        if (row is not None and not np.isnan(row["kf_X"]) and row["state"] == "tracking"
                and (row["throw_id"] or row["accepted"])):
            self.trail.append(self.px(row["kf_X"], row["kf_Y"]))
            self.trail = self.trail[-90:]
            if row["throw_id"] and not np.isnan(row["kf_land_X"]):
                self.land = self.px(row["kf_land_X"], row["kf_land_Y"])
                self.land_t = time.perf_counter()
        if len(self.trail) > 1:
            cv2.polylines(img, [np.array(self.trail, np.int32)], False, (0, 255, 255), 2)
        if self.land is not None and time.perf_counter() - self.land_t < 3.0:
            cross(img, self.land, (255, 0, 255), 12, 2)
        cv2.putText(img, "top view: X right, Y down (+Y toward the lower edge)", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        return img

    def clear(self):
        self.trail, self.land = [], None


# =====================================================================
# main
# =====================================================================

def load_timing(path, disabled):
    if disabled or not Path(path).exists():
        print("Timing: no timing_offset.json used (offset assumed 0). Run check_timing_split.py flash for the "
              "camera offset." if not disabled else "Timing: disabled.")
        return 0.0, None
    d = json.loads(Path(path).read_text())
    off = d["offset_ms"] / 1000.0
    unc = None
    if d.get("drifting"):
        unc = max(abs(d["min_ms"]), abs(d["max_ms"])) / 2000.0
        print(f"Timing: offset {d['offset_ms']:+.1f} ms but it DRIFTS ({d['min_ms']:+.0f}..{d['max_ms']:+.0f} ms); "
              "position noise is increased accordingly.")
    else:
        print(f"Timing: right image older by {d['offset_ms']:+.1f} ms (applied as half-offset on the time stamps).")
    return off, unc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=1)
    ap.add_argument("--calib", default=bc.DEFAULT_CALIBRATION)
    ap.add_argument("--world", default=bc.DEFAULT_WORLD)
    ap.add_argument("--timing", default="timing_offset.json")
    ap.add_argument("--no-timing", action="store_true")
    ap.add_argument("--model", default="yolov8_model/weights/best.pt")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--device", default=None, help="mps (Apple GPU) / cpu")
    ap.add_argument("--out", default="track_log.csv")
    ap.add_argument("--drag-k", type=float, default=0.0, help="air drag constant 1/m (from evaluate_throws --sweep-drag)")
    ap.add_argument("--sigma-m", type=float, default=0.04, help="3D measurement noise (m)")
    ap.add_argument("--ball-radius", type=float, default=0.033)
    ap.add_argument("--catch-h", type=float, default=0.15, help="height (m) at which the robot would catch")
    ap.add_argument("--map-extent", type=float, default=3.0, help="half-width of the top view (m)")
    ap.add_argument("--map-center", default=None, help='"x,y" in metres; default = midpoint of the phones')
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--display-every", type=int, default=3,
                    help="draw/show only every Nth processed frame (default 3). Drawing on a Mac is slow and was "
                         "halving the processing rate; the tracker itself still sees every frame")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics is not installed: pip install ultralytics")
    if not Path(args.model).exists():
        raise SystemExit(f"YOLO weights not found: {args.model}")

    calib = bc.load_calibration(args.calib)
    world = bc.WorldFrame.load(args.world, calib)
    off, unc = load_timing(args.timing, args.no_timing)
    tracker = bt.Tracker(calib, world, offset_s=off, offset_unc_s=unc, sigma_m=args.sigma_m,
                         drag_k=args.drag_k, ball_r=args.ball_radius, catch_h=args.catch_h)
    model = YOLO(args.model)
    kw = dict(imgsz=args.imgsz, conf=args.conf, verbose=False)
    if args.device:
        kw["device"] = args.device

    cam = bc.SplitCamera(args.cam, calib.size)
    print("Warming up the model ...")
    ok, l, r, _ = cam.read()
    model.predict([l, r], **kw)
    model.predict([l, r], **kw)

    center = tuple(float(v) for v in args.map_center.split(",")) if args.map_center else None
    topdown = TopDown(world, calib, half_extent=args.map_extent, center=center)
    proj = Projector(calib, world)
    clock = bt.FrameClock()
    reader = LatestReader(cam)

    fields = None
    fh = open(args.out, "w", newline="")
    writer = None
    seq, frames, start = 0, 0, time.perf_counter()
    tm = dict(yolo=0.0, track=0.0, log=0.0, show=0.0)
    first_seq = None
    last_pred, cur_throw, throws_seen = None, 0, 0
    print("Tracking. Throw the ball through the area seen by both phones.  Q = quit, R = reset.\n")
    try:
        while True:
            p = reader.get(seq, timeout=2.0)
            if p is None:
                print("Camera stopped delivering frames.")
                break
            seq, left, right, t_raw = p
            if first_seq is None:
                first_seq = seq
            t = clock.stamp(seq, t_raw)
            _a = time.perf_counter()
            res = model.predict([left, right], **kw)
            det = None
            conf = np.nan
            pick = pick_pair(calib, candidates(res[0]), candidates(res[1]))
            if pick is not None:
                det, conf = (pick[0], pick[1]), pick[2]
            _b = time.perf_counter()
            row = tracker.step(t, det)
            _c = time.perf_counter()
            row.update(seq=seq, conf=conf,
                       detL_x=det[0][0] if det else np.nan, detL_y=det[0][1] if det else np.nan,
                       detR_x=det[1][0] if det else np.nan, detR_y=det[1][1] if det else np.nan)
            if writer is None:
                fields = ["seq", "t", "detL_x", "detL_y", "detR_x", "detR_y", "conf"] + \
                         [k for k in row.keys() if k not in ("seq", "t", "detL_x", "detL_y", "detR_x", "detR_y", "conf")]
                writer = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
                writer.writeheader()
            writer.writerow(row)
            frames += 1
            if frames % 60 == 0:
                fh.flush()
            _d = time.perf_counter()
            tm["yolo"] += _b - _a; tm["track"] += _c - _b; tm["log"] += _d - _c

            tid = int(row["throw_id"])
            if tid and tid != cur_throw:
                cur_throw = tid
                throws_seen += 1
                topdown.clear()
                print(f"throw {tid} detected")
            if tid and not np.isnan(row["kf_land_X"]):
                last_pred = (tid, row["kf_land_X"], row["kf_land_Y"], row["kf_land_in"])
            if row["state"] in ("landed", "lost") and tid and last_pred and last_pred[0] == tid:
                print(f"  throw {tid} ended ({row['state']}); last predicted landing "
                      f"({last_pred[1]*100:.0f}, {last_pred[2]*100:.0f}) cm, made {last_pred[3]:.2f} s before landing")
                topdown.land, topdown.land_t = topdown.px(last_pred[1], last_pred[2]), time.perf_counter()

            if not args.no_display and frames % max(args.display_every, 1) == 0:
                _e = time.perf_counter()
                cv2.imshow("BINgo tracker", draw_views(left, right, det, row, proj, args.ball_radius))
                cv2.imshow("BINgo top view", topdown.draw(row))
                k = cv2.waitKey(1) & 0xFF
                tm["show"] += time.perf_counter() - _e
                if k == ord("q"):
                    break
                if k == ord("r"):
                    tracker._reset()
                    topdown.clear()
                    print("tracker reset")
            elif not args.no_display:
                topdown.draw(row)            # keep the trail/landing marker up to date without showing
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        cam.release()
        fh.close()
        cv2.destroyAllWindows()
    el = time.perf_counter() - start
    captured = (seq - first_seq + 1) if first_seq else 0
    print(f"\n{frames} frames in {el:.0f} s ({frames/max(el,1e-9):.1f} FPS processed), {throws_seen} throws detected.")
    if captured:
        print(f"Processed {100*frames/captured:.0f}% of the {captured} captured frames "
              f"(target ~100%: every skipped frame is a gap in the ball track).")
    if frames:
        print("Average time per frame (ms): " + ", ".join(f"{k} {1000*v/frames:.1f}" for k, v in tm.items())
              + f"  (display is averaged over all frames; shown every {args.display_every})")
    print(f"Log: {args.out}.  Next: python3 evaluate_throws.py {args.out} --sweep-drag")


if __name__ == "__main__":
    main()