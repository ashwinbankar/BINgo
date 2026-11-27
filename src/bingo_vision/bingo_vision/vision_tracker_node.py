import argparse
import math
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node

import bingo_split_common as bc
import bingo_tracking as bt
from live_tracker_split import (
    Projector,
    TopDown,
    candidates,
    draw_views,
    load_timing,
    pick_pair,
)


class LatestReader:
    """Background camera reader that always exposes the freshest stereo frame."""

    def __init__(self, cam):
        self.cam = cam
        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._seq = 0
        self._frame = None
        self._stopped = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stopped:
            ok, left, right, t_raw = self.cam.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self._cond:
                self._seq += 1
                self._frame = (self._seq, left, right, t_raw)
                self._cond.notify_all()

    def get(self, last_seq, timeout=2.0):
        with self._cond:
            ok = self._cond.wait_for(
                lambda: self._stopped or self._seq > last_seq, timeout=timeout
            )
            if not ok or self._stopped or self._frame is None:
                return None
            return self._frame

    def stop(self):
        with self._cond:
            self._stopped = True
            self._cond.notify_all()
        self._thread.join(timeout=1.0)


def cam_to_robot(x_cam, y_cam, x_home, y_home, heading_deg, mirror_y=True):
    """Transform vision floor (X_cam, Y_cam) into BINgo odom FLU (x_robot, y_robot)."""
    th = math.radians(heading_deg)
    dx = x_cam - x_home
    dy = y_cam - y_home
    x_r = dx * math.cos(th) + dy * math.sin(th)
    y_r = (-dx * math.sin(th) + dy * math.cos(th)) * (-1.0 if mirror_y else 1.0)
    return x_r, y_r


class VisionTrackerNode(Node):
    def __init__(self, args):
        super().__init__("vision_tracker_node")
        self.args = args

        self.x_home, self.y_home, self.heading_deg = (
            float(v) for v in args.robot_start.split(",")
        )
        self.x_min, self.x_max, self.y_min, self.y_max = (
            float(v) for v in args.bounds.split(",")
        )

        self.target_pub = self.create_publisher(Point, "/target_point", 10)
        self.vis_pub = self.create_publisher(PointStamped, "/target_point_vis", 10)

        self.last_pub_xy = None
        self.last_pub_t = 0.0

        self.get_logger().info(
            f"vision_tracker_node ready | home=({self.x_home:.2f}, {self.y_home:.2f}, "
            f"{self.heading_deg:.0f} deg) | mirror_y={args.robot_mirror_y} | "
            f"bounds=[{self.x_min:.2f},{self.x_max:.2f}]x[{self.y_min:.2f},{self.y_max:.2f}] m | "
            f"catch_h={args.catch_h:.2f} m | dry_run_ball={args.dry_run_ball}"
        )

    def publish_goal(self, x_cam, y_cam, label="catch"):
        x_r, y_r = cam_to_robot(
            x_cam, y_cam, self.x_home, self.y_home, self.heading_deg, self.args.robot_mirror_y
        )
        x_c = max(self.x_min, min(self.x_max, x_r))
        y_c = max(self.y_min, min(self.y_max, y_r))

        now = time.perf_counter()
        if self.last_pub_xy is not None:
            moved = math.hypot(x_c - self.last_pub_xy[0], y_c - self.last_pub_xy[1])
            if moved < self.args.min_update_m and (now - self.last_pub_t) < 0.15:
                return

        pt = Point()
        pt.x = float(x_c)
        pt.y = float(y_c)
        pt.z = 0.0
        self.target_pub.publish(pt)

        vis = PointStamped()
        vis.header.stamp = self.get_clock().now().to_msg()
        vis.header.frame_id = "odom"
        vis.point = pt
        self.vis_pub.publish(vis)

        self.last_pub_xy = (x_c, y_c)
        self.last_pub_t = now
        self.get_logger().info(
            f"[{label}] cam=({x_cam:.2f},{y_cam:.2f}) -> robot=({x_c:.2f},{y_c:.2f}) m"
        )


def parse_args(argv=None):
    pkg_config = Path(__file__).resolve().parent.parent / "config"
    ap = argparse.ArgumentParser(description="BINgo stereo-vision ROS 2 tracker node")
    ap.add_argument("--cam", type=int, default=1)
    ap.add_argument("--calib", default=str(pkg_config / "stereo_charuco_calibration_split.npz"))
    ap.add_argument("--world", default=str(pkg_config / "bingo_world_frame_split.npz"))
    ap.add_argument("--timing", default=str(pkg_config / "timing_offset.json"))
    ap.add_argument("--no-timing", action="store_true")
    ap.add_argument("--model", default="best.pt", help="Path to YOLOv8 weights (best.pt)")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--device", default=None, help="cuda / cpu")
    ap.add_argument("--drag-k", type=float, default=0.21)
    ap.add_argument("--sigma-m", type=float, default=0.04)
    ap.add_argument("--ball-radius", type=float, default=0.033)
    ap.add_argument("--catch-h", type=float, default=0.14)
    ap.add_argument(
        "--robot-start",
        default="0.75,0.75,90",
        help='"X_home,Y_home,heading_deg" in vision floor frame',
    )
    ap.add_argument(
        "--robot-mirror-y",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Mirror Y axis for left-handed vision frame (default: True)",
    )
    ap.add_argument(
        "--bounds",
        default="-0.75,0.75,-0.75,0.75",
        help='"x_min,x_max,y_min,y_max" clamp in robot odom frame (m)',
    )
    ap.add_argument(
        "--min-update-m",
        type=float,
        default=0.03,
        help="Minimum target shift (m) before republishing within 150 ms",
    )
    ap.add_argument(
        "--dry-run-ball",
        action="store_true",
        help="Publish stationary/hand-held ball position without requiring a confirmed throw",
    )
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--display-every", type=int, default=3)
    return ap.parse_args(argv)


def main():
    from ultralytics import YOLO

    rclpy.init()
    args = parse_args(rclpy.utilities.remove_ros_args()[1:])

    if not Path(args.model).exists():
        raise SystemExit(f"YOLO weights not found: {args.model}")

    node = VisionTrackerNode(args)
    calib = bc.load_calibration(args.calib)
    world = bc.WorldFrame.load(args.world, calib)
    off, unc = load_timing(args.timing, args.no_timing)
    tracker = bt.Tracker(
        calib,
        world,
        offset_s=off,
        offset_unc_s=unc,
        sigma_m=args.sigma_m,
        drag_k=args.drag_k,
        ball_r=args.ball_radius,
        catch_h=args.catch_h,
    )

    model = YOLO(args.model)
    kw = dict(imgsz=args.imgsz, conf=args.conf, verbose=False)
    if args.device:
        kw["device"] = args.device

    cam = bc.SplitCamera(args.cam, calib.size)
    node.get_logger().info("Warming up YOLO model...")
    _, l_warm, r_warm, _ = cam.read()
    model.predict([l_warm, r_warm], **kw)

    topdown = TopDown(world, calib, half_extent=3.0)
    proj = Projector(calib, world)
    clock = bt.FrameClock()
    reader = LatestReader(cam)

    seq, frames, cur_throw = 0, 0, 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            p = reader.get(seq, timeout=2.0)
            if p is None:
                node.get_logger().warn("Camera stopped delivering frames.")
                break
            seq, left, right, t_raw = p
            t = clock.stamp(seq, t_raw)

            res = model.predict([left, right], **kw)
            det = None
            pick = pick_pair(calib, candidates(res[0]), candidates(res[1]))
            if pick is not None:
                det = (pick[0], pick[1])

            row = tracker.step(t, det)
            frames += 1

            tid = int(row["throw_id"])
            if tid and tid != cur_throw:
                cur_throw = tid
                node.last_pub_xy = None
                topdown.clear()
                node.get_logger().info(f"Throw {tid} confirmed")

            if tid:
                if not np.isnan(row["kf_catch_X"]):
                    node.publish_goal(row["kf_catch_X"], row["kf_catch_Y"], label=f"throw{tid}-catch")
                elif not np.isnan(row["kf_land_X"]):
                    node.publish_goal(row["kf_land_X"], row["kf_land_Y"], label=f"throw{tid}-land")
            elif args.dry_run_ball and row["accepted"] and not np.isnan(row["kf_X"]):
                node.publish_goal(row["kf_X"], row["kf_Y"], label="dry-run")

            if not args.no_display and frames % max(args.display_every, 1) == 0:
                cv2.imshow("BINgo tracker", draw_views(left, right, det, row, proj, args.ball_radius))
                cv2.imshow("BINgo top view", topdown.draw(row))
                k = cv2.waitKey(1) & 0xFF
                if k == ord("q"):
                    break
                if k == ord("r"):
                    tracker._reset()
                    topdown.clear()
                    node.last_pub_xy = None
                    node.get_logger().info("Tracker reset")
            elif not args.no_display:
                topdown.draw(row)
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        cam.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()