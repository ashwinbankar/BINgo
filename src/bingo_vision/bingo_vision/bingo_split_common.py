import sys
"""
BINgo - shared code for the OBS side-by-side pipeline

Everything that every later script needs lives here, so the camera handling,
calibration loading, triangulation and floor-frame maths exist exactly once.

    SplitCamera        reads the OBS Virtual Camera (2560x720) and splits it into
                       the left phone (cam0) and right phone (cam1).
                       It NEVER forces a size with cap.set().
    Calibration        loaded from stereo_charuco_calibration_split.npz
    triangulate()      pixels in both phones -> XYZ in the cam0 frame (metres)
    WorldFrame         cam0 frame <-> floor frame, saved/loaded as an .npz

Conventions (kept from your earlier scripts so nothing downstream changes meaning)
    cam frame   OpenCV: x right, y down, z forward, metres
    p_cam1 = R @ p_cam0 + T                      (R, T from the stereo calibration)
    world       origin = outer corner of the floor ChArUco board
                X, Y along the board, Z = board normal pointing INTO the floor
                height above floor = -Z
    p_cam0 = R_c0w @ p_world + t_c0w             (saved in the world-frame file)
"""

from dataclasses import dataclass
import time

import cv2
import numpy as np

DEFAULT_CALIBRATION = "stereo_charuco_calibration_split.npz"
DEFAULT_WORLD = "bingo_world_frame_split.npz"
ASPECT_TOL = 0.02          # canvas must be 32:9 (two 16:9 phones side by side)

SQUARES_X, SQUARES_Y = 12, 8
MARKER_RATIO = 32.0 / 44.0  # marker / square, from your printed board


# =====================================================================
# calibration
# =====================================================================

@dataclass
class Calibration:
    K0: np.ndarray
    d0: np.ndarray
    K1: np.ndarray
    d1: np.ndarray
    R: np.ndarray          # cam0 -> cam1 rotation
    T: np.ndarray          # cam0 -> cam1 translation, shape (3,)
    size: tuple            # (w, h) of EACH phone image
    square_m: float
    path: str = ""

    @property
    def baseline_m(self):
        return float(np.linalg.norm(self.T))


def load_calibration(path=DEFAULT_CALIBRATION):
    try:
        c = np.load(path)
    except FileNotFoundError:
        raise SystemExit(f"Cannot find '{path}'. Run calibrate_split_charuco.py first.")
    sq = float(c["square_length_m"].reshape(-1)[0]) if "square_length_m" in c.files else 0.044
    return Calibration(
        K0=c["K0"].astype(np.float64), d0=c["dist0"].astype(np.float64),
        K1=c["K1"].astype(np.float64), d1=c["dist1"].astype(np.float64),
        R=c["R"].astype(np.float64), T=c["T"].astype(np.float64).reshape(3),
        size=tuple(int(v) for v in c["image_size"]), square_m=sq, path=str(path))


# =====================================================================
# camera
# =====================================================================

def split_frame(frame):
    """Split the OBS canvas into (left, right). Raises if it is not 32:9."""
    h, w = frame.shape[:2]
    if abs((w / h) / (32 / 9) - 1.0) > ASPECT_TOL:
        raise ValueError(f"Canvas {w}x{h} is not 32:9 (two 16:9 phones side by side).")
    half = w // 2
    return frame[:, :half], frame[:, half:2 * half]


class SplitCamera:
    """The OBS Virtual Camera, split into two phone images.

    Size is READ, never requested: on macOS cap.set(width/height) can silently
    switch the virtual camera to another format (you saw 1920x540 that way).
    The delivered half size must equal the calibration's image size.
    """

    def __init__(self, index, calib_size=None):
        backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_V4L2
        self.cap = cv2.VideoCapture(index, backend)
        if not self.cap.isOpened():
            raise SystemExit(f"Could not open camera {index}. Is the OBS Virtual Camera on? "
                             "Do not use --list: it steals feeds from OBS.")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        time.sleep(1.0)
        ok, first = self.cap.read()
        if not ok:
            self.cap.release()
            raise SystemExit("Camera opened but returned no frame.")
        try:
            left, _ = split_frame(first)
        except ValueError as e:
            self.cap.release()
            raise SystemExit(str(e))
        self.canvas = (first.shape[1], first.shape[0])
        self.size = (left.shape[1], left.shape[0])
        if calib_size is not None and self.size != tuple(calib_size):
            self.cap.release()
            raise SystemExit(f"Each phone image is {self.size[0]}x{self.size[1]} now, but the "
                             f"calibration is {calib_size[0]}x{calib_size[1]}. Positions would "
                             "be wrong. Fix the OBS canvas, or recalibrate.")

    def read(self):
        """-> (ok, left, right, t) with t = time.perf_counter() right after the read."""
        ok, frame = self.cap.read()
        t = time.perf_counter()
        if not ok:
            return False, None, None, t
        left, right = split_frame(frame)
        return True, left, right, t

    def release(self):
        self.cap.release()


# =====================================================================
# triangulation
# =====================================================================

def triangulate(calib, p0, p1):
    """Pixels (N,2) in each phone -> XYZ (N,3) in the cam0 frame, metres, plus
    the worst reprojection error (px) over the two phones for every point."""
    p0 = np.asarray(p0, np.float64).reshape(-1, 1, 2)
    p1 = np.asarray(p1, np.float64).reshape(-1, 1, 2)
    n0 = cv2.undistortPoints(p0, calib.K0, calib.d0).reshape(-1, 2).T
    n1 = cv2.undistortPoints(p1, calib.K1, calib.d1).reshape(-1, 2).T
    P0 = np.hstack([np.eye(3), np.zeros((3, 1))])
    P1 = np.hstack([calib.R, calib.T.reshape(3, 1)])
    h = cv2.triangulatePoints(P0, P1, n0, n1)
    xyz = (h[:3] / h[3]).T

    rv, _ = cv2.Rodrigues(calib.R)
    q0, _ = cv2.projectPoints(xyz.reshape(-1, 1, 3), np.zeros(3), np.zeros(3), calib.K0, calib.d0)
    q1, _ = cv2.projectPoints(xyz.reshape(-1, 1, 3), rv, calib.T.reshape(3, 1), calib.K1, calib.d1)
    e0 = np.linalg.norm(q0.reshape(-1, 2) - p0.reshape(-1, 2), axis=1)
    e1 = np.linalg.norm(q1.reshape(-1, 2) - p1.reshape(-1, 2), axis=1)
    return xyz, np.maximum(e0, e1)


# =====================================================================
# board pose + averaging
# =====================================================================

def make_board(square_m):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), square_m,
                                  square_m * MARKER_RATIO, d)


def board_corner_points(board):
    return np.asarray(board.getChessboardCorners(), np.float64).reshape(-1, 3)


def detect_board(detector, img):
    """-> (corners Nx2 float32, ids N int32), empty arrays if nothing is found."""
    empty = (np.empty((0, 2), np.float32), np.empty((0,), np.int32))
    try:
        c, ids, _, _ = detector.detectBoard(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
    except cv2.error:
        return empty
    if c is None or ids is None or len(ids) == 0:
        return empty
    c = np.asarray(c, np.float32).reshape(-1, 2)
    ids = np.asarray(ids, np.int32).reshape(-1)
    n = min(len(c), len(ids))
    return c[:n], ids[:n]


def board_pose(corners, ids, board_pts, K, dist):
    """Board -> camera pose from one image. Returns (R, t(3,), rms_px) or None."""
    obj = board_pts[ids].reshape(-1, 1, 3)
    img = np.asarray(corners, np.float64).reshape(-1, 1, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or tvec[2, 0] <= 0:
        return None
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj - img) ** 2, axis=2))))
    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.reshape(3), rms


def pose_cam1_to_cam0(R1, t1, calib):
    """A board pose measured in cam1 (p_cam1 = R1 pb + t1), expressed in cam0.
    p_cam0 = R^T (p_cam1 - T) = (R^T R1) pb + R^T (t1 - T)."""
    return calib.R.T @ R1, calib.R.T @ (t1 - calib.T)


def rotation_angle_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def mean_rotation(rots):
    U, _, Vt = np.linalg.svd(np.mean(np.stack(rots), axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def robust_pose_average(rots, trans, rot_floor_deg=0.5):
    """Average many pose estimates, dropping frames that are far from the rest.

    NOTE: averaging only removes RANDOM frame-to-frame noise. A bias shared by every
    frame (calibration error, a warped board) is not reduced at all.
    """
    rots = list(rots)
    trans = np.asarray(trans, np.float64).reshape(len(rots), 3)
    Rm = mean_rotation(rots)
    dev = np.array([rotation_angle_deg(Rm, R) for R in rots])
    keep = dev <= max(rot_floor_deg, 3.0 * float(np.median(dev)))
    if keep.sum() < max(5, len(rots) // 2):
        keep[:] = True
    kept = [R for R, k in zip(rots, keep) if k]
    Rm = mean_rotation(kept)
    tm = np.median(trans[keep], axis=0)
    rot_spread = float(np.sqrt(np.mean([rotation_angle_deg(Rm, R) ** 2 for R in kept])))
    t_spread = float(np.sqrt(np.mean(np.sum((trans[keep] - tm) ** 2, axis=1))) * 1000.0)
    return Rm, tm, dict(n_kept=int(keep.sum()), n_total=len(rots),
                        rot_spread_deg=rot_spread, trans_spread_mm=t_spread)


# =====================================================================
# world frame
# =====================================================================

@dataclass
class WorldFrame:
    """p_cam0 = R_c0w @ p_world + t_c0w"""
    R_c0w: np.ndarray
    t_c0w: np.ndarray

    def world_from_cam0(self, p_cam0):
        p = np.asarray(p_cam0, np.float64).reshape(-1, 3)
        return (p - self.t_c0w.reshape(1, 3)) @ self.R_c0w          # R^T (p - t)

    def to_xyh(self, p_cam0):
        """cam0 point(s) -> (X, Y, height above floor), same units as the input."""
        w = self.world_from_cam0(p_cam0)
        return w[:, 0], w[:, 1], -w[:, 2]

    def cam0_position_world(self):
        return self.world_from_cam0(np.zeros(3))[0]

    def cam1_position_world(self, calib):
        return self.world_from_cam0(-calib.R.T @ calib.T)[0]

    def save(self, path, calib, extra=None):
        np.savez(path, R_cam0_from_world=self.R_c0w, t_cam0_from_world=self.t_c0w.reshape(3, 1),
                 image_size=np.array(calib.size), square_length_m=np.array([calib.square_m]),
                 calibration_file=calib.path,
                 convention="p_cam0 = R @ p_world + t; world Z points into the floor; "
                            "height = -Z; origin = outer corner of the floor board",
                 **(extra or {}))

    @staticmethod
    def load(path=DEFAULT_WORLD, calib=None):
        try:
            w = np.load(path)
        except FileNotFoundError:
            raise SystemExit(f"Cannot find '{path}'. Run define_world_frame_split.py first.")
        if "R_cam0_from_world" not in w.files:
            raise SystemExit(f"'{path}' is from the OLD pipeline (it has no cam0 pose). "
                             "Run define_world_frame_split.py to make a new one.")
        if calib is not None and tuple(int(v) for v in w["image_size"]) != tuple(calib.size):
            raise SystemExit("World frame and calibration have different image sizes. Redo the world frame.")
        return WorldFrame(w["R_cam0_from_world"].astype(np.float64),
                          w["t_cam0_from_world"].astype(np.float64).reshape(3))
