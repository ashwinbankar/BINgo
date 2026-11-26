"""
BINgo - tracking core (no camera, no YOLO): everything that turns per-frame ball detections
into a filtered 3D state and a landing prediction. Used by live_tracker_split.py and by the tests.

Coordinates are the floor frame from define_world_frame_split.py, in METRES:
    X, Y along the floor, H = height above the floor (up). Gravity acts on H.

Pieces
  FrameClock         smooths frame timestamps (the OS read time jitters by a few ms)
  KalmanBall         6-state filter (position + velocity), gravity as a known input
  landing_from_state integrates a state forward (gravity + optional quadratic drag) to the floor
  ballistic_ls_fit   independent cross-check: least-squares fit to the last few raw points
  Tracker            gating, track start/stop, throw segmentation, predictions, one log row per frame

A crumpled paper ball is light, so air drag is NOT negligible (several m/s^2 at 5 m/s).
landing_from_state therefore takes a drag constant k (1/m): a = g - k*|v|*v.  k = 0 is plain
ballistic. evaluate_throws.py can estimate k from your logged throws.
"""

from collections import deque

import numpy as np

import bingo_split_common as bc

G = 9.81


# =====================================================================
# time
# =====================================================================

class FrameClock:
    """Frames arrive at a steady rate; the read time does not. Fit time = a + b*seq over the
    recent frames and use the fitted value. Falls back to the raw time for the first few frames."""

    def __init__(self, window=90):
        self.buf = deque(maxlen=window)

    def stamp(self, seq, t_raw):
        self.buf.append((seq, t_raw))
        if len(self.buf) < 12:
            return t_raw
        s = np.array([b[0] for b in self.buf], float)
        t = np.array([b[1] for b in self.buf], float)
        if s[-1] - s[0] < 10:
            return t_raw
        b, a = np.polyfit(s - s[0], t, 1)
        fit = a + b * (seq - s[0])
        # a big step (camera stall, long pause) invalidates the fit: trust the raw time then
        if abs(fit - t_raw) > 0.05:
            self.buf.clear()
            self.buf.append((seq, t_raw))
            return t_raw
        return float(fit)


# =====================================================================
# filter
# =====================================================================

class KalmanBall:
    def __init__(self, t, p, v, sigma_p, sigma_v):
        self.t = t
        self.x = np.concatenate([p, v]).astype(float)
        self.P = np.diag([sigma_p ** 2] * 3 + [sigma_v ** 2] * 3)

    @staticmethod
    def _g():
        return np.array([0.0, 0.0, -G])

    def predict(self, t_new, sigma_acc, gravity=True):
        dt = t_new - self.t
        if dt <= 0:
            return
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        g = self._g() if gravity else np.zeros(3)
        self.x = F @ self.x + np.concatenate([0.5 * g * dt * dt, g * dt])
        q = sigma_acc ** 2
        I = np.eye(3)
        Q = q * np.block([[I * dt ** 4 / 4, I * dt ** 3 / 2], [I * dt ** 3 / 2, I * dt ** 2]])
        self.P = F @ self.P @ F.T + Q
        self.t = t_new

    def innovation(self, z, sigma_m):
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        y = np.asarray(z, float) - H @ self.x
        S = H @ self.P @ H.T + np.eye(3) * sigma_m ** 2
        return y, S, H

    def update(self, z, sigma_m):
        y, S, H = self.innovation(z, sigma_m)
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    @property
    def p(self):
        return self.x[:3]

    @property
    def v(self):
        return self.x[3:]


# =====================================================================
# prediction
# =====================================================================

def landing_from_state(p, v, k=0.0, r=0.033, catch_h=None, dt=0.004, tmax=3.0):
    """Integrate from position p (X,Y,H) and velocity v with a = (0,0,-g) - k|v|v until the ball
    centre reaches height r. Returns dict(x, y, t_land, [x_c, y_c, t_c]) or None (no landing within
    tmax, or the ball is rising away). t_* are seconds from now. x_c.. is where the descending
    ball crosses catch_h."""
    x, y, h = float(p[0]), float(p[1]), float(p[2])
    vx, vy, vh = float(v[0]), float(v[1]), float(v[2])
    t = 0.0
    out = {}
    if h <= r and vh <= 0:
        return dict(x=x, y=y, t_land=0.0)
    while t < tmax:
        sp = (vx * vx + vy * vy + vh * vh) ** 0.5
        ax, ay, ah = -k * sp * vx, -k * sp * vy, -G - k * sp * vh
        # midpoint (RK2) step
        vx2, vy2, vh2 = vx + 0.5 * dt * ax, vy + 0.5 * dt * ay, vh + 0.5 * dt * ah
        sp2 = (vx2 * vx2 + vy2 * vy2 + vh2 * vh2) ** 0.5
        ax2, ay2, ah2 = -k * sp2 * vx2, -k * sp2 * vy2, -G - k * sp2 * vh2
        nx, ny, nh = x + dt * vx2, y + dt * vy2, h + dt * vh2
        nvx, nvy, nvh = vx + dt * ax2, vy + dt * ay2, vh + dt * ah2
        if catch_h is not None and "x_c" not in out and vh < 0 and h > catch_h >= nh:
            f = (h - catch_h) / max(h - nh, 1e-12)
            out.update(x_c=x + f * (nx - x), y_c=y + f * (ny - y), t_c=t + f * dt)
        if nh <= r and nvh < 0:
            f = (h - r) / max(h - nh, 1e-12)
            out.update(x=x + f * (nx - x), y=y + f * (ny - y), t_land=t + f * dt)
            return out
        x, y, h, vx, vy, vh = nx, ny, nh, nvx, nvy, nvh
        t += dt
    return None


def ballistic_ls_fit(ts, pts, t_ref=None):
    """Least squares p(t) = p0 + v0*tau + 0.5*g*tau^2, tau = t - t_ref, g known.
    Returns (p0, v0) at t_ref and g_est from a free-g fit of the height (a sanity check:
    far from 9.8 means bad timing, bad scale or strong drag)."""
    ts = np.asarray(ts, float)
    P = np.asarray(pts, float)
    t_ref = ts[-1] if t_ref is None else t_ref
    tau = ts - t_ref
    A = np.stack([np.ones_like(tau), tau], 1)
    rhs = P - np.outer(0.5 * tau ** 2, [0, 0, -G])
    sol, *_ = np.linalg.lstsq(A, rhs, rcond=None)
    g_est = float("nan")
    if len(ts) >= 6 and np.ptp(tau) > 0.15:
        c = np.polyfit(tau, P[:, 2], 2)
        g_est = float(-2.0 * c[0])
    return sol[0], sol[1], g_est


# =====================================================================
# tracker
# =====================================================================

class Tracker:
    def __init__(self, calib, world, offset_s=0.0, sigma_m=0.04, sigma_acc=4.0, drag_k=0.0,
                 ball_r=0.033, catch_h=0.15, gate_chi2=25.0, max_coast_s=0.25, min_speed=2.0,
                 max_speed=15.0, fit_window=12, max_reproj_px=15.0, h_range=(-0.10, 3.0), xy_box=None,
                 min_updates_for_prediction=5, offset_unc_s=None,
                 min_travel=0.12, min_updates_confirm=5, coast_unconfirmed_s=0.1, sigma_acc_free=8.0):
        self.calib, self.world = calib, world
        self.offset_s = offset_s
        self.offset_unc_s = abs(offset_s) / 2.0 if offset_unc_s is None else offset_unc_s
        self.sigma_m, self.sigma_acc, self.k = sigma_m, sigma_acc, drag_k
        self.r, self.catch_h = ball_r, catch_h
        self.gate_chi2, self.max_coast = gate_chi2, max_coast_s
        self.min_speed, self.max_speed = min_speed, max_speed
        self.fit_window, self.max_reproj = fit_window, max_reproj_px
        self.h_range, self.xy_box = h_range, xy_box
        self.min_upd = min_updates_for_prediction
        self.min_travel, self.min_upd_confirm = min_travel, min_updates_confirm
        self.coast_unconfirmed, self.sigma_acc_free = coast_unconfirmed_s, sigma_acc_free
        self.throw_count = 0
        self._depth = 3.0
        self._reset()

    def _reset(self):
        self.kf = None
        self.tent = None
        self.confirmed = False
        self.throw_id = 0
        self.samples = []         # (t, X, Y, H) accepted raw measurements of this track
        self.last_accept_t = None
        self.n_upd = 0
        self.cooldown_until = -1e9

    # ---- measurement ------------------------------------------------
    def measure(self, det):
        """det = ((xL,yL),(xR,yR)) pixels -> (X,Y,H,reproj_px, ok, why)"""
        if det is None:
            return None, 0.0, False, "no detection"
        xyz, err = bc.triangulate(self.calib, [det[0]], [det[1]])
        self._depth = float(xyz[0, 2])
        x, y, h = self.world.to_xyh(xyz)
        p = np.array([x[0], y[0], h[0]])
        e = float(err[0])
        if xyz[0, 2] <= 0:
            return p, e, False, "behind camera"
        if e > self.max_reproj:
            # a fast ball seen by two phones that are not exactly in sync looks "inconsistent" by
            # speed * time offset; only a LARGE disagreement is a wrong match
            return p, e, False, f"views disagree ({e:.1f}px)"
        if not (self.h_range[0] <= p[2] <= self.h_range[1]):
            return p, e, False, "height out of range"
        if self.xy_box is not None:
            (x0, x1), (y0, y1) = self.xy_box
            if not (x0 <= p[0] <= x1 and y0 <= p[1] <= y1):
                return p, e, False, "outside workspace"
        return p, e, True, ""

    def _sigma(self):
        s = self.sigma_m
        if self.kf is not None and self.offset_unc_s:
            s = float(np.hypot(s, np.linalg.norm(self.kf.v) * self.offset_unc_s))
        return s

    # ---- one frame --------------------------------------------------
    def step(self, t_frame, det):
        """t_frame: smoothed capture time of the composite frame (s). Returns a dict (one log row)."""
        tm = t_frame - 0.5 * self.offset_s        # average capture time of the two images
        p, reproj, ok, why = self.measure(det)
        row = dict(t=t_frame, X=np.nan, Y=np.nan, H=np.nan, reproj_px=reproj if det else np.nan,
                   meas_ok=int(ok), reason=why, accepted=0, state="idle", throw_id=0,
                   kf_X=np.nan, kf_Y=np.nan, kf_H=np.nan, kf_vX=np.nan, kf_vY=np.nan, kf_vH=np.nan,
                   kf_land_X=np.nan, kf_land_Y=np.nan, kf_land_in=np.nan,
                   kf_catch_X=np.nan, kf_catch_Y=np.nan, kf_catch_in=np.nan,
                   ls_land_X=np.nan, ls_land_Y=np.nan, ls_land_in=np.nan, g_est=np.nan)
        if p is not None:
            row.update(X=p[0], Y=p[1], H=p[2])

        if tm < self.cooldown_until:
            row["state"] = "cooldown"
            return row

        # --- no track yet
        if self.kf is None:
            if self.tent is not None and tm - self.tent[0] > 0.2:
                self.tent = None
            if ok:
                if self.tent is None:
                    self.tent = (tm, p)
                    row["state"] = "tentative"
                else:
                    t0, p0 = self.tent
                    dt = tm - t0
                    if dt > 0.005 and np.linalg.norm(p - p0) / dt <= self.max_speed:
                        v2 = (p - p0) / dt                                # no gravity model until a throw is confirmed
                        sv = self.sigma_m * 1.5 / dt
                        self.kf = KalmanBall(tm, p, v2, self.sigma_m, sv)
                        self.samples = [(t0, *p0), (tm, *p)]
                        self.last_accept_t, self.n_upd = tm, 2
                        row["state"] = "tracking"
                        row["accepted"] = 1
                    else:
                        self.tent = (tm, p)
                        row["state"] = "tentative"
            self._fill_state(row)
            return row

        # --- track exists
        self.kf.predict(tm, self.sigma_acc if self.confirmed else self.sigma_acc_free, gravity=self.confirmed)
        accepted = False
        if ok:
            sig = float(np.hypot(self._sigma(), reproj * self._depth / self.calib.K0[0, 0]))
            y, S, _ = self.kf.innovation(p, sig)
            d2 = float(y @ np.linalg.solve(S, y))
            if d2 <= self.gate_chi2:
                self.kf.update(p, sig)
                self.samples.append((tm, *p))
                self.last_accept_t, self.n_upd = tm, self.n_upd + 1
                accepted = True
            else:
                row["reason"] = f"rejected by gate (d2={d2:.0f})"
        row["accepted"] = int(accepted)
        row["state"] = "tracking"

        if len(self.samples) > 60 and not self.confirmed:
            self.samples = self.samples[-30:]
        if not self.confirmed and accepted and self._looks_like_throw(tm):
            self.confirmed = True
            self.throw_count += 1
            self.throw_id = self.throw_count
        row["throw_id"] = self.throw_id if self.confirmed else 0

        if self.confirmed and self.n_upd >= self.min_upd:
            self._predict(row, tm)
        self._fill_state(row)

        landed = self.confirmed and self.kf.p[2] <= self.r + 0.04 and self.kf.v[2] < 0
        lost = tm - self.last_accept_t > (self.max_coast if self.confirmed else self.coast_unconfirmed)
        if landed or lost:
            row["state"] = "landed" if landed else "lost"
            cool = tm + (0.6 if self.confirmed else 0.0)
            self._reset()
            self.cooldown_until = cool
        return row

    def _looks_like_throw(self, tm):
        """Decide from the MEASURED points only (never from the filter, whose own gravity model can
        make a resting ball look like it is falling): enough consecutive fresh points, real
        travel, real speed, and clearly above the floor."""
        S = self.samples[-5:]
        if self.n_upd < self.min_upd_confirm or len(S) < 5 or tm - self.last_accept_t > 0.07:
            return False
        a = np.array(S)
        tt = a[:, 0] - a[:, 0].mean()
        if np.ptp(tt) < 0.05:
            return False
        v = np.array([np.polyfit(tt, a[:, i], 1)[0] for i in (1, 2, 3)])
        travel = float(np.linalg.norm(a[-1, 1:] - a[0, 1:]))
        return (float(np.linalg.norm(v)) >= self.min_speed and travel >= self.min_travel
                and a[-1, 3] > 2 * self.r)

    def _fill_state(self, row):
        if self.kf is not None:
            row.update(kf_X=self.kf.p[0], kf_Y=self.kf.p[1], kf_H=self.kf.p[2],
                       kf_vX=self.kf.v[0], kf_vY=self.kf.v[1], kf_vH=self.kf.v[2])

    def _predict(self, row, tm):
        out = landing_from_state(self.kf.p, self.kf.v, self.k, self.r, self.catch_h)
        if out is not None:
            row.update(kf_land_X=out["x"], kf_land_Y=out["y"], kf_land_in=out["t_land"])
            if "x_c" in out:
                row.update(kf_catch_X=out["x_c"], kf_catch_Y=out["y_c"], kf_catch_in=out["t_c"])
        S = self.samples[-self.fit_window:]
        if len(S) >= 4:
            a = np.array(S)
            p0, v0, g_est = ballistic_ls_fit(a[:, 0], a[:, 1:], t_ref=tm)
            ls = landing_from_state(p0, v0, self.k, self.r)
            row["g_est"] = g_est
            if ls is not None:
                row.update(ls_land_X=ls["x"], ls_land_Y=ls["y"], ls_land_in=ls["t_land"])