"""
ik.py
=====
Standalone inverse kinematics solver.

Config is always required — there are no hardcoded geometry defaults here
anymore. If no CONFIG message has arrived for a session yet, solve_ik raises
ConfigError rather than silently solving against made-up dimensions that may
not match the arm actually connected on the frontend. This mirrors
kinematics.js on the JS side, which uses the same translations_mm /
joint_axes / joint_type / joint_scale fields from the same config object
(sent to the backend verbatim in a CONFIG message) — so the two FK
implementations can never drift apart on defaults, because neither has any.

Wrist singularity handling (near J5 = 0°)
──────────────────────────────────────────
Two approaches have been designed for this arm; both are documented here.
Only ZCF is currently wired into the solver below.

  ZCF (Zero-Cross J5 Flip) — ACTIVE, implemented below.
    Solve normally first. If the result lands with |j5| < 2°, re-solve with
    j5's bounds locked to whichever side of zero it approached from
    (based on the previous j5 value), so the solver physically cannot
    return a j5 inside the forbidden zone. Other joints compensate
    naturally. No changes needed outside this file.

  PGJ4S (Proximity-Graduated J4 Suppression) — documented, NOT implemented.
    When j5 is within 15° of zero, add a graduated penalty weight on j4 to
    the residual, growing from ~0 at the 15° boundary to a large value at
    j5 = 0. This shifts wrist rotation responsibility from j4 to j6 as j5
    approaches zero, freezing j4 at its zone-entry value and letting j6
    handle all incremental rotation inside the zone. Symmetric in both
    approach directions; only penalizes j4 where it's geometrically
    redundant, so no position accuracy is sacrificed. Would require
    tracking each session's zone-entry j4 value (a per-session concern,
    natural home would be session.py) if it's ever wired in alongside or
    instead of ZCF.

Main function
─────────────
    result = solve_ik(
        joint_angles,   # current joint values  : {"j1".."j6"} — degrees for
                        #                          "rotate" joints, raw
                        #                          slider units for
                        #                          "translate" joints
        config,         # REQUIRED. dict with joint_axes, translations_mm,
                        #           joint_limits_deg, joint_type, joint_scale
        target_xyz,     # target position        : {"x","y","z"} mm
        target_rpy,     # target orientation     : {"roll","pitch","yaw"} degrees
                        #                          pass None to solve position-only
    )

Returns
───────
    {
        "success"    : bool,
        "joints"     : {"j1".."j6"},            ← send this back as SET_JOINTS
        "achieved"   : {"x","y","z"},            ← actual gripper position reached
        "achievedRPY": {"roll","pitch","yaw"},   ← actual orientation reached
        "errNorm"    : float,                     ← position error in mm
        "message"    : str,
    }

Raises
──────
    ConfigError  if config is missing, or missing required fields, for any
    joint in the chain. Callers (session.py) should catch this and skip the
    solve rather than let it crash the connection.
"""

import numpy as np
from scipy.optimize import least_squares

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

JOINT_KEYS = ["j1", "j2", "j3", "j4", "j5", "j6"]
CHAIN      = ["BaseTransform", "Joint1", "Joint2", "Joint3",
              "Joint4", "Joint5", "Joint6"]


class ConfigError(Exception):
    """Raised when solve_ik is called without a complete config for this session."""
    pass


# ---------------------------------------------------------------------------
# Unit helpers
# ---------------------------------------------------------------------------

def _deg2rad(d): return float(d) * np.pi / 180.0
def _rad2deg(r): return float(r) * 180.0 / np.pi

# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _T(tx, ty, tz):
    M = np.eye(4, dtype=float)
    M[0, 3], M[1, 3], M[2, 3] = tx, ty, tz
    return M

def _R(ax, ay, az, ang):
    axis = np.array([ax, ay, az], dtype=float)
    n = np.linalg.norm(axis)
    if n == 0.0:
        return np.eye(4, dtype=float)
    ax, ay, az = axis / n
    c, s, t = np.cos(ang), np.sin(ang), 1.0 - np.cos(ang)
    M = np.eye(4, dtype=float)
    M[:3, :3] = [
        [t*ax*ax + c,     t*ax*ay - s*az,  t*ax*az + s*ay],
        [t*ax*ay + s*az,  t*ay*ay + c,     t*ay*az - s*ax],
        [t*ax*az - s*ay,  t*ay*az + s*ax,  t*az*az + c   ],
    ]
    return M

def _rpy_from_matrix(R):
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    if sy > 1e-6:
        roll  = np.arctan2( R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = np.arctan2( R[1, 0], R[0, 0])
    else:
        roll  = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw   = 0.0
    return roll, pitch, yaw

# ---------------------------------------------------------------------------
# Build model from config — REQUIRED, no fallback defaults
# ---------------------------------------------------------------------------

def _build_model(config):
    """
    Extract translations, axes, limits, joint type, and joint scale from the
    session's config (as sent over CONFIG). Raises ConfigError if config is
    missing or incomplete — see module docstring for why there's no
    fallback here.
    """
    if not config:
        raise ConfigError(
            "No config loaded for this session — send a CONFIG message before IK_TARGET."
        )

    cfg_t = config.get("translations_mm")
    cfg_a = config.get("joint_axes")
    cfg_l = config.get("joint_limits_deg")
    if not cfg_t or not cfg_a or not cfg_l:
        raise ConfigError(
            "Config is missing translations_mm, joint_axes, or joint_limits_deg."
        )

    translations = {}
    for name in CHAIN + ["Ik_bone"]:
        if name not in cfg_t:
            raise ConfigError(f"translations_mm is missing an entry for '{name}'.")
        v = cfg_t[name]
        translations[name] = (float(v[0]), float(v[1]), float(v[2]))

    axes = {}
    for name in CHAIN[1:]:
        if name not in cfg_a:
            raise ConfigError(f"joint_axes is missing an entry for '{name}'.")
        v = cfg_a[name]
        axes[name] = (float(v[0]), float(v[1]), float(v[2]))

    limits = {}
    for k in JOINT_KEYS:
        if k not in cfg_l:
            raise ConfigError(f"joint_limits_deg is missing an entry for '{k}'.")
        limits[k] = (float(cfg_l[k][0]), float(cfg_l[k][1]))

    # joint_type / joint_scale are optional — default to "rotate" / 1.0,
    # matching kinematics.js's fallback for the same fields.
    cfg_type  = config.get("joint_type", {})
    cfg_scale = config.get("joint_scale", {})
    joint_type, joint_scale = {}, {}
    for i, name in enumerate(CHAIN[1:], start=1):
        jkey = f"j{i}"
        joint_type[jkey]  = cfg_type.get(name, "rotate")
        joint_scale[jkey] = float(cfg_scale.get(jkey, 1.0))

    return translations, axes, limits, joint_type, joint_scale

# ---------------------------------------------------------------------------
# Forward kinematics — dispatches per joint on joint_type
# ---------------------------------------------------------------------------

def _fk(q, translations, axes, joint_type, joint_scale):
    """Returns 4x4 world transform of the Ik_bone tip. q[i] is radians for
    "rotate" joints, raw slider units for "translate" joints."""
    M = _T(*translations["BaseTransform"])
    for i, name in enumerate(CHAIN[1:], start=1):
        jkey = f"j{i}"
        axis = axes[name]
        base = translations[name]
        val  = float(q[i - 1])
        if joint_type[jkey] == "translate":
            mm = val * joint_scale[jkey]
            M = M @ _T(base[0] + axis[0]*mm, base[1] + axis[1]*mm, base[2] + axis[2]*mm)
        else:
            M = M @ _T(*base) @ _R(*axis, val)
    M = M @ _T(*translations["Ik_bone"])
    return M

def _fk_xyz(q, translations, axes, joint_type, joint_scale):
    return (_fk(q, translations, axes, joint_type, joint_scale) @ np.array([0., 0., 0., 1.]))[:3]

# ---------------------------------------------------------------------------
# Analytical Jacobian (position only) — per-joint dispatch on joint_type
# ---------------------------------------------------------------------------

def _jacobian(q, translations, axes, joint_type, joint_scale):
    J = np.zeros((3, 6), dtype=float)
    M = _T(*translations["BaseTransform"])
    origins, world_axes, types, scales = [], [], [], []

    for i, name in enumerate(CHAIN[1:], start=1):
        jkey = f"j{i}"
        axis = axes[name]
        base = translations[name]
        val  = float(q[i - 1])

        # The joint rotates about its own origin, i.e. AFTER its base offset
        # (_fk does M @ T(base) @ R). Using M's origin here (before the
        # offset) made the analytic Jacobian wrong by up to ~290 mm/rad.
        origins.append((M @ _T(*base))[:3, 3].copy())
        world_axes.append(M[:3, :3] @ np.array(axis, dtype=float))
        types.append(joint_type[jkey])
        scales.append(joint_scale[jkey])

        if joint_type[jkey] == "translate":
            mm = val * joint_scale[jkey]
            M = M @ _T(base[0] + axis[0]*mm, base[1] + axis[1]*mm, base[2] + axis[2]*mm)
        else:
            M = M @ _T(*base) @ _R(*axis, val)

    tip = (M @ _T(*translations["Ik_bone"]) @ np.array([0., 0., 0., 1.]))[:3]

    for i in range(6):
        if types[i] == "translate":
            # d(position)/d(q) for a prismatic joint is just its world-frame
            # axis scaled by joint_scale — no origin/tip subtraction needed.
            J[:, i] = world_axes[i] * scales[i]
        else:
            J[:, i] = np.cross(world_axes[i], tip - origins[i])

    return J

def _angular_jacobian(q, translations, axes, joint_type):
    """World-frame rotation axis of each revolute joint (0 for prismatic)."""
    Jw = np.zeros((3, 6), dtype=float)
    M = _T(*translations["BaseTransform"])
    for i, name in enumerate(CHAIN[1:], start=1):
        jkey = f"j{i}"
        axis = np.array(axes[name], dtype=float)
        n = np.linalg.norm(axis)
        if joint_type[jkey] != "translate" and n > 0:
            Jw[:, i - 1] = M[:3, :3] @ (axis / n)
            M = M @ _T(*translations[name]) @ _R(*axes[name], float(q[i - 1]))
        else:
            M = M @ _T(*translations[name])  # rotation unchanged by translation
    return Jw

# ---------------------------------------------------------------------------
# Main solver
# ---------------------------------------------------------------------------

def solve_ik(joint_angles, config, target_xyz, target_rpy=None):
    """
    Solve inverse kinematics. See module docstring for parameters/return
    value/raised exceptions.
    """

    translations, axes, limits, joint_type, joint_scale = _build_model(config)

    target = np.array([target_xyz["x"], target_xyz["y"], target_xyz["z"]], dtype=float)

    q0    = np.zeros(6, dtype=float)
    q_min = np.zeros(6, dtype=float)
    q_max = np.zeros(6, dtype=float)

    for i, k in enumerate(JOINT_KEYS):
        raw    = joint_angles.get(k, 0.0)
        lo, hi = limits[k]
        if joint_type[k] == "translate":
            # Raw slider units — no radian conversion, joint_scale already
            # applied inside _fk/_jacobian.
            q0[i], q_min[i], q_max[i] = raw, lo, hi
        else:
            q0[i]    = _deg2rad(raw)
            q_min[i] = _deg2rad(lo)
            q_max[i] = _deg2rad(hi)

    # least_squares refuses a seed outside the bounds. Rounded SET_JOINTS
    # values (4 dp) or a slider at its end stop can land a hair past a
    # limit, so nudge the seed just inside.
    span = q_max - q_min
    q0 = np.clip(q0, q_min + 1e-9 * span, q_max - 1e-9 * span)

    # Orientation targets (world frame)
    w_orient = 1000.0
    w_reg    = 1e-6
    rpy_targets = []  # list of (role, target_rad, weight)

    if target_rpy is not None:
        for role in ("roll", "pitch", "yaw"):
            if target_rpy.get(role) is not None:
                rpy_targets.append((role, _deg2rad(target_rpy[role]), w_orient))

    use_orientation = bool(rpy_targets)

    def residual(q):
        M_full  = _fk(q, translations, axes, joint_type, joint_scale)
        p       = (M_full @ np.array([0., 0., 0., 1.]))[:3]
        pos_err = p - target

        orient_err = []
        if use_orientation:
            r, p_, y = _rpy_from_matrix(M_full[:3, :3])
            achieved = {"roll": r, "pitch": p_, "yaw": y}
            for role, tgt, w in rpy_targets:
                err = achieved[role] - tgt
                err = (err + np.pi) % (2 * np.pi) - np.pi
                orient_err.append(w * err)

        reg = np.sqrt(w_reg) * (q - q0)
        return np.concatenate([pos_err, orient_err, reg])

    def jac(q):
        n_orient = len(rpy_targets)
        J_full   = np.zeros((3 + n_orient + 6, 6), dtype=float)
        J_full[:3, :] = _jacobian(q, translations, axes, joint_type, joint_scale)
        if n_orient == 3:
            # Analytic: d(rpy)/dq = E(rpy)^-1 * w, with w the joint's world
            # rotation axis (0 for prismatic joints). Replaces 7 extra FK
            # evaluations per Jacobian with one cheap matrix product.
            M_full = _fk(q, translations, axes, joint_type, joint_scale)
            r, p_, _y = _rpy_from_matrix(M_full[:3, :3])
            Jw = _angular_jacobian(q, translations, axes, joint_type)
            cp = np.cos(p_)
            if abs(cp) > 1e-3:
                sr, cr, tp = np.sin(r), np.cos(r), np.tan(p_)
                cy, sy = np.cos(_y), np.sin(_y)
                # ZYX Euler rates from world angular velocity
                Einv = np.array([
                    [cy / cp,       sy / cp,       0.0],
                    [-sy,           cy,            0.0],
                    [cy * tp,       sy * tp,       1.0],
                ])
                order = {"roll": 0, "pitch": 1, "yaw": 2}
                D = Einv @ Jw
                for row, (role, _t, w) in enumerate(rpy_targets):
                    J_full[3 + row, :] = w * D[order[role], :]
                J_full[3+n_orient:, :] = np.sqrt(w_reg) * np.eye(6)
                return J_full
        if use_orientation:
            eps, r0 = 1e-6, residual(q)
            for j in range(6):
                dq = np.zeros(6); dq[j] = eps
                r1 = residual(q + dq)
                J_full[3:3+n_orient, j] = (r1[3:3+n_orient] - r0[3:3+n_orient]) / eps
        J_full[3+len(rpy_targets):, :] = np.sqrt(w_reg) * np.eye(6)
        return J_full

    res = least_squares(
        residual, x0=q0, jac=jac,
        bounds=(q_min, q_max),
        method="trf",
        ftol=1e-5, xtol=1e-5, gtol=1e-5,
        max_nfev=500,
    )

    q_sol = res.x

    """
    # ZCF (Zero-Cross J5 Flip) — see module docstring. Only meaningful for
    # a rotate-type j5; a translate-type j5 has no angular wrist singularity
    # to avoid, so skip this entirely in that case.
    if joint_type["j5"] == "rotate" and abs(_rad2deg(q_sol[4])) < 2.0:
        last_j5 = joint_angles.get("j5", 0.0)
        if last_j5 < -1.0:
            q_min[4] = _deg2rad(1.0)
            q_max[4] = _deg2rad(120.0)
        else:
            q_min[4] = _deg2rad(-120.0)
            q_max[4] = _deg2rad(-1.0)

        # x0 must lie within the (now narrowed) bounds or least_squares
        # raises "Initial guess is outside of provided bounds". The original
        # seed q0 is very likely to fall inside the just-excluded zone
        # (that's exactly why ZCF triggered), so clip it before retrying.
        q0_retry = np.clip(q0, q_min, q_max)

        res = least_squares(
            residual, x0=q0_retry, jac=jac,
            bounds=(q_min, q_max),
            method="trf",
            ftol=1e-5, xtol=1e-5, gtol=1e-5,
            max_nfev=500,
        )
        q_sol = res.x
    """
    M_sol    = _fk(q_sol, translations, axes, joint_type, joint_scale)
    achieved = (M_sol @ np.array([0., 0., 0., 1.]))[:3]
    err      = achieved - target
    roll, pitch, yaw = _rpy_from_matrix(M_sol[:3, :3])

    # least_squares' own .success only means "a stopping criterion was met"
    # — it is True even for unreachable targets. Report whether the target
    # was actually reached instead.
    POS_TOL_MM, ORI_TOL_DEG = 0.5, 0.5
    err_norm = float(np.linalg.norm(err))
    reached  = err_norm <= POS_TOL_MM
    if use_orientation:
        ach = {"roll": roll, "pitch": pitch, "yaw": yaw}
        for role, tgt, _w in rpy_targets:
            d = (ach[role] - tgt + np.pi) % (2 * np.pi) - np.pi
            if abs(_rad2deg(d)) > ORI_TOL_DEG:
                reached = False

    joints_out = {}
    for i, k in enumerate(JOINT_KEYS):
        joints_out[k] = float(q_sol[i]) if joint_type[k] == "translate" else _rad2deg(q_sol[i])

    return {
        "success":     bool(reached),
        "joints":      joints_out,
        "achieved":    {"x": float(achieved[0]), "y": float(achieved[1]), "z": float(achieved[2])},
        "achievedRPY": {"roll": _rad2deg(roll), "pitch": _rad2deg(pitch), "yaw": _rad2deg(yaw)},
        "errNorm":     err_norm,
        "message":     str(res.message),
    }