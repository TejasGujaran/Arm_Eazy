/**
 * mock_backend.js — browser port of bridge.py + ik.py ("backup IK")
 * ------------------------------------------------------------------
 * Loaded automatically by robot_bridge.js (connectWithFallback) only when
 * the Python WebSocket server is not reachable, e.g. on the static PFH
 * server. Must sit next to robot_bridge.js (frontend/transport/).
 *
 * Behaves like bridge.py:
 *   Browser → mock : JOINT_STATE, GRIPPER_POSE, CONFIG, IK_TARGET, SOLVE_IK_BUTTON
 *   mock → Browser : HEARTBEAT (1 Hz), SET_JOINTS
 *                    (+ IK_STATUS: status-line text, backup mode only)
 *
 * IK is a line-by-line port of ik.py:
 *   chain      BaseTransform → Joint1..Joint6 → Ik_bone   (config.translations_mm)
 *   FK         rotate:    M · T(base) · R(axis, q)
 *              translate: M · T(base + axis · q · joint_scale)
 *   residual   [p − target,  1000 · wrap(rpy − rpyTarget),  √1e-6 · (q − q0)]
 *   bounds     joint_limits_deg (deg for rotate, raw slider units for translate)
 *   success    |p − target| ≤ 0.5 mm and each RPY error ≤ 0.5°
 *   like bridge.py, the best joints found are always sent back as SET_JOINTS
 * ZCF is commented out in ik.py and PGJ4S is not implemented there, so
 * neither is used here either.
 *
 * Console:  MockBackend.status()   → config check + last IK result
 */
(function (global) {
  "use strict";

  const JOINT_KEYS = ["j1", "j2", "j3", "j4", "j5", "j6"];
  const CHAIN      = ["BaseTransform", "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"];
  const HEARTBEAT_MS = 1000;

  const W_ORIENT = 1000.0, W_REG = 1e-6;          // same weights as ik.py
  const POS_TOL_MM = 0.5, ORI_TOL_DEG = 0.5;      // same success check as ik.py
  const MAX_ITER = 500;                         // ik.py: max_nfev=500
  const MAX_STEP_SCALE = 2e5;                    // max step: 0.2 rad (rotate), 20 units (translate)

  const log = (...a) => console.info("[mock_backend]", ...a);

  class ConfigError extends Error {}

  // ---------------------------------------------------------------------------
  // 4×4 matrix helpers (row-major, 16 numbers) — same maths as ik.py _T / _R
  // ---------------------------------------------------------------------------
  const deg2rad = (d) => +d * Math.PI / 180;
  const rad2deg = (r) => +r * 180 / Math.PI;

  function T(tx, ty, tz) { return [1, 0, 0, tx, 0, 1, 0, ty, 0, 0, 1, tz, 0, 0, 0, 1]; }

  function R(ax, ay, az, ang) {
    const n = Math.hypot(ax, ay, az);
    if (n === 0) return T(0, 0, 0);
    ax /= n; ay /= n; az /= n;
    const c = Math.cos(ang), s = Math.sin(ang), t = 1 - c;
    return [t * ax * ax + c,      t * ax * ay - s * az, t * ax * az + s * ay, 0,
            t * ax * ay + s * az, t * ay * ay + c,      t * ay * az - s * ax, 0,
            t * ax * az - s * ay, t * ay * az + s * ax, t * az * az + c,      0,
            0, 0, 0, 1];
  }

  function mul(A, B) {
    const C = new Array(16);
    for (let i = 0; i < 4; i++)
      for (let j = 0; j < 4; j++)
        C[i * 4 + j] = A[i * 4] * B[j] + A[i * 4 + 1] * B[4 + j] + A[i * 4 + 2] * B[8 + j] + A[i * 4 + 3] * B[12 + j];
    return C;
  }

  // ik.py _rpy_from_matrix (ZYX)
  function rpyFromMatrix(M) {
    const R00 = M[0], R01 = M[1], R02 = M[2], R10 = M[4], R11 = M[5], R12 = M[6], R20 = M[8], R21 = M[9], R22 = M[10];
    const sy = Math.sqrt(R00 * R00 + R10 * R10);
    if (sy > 1e-6) return [Math.atan2(R21, R22), Math.atan2(-R20, sy), Math.atan2(R10, R00)];
    return [Math.atan2(-R12, R11), Math.atan2(-R20, sy), 0];
  }

  const wrap = (e) => ((e + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;  // numpy-style modulo

  // ---------------------------------------------------------------------------
  // Model from config — port of ik.py _build_model (required, no defaults)
  // ---------------------------------------------------------------------------
  function buildModel(config) {
    if (!config) throw new ConfigError("No config loaded — load a robot config first.");
    const cfgT = config.translations_mm, cfgA = config.joint_axes, cfgL = config.joint_limits_deg;
    if (!cfgT || !cfgA || !cfgL) throw new ConfigError("Config is missing translations_mm, joint_axes, or joint_limits_deg.");

    const translations = {};
    for (const name of [...CHAIN, "Ik_bone"]) {
      if (!(name in cfgT)) throw new ConfigError(`translations_mm is missing an entry for '${name}'.`);
      const v = cfgT[name]; translations[name] = [+v[0], +v[1], +v[2]];
    }
    const axes = {};
    for (const name of CHAIN.slice(1)) {
      if (!(name in cfgA)) throw new ConfigError(`joint_axes is missing an entry for '${name}'.`);
      const v = cfgA[name]; axes[name] = [+v[0], +v[1], +v[2]];
    }
    const limits = {};
    for (const k of JOINT_KEYS) {
      if (!(k in cfgL)) throw new ConfigError(`joint_limits_deg is missing an entry for '${k}'.`);
      limits[k] = [+cfgL[k][0], +cfgL[k][1]];
    }
    const cfgType = config.joint_type || {}, cfgScale = config.joint_scale || {};
    const jointType = {}, jointScale = {};
    CHAIN.slice(1).forEach((name, i) => {
      const k = `j${i + 1}`;
      jointType[k]  = cfgType[name] ?? "rotate";
      jointScale[k] = +(cfgScale[k] ?? 1.0);
    });
    return { translations, axes, limits, jointType, jointScale };
  }

  // ---------------------------------------------------------------------------
  // Forward kinematics — port of ik.py _fk
  // q[i]: radians for "rotate", raw slider units for "translate"
  // ---------------------------------------------------------------------------
  function fk(q, m) {
    let M = T(...m.translations.BaseTransform);
    CHAIN.slice(1).forEach((name, i) => {
      const k = `j${i + 1}`, axis = m.axes[name], base = m.translations[name], val = +q[i];
      if (m.jointType[k] === "translate") {
        const mm = val * m.jointScale[k];
        M = mul(M, T(base[0] + axis[0] * mm, base[1] + axis[1] * mm, base[2] + axis[2] * mm));
      } else {
        M = mul(mul(M, T(...base)), R(axis[0], axis[1], axis[2], val));
      }
    });
    return mul(M, T(...m.translations.Ik_bone));
  }
  const fkXyz = (M) => [M[3], M[7], M[11]];

  // ---------------------------------------------------------------------------
  // Bounded least squares (Levenberg–Marquardt with projection onto bounds)
  // standing in for scipy.optimize.least_squares(method="trf")
  // ---------------------------------------------------------------------------
  function solveLinear(A, b) {
    const n = b.length, a = A.map((r, i) => [...r, b[i]]);
    for (let c = 0; c < n; c++) {
      let p = c; for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
      [a[c], a[p]] = [a[p], a[c]];
      const d = a[c][c] || 1e-15;
      for (let r = c + 1; r < n; r++) { const f = a[r][c] / d; for (let k = c; k <= n; k++) a[r][k] -= f * a[c][k]; }
    }
    const x = new Array(n).fill(0);
    for (let r = n - 1; r >= 0; r--) { let s = a[r][n]; for (let k = r + 1; k < n; k++) s -= a[r][k] * x[k]; x[r] = s / (a[r][r] || 1e-15); }
    return x;
  }

  function leastSquares(residual, q0, lo, hi, steps) {
    const n = q0.length, clip = (q) => q.map((v, i) => Math.min(hi[i], Math.max(lo[i], v)));
    const cost = (r) => 0.5 * r.reduce((s, x) => s + x * x, 0);
    let q = clip(q0), r = residual(q), c = cost(r), lambda = 1e-3, it = 0;
    for (; it < MAX_ITER; it++) {
      // numerical Jacobian (central differences, step per joint type)
      const m = r.length, J = Array.from({ length: m }, () => new Array(n));
      for (let k = 0; k < n; k++) {
        const h = steps[k], qp = q.slice(), qm = q.slice();
        qp[k] += h; qm[k] -= h;
        const rp = residual(qp), rm = residual(qm);
        for (let i = 0; i < m; i++) J[i][k] = (rp[i] - rm[i]) / (2 * h);
      }
      const JtJ = Array.from({ length: n }, (_, a) => Array.from({ length: n }, (_, b) => {
        let s = 0; for (let i = 0; i < m; i++) s += J[i][a] * J[i][b]; return s; }));
      const g = Array.from({ length: n }, (_, a) => { let s = 0; for (let i = 0; i < m; i++) s += J[i][a] * r[i]; return s; });
      if (Math.max(...g.map(Math.abs)) < 1e-10) break;

      let accepted = false;
      for (let tries = 0; tries < 10 && !accepted; tries++) {
        const A = JtJ.map((row, a) => row.map((v, b) => v + (a === b ? lambda * (JtJ[a][a] + 1e-9) : 0)));
        let dq = solveLinear(A, g.map((v) => -v));
        // trust region: limit the step (in units of each joint's FD step size
        // scale) so one big step can't slam the arm into its joint limits
        const ratio = Math.max(...dq.map((d, k) => Math.abs(d) / (steps[k] * MAX_STEP_SCALE)));
        if (ratio > 1) dq = dq.map((d) => d / ratio);
        const qn = clip(q.map((v, k) => v + dq[k]));
        const rn = residual(qn), cn = cost(rn);
        if (cn < c) {
          const rel = (c - cn) / Math.max(c, 1e-30);
          q = qn; r = rn; c = cn; lambda = Math.max(1e-9, lambda / 3); accepted = true;
          if (rel < 1e-10) it = MAX_ITER;                 // converged (ftol-like)
        } else lambda *= 5;
      }
      if (!accepted) break;
    }
    return { x: q, iterations: Math.min(it, MAX_ITER) };
  }

  // ---------------------------------------------------------------------------
  // solve_ik — port of ik.py solve_ik (same inputs, same return shape)
  // ---------------------------------------------------------------------------
  function solveIk(jointAngles, config, targetXyz, targetRpy = null) {
    const m = buildModel(config);
    const target = [+targetXyz.x, +targetXyz.y, +targetXyz.z];

    const q0 = [], qMin = [], qMax = [], steps = [];
    JOINT_KEYS.forEach((k, i) => {
      const raw = +(jointAngles[k] ?? 0), [lo, hi] = m.limits[k];
      if (m.jointType[k] === "translate") { q0[i] = raw; qMin[i] = lo; qMax[i] = hi; steps[i] = 1e-4; }
      else { q0[i] = deg2rad(raw); qMin[i] = deg2rad(lo); qMax[i] = deg2rad(hi); steps[i] = 1e-6; }
    });
    for (let i = 0; i < 6; i++) {                                  // nudge seed inside bounds
      const span = qMax[i] - qMin[i];
      q0[i] = Math.min(qMax[i] - 1e-9 * span, Math.max(qMin[i] + 1e-9 * span, q0[i]));
    }

    const rpyTargets = [];                                         // [index, target_rad]
    if (targetRpy) ["roll", "pitch", "yaw"].forEach((role, idx) => {
      if (targetRpy[role] !== null && targetRpy[role] !== undefined) rpyTargets.push([idx, deg2rad(targetRpy[role])]);
    });
    const useOrientation = rpyTargets.length > 0, sqrtReg = Math.sqrt(W_REG);

    const residual = (q) => {
      const M = fk(q, m), p = fkXyz(M);
      const out = [p[0] - target[0], p[1] - target[1], p[2] - target[2]];
      if (useOrientation) {
        const ach = rpyFromMatrix(M);
        for (const [idx, tgt] of rpyTargets) out.push(W_ORIENT * wrap(ach[idx] - tgt));
      }
      for (let i = 0; i < 6; i++) out.push(sqrtReg * (q[i] - q0[i]));
      return out;
    };

    const res = leastSquares(residual, q0, qMin, qMax, steps);
    const qSol = res.x, Msol = fk(qSol, m), achieved = fkXyz(Msol);
    const [roll, pitch, yaw] = rpyFromMatrix(Msol);
    const errNorm = Math.hypot(achieved[0] - target[0], achieved[1] - target[1], achieved[2] - target[2]);
    let reached = errNorm <= POS_TOL_MM;
    if (useOrientation) {
      const ach = [roll, pitch, yaw];
      for (const [idx, tgt] of rpyTargets) if (Math.abs(rad2deg(wrap(ach[idx] - tgt))) > ORI_TOL_DEG) reached = false;
    }

    const joints = {};
    JOINT_KEYS.forEach((k, i) => { joints[k] = m.jointType[k] === "translate" ? qSol[i] : rad2deg(qSol[i]); });
    return {
      success: reached, joints,
      achieved: { x: achieved[0], y: achieved[1], z: achieved[2] },
      achievedRPY: { roll: rad2deg(roll), pitch: rad2deg(pitch), yaw: rad2deg(yaw) },
      errNorm, iterations: res.iterations,
    };
  }

  // ---------------------------------------------------------------------------
  // MockBackend — WebSocket-compatible stand-in for one bridge.py Session
  // ---------------------------------------------------------------------------
  class MockBackend extends EventTarget {
    constructor(url) {
      super();
      this.url = url; this.protocol = ""; this.isMock = true; this.readyState = 0;
      this.onopen = this.onmessage = this.onclose = this.onerror = null;
      // Session state, as in bridge.py
      this.jointState  = Object.fromEntries(JOINT_KEYS.map((k) => [k, 0]));
      this.gripperPose = { x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0 };
      this.config = null;
      this.solveIkPressed = false;
      this.nReceived = 0; this.nSolved = 0; this.last = null;
      MockBackend.instance = this;
      setTimeout(() => this._open(), 0);
    }

    _open() {
      if (this.readyState !== 0) return;
      this.readyState = 1;
      this._fire("open", new Event("open"));
      this._emit({ type: "HEARTBEAT" });
      this._hb = setInterval(() => this._emit({ type: "HEARTBEAT" }), HEARTBEAT_MS);
      log("offline mode — backup IK (port of ik.py) active");
    }

    send(data) {
      if (this.readyState !== 1) throw new DOMException("MockBackend is not open", "InvalidStateError");
      let msg; try { msg = JSON.parse(data); } catch (e) { log("Bad JSON:", e.message); return; }
      if (!msg || typeof msg !== "object") return;
      try { this._handle(msg); }
      catch (e) { console.error("[mock_backend] handler error (connection kept alive):", e); }
    }

    _handle(msg) {
      switch (msg.type) {
        case "JOINT_STATE":
          if (msg.joints && typeof msg.joints === "object") Object.assign(this.jointState, msg.joints);
          return;
        case "GRIPPER_POSE":
          for (const k of Object.keys(this.gripperPose)) if (k in msg) this.gripperPose[k] = msg[k];
          return;
        case "CONFIG":
          if (!msg.config || typeof msg.config !== "object") return log("CONFIG without a config object — ignored");
          this.config = msg.config;
          log("CONFIG received:", Object.keys(msg.config));
          try { buildModel(this.config); log("config OK for IK"); }
          catch (e) { console.warn("[mock_backend] config not usable for IK:", e.message); }
          return;
        case "IK_TARGET":
          return this._ikTarget(msg);
        case "SOLVE_IK_BUTTON":
          this.solveIkPressed = !!msg.pressed;
          return;
      }
    }

    _ikTarget(msg) {
      const t = msg.target;
      if (!t || !["x", "y", "z"].every((k) => k in t)) return log("IK_TARGET without a valid target — ignored");
      this.nReceived++;
      if (!this.config) {
        log("IK_TARGET received but no CONFIG yet — skipping");
        return this._status("no robot config loaded", "red");
      }
      // snapshot, as bridge.py: stored joint state overlaid with the seed sent now
      const seed = { ...this.jointState, ...(msg.seed || {}) };
      const rpy  = msg.useOrientation ? (msg.targetRPY || null) : null;

      const t0 = performance.now();
      let result;
      try { result = solveIk(seed, this.config, t, rpy); }
      catch (e) {
        if (e instanceof ConfigError) { log("IK solve skipped:", e.message); return this._status(e.message, "red"); }
        console.error("[mock_backend] IK error (connection kept alive):", e); return;
      }
      const ms = performance.now() - t0;
      this.nSolved++;
      for (const k of JOINT_KEYS) result.joints[k] = Math.round(result.joints[k] * 1e4) / 1e4;
      this.last = { target: { x: +t.x, y: +t.y, z: +t.z }, success: result.success,
                    errNorm: +result.errNorm.toFixed(4), achieved: result.achieved,
                    iterations: result.iterations, ms: +ms.toFixed(1) };
      log(`[ik] (${(+t.x).toFixed(1)}, ${(+t.y).toFixed(1)}, ${(+t.z).toFixed(1)})  ok=${result.success}  ` +
          `err=${result.errNorm.toFixed(4)}  ${ms.toFixed(0)} ms  [solved ${this.nSolved}/${this.nReceived} received]`);

      // Like bridge.py: always send the best solution found.
      this._emit({ type: "SET_JOINTS", joints: result.joints });
      this._status(result.success ? `solved (${result.errNorm.toFixed(2)} mm, ${ms.toFixed(0)} ms)`
                                  : `target not reachable — closest is ${result.errNorm.toFixed(0)} mm away`,
                   result.success ? "green" : "red");
    }

    // ---- plumbing ------------------------------------------------------------
    _status(text, colour) { this._emit({ type: "IK_STATUS", text, colour }); }

    _emit(obj) {
      if (this.readyState !== 1) return;
      this._fire("message", new MessageEvent("message", { data: JSON.stringify(obj) }));
    }

    _fire(name, ev) {
      const h = this["on" + name];
      if (typeof h === "function") { try { h.call(this, ev); } catch (e) { console.error(e); } }
      this.dispatchEvent(ev);
    }

    close(code = 1000, reason = "") {
      if (this.readyState >= 2) return;
      this.readyState = 2;
      clearInterval(this._hb);
      setTimeout(() => { this.readyState = 3; this._fire("close", new CloseEvent("close", { code, reason, wasClean: true })); }, 0);
    }
  }
  MockBackend.CONNECTING = 0; MockBackend.OPEN = 1; MockBackend.CLOSING = 2; MockBackend.CLOSED = 3;

  // Console helper: is the config usable, and does our FK match the twin right now?
  MockBackend.status = function () {
    const s = MockBackend.instance;
    if (!s) return "mock backend not active";
    let configCheck, fkVsTwin = null;
    try {
      const m = buildModel(s.config);
      configCheck = "OK";
      const q = JOINT_KEYS.map((k) => m.jointType[k] === "translate" ? +s.jointState[k] : deg2rad(s.jointState[k]));
      const p = fkXyz(fk(q, m)), g = s.gripperPose;
      fkVsTwin = { mock_fk: p.map((v) => +v.toFixed(2)), twin_gripper: [g.x, g.y, g.z],
                   diff_mm: +Math.hypot(p[0] - g.x, p[1] - g.y, p[2] - g.z).toFixed(3) };
    } catch (e) { configCheck = e.message; }
    return { config: configCheck, fkVsTwin, lastIK: s.last };
  };
  MockBackend._test = { solveIk, buildModel, fk, rpyFromMatrix };

  global.MockBackend = MockBackend;
})(window);