// shared_utils.js
// Small parsing helpers, plus the single fallback geometry table used
// before a CONFIG message or config file has arrived.
//
// Not an ES module — loaded as a plain <script> so both module files
// (kinematics.js) and non-module files (robot_state.js, and eventually
// robot_bridge.js / config_loader.js / gamepad_bridge.js etc.) can reach it
// the same way, via window.SharedUtils, matching the rest of the frontend's
// script style.

(function () {

  function parseVec3(v, fallback) {
    fallback = fallback || [0, 0, 0];
    let out;
    if (Array.isArray(v)) out = v.map(Number);
    else if (typeof v === "string") out = v.trim().split(/\s+/).map(Number);
    else return fallback;
    return (out.length >= 3 && out.every(Number.isFinite)) ? out.slice(0, 3) : fallback;
  }

  function parseAxisAngle(v, fallback) {
    fallback = fallback || [0, 0, 1, 0];
    let out;
    if (Array.isArray(v)) out = v.map(Number);
    else if (typeof v === "string") out = v.trim().split(/\s+/).map(Number);
    else return fallback;
    return (out.length >= 4 && out.every(Number.isFinite)) ? out.slice(0, 4) : fallback;
  }

  function clamp(v, min, max) {
    return Math.max(min, Math.min(max, v));
  }

  function wrapDeg(a) {
    return ((a + 180) % 360 + 360) % 360 - 180;
  }

  // The ONLY hardcoded robot geometry left in the frontend. It exists purely
  // as a bootstrap so the scene has something to render before config/
  // default_config.json (or a CONFIG message from the backend) arrives.
  // Mirrors config/default_config.json — if you change one, change both,
  // or better, wire this to fetch that file directly instead of hardcoding
  // it here (see the follow-up note in the chat this came from).
  const FALLBACK_GEOMETRY = {
    translations_mm: {
      BaseTransform: [0, 0, 0],
      Joint1: [175, 0, 20],
      Joint2: [187, 0, 138],
      Joint3: [0, 0, 290],
      Joint4: [0, 0, 280],
      Joint5: [0, 0, 0],
      Joint6: [0, 0, 0],
      Ik_bone: [0, 0, 115],
    },
    joint_axes: {
      BaseTransform: [0, 0, 1],
      Joint1: [0, 0, 1],
      Joint2: [0, 1, 0],
      Joint3: [0, 1, 0],
      Joint4: [1, 0, 0],
      Joint5: [0, 1, 0],
      Joint6: [1, 0, 0],
    },
    joint_limits_deg: {
      j1: [-170, 170],
      j2: [-120, 120],
      j3: [-200, 70],
      j4: [-300, 300],
      j5: [-130, 130],
      j6: [-360, 360],
    },
    joint_type: {
      BaseTransform: "rotate",
      Joint1: "rotate",
      Joint2: "rotate",
      Joint3: "rotate",
      Joint4: "rotate",
      Joint5: "rotate",
      Joint6: "rotate",
    },
  };

  window.SharedUtils = { parseVec3, parseAxisAngle, clamp, wrapDeg, FALLBACK_GEOMETRY };

})();
