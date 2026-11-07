// robot_state.js
// Single source of truth for joint values + config on the frontend.
//
// Every input adapter (sliders, gamepad_bridge.js, home_controller.js,
// armAnimation.js / motion_recorder.js) should read and write through this
// object instead of touching the X3D DOM, slider elements, or labels
// directly. render() is the ONLY function that writes to the DOM — it
// replaces _applyOneJoint/applyAnglesDeg (robot_bridge.js),
// writePoseToDOM (armAnimation.js), setSliderLimitsAndValue
// (config_loader.js), and the badge-update logic that was inline in
// twin.html.
//
// NOTE: this file is not yet wired into twin.html or called by the other
// scripts — that integration (updating robot_bridge.js, config_loader.js,
// armAnimation.js, gamepad_bridge.js, home_controller.js, and the slider
// oninput handlers to go through window.RobotState) is a follow-up step.

(function () {
  const JOINT_KEYS = ["j1", "j2", "j3", "j4", "j5", "j6"];
  const NODE_NAMES = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"];

  const state = {
    config: null,   // full arm config, once set via setConfig()
    joints: {
      j1: { valueDeg: 0 }, j2: { valueDeg: 0 }, j3: { valueDeg: 0 },
      j4: { valueDeg: 0 }, j5: { valueDeg: 0 }, j6: { valueDeg: 0 },
    },
  };

  function _fallback() {
    return (window.SharedUtils && window.SharedUtils.FALLBACK_GEOMETRY) || {
      translations_mm: {}, joint_axes: {}, joint_limits_deg: {}, joint_type: {},
    };
  }

  function _axisFor(nodeName) {
    const cfg = state.config;
    if (cfg && cfg.joint_axes && cfg.joint_axes[nodeName]) return cfg.joint_axes[nodeName];
    return _fallback().joint_axes[nodeName] || [0, 0, 1];
  }

  function _typeFor(nodeName) {
    const cfg = state.config;
    if (cfg && cfg.joint_type && cfg.joint_type[nodeName]) return cfg.joint_type[nodeName];
    return _fallback().joint_type[nodeName] || "rotate";
  }

  function _scaleFor(jKey) {
    const cfg = state.config;
    if (cfg && cfg.joint_scale && cfg.joint_scale[jKey] != null) return cfg.joint_scale[jKey];
    return 1.0; // mm per degree — matches ik.py's joint_scale default
  }

  function _baseTranslationFor(nodeName) {
    const cfg = state.config;
    if (cfg && cfg.translations_mm && cfg.translations_mm[nodeName]) return cfg.translations_mm[nodeName];
    return _fallback().translations_mm[nodeName] || [0, 0, 0];
  }

  function _limitsFor(jKey) {
    const cfg = state.config;
    if (cfg && cfg.joint_limits_deg && cfg.joint_limits_deg[jKey]) return cfg.joint_limits_deg[jKey];
    return _fallback().joint_limits_deg[jKey] || [-180, 180];
  }

  // ---- Writers ----

  function setConfig(cfg) {
    state.config = cfg || null;
    render(); // limits, axes, and joint types may all have changed
  }

  function setJointValue(jKey, valueDeg, opts) {
    opts = opts || {};
    if (!state.joints[jKey]) return;
    state.joints[jKey].valueDeg = Number(valueDeg) || 0;
    if (opts.render !== false) render();
  }

  function applyAnglesDeg(anglesObj) {
    JOINT_KEYS.forEach((jKey) => {
      if (anglesObj[jKey] !== undefined) {
        state.joints[jKey].valueDeg = Number(anglesObj[jKey]) || 0;
      }
    });
    render();
  }

  function applyPose(pose) {
    JOINT_KEYS.forEach((jKey) => {
      if (pose[jKey] && pose[jKey].valueDeg !== undefined) {
        state.joints[jKey].valueDeg = pose[jKey].valueDeg;
      }
    });
    render();
  }

  // ---- Readers ----

  function getJointsDeg() {
    const out = {};
    JOINT_KEYS.forEach((jKey) => { out[jKey] = state.joints[jKey].valueDeg; });
    return out;
  }

  function getConfig() {
    return state.config;
  }

  // Snapshot for armAnimation.js / motion_recorder.js / home_controller.js —
  // replaces reading rotation/translation attributes back out of the DOM.
  function getPose() {
    const pose = {};
    JOINT_KEYS.forEach((jKey, i) => {
      const nodeName = NODE_NAMES[i];
      pose[jKey] = {
        valueDeg: state.joints[jKey].valueDeg,
        axis: _axisFor(nodeName),
        type: _typeFor(nodeName),
      };
    });
    return pose;
  }

  // ---- The one function allowed to write to the DOM ----

  function render() {
    JOINT_KEYS.forEach((jKey, i) => {
      const nodeName = NODE_NAMES[i];
      const el     = document.getElementById(nodeName);
      const slider = document.getElementById(jKey);
      const label  = document.getElementById(jKey + "Angle");
      const badge  = document.getElementById(jKey + "TypeBadge");

      const deg  = state.joints[jKey].valueDeg;
      const axis = _axisFor(nodeName);
      const type = _typeFor(nodeName);

      if (el) {
        if (type === "translate") {
          const base = _baseTranslationFor(nodeName);
          const mm = deg * _scaleFor(jKey);
          el.setAttribute(
            "translation",
            `${base[0] + axis[0] * mm} ${base[1] + axis[1] * mm} ${base[2] + axis[2] * mm}`
          );
        } else {
          const rad = deg * Math.PI / 180;
          el.setAttribute("rotation", `${axis[0]} ${axis[1]} ${axis[2]} ${rad}`);
        }
      }

      if (slider) {
        const [minDeg, maxDeg] = _limitsFor(jKey);
        slider.min = minDeg;
        slider.max = maxDeg;
        slider.value = deg;
      }

      if (label) label.textContent = `${Math.round(deg)}°`;

      if (badge) {
        const isTranslate = type === "translate";
        badge.textContent = isTranslate ? "Translate" : "Rotate";
        badge.className = "joint-type-badge " + (isTranslate ? "translate" : "rotate");
      }
    });
  }

  window.RobotState = {
    setConfig, setJointValue, applyAnglesDeg, applyPose,
    getJointsDeg, getConfig, getPose,
    render,
  };
})();
