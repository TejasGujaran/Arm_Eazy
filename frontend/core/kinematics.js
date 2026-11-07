// kinematics.js
// Forward kinematics for the Ik_bone (gripper) tip.
//
// Reads joint values and config from window.RobotState (the frontend's
// single source of truth) instead of parsing X3D DOM rotation/translation
// attributes back out — the DOM is a render target now, not something we
// read from. This also means prismatic (translate) joints are handled
// correctly here, which the old DOM-parsing version did not do.

const CHAIN = ["BaseTransform", "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"];
const JOINT_KEYS_BY_INDEX = ["j1", "j2", "j3", "j4", "j5", "j6"]; // CHAIN[i] pairs with this[i-1]

function I() {
  return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
}

// 4x4 multiply: C = A * B  (row-major)
function mul4(A, B) {
  const C = new Array(16).fill(0);
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      C[r*4 + c] =
        A[r*4 + 0] * B[0*4 + c] +
        A[r*4 + 1] * B[1*4 + c] +
        A[r*4 + 2] * B[2*4 + c] +
        A[r*4 + 3] * B[3*4 + c];
    }
  }
  return C;
}

function T(tx, ty, tz) {
  return [1,0,0,tx, 0,1,0,ty, 0,0,1,tz, 0,0,0,1];
}

// Rotation matrix from axis-angle (Rodrigues), axis need not be normalized
function R_axisAngle(x, y, z, a) {
  const len = Math.hypot(x, y, z) || 1;
  x /= len; y /= len; z /= len;
  const c = Math.cos(a), s = Math.sin(a), t = 1 - c;
  return [
    t*x*x + c,     t*x*y - s*z,   t*x*z + s*y,   0,
    t*x*y + s*z,   t*y*y + c,     t*y*z - s*x,   0,
    t*x*z - s*y,   t*y*z + s*x,   t*z*z + c,     0,
    0,             0,             0,             1
  ];
}

function applyToPoint(M, p) {
  const [x, y, z] = p;
  return [
    M[0]*x + M[1]*y + M[2]*z + M[3],
    M[4]*x + M[5]*y + M[6]*z + M[7],
    M[8]*x + M[9]*y + M[10]*z + M[11]
  ];
}

// Extract RPY from a 4x4 rotation matrix (ZYX Euler / XYZ extrinsic)
function rpyFromMatrix(M) {
  const sy = Math.hypot(M[0], M[4]);
  if (sy > 1e-6) {
    return {
      roll:  Math.atan2( M[9],  M[10]) * 180 / Math.PI,
      pitch: Math.atan2(-M[8],  sy)    * 180 / Math.PI,
      yaw:   Math.atan2( M[4],  M[0])  * 180 / Math.PI
    };
  }
  return {
    roll:  Math.atan2(-M[6],  M[5])  * 180 / Math.PI,
    pitch: Math.atan2(-M[8],  sy)    * 180 / Math.PI,
    yaw:   0
  };
}

function _fallback() {
  return (window.SharedUtils && window.SharedUtils.FALLBACK_GEOMETRY) ||
    { translations_mm: {}, joint_axes: {}, joint_type: {} };
}

function _translationFor(name, config) {
  const raw = config && config.translations_mm && config.translations_mm[name];
  return raw ? window.SharedUtils.parseVec3(raw) : (_fallback().translations_mm[name] || [0, 0, 0]);
}

function _axisFor(name, config) {
  const raw = config && config.joint_axes && config.joint_axes[name];
  return raw ? window.SharedUtils.parseVec3(raw) : (_fallback().joint_axes[name] || [0, 0, 1]);
}

function _typeFor(name, config) {
  return (config && config.joint_type && config.joint_type[name]) || _fallback().joint_type[name] || "rotate";
}

function _scaleFor(jKey, config) {
  return (config && config.joint_scale && config.joint_scale[jKey]) || 1.0;
}

export function getGPosXYZ() {
  const rs = window.RobotState;
  if (!rs) return null;

  const config = rs.getConfig();
  const joints = rs.getJointsDeg(); // {j1..j6} degrees (or mm-equivalent degrees for translate joints)

  let M = I();

  CHAIN.forEach((name, i) => {
    const base = _translationFor(name, config);

    if (i === 0) {
      // BaseTransform: translation only in this model, no joint attached
      M = mul4(M, T(base[0], base[1], base[2]));
      return;
    }

    const jKey = JOINT_KEYS_BY_INDEX[i - 1];
    const axis = _axisFor(name, config);
    const type = _typeFor(name, config);
    const val  = joints[jKey] || 0;

    let local;
    if (type === "translate") {
      const mm = val * _scaleFor(jKey, config);
      local = T(base[0] + axis[0]*mm, base[1] + axis[1]*mm, base[2] + axis[2]*mm);
    } else {
      const rad = val * Math.PI / 180;
      local = mul4(T(base[0], base[1], base[2]), R_axisAngle(axis[0], axis[1], axis[2], rad));
    }
    M = mul4(M, local);
  });

  const ikBone = _translationFor("Ik_bone", config);
  M = mul4(M, T(ikBone[0], ikBone[1], ikBone[2]));

  const [x, y, z] = applyToPoint(M, [0, 0, 0]);
  const rpy = rpyFromMatrix(M);

  return { x, y, z, roll: rpy.roll, pitch: rpy.pitch, yaw: rpy.yaw };
}
