// armAnimation.js
// Stores and plays back joint poses. Reads/writes poses through RobotState
// instead of parsing X3D rotation attributes off the DOM — this also means
// stored poses now work correctly for prismatic (translate) joints, which
// the old DOM-based version silently mishandled (it only ever read/wrote
// the `rotation` attribute).

// ---- STATE ----
let poses = [];     // array of pose objects, each from RobotState.getPose()
let animReq = null;

// ---- HELPERS ----
function lerp(a, b, t) { return a + (b - a) * t; }

function setStatus(msg) {
  const s = document.getElementById("poseStatus");
  if (s) s.textContent = msg;
}

function getDuration() {
  const durationEl = document.getElementById("animDuration");
  return Math.max(50, parseInt(durationEl?.value || "1500", 10));
}

function cancelAnim() {
  if (animReq) cancelAnimationFrame(animReq);
  animReq = null;
}

// ---- PUBLIC: store positions ----

// Stores current robot pose into slot index (1-based for UI friendliness)
function storePosition(slotNumber) {
  const idx = slotNumber - 1;
  poses[idx] = window.RobotState.getPose();
  setStatus(`Stored position P${slotNumber}`);
}

// Optional: append a new position at the end
function addPosition() {
  poses.push(window.RobotState.getPose());
  setStatus(`Added position P${poses.length}`);
}

// Optional: clear all
function clearPositions() {
  cancelAnim();
  poses = [];
  setStatus("Cleared all positions");
}

// ---- CORE: animate through all positions in order ----

function animateSegment(startPose, endPose, duration, onDone) {
  const t0 = performance.now();
  const jointKeys = Object.keys(startPose);

  function step(now) {
    const u = Math.min(1, (now - t0) / duration);

    const curr = {};
    for (const jKey of jointKeys) {
      curr[jKey] = {
        valueDeg: lerp(startPose[jKey].valueDeg, endPose[jKey].valueDeg, u),
      };
    }
    window.RobotState.applyPose(curr);

    if (u < 1) {
      animReq = requestAnimationFrame(step);
    } else {
      animReq = null;
      onDone?.();
    }
  }

  animReq = requestAnimationFrame(step);
}

// Plays P1 -> P2 -> ... -> PN (touches every pose)
function playForward() {
  if (poses.length < 2 || poses.some(p => !p)) {
    setStatus("Need at least 2 stored positions (P1..PN) with no gaps.");
    return;
  }
  cancelAnim();

  const perSegment = getDuration(); // duration per segment (simple + predictable)
  let i = 0;

  setStatus(`Playing forward: P1 → P${poses.length}`);

  const next = () => {
    if (i >= poses.length - 1) {
      setStatus("Done forward");
      return;
    }
    const from = poses[i];
    const to = poses[i + 1];
    i++;

    animateSegment(from, to, perSegment, next);
  };

  next();
}

// Plays PN -> ... -> P1 (touches every pose)
function playReverse() {
  if (poses.length < 2 || poses.some(p => !p)) {
    setStatus("Need at least 2 stored positions (P1..PN) with no gaps.");
    return;
  }
  cancelAnim();

  const perSegment = getDuration();
  let i = poses.length - 1;

  setStatus(`Playing reverse: P${poses.length} → P1`);

  const next = () => {
    if (i <= 0) {
      setStatus("Done reverse");
      return;
    }
    const from = poses[i];
    const to = poses[i - 1];
    i--;

    animateSegment(from, to, perSegment, next);
  };

  next();
}

// Optional: stop button
function stopPlayback() {
  cancelAnim();
  setStatus("Stopped");
}

// ---- Shared entry points used by KRL controller integration and homing ----

// Read current pose
window.readCurrentPose = function () {
  return window.RobotState.getPose();
};

// Animate from current pose to a target pose
// onDone is optional (called when motion finishes)
window.animateToPose = function (targetPose, onDone) {
  const start = window.RobotState.getPose();
  const duration = getDuration();

  cancelAnim();
  animateSegment(start, targetPose, duration, () => {
    if (typeof onDone === "function") onDone();
  });
};
