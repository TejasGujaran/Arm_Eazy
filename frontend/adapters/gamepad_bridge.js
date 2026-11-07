// gamepad_bridge.js
// Uses the browser's native Gamepad API — no Python / WebSocket required.
// Polls at 50 Hz, accumulates position, calls solveIK() automatically.
//
// Mapping (same as GamePad.py):
//   Left  stick X/Y  → Translation X / Y
//   L2 / R2          → Translation Z- / Z+
//   Right stick X/Y  → Roll / Pitch
//   L1 / R1          → Yaw CCW / CW

// ── Tuning ────────────────────────────────────────────────────────────────
const DEADZONE          = 0.08;
let   TRANSLATION_SPEED = 10.0;   // mm per tick  (matches gpScalePos default)
let   ROTATION_SPEED    = 1.0;    // deg per tick (matches gpScaleRot default)
const YAW_FIXED_SPEED   = 1.0;    // deg per tick
const POLL_HZ           = 50;

// ── Axis / button indices (PS/Xbox standard layout) ───────────────────────
const AXIS_LEFT_X  = 0;
const AXIS_LEFT_Y  = 1;
const AXIS_RIGHT_X = 2;
const AXIS_RIGHT_Y = 3;
const AXIS_L2      = 4;   // -1 at rest → +1 fully pressed (some controllers)
const AXIS_R2      = 5;
const BUTTON_L1    = 4;
const BUTTON_R1    = 5;
const BUTTON_L2    = 6;   // fallback if L2/R2 are buttons not axes
const BUTTON_R2    = 7;

// ── State ─────────────────────────────────────────────────────────────────
let _loopHandle   = null;
let _isConnected  = false;
let _gpIndex      = null;   // index into navigator.getGamepads()

// ── Helpers ───────────────────────────────────────────────────────────────

function _deadzone(v) {
  if (Math.abs(v) < DEADZONE) return 0;
  const sign = v > 0 ? 1 : -1;
  return sign * (Math.abs(v) - DEADZONE) / (1 - DEADZONE);
}

/** Remap trigger axis: resting at -1, fully pressed = +1 → output 0..1 */
function _trigger(raw) {
  return Math.max(0, (raw + 1) / 2);
}

function _wrapDeg(a) {
  return ((a + 180) % 360 + 360) % 360 - 180;
}

function gpStatus(msg) {
  const el = document.getElementById("gpStatus");
  if (el) el.textContent = msg;
}

function _enableRPY() {
  const cb  = document.getElementById("useOrientation");
  const div = document.getElementById("rpyInputs");
  if (cb && !cb.checked) {
    cb.checked = true;
    if (div) { div.style.opacity = "1"; div.style.pointerEvents = "auto"; }
  }
}

// ── Main poll loop ────────────────────────────────────────────────────────

function _poll() {
  const pads = navigator.getGamepads();
  const gp   = pads[_gpIndex];

  if (!gp || !gp.connected) {
    gpStatus("❌ Gamepad lost — reconnect");
    _stopLoop();
    return;
  }

  const axes    = gp.axes;
  const buttons = gp.buttons;

  // Translation
  const vx =  _deadzone(axes[AXIS_LEFT_X]  ?? 0) * TRANSLATION_SPEED;
  const vy = -_deadzone(axes[AXIS_LEFT_Y]  ?? 0) * TRANSLATION_SPEED;

  // Z via triggers — try axis first, fall back to button value
  const l2Raw = axes[AXIS_L2] !== undefined ? axes[AXIS_L2] : (buttons[BUTTON_L2]?.value ?? 0) * 2 - 1;
  const r2Raw = axes[AXIS_R2] !== undefined ? axes[AXIS_R2] : (buttons[BUTTON_R2]?.value ?? 0) * 2 - 1;
  const vz = (_trigger(r2Raw) - _trigger(l2Raw)) * TRANSLATION_SPEED;

  // Rotation
  const vYaw  =  _deadzone(axes[AXIS_RIGHT_X] ?? 0) * ROTATION_SPEED;
  const vPitch = -_deadzone(axes[AXIS_RIGHT_Y] ?? 0) * ROTATION_SPEED;

  const l1 = buttons[BUTTON_L1]?.pressed ? 1 : 0;
  const r1 = buttons[BUTTON_R1]?.pressed ? 1 : 0;
  const vRoll = (r1 - l1) * YAW_FIXED_SPEED;

  // Any movement at all?
  const moving = vx || vy || vz || vRoll || vPitch || vYaw;
  if (!moving) return;

    // Read current IK field values and add delta directly

    const baseX    = parseFloat(document.getElementById("GPosx")?.value) || 0;
    const baseY    = parseFloat(document.getElementById("GPosy")?.value) || 0;
    const baseZ    = parseFloat(document.getElementById("GPosz")?.value) || 0;
    const baseRoll  = parseFloat(document.getElementById("tRoll")?.value) || 0;
    const basePitch = parseFloat(document.getElementById("tPitch")?.value) || 0;
    const baseYaw   = parseFloat(document.getElementById("tYaw")?.value) || 0;

    document.getElementById("tx").value     = (baseX + vx).toFixed(2);
    document.getElementById("ty").value     = (baseY + vy).toFixed(2);
    document.getElementById("tz").value     = (baseZ + vz).toFixed(2);
    document.getElementById("tRoll").value  = _wrapDeg(baseRoll  + vRoll).toFixed(1);
    document.getElementById("tPitch").value = _wrapDeg(basePitch + vPitch).toFixed(1);
    document.getElementById("tYaw").value   = _wrapDeg(baseYaw   + vYaw).toFixed(1);
    
   
      if (typeof window.solveIK === "function") window.solveIK();
    }

function _startLoop() {
  if (_loopHandle) return;
  _loopHandle = setInterval(_poll, 1000 / POLL_HZ);
}

function _stopLoop() {
  if (_loopHandle) { clearInterval(_loopHandle); _loopHandle = null; }
  _isConnected = false;
  _gpIndex     = null;
}

// ── Gamepad connect / disconnect events ───────────────────────────────────

window.addEventListener("gamepadconnected", (e) => {
  console.log("[gamepad] Connected:", e.gamepad.id);
  _gpIndex = e.gamepad.index;  // register it, but don't start yet
  gpStatus("⚠️ Gamepad detected — click Connect to activate");
});

window.addEventListener("gamepaddisconnected", (e) => {
  if (e.gamepad.index === _gpIndex) {
    gpStatus("❌ Gamepad disconnected");
    _stopLoop();
  }
});

// ── Public API (buttons in HTML call these) ───────────────────────────────

window.connectGamepad = function() {
  const pads = navigator.getGamepads();
  const found = pads.find(p => p && p.connected);

  if (!found) {
    gpStatus("⚠️ No gamepad detected — press a button on your controller first");
    return;
  }

  _gpIndex     = found.index;
  _isConnected = true;

  // Seed pose from current IK field values so arm doesn't jump
  _enableRPY();
  _startLoop();

  gpStatus(`✅ ${found.id}`);
  console.log("[gamepad] Connected:", found.id);
};

window.disconnectGamepad = function(silent) {
  _stopLoop();
  if (!silent) gpStatus("Disconnected");
  console.log("[gamepad] Disconnected");
};

window.applyGamepadScales = function() {
  TRANSLATION_SPEED = parseFloat(document.getElementById("gpScalePos")?.value) || 10;
  ROTATION_SPEED    = parseFloat(document.getElementById("gpScaleRot")?.value) || 1;

  const st = document.getElementById("scaleStatus");
  if (st) {
    st.textContent = `✅ Applied (pos=${TRANSLATION_SPEED}, rot=${ROTATION_SPEED})`;
    st.style.color = "green";
    setTimeout(() => { st.textContent = ""; }, 3000);
  }
  console.log("[gamepad] Scales updated — translation:", TRANSLATION_SPEED, "rotation:", ROTATION_SPEED);
};

// ── Expose connected state for home_controller.js ─────────────────────────
window.isGamepadConnected = () => _isConnected && _loopHandle !== null;


document.addEventListener("DOMContentLoaded", () => {
  document.getElementById("connectGamepadBtn")
    ?.addEventListener("click", window.connectGamepad);

  document.getElementById("disconnectGamepadBtn")
    ?.addEventListener("click", window.disconnectGamepad);
});