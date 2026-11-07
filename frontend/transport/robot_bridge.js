// =============================================================================
// robot_bridge.js
// =============================================================================
// Single WebSocket connection to a Python backend.
// Connects automatically on page load. Reconnects automatically if dropped.
//
// This file is transport only. Joint reads/writes go through RobotState —
// _applyOneJoint, _getJointAxis, _getJointType, and _readJointsDeg have all
// moved to robot_state.js. setSeedAngle is gone too: slider_adapter.js
// calls RobotState.setJointValue() directly instead.
//
// JS → Python  (published at 20 Hz while connected)
// ─────────────────────────────────────────────────
//   { type: "JOINT_STATE",
//     joints:    { j1, j2, j3, j4, j5, j6 },   // degrees
//     timestamp: <ms since epoch> }
//
//   { type: "GRIPPER_POSE",
//     x, y, z,                                   // mm, world frame
//     roll, pitch, yaw,                           // degrees, world frame
//     timestamp: <ms since epoch> }
//
// JS → Python  (event-driven)
// ────────────────────────────
//   { type: "CONFIG",
//     config: { joint_axes, joint_type, translations_mm,
//               joint_limits_deg, model_paths, ... } }
//     Sent once on connect (if config loaded) and again whenever
//     the user loads a new config file.
//
//   { type: "IK_TARGET",
//     target:         { x, y, z },          // mm
//     seed:           { j1..j6 },           // degrees, current pose
//     orientation:    { roll, pitch, yaw }  // degrees — or null if RPY disabled
//     targetRPY:      { roll, pitch, yaw }  // degrees — always present
//     useOrientation: bool }                // true = enforce RPY in solve
//
//   { type: "SOLVE_IK_BUTTON",
//     pressed: true | false }               // pointerdown / pointerup
//
// Python → JS  (handled any time)
// ────────────────────────────────
//   { type: "HEARTBEAT" }
//     → ikStatus shows "Bridge connection established"
//     → if heartbeats stop arriving, ikStatus shows "Connection failed"
//
//   { type: "SET_JOINTS",
//     joints: { j1, j2, j3, j4, j5, j6 } } // degrees — applied via RobotState
//
//   { type: "IK_STATUS", text, colour }     // backup IK only (mock_backend.js)
//
// Dependencies (must load before this file)
// ──────────────────────────────────────────
//   shared_utils.js, robot_state.js  → window.RobotState
//   kinematics.js (ES module)        → window.getGPosXYZ() set in HTML module block
// =============================================================================

// =====================================================================
// Server check with offline fallback (backup IK)
// ---------------------------------------------------------------------
//   1. Tries the real bridge_server.py WebSocket.
//   2. If it errors or doesn't open within 2.5 s, loads mock_backend.js
//      (from the same folder as this file) and continues in offline mode.
//      Once offline, later reconnect attempts go straight to the mock,
//      so there is no reconnect loop against a dead port.
//   3. URL override:  ?backend=mock  (always offline)
//                     ?backend=real  (never fall back)
// =====================================================================
const ROBOT_BRIDGE_URL = (document.currentScript && document.currentScript.src) || location.href;
// If robot_bridge.js is loaded with type="module", use this line instead:
// const ROBOT_BRIDGE_URL = import.meta.url;

const SERVER_TIMEOUT_MS = 2500;
let useOfflineBackend = false;

function loadMockBackend() {
  if (window.MockBackend) return Promise.resolve(window.MockBackend);
  if (loadMockBackend._p) return loadMockBackend._p;
  return (loadMockBackend._p = new Promise((resolve, reject) => {
    const s = document.createElement("script");
    s.src = new URL("mock_backend.js", ROBOT_BRIDGE_URL).href;
    s.onload = () => (window.MockBackend ? resolve(window.MockBackend) : reject(new Error("MockBackend missing")));
    s.onerror = () => reject(new Error("could not load " + s.src));
    document.head.appendChild(s);
  }));
}

// Returns a WebSocket-like object straight away; the app attaches
// onopen / onmessage / onclose / onerror and calls send() as usual.
function connectWithFallback(url) {
  const forced = new URLSearchParams(location.search).get("backend");
  const sock = new EventTarget();
  sock.url = url;
  sock.isMock = false;
  sock.onopen = sock.onmessage = sock.onclose = sock.onerror = null;
  let inner = null;

  Object.defineProperty(sock, "readyState", { get: () => (inner ? inner.readyState : 0) });
  sock.send  = (d) => { if (!inner) throw new DOMException("not connected yet", "InvalidStateError"); inner.send(d); };
  sock.close = (c, r) => { if (inner) inner.close(c, r); };

  function attach(target, hideEarlyFailure) {
    if (inner) inner._detached = true;
    inner = target;
    let opened = false;
    for (const name of ["open", "message", "close", "error"]) {
      target.addEventListener(name, (ev) => {
        if (target._detached) return;
        if (name === "open") opened = true;
        // Hide the failed real socket's error/close so the app doesn't start its reconnect loop.
        if (hideEarlyFailure && !opened && (name === "error" || name === "close")) return;
        const h = sock["on" + name];
        if (typeof h === "function") { try { h.call(sock, ev); } catch (e) { console.error(e); } }
        sock.dispatchEvent(new ev.constructor(ev.type, ev));
      });
    }
  }

  let switching = false;
  function goOffline(reason) {
    if (switching || sock.isMock) return;
    switching = true;
    useOfflineBackend = true;
    console.warn(`[robot_bridge] backend not reachable (${reason}) → offline mode`);
    if (inner) { inner._detached = true; try { inner.close(); } catch {} inner = null; }
    loadMockBackend()
      .then((Mock) => { sock.isMock = true; attach(new Mock(url), false); })
      .catch((e) => {
        console.error("[robot_bridge] offline mode unavailable:", e);
        const h = sock.onerror; if (typeof h === "function") h.call(sock, new Event("error"));
      });
  }

  if (forced === "mock" || (useOfflineBackend && forced !== "real")) {
    goOffline(forced === "mock" ? "?backend=mock" : "already offline");
    return sock;
  }

  let ws;
  try { ws = new WebSocket(url); }
  catch (e) {                                   // e.g. ws:// blocked on an https page
    if (forced === "real") throw e;
    goOffline(e.name || "blocked");
    return sock;
  }

  const fallback = forced !== "real";
  attach(ws, fallback);
  if (fallback) {
    const timer = setTimeout(() => { if (ws.readyState !== 1) goOffline("timeout"); }, SERVER_TIMEOUT_MS);
    ws.addEventListener("open",  () => clearTimeout(timer));
    ws.addEventListener("error", () => { if (ws.readyState !== 1) { clearTimeout(timer); goOffline("connection error"); } });
  }
  return sock;
}
// ============================ end fallback ============================



// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const JOINT_KEYS       = ["j1", "j2", "j3", "j4", "j5", "j6"];
const PUBLISH_MS       = 50;    // 20 Hz publish rate
const WS_PORT          = 9090;  // must match bridge_server.py
const RECONNECT_MS     = 3000;  // retry interval when Python is not running
const HEARTBEAT_TIMEOUT_MS = 3000;  // declare "failed" if no heartbeat within this

// ---------------------------------------------------------------------------
// Arm config — pushed to RobotState here, then forwarded to the backend
// ---------------------------------------------------------------------------

window.setRobotConfig = function (cfg) {
  if (window.RobotState) window.RobotState.setConfig(cfg);
  console.log("[robot_bridge] Config set:", cfg);
  if (_ws && _ws.readyState === WebSocket.OPEN) {
    _ws.send(JSON.stringify({ type: "CONFIG", config: cfg }));
    console.log("[robot_bridge] CONFIG sent to Python");
  }
};

// ---------------------------------------------------------------------------
// Status helpers
// ---------------------------------------------------------------------------

const OFFLINE_STATUS = "Connection failed - using backup IK";
let _lastIkStatus = null;   // last backup-IK result, kept on the status line

function _setIkStatus(msg, colour) {
  const el = document.getElementById("ikStatus");
  if (!el) return;
  el.textContent = msg;
  el.style.color = colour || "#555";
}

// ---------------------------------------------------------------------------
// Inbound: SET_JOINTS
// ---------------------------------------------------------------------------

function _handleSetJoints(msg) {
  if (!msg.joints || typeof msg.joints !== "object") {
    console.warn("[robot_bridge] SET_JOINTS: missing joints object");
    return;
  }
  if (!window.RobotState) {
    console.warn("[robot_bridge] SET_JOINTS: RobotState not loaded");
    return;
  }
  window.RobotState.applyAnglesDeg(msg.joints);
}

// ---------------------------------------------------------------------------
// Inbound message router
// ---------------------------------------------------------------------------

function _onMessage(event) {
  let msg;
  try { msg = JSON.parse(event.data); }
  catch (e) { console.warn("[robot_bridge] Bad JSON:", event.data); return; }

  switch (msg.type) {
    case "HEARTBEAT":
      if (_ws && _ws.isMock) {
        _setIkStatus(_lastIkStatus ? `${OFFLINE_STATUS} · ${_lastIkStatus.text}` : OFFLINE_STATUS,
                     _lastIkStatus ? _lastIkStatus.colour : "orange");
      } else {
        _setIkStatus("Bridge connection established", "green");
      }
      _resetHeartbeatWatchdog();
      break;
    case "IK_STATUS":
      _lastIkStatus = { text: msg.text.replace(/^Backup IK:\s*/, ""), colour: msg.colour };
      _setIkStatus(`${OFFLINE_STATUS} · ${_lastIkStatus.text}`, msg.colour);
      break;
    case "SET_JOINTS":
      _handleSetJoints(msg);
      break;
    default:
      console.log("[robot_bridge] Unhandled msg type:", msg.type, msg);
  }
}

// ---------------------------------------------------------------------------
// Heartbeat watchdog — if no HEARTBEAT arrives within timeout, show failed
// ---------------------------------------------------------------------------

let _heartbeatWatchdog = null;

function _resetHeartbeatWatchdog() {
  if (_heartbeatWatchdog) clearTimeout(_heartbeatWatchdog);
  _heartbeatWatchdog = setTimeout(() => {
    _setIkStatus("Connection failed", "red");
  }, HEARTBEAT_TIMEOUT_MS);
}

function _clearHeartbeatWatchdog() {
  if (_heartbeatWatchdog) { clearTimeout(_heartbeatWatchdog); _heartbeatWatchdog = null; }
}

// ---------------------------------------------------------------------------
// Outbound publish loop
// ---------------------------------------------------------------------------

let _publishTimer = null;

function _startPublishing() {
  if (_publishTimer) clearInterval(_publishTimer);
  _publishTimer = setInterval(_publish, PUBLISH_MS);
}

function _stopPublishing() {
  if (_publishTimer) { clearInterval(_publishTimer); _publishTimer = null; }
}

function _publish() {
  if (!_ws || _ws.readyState !== WebSocket.OPEN) return;

  const ts = Date.now();

  _ws.send(JSON.stringify({
    type:      "JOINT_STATE",
    joints:    window.RobotState ? window.RobotState.getJointsDeg() : {},
    timestamp: ts,
  }));

  if (typeof window.getGPosXYZ === "function") {
    const pose = window.getGPosXYZ();
    if (pose) {
      _ws.send(JSON.stringify({
        type:      "GRIPPER_POSE",
        x:         +pose.x.toFixed(3),
        y:         +pose.y.toFixed(3),
        z:         +pose.z.toFixed(3),
        roll:      +(pose.roll  ?? 0).toFixed(3),
        pitch:     +(pose.pitch ?? 0).toFixed(3),
        yaw:       +(pose.yaw   ?? 0).toFixed(3),
        timestamp: ts,
      }));
    }
  }
}

// ---------------------------------------------------------------------------
// solveIK — called by Solve IK button
// ---------------------------------------------------------------------------

window.solveIK = function () {
  if (!_ws || _ws.readyState !== WebSocket.OPEN) return;

  const target = {
    x: parseFloat(document.getElementById("tx").value),
    y: parseFloat(document.getElementById("ty").value),
    z: parseFloat(document.getElementById("tz").value),
  };

  const useRPY = document.getElementById("useOrientation")?.checked ?? false;
  const rpy = {
    roll:  parseFloat(document.getElementById("tRoll").value),
    pitch: parseFloat(document.getElementById("tPitch").value),
    yaw:   parseFloat(document.getElementById("tYaw").value),
  };

  _ws.send(JSON.stringify({
    type:           "IK_TARGET",
    target,
    seed:           window.RobotState ? window.RobotState.getJointsDeg() : {},
    orientation:    useRPY ? rpy : null,
    targetRPY:      rpy,
    useOrientation: useRPY,
  }));
};

// ---------------------------------------------------------------------------
// Solve IK button — send pressed state over WebSocket
// ---------------------------------------------------------------------------

document.addEventListener("DOMContentLoaded", function () {
  const btn = document.querySelector("button[onclick='solveIK()']");
  if (!btn) return;

  btn.addEventListener("pointerdown", () => {
    if (_ws && _ws.readyState === WebSocket.OPEN)
      _ws.send(JSON.stringify({ type: "SOLVE_IK_BUTTON", pressed: true }));
  });
  btn.addEventListener("pointerup", () => {
    if (_ws && _ws.readyState === WebSocket.OPEN)
      _ws.send(JSON.stringify({ type: "SOLVE_IK_BUTTON", pressed: false }));
  });
  btn.addEventListener("pointerleave", () => {
    if (_ws && _ws.readyState === WebSocket.OPEN)
      _ws.send(JSON.stringify({ type: "SOLVE_IK_BUTTON", pressed: false }));
  });
});

// ---------------------------------------------------------------------------
// WebSocket — auto-connect and auto-reconnect
// ---------------------------------------------------------------------------

let _ws = null;

function _connect() {
  // Already connecting or open — do nothing
  if (_ws && (_ws.readyState === WebSocket.CONNECTING ||
              _ws.readyState === WebSocket.OPEN)) return;

  const proto = location.protocol === "https:" ? "wss" : "ws";
  const url = `${proto}://${location.hostname || "localhost"}:${WS_PORT}`;
  _ws = connectWithFallback(url);

  _ws.onopen = () => {
    if (_ws.isMock) {
      console.log("[robot_bridge] Python backend not reachable — using backup IK (mock_backend.js)");
      _setIkStatus(OFFLINE_STATUS, "orange");
    } else {
      console.log(`[robot_bridge] Connected to ${url}`);
    }
    _startPublishing();
    const cfg = window.RobotState ? window.RobotState.getConfig() : null;
    if (cfg) {
      _ws.send(JSON.stringify({ type: "CONFIG", config: cfg }));
    }
  };

  _ws.onclose = () => {
    _stopPublishing();
    _clearHeartbeatWatchdog();
    _setIkStatus("Connection failed", "red");
    console.log("[robot_bridge] Closed — retrying in", RECONNECT_MS, "ms");
    setTimeout(_connect, RECONNECT_MS);
  };

  _ws.onerror = () => {
    // onclose fires right after onerror, so reconnect is handled there
  };

  _ws.onmessage = _onMessage;
}

// Initialise status and start connecting immediately
_setIkStatus("Not connected", "#555");
_connect();