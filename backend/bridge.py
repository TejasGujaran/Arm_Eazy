"""
bridge.py
=========
Arm_Eazy bridge — the only thing you need to run.

One file, one port:
  * serves the frontend (frontend/) and configs (config/) over plain HTTP
  * handles the WebSocket link to the browser (digital twin)
  * solves IK (via ik.py) and sends SET_JOINTS back

Run:
    python bridge.py                 # quiet: one short line per solve
    python bridge.py --verbose       # full IK_TARGET / result dumps
    python bridge.py --host 0.0.0.0 --port 9090

Then open:
    http://localhost:9090

Message flow
------------
Browser → server : JOINT_STATE, GRIPPER_POSE, CONFIG, IK_TARGET, SOLVE_IK_BUTTON
Server → browser : HEARTBEAT (1 Hz), SET_JOINTS

IK under gamepad control ("latest wins")
----------------------------------------
The gamepad can send IK_TARGET faster than scipy solves. Each connection has
ONE background worker that always solves the most recent target. Targets that
arrive while a solve is running only overwrite the pending target; stale
targets are never solved, and the WebSocket receive loop never blocks on a
solve (so JOINT_STATE, heartbeats and ping/pong keep flowing).

Each browser tab gets its own Session, so several tabs/users don't overwrite
each other's config or joint state.
"""

import argparse
import asyncio
import json
import time
from pathlib import Path

import websockets
from websockets.http11 import Response

from ik import solve_ik, ConfigError

# =============================================================================
# Paths / settings
# =============================================================================

BACKEND_DIR  = Path(__file__).resolve().parent
REPO_ROOT    = BACKEND_DIR.parent
FRONTEND_DIR = REPO_ROOT / "frontend"
CONFIG_DIR   = REPO_ROOT / "config"          # served under /config/
SHARED_TYPES = REPO_ROOT / "shared" / "message_types.json"

DEFAULT_HOST       = "localhost"
DEFAULT_PORT       = 9090                    # must match WS_PORT in robot_bridge.js
HEARTBEAT_INTERVAL = 1.0                     # seconds

JOINT_KEYS = ["j1", "j2", "j3", "j4", "j5", "j6"]

VERBOSE = False                              # set from --verbose in main()


def log(msg):
    print(msg, flush=True)


# =============================================================================
# Message types — shared with the frontend via shared/message_types.json
# =============================================================================

def _load_message_types():
    defaults = {
        "client_to_server": {k: k for k in
                             ("JOINT_STATE", "GRIPPER_POSE", "CONFIG", "IK_TARGET", "SOLVE_IK_BUTTON")},
        "server_to_client": {k: k for k in ("HEARTBEAT", "SET_JOINTS")},
    }
    try:
        with open(SHARED_TYPES, "r") as f:
            schema = json.load(f)
        return schema["client_to_server"], schema["server_to_client"]
    except (OSError, KeyError, json.JSONDecodeError) as e:
        log(f"[bridge] Could not read {SHARED_TYPES} ({e!r}) — using built-in message names")
        return defaults["client_to_server"], defaults["server_to_client"]


_C2S, _S2C = _load_message_types()

JOINT_STATE     = _C2S["JOINT_STATE"]
GRIPPER_POSE    = _C2S["GRIPPER_POSE"]
CONFIG          = _C2S["CONFIG"]
IK_TARGET       = _C2S["IK_TARGET"]
SOLVE_IK_BUTTON = _C2S["SOLVE_IK_BUTTON"]

HEARTBEAT  = _S2C["HEARTBEAT"]
SET_JOINTS = _S2C["SET_JOINTS"]


# =============================================================================
# Static file serving (plain HTTP GET on the same port as the WebSocket)
# =============================================================================

CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js":   "application/javascript",
    ".mjs":  "application/javascript",
    ".css":  "text/css",
    ".json": "application/json",
    ".x3d":  "model/x3d+xml",
    ".svg":  "image/svg+xml",
    ".png":  "image/png",
    ".jpg":  "image/jpeg",
    ".jpeg": "image/jpeg",
    ".ico":  "image/x-icon",
    ".pdf":  "application/pdf",
    ".csv":  "text/csv",
}


def _resolve_static_path(url_path):
    """Map a URL path to a file; None if it tries to escape its root via ../"""
    if url_path == "/":
        url_path = "/index.html"

    if url_path.startswith("/config/"):
        root, rel = CONFIG_DIR, url_path[len("/config/"):]
    else:
        root, rel = FRONTEND_DIR, url_path.lstrip("/")

    root = root.resolve()
    file_path = (root / rel).resolve()
    try:
        file_path.relative_to(root)
    except ValueError:
        return None
    return file_path


async def process_request(connection, request):
    # Let WebSocket upgrades through; answer everything else as static files.
    if request.headers.get("Upgrade", "").lower() == "websocket":
        return None

    url_path  = request.path.split("?")[0]
    file_path = _resolve_static_path(url_path)

    if file_path is None:
        return Response(403, "Forbidden", websockets.Headers(), b"Forbidden")
    if not file_path.is_file():
        return Response(404, "Not Found", websockets.Headers(), b"Not found")

    content_type = CONTENT_TYPES.get(file_path.suffix.lower(), "application/octet-stream")
    headers = websockets.Headers({"Content-Type": content_type, "Cache-Control": "no-cache"})
    return Response(200, "OK", headers, file_path.read_bytes())


# =============================================================================
# Session — all state for one browser connection
# =============================================================================

class Session:
    def __init__(self, websocket):
        self.ws = websocket

        # Latest joint values from the browser (deg for rotate joints,
        # raw slider units for translate joints)
        self.joint_state = {k: 0.0 for k in JOINT_KEYS}

        # Latest gripper pose from the browser (mm + deg)
        self.gripper_pose = {"x": 0.0, "y": 0.0, "z": 0.0,
                             "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

        # Arm config from the browser; None until CONFIG arrives
        self.config = None

        # Latest IK request
        self.ik_target = {
            "target": {"x": 0.0, "y": 0.0, "z": 0.0},
            "seed": {},
            "targetRPY": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "useOrientation": False,
        }

        self.solve_ik_pressed = False

        # Latest-wins IK worker
        self._ik_task = None
        self._ik_pending = False

        # Stats (shown in quiet mode)
        self._n_received = 0
        self._n_solved = 0

    # ---- inbound handlers ---------------------------------------------------

    def _on_joint_state(self, data):
        joints = data.get("joints")
        if isinstance(joints, dict):
            self.joint_state.update(joints)

    def _on_gripper_pose(self, data):
        for key in self.gripper_pose:
            if key in data:
                self.gripper_pose[key] = data[key]

    def _on_config(self, data):
        cfg = data.get("config")
        if not isinstance(cfg, dict):
            log("[session] CONFIG without a config object — ignored")
            return
        self.config = cfg
        log(f"[session] CONFIG received: {list(cfg.keys())}")

    def _on_ik_target(self, data):
        t = data.get("target")
        if not isinstance(t, dict) or not all(k in t for k in ("x", "y", "z")):
            log("[session] IK_TARGET without a valid target — ignored")
            return False

        self.ik_target["target"]         = {k: float(t[k]) for k in ("x", "y", "z")}
        self.ik_target["seed"]           = data.get("seed") or {}
        self.ik_target["targetRPY"]      = data.get("targetRPY") or self.ik_target["targetRPY"]
        self.ik_target["useOrientation"] = bool(data.get("useOrientation", False))
        self._n_received += 1

        if VERBOSE:
            rpy = self.ik_target["targetRPY"]
            log("─" * 50)
            log("  IK_TARGET received")
            log(f"  Target XYZ     : x={t['x']}  y={t['y']}  z={t['z']}")
            log(f"  Target RPY     : roll={rpy.get('roll')}  pitch={rpy.get('pitch')}  yaw={rpy.get('yaw')}")
            log(f"  Use orientation: {self.ik_target['useOrientation']}")
            log(f"  Seed joints    : {self.ik_target['seed']}")
            log("─" * 50)
        return True

    def _on_solve_ik_button(self, data):
        self.solve_ik_pressed = bool(data.get("pressed", False))
        if VERBOSE:
            log(f"[session] SOLVE_IK_BUTTON pressed={self.solve_ik_pressed}")

    def handle(self, data):
        """Route one inbound message. Never blocks on IK."""
        msg_type = data.get("type")

        if msg_type == JOINT_STATE:
            self._on_joint_state(data)
        elif msg_type == GRIPPER_POSE:
            self._on_gripper_pose(data)
        elif msg_type == CONFIG:
            self._on_config(data)
        elif msg_type == IK_TARGET:
            if self._on_ik_target(data):
                self._request_ik()
        elif msg_type == SOLVE_IK_BUTTON:
            self._on_solve_ik_button(data)
        elif VERBOSE:
            log(f"[session] Unknown message type: {msg_type}")

    # ---- IK -----------------------------------------------------------------

    def _request_ik(self):
        if self.config is None:
            log("[session] IK_TARGET received but no CONFIG yet — skipping")
            return
        self._ik_pending = True
        if self._ik_task is None or self._ik_task.done():
            self._ik_task = asyncio.create_task(self._ik_worker())

    def _snapshot(self):
        """Copy everything the solver needs; the next message may change it mid-solve."""
        seed = dict(self.joint_state)
        seed.update(self.ik_target["seed"])   # browser's joints at send time = freshest
        return {
            "seed": seed,
            "config": dict(self.config),
            "target": dict(self.ik_target["target"]),
            "rpy": dict(self.ik_target["targetRPY"]) if self.ik_target["useOrientation"] else None,
        }

    @staticmethod
    def _solve(snap):
        result = solve_ik(
            joint_angles=snap["seed"],
            config=snap["config"],
            target_xyz=snap["target"],
            target_rpy=snap["rpy"],
        )
        result["joints"] = {k: round(float(v), 4) for k, v in result["joints"].items()}
        return result

    async def _ik_worker(self):
        loop = asyncio.get_running_loop()
        while self._ik_pending:
            self._ik_pending = False
            snap = self._snapshot()
            t0 = time.perf_counter()
            try:
                # scipy is blocking — run it off the event loop
                result = await loop.run_in_executor(None, self._solve, snap)
            except ConfigError as e:
                log(f"[session] IK solve skipped: {e}")
                continue
            except Exception as e:
                log(f"[session] IK error (connection kept alive): {e!r}")
                continue
            dt_ms = (time.perf_counter() - t0) * 1000
            self._n_solved += 1

            if VERBOSE:
                log(str(result))
            else:
                tg = snap["target"]
                log(f"[ik] ({tg['x']:.1f}, {tg['y']:.1f}, {tg['z']:.1f})  "
                    f"ok={result.get('success')}  err={result.get('errNorm', float('nan')):.4f}  "
                    f"{dt_ms:.0f} ms  [solved {self._n_solved}/{self._n_received} received]")

            # Joints are always sent so the twin follows the best solution found,
            # same as before; success/err are in the log above.
            try:
                await self.send_joints(result["joints"])
            except websockets.ConnectionClosed:
                return

    # ---- outbound -----------------------------------------------------------

    async def send_joints(self, joints):
        await self.ws.send(json.dumps({"type": SET_JOINTS, "joints": joints}))
        if VERBOSE:
            log(f"[bridge] → {SET_JOINTS} {joints}")

    async def heartbeat_loop(self):
        try:
            while True:
                await asyncio.sleep(HEARTBEAT_INTERVAL)
                await self.ws.send(json.dumps({"type": HEARTBEAT}))
        except websockets.ConnectionClosed:
            pass

    def close(self):
        if self._ik_task is not None and not self._ik_task.done():
            self._ik_task.cancel()


# =============================================================================
# WebSocket handler — one Session per connection
# =============================================================================

async def handler(websocket):
    log(f"[bridge] Client connected: {websocket.remote_address}")
    session = Session(websocket)
    heartbeat = asyncio.create_task(session.heartbeat_loop())

    try:
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError as e:
                log(f"[bridge] Bad JSON: {e}")
                continue
            if not isinstance(data, dict):
                continue
            try:
                session.handle(data)
            except Exception as e:
                # Never let one bad message tear down the connection
                log(f"[bridge] handler error (connection kept alive): {e!r}")
    except websockets.ConnectionClosedError:
        pass
    finally:
        heartbeat.cancel()
        session.close()
        log(f"[bridge] Client disconnected: {websocket.remote_address}")


# =============================================================================
# Entry point
# =============================================================================

async def serve(host, port):
    async with websockets.serve(handler, host, port, process_request=process_request):
        log(f"[bridge] Serving frontend + WebSocket on http://{host}:{port}"
            f"  (verbose={VERBOSE})")
        await asyncio.Future()  # run forever


def main():
    global VERBOSE
    ap = argparse.ArgumentParser(description="Arm_Eazy bridge server")
    ap.add_argument("--host", default=DEFAULT_HOST)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--verbose", "-v", action="store_true",
                    help="print every IK_TARGET, full IK result and SET_JOINTS")
    args = ap.parse_args()
    VERBOSE = args.verbose

    try:
        asyncio.run(serve(args.host, args.port))
    except KeyboardInterrupt:
        log("\n[bridge] Stopped")


if __name__ == "__main__":
    main()
