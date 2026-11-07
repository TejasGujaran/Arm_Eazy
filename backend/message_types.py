"""
message_types.py
=================
Loads shared/message_types.json so the backend uses the exact same message
type strings as the frontend — no separate Python literal that could drift
out of sync with robot_bridge.js.
"""

import json
from pathlib import Path

_SHARED_PATH = Path(__file__).resolve().parent.parent / "shared" / "message_types.json"

with open(_SHARED_PATH, "r") as _f:
    _SCHEMA = json.load(_f)

SCHEMA_VERSION = _SCHEMA["schema_version"]

CLIENT_TO_SERVER = _SCHEMA["client_to_server"]
SERVER_TO_CLIENT = _SCHEMA["server_to_client"]

# Client → server
JOINT_STATE     = CLIENT_TO_SERVER["JOINT_STATE"]
GRIPPER_POSE    = CLIENT_TO_SERVER["GRIPPER_POSE"]
CONFIG          = CLIENT_TO_SERVER["CONFIG"]
IK_TARGET       = CLIENT_TO_SERVER["IK_TARGET"]
SOLVE_IK_BUTTON = CLIENT_TO_SERVER["SOLVE_IK_BUTTON"]

# Server → client
HEARTBEAT  = SERVER_TO_CLIENT["HEARTBEAT"]
SET_JOINTS = SERVER_TO_CLIENT["SET_JOINTS"]
