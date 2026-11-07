// home_controller.js
// Set/go/default-home logic. Poses now come from RobotState.getPose()
// instead of readPoseFromDOM (armAnimation.js) — this also means homing
// works correctly for prismatic joints, which the DOM-parsing version did
// not handle.

let homePose = null;


function disconnectGamepadForHoming() {
    if (typeof window.isGamepadConnected === "function" && window.isGamepadConnected()) {
        window.disconnectGamepad(true);   // ← true = homing disconnect
        return true;
    }
    return false;
}

function reconnectGamepadAfterHoming(wasConnected) {
  if (wasConnected && typeof window.connectGamepad === "function") {
    window.connectGamepad();
  }
}

function syncIKFieldsFromPose() {
  // Wait for kinematics.js to update GPos fields after animation lands
  setTimeout(function () {
    const gx = parseFloat(document.getElementById("GPosx").value);
    const gy = parseFloat(document.getElementById("GPosy").value);
    const gz = parseFloat(document.getElementById("GPosz").value);

    if (!isNaN(gx)) document.getElementById("tx").value = gx.toFixed(2);
    if (!isNaN(gy)) document.getElementById("ty").value = gy.toFixed(2);
    if (!isNaN(gz)) document.getElementById("tz").value = gz.toFixed(2);

    document.getElementById("tRoll").value  = parseFloat(document.getElementById("j6").value) || 0;
    document.getElementById("tPitch").value = parseFloat(document.getElementById("j5").value) || 0;
    document.getElementById("tYaw").value   = parseFloat(document.getElementById("j4").value) || 0;

    console.log("[homing] IK fields synced →",
      "xyz:", gx, gy, gz,
      "rpy:", document.getElementById("tRoll").value,
      document.getElementById("tPitch").value,
      document.getElementById("tYaw").value
    );

  }, 150); // 150ms — enough for kinematics.js 100ms poll to fire once after landing
}



function setHome() {
  try {
    homePose = window.RobotState.getPose();
    homeStatus("Home position set");
  } catch (err) {
    console.error(err);
    homeStatus("Failed to set Home (check RobotState / joint config)");
  }
}

function goHome() {
  if (!homePose) {
    homeStatus("Home not set yet");
    return;
  }

  try {
	const wasConnected = disconnectGamepadForHoming();
    cancelAnim(); // stop any ongoing animation (armAnimation.js)

    const currentPose = window.RobotState.getPose(); // snapshot current
    const duration = getDuration();        // uses #animDuration from armAnimation.js

    homeStatus("Going Home...");

    animateSegment(currentPose, homePose, duration, () => {
      homeStatus("Arrived Home");
	  syncIKFieldsFromPose();
		reconnectGamepadAfterHoming(wasConnected);
    });
  } catch (err) {
    console.error(err);
    homeStatus("Failed to Go Home (check RobotState / joint config)");
  }
}

function homeStatus(msg) {
  const el = document.getElementById("homeStatus");
  if (!el) {
    console.warn("homeStatus element not found:", msg);
    return;
  }
  el.innerText = msg;
}

function defaultHome() {
  try {
	const wasConnected = disconnectGamepadForHoming();
    cancelAnim();

    const currentPose = window.RobotState.getPose();
    const duration = getDuration();

    // target: same joints, all values = 0
    const zeroPose = {};
    for (const jKey of Object.keys(currentPose)) {
      zeroPose[jKey] = { valueDeg: 0 };
    }

    homeStatus("Going Default Home...");

    animateSegment(currentPose, zeroPose, duration, () => {
      homeStatus("Arrived Default Home");
	  syncIKFieldsFromPose();
		reconnectGamepadAfterHoming(wasConnected);
	  
    });
	
  } catch (err) {
    console.error(err);
    homeStatus("Failed Default Home (check RobotState / joint config)");
  }
}

// Expose for inline onclick handlers (if your buttons call these names)
window.setHome = setHome;
window.goHome = goHome;
window.defaultHome = defaultHome;
