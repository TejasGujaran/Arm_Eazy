// slider_adapter.js
// Wires the six joint sliders to RobotState.setJointValue() instead of the
// old inline oninput="Joint1.setAttribute(...)" handlers that wrote
// straight to the X3D DOM and called setSeedAngle().
//
// IMPORTANT — twin.html still needs a small edit for this to be the ONLY
// path: remove the inline oninput="..." attribute from each slider
// (#j1..#j6). Until that's done, both the old inline handler and this
// adapter will fire on every slider move — same end result, just redundant
// work, not a bug — so it's safe to load this file before making that edit.

(function () {
  const JOINT_KEYS = ["j1", "j2", "j3", "j4", "j5", "j6"];

  function _wireSlider(jKey) {
    const slider = document.getElementById(jKey);
    if (!slider) return;

    slider.addEventListener("input", () => {
      if (!window.RobotState) return;
      window.RobotState.setJointValue(jKey, slider.value);
    });
  }

  function _init() {
    JOINT_KEYS.forEach(_wireSlider);
  }

  document.addEventListener("DOMContentLoaded", _init);
})();
