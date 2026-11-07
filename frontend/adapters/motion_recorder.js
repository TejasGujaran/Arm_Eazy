// =============================================================================
// motion_recorder.js
// =============================================================================
// Records live joint positions at a fixed sample rate and exports them as JSON.
// Also provides a music-player-style playback engine that reads the JSON back
// and drives the arm through the recorded motion.
//
// Recording format (saved to .json)
// ──────────────────────────────────
// {
//   "version"   : 1,
//   "sampleRateHz": 20,
//   "durationMs": 3450,
//   "frames"    : [
//     { "t": 0,    "j1": 0, "j2": 0, "j3": 0, "j4": 0, "j5": 0, "j6": 0 },
//     { "t": 50,   "j1": 1.2, ... },
//     ...
//   ]
// }
//
// Dependencies (must already be loaded):
//   robot_state.js  → window.RobotState.getJointsDeg() / applyAnglesDeg()
//   armAnimation.js → cancelAnim
// =============================================================================

(function (global) {
  "use strict";

  // ── Constants ───────────────────────────────────────────────────────────────
  const JOINT_KEYS  = ["j1", "j2", "j3", "j4", "j5", "j6"];
  const SAMPLE_HZ   = 120;          // frames per second to record
  const SAMPLE_MS   = 1000 / SAMPLE_HZ;

  // ── Internal state ──────────────────────────────────────────────────────────
  let _recTimer    = null;   // setInterval handle while recording
  let _recFrames   = [];     // collected frames during recording
  let _recStart    = 0;      // performance.now() when recording began

  let _playFrames  = [];     // frames loaded from JSON for playback
  let _playDur     = 0;      // total duration of loaded recording (ms)
  let _playTimer   = null;   // requestAnimationFrame handle
  let _playStart   = null;   // performance.now() when playback began
  let _playOffset  = 0;      // offset in ms (when scrubbing then resuming)
  let _isPlaying   = false;
  let _isScrubbing = false;

  // ── Helper: read current joint values from RobotState ──────────────────────
  function _readJointsDeg() {
    if (window.RobotState) return window.RobotState.getJointsDeg();
    const out = {};
    JOINT_KEYS.forEach(k => { out[k] = 0; });
    return out;
  }

  // ── Helper: apply values to the arm (X3D + sliders + labels via RobotState) ─
  function _applyJointsDeg(joints) {
    if (window.RobotState) {
      window.RobotState.applyAnglesDeg(joints);
      return;
    }
    console.warn("[motion_recorder] RobotState not loaded — cannot apply joints");
  }

  // ── Helper: interpolate between two frames at fraction t (0-1) ─────────────
  function _interpFrames(a, b, t) {
    const out = {};
    for (const k of JOINT_KEYS) {
      out[k] = a[k] + (b[k] - a[k]) * t;
    }
    return out;
  }

  // ── Helper: find the two surrounding frames for a given time (ms) ──────────
  function _framesAt(ms) {
    const frames = _playFrames;
    if (!frames.length) return null;
    if (ms <= frames[0].t)  return { joints: frames[0], frac: 0 };
    if (ms >= frames[frames.length - 1].t) return { joints: frames[frames.length - 1], frac: 0 };

    // Binary search
    let lo = 0, hi = frames.length - 1;
    while (hi - lo > 1) {
      const mid = (lo + hi) >> 1;
      if (frames[mid].t <= ms) lo = mid; else hi = mid;
    }
    const a = frames[lo], b = frames[hi];
    const t = (ms - a.t) / (b.t - a.t);
    return { joints: _interpFrames(a, b, t), frac: 0 };
  }

  // ── Helper: seek arm to specific time position (ms) ────────────────────────
  function _seekTo(ms) {
    const r = _framesAt(ms);
    if (r) _applyJointsDeg(r.joints);
    _updatePlayerUI(ms);
  }

  // ── Status helpers ──────────────────────────────────────────────────────────
  function _setRecStatus(msg, color) {
    const el = document.getElementById("recStatus");
    if (!el) return;
    el.textContent = msg;
    el.style.color = color || "#333";
  }

  function _setPlayerStatus(msg, color) {
    const el = document.getElementById("playerStatus");
    if (!el) return;
    el.textContent = msg;
    el.style.color = color || "#333";
  }

  // ── Update the playback UI (time label + progress bar) ─────────────────────
  function _updatePlayerUI(currentMs) {
    const bar   = document.getElementById("playerBar");
    const label = document.getElementById("playerTime");
    if (!_playDur) return;
    const frac = Math.min(1, currentMs / _playDur);
    if (bar)   bar.value = frac * 1000;
    if (label) label.textContent = `${(currentMs / 1000).toFixed(2)}s / ${(_playDur / 1000).toFixed(2)}s`;
  }

  // ── Playback loop ───────────────────────────────────────────────────────────
  function _playLoop(now) {
    if (!_isPlaying || _isScrubbing) return;

    const elapsed = (now - _playStart) + _playOffset;

    if (elapsed >= _playDur) {
      _seekTo(_playDur);
      _stopPlayback();
      _setPlayerStatus("Playback complete", "green");
      return;
    }

    _seekTo(elapsed);
    _playTimer = requestAnimationFrame(_playLoop);
  }

  function _stopPlayback() {
    _isPlaying = false;
    if (_playTimer) { cancelAnimationFrame(_playTimer); _playTimer = null; }
    _updatePlayBtn();
  }

  function _updatePlayBtn() {
    const btn = document.getElementById("playerPlayBtn");
    if (!btn) return;
    btn.textContent = _isPlaying ? "⏸ Pause" : "▶ Play";
  }

  // ── Waveform visualisation ──────────────────────────────────────────────────
  // Draws a simplified "waveform" (joint angle curves over time) on a canvas.
  const WAVEFORM_COLORS = [
    "#e74c3c","#e67e22","#f1c40f","#2ecc71","#3498db","#9b59b6"
  ];

  function _drawWaveform(frames, durationMs) {
    const canvas = document.getElementById("playerWaveform");
    if (!canvas || !frames.length) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width, H = canvas.height;

    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "#1a1a2e";
    ctx.fillRect(0, 0, W, H);

    // Per-joint min/max for normalisation
    const ranges = JOINT_KEYS.map(k => {
      let mn = Infinity, mx = -Infinity;
      for (const f of frames) { mn = Math.min(mn, f[k]); mx = Math.max(mx, f[k]); }
      return { min: mn, max: mx, span: (mx - mn) || 1 };
    });

    JOINT_KEYS.forEach((k, ji) => {
      ctx.beginPath();
      ctx.strokeStyle = WAVEFORM_COLORS[ji];
      ctx.lineWidth   = 1.5;
      ctx.globalAlpha = 0.85;

      const laneH = H / JOINT_KEYS.length;
      const baseY = ji * laneH + laneH * 0.1;
      const drawH = laneH * 0.8;

      frames.forEach((f, fi) => {
        const x = (f.t / durationMs) * W;
        const norm = (f[k] - ranges[ji].min) / ranges[ji].span;
        const y = baseY + drawH - norm * drawH;
        fi === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
      });

      ctx.stroke();
    });
    ctx.globalAlpha = 1;

    // Lane labels
    JOINT_KEYS.forEach((k, ji) => {
      const laneH = H / JOINT_KEYS.length;
      ctx.fillStyle = WAVEFORM_COLORS[ji];
      ctx.font = "bold 9px monospace";
      ctx.fillText(`J${ji + 1}`, 4, ji * laneH + 12);
    });
  }

  // Draw playhead over waveform
  function _drawPlayhead(ms) {
    const canvas = document.getElementById("playerWaveform");
    if (!canvas || !_playDur) return;

    // Full redraw (cheap since it's a small canvas)
    _drawWaveform(_playFrames, _playDur);

    const ctx = canvas.getContext("2d");
    const x   = (ms / _playDur) * canvas.width;
    ctx.beginPath();
    ctx.strokeStyle = "#fff";
    ctx.lineWidth   = 2;
    ctx.setLineDash([4, 3]);
    ctx.moveTo(x, 0);
    ctx.lineTo(x, canvas.height);
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // ── Recording timer loop ────────────────────────────────────────────────────
  function _recordTick() {
    const t      = Math.round(performance.now() - _recStart);
    const angles = _readJointsDeg();
    _recFrames.push({ t, ...angles });

    // Live waveform refresh every ~10 frames
    if (_recFrames.length % 10 === 0) {
      _drawWaveform(_recFrames, t);
    }

    const sec = (t / 1000).toFixed(1);
    _setRecStatus(`🔴 Recording… ${sec}s  (${_recFrames.length} frames)`, "#c0392b");
  }

  // ── Public: start recording ─────────────────────────────────────────────────
  global.startRecording = function () {
    if (_recTimer) return;
    if (typeof cancelAnim === "function") cancelAnim();

    _recFrames = [];
    _recStart  = performance.now();
    _recTimer  = setInterval(_recordTick, SAMPLE_MS);
    _setRecStatus("🔴 Recording started…", "#c0392b");

    document.getElementById("recStartBtn")?.setAttribute("disabled", true);
    document.getElementById("recStopBtn")?.removeAttribute("disabled");
  };

  // ── Public: stop recording + download ──────────────────────────────────────
  global.stopRecording = function () {
    if (!_recTimer) return;
    clearInterval(_recTimer);
    _recTimer = null;

    document.getElementById("recStartBtn")?.removeAttribute("disabled");
    document.getElementById("recStopBtn")?.setAttribute("disabled", true);

    if (_recFrames.length < 2) {
      _setRecStatus("No data recorded.", "orange");
      return;
    }

    const durationMs = _recFrames[_recFrames.length - 1].t;
    const payload = {
      version:      1,
      sampleRateHz: SAMPLE_HZ,
      durationMs,
      frames:       _recFrames,
    };

    const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
    const url  = URL.createObjectURL(blob);
    const a    = document.createElement("a");
    a.href     = url;
    a.download = `arm_motion_${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);

    _setRecStatus(
      `✅ Saved — ${_recFrames.length} frames, ${(durationMs / 1000).toFixed(2)}s`,
      "green"
    );

    // Draw final waveform in the recorder panel
    _drawWaveform(_recFrames, durationMs);
  };

  // ── Public: load recording from file ───────────────────────────────────────
  global.loadRecording = function (file) {
    if (!file) return;
    const reader = new FileReader();
    reader.onload = function (e) {
      try {
        const data = JSON.parse(e.target.result);
        if (!data.frames || !data.frames.length) throw new Error("No frames in file.");

        _playFrames  = data.frames;
        _playDur     = data.durationMs || _playFrames[_playFrames.length - 1].t;
        _playOffset  = 0;
        _isPlaying   = false;

        _stopPlayback();
        _seekTo(0);
        _drawWaveform(_playFrames, _playDur);
        _updatePlayerUI(0);
        _updatePlayBtn();

        _setPlayerStatus(
          `Loaded: ${_playFrames.length} frames, ${(_playDur / 1000).toFixed(2)}s  @ ${data.sampleRateHz ?? "?"}Hz`,
          "green"
        );

        document.getElementById("playerControls")?.style.removeProperty("display");

      } catch (err) {
        _setPlayerStatus("Error loading file: " + err.message, "red");
      }
    };
    reader.readAsText(file);
  };

  // ── Public: play / pause toggle ────────────────────────────────────────────
  global.togglePlayback = function () {
    if (!_playFrames.length) {
      _setPlayerStatus("Load a recording first.", "orange");
      return;
    }

    if (_isPlaying) {
      // Pause: store elapsed offset
      _playOffset += performance.now() - _playStart;
      _stopPlayback();
      _setPlayerStatus("Paused", "#e67e22");
    } else {
      // Play (or resume)
      if (typeof cancelAnim === "function") cancelAnim();
      if (_playOffset >= _playDur) _playOffset = 0;   // restart if at end
      _isPlaying = true;
      _playStart = performance.now();
      _updatePlayBtn();
      _setPlayerStatus("Playing…", "#27ae60");
      _playTimer = requestAnimationFrame(_playLoop);
    }
  };

  // ── Public: stop and rewind ─────────────────────────────────────────────────
  global.stopAndRewind = function () {
    _stopPlayback();
    _playOffset = 0;
    _seekTo(0);
    _drawPlayhead(0);
    _setPlayerStatus("Stopped", "#555");
  };

  // ── Scrub bar interaction ──────────────────────────────────────────────────
  global.onPlayerBarInput = function (el) {
    _isScrubbing = true;
    const ms = (el.value / 1000) * _playDur;
    _playOffset = ms;
    _seekTo(ms);
    _drawPlayhead(ms);
  };

  global.onPlayerBarChange = function (el) {
    _isScrubbing = false;
    const ms = (el.value / 1000) * _playDur;
    _playOffset = ms;
    if (_isPlaying) {
      _playStart = performance.now();
    }
  };

  // ── Waveform canvas click-to-seek ──────────────────────────────────────────
  global.onWaveformClick = function (e, canvas) {
    if (!_playDur) return;
    const rect = canvas.getBoundingClientRect();
    const x    = e.clientX - rect.left;
    const ms   = (x / rect.width) * _playDur;
    _playOffset = ms;
    if (_isPlaying) _playStart = performance.now();
    _seekTo(ms);
    _drawPlayhead(ms);

    const bar = document.getElementById("playerBar");
    if (bar) bar.value = (ms / _playDur) * 1000;
  };

})(window);
