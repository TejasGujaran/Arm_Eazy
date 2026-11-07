// =============================
// CONFIG LOADER MODULE
// =============================
// Applies a loaded arm config to the scene. RobotState owns joint axes,
// types, limits, sliders, badges, and per-joint rotation/translation
// rendering (see robot_state.js). This file handles the parts RobotState
// doesn't: X3D model file paths, and the base translation offset for every
// part in the chain (including Joint1..6 — for "rotate" joints this is the
// ONLY place that offset gets set, since RobotState.render() only touches
// the `rotation` attribute for those; for "translate" joints RobotState
// recomputes translation from this same base value plus the joint's
// current value, so setting it here first is harmless and keeps behavior
// correct even before the first render() call).

(function () {
// Everything below is private to this file, so its names can never clash
// with top-level names in the other <script> files on the page. The
// functions other files may call are exported on window at the end.

console.info("[config_loader] loaded (zip / local-file support)");

// =============================
// LOCAL MODEL FILES (picked from the user's PC)
// =============================
// A web page cannot open files on the visitor's PC by path. When the user
// picks a config together with its .x3d files, or a .zip of both,
// we turn each picked model file into a blob: URL and use that instead of
// the path written in model_paths. Paths with no picked file are left as
// they are and load from the server, as before.

let _localFiles = new Map();   // lower-case file name → File
let _blobUrls   = new Map();   // lower-case file name → blob: URL

function _baseName(path) {
  return String(path).replace(/\\/g, "/").split("/").pop().toLowerCase();
}

function _isLocalOnlyPath(path) {
  // absolute PC paths the browser will never load from a web page
  return /^[a-zA-Z]:[\\/]/.test(path) || /^file:/i.test(path) || /^\\\\/.test(path);
}

let _activeBlobs = [];         // blob: URLs used by the robot chosen in the mapper

function _x3dBlobUrl(file, bucket) {
  // re-wrap with an XML type: .x3d files often have an empty MIME type
  const url = URL.createObjectURL(new Blob([file], { type: "model/x3d+xml" }));
  bucket.push(url);
  return url;
}

function _releaseActiveModels() {
  for (const url of _blobUrls.values()) URL.revokeObjectURL(url);
  for (const url of _activeBlobs) URL.revokeObjectURL(url);
  _blobUrls = new Map();
  _activeBlobs = [];
}

function setLocalModelFiles(files) {
  _releaseActiveModels();                                             // free the previous robot's files
  _localFiles = new Map();
  for (const f of files) {
    if (/\.x3d$/i.test(f.name)) _localFiles.set(f.name.toLowerCase(), f);
  }
}

function resolveModelPath(path) {
  const key = _baseName(path);
  const file = _localFiles.get(key);
  if (!file) return path;                                            // server file, unchanged
  if (!_blobUrls.has(key)) {
    // re-wrap with an XML type: .x3d files often have an empty MIME type
    _blobUrls.set(key, URL.createObjectURL(new Blob([file], { type: "model/x3d+xml" })));
  }
  return _blobUrls.get(key);
}

function _setInlineRaw(transformId, url) {
  const tf = document.getElementById(transformId);
  const inline = tf && tf.querySelector("Inline");
  if (inline) inline.setAttribute("url", url ? `"${url}"` : "");
}

function _getInlineRaw(transformId) {
  const tf = document.getElementById(transformId);
  const inline = tf && tf.querySelector("Inline");
  return inline ? inline.getAttribute("url") : null;
}

function setInlineUrl(transformId, path) {
  const tf = document.getElementById(transformId);
  if (!tf) return;

  const inline = tf.querySelector("Inline");
  if (!inline) return;

  inline.setAttribute("url", `"${resolveModelPath(path)}"`);
}

function setTransformTranslation(transformId, vec3) {
  const tf = document.getElementById(transformId);
  if (!tf) return;

  const [x, y, z] = window.SharedUtils ? window.SharedUtils.parseVec3(vec3) : vec3;
  tf.setAttribute("translation", `${x} ${y} ${z}`);
}

// modelOverrides (optional): { part: url } chosen in the model mapper.
//   url string → use it;  "" → no model;  part not listed → use cfg.model_paths
function applyArmConfig(cfg, modelOverrides = null) {

  const parts = [
    "BaseTransform","Joint1","Joint2","Joint3",
    "Joint4","Joint5","Joint6","Ik_bone"
  ];

  // ===== MODEL PATHS =====
  // (cfg itself is not changed: the backend still gets the original paths)
  const missingLocal = [];
  parts.forEach(part => {
    if (modelOverrides && part in modelOverrides) return _setInlineRaw(part, modelOverrides[part]);
    const path = cfg.model_paths && cfg.model_paths[part];
    if (!path) return;
    if (_isLocalOnlyPath(path) && !_localFiles.has(_baseName(path))) missingLocal.push(_baseName(path));
    setInlineUrl(part, path);
  });

  // ===== BASE TRANSLATIONS (see file header for why this covers all parts) =====
  if (cfg.translations_mm) {
    parts.forEach(part => {
      if (cfg.translations_mm[part]) {
        setTransformTranslation(part, cfg.translations_mm[part]);
      }
    });
  }

  // ===== JOINTS: axes, type, limits, sliders, badges, rotation/translation =====
  if (window.RobotState) {
    window.RobotState.setConfig(cfg);
  } else {
    console.warn("[config_loader] RobotState not loaded — joints will not update");
  }

  // ===== PUSH TO BACKEND (and let robot_bridge.js know the new config) =====
  if (window.setRobotConfig) {
    window.setRobotConfig(cfg);
  }

  const status = document.getElementById("configStatus");
  if (status) {
    if (missingLocal.length) {
      status.textContent = `Config applied, but these model files are on your PC and were not selected: ${missingLocal.join(", ")} — ` +
                           `pick the config together with its .x3d files, or a .zip of both.`;
      status.style.color = "#c2410c";
    } else {
      status.textContent = _localFiles.size ? `Config applied ✔ (${_localFiles.size} local model files)` : "Config applied ✔";
      status.style.color = "";
    }
  }
}


// =============================
// FILE INPUT HANDLER
// =============================

// =============================
// ZIP SUPPORT (no library: uses the browser's DecompressionStream)
// =============================
// A robot can be shared as ONE .zip containing its config .json and its
// .x3d files (folders inside the zip are fine). The zip is unpacked in the
// browser; nothing is uploaded anywhere.

async function unzipToFiles(zipFile) {
  const buf = new Uint8Array(await zipFile.arrayBuffer());
  const dv  = new DataView(buf.buffer, buf.byteOffset, buf.byteLength);

  // End of central directory record: scan back from the end
  let eocd = -1;
  for (let i = buf.length - 22; i >= Math.max(0, buf.length - 65557); i--) {
    if (dv.getUint32(i, true) === 0x06054b50) { eocd = i; break; }
  }
  if (eocd < 0) throw new Error(`${zipFile.name} is not a valid zip file.`);

  const count  = dv.getUint16(eocd + 10, true);
  let   offset = dv.getUint32(eocd + 16, true);
  if (offset === 0xffffffff) throw new Error("ZIP64 archives are not supported — please re-zip normally.");

  const utf8 = new TextDecoder("utf-8");
  const out = [];
  for (let n = 0; n < count; n++) {
    if (dv.getUint32(offset, true) !== 0x02014b50) throw new Error("Corrupt zip (central directory).");
    const flags    = dv.getUint16(offset + 8, true);
    const method   = dv.getUint16(offset + 10, true);
    const compSize = dv.getUint32(offset + 20, true);
    const nameLen  = dv.getUint16(offset + 28, true);
    const extraLen = dv.getUint16(offset + 30, true);
    const commLen  = dv.getUint16(offset + 32, true);
    const local    = dv.getUint32(offset + 42, true);
    const name     = utf8.decode(buf.subarray(offset + 46, offset + 46 + nameLen));
    offset += 46 + nameLen + extraLen + commLen;

    // skip folders, macOS junk and hidden files
    if (name.endsWith("/") || name.startsWith("__MACOSX/") || /(^|\/)\./.test(name)) continue;
    if (flags & 1) throw new Error(`"${name}" is password-protected — zip it without a password.`);

    const lNameLen  = dv.getUint16(local + 26, true);
    const lExtraLen = dv.getUint16(local + 28, true);
    const start = local + 30 + lNameLen + lExtraLen;
    const raw   = buf.subarray(start, start + compSize);

    let data;
    if (method === 0) data = raw;                                         // stored
    else if (method === 8) {                                              // deflate
      if (typeof DecompressionStream === "undefined") throw new Error("This browser cannot unpack zip files — please update it.");
      const stream = new Blob([raw]).stream().pipeThrough(new DecompressionStream("deflate-raw"));
      data = new Uint8Array(await new Response(stream).arrayBuffer());
    } else throw new Error(`"${name}" uses an unsupported zip compression (method ${method}).`);

    const file = new File([data], name.split("/").pop());
    file.zipPath = name;                       // full path inside the zip, shown in the mapper
    out.push(file);
  }
  return out;
}

async function _expandZips(files) {
  const out = [];
  for (const f of files) {
    if (/\.zip$/i.test(f.name)) out.push(...await unzipToFiles(f));
    else out.push(f);
  }
  return out;
}

// =============================
// MODEL MAPPER — choose which .x3d goes on which joint
// =============================
const MODEL_PARTS = ["BaseTransform", "Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"];
const PART_LABEL  = { BaseTransform: "Base", Joint1: "Joint 1", Joint2: "Joint 2", Joint3: "Joint 3",
                      Joint4: "Joint 4", Joint5: "Joint 5", Joint6: "Joint 6" };
const KEEP = "__keep__", NONE = "__none__";

// Best guess: 1) file named in the config's model_paths, 2) "base" in the
// name → Base, 3) a number in the name (joint3, A3, link_3 …) → that joint,
// 4) remaining files in natural order (Arm_0 … Arm_5 → Joint 1 … 6).
function guessModelAssignment(x3dFiles, cfg) {
  const byName = new Map(x3dFiles.map(f => [f.name.toLowerCase(), f]));
  const used = new Set(), out = {};
  const take = (part, f) => { if (f && !used.has(f) && !(part in out)) { out[part] = f; used.add(f); } };

  if (cfg && cfg.model_paths)
    for (const part of MODEL_PARTS) if (cfg.model_paths[part]) take(part, byName.get(_baseName(cfg.model_paths[part])));

  const sorted = x3dFiles.slice().sort((a, b) => a.name.localeCompare(b.name, undefined, { numeric: true, sensitivity: "base" }));
  take("BaseTransform", sorted.find(f => !used.has(f) && /base/i.test(f.name)));

  for (const f of sorted) {
    if (used.has(f)) continue;
    const m = f.name.match(/(?:joint|axis|link|a|j)[\s_-]?([1-6])(?!\d)/i);
    if (m) take(`Joint${m[1]}`, f);
  }

  const rest = sorted.filter(f => !used.has(f));
  const free = MODEL_PARTS.filter(p => !(p in out));
  if (free[0] === "BaseTransform" && rest.length >= free.length) take("BaseTransform", rest.shift());
  for (const part of free) if (!(part in out) && rest.length) take(part, rest.shift());
  return out;
}

// Apply, then free every blob: URL that no part is showing any more.
function _commitModels(newBlobs, applyFn) {
  const prev = [..._activeBlobs, ..._blobUrls.values(), ...newBlobs];
  _activeBlobs = []; _blobUrls = new Map(); _localFiles = new Map();
  applyFn();
  const inUse = new Set(MODEL_PARTS.map(p => (_getInlineRaw(p) || "").replace(/^"|"$/g, "")));
  for (const u of new Set(prev)) { if (inUse.has(u)) _activeBlobs.push(u); else URL.revokeObjectURL(u); }
}

// "Reassign models…" next to the status, to reopen the mapper for the last zip
function _showReassign(files, sourceName) {
  const status = document.getElementById("configStatus");
  if (!status) return;
  let btn = document.getElementById("reassignModelsBtn");
  if (!btn) {
    btn = document.createElement("button");
    btn.id = "reassignModelsBtn"; btn.type = "button";
    btn.textContent = "Reassign models…";
    btn.style.marginLeft = "8px";
    status.insertAdjacentElement("afterend", btn);
  }
  btn.style.display = "";
  btn.title = `Choose which file from ${sourceName} goes on which joint`;
  btn.onclick = () => { try { openModelMapper(files, sourceName); } catch (e) { status.textContent = "Error: " + e.message; } };
}
function _hideReassign() {
  const btn = document.getElementById("reassignModelsBtn");
  if (btn) btn.style.display = "none";
}

// Zip whose config names a file for every part that has a path:
// set the twin up directly. Returns false if the mapper is needed.
async function _tryAutoApply(files, sourceName) {
  let picked;
  try { picked = await _pickConfig(files); } catch { return false; }
  const cfg = picked.cfg, paths = cfg.model_paths || {};
  const x3d = new Map(files.filter(f => /\.x3d$/i.test(f.name)).map(f => [f.name.toLowerCase(), f]));
  const wanted = MODEL_PARTS.filter(p => paths[p]);
  if (!wanted.length || !wanted.every(p => x3d.has(_baseName(paths[p])))) return false;

  const blobs = [], overrides = {};
  for (const p of wanted) overrides[p] = _x3dBlobUrl(x3d.get(_baseName(paths[p])), blobs);
  _commitModels(blobs, () => applyArmConfig(cfg, overrides));
  const status = document.getElementById("configStatus");
  if (status) status.textContent = `Config applied ✔ (${picked.name} + ${wanted.length} models from ${sourceName})`;
  console.log(`[config_loader] ${sourceName}: ${picked.name}, ${wanted.length} models assigned from the config`);
  _showReassign(files, sourceName);
  return true;
}

function openModelMapper(allFiles, sourceName) {
  const x3d   = allFiles.filter(f => /\.x3d$/i.test(f.name));
  const jsons = allFiles.filter(f => /\.json$/i.test(f.name));
  const status = document.getElementById("configStatus");
  if (!x3d.length) throw new Error(`No .x3d model files in ${sourceName}.`);

  const current = window.RobotState && window.RobotState.getConfig ? window.RobotState.getConfig() : null;
  const originals = Object.fromEntries(MODEL_PARTS.map(p => [p, _getInlineRaw(p)]));
  const sessionBlobs = [], blobOf = new Map();
  const urlFor = (f) => { if (!blobOf.has(f)) blobOf.set(f, _x3dBlobUrl(f, sessionBlobs)); return blobOf.get(f); };
  const label  = (f) => f.zipPath || f.webkitRelativePath || f.name;

  // ---- dialog ------------------------------------------------------------
  const overlay = document.createElement("div");
  overlay.id = "modelMapper";
  overlay.style.cssText = "position:fixed;inset:0;background:rgba(0,0,0,.35);display:flex;align-items:center;justify-content:center;z-index:10000;font-family:Arial,sans-serif";
  const box = document.createElement("div");
  box.style.cssText = "background:#fff;border-radius:8px;padding:18px 20px;min-width:440px;max-width:90vw;max-height:85vh;overflow:auto;box-shadow:0 8px 30px rgba(0,0,0,.25)";
  box.innerHTML = `<h3 style="margin:0 0 4px">Assign model files</h3>
    <div style="font-size:.85em;color:#555;margin-bottom:12px"></div>`;
  box.querySelector("div").textContent = `${sourceName} — ${x3d.length} model file(s). The robot updates as you choose.`;

  const grid = document.createElement("div");
  grid.style.cssText = "display:grid;grid-template-columns:auto 1fr;gap:6px 10px;align-items:center";
  box.appendChild(grid);

  const addRow = (text, el) => {
    const l = document.createElement("span"); l.textContent = text; l.style.cssText = "width:auto;font-size:.9em";
    grid.append(l, el); el.style.cssText = "padding:3px;min-width:260px";
  };

  // config to use for geometry
  const cfgSel = document.createElement("select");
  jsons.forEach((f, i) => cfgSel.add(new Option(`Config: ${label(f)}`, String(i))));
  if (current) cfgSel.add(new Option("Keep current config (geometry unchanged)", "current"));
  if (!cfgSel.options.length) throw new Error(`No config .json in ${sourceName} and no config loaded yet.`);
  addRow("Config:", cfgSel);

  const selects = {};
  for (const part of MODEL_PARTS) {
    const sel = document.createElement("select");
    sel.add(new Option("— keep current model —", KEEP));
    sel.add(new Option("— no model —", NONE));
    x3d.forEach((f, i) => sel.add(new Option(label(f), String(i))));
    sel.addEventListener("change", () => preview(part));
    selects[part] = sel;
    addRow(PART_LABEL[part] + ":", sel);
  }

  const btns = document.createElement("div");
  btns.style.cssText = "display:flex;justify-content:flex-end;gap:8px;margin-top:14px";
  const cancel = document.createElement("button"); cancel.textContent = "Cancel";
  const apply  = document.createElement("button"); apply.textContent = "Apply"; apply.style.fontWeight = "bold";
  btns.append(cancel, apply); box.appendChild(btns);
  overlay.appendChild(box); document.body.appendChild(overlay);

  // ---- behaviour ---------------------------------------------------------
  let cfgCache = new Map();
  async function chosenConfig() {
    const v = cfgSel.value;
    if (v === "current") return current;
    if (!cfgCache.has(v)) cfgCache.set(v, JSON.parse(await jsons[+v].text()));
    return cfgCache.get(v);
  }
  function overrideFor(part) {
    const v = selects[part].value;
    return v === KEEP ? originals[part] && originals[part].replace(/^"|"$/g, "") : v === NONE ? "" : urlFor(x3d[+v]);
  }
  function preview(part) { _setInlineRaw(part, overrideFor(part)); }

  async function guessAll() {
    let cfg = null;
    try { cfg = await chosenConfig(); } catch (e) { if (status) status.textContent = "Error: " + e.message; }
    const g = guessModelAssignment(x3d, cfg);
    for (const part of MODEL_PARTS) {
      selects[part].value = g[part] ? String(x3d.indexOf(g[part])) : KEEP;
      preview(part);
    }
  }
  cfgSel.addEventListener("change", guessAll);

  const close = () => overlay.remove();
  cancel.addEventListener("click", () => {
    for (const part of MODEL_PARTS) _setInlineRaw(part, originals[part] ? originals[part].replace(/^"|"$/g, "") : "");
    for (const url of sessionBlobs) URL.revokeObjectURL(url);
    close();
    if (status) { status.textContent = "Model assignment cancelled"; status.style.color = ""; }
  });
  apply.addEventListener("click", async () => {
    try {
      const cfg = await chosenConfig();
      const overrides = {};
      for (const part of MODEL_PARTS) if (selects[part].value !== KEEP) overrides[part] = overrideFor(part);
      close();
      _commitModels(sessionBlobs, () => applyArmConfig(cfg, overrides));
      if (status) status.textContent = `Config applied ✔ (models from ${sourceName})`;
      _showReassign(allFiles, sourceName);
    } catch (e) {
      if (status) { status.textContent = "Error: " + e.message; status.style.color = "#b91c1c"; }
    }
  });

  guessAll();
  return { overlay, selects, cfgSel, apply, cancel };   // handles for testing
}

// Pick the config among the selected files: the only .json, or the one that
// looks like a robot config (has model_paths / translations_mm).
async function _pickConfig(files) {
  const jsons = files.filter(f => /\.json$/i.test(f.name));
  if (!jsons.length) throw new Error("No .json config among the selected files.");
  let fallback = null;
  for (const f of jsons) {
    try {
      const cfg = JSON.parse(await f.text());
      if (cfg && typeof cfg === "object") {
        if (cfg.model_paths || cfg.translations_mm) return { cfg, name: f.name };
        fallback = fallback || { cfg, name: f.name };
      }
    } catch { /* not valid JSON — try the next one */ }
  }
  if (fallback) return fallback;
  throw new Error("Invalid JSON structure.");
}

// A .zip → set up from its config, or let the user assign files to joints
async function _loadFromFiles(fileList) {
  const status = document.getElementById("configStatus");
  let files = Array.from(fileList || []);
  if (!files.length) return;
  try {
    if (status) { status.textContent = "Loading..."; status.style.color = ""; }
    const zipName = (files.find(f => /\.zip$/i.test(f.name)) || {}).name;
    files = await _expandZips(files);
    if (zipName) {
      if (await _tryAutoApply(files, zipName)) return;
      if (status) status.textContent = "Choose which model goes on which joint…";
      return openModelMapper(files, zipName);
    }
    _hideReassign();
    const { cfg, name } = await _pickConfig(files);
    setLocalModelFiles(files);
    console.log(`[config_loader] ${name} + ${_localFiles.size} local model file(s)`);
    applyArmConfig(cfg);
  } catch (err) {
    if (status) { status.textContent = "Error: " + err.message; status.style.color = "#b91c1c"; }
  }
}

function setupConfigLoader() {
  const input = document.getElementById("configFileInput");
  if (!input) return;

  // Allow the config + its .x3d files to be selected together
  input.multiple = true;
  input.accept = ".json,application/json,.x3d,.zip,application/zip";
  input.title = "Select the config .json (plus its .x3d files), or one .zip containing both";
  input.addEventListener("change", () => { _loadFromFiles(input.files); input.value = ""; });
}

// =============================
// DEFAULT CONFIG BOOTSTRAP
// =============================
// Load config/default_config.json automatically on startup, same as
// config_editor.html does, so the twin doesn't rely solely on the user
// picking a file before anything works.

async function loadDefaultConfigOnStartup() {
  try {
    const res = await fetch("../../config/default_config.json");
    if (!res.ok) throw new Error("HTTP " + res.status);
    const cfg = await res.json();
    applyArmConfig(cfg);
  } catch (err) {
    console.warn("[config_loader] Could not load default_config.json, using SharedUtils fallback:", err.message);
    // RobotState/kinematics.js already fall back to window.SharedUtils.FALLBACK_GEOMETRY
    // on their own when state.config is null, so no further action is needed here.
  }
}

document.addEventListener("DOMContentLoaded", () => {
  setupConfigLoader();
  loadDefaultConfigOnStartup();
});

// ---- public API (same global names as before, plus the zip helpers) ----
Object.assign(window, {
  applyArmConfig, setInlineUrl, setTransformTranslation,
  setupConfigLoader, loadDefaultConfigOnStartup,
  unzipToFiles, openModelMapper, guessModelAssignment, _loadFromFiles,
});
})();