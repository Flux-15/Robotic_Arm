/* ============================================================
   ARM CONSOLE — Logic & Interaction
   FK/IK kinematics · Serial communication · Theme · Navigation
   ============================================================ */

// -----------------------------------------------------------------
// Constants
// -----------------------------------------------------------------
const D2R = Math.PI / 180;
const R2D = 180 / Math.PI;
const JOINTS = [
  { id: 't1', label: 'base' },
  { id: 't2', label: 'shoulder' },
  { id: 't3', label: 'elbow' },
  { id: 't4', label: 'wrist' },
  { id: 't5', label: 'roll' }
];

// -----------------------------------------------------------------
// State
// -----------------------------------------------------------------
function freshArmState() {
  return {
    mode: 'fk',
    fk: { t1: 0, t2: 30, t3: -40, t4: -10, t5: 0 },
    ik: { px: 250, py: 0, pz: 150, psi: -30, t5: 0, elbow: 1 },
    cfg: { L0: 75, A: 14, B: 12, L1: 102, L2: 130, L34: 173 },
    invert: { t1: false, t2: false, t3: false, t4: false, t5: false },
    port: null, reader: null, writer: null, connected: false
  };
}
let arms = { A: freshArmState(), B: freshArmState() };
let activeArm = 'A';
let topology = 'single';

// -----------------------------------------------------------------
// Theme
// -----------------------------------------------------------------
function initTheme() {
  const saved = localStorage.getItem('armconsole-theme');
  const theme = saved || 'dark';
  document.documentElement.setAttribute('data-theme', theme);
  updateThemeUI(theme);
}
function toggleTheme() {
  const current = document.documentElement.getAttribute('data-theme');
  const next = current === 'dark' ? 'light' : 'dark';
  document.documentElement.setAttribute('data-theme', next);
  localStorage.setItem('armconsole-theme', next);
  updateThemeUI(next);
  // Re-render canvases with new theme colors
  render();
  drawWiring();
}
function updateThemeUI(theme) {
  const icon = document.getElementById('themeIcon');
  const label = document.getElementById('themeLabel');
  if (theme === 'dark') {
    icon.innerHTML = '<path d="M21 12.79A9 9 0 1111.21 3 7 7 0 0021 12.79z"/>';
    label.textContent = 'Light mode';
  } else {
    icon.innerHTML = '<circle cx="12" cy="12" r="5"/><line x1="12" y1="1" x2="12" y2="3"/><line x1="12" y1="21" x2="12" y2="23"/><line x1="4.22" y1="4.22" x2="5.64" y2="5.64"/><line x1="18.36" y1="18.36" x2="19.78" y2="19.78"/><line x1="1" y1="12" x2="3" y2="12"/><line x1="21" y1="12" x2="23" y2="12"/><line x1="4.22" y1="19.78" x2="5.64" y2="18.36"/><line x1="18.36" y1="5.64" x2="19.78" y2="4.22"/>';
    label.textContent = 'Dark mode';
  }
}
document.getElementById('themeToggle').addEventListener('click', toggleTheme);

// -----------------------------------------------------------------
// Navigation
// -----------------------------------------------------------------
function switchSection(name) {
  document.querySelectorAll('.section').forEach(s => s.classList.remove('active'));
  document.querySelectorAll('.nav-item').forEach(n => n.classList.remove('active'));
  const sec = document.getElementById('sec-' + name);
  const nav = document.getElementById('nav-' + name);
  if (sec) sec.classList.add('active');
  if (nav) nav.classList.add('active');

  // Close mobile sidebar
  document.getElementById('sidebar').classList.remove('open');
  document.getElementById('sidebarOverlay').classList.remove('visible');
}
document.querySelectorAll('.nav-item').forEach(btn => {
  btn.addEventListener('click', () => switchSection(btn.dataset.section));
});

// Mobile menu
document.getElementById('mobileMenuBtn').addEventListener('click', () => {
  document.getElementById('sidebar').classList.toggle('open');
  document.getElementById('sidebarOverlay').classList.toggle('visible');
});
document.getElementById('sidebarOverlay').addEventListener('click', () => {
  document.getElementById('sidebar').classList.remove('open');
  document.getElementById('sidebarOverlay').classList.remove('visible');
});

// -----------------------------------------------------------------
// Toast Notifications
// -----------------------------------------------------------------
function showToast(message, type = 'info') {
  const container = document.getElementById('toastContainer');
  const toast = document.createElement('div');
  toast.className = `toast toast--${type}`;

  let iconSvg = '';
  if (type === 'success') {
    iconSvg = '<svg class="toast__icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><polyline points="20 6 9 17 4 12"/></svg>';
  } else if (type === 'error') {
    iconSvg = '<svg class="toast__icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="10"/><line x1="15" y1="9" x2="9" y2="15"/><line x1="9" y1="9" x2="15" y2="15"/></svg>';
  } else {
    iconSvg = '<svg class="toast__icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"><circle cx="12" cy="12" r="10"/><line x1="12" y1="16" x2="12" y2="12"/><line x1="12" y1="8" x2="12.01" y2="8"/></svg>';
  }
  toast.innerHTML = iconSvg + `<span>${message}</span>`;
  container.appendChild(toast);

  setTimeout(() => {
    toast.classList.add('leaving');
    toast.addEventListener('animationend', () => toast.remove());
  }, 2200);
}

// -----------------------------------------------------------------
// Slider fill update
// -----------------------------------------------------------------
function updateSliderFill(input) {
  const min = parseFloat(input.min);
  const max = parseFloat(input.max);
  const val = parseFloat(input.value);
  const pct = ((val - min) / (max - min)) * 100;
  input.style.setProperty('--fill-pct', pct + '%');
}
function initAllSliders() {
  document.querySelectorAll('input[type="range"]').forEach(updateSliderFill);
}

// -----------------------------------------------------------------
// CSS Variable Helper (for theme-aware canvas)
// -----------------------------------------------------------------
function cssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

// -----------------------------------------------------------------
// Legend colors (theme-aware)
// -----------------------------------------------------------------
function updateLegendColors() {
  const linkDot = document.getElementById('legendLink');
  const jointDot = document.getElementById('legendJoint');
  const effectorDot = document.getElementById('legendEffector');
  if (linkDot) linkDot.style.background = cssVar('--canvas-link');
  if (jointDot) jointDot.style.background = cssVar('--canvas-joint');
  if (effectorDot) effectorDot.style.background = cssVar('--canvas-effector');
}

// -----------------------------------------------------------------
// FK / IK Kinematics (unchanged)
// -----------------------------------------------------------------
function fk(t1, t2, t3, t4, c) {
  const t1r = t1 * D2R, t2r = t2 * D2R, t3r = t3 * D2R, t4r = t4 * D2R;
  const sum23 = t2r + t3r, sum234 = t2r + t3r + t4r;
  const r = c.B + c.L1 * Math.cos(t2r) + c.L2 * Math.cos(sum23) + c.L34 * Math.cos(sum234);
  const z = c.L0 + c.A + c.L1 * Math.sin(t2r) + c.L2 * Math.sin(sum23) + c.L34 * Math.sin(sum234);
  const Px = r * Math.cos(t1r), Py = r * Math.sin(t1r);
  return { Px, Py, Pz: z, psi: (t2 + t3 + t4), r, joints: fkJoints(t2r, t3r, t4r, c) };
}
function fkJoints(t2r, t3r, t4r, c) {
  const sum23 = t2r + t3r, sum234 = t2r + t3r + t4r;
  const p0 = { r: c.B, z: c.L0 + c.A };
  const p1 = { r: p0.r + c.L1 * Math.cos(t2r), z: p0.z + c.L1 * Math.sin(t2r) };
  const p2 = { r: p1.r + c.L2 * Math.cos(sum23), z: p1.z + c.L2 * Math.sin(sum23) };
  const p3 = { r: p2.r + c.L34 * Math.cos(sum234), z: p2.z + c.L34 * Math.sin(sum234) };
  return [{ r: 0, z: c.L0 }, p0, p1, p2, p3];
}
function ik(Px, Py, Pz, psi, elbowSign, c) {
  const t1 = Math.atan2(Py, Px) * R2D;
  const rr = Math.hypot(Px, Py) - c.B;
  const zz = Pz - c.L0 - c.A;
  const psir = psi * D2R;
  const Wr = rr - c.L34 * Math.cos(psir);
  const Wz = zz - c.L34 * Math.sin(psir);
  const L1 = c.L1, L2 = c.L2;
  let D = (Wr * Wr + Wz * Wz - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  const reachable = D >= -1 && D <= 1;
  D = Math.max(-1, Math.min(1, D));
  const t3r = Math.atan2(elbowSign * Math.sqrt(1 - D * D), D);
  const t2r = Math.atan2(Wz, Wr) - Math.atan2(L2 * Math.sin(t3r), L1 + L2 * Math.cos(t3r));
  const t2 = t2r * R2D, t3 = t3r * R2D;
  const t4 = psi - t2 - t3;
  return { t1, t2, t3, t4, reachable, joints: fkJoints(t2r, t3r, t4 * D2R, c) };
}
function fmt(n) { return (Math.round(n * 100) / 100).toFixed(2); }

// -----------------------------------------------------------------
// Canvas Drawing (theme-aware)
// -----------------------------------------------------------------
function drawSide(joints) {
  const cv = document.getElementById('sideView'), ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);

  const gridColor = cssVar('--canvas-grid');
  const linkColor = cssVar('--canvas-link');
  const jointColor = cssVar('--canvas-joint');
  const effectorColor = cssVar('--canvas-effector');

  const scale = 0.45, ox = 40, oz = H - 40;

  // Grid lines
  ctx.strokeStyle = gridColor;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, oz); ctx.lineTo(W, oz);
  ctx.moveTo(ox, 0); ctx.lineTo(ox, H);
  ctx.stroke();

  // Arm links
  ctx.strokeStyle = linkColor;
  ctx.lineWidth = 4;
  ctx.lineCap = 'round';
  ctx.beginPath();
  joints.forEach((p, i) => {
    const x = ox + p.r * scale, y = oz - p.z * scale;
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();

  // Joints
  joints.forEach((p, i) => {
    const x = ox + p.r * scale, y = oz - p.z * scale;
    ctx.fillStyle = (i === joints.length - 1) ? effectorColor : jointColor;
    ctx.beginPath();
    ctx.arc(x, y, i === joints.length - 1 ? 6 : 5, 0, 7);
    ctx.fill();
  });
}

function drawTop(r, t1deg) {
  const cv = document.getElementById('topView'), ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.clearRect(0, 0, W, H);

  const gridColor = cssVar('--canvas-grid');
  const linkColor = cssVar('--canvas-link');
  const jointColor = cssVar('--canvas-joint');
  const effectorColor = cssVar('--canvas-effector');
  const circleColor = cssVar('--canvas-circle');

  const cx = W / 2, cy = H / 2, scale = 0.45;

  // Grid
  ctx.strokeStyle = gridColor;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, cy); ctx.lineTo(W, cy);
  ctx.moveTo(cx, 0); ctx.lineTo(cx, H);
  ctx.stroke();

  // Reach circle
  ctx.beginPath();
  ctx.arc(cx, cy, Math.abs(r) * scale, 0, Math.PI * 2);
  ctx.strokeStyle = circleColor;
  ctx.setLineDash([3, 4]);
  ctx.stroke();
  ctx.setLineDash([]);

  // Arm projection
  const t1r = t1deg * D2R;
  const ex = cx + r * scale * Math.cos(t1r);
  const ey = cy - r * scale * Math.sin(t1r);
  ctx.strokeStyle = linkColor;
  ctx.lineWidth = 4;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(cx, cy); ctx.lineTo(ex, ey);
  ctx.stroke();

  // Joints
  ctx.fillStyle = jointColor;
  ctx.beginPath(); ctx.arc(cx, cy, 5, 0, 7); ctx.fill();
  ctx.fillStyle = effectorColor;
  ctx.beginPath(); ctx.arc(ex, ey, 6, 0, 7); ctx.fill();
}

// -----------------------------------------------------------------
// Wiring Diagram (theme-aware)
// -----------------------------------------------------------------
function drawWiring() {
  const svg = document.getElementById('wireSvg');
  const boardFill = cssVar('--bg-elevated');
  const boardStroke = cssVar('--border-strong');
  const wireStroke = cssVar('--border-strong');
  const textColor = cssVar('--text-secondary');
  const mutedColor = cssVar('--text-muted');
  const connColor = cssVar('--success');
  const disconnColor = cssVar('--border');

  const board = (x, y, w, h, label) =>
    `<rect x="${x}" y="${y}" width="${w}" height="${h}" rx="5" fill="${boardFill}" stroke="${boardStroke}"/>
     <text x="${x + w / 2}" y="${y + h / 2 + 4}" font-size="8.5" fill="${textColor}" font-family="JetBrains Mono" text-anchor="middle">${label}</text>`;
  const arm = (x, y, label, connected) =>
    `<circle cx="${x}" cy="${y}" r="9" fill="${connected ? connColor + '15' : boardFill}" stroke="${connected ? connColor : disconnColor}" stroke-width="1.4"/>
     <text x="${x}" y="${y + 3.5}" font-size="8.5" fill="${connected ? connColor : mutedColor}" font-family="JetBrains Mono" text-anchor="middle">${label}</text>`;
  const wire = (x1, y1, x2, y2) =>
    `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${wireStroke}" stroke-width="1.4" stroke-dasharray="3,3"/>`;

  let html = '';
  if (topology === 'single') {
    html += board(20, 20, 90, 36, 'ESP32 · PCA9685');
    html += wire(110, 38, 150, 38);
    html += arm(165, 38, 'A', arms.A.connected);
  } else if (topology === 'shared') {
    html += board(20, 20, 90, 36, 'ESP32 · PCA9685');
    html += wire(110, 30, 150, 15); html += wire(110, 46, 150, 61);
    html += arm(165, 15, 'A', arms.A.connected);
    html += arm(165, 61, 'B', arms.B.connected);
  } else {
    html += board(10, 8, 90, 26, 'ESP32 A');
    html += board(10, 42, 90, 26, 'ESP32 B');
    html += wire(100, 21, 140, 21); html += wire(100, 55, 140, 55);
    html += arm(155, 21, 'A', arms.A.connected);
    html += arm(155, 55, 'B', arms.B.connected);
  }
  svg.innerHTML = html;
}

// -----------------------------------------------------------------
// Invert List UI
// -----------------------------------------------------------------
function buildInvertList() {
  const el = document.getElementById('invertList');
  el.innerHTML = JOINTS.map(j => `
    <div class="invert-row">
      <span>${j.id} ${j.label}</span>
      <input type="checkbox" class="invert-toggle" data-joint="${j.id}" ${arms[activeArm].invert[j.id] ? 'checked' : ''}>
    </div>`).join('');
  el.querySelectorAll('.invert-toggle').forEach(cb => {
    cb.addEventListener('change', e => {
      arms[activeArm].invert[e.target.dataset.joint] = e.target.checked;
      render();
    });
  });
}

// -----------------------------------------------------------------
// Main Render
// -----------------------------------------------------------------
function render() {
  const st = arms[activeArm];
  const c = st.cfg;
  let t1, t2, t3, t4, t5, pose, joints, ikRes;

  if (st.mode === 'fk') {
    t1 = st.fk.t1; t2 = st.fk.t2; t3 = st.fk.t3; t4 = st.fk.t4; t5 = st.fk.t5;
    pose = fk(t1, t2, t3, t4, c);
    joints = pose.joints;
    document.getElementById('unreachWarn').style.display = 'none';
  } else {
    const { px, py, pz, psi, t5: ikt5, elbow } = st.ik;
    ikRes = ik(px, py, pz, psi, elbow, c);
    t1 = ikRes.t1; t2 = ikRes.t2; t3 = ikRes.t3; t4 = ikRes.t4; t5 = ikt5;
    joints = ikRes.joints;
    pose = fk(t1, t2, t3, t4, c);
    document.getElementById('unreachWarn').style.display = ikRes.reachable ? 'none' : 'block';
  }

  drawSide(joints);
  drawTop(pose.r, t1);
  updateLegendColors();

  document.getElementById('readout').innerHTML =
    `<span class="k">theta1</span> <span class="v">${fmt(t1)}</span>°&nbsp;&nbsp;` +
    `<span class="k">theta2</span> <span class="v">${fmt(t2)}</span>°<br>` +
    `<span class="k">theta3</span> <span class="v">${fmt(t3)}</span>°&nbsp;&nbsp;` +
    `<span class="k">theta4</span> <span class="v">${fmt(t4)}</span>°<br>` +
    `<span class="k">theta5</span> <span class="v">${fmt(t5)}</span>° (roll, free)<br>` +
    `<span class="k">Px</span> <span class="v">${fmt(pose.Px)}</span>&nbsp;&nbsp;` +
    `<span class="k">Py</span> <span class="v">${fmt(pose.Py)}</span>&nbsp;&nbsp;` +
    `<span class="k">Pz</span> <span class="v">${fmt(pose.Pz)}</span>&nbsp;mm<br>` +
    `<span class="k">psi</span> <span class="v">${fmt(pose.psi)}</span>° (wrist angle vs horizon)`;

  const raw = { t1, t2, t3, t4, t5 };
  const sent = {};
  JOINTS.forEach(j => sent[j.id] = st.invert[j.id] ? -raw[j.id] : raw[j.id]);

  const armPrefix = (topology === 'shared') ? (activeArm + ' ') : '';
  const cmd = `${armPrefix}J ${fmt(sent.t1)} ${fmt(sent.t2)} ${fmt(sent.t3)} ${fmt(sent.t4)} ${fmt(sent.t5)} 90`;
  document.getElementById('cmdStr').textContent = cmd;

  updateSendAvailability();
  if (document.getElementById('liveSend').checked && canSendActiveArm()) {
    sendToArm(activeArm, cmd, true);
  }
}

// -----------------------------------------------------------------
// UI Binding — FK/IK Inputs
// -----------------------------------------------------------------
function bindFkIkInputs() {
  document.querySelectorAll('#fkControls input[type=range]').forEach(inp => {
    inp.addEventListener('input', () => {
      arms[activeArm].fk[inp.id] = +inp.value;
      document.getElementById(inp.id + 'v').textContent = inp.value + '°';
      updateSliderFill(inp);
      render();
    });
  });
  document.getElementById('t5ik').addEventListener('input', (e) => {
    arms[activeArm].ik.t5 = +e.target.value;
    document.getElementById('t5ikv').textContent = e.target.value + '°';
    updateSliderFill(e.target);
    render();
  });
  const ikMap = { px: 'px', py: 'py', pz: 'pz', psi: 'psi', elbow: 'elbow' };
  Object.keys(ikMap).forEach(id => {
    const el = document.getElementById(id);
    el.addEventListener('input', () => { arms[activeArm].ik[ikMap[id]] = +el.value; render(); });
    el.addEventListener('change', () => { arms[activeArm].ik[ikMap[id]] = +el.value; render(); });
  });
  document.querySelectorAll('.config-grid input').forEach(inp => {
    const key = inp.id.replace('c', '');
    inp.addEventListener('input', () => { arms[activeArm].cfg[key] = +inp.value; render(); });
  });
}

function loadArmIntoUI() {
  const st = arms[activeArm];
  document.getElementById('vizArmLabel').textContent = 'Arm ' + activeArm;

  JOINTS.slice(0, 5).forEach(j => {
    const el = document.getElementById(j.id);
    if (el) {
      el.value = st.fk[j.id];
      document.getElementById(j.id + 'v').textContent = st.fk[j.id] + '°';
      updateSliderFill(el);
    }
  });
  document.getElementById('px').value = st.ik.px;
  document.getElementById('py').value = st.ik.py;
  document.getElementById('pz').value = st.ik.pz;
  document.getElementById('psi').value = st.ik.psi;
  const t5ikEl = document.getElementById('t5ik');
  t5ikEl.value = st.ik.t5;
  document.getElementById('t5ikv').textContent = st.ik.t5 + '°';
  updateSliderFill(t5ikEl);
  document.getElementById('elbow').value = st.ik.elbow;

  document.getElementById('cL0').value = st.cfg.L0;
  document.getElementById('cA').value = st.cfg.A;
  document.getElementById('cB').value = st.cfg.B;
  document.getElementById('cL1').value = st.cfg.L1;
  document.getElementById('cL2').value = st.cfg.L2;
  document.getElementById('cL34').value = st.cfg.L34;

  document.getElementById('btnFK').classList.toggle('active', st.mode === 'fk');
  document.getElementById('btnIK').classList.toggle('active', st.mode === 'ik');
  document.getElementById('fkControls').style.display = st.mode === 'fk' ? 'block' : 'none';
  document.getElementById('ikControls').style.display = st.mode === 'ik' ? 'block' : 'none';

  buildInvertList();
  render();
}

// Mode buttons
document.getElementById('btnFK').addEventListener('click', () => { arms[activeArm].mode = 'fk'; loadArmIntoUI(); });
document.getElementById('btnIK').addEventListener('click', () => { arms[activeArm].mode = 'ik'; loadArmIntoUI(); });

// Copy / Send
document.getElementById('copyBtn').addEventListener('click', () => {
  navigator.clipboard.writeText(document.getElementById('cmdStr').textContent);
  showToast('Command copied to clipboard', 'success');
});
document.getElementById('sendBtn').addEventListener('click', () => {
  sendToArm(activeArm, document.getElementById('cmdStr').textContent, false);
});
document.getElementById('liveSend').addEventListener('change', render);

// -----------------------------------------------------------------
// Arm Tabs
// -----------------------------------------------------------------
document.getElementById('armTabs').addEventListener('click', (e) => {
  const tab = e.target.closest('.arm-tab');
  if (!tab) return;
  activeArm = tab.dataset.arm;
  document.querySelectorAll('.arm-tab').forEach(t => t.classList.toggle('active', t.dataset.arm === activeArm));
  loadArmIntoUI();
});

// -----------------------------------------------------------------
// Topology Switching
// -----------------------------------------------------------------
document.querySelectorAll('.topo-option').forEach(btn => {
  btn.addEventListener('click', () => {
    topology = btn.dataset.topo;
    document.querySelectorAll('.topo-option').forEach(b => b.classList.toggle('active', b === btn));
    const tabB = document.getElementById('tabB');
    const badgeB = document.getElementById('badgeB');
    tabB.style.display = (topology === 'single') ? 'none' : 'flex';
    badgeB.style.display = (topology === 'single') ? 'none' : 'inline-flex';
    if (topology === 'single' && activeArm === 'B') {
      activeArm = 'A';
      document.querySelectorAll('.arm-tab').forEach(t => t.classList.toggle('active', t.dataset.arm === 'A'));
    }
    drawWiring();
    buildConnUI();
    loadArmIntoUI();
  });
});

// -----------------------------------------------------------------
// Web Serial
// -----------------------------------------------------------------
const hasSerial = 'serial' in navigator;
if (!hasSerial) document.getElementById('noSerial').style.display = 'block';

function logLine(cls, text) {
  const c = document.getElementById('console');
  const row = document.createElement('div');
  row.className = cls;
  row.textContent = text;
  c.appendChild(row);
  c.scrollTop = c.scrollHeight;
}

async function connectTarget(target) {
  if (!hasSerial) return;
  try {
    const port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });
    const writer = port.writable.getWriter();

    if (target === 'shared') {
      arms.A.port = port; arms.A.writer = writer; arms.A.connected = true;
      arms.B.port = port; arms.B.writer = writer; arms.B.connected = true;
    } else {
      arms[target].port = port; arms[target].writer = writer; arms[target].connected = true;
    }
    logLine('sys', `--- connected (${target}) ---`);
    showToast(`Connected to ${target}`, 'success');
    readLoop(port, target);
  } catch (err) {
    logLine('sys', `connect failed: ${err.message}`);
    showToast(`Connection failed: ${err.message}`, 'error');
  }
  buildConnUI(); drawWiring(); updateSendAvailability();
}

async function readLoop(port, target) {
  const decoder = new TextDecoderStream();
  const readableClosed = port.readable.pipeTo(decoder.writable);
  const reader = decoder.readable.getReader();
  if (target === 'shared') { arms.A.reader = reader; arms.B.reader = reader; } else { arms[target].reader = reader; }
  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value) value.split(/\r?\n/).filter(Boolean).forEach(l => logLine('rx', '< ' + l));
    }
  } catch (e) { /* port closed */ }
}

async function disconnectTarget(target) {
  const list = (target === 'shared') ? ['A', 'B'] : [target];
  for (const a of list) {
    const st = arms[a];
    try { if (st.writer) { st.writer.releaseLock?.(); } } catch (e) {}
    try { if (st.reader) { await st.reader.cancel(); st.reader.releaseLock?.(); } } catch (e) {}
    try { if (st.port) await st.port.close(); } catch (e) {}
    st.port = null; st.writer = null; st.reader = null; st.connected = false;
  }
  logLine('sys', `--- disconnected (${target}) ---`);
  showToast(`Disconnected from ${target}`, 'info');
  buildConnUI(); drawWiring(); updateSendAvailability();
}

async function sendRaw(writer, text) {
  if (!writer) return;
  const data = new TextEncoder().encode(text + '\n');
  await writer.write(data);
  logLine('tx', '> ' + text);
}

function canSendActiveArm() { return !!arms[activeArm].writer; }

async function sendToArm(armKey, cmd, silent) {
  const st = arms[armKey];
  if (!st.writer) { if (!silent) logLine('sys', 'not connected'); return; }
  await sendRaw(st.writer, cmd);
}

function updateSendAvailability() {
  document.getElementById('sendBtn').disabled = !canSendActiveArm();
  document.getElementById('consoleSendBtn').disabled = !canSendActiveArm();

  // Header badges
  const dotA = document.getElementById('dotA');
  const dotB = document.getElementById('dotB');
  if (dotA) dotA.classList.toggle('on', arms.A.connected);
  if (dotB) dotB.classList.toggle('on', arms.B.connected);

  // Arm tab dots
  const dotA2 = document.getElementById('dotA2');
  const dotB2 = document.getElementById('dotB2');
  if (dotA2) dotA2.classList.toggle('on', arms.A.connected);
  if (dotB2) dotB2.classList.toggle('on', arms.B.connected);

  // Nav connection badge
  const badge = document.getElementById('nav-conn-badge');
  if (badge) {
    const anyConnected = arms.A.connected || arms.B.connected;
    badge.classList.toggle('connected', anyConnected);
  }
}

function buildConnUI() {
  const grid = document.getElementById('connGrid');
  grid.innerHTML = '';
  const rowHtml = (label, connected, onClick) => {
    const row = document.createElement('div');
    row.className = 'conn-row';
    row.innerHTML = `<span class="conn-status ${connected ? 'on' : ''}"></span><span class="conn-label">${label}</span>`;
    const btn = document.createElement('button');
    btn.className = 'conn-btn' + (connected ? ' danger' : '');
    btn.textContent = connected ? 'disconnect' : 'connect';
    btn.disabled = !hasSerial;
    btn.addEventListener('click', onClick);
    row.appendChild(btn);
    grid.appendChild(row);
  };

  if (topology === 'single') {
    rowHtml('Arm A — port', arms.A.connected, () => arms.A.connected ? disconnectTarget('A') : connectTarget('A'));
  } else if (topology === 'shared') {
    rowHtml('Arm A + B — shared port', arms.A.connected, () => arms.A.connected ? disconnectTarget('shared') : connectTarget('shared'));
  } else {
    rowHtml('Arm A — port', arms.A.connected, () => arms.A.connected ? disconnectTarget('A') : connectTarget('A'));
    rowHtml('Arm B — port', arms.B.connected, () => arms.B.connected ? disconnectTarget('B') : connectTarget('B'));
  }
  updateSendAvailability();
}

// Console input
document.getElementById('consoleInput').addEventListener('keydown', (e) => {
  if (e.key === 'Enter') doConsoleSend();
});
document.getElementById('consoleSendBtn').addEventListener('click', doConsoleSend);
function doConsoleSend() {
  const inp = document.getElementById('consoleInput');
  if (!inp.value.trim()) return;
  sendToArm(activeArm, inp.value.trim(), false);
  inp.value = '';
}

// -----------------------------------------------------------------
// Init
// -----------------------------------------------------------------
initTheme();
bindFkIkInputs();
initAllSliders();
drawWiring();
buildConnUI();
loadArmIntoUI();
