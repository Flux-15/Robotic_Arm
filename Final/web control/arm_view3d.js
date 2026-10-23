/* ============================================================
   ARM VIEW 3D — three.js viewer for the Arm Console
   Replaces the 2D side/top canvases. Driven by the same FK/IK
   numbers: call ArmView3D.update({...}) from render().

   Coordinate mapping (kinematics -> three.js, Y-up):
     reach r  -> +X     height z -> +Y     kinematic y -> -Z
     theta1 (base) = rotation about the vertical (Y) axis
   ============================================================ */
(function () {
  'use strict';

  const D2R = Math.PI / 180;
  const host = document.getElementById('view3d');
  if (!host) { window.ArmView3D = { update() {} }; return; }

  // ---- graceful fallbacks -----------------------------------------
  function fail(msg) {
    host.innerHTML = '<div class="viz3d-error">' + msg + '</div>';
    window.ArmView3D = { update() {}, setView() {}, resize() {} };
  }
  if (typeof THREE === 'undefined') {
    return fail('3D view needs three.js (loaded from a CDN). Check your internet connection and reload.');
  }
  let renderer;
  try { renderer = new THREE.WebGLRenderer({ antialias: true }); }
  catch (e) { return fail('WebGL is not available in this browser, so the 3D view cannot start.'); }

  // ---- joint colours (legend uses the same ones) ------------------
  const JOINTS = [
    { id: 'θ1', name: 'base',     hex: 0x3b82f6 },
    { id: 'θ2', name: 'shoulder', hex: 0x06b6d4 },
    { id: 'θ3', name: 'elbow',    hex: 0x22c55e },
    { id: 'θ4', name: 'wrist',    hex: 0xeab308 },
    { id: 'θ5', name: 'roll',     hex: 0xa855f7 }
  ];
  const hexStr = h => '#' + h.toString(16).padStart(6, '0');
  const legend = document.getElementById('view3dLegend');
  if (legend) {
    legend.innerHTML = JOINTS.map(j =>
      `<span><i class="legend-dot" style="background:${hexStr(j.hex)}"></i>${j.id} ${j.name}</span>`).join('') +
      `<span><i class="legend-dot" style="background:var(--canvas-effector)"></i>gripper</span>`;
  }

  // ---- renderer / scene / camera ----------------------------------
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  host.appendChild(renderer.domElement);
  const dom = renderer.domElement;

  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(38, 1, 1, 6000);

  scene.add(new THREE.HemisphereLight(0xffffff, 0x555566, 0.8));
  const sun = new THREE.DirectionalLight(0xffffff, 0.7);
  sun.position.set(300, 700, 400);
  scene.add(sun);
  const fill = new THREE.DirectionalLight(0xffffff, 0.25);
  fill.position.set(-400, 200, -300);
  scene.add(fill);

  // ---- helpers ----------------------------------------------------
  const unitCyl = new THREE.CylinderGeometry(1, 1, 1, 28);
  const unitBox = new THREE.BoxGeometry(1, 1, 1);
  const unitSphere = new THREE.SphereGeometry(1, 20, 14);
  const mat = (hex, o) => new THREE.MeshStandardMaterial(
    Object.assign({ color: hex, metalness: 0.25, roughness: 0.55 }, o || {}));
  function mesh(geo, material, parent) {
    const m = new THREE.Mesh(geo, material);
    (parent || scene).add(m);
    return m;
  }
  // place a unit cylinder (axis = Y) between two points in the arm plane
  function setSeg(m, a, b, radius) {
    const dx = b.x - a.x, dy = b.y - a.y;
    const len = Math.hypot(dx, dy) || 0.001;
    m.position.set((a.x + b.x) / 2, (a.y + b.y) / 2, 0);
    m.rotation.set(0, 0, Math.atan2(dy, dx) - Math.PI / 2);
    m.scale.set(radius, len, radius);
  }

  // ---- materials (theme-dependent ones are recoloured in applyTheme)
  const mLink  = mat(0xa1a1aa);
  const mPed   = mat(0x71717a);
  const mGrip  = mat(0xf97316);
  const mBase  = mat(JOINTS[0].hex);
  const mShl   = mat(JOINTS[1].hex);
  const mElb   = mat(JOINTS[2].hex);
  const mWri   = mat(JOINTS[3].hex);
  const mRoll  = mat(JOINTS[4].hex);
  const mNub   = mat(0xffffff);
  const mTip   = mat(0xf97316, { emissive: 0xf97316, emissiveIntensity: 0.5 });
  const mDrop  = mat(0xf97316, { transparent: true, opacity: 0.4 });
  const mRing  = new THREE.MeshBasicMaterial({ color: 0x3f3f46, transparent: true, opacity: 0.6, side: THREE.DoubleSide });
  const mTarget = new THREE.MeshBasicMaterial({ color: 0xf97316, wireframe: true });

  // ---- static parts -----------------------------------------------
  const pedestal = mesh(unitCyl, mPed);
  const reachRing = mesh(new THREE.RingGeometry(0.994, 1, 120), mRing);
  reachRing.rotation.x = -Math.PI / 2;
  reachRing.position.y = 0.5;
  let grid = null;

  const targetMarker = mesh(unitSphere, mTarget);
  targetMarker.scale.setScalar(10);
  targetMarker.visible = false;

  // ---- arm (everything inside `yaw` turns with theta1) -------------
  const yaw = new THREE.Group();
  scene.add(yaw);

  const turntable = mesh(unitCyl, mBase, yaw);
  const nub = mesh(unitBox, mNub, yaw);          // pointer so base rotation is visible
  nub.scale.set(34, 6, 10);

  const links = [
    mesh(unitCyl, mLink, yaw),   // shoulder bracket (offset)
    mesh(unitCyl, mLink, yaw),   // upper arm
    mesh(unitCyl, mLink, yaw),   // forearm
    mesh(unitCyl, mLink, yaw)    // wrist -> gripper
  ];
  const LINK_R = [9, 12, 10, 7];

  // hinge hubs (cylinder axis = Z, i.e. the pitch axis of each joint)
  function hub(material) {
    const m = mesh(unitCyl, material, yaw);
    m.rotation.x = Math.PI / 2;
    m.scale.set(15, 34, 15);
    return m;
  }
  const hubs = [hub(mShl), hub(mElb), hub(mWri)];

  // gripper
  const gripper = new THREE.Group();
  yaw.add(gripper);
  const collar = mesh(unitCyl, mRoll, gripper);   // roll joint marker
  collar.rotation.z = Math.PI / 2;
  collar.scale.set(15, 12, 15);
  collar.position.x = -4;
  const rollG = new THREE.Group();
  gripper.add(rollG);
  const palm = mesh(unitBox, mLink, rollG);
  palm.scale.set(14, 28, 40);
  palm.position.x = 7;
  const FINGER_LEN = 46;
  const fingerA = mesh(unitBox, mGrip, rollG);
  const fingerB = mesh(unitBox, mGrip, rollG);
  fingerA.scale.set(FINGER_LEN, 10, 6);
  fingerB.scale.set(FINGER_LEN, 10, 6);

  const tip = mesh(unitSphere, mTip, yaw);
  tip.scale.setScalar(5);
  const drop = mesh(unitCyl, mDrop, yaw);          // line from tip down to floor
  const dropDot = mesh(unitCyl, mDrop, yaw);       // spot on the floor under the tip
  dropDot.scale.set(7, 1, 7);

  // ---- theme ------------------------------------------------------
  const cssVar = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  let themeSig = '';
  function applyTheme() {
    const bg   = cssVar('--canvas-bg') || '#111114';
    const g1   = cssVar('--border-strong') || '#3f3f46';
    const g2   = cssVar('--border') || '#27272a';
    const link = cssVar('--text-secondary') || '#a1a1aa';
    const eff  = cssVar('--canvas-effector') || '#f97316';
    const sig = [bg, g1, g2, link, eff].join('|');
    if (sig === themeSig) return;
    themeSig = sig;

    scene.background = new THREE.Color(bg);
    mLink.color.set(link);
    mPed.color.set(g1);
    mGrip.color.set(eff);
    mTip.color.set(eff); mTip.emissive.set(eff);
    mDrop.color.set(eff);
    mTarget.color.set(eff);
    mRing.color.set(g1);

    if (grid) { scene.remove(grid); grid.geometry.dispose(); grid.material.dispose(); }
    grid = new THREE.GridHelper(800, 16, g1, g2);
    scene.add(grid);
  }

  // ---- camera / orbit ---------------------------------------------
  const view = { az: 0.75, el: 0.45, dist: 880 };
  const target = new THREE.Vector3(90, 160, 0);
  const PRESETS = {
    Iso:   { az: 0.75,           el: 0.45 },
    Front: { az: 0,              el: 0.08 },
    Side:  { az: Math.PI / 2,    el: 0.02 },
    Top:   { az: Math.PI / 2,    el: 1.5 }
  };
  function placeCamera() {
    const ce = Math.cos(view.el), se = Math.sin(view.el);
    camera.position.set(
      target.x + view.dist * ce * Math.cos(view.az),
      target.y + view.dist * se,
      target.z + view.dist * ce * Math.sin(view.az));
    camera.lookAt(target);
  }
  let queued = false;
  function requestDraw() {
    if (queued) return;
    queued = true;
    requestAnimationFrame(() => { queued = false; placeCamera(); renderer.render(scene, camera); });
  }
  function setView(name) {
    const p = PRESETS[name]; if (!p) return;
    view.az = p.az; view.el = p.el; view.dist = 880;
    target.set(90, 160, 0);
    requestDraw();
  }

  let drag = null;
  dom.addEventListener('pointerdown', e => {
    drag = { x: e.clientX, y: e.clientY, pan: e.button === 2 || e.shiftKey };
    dom.setPointerCapture(e.pointerId);
    host.classList.add('grabbing');
  });
  dom.addEventListener('pointermove', e => {
    if (!drag) return;
    const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
    drag.x = e.clientX; drag.y = e.clientY;
    if (drag.pan) {
      camera.updateMatrix();
      const right = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 0);
      const up = new THREE.Vector3().setFromMatrixColumn(camera.matrix, 1);
      const k = view.dist * 0.0012;
      target.addScaledVector(right, -dx * k).addScaledVector(up, dy * k);
    } else {
      view.az += dx * 0.008;
      view.el = Math.max(0.01, Math.min(1.5, view.el + dy * 0.008));
    }
    requestDraw();
  });
  const endDrag = () => { drag = null; host.classList.remove('grabbing'); };
  dom.addEventListener('pointerup', endDrag);
  dom.addEventListener('pointercancel', endDrag);
  dom.addEventListener('wheel', e => {
    e.preventDefault();
    view.dist = Math.max(250, Math.min(2200, view.dist * Math.exp(e.deltaY * 0.001)));
    requestDraw();
  }, { passive: false });
  dom.addEventListener('contextmenu', e => e.preventDefault());
  dom.addEventListener('dblclick', () => setView('Iso'));

  // view-preset buttons
  const bar = document.createElement('div');
  bar.className = 'viz3d-toolbar';
  Object.keys(PRESETS).forEach(name => {
    const b = document.createElement('button');
    b.type = 'button';
    b.textContent = name;
    b.addEventListener('click', () => setView(name));
    bar.appendChild(b);
  });
  host.appendChild(bar);
  const hint = document.createElement('div');
  hint.className = 'viz3d-hint';
  hint.textContent = 'drag: rotate · right-drag/shift: pan · scroll: zoom · double-click: reset';
  host.appendChild(hint);

  // ---- resize -----------------------------------------------------
  function resize() {
    const w = host.clientWidth, h = host.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    requestDraw();
  }
  if (typeof ResizeObserver !== 'undefined') new ResizeObserver(resize).observe(host);
  window.addEventListener('resize', resize);

  // ---- pose update (called from render() in arm_console.js) -------
  /*
    s = {
      joints: [{r,z} x5]   from fk()/ik():  base pivot, shoulder, elbow, wrist, tip
      t1, t5:  degrees      base yaw, gripper roll
      grip:    0..180       0 = closed, 180 = open (same as firmware G n)
      cfg:     link lengths {L0,A,B,L1,L2,L34}
      target:  {x,y,z}|null IK target (mm) or null in FK mode
    }
  */
  function update(s) {
    applyTheme();
    const c = s.cfg, J = s.joints.map(p => ({ x: p.r, y: p.z }));

    // static geometry that depends on link lengths
    const pedH = Math.max(1, c.L0 - 6);
    pedestal.scale.set(38, pedH, 38);
    pedestal.position.set(0, pedH / 2, 0);
    const maxReach = c.B + c.L1 + c.L2 + c.L34;
    reachRing.scale.set(maxReach, maxReach, 1);

    // base rotation
    yaw.rotation.y = s.t1 * D2R;
    turntable.scale.set(46, 12, 46);
    turntable.position.set(0, c.L0, 0);
    nub.position.set(30, c.L0 + 9, 0);

    // gripper placement: last GL mm of the wrist link
    const wrist = J[3], tipP = J[4];
    const ang = Math.atan2(tipP.y - wrist.y, tipP.x - wrist.x);
    const wlen = Math.hypot(tipP.x - wrist.x, tipP.y - wrist.y);
    const GL = Math.min(60, wlen * 0.6);
    const gOrigin = { x: tipP.x - Math.cos(ang) * GL, y: tipP.y - Math.sin(ang) * GL };

    setSeg(links[0], J[0], J[1], LINK_R[0]);
    setSeg(links[1], J[1], J[2], LINK_R[1]);
    setSeg(links[2], J[2], J[3], LINK_R[2]);
    setSeg(links[3], J[3], gOrigin, LINK_R[3]);

    hubs[0].position.set(J[1].x, J[1].y, 0);
    hubs[1].position.set(J[2].x, J[2].y, 0);
    hubs[2].position.set(J[3].x, J[3].y, 0);

    gripper.position.set(gOrigin.x, gOrigin.y, 0);
    gripper.rotation.set(0, 0, ang);
    rollG.rotation.x = s.t5 * D2R;

    // fingers: 0 = touching, 180 = fully open (56 mm gap)
    const gap = Math.max(0, Math.min(180, s.grip)) / 180 * 56;
    const fx = 14 + FINGER_LEN / 2;
    fingerA.position.set(fx, 0,  gap / 2 + 3);
    fingerB.position.set(fx, 0, -gap / 2 - 3);

    // tip marker + vertical drop line to the floor
    tip.position.set(tipP.x, tipP.y, 0);
    const h = Math.max(0.001, Math.abs(tipP.y));
    drop.scale.set(1.2, h, 1.2);
    drop.position.set(tipP.x, tipP.y / 2, 0);
    dropDot.position.set(tipP.x, 0.6, 0);

    // IK target (world frame: kin (x,y,z) -> three (x, z, -y))
    if (s.target) {
      targetMarker.visible = true;
      targetMarker.position.set(s.target.x, s.target.z, -s.target.y);
    } else {
      targetMarker.visible = false;
    }

    requestDraw();
  }

  window.ArmView3D = { update, setView, resize };
  resize();
})();
