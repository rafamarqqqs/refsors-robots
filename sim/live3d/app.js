// Live 3D view of the harbor simulation. Frames come from relay.py as
// Server-Sent Events; the page has no channel back to ROS.
import * as THREE from 'three';
import { OrbitControls } from './vendor/three/OrbitControls.js';

// Simplified robot shapes in each Gazebo model frame (base_link), in meters.
// Chassis sizes follow the Husky A200 and Jackal specs. Wheel positions, laser
// mounts and the Husky's upside-down laser were measured with tf in the
// running simulation.
const ROBOTS = {
  larger_robot: {
    model: 'husky', name: 'Husky', color: '#e3a52b', key: '1',
    body: { size: [0.99, 0.67, 0.26], z: 0.11 },
    wheel: { r: 0.165, width: 0.11, x: 0.256, y: 0.285, z: 0.033 },
    laser: [0.337, 0, 0.308],
    laserUpsideDown: true,
  },
  smaller_robot: {
    model: 'jackal', name: 'Jackal', color: '#3cb4c9', key: '2',
    body: { size: [0.42, 0.31, 0.16], z: 0.08 },
    wheel: { r: 0.098, width: 0.04, x: 0.131, y: 0.187, z: 0.035 },
    laser: [0.12, 0, 0.333],
    laserUpsideDown: false,
  },
};
const TARGET_MODEL = 'bomb';
const LEVELS = { 4: 'warn', 8: 'error', 16: 'error' };
const MAX_HITS = 50000;
const MAX_TRAIL = 20000;
const MAX_LOG_LINES = 150;
const STALE_AFTER = 3; // seconds without new data before the view stops showing "Live"
const HOME = { position: new THREE.Vector3(-26, -34, 30), target: new THREE.Vector3(0, 0, 0) };

const $ = (id) => document.getElementById(id);
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// ------------------------------------------------------------------ scene

const view = $('view');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
view.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 400);
camera.up.set(0, 0, 1); // Gazebo is Z-up; set before OrbitControls reads it.
camera.position.copy(HOME.position);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.copy(HOME.target);
controls.enableDamping = true;
controls.screenSpacePanning = false;
controls.maxPolarAngle = Math.PI * 0.495;
controls.minDistance = 1.5;
controls.maxDistance = 160;
controls.listenToKeyEvents(window);

scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 1.6));
const sun = new THREE.DirectionalLight(0xffffff, 1.8);
// Direction of the "sun" light in world/harbor.world.
sun.position.set(0.5, -0.1, 0.9).multiplyScalar(30);
scene.add(sun);

const mat = {
  ground: new THREE.MeshStandardMaterial({ roughness: 1 }),
  container: new THREE.MeshStandardMaterial({ roughness: 0.85 }),
  edge: new THREE.LineBasicMaterial(),
  grid: new THREE.LineBasicMaterial({ transparent: true, opacity: 0.6 }),
  target: new THREE.MeshStandardMaterial({ color: 0xe5484d, roughness: 0.5 }),
};

function applyTheme() {
  scene.background = new THREE.Color(css('--scene'));
  mat.ground.color.set(css('--ground'));
  mat.container.color.set(css('--container'));
  mat.edge.color.set(css('--edge'));
  mat.grid.color.set(css('--grid'));
}
applyTheme();
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);

function grid(half) {
  const points = [];
  for (let i = -half; i <= half; i++) {
    points.push(i, -half, 0, i, half, 0, -half, i, 0, half, i, 0);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
  const lines = new THREE.LineSegments(geometry, mat.grid);
  lines.position.z = 0.003;
  return lines;
}
scene.add(grid(20));

const layers = {};
for (const name of ['scans', 'hits', 'trails', 'goals']) {
  layers[name] = new THREE.Group();
  scene.add(layers[name]);
}

// ------------------------------------------------------------------ world from SDF

const kids = (el, tag) => [...el.children].filter((c) => c.tagName === tag);
const kid = (el, tag) => kids(el, tag)[0];
const nums = (text) => (text || '').trim().split(/\s+/).filter(Boolean).map(Number);

function setPose(obj, text) {
  const [x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0] = nums(text);
  obj.position.set(x, y, z);
  obj.rotation.set(roll, pitch, yaw, 'ZYX'); // SDF roll-pitch-yaw about fixed axes
}

function visualObject(visual) {
  const geom = kid(visual, 'geometry');
  const material = visual.querySelector('material > script > name')?.textContent.trim();
  const box = kid(geom, 'box');
  const cylinder = kid(geom, 'cylinder');
  const plane = kid(geom, 'plane');
  let geometry;
  if (box) {
    geometry = new THREE.BoxGeometry(...nums(kid(box, 'size').textContent));
  } else if (cylinder) {
    const radius = Number(kid(cylinder, 'radius').textContent);
    geometry = new THREE.CylinderGeometry(radius, radius, Number(kid(cylinder, 'length').textContent), 32);
    geometry.rotateX(Math.PI / 2); // SDF cylinders run along Z
  } else if (plane) {
    geometry = new THREE.PlaneGeometry(...nums(kid(plane, 'size').textContent));
  } else {
    return null;
  }
  const surface = { 'Gazebo/White': mat.ground, 'Gazebo/Red': mat.target }[material] || mat.container;
  const obj = new THREE.Group();
  setPose(obj, kid(visual, 'pose')?.textContent);
  obj.add(new THREE.Mesh(geometry, surface));
  if (box) obj.add(new THREE.LineSegments(new THREE.EdgesGeometry(geometry), mat.edge));
  return obj;
}

// Every model Gazebo reports, by name: the drawn object and its latest pose.
const models = new Map();

function addModel(name, obj) {
  scene.add(obj);
  const entry = { obj, p: obj.position.clone(), q: obj.quaternion.clone(), placed: false };
  models.set(name, entry);
  return entry;
}

async function loadWorld() {
  const response = await fetch('world.sdf');
  const xml = new DOMParser().parseFromString(await response.text(), 'application/xml');
  const world = xml.querySelector('world');
  const statePoses = {};
  for (const state of kids(world, 'state')) {
    const [secs = 0, nsecs = 0] = nums(kid(state, 'sim_time')?.textContent);
    simStart = secs + nsecs / 1e9;
    for (const model of kids(state, 'model')) statePoses[model.getAttribute('name')] = kid(model, 'pose')?.textContent;
  }
  for (const model of kids(world, 'model')) {
    const name = model.getAttribute('name');
    const obj = new THREE.Group();
    // Gazebo starts from the poses saved in <state>; live poses replace them.
    setPose(obj, statePoses[name] ?? kid(model, 'pose')?.textContent);
    for (const link of kids(model, 'link')) {
      const linkObj = new THREE.Group();
      setPose(linkObj, kid(link, 'pose')?.textContent);
      for (const visual of kids(link, 'visual')) {
        const v = visualObject(visual);
        if (v) linkObj.add(v);
      }
      obj.add(linkObj);
    }
    addModel(name, obj).placed = true;
  }
}

// ------------------------------------------------------------------ robots

function robotObject(spec) {
  const obj = new THREE.Group();
  const paint = new THREE.MeshStandardMaterial({ color: spec.color, roughness: 0.55 });
  const rubber = new THREE.MeshStandardMaterial({ color: 0x202226, roughness: 0.9 });
  const body = new THREE.Mesh(new THREE.BoxGeometry(...spec.body.size), paint);
  body.position.z = spec.body.z;
  obj.add(body);
  const { r, width, x, y, z } = spec.wheel;
  const wheelGeometry = new THREE.CylinderGeometry(r, r, width, 24); // axis along Y, the axle
  for (const sx of [-1, 1]) {
    for (const sy of [-1, 1]) {
      const wheel = new THREE.Mesh(wheelGeometry, rubber);
      wheel.position.set(sx * x, sy * y, z);
      obj.add(wheel);
    }
  }
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.08, 0.22, 16), rubber);
  nose.rotation.z = -Math.PI / 2; // point along +X, the robot's heading
  nose.position.set(spec.body.size[0] / 2 - 0.12, 0, spec.body.z + spec.body.size[2] / 2 + 0.06);
  obj.add(nose);
  const laser = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.08, 16).rotateX(Math.PI / 2), rubber);
  laser.position.set(...spec.laser);
  obj.add(laser);
  obj.visible = false;
  return obj;
}

function points(color, size, max, opacity = 1) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(max * 3), 3));
  geometry.setDrawRange(0, 0);
  const obj = new THREE.Points(geometry, new THREE.PointsMaterial({
    color, size, transparent: opacity < 1, opacity, depthWrite: opacity === 1,
  }));
  obj.frustumCulled = false; // the bounding sphere would stay at the empty buffer's origin
  return obj;
}

function line(color, max, dashed = false) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(max * 3), 3));
  geometry.setDrawRange(0, 0);
  const material = dashed
    ? new THREE.LineDashedMaterial({ color, dashSize: 0.5, gapSize: 0.3 })
    : new THREE.LineBasicMaterial({ color });
  const obj = new THREE.Line(geometry, material);
  obj.frustumCulled = false; // as for points()
  return obj;
}

function goalMarker(color) {
  const material = new THREE.MeshBasicMaterial({ color, transparent: true, side: THREE.DoubleSide });
  const marker = new THREE.Group();
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.35, 0.55, 40), material);
  ring.position.z = 0.02;
  const pole = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, 1.6, 8).rotateX(Math.PI / 2), material);
  pole.position.z = 0.8;
  marker.add(ring, pole);
  marker.visible = false;
  return { marker, material };
}

const robots = {};
for (const [id, spec] of Object.entries(ROBOTS)) {
  const entry = addModel(spec.model, robotObject(spec));
  const goal = goalMarker(spec.color);
  const robot = {
    id, spec, entry,
    scan: points(spec.color, 0.16, 2048),
    hits: points(spec.color, 0.1, MAX_HITS, 0.55),
    hitKeys: new Set(),
    trail: line(spec.color, MAX_TRAIL),
    goal: null,
    goalMarker: goal.marker,
    goalMaterial: goal.material,
    goalLine: line(spec.color, 2, true),
    path: line(spec.color, 4096),
    pathPoints: points(spec.color, 0.25, 4096),
    samples: [],
  };
  layers.scans.add(robot.scan);
  layers.hits.add(robot.hits);
  layers.trails.add(robot.trail);
  layers.goals.add(robot.goalMarker, robot.goalLine, robot.path, robot.pathPoints);
  robots[id] = robot;
}

// ------------------------------------------------------------------ live data

const tmp = new THREE.Vector3();
const tmpMatrix = new THREE.Matrix4();
const unit = new THREE.Vector3(1, 1, 1);

function setModelPose(name, pose) {
  const entry = models.get(name);
  if (!entry) return;
  entry.p.set(pose[0], pose[1], pose[2]);
  entry.q.set(pose[3], pose[4], pose[5], pose[6]);
  if (!entry.placed || !entry.obj.visible) {
    entry.obj.position.copy(entry.p);
    entry.obj.quaternion.copy(entry.q);
    entry.placed = true;
    entry.obj.visible = true;
  }
}

function writePoint(attribute, index, v) {
  attribute.array[index * 3] = v.x;
  attribute.array[index * 3 + 1] = v.y;
  attribute.array[index * 3 + 2] = v.z;
}

function onScan(robot, scan) {
  if (!robot.entry.placed) return;
  // Uses the newest reported pose, not the smoothed one drawn on screen.
  tmpMatrix.compose(robot.entry.p, robot.entry.q, unit);
  const [lx, ly, lz] = robot.spec.laser;
  const side = robot.spec.laserUpsideDown ? -1 : 1; // a laser rolled 180° sweeps mirrored
  const scanPos = robot.scan.geometry.attributes.position;
  const hitPos = robot.hits.geometry.attributes.position;
  let n = 0;
  let hits = robot.hits.geometry.drawRange.count;
  for (let i = 0; i < scan.ranges.length && n < 2048; i++) {
    const range = scan.ranges[i];
    if (!(range > scan.range_min && range < scan.range_max)) continue;
    const angle = scan.angle_min + i * scan.angle_increment;
    tmp.set(lx + range * Math.cos(angle), ly + side * range * Math.sin(angle), lz).applyMatrix4(tmpMatrix);
    writePoint(scanPos, n++, tmp);
    const key = Math.round(tmp.x * 10) * 10000 + Math.round(tmp.y * 10);
    if (hits < MAX_HITS && !robot.hitKeys.has(key)) {
      robot.hitKeys.add(key);
      writePoint(hitPos, hits++, tmp);
    }
  }
  robot.scan.geometry.setDrawRange(0, n);
  scanPos.needsUpdate = true;
  robot.hits.geometry.setDrawRange(0, hits);
  hitPos.needsUpdate = true;
}

function extendTrail(robot, t) {
  const pos = robot.trail.geometry.attributes.position;
  const count = robot.trail.geometry.drawRange.count;
  const p = robot.entry.p;
  if (count > 0) {
    const last = tmp.fromArray(pos.array, (count - 1) * 3);
    if (Math.hypot(last.x - p.x, last.y - p.y) < 0.15) return;
  }
  if (count >= MAX_TRAIL) return;
  writePoint(pos, count, tmp.set(p.x, p.y, 0.04));
  robot.trail.geometry.setDrawRange(0, count + 1);
  pos.needsUpdate = true;
  robot.samples.push([t, p.x, p.y]);
}

function setPath(robot, path) {
  const pos = robot.path.geometry.attributes.position;
  const dots = robot.pathPoints.geometry.attributes.position;
  const n = Math.min(path.x.length, 4096);
  for (let i = 0; i < n; i++) {
    tmp.set(path.x[i], path.y[i], 0.08);
    writePoint(pos, i, tmp);
    writePoint(dots, i, tmp);
  }
  robot.path.geometry.setDrawRange(0, n);
  robot.pathPoints.geometry.setDrawRange(0, n);
  pos.needsUpdate = dots.needsUpdate = true;
}

function setGoal(robot, x, y) {
  robot.goal = { x, y, status: 'active' };
  robot.goalMarker.position.set(x, y, 0);
  robot.goalMarker.visible = true;
  robot.goalMaterial.opacity = 1;
}

function endGoal(robot, status) {
  if (!robot.goal) return;
  robot.goal.status = status;
  robot.goalMaterial.opacity = 0.35;
}

function clearTraces() {
  for (const robot of Object.values(robots)) {
    robot.hitKeys.clear();
    robot.hits.geometry.setDrawRange(0, 0);
    robot.trail.geometry.setDrawRange(0, 0);
    robot.path.geometry.setDrawRange(0, 0);
    robot.pathPoints.geometry.setDrawRange(0, 0);
    robot.samples = [];
  }
}

// ------------------------------------------------------------------ log and status

let lastLogId = 0;
const logList = $('log');

// Sets the mission line and the robots' goals from a log entry.
function applyState(entry) {
  const nav = /^\/(larger_robot|smaller_robot)\/Navigation$/.exec(entry.node);
  if (nav) {
    const robot = robots[nav[1]];
    const goal = /Driving to (-?[\d.]+) (-?[\d.]+)/.exec(entry.msg);
    if (goal) setGoal(robot, Number(goal[1]), Number(goal[2]));
    else if (entry.msg.startsWith('Reached destination')) endGoal(robot, 'reached');
    else if (entry.msg.startsWith("Time's up")) endGoal(robot, 'timed out');
  }
  if (entry.node === '/Application') $('mission').textContent = entry.msg;
}

function onLog(entry) {
  if (entry.id <= lastLogId) return;
  lastLogId = entry.id;
  applyState(entry);
  const li = document.createElement('li');
  const meta = document.createElement('span');
  meta.textContent = `${formatSimTime(Math.max(entry.t - simStart, 0))} ${entry.node.slice(1)}`;
  li.append(meta, entry.msg);
  if (LEVELS[entry.level]) li.dataset.level = LEVELS[entry.level];
  logList.prepend(li);
  while (logList.children.length > MAX_LOG_LINES) logList.lastChild.remove();
}

const robotCards = {};
for (const robot of Object.values(robots)) {
  const card = document.createElement('article');
  card.className = 'robot';
  card.style.setProperty('--swatch', robot.spec.color);
  card.innerHTML = `<h2><span></span><small></small></h2>
    <dl><dt>Position</dt><dd data-f="pos">–</dd><dt>Heading</dt><dd data-f="yaw">–</dd>
    <dt>Speed</dt><dd data-f="speed">–</dd><dt>Goal</dt><dd data-f="goal">–</dd></dl>`;
  card.querySelector('h2 span').textContent = robot.spec.name;
  card.querySelector('h2 small').textContent = robot.id;
  $('robots').append(card);
  robotCards[robot.id] = card;
}

const yawOf = (q) => Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
const fmt = (v) => v.toFixed(1);

function formatSimTime(t) {
  const m = Math.floor(t / 60);
  return `${m}:${(t - m * 60).toFixed(1).padStart(4, '0')}`;
}

let simT = null;
let simStart = 0; // <sim_time> saved in the world file, where Gazebo's clock starts
let rtfSamples = [];

function updatePanel() {
  for (const robot of Object.values(robots)) {
    const card = robotCards[robot.id];
    const field = (f) => card.querySelector(`[data-f="${f}"]`);
    if (!robot.entry.placed) continue;
    const { p, q } = robot.entry;
    field('pos').textContent = `x ${fmt(p.x)}, y ${fmt(p.y)} m`;
    field('yaw').textContent = `${Math.round((yawOf(q) * 180) / Math.PI)}°`;
    const recent = robot.samples.filter((s) => simT - s[0] <= 3);
    robot.samples = recent;
    let speed = 0;
    if (recent.length > 1 && simT > recent[0][0]) {
      speed = Math.hypot(p.x - recent[0][1], p.y - recent[0][2]) / (simT - recent[0][0]);
    }
    field('speed').textContent = `${speed.toFixed(2)} m/s`;
    const g = robot.goal;
    field('goal').textContent = g ? `x ${fmt(g.x)}, y ${fmt(g.y)}${g.status === 'active' ? '' : ` (${g.status})`}` : '–';
  }
  if (simT != null) $('simtime').textContent = formatSimTime(Math.max(simT - simStart, 0));
  if (rtfSamples.length > 1) {
    const [a, b] = [rtfSamples[0], rtfSamples[rtfSamples.length - 1]];
    if (b[0] > a[0]) $('rtf').textContent = `${((b[1] - a[1]) / (b[0] - a[0])).toFixed(2)}×`;
  }
}

function setConn(state, text) {
  const el = $('conn');
  el.dataset.state = state;
  if (el.textContent !== text) el.textContent = text;
}

let lastFrameAt = 0;
let lastFrame = null;

function status(frame) {
  if (!frame.ros) return setConn('wait', 'Connected. Waiting for the ROS master…');
  if (!Object.keys(frame.models).length) return setConn('wait', 'Waiting for Gazebo…');
  if (frame.age > STALE_AFTER) return setConn('wait', `No Gazebo data for ${Math.round(frame.age)} s`);
  const missing = Object.values(ROBOTS).filter((s) => !(s.model in frame.models)).map((s) => s.name);
  if (missing.length) return setConn('wait', `Waiting for ${missing.join(' and ')} to spawn…`);
  setConn('live', 'Live');
}

function onFrame(frame) {
  lastFrameAt = performance.now();
  lastFrame = frame;
  if (frame.snapshot) {
    // A simulation restart starts the clock again; drop what the old run left.
    if (simT != null && frame.t != null && frame.t + 1 < simT) clearTraces();
    lastLogId = 0;
    logList.replaceChildren();
    for (const robot of Object.values(robots)) {
      robot.goal = null;
      robot.goalMarker.visible = false;
    }
    $('mission').textContent = '';
    // The relay keeps the latest mission line and goal events beyond its log window.
    for (const entry of frame.state || []) applyState(entry);
  }
  if (frame.t != null) {
    simT = frame.t;
    rtfSamples.push([frame.wall, frame.t]);
    rtfSamples = rtfSamples.filter((s) => frame.wall - s[0] <= 5);
  }
  for (const [name, pose] of Object.entries(frame.models)) setModelPose(name, pose);
  for (const robot of Object.values(robots)) {
    if (frame.models[robot.spec.model] && simT != null) extendTrail(robot, simT);
  }
  for (const [id, scan] of Object.entries(frame.scans || {})) if (robots[id]) onScan(robots[id], scan);
  for (const [id, path] of Object.entries(frame.paths || {})) if (robots[id]) setPath(robots[id], path);
  for (const entry of frame.logs || []) onLog(entry);
  status(frame);
}

let source = null;
function connect() {
  source = new EventSource('events');
  source.onmessage = (event) => onFrame(JSON.parse(event.data));
  source.onerror = () => setConn('down', 'Disconnected. Retrying…');
}

setInterval(() => {
  updatePanel();
  const silent = (performance.now() - lastFrameAt) / 1000;
  if (lastFrame && lastFrame.ros && source?.readyState === EventSource.OPEN && silent > STALE_AFTER) {
    setConn('wait', `No data for ${Math.round(silent)} s`);
  }
}, 250);

// ------------------------------------------------------------------ labels

const labelBox = $('labels');
const labels = [];
function addLabel(text, sub, color, entry, height) {
  const el = document.createElement('div');
  el.className = 'label';
  el.style.setProperty('--swatch', color);
  el.textContent = text;
  if (sub) {
    const small = document.createElement('small');
    small.textContent = ` ${sub}`;
    el.append(small);
  }
  labelBox.append(el);
  labels.push({ el, entry, height });
}
for (const robot of Object.values(robots)) addLabel(robot.spec.name, robot.id, robot.spec.color, robot.entry, 1.1);

function placeLabels() {
  if (labelBox.hidden) return;
  const { clientWidth: w, clientHeight: h } = view;
  for (const label of labels) {
    if (!label.entry || !label.entry.placed || !label.entry.obj.visible) {
      label.el.style.display = 'none';
      continue;
    }
    tmp.copy(label.entry.obj.position);
    tmp.z += label.height;
    tmp.project(camera);
    const onScreen = tmp.z < 1 && Math.abs(tmp.x) <= 1.1 && Math.abs(tmp.y) <= 1.1;
    label.el.style.display = onScreen ? '' : 'none';
    if (onScreen) label.el.style.translate = `${((tmp.x + 1) / 2) * w}px ${((1 - tmp.y) / 2) * h}px`;
  }
}

// ------------------------------------------------------------------ camera

let follow = null;
const followFrom = new THREE.Vector3();

function setFollow(id) {
  follow = follow === id ? null : id;
  for (const button of document.querySelectorAll('[data-follow]')) {
    button.setAttribute('aria-pressed', String(button.dataset.follow === follow));
  }
  if (!follow) return;
  const obj = robots[follow].entry.obj;
  // Look down at 60° from the current side, high enough to see over most containers.
  const offset = camera.position.clone().sub(controls.target).setZ(0);
  if (offset.lengthSq() < 1e-6) offset.set(0, -1, 0);
  offset.setLength(12 * Math.cos(Math.PI / 3)).setZ(12 * Math.sin(Math.PI / 3));
  controls.target.copy(obj.position);
  camera.position.copy(obj.position).add(offset);
  followFrom.copy(obj.position);
}

function followStep() {
  if (!follow) return;
  const p = robots[follow].entry.obj.position;
  tmp.copy(p).sub(followFrom);
  camera.position.add(tmp);
  controls.target.add(tmp);
  followFrom.copy(p);
}

function topView() {
  if (!follow) controls.target.copy(HOME.target);
  const height = follow ? Math.max(camera.position.distanceTo(controls.target), 8) : 58;
  camera.position.copy(controls.target).add(new THREE.Vector3(0, -0.01, height));
}

function resetView() {
  if (follow) setFollow(follow);
  camera.position.copy(HOME.position);
  controls.target.copy(HOME.target);
}

for (const button of document.querySelectorAll('[data-follow]')) {
  button.addEventListener('click', () => setFollow(button.dataset.follow));
}
$('top').addEventListener('click', topView);
$('reset').addEventListener('click', resetView);

window.addEventListener('keydown', (event) => {
  if (event.target.closest?.('input, textarea, select') || event.metaKey || event.ctrlKey || event.altKey) return;
  const robot = Object.values(robots).find((r) => r.spec.key === event.key);
  if (robot) setFollow(robot.id);
  else if (event.key === 'Escape' && follow) setFollow(follow);
  else if (event.key === 't' || event.key === 'T') topView();
  else if (event.key === 'r' || event.key === 'R') resetView();
});

for (const box of document.querySelectorAll('[data-layer]')) {
  box.addEventListener('change', () => {
    if (box.dataset.layer === 'labels') labelBox.hidden = !box.checked;
    else layers[box.dataset.layer].visible = box.checked;
  });
}

// ------------------------------------------------------------------ loop

function resize() {
  const { clientWidth: w, clientHeight: h } = view;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(view);
resize();

const clock = new THREE.Clock();
renderer.setAnimationLoop(() => {
  // Poses arrive about 15 times a second; ease toward them for smooth motion.
  const k = 1 - Math.exp(-Math.min(clock.getDelta(), 0.1) * 10);
  for (const entry of models.values()) {
    if (!entry.placed) continue;
    entry.obj.position.lerp(entry.p, k);
    entry.obj.quaternion.slerp(entry.q, k);
  }
  for (const robot of Object.values(robots)) {
    const g = robot.goal;
    const active = g && g.status === 'active' && robot.entry.placed;
    robot.goalLine.visible = Boolean(active);
    if (!active) continue;
    const pos = robot.goalLine.geometry.attributes.position;
    const p = robot.entry.obj.position;
    pos.array.set([p.x, p.y, 0.06, g.x, g.y, 0.06]);
    pos.needsUpdate = true;
    robot.goalLine.geometry.setDrawRange(0, 2);
    robot.goalLine.computeLineDistances();
  }
  followStep();
  controls.update();
  renderer.render(scene, camera);
  placeLabels();
});

// The world comes first: it gives the clock's start for the log times.
loadWorld().then(() => {
  const entry = models.get(TARGET_MODEL);
  if (entry) addLabel('Target', 'bomb', '#e5484d', entry, 0.9);
  connect();
});
