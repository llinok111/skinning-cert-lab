import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import * as C from './src/core.mjs';
import * as A from './baseline/analytic.mjs';

const errBox = document.getElementById('err');
const state = { angleDeg: 180, method: 'lbs', data: null, lbs: null, dqs: null, joint: [], restRadius: null };

function bonesFor(deg) {
  const th = (deg * Math.PI) / 180;
  return [
    { rotation: [1, 0, 0, 0], translation: [0, 0, 0] },
    { rotation: C.quatFromAxisAngle([0, 1, 0], th), translation: [0, 0, 0] },
  ];
}

function radialMean(arr, idx) {
  let s = 0;
  for (const v of idx) s += Math.hypot(arr[v * 3], arr[v * 3 + 2]);
  return s / idx.length;
}

const app = document.getElementById('app');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
app.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0d13);
scene.fog = new THREE.Fog(0x0a0d13, 14, 30);

const camera = new THREE.PerspectiveCamera(42, window.innerWidth / window.innerHeight, 0.1, 100);
camera.position.set(4.6, 2.4, 6.2);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.minDistance = 3;
controls.maxDistance = 18;

scene.add(new THREE.HemisphereLight(0xbcd6ff, 0x1a1f2b, 1.1));
const key = new THREE.DirectionalLight(0xffffff, 2.0);
key.position.set(5, 8, 6);
scene.add(key);
const rim = new THREE.DirectionalLight(0x6ea8ff, 1.2);
rim.position.set(-6, 2, -5);
scene.add(rim);

const grid = new THREE.GridHelper(14, 28, 0x1c2636, 0x141c29);
grid.position.y = -2.6;
scene.add(grid);

const geometry = new THREE.BufferGeometry();
let mesh = null;
let colors = null;
let rest = null;

function buildMesh(data) {
  rest = Float64Array.from(data.vertices);
  geometry.setAttribute('position', new THREE.BufferAttribute(Float32Array.from(rest), 3));
  geometry.setIndex(new THREE.BufferAttribute(Uint32Array.from(data.indices), 1));
  colors = new Float32Array(rest.length);
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
  geometry.computeVertexNormals();

  const material = new THREE.MeshStandardMaterial({
    vertexColors: true, roughness: 0.42, metalness: 0.08, side: THREE.DoubleSide,
  });
  mesh = new THREE.Mesh(geometry, material);
  scene.add(mesh);

  const ringGeo = new THREE.TorusGeometry(1.0, 0.006, 8, 96);
  const ring = new THREE.Mesh(ringGeo, new THREE.MeshBasicMaterial({ color: 0x3f5a7d }));
  ring.rotation.x = Math.PI / 2;
  scene.add(ring);
}

function paint(deformed) {
  const pos = geometry.attributes.position.array;
  const rDef = C.radialDistances(deformed);
  const rRest = state.restRadius;
  for (let v = 0; v < rRest.length; v++) {
    const ratio = rRest[v] > 1e-9 ? Math.min(1, rDef[v] / rRest[v]) : 1;
    const h = 0.6 * ratio; // red at collapse, blue at preserved radius
    const c = new THREE.Color().setHSL(h, 0.82, 0.52 + 0.1 * ratio);
    colors[v * 3] = c.r; colors[v * 3 + 1] = c.g; colors[v * 3 + 2] = c.b;
    pos[v * 3] = deformed[v * 3];
    pos[v * 3 + 1] = deformed[v * 3 + 1];
    pos[v * 3 + 2] = deformed[v * 3 + 2];
  }
  geometry.attributes.position.needsUpdate = true;
  geometry.attributes.color.needsUpdate = true;
  geometry.computeVertexNormals();
  const n = geometry.attributes.normal.array;
  for (let i = 0; i < n.length; i += 3) {
    if (!Number.isFinite(n[i]) || Math.hypot(n[i], n[i + 1], n[i + 2]) < 1e-9) {
      n[i] = 0; n[i + 1] = 1; n[i + 2] = 0;
    }
  }
  geometry.attributes.normal.needsUpdate = true;
  geometry.computeBoundingSphere();
}

function fmt(x, digits = 6) { return Number(x).toFixed(digits); }

function evaluate() {
  const { angleDeg, data, joint } = state;
  const th = (angleDeg * Math.PI) / 180;
  const bones = bonesFor(angleDeg);
  const lbs = C.skinLinear(rest, state.weights, bones, 2);
  const dqs = C.skinDualQuaternion(rest, state.weights, bones, 2);
  state.lbs = lbs; state.dqs = dqs;

  const chosen = state.method === 'lbs' ? lbs : dqs;
  paint(chosen);

  // All displayed numbers come from core / the independent analytic oracle.
  const lbsJoint = radialMean(lbs, joint);
  const dqsJoint = radialMean(dqs, joint);
  const oracle = A.jointRadiusRatio(th);
  const rLbs = C.radialDistances(lbs);
  const rDqs = C.radialDistances(dqs);
  let oracleErr = 0, dqsErr = 0;
  for (let v = 0; v < state.restRadius.length; v++) {
    if (state.restRadius[v] < 1e-9) continue;
    const p = A.lbs2d(rest[v * 3], rest[v * 3 + 2], state.weights[v * 2 + 1], th);
    oracleErr = Math.max(oracleErr, Math.abs(Math.hypot(p.x, p.z) - rLbs[v]));
    dqsErr = Math.max(dqsErr, Math.abs(rDqs[v] - state.restRadius[v]));
  }
  const vol0 = Math.abs(C.signedVolume(rest, state.indices));
  const volL = Math.abs(C.signedVolume(lbs, state.indices)) / vol0;
  const volD = Math.abs(C.signedVolume(dqs, state.indices)) / vol0;

  const set = (id, text, cls) => {
    const el = document.getElementById(id);
    el.textContent = text;
    el.className = cls || '';
  };
  set('s-lbs', fmt(lbsJoint), lbsJoint < 0.9 ? 'bad' : '');
  set('s-oracle', fmt(oracle), 'warn');
  set('s-dqs', fmt(dqsJoint), 'good');
  set('s-oracle-err', oracleErr.toExponential(2));
  set('s-dqs-err', dqsErr.toExponential(2));
  set('s-vol-l', fmt(volL, 5), volL < 0.95 ? 'bad' : '');
  set('s-vol-d', fmt(volD, 5), 'good');
  set('s-w', C.weightSumErrors(state.weights, 2).maxAbsError.toExponential(2), 'good');
  document.getElementById('angle-v').textContent = `${angleDeg}°`;
  document.querySelectorAll('.modes button').forEach((b) => b.classList.remove('on'));
  document.getElementById(state.method === 'lbs' ? 'm-lbs' : 'm-dqs').classList.add('on');
}

document.getElementById('angle').addEventListener('input', (e) => {
  state.angleDeg = Number(e.target.value);
  evaluate();
});
document.getElementById('m-lbs').addEventListener('click', () => { state.method = 'lbs'; evaluate(); });
document.getElementById('m-dqs').addEventListener('click', () => { state.method = 'dqs'; evaluate(); });

function resize() {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
}
window.addEventListener('resize', resize);

fetch('./data/rig.json')
  .then((r) => {
    if (!r.ok) throw new Error(`data/rig.json -> HTTP ${r.status}`);
    return r.json();
  })
  .then((data) => {
    state.data = data;
    state.weights = Float64Array.from(data.weights);
    state.indices = Int32Array.from(data.indices);
    for (let v = 0; v < data.meta.vertexCount; v++) {
      if (Math.abs(data.vertices[v * 3 + 1]) < 1e-9
        && Math.hypot(data.vertices[v * 3], data.vertices[v * 3 + 2]) > 0.5) state.joint.push(v);
    }
    buildMesh(data);
    state.restRadius = C.radialDistances(rest);
    evaluate();
    animate();
  })
  .catch((err) => {
    errBox.textContent = `failed to load data: ${err.message}`;
    console.error(err);
  });

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}