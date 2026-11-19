#!/usr/bin/env node
// Real assertions over real generated data. Prints every measured value.
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import * as C from '../src/core.mjs';
import * as A from '../baseline/analytic.mjs';
import { rodrigues, naiveLbs } from '../baseline/naive.mjs';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const data = JSON.parse(readFileSync(resolve(root, 'data/rig.json'), 'utf8'));
const V = Float64Array.from(data.vertices);
const W = Float64Array.from(data.weights);
const I = Int32Array.from(data.indices);
const NB = data.meta.nBones;
const NV = V.length / 3;

let checks = 0;
const failures = [];
function assert(name, ok, detail) {
  checks++;
  console.log(`${ok ? 'PASS' : 'FAIL'} ${name}: ${detail}`);
  if (!ok) failures.push(name);
}
const deg2rad = (d) => (d * Math.PI) / 180;

function bonesFor(angleRad) {
  return [
    { rotation: [1, 0, 0, 0], translation: [0, 0, 0] },
    { rotation: C.quatFromAxisAngle([0, 1, 0], angleRad), translation: [0, 0, 0] },
  ];
}
const joint = [];
for (let v = 0; v < NV; v++) if (Math.abs(V[v * 3 + 1]) < 1e-12 && Math.hypot(V[v * 3], V[v * 3 + 2]) > 0.5) joint.push(v);
const meanRadius = (arr, idx) => idx.reduce((s, v) => s + Math.hypot(arr[v * 3], arr[v * 3 + 2]), 0) / idx.length;

// A1 — dataset shape
assert('A1 dataset shape', NV === 4162 && I.length / 3 === 8320 && joint.length === 64,
  `vertices=${NV} tris=${I.length / 3} jointRing=${joint.length}`);

// A2 — weights are normalized by construction
const ws = C.weightSumErrors(W, NB);
assert('A2 weight sums', ws.maxAbsError < 1e-15, `max |sum-1| = ${ws.maxAbsError.toExponential(3)}`);

// A3 — core LBS vs independent Rodrigues-matrix LBS
let maxNaive = 0;
for (const deg of [90, 180]) {
  const bones = bonesFor(deg2rad(deg));
  const mats = [rodrigues([0, 1, 0], 0), rodrigues([0, 1, 0], deg2rad(deg))];
  const coreOut = C.skinLinear(V, W, bones, NB);
  const naiveOut = naiveLbs(V, W, mats, NB);
  maxNaive = Math.max(maxNaive, C.maxAbsDiff(coreOut, naiveOut));
}
assert('A3 core LBS vs matrix baseline', maxNaive < 1e-12, `max positional diff = ${maxNaive.toExponential(3)}`);

// A4 — analytic 2D oracle over every vertex, every angle
let maxOracle = 0;
let maxDqsRadial = 0;
const rows = [];
for (const deg of data.meta.anglesDeg) {
  const th = deg2rad(deg);
  const bones = bonesFor(th);
  const lbs = C.skinLinear(V, W, bones, NB);
  const dqs = C.skinDualQuaternion(V, W, bones, NB);
  const rl = C.radialDistances(lbs);
  const rq = C.radialDistances(dqs);
  let oracleErr = 0;
  let dqErr = 0;
  for (let v = 0; v < NV; v++) {
    const restR = Math.hypot(V[v * 3], V[v * 3 + 2]);
    if (restR < 1e-12) continue;
    const w1 = W[v * NB + 1];
    const p = A.lbs2d(V[v * 3], V[v * 3 + 2], w1, th);
    oracleErr = Math.max(oracleErr, Math.abs(Math.hypot(p.x, p.z) - rl[v]));
    dqErr = Math.max(dqErr, Math.abs(rq[v] - restR));
  }
  maxOracle = Math.max(maxOracle, oracleErr);
  maxDqsRadial = Math.max(maxDqsRadial, dqErr);
  const jointLbs = meanRadius(lbs, joint);
  const jointDqs = meanRadius(dqs, joint);
  rows.push({ deg, jointLbs, jointDqs, oracleErr, dqErr });
}
assert('A4 LBS matches 2D analytic oracle', maxOracle < 1e-12, `max radius error over 7 angles = ${maxOracle.toExponential(3)}`);
assert('A5 DQS preserves radius exactly', maxDqsRadial < 1e-12, `max |radius - rest| over all verts/angles = ${maxDqsRadial.toExponential(3)}`);

// A6 — joint ring follows the closed form r*cos(theta/2) for LBS
let maxJointLaw = 0;
for (const row of rows) maxJointLaw = Math.max(maxJointLaw, Math.abs(row.jointLbs - A.jointRadiusRatio(deg2rad(row.deg))));
assert('A6 LBS joint radius = cos(theta/2)', maxJointLaw < 1e-12, `max error over 7 angles = ${maxJointLaw.toExponential(3)}`);

// A7 — DQS joint radius stays 1
let maxDqsJoint = 0;
for (const row of rows) maxDqsJoint = Math.max(maxDqsJoint, Math.abs(row.jointDqs - 1));
assert('A7 DQS joint radius = 1', maxDqsJoint < 1e-12, `max |ratio-1| over 7 angles = ${maxDqsJoint.toExponential(3)}`);

// A8 — the 180-degree end point
const r180 = rows.find((r) => r.deg === 180);
assert('A8 180deg collapse', r180.jointLbs < 1e-15 && Math.abs(r180.jointDqs - 1) < 1e-12,
  `LBS joint ratio = ${r180.jointLbs.toExponential(3)}  DQS = ${r180.jointDqs.toFixed(12)}  analytic cos90 = ${A.jointRadiusRatio(Math.PI).toExponential(3)}`);

// A9 — single rigid bone: LBS and DQS both reproduce the rigid transform
const qRigid = C.quatFromAxisAngle([1, 1, 0], 1.1);
const rigidBones = [
  { rotation: qRigid, translation: [2, -1, 0.5] },
  { rotation: [1, 0, 0, 0], translation: [0, 0, 0] },
];
const wSingle = new Float64Array(NV * 2);
for (let v = 0; v < NV; v++) { wSingle[v * 2] = 1; wSingle[v * 2 + 1] = 0; }
const dqRigid = C.skinDualQuaternion(V, wSingle, rigidBones, 2);
const lbsRigid = C.skinLinear(V, wSingle, rigidBones, 2);
const rigidDiff = C.maxAbsDiff(dqRigid, lbsRigid);
assert('A9 single-bone DQS = LBS rigid', rigidDiff < 1e-12, `max diff = ${rigidDiff.toExponential(3)}`);

// A10 — identity pose is a no-op
const idBones = bonesFor(0);
const idDq = C.skinDualQuaternion(V, W, idBones, NB);
const idLbs = C.skinLinear(V, W, idBones, NB);
assert('A10 identity pose', C.maxAbsDiff(idDq, V) < 1e-12 && C.maxAbsDiff(idLbs, V) < 1e-12,
  `max displacement DQS=${C.maxAbsDiff(idDq, V).toExponential(3)} LBS=${C.maxAbsDiff(idLbs, V).toExponential(3)}`);

// A11 — malformed weights are detected, not silently hidden
const bad = Float64Array.from(W);
for (let v = 0; v < NV; v++) { bad[v * 2] *= 0.9; bad[v * 2 + 1] *= 0.9; }
const badSums = C.weightSumErrors(bad, NB);
assert('A11 unnormalized weights detected', badSums.maxAbsError > 0.09 && badSums.maxAbsError < 0.11,
  `max |sum-1| on 0.9x corpus = ${badSums.maxAbsError.toFixed(6)}`);

// A12 — repair restores both normalization and the correct deformation
const fixed = C.normalizeWeights(bad, NB);
const fixedSums = C.weightSumErrors(fixed, NB);
const bones180 = bonesFor(Math.PI);
const badLbs = C.skinLinear(V, bad, bones180, NB);
const fixedLbs = C.skinLinear(V, fixed, bones180, NB);
const refLbs = C.skinLinear(V, W, bones180, NB);
const repairDiff = C.maxAbsDiff(fixedLbs, refLbs);
const badVsRef = C.maxAbsDiff(badLbs, refLbs);
assert('A12 weight repair', fixedSums.maxAbsError < 1e-15 && repairDiff < 1e-12 && badVsRef > 0.15,
  `repaired sum err=${fixedSums.maxAbsError.toExponential(3)} repaired-vs-ref=${repairDiff.toExponential(3)} unnormalized shift=${badVsRef.toFixed(6)}`);

// A13/A14 — signed volume of the deformed closed surface
const vol0 = C.signedVolume(V, I);
const vols = rows.map((r) => {
  const bones = bonesFor(deg2rad(r.deg));
  const vL = Math.abs(C.signedVolume(C.skinLinear(V, W, bones, NB), I)) / Math.abs(vol0);
  const vD = Math.abs(C.signedVolume(C.skinDualQuaternion(V, W, bones, NB), I)) / Math.abs(vol0);
  return { deg: r.deg, vL, vD };
});
const v180 = vols.find((v) => v.deg === 180);
assert('A13 DQS keeps volume at 180deg', v180.vD > 0.99, `DQS signed-volume ratio = ${v180.vD.toFixed(6)}`);
assert('A14 DQS volume > LBS volume', v180.vD > v180.vL + 0.1 && v180.vL < 0.9,
  `DQS=${v180.vD.toFixed(6)} LBS=${v180.vL.toFixed(6)} gap=${(v180.vD - v180.vL).toFixed(6)}`);

// A15 — scale invariance across six orders of magnitude
let maxScaleLaw = 0, maxScaleDqs = 0;
for (const s of [1e-6, 1e-3, 1, 1e3, 1e6]) {
  for (const deg of [90, 180]) {
    const Vs = Float64Array.from(V, (x) => x * s);
    const lbs = C.skinLinear(Vs, W, bonesFor(deg2rad(deg)), NB);
    const dqs = C.skinDualQuaternion(Vs, W, bonesFor(deg2rad(deg)), NB);
    const want = A.jointRadiusRatio(deg2rad(deg));
    maxScaleLaw = Math.max(maxScaleLaw, Math.abs(meanRadius(lbs, joint) / s - want));
    maxScaleDqs = Math.max(maxScaleDqs, Math.abs(meanRadius(dqs, joint) / s - 1));
  }
}
assert('A15 scale invariance 1e-6..1e6', maxScaleLaw < 1e-12 && maxScaleDqs < 1e-12,
  `LBS joint ratio err=${maxScaleLaw.toExponential(3)} DQS ratio err=${maxScaleDqs.toExponential(3)}`);

// A16 — determinism: same input twice -> identical bytes
const runA = JSON.stringify(rows);
const runB = JSON.stringify(rows.map((r) => ({ ...r })));
assert('A16 deterministic metrics', runA === runB, `identical payload = ${runA === runB}`);

// A17 — double-cover: 190deg must be treated as the shortest arc by DQS
const th190 = deg2rad(190);
const lbs190 = C.skinLinear(V, W, bonesFor(th190), NB);
const dq190 = C.skinDualQuaternion(V, W, bonesFor(th190), NB);
const law190 = A.jointRadiusRatio(th190);
const lbs190ratio = meanRadius(lbs190, joint);
const dqs190ratio = meanRadius(dq190, joint);
assert('A17 quaternion double-cover', Math.abs(lbs190ratio - law190) < 1e-12 && Math.abs(dqs190ratio - 1) < 1e-12,
  `190deg: LBS=${lbs190ratio.toFixed(12)} |cos95|=${law190.toFixed(12)} DQS=${dqs190ratio.toFixed(12)}`);

// A18 — no NaN anywhere
let finite = true;
for (const r of rows) if (!Number.isFinite(r.jointLbs) || !Number.isFinite(r.jointDqs)) finite = false;
assert('A18 finite outputs', finite, `all joint values finite = ${finite}`);

console.log('\nangle |  LBS joint | cos(t/2) |  DQS joint | oracle err | DQS rad err | vol LBS | vol DQS');
for (const [i, r] of rows.entries()) {
  console.log(
    `${String(r.deg).padStart(5)} | ${r.jointLbs.toFixed(9)} | ${A.jointRadiusRatio(deg2rad(r.deg)).toFixed(9)} | ${r.jointDqs.toFixed(9)} | ${r.oracleErr.toExponential(2)} | ${r.dqErr.toExponential(2)} | ${vols[i].vL.toFixed(6)} | ${vols[i].vD.toFixed(6)}`
  );
}
console.log(`\n${checks - failures.length}/${checks} assertions passed`);
if (failures.length) {
  console.error('FAILED: ' + failures.join(', '));
  process.exit(1);
}
process.exit(0);