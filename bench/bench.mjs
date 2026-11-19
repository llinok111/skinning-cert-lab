#!/usr/bin/env node
// 1 warmup + 10 measured samples; cold start is the very first call, recorded separately.
import { readFileSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { performance } from 'node:perf_hooks';
import * as C from '../src/core.mjs';
import * as A from '../baseline/analytic.mjs';

const root = resolve(dirname(fileURLToPath(import.meta.url)), '..');
const data = JSON.parse(readFileSync(resolve(root, 'data/rig.json'), 'utf8'));
const V = Float64Array.from(data.vertices);
const W = Float64Array.from(data.weights);
const I = Int32Array.from(data.indices);
const NV = V.length / 3;
const ITER = 2000;
const SAMPLES = 7;
const bones = [
  { rotation: [1, 0, 0, 0], translation: [0, 0, 0] },
  { rotation: C.quatFromAxisAngle([0, 1, 0], Math.PI * 0.9), translation: [0, 0, 0] },
];

function percentile(sorted, p) {
  const idx = Math.min(sorted.length - 1, Math.max(0, Math.round((p / 100) * (sorted.length - 1))));
  return sorted[idx];
}

function measure(fn) {
  const t0 = performance.now();
  for (let k = 0; k < ITER; k++) fn();
  return performance.now() - t0;
}

const out = new Float64Array(V.length);
function coldOnce(fn) {
  const t0 = performance.now();
  fn();
  return performance.now() - t0;
}
const coldLbs = coldOnce(() => C.skinLinear(V, W, bones, 2, out));
const coldDqs = coldOnce(() => C.skinDualQuaternion(V, W, bones, 2, out));

// warmup
for (let k = 0; k < 5; k++) { C.skinLinear(V, W, bones, 2, out); C.skinDualQuaternion(V, W, bones, 2, out); }

function stats(cold, fn) {
  const xs = [];
  for (let s = 0; s < SAMPLES; s++) xs.push(measure(fn));
  const sorted = [...xs].sort((a, b) => a - b);
  const mean = xs.reduce((a, b) => a + b, 0) / xs.length;
  const variance = xs.reduce((a, b) => a + (b - mean) ** 2, 0) / (xs.length - 1);
  const stddev = Math.sqrt(variance);
  return {
    samples: SAMPLES,
    iterationsPerSample: ITER,
    coldStartMs: cold,
    minMs: sorted[0],
    medianMs: percentile(sorted, 50),
    p95Ms: percentile(sorted, 95),
    maxMs: sorted[sorted.length - 1],
    stddevMs: stddev,
    rsdPct: (stddev / mean) * 100,
    msPer10kVertices: (percentile(sorted, 50) / (ITER * NV)) * 10000,
  };
}

const lbsStats = stats(coldLbs, () => C.skinLinear(V, W, bones, 2, out));
const dqsStats = stats(coldDqs, () => C.skinDualQuaternion(V, W, bones, 2, out));

// Correctness snapshot, recomputed here so result.json is self-contained.
const lbs180 = C.skinLinear(V, W, bonesForShipped(180), 2);
const dqs180 = C.skinDualQuaternion(V, W, bonesForShipped(180), 2);
function bonesForShipped(deg) {
  return [
    { rotation: [1, 0, 0, 0], translation: [0, 0, 0] },
    { rotation: C.quatFromAxisAngle([0, 1, 0], (deg * Math.PI) / 180), translation: [0, 0, 0] },
  ];
}
const joint = [];
for (let v = 0; v < NV; v++) if (Math.abs(V[v * 3 + 1]) < 1e-12) joint.push(v);
const meanR = (arr) => joint.reduce((s, v) => s + Math.hypot(arr[v * 3], arr[v * 3 + 2]), 0) / joint.length;

let oracleMax = 0;
for (const deg of data.meta.anglesDeg) {
  const th = (deg * Math.PI) / 180;
  const lbs = C.skinLinear(V, W, bonesForShipped(deg), 2);
  for (let v = 0; v < NV; v++) {
    const rest = Math.hypot(V[v * 3], V[v * 3 + 2]);
    if (rest < 1e-12) continue;
    const p = A.lbs2d(V[v * 3], V[v * 3 + 2], W[v * 2 + 1], th);
    oracleMax = Math.max(oracleMax, Math.abs(Math.hypot(p.x, p.z) - Math.hypot(lbs[v * 3], lbs[v * 3 + 2])));
  }
}
const vol0 = Math.abs(C.signedVolume(V, I));
const volL = Math.abs(C.signedVolume(lbs180, I)) / vol0;
const volD = Math.abs(C.signedVolume(dqs180, I)) / vol0;

const payload = {
  generatedBy: 'bench/bench.mjs',
  dataset: {
    name: data.meta.name,
    vertices: NV,
    triangles: I.length / 3,
    bones: 2,
    jointRingVertices: joint.length,
  },
  workload: { iterationsPerSample: ITER, samples: SAMPLES, pose: 'twist 0.9*pi (162 deg)' },
  methods: { lbs: lbsStats, dqs: dqsStats },
  correctness: {
    lbsJointRadiusAt180: meanR(lbs180),
    dqsJointRadiusAt180: meanR(dqs180),
    analyticCos90: A.jointRadiusRatio(Math.PI),
    lbsOracleMaxAbsRadiusError: oracleMax,
    weightSumMaxAbsError: C.weightSumErrors(W, 2).maxAbsError,
    signedVolumeRatioAt180: { lbs: volL, dqs: volD },
  },
};
writeFileSync(resolve(root, 'bench/result.json'), JSON.stringify(payload, null, 2) + '\n');
console.log(JSON.stringify(payload, null, 2));