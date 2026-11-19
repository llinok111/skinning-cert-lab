#!/usr/bin/env node
// Deterministic sample rig: a capped cylinder along +Y, two bones, smooth weights.
// Same command twice -> byte-identical data/rig.json (no Date/Math.random).
import { writeFileSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const root = resolve(here, '..');
const SEGMENTS = 64;
const RINGS = 64;          // even -> a ring sits exactly at y = 0
const RADIUS = 1;
const HEIGHT = 4;
const BLEND_HALF = 0.5;    // weight ramp is complete within |y| <= 0.5

const vertices = [];
const weights = [];
const indices = [];

function smoothstep(t) {
  const c = Math.min(1, Math.max(0, t));
  return c * c * (3 - 2 * c);
}

function weightBone1(y) {
  return smoothstep((y + BLEND_HALF) / (2 * BLEND_HALF));
}

for (let j = 0; j <= RINGS; j++) {
  const y = -HEIGHT / 2 + (HEIGHT * j) / RINGS;
  const w1 = weightBone1(y);
  for (let i = 0; i < SEGMENTS; i++) {
    const a = (2 * Math.PI * i) / SEGMENTS;
    vertices.push(RADIUS * Math.cos(a), y, RADIUS * Math.sin(a));
    weights.push(1 - w1, w1);
  }
}

for (let j = 0; j < RINGS; j++) {
  for (let i = 0; i < SEGMENTS; i++) {
    const a = j * SEGMENTS + i;
    const b = j * SEGMENTS + ((i + 1) % SEGMENTS);
    const c = (j + 1) * SEGMENTS + i;
    const d = (j + 1) * SEGMENTS + ((i + 1) % SEGMENTS);
    indices.push(a, c, b, b, c, d);
  }
}

// Caps: one center vertex per cap, fanned to the boundary ring.
for (const [ring, sign] of [[0, -1], [RINGS, 1]]) {
  const y = (-HEIGHT / 2) * sign;
  const w1 = weightBone1(y);
  const center = vertices.length / 3;
  vertices.push(0, y, 0);
  weights.push(1 - w1, w1);
  for (let i = 0; i < SEGMENTS; i++) {
    const a = ring * SEGMENTS + i;
    const b = ring * SEGMENTS + ((i + 1) % SEGMENTS);
    if (sign < 0) indices.push(center, b, a);
    else indices.push(center, a, b);
  }
}

const payload = {
  meta: {
    name: 'capped-cylinder-two-bone-rig',
    segments: SEGMENTS,
    rings: RINGS,
    radius: RADIUS,
    height: HEIGHT,
    blendHalf: BLEND_HALF,
    nBones: 2,
    vertexCount: vertices.length / 3,
    triangleCount: indices.length / 3,
    anglesDeg: [0, 30, 60, 90, 120, 150, 180],
    generator: 'tools/gen-sample.mjs',
  },
  vertices,
  indices,
  weights,
};

mkdirSync(resolve(root, 'data'), { recursive: true });
const text = JSON.stringify(payload);
writeFileSync(resolve(root, 'data/rig.json'), text);
console.log(`wrote data/rig.json  vertices=${payload.meta.vertexCount} tris=${payload.meta.triangleCount} bytes=${Buffer.byteLength(text)}`);