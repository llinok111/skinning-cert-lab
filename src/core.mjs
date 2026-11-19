// Pure skinning math: no three.js, no DOM, no I/O, no environment reads.
// Quaternions are [w, x, y, z]; dual quaternions are {real: q, dual: d}.

export function quatFromAxisAngle(axis, angle) {
  const [ax, ay, az] = axis;
  const len = Math.hypot(ax, ay, az);
  if (len === 0) throw new RangeError('axis must be non-zero');
  const s = Math.sin(angle / 2) / len;
  return [Math.cos(angle / 2), ax * s, ay * s, az * s];
}

export function quatMultiply(a, b) {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}

export function quatConjugate(q) {
  return [q[0], -q[1], -q[2], -q[3]];
}

export function quatNormalize(q) {
  const len = Math.hypot(q[0], q[1], q[2], q[3]);
  if (len === 0) throw new RangeError('cannot normalize zero quaternion');
  return [q[0] / len, q[1] / len, q[2] / len, q[3] / len];
}

export function quatDot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

export function quatRotate(q, v, out = [0, 0, 0]) {
  const [qw, qx, qy, qz] = q;
  const [vx, vy, vz] = v;
  // t = 2 * (q.xyz x v)
  const tx = 2 * (qy * vz - qz * vy);
  const ty = 2 * (qz * vx - qx * vz);
  const tz = 2 * (qx * vy - qy * vx);
  out[0] = vx + qw * tx + (qy * tz - qz * ty);
  out[1] = vy + qw * ty + (qz * tx - qx * tz);
  out[2] = vz + qw * tz + (qx * ty - qy * tx);
  return out;
}

// d = 0.5 * (0, t) * q  ->  point transform is rotate(q, p) + t
export function dualQuatFromRigid(q, t = [0, 0, 0]) {
  const tq = quatMultiply([0, t[0], t[1], t[2]], q);
  return { real: q, dual: [0.5 * tq[0], 0.5 * tq[1], 0.5 * tq[2], 0.5 * tq[3]] };
}

export function transformPointDualQuat(dq, p, out = [0, 0, 0]) {
  const q = dq.real;
  const d = dq.dual;
  const r = quatRotate(q, p, out);
  // translation = 2 * d * conj(q)
  const tx = 2 * (-d[0] * q[1] + d[1] * q[0] - d[2] * q[3] + d[3] * q[2]);
  const ty = 2 * (-d[0] * q[2] + d[1] * q[3] + d[2] * q[0] - d[3] * q[1]);
  const tz = 2 * (-d[0] * q[3] - d[1] * q[2] + d[2] * q[1] + d[3] * q[0]);
  out[0] = r[0] + tx;
  out[1] = r[1] + ty;
  out[2] = r[2] + tz;
  return out;
}

// Dual-quaternion linear blend (DLB). Signs are aligned against the first
// non-zero-weight bone so q and -q describe the same rotation.
export function blendDualQuatsInto(bones, weights, realOut, dualOut) {
  let ref = -1;
  for (let b = 0; b < bones.length; b++) {
    if (weights[b] > 1e-12) { ref = b; break; }
  }
  if (ref < 0) throw new RangeError('no bone has positive weight');
  const rq = bones[ref].real;
  let bw = 0, bx = 0, by = 0, bz = 0;
  let dw = 0, dx = 0, dy = 0, dz = 0;
  for (let b = 0; b < bones.length; b++) {
    const w = weights[b];
    if (w === 0) continue;
    const sign = quatDot(bones[b].real, rq) < 0 ? -1 : 1;
    const q = bones[b].real;
    const d = bones[b].dual;
    bw += sign * w * q[0]; bx += sign * w * q[1];
    by += sign * w * q[2]; bz += sign * w * q[3];
    dw += sign * w * d[0]; dx += sign * w * d[1];
    dy += sign * w * d[2]; dz += sign * w * d[3];
  }
  const len = Math.hypot(bw, bx, by, bz);
  if (len < 1e-9) throw new RangeError('dual-quaternion blend is singular');
  realOut[0] = bw / len; realOut[1] = bx / len;
  realOut[2] = by / len; realOut[3] = bz / len;
  dualOut[0] = dw / len; dualOut[1] = dx / len;
  dualOut[2] = dy / len; dualOut[3] = dz / len;
  return realOut;
}

export function blendDualQuats(bones, weights) {
  const real = [0, 0, 0, 0];
  const dual = [0, 0, 0, 0];
  blendDualQuatsInto(bones, weights, real, dual);
  return { real, dual };
}

export function boneDualQuats(bones) {
  return bones.map((b) => dualQuatFromRigid(quatNormalize(b.rotation), b.translation || [0, 0, 0]));
}

// Linear blend skinning, deliberately WITHOUT weight renormalization so that
// malformed weights produce the real (wrong) result instead of being hidden.
export function skinLinear(vertices, weights, bones, nBones, out = null) {
  const n = vertices.length / 3;
  const dst = out || new Float64Array(vertices.length);
  const qs = bones.map((b) => quatNormalize(b.rotation));
  const ts = bones.map((b) => b.translation || [0, 0, 0]);
  for (let v = 0; v < n; v++) {
    let x = 0, y = 0, z = 0;
    const bx = v * 3, bw = v * nBones;
    const vx = vertices[bx], vy = vertices[bx + 1], vz = vertices[bx + 2];
    for (let b = 0; b < nBones; b++) {
      const w = weights[bw + b];
      if (w === 0) continue;
      const q = qs[b];
      const qw = q[0], qx = q[1], qy = q[2], qz = q[3];
      const tx = 2 * (qy * vz - qz * vy);
      const ty = 2 * (qz * vx - qx * vz);
      const tz = 2 * (qx * vy - qy * vx);
      const rx = vx + qw * tx + (qy * tz - qz * ty);
      const ry = vy + qw * ty + (qz * tx - qx * tz);
      const rz = vz + qw * tz + (qx * ty - qy * tx);
      const tr = ts[b];
      x += w * (rx + tr[0]);
      y += w * (ry + tr[1]);
      z += w * (rz + tr[2]);
    }
    dst[bx] = x; dst[bx + 1] = y; dst[bx + 2] = z;
  }
  return dst;
}

export function skinDualQuaternion(vertices, weights, bones, nBones, out = null) {
  const n = vertices.length / 3;
  const dst = out || new Float64Array(vertices.length);
  const dqs = boneDualQuats(bones);
  const ws = new Float64Array(nBones);
  const rq = [0, 0, 0, 0];
  const rd = [0, 0, 0, 0];
  for (let v = 0; v < n; v++) {
    const bw = v * nBones;
    for (let b = 0; b < nBones; b++) ws[b] = weights[bw + b];
    blendDualQuatsInto(dqs, ws, rq, rd);
    const bx = v * 3;
    const vx = vertices[bx], vy = vertices[bx + 1], vz = vertices[bx + 2];
    const qw = rq[0], qx = rq[1], qy = rq[2], qz = rq[3];
    const tx = 2 * (qy * vz - qz * vy);
    const ty = 2 * (qz * vx - qx * vz);
    const tz = 2 * (qx * vy - qy * vx);
    const rx = vx + qw * tx + (qy * tz - qz * ty);
    const ry = vy + qw * ty + (qz * tx - qx * tz);
    const rz = vz + qw * tz + (qx * ty - qy * tx);
    const dw = rd[0], dx = rd[1], dy = rd[2], dz = rd[3];
    dst[bx] = rx + 2 * (-dw * qx + dx * qw - dy * qz + dz * qy);
    dst[bx + 1] = ry + 2 * (-dw * qy + dx * qz + dy * qw - dz * qx);
    dst[bx + 2] = rz + 2 * (-dw * qz - dx * qy + dy * qx + dz * qw);
  }
  return dst;
}

export function weightSumErrors(weights, nBones) {
  const n = weights.length / nBones;
  const errs = new Float64Array(n);
  let max = 0;
  for (let v = 0; v < n; v++) {
    let s = 0;
    for (let b = 0; b < nBones; b++) s += weights[v * nBones + b];
    const e = Math.abs(s - 1);
    errs[v] = e;
    if (e > max) max = e;
  }
  return { maxAbsError: max, perVertex: errs };
}

export function normalizeWeights(weights, nBones) {
  const n = weights.length / nBones;
  const dst = new Float64Array(weights.length);
  for (let v = 0; v < n; v++) {
    let s = 0;
    for (let b = 0; b < nBones; b++) s += weights[v * nBones + b];
    if (s === 0) throw new RangeError(`vertex ${v} has zero total weight`);
    for (let b = 0; b < nBones; b++) dst[v * nBones + b] = weights[v * nBones + b] / s;
  }
  return dst;
}

export function radialDistances(vertices) {
  const n = vertices.length / 3;
  const out = new Float64Array(n);
  for (let v = 0; v < n; v++) {
    out[v] = Math.hypot(vertices[v * 3], vertices[v * 3 + 2]);
  }
  return out;
}

export function maxAbsDiff(a, b) {
  let m = 0;
  for (let i = 0; i < a.length; i++) {
    const d = Math.abs(a[i] - b[i]);
    if (d > m) m = d;
  }
  return m;
}

export function edgeLengthStats(rest, deformed, indices) {
  let maxRel = 0, sumRel = 0, count = 0;
  for (let t = 0; t < indices.length; t += 3) {
    for (let e = 0; e < 3; e++) {
      const a = indices[t + e] * 3;
      const b = indices[t + ((e + 1) % 3)] * 3;
      const l0 = Math.hypot(rest[a] - rest[b], rest[a + 1] - rest[b + 1], rest[a + 2] - rest[b + 2]);
      if (l0 < 1e-12) continue;
      const l1 = Math.hypot(deformed[a] - deformed[b], deformed[a + 1] - deformed[b + 1], deformed[a + 2] - deformed[b + 2]);
      const rel = Math.abs(l1 - l0) / l0;
      if (rel > maxRel) maxRel = rel;
      sumRel += rel;
      count++;
    }
  }
  return { maxRelativeChange: maxRel, meanRelativeChange: count ? sumRel / count : 0, edges: count };
}

export function signedVolume(vertices, indices) {
  let vol = 0;
  for (let t = 0; t < indices.length; t += 3) {
    const a = indices[t] * 3, b = indices[t + 1] * 3, c = indices[t + 2] * 3;
    const ax = vertices[a], ay = vertices[a + 1], az = vertices[a + 2];
    const bx = vertices[b], by = vertices[b + 1], bz = vertices[b + 2];
    const cx = vertices[c], cy = vertices[c + 1], cz = vertices[c + 2];
    vol += (ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx)) / 6;
  }
  return vol;
}