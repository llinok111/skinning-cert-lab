// Naive matrix LBS baseline: Rodrigues rotation matrices, no quaternions and no
// imports from src/core.mjs. Used only to cross-check core's vertex positions.

export function rodrigues(axis, angle) {
  const [x, y, z] = axis;
  const n = Math.hypot(x, y, z);
  const ux = x / n, uy = y / n, uz = z / n;
  const c = Math.cos(angle), s = Math.sin(angle), C = 1 - c;
  return [
    [c + ux * ux * C, ux * uy * C - uz * s, ux * uz * C + uy * s],
    [uy * ux * C + uz * s, c + uy * uy * C, uy * uz * C - ux * s],
    [uz * ux * C - uy * s, uz * uy * C + ux * s, c + uz * uz * C],
  ];
}

export function applyMat(m, v) {
  return [
    m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
    m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
    m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
  ];
}

export function naiveLbs(vertices, weights, matrices, nBones) {
  const out = new Float64Array(vertices.length);
  const n = vertices.length / 3;
  for (let v = 0; v < n; v++) {
    let x = 0, y = 0, z = 0;
    for (let b = 0; b < nBones; b++) {
      const w = weights[v * nBones + b];
      if (w === 0) continue;
      const p = [vertices[v * 3], vertices[v * 3 + 1], vertices[v * 3 + 2]];
      const q = applyMat(matrices[b], p);
      x += w * q[0]; y += w * q[1]; z += w * q[2];
    }
    out[v * 3] = x; out[v * 3 + 1] = y; out[v * 3 + 2] = z;
  }
  return out;
}