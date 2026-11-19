// Independent analytic oracle in 2D complex arithmetic.
// It deliberately shares NO code or constants with src/core.mjs: twist about +Y
// leaves y untouched, so the xz part of every LBS vertex is
//     ((1-w) + w * e^{i*theta}) * (x + i*z)
// and its radius has the closed form below. This is the "candy wrapper" law.

export function lbs2d(x, z, w, theta) {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  const re = ((1 - w) + w * c) * x - (w * s) * z;
  const im = ((1 - w) + w * c) * z + (w * s) * x;
  return { x: re, z: im };
}

export function lbsRadiusFactor(w, theta) {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return Math.hypot((1 - w) + w * c, w * s);
}

// At a 50/50 joint between identity and a twist of theta:
export function jointRadiusRatio(theta) {
  return Math.abs(Math.cos(theta / 2));
}