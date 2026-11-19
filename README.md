# skinning-cert-lab

**One-line promise:** in a 2-bone 180° twist, naive linear-blend skinning collapses the joint-ring radius to **0.000000** while dual-quaternion skinning holds **1.000000**; both are checked against an independent analytic law `r·cos(θ/2)` with a maximum error of **8.9e-16** (baseline: `baseline/analytic.mjs`, 2D complex arithmetic, shares no code with `src/core.mjs`).

Upstream: none

## What this is

A skinning **certificate**, not another demo. A capped cylinder (4,162 vertices, 8,320 triangles, 2 bones, smoothstep weights) is twisted around +Y. For every vertex the repo computes:

- **LBS** (linear blend skinning) — the classic `Σ wᵢ (Rᵢp + tᵢ)`.
- **DQS** (dual-quaternion linear blend, DLB) — rigid transforms blended as dual quaternions, with shortest-arc sign alignment.
- **An analytic oracle** — a twist about +Y leaves `y` alone, so the xz part of LBS is `((1−w) + w·e^{iθ})·(x+iz)`. Its radius has a closed form. At a 50/50 joint this is exactly `r·cos(θ/2)`; at 180° it is exactly zero.

The page shows the same numbers the node gate asserts. Drag the twist slider: LBS turns red and pinches at the joint, DQS stays blue. Every value on screen is recomputed by `src/core.mjs` in your browser — the identical module `bench/verify.mjs` imports in node.

## 本轮创新

**B 档 — 首次把“糖纸塌陷（candy wrapper）”做成带独立解析预言机的可现场复算证书。** 台账里没有蒙皮/骨骼变形项目；与 `selfint-*`（自交）、`winding-hull-lab`（内外判定）不同，这里证明的是：LBS 关节半径严格服从 `r·cos(θ/2)`，180° 时塌陷到 0，而 DQS 在同一条环上保持 1.000000，且 DQS 的签名体积比 0.997037 vs LBS 0.870650。关键在于预言机是 2D 复数独立实现，与主实现不共享四元数代码或魔法常量，所以“一致”不是共谋；同时做 1e-6..1e6 五个数量级的尺度不变性复证。

## Quick start (one command)

```bash
node tools/gen-sample.mjs && node bench/verify.mjs
```

Expected tail: `18/18 assertions passed` (exit 0). Then the timing + JSON evidence:

```bash
node bench/bench.mjs
```

To open the interactive page (no build step, no npm install):

```bash
python3 -m http.server 4173
# open http://localhost:4173
```

`index.html` uses a pinned importmap (`three@0.169.0` from unpkg); the only interaction is the twist slider plus an LBS/DQS view toggle, with orbit/zoom.

## Evidence (real runs)

### Correctness — `node bench/verify.mjs`

| Check | Measured | Assertion |
| --- | --- | --- |
| LBS joint radius @180° | 6.68e-17 (i.e. 0) | `< 1e-15` |
| DQS joint radius @180° | 0.9999999999999999 | `|ratio−1| < 1e-12` |
| analytic `cos(90°)` | 6.12e-17 | reference |
| LBS vs 2D oracle, 7 angles, every vertex | max **8.88e-16** | `< 1e-12` |
| DQS radial deviation, all verts/angles | max **1.11e-15** | `< 1e-12` |
| core LBS vs Rodrigues-matrix baseline | max **3.33e-16** | `< 1e-12` |
| single-bone DQS vs LBS | max **4.44e-16** | `< 1e-12` |
| weight sums `|Σw−1|` | **0** | `< 1e-15` |
| unnormalized corpus (0.9×) detected | 0.100000 | `0.09..0.11` |
| repair restores ref deformation | 0.000e+0, unnormalized shift 0.200000 | `< 1e-12` |
| scale 1e-6..1e6, joint law | LBS err 7.12e-17, DQS err 4.44e-16 | `< 1e-12` |
| 190° double-cover | LBS 0.087155743 = `|cos95°|`; DQS 1.000000 | `< 1e-12` |

The full table with per-angle values is printed by the script; 18 assertions, exit 0.

### Performance — `node bench/bench.mjs` → `bench/result.json`

| Method | cold start | median (2,000 calls) | per 10k verts | p95 | RSD |
| --- | --- | --- | --- | --- | --- |
| LBS | 5.68 ms | 168.35 ms | 0.2023 ms | 195.76 ms | 6.09% |
| DQS | 7.85 ms | 904.10 ms | 1.0861 ms | 936.28 ms | 1.88% |

DQS costs **5.37×** LBS per call on this 4,162-vertex rig. That is the honest price of the certificate; this repo does not claim DQS is free. 7 samples × 2,000 calls, median/p95/stddev, cold start recorded separately.

### Volume at 180°

Signed volume of the deformed closed surface, normalized by the rest volume: **LBS 0.870650**, **DQS 0.997037**.

## Known limitations (read this before quoting a number)

- **Absolute tolerance is 1e-12 on unit-scale geometry.** The scale test asserts only *dimensionless ratios* (joint radius / rest radius) from 1e-6 to 1e6; absolute positional comparisons at 1e6 would carry ~1e-10 absolute error. Do not reuse the 1e-12 constant on other scales.
- **DQS is not distortion-free.** It fixes the candy-wrapper radius/volume collapse, but it is not a global isometry. On this coarse rig the maximum edge-length relative change at 180° is **4.952** for DQS vs **2.147** for LBS (`src/core.mjs#edgeLengthStats`); the win is radius and volume, not shear. Reproduce with a two-line `node --input-type=module -e` call or extend `bench/verify.mjs`.
- **Rigid bones only.** DQS here assumes rotation+translation. Bones with scale/shear are out of scope; LBS can represent them, DQS cannot.
- **Two bones, one twist axis.** The analytic oracle is exact only because every bone rotates about +Y and `y` is invariant. A bend would need a different oracle.
- **Antipodal blends are singular.** If the weighted quaternion sum has norm < 1e-9 (e.g. 180° opposing rotations with equal weights), `blendDualQuats` throws instead of returning garbage.
- **Synthetic corpus.** The cylinder is generated by `tools/gen-sample.mjs` (fixed topology, no randomness); real scanned rigs have irregular weights and the shear numbers will differ.
- **Signed volume can be misleading** once the deformed surface self-intersects (LBS at large angles does); it is a surface integral, not a boolean volume.
- The browser needs the pinned CDN to load three; node gates need no network.

## Files

```
src/core.mjs          pure math: quaternions, dual quaternions, LBS/DQS, metrics (no three, no DOM)
baseline/analytic.mjs independent 2D-complex oracle (no shared code/constants)
baseline/naive.mjs    Rodrigues-matrix LBS, used only to cross-check core positions
tools/gen-sample.mjs  deterministic capped-cylinder rig -> data/rig.json
bench/verify.mjs      18 assertions over the generated data
bench/bench.mjs       warmup + 7 samples x 2,000 calls; writes bench/result.json
index.html, main.js   importmap page; slider + LBS/DQS toggle, live core recompute
```

## License

MIT — Copyright (c) 2026 llinok111. See `LICENSE`.