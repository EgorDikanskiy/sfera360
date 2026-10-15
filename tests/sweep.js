// Устойчивость: разные «съёмки» (seed), разная амплитуда движения рук, прохожий. Ищем провалы.
const C = require('../src/core.js');
const rows = [];
for (const hh of [0, 0.15, 0.3]) for (const seed of [1, 2, 3, 4]) {
  const w = 450, h = 600, fTrue = 471;
  const demo = C.renderDemoFrames({ seed, handheld: hh, person: true, w, h, fTrue, drift: 0.2 });
  const t = Date.now();
  let res, err = null;
  try { res = C.stitchPanorama(demo.frames, { f0: demo.f0, maxW: 2048, outW: 2048 }, () => {}); } catch (e) { err = e.message; }
  if (err) { console.log(`hh=${hh} seed=${seed} ERROR ${err}`); continue; }
  const truth = demo.frames.map(f => f.C2Wtrue), est = res.cams.map(a => Float64Array.from(a)), M = new Float64Array(9);
  for (let i = 0; i < est.length; i++) { const P = C.m3mul(truth[i], C.m3T(est[i])); for (let k = 0; k < 9; k++) M[k] += P[k]; }
  const G = C.polar3(M), W = res.W, H = res.H; let se = 0, n = 0, nan = 0;
  for (let v = 0; v < H; v += 2) for (let u = 0; u < W; u += 2) { const d = C.dirFromLonLat((u + .5) / W * 2 * Math.PI - Math.PI, Math.PI / 2 - (v + .5) / H * Math.PI); const dt = [G[0] * d[0] + G[1] * d[1] + G[2] * d[2], G[3] * d[0] + G[4] * d[1] + G[5] * d[2], G[6] * d[0] + G[7] * d[1] + G[8] * d[2]]; const c = C.demoSceneRay([0, 0, 0], dt, null); for (let q = 0; q < 3; q++) { const val = res.rgba[(v * W + u) * 4 + q]; if (val !== val) nan++; const e = val - C.lin2srgb(c[q]); se += e * e; n++; } }
  const per = est.map((E, i) => C.rotAngle(C.m3mul(C.m3mul(G, E), C.m3T(truth[i]))) * 180 / Math.PI);
  const s = res.stats;
  console.log(`hh=${hh} seed=${seed} PSNR ${(10 * Math.log10(65025 / (se / n))).toFixed(2)} rms ${s.rmsRot.toFixed(2)}→${s.rms.toFixed(2)} par ${s.parallax} k1 ${s.k1.toFixed(3)} f ${s.f.toFixed(0)}/${fTrue} linked ${s.linked}/${s.frames} susp ${s.suspicious} camErr max ${Math.max(...per).toFixed(2)}° ${Date.now() - t}ms`);
}
