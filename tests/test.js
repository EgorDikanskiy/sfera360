const C = require('../src/core.js');
const fs = require('fs');
const seed = +(process.argv[2] || 7);
let t = Date.now();
const demo = C.renderDemoFrames({ seed, drift: +(process.argv[3] || 0.12) });
console.log('frames', demo.frames.length, 'render ms', Date.now() - t, 'f0', demo.f0, 'fTrue', demo.fTrue);
// исходная ошибка гироскопа (относительные повороты соседей)
const relErr = (A, B) => {
  let s = 0, n = 0, mx = 0;
  for (let i = 0; i < A.length; i++) for (let j = i + 1; j < A.length; j++) {
    const ra = C.m3mul(C.m3T(A[j]), A[i]), rb = C.m3mul(C.m3T(B[j]), B[i]);
    const e = C.rotAngle(C.m3mul(ra, C.m3T(rb))) * 180 / Math.PI; s += e; n++; mx = Math.max(mx, e);
  }
  return [s / n, mx];
};
const truth = demo.frames.map(f => f.C2Wtrue);
console.log('gyro rel err mean/max deg', relErr(demo.frames.map(f => f.C2W), truth).map(v => v.toFixed(3)));
t = Date.now();
let last = '';
const res = C.stitchPanorama(demo.frames, { f0: demo.f0, maxW: 4096 }, r => { if (r.stage !== last) { last = r.stage; process.stdout.write(`[${r.stage} ${Date.now() - t}ms] `); } if (r.note) process.stdout.write(`(${r.stage}: rms ${r.note}) `); });
console.log('\nstitch ms', Date.now() - t);
console.log(JSON.stringify(Object.assign({}, res.stats, { gains: undefined })));
const est = res.cams.map(a => Float64Array.from(a));
console.log('est rel err mean/max deg', relErr(est, truth).map(v => v.toFixed(4)), 'f err %', ((res.stats.f / demo.fTrue - 1) * 100).toFixed(2));
// эталон в той же системе координат
const M = new Float64Array(9);
for (let i = 0; i < est.length; i++) { const P = C.m3mul(truth[i], C.m3T(est[i])); for (let k = 0; k < 9; k++) M[k] += P[k]; }
const G = C.polar3(M);
const W = res.W, H = res.H; let se = 0, n = 0;
const gt = new Uint8Array(W * H * 3);
for (let v = 0; v < H; v++) for (let u = 0; u < W; u++) {
  const lon = (u + 0.5) / W * 2 * Math.PI - Math.PI, lat = Math.PI / 2 - (v + 0.5) / H * Math.PI;
  const d = C.dirFromLonLat(lon, lat);
  const dt = [G[0] * d[0] + G[1] * d[1] + G[2] * d[2], G[3] * d[0] + G[4] * d[1] + G[5] * d[2], G[6] * d[0] + G[7] * d[1] + G[8] * d[2]];
  const c = C.demoScene(dt);
  for (let q = 0; q < 3; q++) {
    const g = C.lin2srgb(c[q]); gt[(v * W + u) * 3 + q] = g;
    if (res.covered[v * W + u]) { const e = res.rgba[(v * W + u) * 4 + q] - g; se += e * e; n++; }
  }
}
console.log('PSNR vs truth (covered):', (10 * Math.log10(255 * 255 / (se / n))).toFixed(2), 'dB; global misalign deg', (C.rotAngle(G) * 180 / Math.PI).toFixed(3));
const rgb = new Uint8Array(W * H * 3); for (let i = 0; i < W * H; i++) for (let q = 0; q < 3; q++) rgb[i * 3 + q] = res.rgba[i * 4 + q];
fs.writeFileSync('/tmp/claude-0/out.rgb', rgb); fs.writeFileSync('/tmp/claude-0/gt.rgb', gt);
fs.writeFileSync('/tmp/claude-0/dims.json', JSON.stringify({ W, H }));
// один исходный кадр
const f0 = demo.frames[0]; fs.writeFileSync('/tmp/claude-0/f0.rgb', f0.rgb); fs.writeFileSync('/tmp/claude-0/f0.json', JSON.stringify({ w: f0.w, h: f0.h }));
// ошибки по кадрам после глобального выравнивания
const per = est.map((E, i) => C.rotAngle(C.m3mul(C.m3mul(G, E), C.m3T(truth[i]))) * 180 / Math.PI);
console.log('per-cam abs err deg:', per.map((e, i) => `${i}:${demo.plan[i].pitch}/${e.toFixed(2)}`).join(' '));
const up = [G[2], G[5], G[8]]; console.log('horizon tilt deg', (Math.acos(Math.min(1, up[2])) * 180 / Math.PI).toFixed(3));
