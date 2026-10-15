// Стенд: синтетическая съёмка → склейка → сравнение с эталоном + крупные планы проблемных мест.
// ENV: SEED, HH (радиус вращения вокруг тела, м), PERSON=1, K1, DRIFT, TAG, STITCH (JSON опций склейки)
const C = require('../src/core.js'), fs = require('fs'), { execFileSync } = require('child_process');
const env = process.env, OUT = '/tmp/claude-0/bench/'; fs.mkdirSync(OUT, { recursive: true });
const tag = env.TAG || 'run';
let t = Date.now();
const demo = C.renderDemoFrames({ seed: +(env.SEED || 7), handheld: +(env.HH || 0), person: env.PERSON === '1', k1: +(env.K1 || 0), drift: env.DRIFT ? +env.DRIFT : undefined });
console.log(`[${tag}] frames ${demo.frames.length} render ${Date.now() - t}ms`);
t = Date.now();
const opts = Object.assign({ f0: demo.f0, maxW: 4096 }, env.STITCH ? JSON.parse(env.STITCH) : {});
const notes = [];
const res = C.stitchPanorama(demo.frames, opts, r => { if (r.note) notes.push(r.stage + ':' + r.note); });
console.log(`[${tag}] stitch ${Date.now() - t}ms`, notes.join(' '), JSON.stringify(Object.assign({}, res.stats, { gains: undefined })));
const truth = demo.frames.map(f => f.C2Wtrue), est = res.cams.map(a => Float64Array.from(a));
const M = new Float64Array(9);
for (let i = 0; i < est.length; i++) { const P = C.m3mul(truth[i], C.m3T(est[i])); for (let k = 0; k < 9; k++) M[k] += P[k]; }
const G = C.polar3(M), W = res.W, H = res.H;
// эталон из центра (средней точки съёмки)
const gt = new Uint8ClampedArray(W * H * 4); let se = 0, n = 0, seN = 0, nN = 0;
for (let v = 0; v < H; v++) for (let u = 0; u < W; u++) {
  const d = C.dirFromLonLat((u + 0.5) / W * 2 * Math.PI - Math.PI, Math.PI / 2 - (v + 0.5) / H * Math.PI);
  const dt = [G[0] * d[0] + G[1] * d[1] + G[2] * d[2], G[3] * d[0] + G[4] * d[1] + G[5] * d[2], G[6] * d[0] + G[7] * d[1] + G[8] * d[2]];
  const c = C.demoSceneRay([0, 0, 0], dt, null), p = (v * W + u) * 4;
  for (let q = 0; q < 3; q++) { const g = C.lin2srgb(c[q]); gt[p + q] = g; const e = res.rgba[p + q] - g; se += e * e; n++; }
  gt[p + 3] = 255;
}
const psnr = 10 * Math.log10(255 * 255 / (se / n));
const per = est.map((E, i) => C.rotAngle(C.m3mul(C.m3mul(G, E), C.m3T(truth[i]))) * 180 / Math.PI);
console.log(`[${tag}] PSNR ${psnr.toFixed(2)} dB  cam err mean ${(per.reduce((a, b) => a + b, 0) / per.length).toFixed(3)}° max ${Math.max(...per).toFixed(3)}°`);
// крупные планы: витрина, колонна, скамья/прохожий, пол
const Ginv = C.m3T(G);
const look = (wx, wy, wz, fov) => { const d = [wx, wy, wz], n0 = Math.hypot(...d); const de = [d[0] / n0, d[1] / n0, d[2] / n0]; const e = [Ginv[0] * de[0] + Ginv[1] * de[1] + Ginv[2] * de[2], Ginv[3] * de[0] + Ginv[4] * de[1] + Ginv[5] * de[2], Ginv[6] * de[0] + Ginv[7] * de[1] + Ginv[8] * de[2]]; const ll = C.lonLatFromDir(e); return [C.c2wLookAt(ll[0], ll[1]), fov]; };
const views = { vitrine: look(-1.3, 1.1, -0.6, 55 * Math.PI / 180), column: look(2.2, 1.6, 0.2, 60 * Math.PI / 180), bench: look(-0.6, -3.0, -0.8, 70 * Math.PI / 180), wall: look(0, 6.5, 0.3, 50 * Math.PI / 180) };
const raw = (name, buf, w, h) => { fs.writeFileSync(OUT + name + '.rgba', Buffer.from(buf.buffer, buf.byteOffset, buf.byteLength)); return [OUT + name + '.rgba', w, h]; };
const files = [];
for (const [k, [Cv, fov]] of Object.entries(views)) {
  files.push(raw(`${tag}_${k}`, C.renderPerspective(res.rgba, W, H, Cv, fov, 400), 400, 400));
  files.push(raw(`gt_${k}`, C.renderPerspective(gt, W, H, Cv, fov, 400), 400, 400));
}
files.push(raw(`${tag}_erp`, res.rgba, W, H));
execFileSync('python3', ['-I', '-c', `
import sys
from PIL import Image
args=sys.argv[1:]
for i in range(0,len(args),3):
    p,w,h=args[i],int(args[i+1]),int(args[i+2])
    im=Image.frombytes('RGBA',(w,h),open(p,'rb').read()).convert('RGB')
    im.save(p.replace('.rgba','.png'))
`, ...files.flat().map(String)]);
fs.writeFileSync(OUT + tag + '.json', JSON.stringify({ psnr, stats: res.stats, per }));
if (res.depth) {
  const dd = res.depth, Wd = dd.GW, Hd = dd.GH, img = new Uint8ClampedArray(Wd * Hd * 4);
  const mx = Math.max(...dd.iD);
  for (let k = 0; k < Wd * Hd; k++) { const v = Math.round(255 * dd.iD[k] / mx); img[k * 4] = v; img[k * 4 + 1] = v; img[k * 4 + 2] = v; img[k * 4 + 3] = 255; }
  fs.writeFileSync(OUT + tag + '_depth.rgba', Buffer.from(img.buffer));
  execFileSync('python3', ['-I', '-c', `from PIL import Image; Image.frombytes('RGBA',(${Wd},${Hd}),open('${OUT + tag}_depth.rgba','rb').read()).resize((${Wd * 4},${Hd * 4})).save('${OUT + tag}_depth.png')`]);
  // глубина у вазы и у стены за ней (в единицах «руки»): истина 1.7/0.3≈5.7 и ~6.5/0.3
  const probe = (x, y, z) => { const n = Math.hypot(x, y, z), de = [x / n, y / n, z / n]; const e = [Ginv[0] * de[0] + Ginv[1] * de[1] + Ginv[2] * de[2], Ginv[3] * de[0] + Ginv[4] * de[1] + Ginv[5] * de[2], Ginv[6] * de[0] + Ginv[7] * de[1] + Ginv[8] * de[2]]; const ll = C.lonLatFromDir(e); return (1 / C.depthAt(dd, ll[0], ll[1])).toFixed(2); };
  console.log(`[${tag}] depth vase ${probe(-1.3, 1.1, -0.42)} pedestal ${probe(-1.3, 1.1, -1.0)} column ${probe(2.2, 1.6, 0)} wall ${probe(0, 6.5, 0)} floor ${probe(0, -1, -1.6)}; median ${(1 / dd.median).toFixed(2)}`);
}
