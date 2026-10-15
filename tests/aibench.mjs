// Демо без надира → склейка → ИИ-дорисовка пустот (onnxruntime-web WASM в Node) → снимки.
import { createRequire } from 'module';
import { execFileSync } from 'child_process';
import fs from 'fs';
const require = createRequire(import.meta.url);
const C = require('../src/core.js');
const ort = await import('/tmp/claude-0/ortnode/node_modules/onnxruntime-web/dist/ort.node.min.mjs').catch(() => import('/tmp/claude-0/ortnode/node_modules/onnxruntime-web/dist/ort.wasm.min.mjs'));
ort.env.wasm.numThreads = 2;
const sess = await ort.InferenceSession.create(new URL('../models/migan_pipeline_v2.onnx', import.meta.url).pathname);
const run = async (rgba, mask, S) => {
  const img = new Uint8Array(3 * S * S); for (let i = 0; i < S * S; i++) for (let c = 0; c < 3; c++) img[c * S * S + i] = rgba[i * 4 + c];
  const t = Date.now();
  const o = await sess.run({ image: new ort.Tensor('uint8', img, [1, 3, S, S]), mask: new ort.Tensor('uint8', mask, [1, 1, S, S]) });
  const r = o.result.data, out = new Uint8ClampedArray(S * S * 4);
  for (let i = 0; i < S * S; i++) { for (let c = 0; c < 3; c++) out[i * 4 + c] = r[c * S * S + i]; out[i * 4 + 3] = 255; }
  console.log('  tile', S, Date.now() - t, 'ms');
  return out;
};
const OUT = '/tmp/claude-0/bench/'; fs.mkdirSync(OUT, { recursive: true });
const mode = process.argv[2] || 'demo';
let res;
if (mode === 'demo') {
  const demo = C.renderDemoFrames({ seed: 7, handheld: 0.12, person: true, skipNadir: true });
  res = C.stitchPanorama(demo.frames, { f0: demo.f0, maxW: 4096, deferFill: true }, () => {});
} else {
  const A = 380 * Math.PI / 180, w = 4200, fc = w / A, h = Math.round(2 * fc * Math.tan(32 * Math.PI / 180)), rgb = new Uint8Array(w * h * 3);
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) { const c = C.demoScene(C.dirFromLonLat((x + 0.5) / fc - Math.PI, Math.atan((h / 2 - y - 0.5) / fc))); for (let q = 0; q < 3; q++) rgb[(y * w + x) * 3 + q] = C.lin2srgb(c[q]); }
  res = C.phonePanoToERP({ w, h, rgb }, { mode: 'cyl', hfov: 380, maxW: 4096, deferFill: true });
}
const W = res.W, H = res.H;
const pp = { rgba: new Uint8ClampedArray(res.rgba), covered: new Uint8Array(res.covered) };
C.fillHoles(pp.rgba, pp.covered, W, H);
let t = Date.now();
const n = await C.aiFillHoles(res.rgba, res.covered, W, H, run, f => {}, JSON.parse(process.env.AIOPTS || '{}'));
console.log(mode, 'AI filled', n, 'px', (n / (W * H) * 100).toFixed(1) + '%', Date.now() - t, 'ms');
C.fillHoles(res.rgba, res.covered, W, H);
const save = (name, buf, w, h) => { fs.writeFileSync(OUT + name + '.rgba', Buffer.from(buf.buffer, buf.byteOffset, buf.byteLength)); execFileSync('python3', ['-I', '-c', `from PIL import Image; Image.frombytes('RGBA',(${w},${h}),open('${OUT + name}.rgba','rb').read()).convert('RGB').save('${OUT + name}.png')`]); };
const views = mode === 'demo' ? { floor: [0.3, -1.45, 100], floor2: [2.5, -1.2, 90] } : { ceil: [0.5, 1.2, 100], floor: [0.5, -1.2, 100], side: [1.5, 0.35, 90] };
for (const [k, [lon, lat, fov]] of Object.entries(views)) {
  const Cv = C.c2wLookAt(lon, lat);
  save(`${mode}_ai_${k}`, C.renderPerspective(res.rgba, W, H, Cv, fov * Math.PI / 180, 420), 420, 420);
  save(`${mode}_pp_${k}`, C.renderPerspective(pp.rgba, W, H, Cv, fov * Math.PI / 180, 420), 420, 420);
}
save(`${mode}_ai_erp`, C.downscaleRGBA(res.rgba, W, H, 2048).rgba, 2048, 1024);
