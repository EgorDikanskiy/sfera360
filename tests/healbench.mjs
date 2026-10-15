// Лечение швов нейросетью на съёмке с рук: до/после на крупных планах
import { createRequire } from 'module';
import { execFileSync } from 'child_process';
import fs from 'fs';
const require = createRequire(import.meta.url);
const C = require('../src/core.js');
const ort = await import('/tmp/claude-0/ortnode/node_modules/onnxruntime-web/dist/ort.node.min.mjs');
ort.env.wasm.numThreads = 2;
const sess = await ort.InferenceSession.create(new URL('../models/migan_pipeline_v2.onnx', import.meta.url).pathname);
const run = async (rgba, mask, S) => {
  const img = new Uint8Array(3 * S * S); for (let i = 0; i < S * S; i++) for (let c = 0; c < 3; c++) img[c * S * S + i] = rgba[i * 4 + c];
  const o = await sess.run({ image: new ort.Tensor('uint8', img, [1, 3, S, S]), mask: new ort.Tensor('uint8', mask, [1, 1, S, S]) });
  const r = o.result.data, out = new Uint8ClampedArray(S * S * 4); for (let i = 0; i < S * S; i++) { for (let c = 0; c < 3; c++) out[i * 4 + c] = r[c * S * S + i]; out[i * 4 + 3] = 255; } return out;
};
const demo = C.renderDemoFrames({ seed: 7, handheld: +(process.env.HH || 0.3), person: true });
const res = C.stitchPanorama(demo.frames, { f0: demo.f0, maxW: 4096, outW: 4096 }, () => {});
const before = new Uint8ClampedArray(res.rgba);
const opts = JSON.parse(process.env.HEAL || '{}');
const t = Date.now();
const r = await C.aiHealSeams(res.rgba, res.covered, res.W, res.H, res.seams, run, null, opts);
console.log('healed', r, Date.now() - t, 'ms');
const truth = demo.frames.map(f => f.C2Wtrue), est = res.cams.map(a => Float64Array.from(a)), M = new Float64Array(9);
for (let i = 0; i < est.length; i++) { const P = C.m3mul(truth[i], C.m3T(est[i])); for (let k = 0; k < 9; k++) M[k] += P[k]; }
const Ginv = C.m3T(C.polar3(M));
const look = (wx, wy, wz, fov) => { const n0 = Math.hypot(wx, wy, wz), de = [wx / n0, wy / n0, wz / n0]; const e = [Ginv[0] * de[0] + Ginv[1] * de[1] + Ginv[2] * de[2], Ginv[3] * de[0] + Ginv[4] * de[1] + Ginv[5] * de[2], Ginv[6] * de[0] + Ginv[7] * de[1] + Ginv[8] * de[2]]; const ll = C.lonLatFromDir(e); return [C.c2wLookAt(ll[0], ll[1]), fov * Math.PI / 180]; };
const views = { vitrine: look(-1.3, 1.1, -0.6, 55), floor: look(0.5, -1.5, -1.6, 80), bench: look(-0.6, -3.0, -0.8, 70), column: look(2.2, 1.6, 0.2, 60) };
const OUT = '/tmp/claude-0/bench/', files = [];
for (const [k, [Cv, fov]] of Object.entries(views)) for (const [tag, buf] of [['before', before], ['healed', res.rgba]]) {
  const im = C.renderPerspective(buf, res.W, res.H, Cv, fov, 380); const f = OUT + `heal_${tag}_${k}.rgba`; fs.writeFileSync(f, Buffer.from(im.buffer)); files.push(f);
}
execFileSync('python3', ['-I', '-c', `
import sys
from PIL import Image
fs=sys.argv[1:]
c=Image.new('RGB',(4*390,2*390),'white')
for i,f in enumerate(fs):
  im=Image.frombytes('RGBA',(380,380),open(f,'rb').read()).convert('RGB'); c.paste(im,((i//2)*390,(i%2)*390))
c.save('${OUT}heal_cmp.png')`, ...files]);
