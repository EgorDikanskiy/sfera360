// Время этапов на «телефонном» разрешении кадров (съёмка с рук). ENV: W,H кадра, OUTW
const C = require('../src/core.js');
const w = +(process.env.FW || 1080), h = +(process.env.FH || 1440);
let t = Date.now();
const demo = C.renderDemoFrames({ seed: 3, handheld: 0.15, person: true, w, h, fTrue: 0.78 * h });
console.log('render', Date.now() - t, 'ms', demo.frames.length, 'frames', w + 'x' + h);
const times = {}; let last = null, lt = Date.now();
t = Date.now();
const res = C.stitchPanorama(demo.frames, { f0: demo.f0, outW: +(process.env.OUTW || 8192), maxW: 8192, deferFill: true }, r => {
  if (r.stage !== last) { const now = Date.now(); if (last) times[last] = (times[last] || 0) + now - lt; last = r.stage; lt = now; }
});
times[last] = (times[last] || 0) + Date.now() - lt;
console.log('total', Date.now() - t, 'ms', JSON.stringify(times));
console.log(JSON.stringify(Object.assign({}, res.stats, { gains: undefined })));
