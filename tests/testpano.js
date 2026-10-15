const C = require('../src/core.js');
// синтетическая цилиндрическая панорама 380°, вертикаль ±32°
const A = 380 * Math.PI / 180, w = 4200, fc = w / A, h = Math.round(2 * fc * Math.tan(32 * Math.PI / 180));
const rgb = new Uint8Array(w * h * 3);
for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
  const lon = (x + 0.5) / fc - Math.PI, lat = Math.atan((h / 2 - y - 0.5) / fc);
  const c = C.demoScene(C.dirFromLonLat(lon, lat)); for (let q = 0; q < 3; q++) rgb[(y * w + x) * 3 + q] = C.lin2srgb(c[q]);
}
const per = C.detectPeriod(rgb, w, h); console.log('size', w, h, 'period', per, 'expected', Math.round(fc * 2 * Math.PI), 'A est', per && (360 * w / per.period).toFixed(1));
const r = C.phonePanoToERP({ w, h, rgb }, { mode: 'cyl', hfov: per ? 360 * w / per.period : 360, maxW: 4096 });
console.log(r.W, r.H, r.stats);
require('fs').writeFileSync('/tmp/claude-0/pano.rgb', Buffer.from(r.rgba.filter((_, i) => i % 4 !== 3)));
require('fs').writeFileSync('/tmp/claude-0/pdims.json', JSON.stringify({ W: r.W, H: r.H }));
