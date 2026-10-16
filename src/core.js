'use strict';
/* Сфера 360 — ядро склейки.
   Мир: X — восток, Y — север, Z — вверх (как у deviceorientation).
   Камера: x вправо, y вниз, z вперёд. C2W — матрица 3x3 (по строкам), world = C2W * cam. */

// ───────────────────────── линейная алгебра 3x3 ─────────────────────────
function m3mul(a, b) {
  const r = new Float64Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++)
    r[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
  return r;
}
function m3T(a) { return Float64Array.of(a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]); }
function m3copy(a) { return Float64Array.from(a); }
function rotX(t) { const c = Math.cos(t), s = Math.sin(t); return Float64Array.of(1, 0, 0, 0, c, -s, 0, s, c); }
function rotY(t) { const c = Math.cos(t), s = Math.sin(t); return Float64Array.of(c, 0, s, 0, 1, 0, -s, 0, c); }
function rotZ(t) { const c = Math.cos(t), s = Math.sin(t); return Float64Array.of(c, -s, 0, s, c, 0, 0, 0, 1); }
function expSO3(wx, wy, wz) {
  const th = Math.hypot(wx, wy, wz);
  if (th < 1e-12) return Float64Array.of(1, -wz, wy, wz, 1, -wx, -wy, wx, 1);
  const kx = wx / th, ky = wy / th, kz = wz / th, c = Math.cos(th), s = Math.sin(th), v = 1 - c;
  return Float64Array.of(
    c + kx * kx * v, kx * ky * v - kz * s, kx * kz * v + ky * s,
    ky * kx * v + kz * s, c + ky * ky * v, ky * kz * v - kx * s,
    kz * kx * v - ky * s, kz * ky * v + kx * s, c + kz * kz * v);
}
function logSO3(R) {
  const tr = R[0] + R[4] + R[8];
  let c = (tr - 1) / 2; c = Math.max(-1, Math.min(1, c));
  const th = Math.acos(c);
  const vx = R[7] - R[5], vy = R[2] - R[6], vz = R[3] - R[1];
  if (th < 1e-9) return [vx / 2, vy / 2, vz / 2];
  const s = th / (2 * Math.sin(th));
  if (Math.sin(th) < 1e-6) { // около 180°
    const x = Math.sqrt(Math.max(0, (R[0] + 1) / 2)), y = Math.sqrt(Math.max(0, (R[4] + 1) / 2)), z = Math.sqrt(Math.max(0, (R[8] + 1) / 2));
    return [x * th, Math.sign(R[1] || 1) * y * th, Math.sign(R[2] || 1) * z * th];
  }
  return [vx * s, vy * s, vz * s];
}
function rotAngle(R) { const c = Math.max(-1, Math.min(1, (R[0] + R[4] + R[8] - 1) / 2)); return Math.acos(c); }
function det3(m) { return m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]); }
// ортогональный полярный множитель (итерации Хайэма): ближайшее вращение к M
function polar3(M) {
  let X = m3copy(M);
  for (let it = 0; it < 30; it++) {
    const d = det3(X); if (Math.abs(d) < 1e-15) break;
    const C = Float64Array.of(
      X[4] * X[8] - X[5] * X[7], -(X[3] * X[8] - X[5] * X[6]), X[3] * X[7] - X[4] * X[6],
      -(X[1] * X[8] - X[2] * X[7]), X[0] * X[8] - X[2] * X[6], -(X[0] * X[7] - X[1] * X[6]),
      X[1] * X[5] - X[2] * X[4], -(X[0] * X[5] - X[2] * X[3]), X[0] * X[4] - X[1] * X[3]);
    let diff = 0;
    for (let k = 0; k < 9; k++) { const nv = 0.5 * (X[k] + C[k] / d); diff += Math.abs(nv - X[k]); X[k] = nv; }
    if (diff < 1e-13) break;
  }
  return X;
}
function normalize3(v) { const n = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / n, v[1] / n, v[2] / n]; }

// deviceorientation (градусы) + угол экрана → C2W задней камеры
function gyroToC2W(alpha, beta, gamma, screenAngle) {
  const d = Math.PI / 180;
  const R = m3mul(m3mul(rotZ((alpha || 0) * d), rotX((beta || 0) * d)), rotY((gamma || 0) * d));
  const S = rotZ((screenAngle || 0) * d);
  const F = Float64Array.of(1, 0, 0, 0, -1, 0, 0, 0, -1);
  return m3mul(m3mul(R, S), F);
}
function dirFromLonLat(lon, lat) { const c = Math.cos(lat); return [c * Math.sin(lon), c * Math.cos(lon), Math.sin(lat)]; }
function lonLatFromDir(d) { return [Math.atan2(d[0], d[1]), Math.asin(Math.max(-1, Math.min(1, d[2])))]; }
function camForward(C2W) { return [C2W[2], C2W[5], C2W[8]]; }

function c2wLookAt(lon, lat) {
  const fwd = dirFromLonLat(lon, lat);
  let up = [0, 0, 1]; if (Math.abs(fwd[2]) > 0.999) up = [Math.sin(lon), Math.cos(lon), 0];
  const right = normalize3([fwd[1] * up[2] - fwd[2] * up[1], fwd[2] * up[0] - fwd[0] * up[2], fwd[0] * up[1] - fwd[1] * up[0]]);
  const down = [fwd[1] * right[2] - fwd[2] * right[1], fwd[2] * right[0] - fwd[0] * right[2], fwd[0] * right[1] - fwd[1] * right[0]];
  return Float64Array.of(right[0], down[0], fwd[0], right[1], down[1], fwd[1], right[2], down[2], fwd[2]);
}

// ───────────────────────── план съёмки ─────────────────────────
// hfov/vfov — углы кадра в градусах (как держат телефон). Возвращает цели {yaw, pitch} в градусах.
function makePlan(hfov, vfov, overlap) {
  const ov = overlap == null ? 0.33 : overlap;
  const capR = Math.min(hfov, vfov) / 2;
  const step = vfov * (1 - ov);
  const rowsUp = [];
  for (let p = step; ; p += step) {
    rowsUp.push(Math.min(p, 90 - capR * 0.55));
    if (p + vfov / 2 >= 90 - capR + 8 || rowsUp.length > 4) break;
  }
  const targets = [];
  const addRow = (p, idx) => {
    const L = Math.max(0, Math.abs(p) - vfov / 4) * Math.PI / 180;
    const n = Math.max(3, Math.ceil(360 * Math.cos(L) / (hfov * (1 - ov))));
    const off = (idx % 2) ? 180 / n : 0;
    for (let k = 0; k < n; k++) targets.push({ yaw: (off + k * 360 / n) % 360, pitch: p, row: idx });
  };
  addRow(0, 0);
  rowsUp.forEach((p, k) => addRow(p, 2 * k + 1));
  rowsUp.forEach((p, k) => addRow(-p, 2 * k + 2));
  targets.push({ yaw: 0, pitch: 90, row: 98, pole: true });
  targets.push({ yaw: 0, pitch: -90, row: 99, pole: true });
  return targets;
}

// ───────────────────────── цвет ─────────────────────────
const SRGB2LIN = new Float32Array(256);
for (let i = 0; i < 256; i++) { const c = i / 255; SRGB2LIN[i] = c <= 0.04045 ? c / 12.92 : Math.pow((c + 0.055) / 1.055, 2.4); }
const LIN2SRGB_N = 4096;
const LIN2SRGB = new Uint8Array(LIN2SRGB_N + 1);
for (let i = 0; i <= LIN2SRGB_N; i++) {
  const c = i / LIN2SRGB_N; const s = c <= 0.0031308 ? c * 12.92 : 1.055 * Math.pow(c, 1 / 2.4) - 0.055;
  LIN2SRGB[i] = Math.max(0, Math.min(255, Math.round(s * 255)));
}
function lin2srgb(v) { if (!(v > 0)) return 0; if (v >= 1) return 255; return LIN2SRGB[(v * LIN2SRGB_N + 0.5) | 0]; }

// ───────────────────────── пирамиды и признаки ─────────────────────────
function grayDown(rgb, w, h, k) {
  const W = Math.floor(w / k), H = Math.floor(h / k), d = new Float32Array(W * H), inv = 1 / (k * k * 255);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    let s = 0;
    for (let yy = 0; yy < k; yy++) { let p = ((y * k + yy) * w + x * k) * 3; for (let xx = 0; xx < k; xx++, p += 3) s += 0.299 * rgb[p] + 0.587 * rgb[p + 1] + 0.114 * rgb[p + 2]; }
    d[y * W + x] = s * inv;
  }
  return { w: W, h: H, d };
}
function halve(img) {
  const W = Math.floor(img.w / 2), H = Math.floor(img.h / 2), d = new Float32Array(W * H), s = img.d, w = img.w;
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    const p = 2 * y * w + 2 * x; d[y * W + x] = 0.25 * (s[p] + s[p + 1] + s[p + w] + s[p + w + 1]);
  }
  return { w: W, h: H, d };
}
function buildPyramid(rgb, w, h, levels) {
  const k = Math.max(1, Math.round(Math.max(w, h) / 960));
  const pyr = [grayDown(rgb, w, h, k)];
  for (let l = 1; l < levels; l++) pyr.push(halve(pyr[l - 1]));
  return { k, levels: pyr };
}
function sampleF(img, x, y) {
  if (x < 0 || y < 0 || x > img.w - 1 || y > img.h - 1) return NaN;
  let x0 = x | 0, y0 = y | 0; if (x0 >= img.w - 1) x0 = img.w - 2; if (y0 >= img.h - 1) y0 = img.h - 2;
  const fx = x - x0, fy = y - y0, w = img.w, d = img.d, p = y0 * w + x0;
  return (d[p] * (1 - fx) + d[p + 1] * fx) * (1 - fy) + (d[p + w] * (1 - fx) + d[p + w + 1] * fx) * fy;
}
// Ши–Томаси с раскладкой по ячейкам (равномерное покрытие кадра)
function detectCorners(g, cell, maxN) {
  const w = g.w, h = g.h, d = g.d;
  const ixx = new Float32Array(w * h), iyy = new Float32Array(w * h), ixy = new Float32Array(w * h);
  for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) {
    const p = y * w + x;
    const gx = (d[p + 1] - d[p - 1]) * 2 + d[p - w + 1] - d[p - w - 1] + d[p + w + 1] - d[p + w - 1];
    const gy = (d[p + w] - d[p - w]) * 2 + d[p + w - 1] - d[p - w - 1] + d[p + w + 1] - d[p - w + 1];
    ixx[p] = gx * gx; iyy[p] = gy * gy; ixy[p] = gx * gy;
  }
  const r = 2, resp = new Float32Array(w * h);
  // интегральные изображения
  const I = (src) => { const S = new Float64Array((w + 1) * (h + 1)); for (let y = 0; y < h; y++) { let row = 0; for (let x = 0; x < w; x++) { row += src[y * w + x]; S[(y + 1) * (w + 1) + x + 1] = S[y * (w + 1) + x + 1] + row; } } return S; };
  const Sxx = I(ixx), Syy = I(iyy), Sxy = I(ixy);
  const box = (S, x, y) => S[(y + r + 1) * (w + 1) + x + r + 1] - S[(y - r) * (w + 1) + x + r + 1] - S[(y + r + 1) * (w + 1) + x - r] + S[(y - r) * (w + 1) + x - r];
  let mx = 0;
  const m = 8;
  for (let y = m; y < h - m; y++) for (let x = m; x < w - m; x++) {
    const a = box(Sxx, x, y), b = box(Sxy, x, y), c = box(Syy, x, y);
    const tr = a + c, dt = a * c - b * b;
    const v = tr / 2 - Math.sqrt(Math.max(0, tr * tr / 4 - dt));
    resp[y * w + x] = v; if (v > mx) mx = v;
  }
  const thr = mx * 0.008;
  const cand = [];
  for (let cy = m; cy < h - m; cy += cell) for (let cx = m; cx < w - m; cx += cell) {
    let best = thr, bx = -1, by = -1;
    for (let y = cy; y < Math.min(cy + cell, h - m); y++) for (let x = cx; x < Math.min(cx + cell, w - m); x++) {
      const v = resp[y * w + x];
      if (v > best && v >= resp[y * w + x - 1] && v >= resp[y * w + x + 1] && v >= resp[(y - 1) * w + x] && v >= resp[(y + 1) * w + x]) { best = v; bx = x; by = y; }
    }
    if (bx >= 0) cand.push([bx, by, best]);
  }
  cand.sort((p, q) => q[2] - p[2]);
  return cand.slice(0, maxN);
}

// ───────────────────────── сопоставление ─────────────────────────
function Kmat(f, cx, cy) { return Float64Array.of(f, 0, cx, 0, f, cy, 0, 0, 1); }
function Kinv(f, cx, cy) { return Float64Array.of(1 / f, 0, -cx / f, 0, 1 / f, -cy / f, 0, 0, 1); }
function levelA(kf) { const b = 0.5 / kf - 0.5; return Float64Array.of(1 / kf, 0, b, 0, 1 / kf, b, 0, 0, 1); }
function levelAinv(kf) { const b = 0.5 / kf - 0.5; return Float64Array.of(kf, 0, -b * kf, 0, kf, -b * kf, 0, 0, 1); }
function hApply(H, x, y, out) {
  const z = H[6] * x + H[7] * y + H[8];
  if (z <= 1e-9) { out[0] = NaN; out[1] = NaN; return; }
  out[0] = (H[0] * x + H[1] * y + H[2]) / z; out[1] = (H[3] * x + H[4] * y + H[5]) / z;
}
// гомография пикселей i → j (полное разрешение)
function pairHomography(ci, cj, f) {
  const Rrel = m3mul(m3T(cj.C2W), ci.C2W);
  return m3mul(m3mul(Kmat(f, cj.cx, cj.cy), Rrel), Kinv(f, ci.cx, ci.cy));
}
function makeTemplate(img, x, y, t) {
  const n = 2 * t + 1, T = new Float32Array(n * n);
  let s = 0, ss = 0;
  for (let v = -t, q = 0; v <= t; v++) for (let u = -t; u <= t; u++, q++) {
    const val = sampleF(img, x + u, y + v); if (val !== val) return null;
    T[q] = val; s += val; ss += val * val;
  }
  const N = n * n, mean = s / N, varr = ss - s * s / N;
  if (varr < 1e-5 * N) return null;
  const inv = 1 / Math.sqrt(varr);
  for (let q = 0; q < N; q++) T[q] = (T[q] - mean) * inv;
  return T;
}
// поиск по окну: S берётся из j через гомографию уровня, ищем смещение o ∈ [-R,R]²
const _hp = [0, 0];
function searchNCC(T, t, imgJ, HL, cx, cy, R, sxL, syL) {
  const n = 2 * t + 1, sn = 2 * (t + R) + 1, S = new Float32Array(sn * sn);
  sxL = sxL || 0; syL = syL || 0;
  let bad = 0;
  for (let v = 0, q = 0; v < sn; v++) for (let u = 0; u < sn; u++, q++) {
    hApply(HL, cx + u - t - R, cy + v - t - R, _hp);
    const val = _hp[0] === _hp[0] ? sampleF(imgJ, _hp[0] + sxL, _hp[1] + syL) : NaN;
    S[q] = val; if (val !== val) bad++;
  }
  if (bad > sn * sn * 0.5) return null;
  const R2 = 2 * R + 1, score = new Float32Array(R2 * R2);
  let best = -2, bi = -1;
  const N = n * n;
  for (let oy = 0; oy < R2; oy++) for (let ox = 0; ox < R2; ox++) {
    let s = 0, ss = 0, dot = 0, ok = true;
    for (let v = 0; v < n && ok; v++) {
      const row = (oy + v) * sn + ox, trow = v * n;
      for (let u = 0; u < n; u++) {
        const val = S[row + u]; if (val !== val) { ok = false; break; }
        s += val; ss += val * val; dot += T[trow + u] * val;
      }
    }
    let sc = -2;
    if (ok) { const vv = ss - s * s / N; sc = vv > 1e-7 ? dot / Math.sqrt(vv) : -2; }
    score[oy * R2 + ox] = sc;
    if (sc > best) { best = sc; bi = oy * R2 + ox; }
  }
  if (bi < 0) return null;
  const by = (bi / R2) | 0, bx = bi % R2;
  let dx = 0, dy = 0;
  if (bx > 0 && bx < R2 - 1) { const a = score[bi - 1], c = score[bi + 1]; if (a > -1.5 && c > -1.5) { const den = a - 2 * best + c; if (den < -1e-6) dx = Math.max(-0.5, Math.min(0.5, 0.5 * (a - c) / den)); } }
  if (by > 0 && by < R2 - 1) { const a = score[bi - R2], c = score[bi + R2]; if (a > -1.5 && c > -1.5) { const den = a - 2 * best + c; if (den < -1e-6) dy = Math.max(-0.5, Math.min(0.5, 0.5 * (a - c) / den)); } }
  return { ox: bx - R + dx, oy: by - R + dy, score: best };
}

// поворот по двум лучам (триада) и по многим (Кабш через полярное разложение)
function rotFrom2(a1, a2, b1, b2) {
  const fa = triad(a1, a2), fb = triad(b1, b2); if (!fa || !fb) return null;
  return m3mul(fb, m3T(fa));
}
function triad(p, q) {
  const e1 = p; let e2 = [p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0]];
  const n = Math.hypot(e2[0], e2[1], e2[2]); if (n < 1e-6) return null; e2 = [e2[0] / n, e2[1] / n, e2[2] / n];
  const e3 = [e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]];
  return Float64Array.of(e1[0], e2[0], e3[0], e1[1], e2[1], e3[1], e1[2], e2[2], e3[2]);
}
function kabsch(A, B, idx) {
  const M = new Float64Array(9);
  for (const k of idx) { const a = A[k], b = B[k]; for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) M[r * 3 + c] += b[r] * a[c]; }
  return polar3(M);
}
function rng(seed) { let s = seed >>> 0 || 1; return () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s / 4294967296; }; }
function ransacRotation(A, B, thrRad, iters, rand) {
  const n = A.length; if (n < 6) return null;
  const cthr = Math.cos(thrRad);
  let bestIdx = null;
  for (let it = 0; it < iters; it++) {
    const i1 = (rand() * n) | 0; let i2 = (rand() * n) | 0; if (i2 === i1) i2 = (i1 + 1) % n;
    const R = rotFrom2(A[i1], A[i2], B[i1], B[i2]); if (!R) continue;
    const idx = [];
    for (let k = 0; k < n; k++) {
      const a = A[k], b = B[k];
      const x = R[0] * a[0] + R[1] * a[1] + R[2] * a[2], y = R[3] * a[0] + R[4] * a[1] + R[5] * a[2], z = R[6] * a[0] + R[7] * a[1] + R[8] * a[2];
      if (x * b[0] + y * b[1] + z * b[2] > cthr) idx.push(k);
    }
    if (!bestIdx || idx.length > bestIdx.length) { bestIdx = idx; if (idx.length > 0.9 * n && it > 20) break; }
  }
  if (!bestIdx || bestIdx.length < 6) return null;
  // уточнение по всем инлаерам и повторный отбор
  let R = kabsch(A, B, bestIdx);
  for (let pass = 0; pass < 2; pass++) {
    const idx = [];
    for (let k = 0; k < n; k++) {
      const a = A[k], b = B[k];
      const x = R[0] * a[0] + R[1] * a[1] + R[2] * a[2], y = R[3] * a[0] + R[4] * a[1] + R[5] * a[2], z = R[6] * a[0] + R[7] * a[1] + R[8] * a[2];
      if (x * b[0] + y * b[1] + z * b[2] > cthr) idx.push(k);
    }
    if (idx.length < 6) break;
    bestIdx = idx; R = kabsch(A, B, idx);
  }
  return { R, inliers: bestIdx };
}
function camRay(cam, f, x, y) { const u = undistOff(cam, x - cam.cx, y - cam.cy); return normalize3([u[0] / f, u[1] / f, 1]); }

// кандидаты пар по пересечению кадров
function overlapFraction(ci, cj, f) {
  const H = pairHomography(ci, cj, f), o = [0, 0]; let inside = 0, tot = 0;
  for (let gy = 0; gy < 7; gy++) for (let gx = 0; gx < 7; gx++) {
    const x = (gx + 0.5) / 7 * ci.w, y = (gy + 0.5) / 7 * ci.h; tot++;
    hApply(H, x, y, o); if (o[0] === o[0] && o[0] > 0 && o[1] > 0 && o[0] < cj.w && o[1] < cj.h) inside++;
  }
  return inside / tot;
}

// сопоставление пары: грубо (консенсус сдвига) или точно (от текущих поз)
function matchPair(ci, cj, f, mode, rand, parallaxTol) {
  let perPoint = null;
  const PI = ci.pyr, PJ = cj.pyr, nLev = PI.levels.length;
  const H = pairHomography(ci, cj, f);
  const kfI = l => PI.k * (1 << l), kfJ = l => PJ.k * (1 << l);
  const HL = []; for (let l = 0; l < nLev; l++) HL.push(m3mul(m3mul(levelA(kfJ(l)), H), levelAinv(kfI(l))));
  const margin = 0.04 * Math.max(cj.w, cj.h), o = [0, 0];
  // углы i внутри предсказанной области перекрытия
  const pts = [];
  for (const c of ci.corners) { hApply(H, c[0], c[1], o); if (o[0] === o[0] && o[0] > margin && o[1] > margin && o[0] < cj.w - margin && o[1] < cj.h - margin) pts.push(c); }
  if (pts.length < 8) return { matches: [], shiftOK: false };
  const toL = (x, kf) => (x + 0.5) / kf - 0.5;
  let shift = [0, 0], startL, startR;
  const fL = l => f / kfI(l);
  if (mode === 'coarse') {
    // уровень, где кадр ~120–240 px
    let L = nLev - 1; while (L > 1 && Math.max(PI.levels[L].w, PI.levels[L].h) < (parallaxTol ? 200 : 95)) L--;
    const R = parallaxTol ? Math.max(6, Math.ceil(fL(L) * 6 * Math.PI / 180)) : Math.max(8, Math.ceil(fL(L) * 11 * Math.PI / 180)), t = parallaxTol ? 5 : 4;
    const offs = [];
    const use = pts.slice(0, 90);
    perPoint = new Map();
    for (const c of use) {
      const x = toL(c[0], kfI(L)), y = toL(c[1], kfI(L));
      const T = makeTemplate(PI.levels[L], x, y, t); if (!T) continue;
      const r = searchNCC(T, t, PJ.levels[L], HL[L], x, y, R); if (!r || r.score < (parallaxTol ? 0.5 : 0.6)) continue;
      offs.push([r.ox, r.oy, r.score]); perPoint.set(c, [r.ox * 2, r.oy * 2]);
    }
    if (offs.length < 4) return { matches: [], shiftOK: false };
    if (parallaxTol) { shift = [0, 0]; startL = L - 1; startR = 3; }
    else {
    // мода 2D-гистограммы
    const B = 2 * R + 1, hist = new Float32Array(B * B);
    for (const q of offs) { const bx = Math.round(q[0]) + R, by = Math.round(q[1]) + R; for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) { const X = bx + dx, Y = by + dy; if (X >= 0 && Y >= 0 && X < B && Y < B) hist[Y * B + X] += (dx || dy) ? 0.5 : 1; } }
    let bi = 0; for (let k = 1; k < hist.length; k++) if (hist[k] > hist[bi]) bi = k;
    const mx = (bi % B) - R, my = ((bi / B) | 0) - R;
    let sx = 0, sy = 0, cnt = 0;
    for (const q of offs) if (Math.abs(q[0] - mx) <= 1.6 && Math.abs(q[1] - my) <= 1.6) { sx += q[0]; sy += q[1]; cnt++; }
    if (cnt < 4 || cnt < 0.22 * offs.length) return { matches: [], shiftOK: false };
    shift = [sx / cnt * 2, sy / cnt * 2]; // в координатах уровня L-1
    startL = L - 1; startR = 6;
    }
  } else {
    startL = Math.min(1, nLev - 1); startR = 5;
  }
  const tFor = l => (l >= 2 ? 5 : l === 1 ? 6 : 8);
  const res = [];
  const use = perPoint && parallaxTol ? [...perPoint.keys()] : pts.slice(0, mode === 'coarse' ? 90 : 160);
  // с дисторсией гомография поворота неточна у краёв — добавляем поправку предсказания для каждой точки
  const lens = mode !== 'coarse' && (ci.k1 || cj.k1);
  const Rrel = lens ? m3mul(m3T(cj.C2W), ci.C2W) : null;
  for (const c of use) {
    let dx = shift[0], dy = shift[1], ok = true, score = 0, sJ = [0, 0];
    if (perPoint && parallaxTol) { const pp = perPoint.get(c); dx = pp[0]; dy = pp[1]; }
    if (lens) {
      const a = mv(Rrel, camRay(ci, f, c[0], c[1])); if (a[2] <= 1e-6) continue;
      const pd = distOff(cj, f * a[0] / a[2], f * a[1] / a[2]); hApply(H, c[0], c[1], o);
      sJ = [pd[0] + cj.cx - o[0], pd[1] + cj.cy - o[1]];
    }
    for (let l = startL; l >= 0; l--) {
      const x = toL(c[0], kfI(l)), y = toL(c[1], kfI(l)), t = tFor(l);
      const T = makeTemplate(PI.levels[l], x, y, t); if (!T) { ok = false; break; }
      const R = l === startL ? startR : 2;
      const r = searchNCC(T, t, PJ.levels[l], HL[l], x + dx, y + dy, R, sJ[0] / kfJ(l), sJ[1] / kfJ(l));
      if (!r || r.score < (l === 0 ? 0.78 : 0.55)) { ok = false; break; }
      dx += r.ox; dy += r.oy; score = r.score;
      if (l > 0) { dx *= 2; dy *= 2; }
    }
    if (!ok) continue;
    // смещение уровня 0 → полное разрешение
    const k0 = kfI(0), px = c[0] + dx * k0, py = c[1] + dy * k0;
    hApply(H, px, py, o); if (o[0] !== o[0]) continue;
    res.push({ xi: c[0], yi: c[1], xj: o[0] + sJ[0], yj: o[1] + sJ[1], s: score });
  }
  if (res.length < 8) return { matches: [], shiftOK: mode === 'coarse' };
  // RANSAC по модели чистого вращения
  const A = res.map(m => camRay(ci, f, m.xi, m.yi)), Bv = res.map(m => camRay(cj, f, m.xj, m.yj));
  const thr = parallaxTol ? (mode === 'coarse' ? 1.2 : 0.6) * Math.PI / 180 : (mode === 'coarse' ? 4.5 : 2.2) / f;
  let rr = ransacRotation(A, Bv, thr, 250, rand);
  // параллакс (съёмка с рук): единый поворот объясняет не все точки — расширяем порог, остальное решит связка
  if (mode !== 'coarse' && (!rr || rr.inliers.length < 0.55 * res.length)) { const r2 = ransacRotation(A, Bv, 8 / f, 250, rand); if (r2 && (!rr || r2.inliers.length > rr.inliers.length * 1.25)) rr = r2; }
  if (!rr || rr.inliers.length < (parallaxTol ? 8 : 10) || rr.inliers.length < 0.3 * res.length) return { matches: [], shiftOK: true };
  // согласие с гироскопом: относительный поворот не должен уходить далеко от приора
  const Rprior = m3mul(m3T(cj.C2Wprior), ci.C2Wprior);
  if (rotAngle(m3mul(rr.R, m3T(Rprior))) > (parallaxTol ? 8 : 20) * Math.PI / 180) return { matches: [], shiftOK: true };
  return { matches: rr.inliers.map(k => res[k]), shiftOK: true };
}

// ───────────────────────── калибровка фокуса по гироскопу ─────────────────────────
// Повороты между кадрами гироскоп знает хорошо; ошибка фокуса проявляется как расхождение перекрытий,
// растущее от центра. Перебираем фокус и берём тот, при котором соседние кадры совпадают лучше всего
// (корреляция градиентов в общей области, кадры уменьшены до ~240 px).
function calibrateFocal(cams, f0, report) {
  const lev = c => { const L = c.pyr.levels; let k = 0; while (k < L.length - 1 && Math.max(L[k].w, L[k].h) > 260) k++; return { img: L[k], kf: c.pyr.k * (1 << k) }; };
  const grads = cams.map(c => {
    const { img, kf } = lev(c), w = img.w, h = img.h, d = img.d, gx = new Float32Array(w * h), gy = new Float32Array(w * h);
    for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) { const p = y * w + x; gx[p] = d[p + 1] - d[p - 1]; gy[p] = d[p + w] - d[p - w]; }
    return { w, h, gx, gy, kf };
  });
  // пары: соседние по направлению, с заметным перекрытием
  const pairs = [];
  for (let i = 0; i < cams.length; i++) for (let j = i + 1; j < cams.length; j++) {
    const a = camForward(cams[i].C2W), b = camForward(cams[j].C2W), cs = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const ang = Math.acos(Math.max(-1, Math.min(1, cs))) * 180 / Math.PI;
    if (ang > 18 && ang < 50) pairs.push({ i, j, ang });
  }
  pairs.sort((p, q) => q.ang - p.ang); // большие углы чувствительнее к фокусу
  const use = pairs.slice(0, 40);
  const score = (fs) => {
    let tot = 0, cnt = 0;
    for (const p of use) {
      const A = grads[p.i], B = grads[p.j], ca = cams[p.i], cb = cams[p.j];
      const fa = fs / A.kf, fb = fs / B.kf, cxa = (A.w - 1) / 2, cya = (A.h - 1) / 2, cxb = (B.w - 1) / 2, cyb = (B.h - 1) / 2;
      const R = m3mul(m3T(cb.C2W), ca.C2W);
      let sab = 0, saa = 0, sbb = 0, n = 0;
      for (let y = 4; y < A.h - 4; y += 2) for (let x = 4; x < A.w - 4; x += 2) {
        const pa = y * A.w + x, ga = Math.abs(A.gx[pa]) + Math.abs(A.gy[pa]); if (ga < 0.02) continue;
        const vx = (x - cxa) / fa, vy = (y - cya) / fa;
        const X = R[0] * vx + R[1] * vy + R[2], Y = R[3] * vx + R[4] * vy + R[5], Z = R[6] * vx + R[7] * vy + R[8];
        if (Z <= 0.1) continue;
        const xb = fb * X / Z + cxb, yb = fb * Y / Z + cyb;
        if (xb < 2 || yb < 2 || xb > B.w - 3 || yb > B.h - 3) continue;
        const pb = (yb | 0) * B.w + (xb | 0);
        // градиент в B, повёрнутый в систему A (поворот плоскости изображения мал — достаточно скаляра модуля и знака проекций)
        const ax = A.gx[pa], ay = A.gy[pa], bx = B.gx[pb], by = B.gy[pb];
        sab += ax * bx + ay * by; saa += ax * ax + ay * ay; sbb += bx * bx + by * by; n++;
      }
      if (n > 80 && saa > 0 && sbb > 0) { tot += sab / Math.sqrt(saa * sbb); cnt++; }
    }
    return cnt ? tot / cnt : -1;
  };
  // скан по абсолютной шкале (доля длинной стороны кадра), независимо от стартового f0:
  // оценка по гироскопу шумная, поэтому берём максимум сглаженной кривой, а не отдельный пик
  const L = Math.max(cams[0].w, cams[0].h), lo = 0.66, hi = 1.0, st = 0.015, scan = [];
  const nS = Math.round((hi - lo) / st) + 1;
  for (let k = 0; k < nS; k++) { const q = lo + k * st; scan.push([q * L / f0, score(q * L)]); if (report) report((k + 1) / nS); }
  let best = f0, bs = -2;
  for (let k = 1; k < nS - 1; k++) {
    const v = (scan[k - 1][1] + 2 * scan[k][1] + scan[k + 1][1]) / 4;
    if (v > bs) { bs = v; best = scan[k][0] * f0; }
  }
  return { f: best, score: bs, scan, pairs: use.length };
}

// ───────────────────────── параллакс в реальном времени (подсказки при съёмке) ─────────────────────────
// live, ref: { w, h, g: Float32Array (яркость 0..1) }, C2W живого и опорного, f — фокус в пикселях этих картинок.
// Возвращает медианный сдвиг (ошибка гироскопа) и разброс сдвигов (параллакс) в градусах.
function measureParallax(live, ref, C2Wl, C2Wr, f, opts) {
  opts = opts || {};
  const R = m3mul(m3T(C2Wr), C2Wl); // луч живой камеры → опорная
  const cxl = (live.w - 1) / 2, cyl = (live.h - 1) / 2, cxr = (ref.w - 1) / 2, cyr = (ref.h - 1) / 2;
  const T = opts.t || 7, S = opts.r || 14, GX = opts.gx || 5, GY = opts.gy || 6, n = 2 * T + 1, N = n * n;
  const mapRef = (x, y, out) => {
    const vx = (x - cxl) / f, vy = (y - cyl) / f, X = R[0] * vx + R[1] * vy + R[2], Y = R[3] * vx + R[4] * vy + R[5], Z = R[6] * vx + R[7] * vy + R[8];
    if (Z <= 0.05) return false; out[0] = f * X / Z + cxr; out[1] = f * Y / Z + cyr; return true;
  };
  const smp = (img, x, y) => {
    if (x < 0 || y < 0 || x > img.w - 1.001 || y > img.h - 1.001) return NaN;
    const x0 = x | 0, y0 = y | 0, fx = x - x0, fy = y - y0, p = y0 * img.w + x0, g = img.g;
    return (g[p] * (1 - fx) + g[p + 1] * fx) * (1 - fy) + (g[p + img.w] * (1 - fx) + g[p + img.w + 1] * fx) * fy;
  };
  const vecs = [], o = [0, 0], Tl = new Float32Array(N), Wr = new Float32Array((2 * (T + S) + 1) * (2 * (T + S) + 1));
  for (let gy = 0; gy < GY; gy++) for (let gx = 0; gx < GX; gx++) {
    const bx = (gx + 0.5) / GX * live.w, by = (gy + 0.5) / GY * live.h;
    if (bx < T + 1 || by < T + 1 || bx > live.w - T - 2 || by > live.h - T - 2) continue;
    if (!mapRef(bx, by, o)) continue;
    const rx = o[0], ry = o[1];
    if (rx < T + 2 || ry < T + 2 || rx > ref.w - T - 3 || ry > ref.h - T - 3) continue;
    // шаблон из живого кадра
    let m = 0, v = 0;
    for (let y = -T, q = 0; y <= T; y++) for (let x = -T; x <= T; x++, q++) { const val = smp(live, bx + x, by + y); Tl[q] = val; m += val; v += val * val; }
    m /= N; v -= N * m * m; if (v < N * 0.0004) continue; // нет текстуры
    const iv = 1 / Math.sqrt(v); for (let q = 0; q < N; q++) Tl[q] = (Tl[q] - m) * iv;
    // окно опорного кадра, перенесённое в геометрию живого кадра (учитывает поворот изображения между кадрами)
    const sn = 2 * (T + S) + 1;
    for (let y = 0, q = 0; y < sn; y++) for (let x = 0; x < sn; x++, q++) Wr[q] = mapRef(bx - T - S + x, by - T - S + y, o) ? smp(ref, o[0], o[1]) : NaN;
    let best = -2, bxs = 0, bys = 0; const sc = new Float32Array((2 * S + 1) * (2 * S + 1));
    for (let sy = 0; sy <= 2 * S; sy++) for (let sx = 0; sx <= 2 * S; sx++) {
      let s1 = 0, s2 = 0, d = 0, ok = true;
      for (let y = 0; y < n && ok; y++) { const row = (sy + y) * sn + sx, tr = y * n; for (let x = 0; x < n; x++) { const w = Wr[row + x]; if (w !== w) { ok = false; break; } s1 += w; s2 += w * w; d += Tl[tr + x] * w; } }
      let c = -2; if (ok) { const vv = s2 - s1 * s1 / N; c = vv > 1e-8 ? d / Math.sqrt(vv) : -2; }
      sc[sy * (2 * S + 1) + sx] = c; if (c > best) { best = c; bxs = sx; bys = sy; }
    }
    if (best < (opts.minScore || 0.75)) continue;
    let second = -2; for (let sy = 0; sy <= 2 * S; sy++) for (let sx = 0; sx <= 2 * S; sx++) if (Math.abs(sx - bxs) > 2 || Math.abs(sy - bys) > 2) second = Math.max(second, sc[sy * (2 * S + 1) + sx]);
    if (second > best - 0.05) continue;
    vecs.push([bxs - S, bys - S, best, bx, by]);
  }
  if (vecs.length < 4) return { ok: false, n: vecs.length };
  const xs = vecs.map(v => v[0]).sort((a, b) => a - b), ys = vecs.map(v => v[1]).sort((a, b) => a - b);
  const mx = xs[xs.length >> 1], my = ys[ys.length >> 1];
  // поворот плоскости кадра (ошибка гироскопа по крену) — это не параллакс, вычитаем его
  const ths = [];
  for (const v of vecs) { const rx = v[3] - cxl, ry = v[4] - cyl, r2 = rx * rx + ry * ry; if (r2 > 0.0225 * live.w * live.w) ths.push((rx * (v[1] - my) - ry * (v[0] - mx)) / r2); }
  ths.sort((a, b) => a - b);
  const th = ths.length >= 3 ? ths[ths.length >> 1] : 0;
  const res = vecs.map(v => Math.hypot(v[0] - mx + th * (v[4] - cyl), v[1] - my - th * (v[3] - cxl))).sort((a, b) => a - b);
  // разброс: 80-й перцентиль отклонений (до 1 px — это просто точность измерения)
  const spread = res[Math.min(res.length - 1, Math.floor(res.length * 0.8))];
  const toDeg = 180 / Math.PI / f;
  // tx, ty, th: поправка к гироскопу — где искать живую картинку в опорном кадре: x + t + th·(−(y−cy), x−cx)
  return { ok: true, n: vecs.length, gyroErr: Math.hypot(mx, my) * toDeg, parallax: Math.max(0, spread - 0.7) * toDeg, vecs, tx: mx, ty: my, th };
}

// ───────────────────────── связка (bundle adjustment по лучам) ─────────────────────────
function cholSolve(A, b, n) {
  const L = new Float64Array(n * n);
  for (let i = 0; i < n; i++) for (let j = 0; j <= i; j++) {
    let s = A[i * n + j]; for (let k = 0; k < j; k++) s -= L[i * n + k] * L[j * n + k];
    if (i === j) { if (s <= 0) return null; L[i * n + i] = Math.sqrt(s); } else L[i * n + j] = s / L[j * n + j];
  }
  const y = new Float64Array(n);
  for (let i = 0; i < n; i++) { let s = b[i]; for (let k = 0; k < i; k++) s -= L[i * n + k] * y[k]; y[i] = s / L[i * n + i]; }
  const x = new Float64Array(n);
  for (let i = n - 1; i >= 0; i--) { let s = y[i]; for (let k = i + 1; k < n; k++) s -= L[k * n + i] * x[k]; x[i] = s / L[i * n + i]; }
  return x;
}
// Связка поворотов: невязки лучей в пикселях, параметры — повороты (кадр 0 закреплён), log f и дисторсия k1.
// Дисторсия: смещение от центра x_u = x_d·(1 + k1·r²/R0²), R0 — полудиагональ кадра.
function bundleAdjust(cams, M, f0, iters, k1init, pri) {
  pri = pri || {};
  const N = cams.length, P = 3 * (N - 1) + 2, iphi = P - 2, ik1 = P - 1, R0 = cams[0].R0;
  const pIdx = c => (c === 0 ? -1 : 3 * (c - 1));
  const sigR = pri.sigR || 0.35, sigF = pri.sigF || 0.25, sigK = pri.sigK || 0.15; // приоры: гироскоп ±20°, фокус ±25%, дисторсия ±0.15
  const sigRel = pri.sigRel || 0; // относительный поворот соседних кадров по гироскопу
  const relG = []; for (let c = 0; c + 1 < N; c++) relG.push(m3mul(m3T(cams[c].C2Wprior), cams[c + 1].C2Wprior));
  const relRes = (Cs, c) => logSO3(m3mul(m3mul(m3T(Cs[c]), Cs[c + 1]), m3T(relG[c])));
  let phi = 0, k1 = k1init || 0;
  const MW = M.w, MH = M.hub; // вес и порог Хьюбера на каждое совпадение (если заданы)
  const sigUp = pri.sigUp || 0, upG = sigUp ? cams.map(c => [c.C2Wprior[6], c.C2Wprior[7], c.C2Wprior[8]]) : null; // «верх» мира в системе камеры по гироскопу
  const upRes = (C, c) => { const g = upG[c]; return [C[0] * g[0] + C[1] * g[1] + C[2] * g[2], C[3] * g[0] + C[4] * g[1] + C[5] * g[2]]; }; // x,y мирового образа (должны быть 0)
  const cost = (C2Ws, ph, kk) => {
    const f = f0 * Math.exp(ph); let E = 0;
    for (let k = 0; k < M.n; k++) {
      const huber = MH ? MH[k] : 2, wk = MW ? MW[k] : 1;
      const wi = mv(C2Ws[M.i[k]], rayUK(M.xi[k], M.yi[k], f, kk, R0).u), wj = mv(C2Ws[M.j[k]], rayUK(M.xj[k], M.yj[k], f, kk, R0).u);
      const e = Math.hypot(wi[0] - wj[0], wi[1] - wj[1], wi[2] - wj[2]) * f;
      E += wk * (e <= huber ? e * e : 2 * huber * e - huber * huber);
    }
    if (sigUp) for (let c = 0; c < N; c++) { const r = upRes(C2Ws[c], c); E += (r[0] * r[0] + r[1] * r[1]) / (sigUp * sigUp); }
    for (let c = 1; c < N; c++) { const l = logSO3(m3mul(C2Ws[c], m3T(cams[c].C2Wprior))); E += (l[0] * l[0] + l[1] * l[1] + l[2] * l[2]) / (sigR * sigR); }
    if (sigRel) for (let c = 0; c + 1 < N; c++) { if (cams[c].group !== cams[c + 1].group) continue; const r = relRes(C2Ws, c), e = Math.hypot(r[0], r[1], r[2]) / sigRel; E += e * e; }
    return E + ph * ph / (sigF * sigF) + kk * kk / (sigK * sigK);
  };
  let C2W = cams.map(c => m3copy(c.C2W));
  let E = cost(C2W, phi, k1), lambda = 1e-3;
  for (let it = 0; it < iters; it++) {
    const A = new Float64Array(P * P), b = new Float64Array(P);
    const f = f0 * Math.exp(phi), S = f; // невязки в пикселях текущего фокуса
    for (let k = 0; k < M.n; k++) {
      const ci = M.i[k], cj = M.j[k], huber = MH ? MH[k] : 2, wk = MW ? MW[k] : 1;
      const ui = rayUK(M.xi[k], M.yi[k], f, k1, R0), uj = rayUK(M.xj[k], M.yj[k], f, k1, R0);
      const Ri = C2W[ci], Rj = C2W[cj];
      const wi = mv(Ri, ui.u), wj = mv(Rj, uj.u);
      const r = [(wi[0] - wj[0]) * S, (wi[1] - wj[1]) * S, (wi[2] - wj[2]) * S];
      const e = Math.hypot(r[0], r[1], r[2]); const wt = wk * (e <= huber ? 1 : huber / e);
      const dwi = mv(Ri, ui.du), dwj = mv(Rj, uj.du), dki = mv(Ri, ui.dk), dkj = mv(Rj, uj.dk);
      // d(f·Δw)/d log f = f·Δw + f·dΔw/d log f
      const Jf = [(dwi[0] - dwj[0]) * S + r[0], (dwi[1] - dwj[1]) * S + r[1], (dwi[2] - dwj[2]) * S + r[2]];
      const Jk = [(dki[0] - dkj[0]) * S, (dki[1] - dkj[1]) * S, (dki[2] - dkj[2]) * S];
      // Ji = -[wi]x, Jj = +[wj]x (в пикселях)
      const Ji = skew(wi, -S), Jj = skew(wj, S);
      const blocks = []; if (ci !== 0) blocks.push([pIdx(ci), Ji]); if (cj !== 0) blocks.push([pIdx(cj), Jj]);
      for (const [pa, Ja] of blocks) {
        for (let a = 0; a < 3; a++) {
          let g = 0; for (let q = 0; q < 3; q++) g += Ja[q * 3 + a] * r[q]; b[pa + a] += wt * g;
          let gf = 0, gk = 0; for (let q = 0; q < 3; q++) { gf += Ja[q * 3 + a] * Jf[q]; gk += Ja[q * 3 + a] * Jk[q]; }
          A[(pa + a) * P + iphi] += wt * gf; A[iphi * P + pa + a] += wt * gf;
          A[(pa + a) * P + ik1] += wt * gk; A[ik1 * P + pa + a] += wt * gk;
          for (const [pb, Jb] of blocks) for (let c = 0; c < 3; c++) {
            let s = 0; for (let q = 0; q < 3; q++) s += Ja[q * 3 + a] * Jb[q * 3 + c];
            A[(pa + a) * P + pb + c] += wt * s;
          }
        }
      }
      const ff = Jf[0] * Jf[0] + Jf[1] * Jf[1] + Jf[2] * Jf[2], kk = Jk[0] * Jk[0] + Jk[1] * Jk[1] + Jk[2] * Jk[2], fk = Jf[0] * Jk[0] + Jf[1] * Jk[1] + Jf[2] * Jk[2];
      b[iphi] += wt * (Jf[0] * r[0] + Jf[1] * r[1] + Jf[2] * r[2]); b[ik1] += wt * (Jk[0] * r[0] + Jk[1] * r[1] + Jk[2] * r[2]);
      A[iphi * P + iphi] += wt * ff; A[ik1 * P + ik1] += wt * kk; A[iphi * P + ik1] += wt * fk; A[ik1 * P + iphi] += wt * fk;
    }
    for (let c = 1; c < N; c++) {
      const l = logSO3(m3mul(C2W[c], m3T(cams[c].C2Wprior))), p = pIdx(c);
      for (let a = 0; a < 3; a++) { A[(p + a) * P + p + a] += 1 / (sigR * sigR); b[p + a] += l[a] / (sigR * sigR); }
    }
    A[iphi * P + iphi] += 1 / (sigF * sigF); b[iphi] += phi / (sigF * sigF);
    A[ik1 * P + ik1] += 1 / (sigK * sigK); b[ik1] += k1 / (sigK * sigK);
    if (sigUp) for (let c = 1; c < N; c++) {
      // r' = r + (δy, −δx): наклон по гравитации
      const r = upRes(C2W[c], c), p = pIdx(c), w = 1 / (sigUp * sigUp);
      b[p + 1] += w * r[0]; b[p] -= w * r[1]; A[(p + 1) * P + p + 1] += w; A[p * P + p] += w;
    }
    if (sigRel) for (let c = 0; c + 1 < N; c++) {
      if (cams[c].group !== cams[c + 1].group) continue; // цепочка — только внутри одной панорамы/прохода
      const r = relRes(C2W, c), w = 1 / (sigRel * sigRel), Ck = C2W[c];
      // r_new ≈ r + C_kᵀ(δ_{k+1} − δ_k): Якобианы J_{k+1} = C_kᵀ, J_k = −C_kᵀ; Jᵀ = ±C_k
      const pk = pIdx(c), pk1 = pIdx(c + 1);
      const Cr = [Ck[0] * r[0] + Ck[1] * r[1] + Ck[2] * r[2], Ck[3] * r[0] + Ck[4] * r[1] + Ck[5] * r[2], Ck[6] * r[0] + Ck[7] * r[1] + Ck[8] * r[2]];
      for (let a = 0; a < 3; a++) {
        if (pk1 >= 0) { b[pk1 + a] += w * Cr[a]; A[(pk1 + a) * P + pk1 + a] += w; }
        if (pk >= 0) { b[pk + a] -= w * Cr[a]; A[(pk + a) * P + pk + a] += w; }
        if (pk >= 0 && pk1 >= 0) { A[(pk + a) * P + pk1 + a] -= w; A[(pk1 + a) * P + pk + a] -= w; }
      }
    }
    let improved = false;
    for (let tries = 0; tries < 8 && !improved; tries++) {
      const Ad = Float64Array.from(A);
      for (let q = 0; q < P; q++) Ad[q * P + q] += lambda * A[q * P + q] + 1e-12;
      const nb = b.map(v => -v);
      const dx = cholSolve(Ad, nb, P); if (!dx) { lambda *= 10; continue; }
      const C2Wn = C2W.map((R, c) => { if (c === 0) return R; const p = pIdx(c); return m3mul(expSO3(dx[p], dx[p + 1], dx[p + 2]), R); });
      const phin = phi + dx[iphi], k1n = k1 + dx[ik1];
      const En = cost(C2Wn, phin, k1n);
      if (En < E) { C2W = C2Wn; phi = phin; k1 = k1n; const rel = (E - En) / E; E = En; lambda = Math.max(1e-7, lambda / 3); improved = true; if (rel < 1e-7) it = iters; }
      else lambda *= 4;
    }
    if (!improved) break;
  }
  // итоговая ошибка в пикселях
  const f = f0 * Math.exp(phi); let s2 = 0;
  const per = new Float64Array(M.n);
  for (let k = 0; k < M.n; k++) {
    const wi = mv(C2W[M.i[k]], rayUK(M.xi[k], M.yi[k], f, k1, R0).u), wj = mv(C2W[M.j[k]], rayUK(M.xj[k], M.yj[k], f, k1, R0).u);
    const e = Math.hypot(wi[0] - wj[0], wi[1] - wj[1], wi[2] - wj[2]) * f; per[k] = e; s2 += e * e;
  }
  return { C2W, f, k1, rms: M.n ? Math.sqrt(s2 / M.n) : 0, per };
}
// луч по смещению от центра (с учётом дисторсии) и производные по log f и k1
function rayUK(x, y, f, k1, R0) {
  const rr = (x * x + y * y) / (R0 * R0), sc = 1 + k1 * rr, xu = x * sc, yu = y * sc;
  const vx = xu / f, vy = yu / f, n = Math.sqrt(vx * vx + vy * vy + 1);
  const u = [vx / n, vy / n, 1 / n];
  const dv = [-vx, -vy, 0]; const ud = u[0] * dv[0] + u[1] * dv[1];
  const dk = [x * rr / f, y * rr / f, 0]; const uk = u[0] * dk[0] + u[1] * dk[1];
  return { u, du: [(dv[0] - u[0] * ud) / n, (dv[1] - u[1] * ud) / n, (dv[2] - u[2] * ud) / n], dk: [(dk[0] - u[0] * uk) / n, (dk[1] - u[1] * uk) / n, (dk[2] - u[2] * uk) / n] };
}
// смещение от центра: искажённое → неискажённое и обратно
function undistOff(cam, x, y) { const k1 = cam.k1 || 0; if (!k1) return [x, y]; const s = 1 + k1 * (x * x + y * y) / (cam.R0 * cam.R0); return [x * s, y * s]; }
function distOff(cam, xu, yu) {
  const k1 = cam.k1 || 0; if (!k1) return [xu, yu];
  if (Math.abs(xu) > 1.5 * cam.cx || Math.abs(yu) > 1.5 * cam.cy) return [xu * 10, yu * 10]; // заведомо вне кадра
  let x = xu, y = yu; const iR = 1 / (cam.R0 * cam.R0);
  for (let it = 0; it < 5; it++) { const s = 1 + k1 * (x * x + y * y) * iR; x = xu / s; y = yu / s; }
  return [x, y];
}
function rayU(x, y, f) {
  const vx = x / f, vy = y / f, n = Math.sqrt(vx * vx + vy * vy + 1);
  const u = [vx / n, vy / n, 1 / n];
  const dv = [-vx, -vy, 0]; const ud = u[0] * dv[0] + u[1] * dv[1];
  return { u, du: [(dv[0] - u[0] * ud) / n, (dv[1] - u[1] * ud) / n, (dv[2] - u[2] * ud) / n] };
}
function rayW(R, x, y, f) { const u = rayU(x, y, f).u; return mv(R, u); }
function mv(R, v) { return [R[0] * v[0] + R[1] * v[1] + R[2] * v[2], R[3] * v[0] + R[4] * v[1] + R[5] * v[2], R[6] * v[0] + R[7] * v[1] + R[8] * v[2]]; }
function skew(w, s) { return Float64Array.of(0, -s * w[2], s * w[1], s * w[2], 0, -s * w[0], -s * w[1], s * w[0], 0); }

// ───────────────────────── «бегущий затвор» (rolling shutter) ─────────────────────────
// Камера телефона читает кадр построчно ~20–30 мс. Если телефон поворачивается (съёмка проходами),
// каждая строка снята в свой момент: картинка растянута/скошена, и прямые края на швах ломаются.
// Модель: строка/столбец p снят в момент t = τ·(p − c) (τ — сек на пиксель по x и y: направление
// считывания зависит от того, как держат телефон), поворот за это время — exp([ω]·t), ω — угловая
// скорость в системе камеры (по гироскопу в момент снимка). τ общий для всех кадров и оценивается по совпадениям.
function rsCorrectPoint(cam, f, x, y, tau) {
  const om = cam.omega; if (!om) return [x, y];
  const dx = x - cam.cx, dy = y - cam.cy, t = tau[0] * dx + tau[1] * dy;
  if (!t) return [x, y];
  const u0 = undistOff(cam, dx, dy), u = [u0[0] / f, u0[1] / f, 1];
  const v = mv(expSO3(om[0] * t, om[1] * t, om[2] * t), u);
  const q = distOff(cam, f * v[0] / v[2], f * v[1] / v[2]);
  return [q[0] + cam.cx, q[1] + cam.cy];
}
function estimateRS(cams, f, pairs, tau) {
  // линейная задача на поправку δτ при текущих позах: e(τ+δ) ≈ e + δx·gx + δy·gy
  let A00 = 0, A01 = 0, A11 = 0, b0 = 0, b1 = 0;
  const cr = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  for (const p of pairs) {
    const ci = cams[p.i], cj = cams[p.j]; if (!ci.omega && !cj.omega) continue;
    for (const m of p.matches) {
      const a = rsCorrectPoint(ci, f, m.oxi, m.oyi, tau), b = rsCorrectPoint(cj, f, m.oxj, m.oyj, tau);
      const ui = camRay(ci, f, a[0], a[1]), uj = camRay(cj, f, b[0], b[1]);
      const wi = mv(ci.C2W, ui), wj = mv(cj.C2W, uj);
      const e = [(wi[0] - wj[0]) * f, (wi[1] - wj[1]) * f, (wi[2] - wj[2]) * f];
      const ai = ci.omega ? mv(ci.C2W, cr(ci.omega, ui)) : [0, 0, 0], aj = cj.omega ? mv(cj.C2W, cr(cj.omega, uj)) : [0, 0, 0];
      const dix = m.oxi - ci.cx, diy = m.oyi - ci.cy, djx = m.oxj - cj.cx, djy = m.oyj - cj.cy;
      const gx = [0, 1, 2].map(k => (dix * ai[k] - djx * aj[k]) * f), gy = [0, 1, 2].map(k => (diy * ai[k] - djy * aj[k]) * f);
      const en = Math.hypot(e[0], e[1], e[2]), w = en < 3 ? 1 : 3 / en;
      for (let k = 0; k < 3; k++) { A00 += w * gx[k] * gx[k]; A01 += w * gx[k] * gy[k]; A11 += w * gy[k] * gy[k]; b0 += w * gx[k] * e[k]; b1 += w * gy[k] * e[k]; }
    }
  }
  const pr = 1 / (2e-5 * 2e-5); A00 += pr; A11 += pr; b0 += pr * tau[0]; b1 += pr * tau[1];
  const det = A00 * A11 - A01 * A01; if (!(det > 0)) return tau.slice();
  const d0 = -(A11 * b0 - A01 * b1) / det, d1 = -(A00 * b1 - A01 * b0) / det;
  const lim = 4e-5; return [Math.max(-lim, Math.min(lim, tau[0] + d0)), Math.max(-lim, Math.min(lim, tau[1] + d1))];
}
// переснимаем кадр «глобальным затвором»: пиксель p берём из точки q, для которой rsCorrectPoint(q) = p
function rsWarp(cam, f, tau) {
  if (!cam.omega) return;
  const w = cam.w, h = cam.h, G = 24, gw = Math.ceil(w / G) + 1, gh = Math.ceil(h / G) + 1, dx = new Float32Array(gw * gh), dy = new Float32Array(gw * gh);
  for (let gy = 0; gy < gh; gy++) for (let gx = 0; gx < gw; gx++) {
    const px = Math.min(w - 1, gx * G), py = Math.min(h - 1, gy * G);
    let qx = px, qy = py;
    for (let it = 0; it < 3; it++) { const c = rsCorrectPoint(cam, f, qx, qy, tau); qx -= c[0] - px; qy -= c[1] - py; }
    dx[gy * gw + gx] = qx - px; dy[gy * gw + gx] = qy - py;
  }
  const src = cam.rgb, out = new Uint8Array(src.length);
  for (let y = 0; y < h; y++) {
    const fy = y / G, y0 = Math.min(gh - 2, fy | 0), ty = fy - y0;
    for (let x = 0; x < w; x++) {
      const fx = x / G, x0 = Math.min(gw - 2, fx | 0), tx = fx - x0, g = y0 * gw + x0;
      const ox = (dx[g] * (1 - tx) + dx[g + 1] * tx) * (1 - ty) + (dx[g + gw] * (1 - tx) + dx[g + gw + 1] * tx) * ty;
      const oy = (dy[g] * (1 - tx) + dy[g + 1] * tx) * (1 - ty) + (dy[g + gw] * (1 - tx) + dy[g + gw + 1] * tx) * ty;
      let sx = x + ox, sy = y + oy; if (sx < 0) sx = 0; if (sy < 0) sy = 0; if (sx > w - 1.001) sx = w - 1.001; if (sy > h - 1.001) sy = h - 1.001;
      const xi = sx | 0, yi = sy | 0, ax = sx - xi, ay = sy - yi, p = (yi * w + xi) * 3, q = p + w * 3, o = (y * w + x) * 3;
      for (let c = 0; c < 3; c++) out[o + c] = (src[p + c] * (1 - ax) + src[p + 3 + c] * ax) * (1 - ay) + (src[q + c] * (1 - ax) + src[q + 3 + c] * ax) * ay + 0.5;
    }
  }
  cam.rgb = out;
}

// ───────────────────────── связка с параллаксом ─────────────────────────
// Телефон в руках движется по дуге вокруг тела: центр кадра c_i = (Fx, Fy, γ·Fz) + δc_i (в единицах «длины руки»),
// где F — направление съёмки. Для каждой пары совпадений неизвестна обратная глубина q = exp(s) (исключается дополнением Шура).
// Невязка: направление на точку из камеры j против наблюдаемого луча w_j, в пикселях.
function bundleAdjustParallax(cams, M, f0, iters, report) {
  const N = cams.length, nR = 3 * (N - 1), oC = nR, iPhi = oC + 3 * N, iGam = iPhi + 1, P = iGam + 1;
  const rIdx = c => (c === 0 ? -1 : 3 * (c - 1)), cIdx = c => oC + 3 * c;
  const sigR = 0.35, sigC = 0.25, sigF = 0.25, sigG = 0.6, sigS = 3, s0 = Math.log(0.15), HUB = 2;
  let C2W = cams.map(c => m3copy(c.C2W)), dC = cams.map(() => [0, 0, 0]), phi = 0, gam = 0.5;
  const s = new Float64Array(M.n);
  const center = (R, d, g) => [R[2] + d[0], R[5] + d[1], g * R[8] + d[2]];
  // начальные глубины: одномерная триангуляция
  {
    const f = f0;
    for (let k = 0; k < M.n; k++) {
      const i = M.i[k], j = M.j[k];
      const wi = rayW(C2W[i], M.xi[k], M.yi[k], f), wj = rayW(C2W[j], M.xj[k], M.yj[k], f);
      const ci = center(C2W[i], dC[i], gam), cj = center(C2W[j], dC[j], gam), d = [ci[0] - cj[0], ci[1] - cj[1], ci[2] - cj[2]];
      const a = cross(wi, wj), b = cross(d, wj), bb = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
      let q = bb > 1e-12 ? -(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / bb : 0.15;
      q = Math.max(0.01, Math.min(3, q));
      s[k] = Math.log(q);
    }
  }
  const resid = (Rs, Ds, ph, g, sv, k, out) => {
    const f = f0 * Math.exp(ph), i = M.i[k], j = M.j[k];
    const wi = rayW(Rs[i], M.xi[k], M.yi[k], f), wj = rayW(Rs[j], M.xj[k], M.yj[k], f);
    const ci = center(Rs[i], Ds[i], g), cj = center(Rs[j], Ds[j], g), q = Math.exp(sv[k]);
    const v = [wi[0] + q * (ci[0] - cj[0]), wi[1] + q * (ci[1] - cj[1]), wi[2] + q * (ci[2] - cj[2])], n = Math.hypot(v[0], v[1], v[2]);
    out[0] = (v[0] / n - wj[0]) * f0; out[1] = (v[1] / n - wj[1]) * f0; out[2] = (v[2] / n - wj[2]) * f0;
  };
  const r3 = [0, 0, 0];
  const cost = (Rs, Ds, ph, g, sv) => {
    let E = 0;
    for (let k = 0; k < M.n; k++) { resid(Rs, Ds, ph, g, sv, k, r3); const e = Math.hypot(r3[0], r3[1], r3[2]); E += e <= HUB ? e * e : 2 * HUB * e - HUB * HUB; E += (sv[k] - s0) * (sv[k] - s0) / (sigS * sigS); }
    for (let c = 1; c < N; c++) { const l = logSO3(m3mul(Rs[c], m3T(cams[c].C2Wprior))); E += (l[0] * l[0] + l[1] * l[1] + l[2] * l[2]) / (sigR * sigR); }
    for (let c = 0; c < N; c++) E += (Ds[c][0] ** 2 + Ds[c][1] ** 2 + Ds[c][2] ** 2) / (sigC * sigC);
    return E + ph * ph / (sigF * sigF) + (g - 0.5) * (g - 0.5) / (sigG * sigG);
  };
  let E = cost(C2W, dC, phi, gam, s), lambda = 1e-3;
  const idx = new Int32Array(14), J = new Float64Array(3 * 14);
  for (let it = 0; it < iters; it++) {
    const A = new Float64Array(P * P), b = new Float64Array(P);
    const aqq = new Float64Array(M.n), bq = new Float64Array(M.n), Axq = [];
    const f = f0 * Math.exp(phi);
    for (let k = 0; k < M.n; k++) {
      const i = M.i[k], j = M.j[k], Ri = C2W[i], Rj = C2W[j];
      const ui = rayU(M.xi[k], M.yi[k], f), uj = rayU(M.xj[k], M.yj[k], f);
      const wi = mv(Ri, ui.u), wj = mv(Rj, uj.u);
      const Fi = [Ri[2], Ri[5], Ri[8]], Fj = [Rj[2], Rj[5], Rj[8]];
      const ci = center(Ri, dC[i], gam), cj = center(Rj, dC[j], gam), q = Math.exp(s[k]);
      const dcv = [ci[0] - cj[0], ci[1] - cj[1], ci[2] - cj[2]];
      const v = [wi[0] + q * dcv[0], wi[1] + q * dcv[1], wi[2] + q * dcv[2]], nv = Math.hypot(v[0], v[1], v[2]), vh = [v[0] / nv, v[1] / nv, v[2] / nv];
      const r = [(vh[0] - wj[0]) * f0, (vh[1] - wj[1]) * f0, (vh[2] - wj[2]) * f0];
      const e = Math.hypot(r[0], r[1], r[2]), wt = e <= HUB ? 1 : HUB / e;
      // Pv = (I - vh vhᵀ)/|v| · f0
      const Pv = new Float64Array(9); for (let a = 0; a < 3; a++) for (let c = 0; c < 3; c++) Pv[a * 3 + c] = ((a === c ? 1 : 0) - vh[a] * vh[c]) / nv * f0;
      const mulP = (x) => [Pv[0] * x[0] + Pv[1] * x[1] + Pv[2] * x[2], Pv[3] * x[0] + Pv[4] * x[1] + Pv[5] * x[2], Pv[6] * x[0] + Pv[7] * x[1] + Pv[8] * x[2]];
      let nc = 0; J.fill(0);
      const col = (pi, vec) => { idx[nc] = pi; J[nc] = vec[0]; J[14 + nc] = vec[1]; J[28 + nc] = vec[2]; nc++; };
      // вращение i: dv = -[wi]x δ + q Γ(-[Fi]x) δ  → столбцы для δ = e_a
      for (let a = 0; a < 3; a++) {
        const ea = [0, 0, 0]; ea[a] = 1;
        if (i !== 0) { const t1 = cross(ea, wi), t2 = cross(ea, Fi); col(rIdx(i) + a, mulP([t1[0] + q * t2[0], t1[1] + q * t2[1], t1[2] + q * gam * t2[2]])); }
        if (j !== 0) { const t2 = cross(ea, Fj), t3 = cross(ea, wj); const pv = mulP([-q * t2[0], -q * t2[1], -q * gam * t2[2]]); col(rIdx(j) + a, [pv[0] - t3[0] * f0, pv[1] - t3[1] * f0, pv[2] - t3[2] * f0]); }
      }
      for (let a = 0; a < 3; a++) { const ea = [0, 0, 0]; ea[a] = q; col(cIdx(i) + a, mulP(ea)); const eb = [0, 0, 0]; eb[a] = -q; col(cIdx(j) + a, mulP(eb)); }
      { const dwi = mv(Ri, ui.du), dwj = mv(Rj, uj.du); const pv = mulP(dwi); col(iPhi, [pv[0] - dwj[0] * f0, pv[1] - dwj[1] * f0, pv[2] - dwj[2] * f0]); }
      col(iGam, mulP([0, 0, q * (Fi[2] - Fj[2])]));
      const Jq = mulP([q * dcv[0], q * dcv[1], q * dcv[2]]);
      // накопление
      for (let a = 0; a < nc; a++) {
        const ja0 = J[a], ja1 = J[14 + a], ja2 = J[28 + a], pa = idx[a];
        b[pa] += wt * (ja0 * r[0] + ja1 * r[1] + ja2 * r[2]);
        for (let c = 0; c < nc; c++) A[pa * P + idx[c]] += wt * (ja0 * J[c] + ja1 * J[14 + c] + ja2 * J[28 + c]);
      }
      aqq[k] = wt * (Jq[0] * Jq[0] + Jq[1] * Jq[1] + Jq[2] * Jq[2]) + 1 / (sigS * sigS);
      bq[k] = wt * (Jq[0] * r[0] + Jq[1] * r[1] + Jq[2] * r[2]) + (s[k] - s0) / (sigS * sigS);
      const ax = new Float64Array(nc); for (let a = 0; a < nc; a++) ax[a] = wt * (J[a] * Jq[0] + J[14 + a] * Jq[1] + J[28 + a] * Jq[2]);
      Axq.push([Int32Array.from(idx.subarray(0, nc)), ax]);
    }
    for (let c = 1; c < N; c++) { const l = logSO3(m3mul(C2W[c], m3T(cams[c].C2Wprior))), p = rIdx(c); for (let a = 0; a < 3; a++) { A[(p + a) * P + p + a] += 1 / (sigR * sigR); b[p + a] += l[a] / (sigR * sigR); } }
    for (let c = 0; c < N; c++) { const p = cIdx(c); for (let a = 0; a < 3; a++) { A[(p + a) * P + p + a] += 1 / (sigC * sigC); b[p + a] += dC[c][a] / (sigC * sigC); } }
    A[iPhi * P + iPhi] += 1 / (sigF * sigF); b[iPhi] += phi / (sigF * sigF);
    A[iGam * P + iGam] += 1 / (sigG * sigG); b[iGam] += (gam - 0.5) / (sigG * sigG);
    let improved = false;
    for (let tries = 0; tries < 8 && !improved; tries++) {
      const Ad = Float64Array.from(A), bd = Float64Array.from(b);
      for (let qd = 0; qd < P; qd++) Ad[qd * P + qd] += lambda * A[qd * P + qd] + 1e-9;
      const aqqD = new Float64Array(M.n);
      for (let k = 0; k < M.n; k++) {
        const a = aqq[k] * (1 + lambda) + 1e-12; aqqD[k] = a;
        const [ix, ax] = Axq[k], n = ix.length;
        for (let u = 0; u < n; u++) { const fu = ax[u] / a; bd[ix[u]] -= fu * bq[k]; for (let w = 0; w < n; w++) Ad[ix[u] * P + ix[w]] -= fu * ax[w]; }
      }
      const dx = cholSolve(Ad, bd.map(v => -v), P); if (!dx) { lambda *= 10; continue; }
      const sN = new Float64Array(M.n);
      for (let k = 0; k < M.n; k++) { const [ix, ax] = Axq[k]; let t = bq[k]; for (let u = 0; u < ix.length; u++) t += ax[u] * dx[ix[u]]; sN[k] = s[k] + Math.max(-2, Math.min(2, -t / aqqD[k])); }
      const RN = C2W.map((R, c) => { if (c === 0) return R; const p = rIdx(c); return m3mul(expSO3(dx[p], dx[p + 1], dx[p + 2]), R); });
      const DN = dC.map((d, c) => { const p = cIdx(c); return [d[0] + dx[p], d[1] + dx[p + 1], d[2] + dx[p + 2]]; });
      const phN = phi + dx[iPhi], gN = gam + dx[iGam];
      const En = cost(RN, DN, phN, gN, sN);
      if (En < E) { const rel = (E - En) / E; C2W = RN; dC = DN; phi = phN; gam = gN; s.set(sN); E = En; lambda = Math.max(1e-7, lambda / 3); improved = true; if (rel < 1e-6) it = iters; }
      else lambda *= 4;
    }
    if (report) report(it / iters);
    if (!improved) break;
  }
  const per = new Float64Array(M.n); let s2 = 0;
  for (let k = 0; k < M.n; k++) { resid(C2W, dC, phi, gam, s, k, r3); const e = Math.hypot(r3[0], r3[1], r3[2]); per[k] = e; s2 += e * e; }
  const centers = C2W.map((R, c) => center(R, dC[c], gam));
  return { C2W, centers, f: f0 * Math.exp(phi), gamma: gam, s, rms: M.n ? Math.sqrt(s2 / M.n) : 0, per };
}
function cross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }

// Плотная глубина: для угловых точек кадра i ищем пару в кадре j вдоль эпиполярной линии
// (перебор обратной глубины q при известных позах и центрах). Возвращает точки сцены.
function epipolarPoints(cams, f, pairs, report) {
  const pts = [], o = [0, 0];
  const L = 1, tpl = 6;
  pairs.forEach((pr, pk) => {
    const ci = cams[pr.i], cj = cams[pr.j]; if (!ci.pyr || !cj.pyr) return;
    const Ii = ci.pyr.levels[L], Ij = cj.pyr.levels[L], Ii0 = ci.pyr.levels[0], Ij0 = cj.pyr.levels[0];
    const kI = ci.pyr.k * 2, kJ = cj.pyr.k * 2, k0I = ci.pyr.k, k0J = cj.pyr.k;
    const H = pairHomography(ci, cj, f), W2Cj = m3T(cj.C2W);
    const dc = [ci.center[0] - cj.center[0], ci.center[1] - cj.center[1], ci.center[2] - cj.center[2]];
    const proj = (v) => { const z = W2Cj[6] * v[0] + W2Cj[7] * v[1] + W2Cj[8] * v[2]; if (z <= 1e-6) return null; const d = distOff(cj, f * (W2Cj[0] * v[0] + W2Cj[1] * v[1] + W2Cj[2] * v[2]) / z, f * (W2Cj[3] * v[0] + W2Cj[4] * v[1] + W2Cj[5] * v[2]) / z); return [d[0] + cj.cx, d[1] + cj.cy]; };
    let used = 0;
    for (const c of ci.corners) {
      if (used > 260) break;
      hApply(H, c[0], c[1], o); if (o[0] !== o[0]) continue;
      const m = 20; if (o[0] < -200 || o[1] < -200 || o[0] > cj.w + 200 || o[1] > cj.h + 200) continue;
      const uo = undistOff(ci, c[0] - ci.cx, c[1] - ci.cy), w = rayW(ci.C2W, uo[0], uo[1], f);
      // кривая кандидатов: q от 0 (бесконечность) до qMax
      const cand = []; let last = null;
      for (let qq = 0; qq <= 0.9; qq += 0.004) {
        const p = proj([w[0] + qq * dc[0], w[1] + qq * dc[1], w[2] + qq * dc[2]]); if (!p) break;
        if (p[0] < m || p[1] < m || p[0] > cj.w - m || p[1] > cj.h - m) { if (cand.length) break; else continue; }
        if (last && Math.hypot(p[0] - last[0], p[1] - last[1]) < 0.9 * kJ) continue;
        cand.push([p[0], p[1], qq]); last = p;
      }
      if (cand.length < 3) continue;
      used++;
      const xi = (c[0] + 0.5) / kI - 0.5, yi = (c[1] + 0.5) / kI - 0.5;
      const T = makeTemplate(Ii, xi, yi, tpl); if (!T) continue;
      // образец из j: патч, повёрнутый по гомографии поворота, сдвинутый в точку кандидата
      const n = 2 * tpl + 1, N = n * n, S = new Float32Array(N);
      let best = -2, bi = -1, second = -2; const sc = new Float32Array(cand.length);
      // локальные смещения гомографии вокруг точки i (в пикселях j полного разрешения)
      const off = new Float32Array(N * 2);
      for (let v = -tpl, q = 0; v <= tpl; v++) for (let u = -tpl; u <= tpl; u++, q++) { hApply(H, c[0] + u * kI, c[1] + v * kI, o); off[q * 2] = o[0]; off[q * 2 + 1] = o[1]; }
      const hc = [0, 0]; hApply(H, c[0], c[1], hc);
      for (let k = 0; k < cand.length; k++) {
        const dx = cand[k][0] - hc[0], dy = cand[k][1] - hc[1];
        let s = 0, ss = 0, dot = 0, ok = true;
        for (let q = 0; q < N; q++) {
          const val = sampleF(Ij, (off[q * 2] + dx + 0.5) / kJ - 0.5, (off[q * 2 + 1] + dy + 0.5) / kJ - 0.5);
          if (val !== val) { ok = false; break; } S[q] = val; s += val; ss += val * val; dot += T[q] * val;
        }
        const vv = ss - s * s / N; const r = ok && vv > 1e-7 ? dot / Math.sqrt(vv) : -2; sc[k] = r;
        if (r > best) { best = r; bi = k; }
      }
      if (bi < 0 || best < 0.82) continue;
      for (let k = 0; k < cand.length; k++) if (Math.abs(k - bi) > 3 && sc[k] > second) second = sc[k];
      if (second > best - 0.04) continue; // неоднозначно (повтор текстуры)
      // уточнение глубины параболой
      let qb = cand[bi][2];
      if (bi > 0 && bi < cand.length - 1) { const a = sc[bi - 1], b2 = sc[bi + 1], den = a - 2 * best + b2; if (den < -1e-6) { const t = Math.max(-0.5, Math.min(0.5, 0.5 * (a - b2) / den)); qb += t * (t > 0 ? cand[bi + 1][2] - qb : qb - cand[bi - 1][2]); } }
      if (qb < 1e-3) { pts.push([w[0] * 1e3, w[1] * 1e3, w[2] * 1e3, 0.5]); continue; }
      pts.push([ci.center[0] + w[0] / qb, ci.center[1] + w[1] / qb, ci.center[2] + w[2] / qb, best]);
    }
    if (report) report((pk + 1) / pairs.length);
  });
  return pts;
}

// Поле обратной глубины на сфере (сетка GW×GH по долготе/широте) из облака точек; пустоты — гладкая интерполяция.
function buildDepthField(points, GW, GH, smooth) {
  const sum = new Float64Array(GW * GH), cnt = new Float64Array(GW * GH);
  const all = [], cellVals = new Map();
  for (const p of points) {
    const n = Math.hypot(p[0], p[1], p[2]); if (!(n > 1e-6)) continue;
    const iD = 1 / n, lon = Math.atan2(p[0], p[1]), lat = Math.asin(p[2] / n);
    const x = Math.min(GW - 1, Math.max(0, Math.floor((lon / (2 * Math.PI) + 0.5) * GW))), y = Math.min(GH - 1, Math.max(0, Math.floor((0.5 - lat / Math.PI) * GH)));
    sum[y * GW + x] += iD; cnt[y * GW + x] += 1; all.push(iD);
    const key = y * GW + x; if (!cellVals.has(key)) cellVals.set(key, []); cellVals.get(key).push(iD);
  }
  all.sort((a, b) => a - b);
  const med = all.length ? all[all.length >> 1] : 0;
  const data = new Float64Array(GW * GH), wd = new Float64Array(GW * GH);
  // медиана в ячейке (устойчиво к выбросам), вес — по числу точек
  for (const [k, arr] of cellVals) { arr.sort((a, b) => a - b); data[k] = arr[arr.length >> 1]; wd[k] = Math.min(6, arr.length) * 2; }
  let D = new Float64Array(GW * GH).fill(med), D2 = new Float64Array(GW * GH);
  // грубое приближение (вдвое меньшая сетка) — чтобы пустоты заполнялись быстро
  if (GW >= 48 && points.length) {
    const sub = buildDepthField(points, GW / 2, GH / 2, smooth);
    for (let y = 0; y < GH; y++) for (let x = 0; x < GW; x++) D[y * GW + x] = depthAt(sub, (x + 0.5) / GW * 2 * Math.PI - Math.PI, Math.PI / 2 - (y + 0.5) / GH * Math.PI);
  }
  for (let k = 0; k < GW * GH; k++) if (cnt[k] > 0) D[k] = data[k];
  const lam = smooth != null ? smooth : 0.6;
  for (let it = 0; it < 150; it++) {
    for (let y = 0; y < GH; y++) for (let x = 0; x < GW; x++) {
      const k = y * GW + x, l = y * GW + (x + GW - 1) % GW, r = y * GW + (x + 1) % GW, u = y > 0 ? k - GW : k, d = y < GH - 1 ? k + GW : k;
      const nb = (D[l] + D[r] + D[u] + D[d]) * lam;
      D2[k] = (nb + wd[k] * data[k] + (wd[k] ? 0 : 0.002 * med)) / (4 * lam + wd[k] + (wd[k] ? 0 : 0.002));
    }
    const t = D; D = D2; D2 = t;
  }
  return { GW, GH, iD: Float32Array.from(D), median: med };
}
function depthAt(field, lon, lat) {
  const GW = field.GW, GH = field.GH;
  let x = (lon / (2 * Math.PI) + 0.5) * GW - 0.5, y = (0.5 - lat / Math.PI) * GH - 0.5;
  if (y < 0) y = 0; if (y > GH - 1) y = GH - 1;
  const x0 = Math.floor(x), y0 = Math.min(GH - 2, Math.floor(y)), fx = x - x0, fy = y - y0;
  const xa = ((x0 % GW) + GW) % GW, xb = (xa + 1) % GW, D = field.iD;
  return (D[y0 * GW + xa] * (1 - fx) + D[y0 * GW + xb] * fx) * (1 - fy) + (D[(y0 + 1) * GW + xa] * (1 - fx) + D[(y0 + 1) * GW + xb] * fx) * fy;
}

// ───────────────────────── выравнивание яркости ─────────────────────────
function patchMeanLin(cam, x, y, out) {
  const r = Math.max(2, Math.round(Math.max(cam.w, cam.h) / 400));
  const x0 = Math.max(0, Math.round(x) - r), x1 = Math.min(cam.w - 1, Math.round(x) + r);
  const y0 = Math.max(0, Math.round(y) - r), y1 = Math.min(cam.h - 1, Math.round(y) + r);
  let a = 0, b = 0, c = 0, n = 0;
  for (let yy = y0; yy <= y1; yy++) for (let xx = x0; xx <= x1; xx++) { const p = (yy * cam.w + xx) * 3; a += SRGB2LIN[cam.rgb[p]]; b += SRGB2LIN[cam.rgb[p + 1]]; c += SRGB2LIN[cam.rgb[p + 2]]; n++; }
  out[0] = a / n; out[1] = b / n; out[2] = c / n;
}
function solveGains(cams, pairs) {
  const N = cams.length, gains = cams.map(() => [1, 1, 1]);
  const mi = [0, 0, 0], mj = [0, 0, 0];
  const terms = [];
  for (const p of pairs) {
    if (p.matches.length < 6) continue;
    const si = [0, 0, 0], sj = [0, 0, 0];
    for (const m of p.matches) { patchMeanLin(cams[p.i], m.xi, m.yi, mi); patchMeanLin(cams[p.j], m.xj, m.yj, mj); for (let c = 0; c < 3; c++) { si[c] += mi[c]; sj[c] += mj[c]; } }
    terms.push({ i: p.i, j: p.j, n: p.matches.length, si, sj });
  }
  if (!terms.length) return gains;
  const avgN = terms.reduce((s, t) => s + t.n, 0) / terms.length, lam = 0.02 * avgN;
  for (let c = 0; c < 3; c++) {
    const A = new Float64Array(N * N), b = new Float64Array(N);
    for (const t of terms) {
      if (t.si[c] < 1e-4 || t.sj[c] < 1e-4) continue;
      const r = Math.log(t.sj[c] / t.si[c]), w = t.n; // l_i - l_j = r
      A[t.i * N + t.i] += w; A[t.j * N + t.j] += w; A[t.i * N + t.j] -= w; A[t.j * N + t.i] -= w;
      b[t.i] += w * r; b[t.j] -= w * r;
    }
    for (let i = 0; i < N; i++) A[i * N + i] += lam;
    const x = cholSolve(A, b, N); if (!x) continue;
    for (let i = 0; i < N; i++) gains[i][c] = Math.max(0.5, Math.min(2, Math.exp(x[i])));
  }
  return gains;
}

// Яркость кадров по областям перекрытия (не нужны найденные точки — работает и на белом потолке).
// Сравниваем размытые (низкая полоса) цвета в сетке точек общей области; медиана отношений устойчива к параллаксу.
function solveGainsOverlap(cams, f) {
  const N = cams.length, terms = [];
  for (const c of cams) if (!c.low) c.low = lowBand(c);
  const smp = (lb, x, y, out) => {
    let lx = (x + 0.5) / lb.D - 0.5, ly = (y + 0.5) / lb.D - 0.5;
    lx = Math.max(0, Math.min(lb.lw - 1.001, lx)); ly = Math.max(0, Math.min(lb.lh - 1.001, ly));
    const x0 = lx | 0, y0 = ly | 0, fx = lx - x0, fy = ly - y0, q = (y0 * lb.lw + x0) * 3, qr = q + lb.lw * 3, a = lb.a;
    for (let c = 0; c < 3; c++) out[c] = (a[q + c] * (1 - fx) + a[q + 3 + c] * fx) * (1 - fy) + (a[qr + c] * (1 - fx) + a[qr + 3 + c] * fx) * fy;
  };
  const ci3 = [0, 0, 0], cj3 = [0, 0, 0], t1 = [0, 0, 0];
  // ровность: яркость в радиусе ~3 клеток (≈25 px) отличается не больше чем на 6 %
  const flatAt = (lb, x, y) => {
    smp(lb, x, y, t1); const c0 = t1[0] + t1[1] + t1[2]; if (c0 <= 0) return false;
    for (const [dx, dy] of [[24, 0], [-24, 0], [0, 24], [0, -24]]) { smp(lb, x + dx, y + dy, t1); if (Math.abs(t1[0] + t1[1] + t1[2] - c0) > 0.06 * c0) return false; }
    return true;
  };
  for (let i = 0; i < N; i++) for (let j = i + 1; j < N; j++) {
    const a = camForward(cams[i].C2W), b = camForward(cams[j].C2W);
    if (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] < Math.cos(80 * Math.PI / 180)) continue;
    const Rrel = m3mul(m3T(cams[j].C2W), cams[i].C2W), A = cams[i], B = cams[j], lr = [[], [], []];
    const GX = 18, GY = 24, mx = 0.06 * B.w, my = 0.06 * B.h;
    for (let gy = 0; gy < GY; gy++) for (let gx = 0; gx < GX; gx++) {
      const x = (gx + 0.5) / GX * A.w, y = (gy + 0.5) / GY * A.h;
      const v = mv(Rrel, camRay(A, f, x, y)); if (v[2] <= 1e-6) continue;
      const pd = distOff(B, f * v[0] / v[2], f * v[1] / v[2]), xj = pd[0] + B.cx, yj = pd[1] + B.cy;
      if (xj < mx || yj < my || xj > B.w - mx || yj > B.h - my) continue;
      smp(A.low, x, y, ci3); smp(B.low, xj, yj, cj3);
      // только ровные участки: там небольшой сдвиг от параллакса не меняет цвет
      if (!flatAt(A.low, x, y) || !flatAt(B.low, xj, yj)) continue;
      for (let c = 0; c < 3; c++) if (ci3[c] > 0.004 && cj3[c] > 0.004 && ci3[c] < 0.92 && cj3[c] < 0.92) lr[c].push(Math.log(cj3[c] / ci3[c]));
    }
    if (lr[1].length < 12) continue;
    // мода: окно ±0.06 с наибольшим числом значений; доля попавших — уверенность пары
    let agree = 0;
    const med = lr.map((arr, c) => {
      if (arr.length < 12) return null; arr.sort((p, q) => p - q);
      let best = 0, bi = 0, lo = 0;
      for (let hi = 0; hi < arr.length; hi++) { while (arr[hi] - arr[lo] > 0.12) lo++; if (hi - lo + 1 > best) { best = hi - lo + 1; bi = lo; } }
      if (c === 1) agree = best / arr.length;
      const win = arr.slice(bi, bi + best); return win[win.length >> 1];
    });
    if (agree < 0.35) continue;
    terms.push({ i, j, n: lr[1].length * agree, med });
  }
  const gains = cams.map(() => [1, 1, 1]);
  if (!terms.length) return gains;
  for (let c = 0; c < 3; c++) {
    const A = new Float64Array(N * N), b = new Float64Array(N);
    for (const t of terms) {
      if (t.med[c] == null) continue;
      const r = t.med[c], w = Math.sqrt(t.n); // l_i - l_j = r
      A[t.i * N + t.i] += w; A[t.j * N + t.j] += w; A[t.i * N + t.j] -= w; A[t.j * N + t.i] -= w;
      b[t.i] += w * r; b[t.j] -= w * r;
    }
    for (let i = 0; i < N; i++) A[i * N + i] += 0.08;
    const x = cholSolve(A, b, N); if (!x) continue;
    let mean = 0; for (let i = 0; i < N; i++) mean += x[i]; mean /= N;
    for (let i = 0; i < N; i++) gains[i][c] = Math.max(0.25, Math.min(4, Math.exp(x[i] - mean)));
  }
  return gains;
}

// ───────────────────────── швы в обход объектов (graph cut, α-расширение) ─────────────────────────
// Максимальный поток (Диниц) на небольших графах.
function MaxFlow(n, mEst) {
  this.n = n; this.head = new Int32Array(n).fill(-1);
  this.to = new Int32Array(mEst * 2); this.nxt = new Int32Array(mEst * 2); this.cap = new Float64Array(mEst * 2); this.m = 0;
}
MaxFlow.prototype.edge = function (u, v, c, rc) {
  if (this.m + 2 > this.to.length) { const g = (a, T) => { const b = new T(a.length * 2); b.set(a); return b; }; this.to = g(this.to, Int32Array); this.nxt = g(this.nxt, Int32Array); this.cap = g(this.cap, Float64Array); }
  const m = this.m;
  this.to[m] = v; this.cap[m] = c; this.nxt[m] = this.head[u]; this.head[u] = m;
  this.to[m + 1] = u; this.cap[m + 1] = rc || 0; this.nxt[m + 1] = this.head[v]; this.head[v] = m + 1;
  this.m += 2;
};
MaxFlow.prototype.run = function (s, t) {
  const n = this.n, level = new Int32Array(n), it = new Int32Array(n), queue = new Int32Array(n), stack = new Int32Array(n), path = new Int32Array(n);
  const { to, nxt, cap, head } = this; let flow = 0;
  for (;;) {
    level.fill(-1); level[s] = 0; let qh = 0, qt = 0; queue[qt++] = s;
    while (qh < qt) { const u = queue[qh++]; for (let e = head[u]; e >= 0; e = nxt[e]) if (cap[e] > 1e-12 && level[to[e]] < 0) { level[to[e]] = level[u] + 1; queue[qt++] = to[e]; } }
    if (level[t] < 0) break;
    for (let i = 0; i < n; i++) it[i] = head[i];
    // итеративный поиск блокирующего потока
    for (;;) {
      let sp = 0, u = s; stack[0] = s;
      while (u !== t) {
        let e = it[u];
        while (e >= 0 && !(cap[e] > 1e-12 && level[to[e]] === level[u] + 1)) e = nxt[e];
        it[u] = e;
        if (e < 0) { if (sp === 0) { u = -1; break; } level[u] = -1; sp--; u = stack[sp]; it[u] = nxt[it[u]]; continue; }
        path[sp] = e; stack[++sp] = to[e]; u = to[e];
      }
      if (u < 0) break;
      let push = Infinity; for (let k = 0; k < sp; k++) push = Math.min(push, cap[path[k]]);
      for (let k = 0; k < sp; k++) { cap[path[k]] -= push; cap[path[k] ^ 1] += push; }
      flow += push;
    }
  }
  // множество вершин, достижимых из источника в остаточной сети
  const seen = new Uint8Array(n); let qh = 0, qt = 0; queue[qt++] = s; seen[s] = 1;
  while (qh < qt) { const u = queue[qh++]; for (let e = head[u]; e >= 0; e = nxt[e]) if (cap[e] > 1e-12 && !seen[to[e]]) { seen[to[e]] = 1; queue[qt++] = to[e]; } }
  return { flow, sourceSide: seen };
};

// Для каждой ячейки сетки LW×LH: какие кадры её видят и какого она цвета в каждом.
// Энергия: данные = предпочтение центра кадра + отклонение от медианы кандидатов (убирает прохожих),
//          шов   = разница цветов двух кадров по обе стороны шва (шов идёт там, где кадры совпадают).
function seamLabels(cams, f, gains, depth, LW, report, sopt) {
  sopt = sopt || {};
  const LH = LW / 2, NC = LW * LH, K = 6, NI = cams.length;
  const candI = new Int16Array(NC * K).fill(-1), candC = new Float32Array(NC * K * 3), candW = new Float32Array(NC * K), candG = new Float32Array(NC * K), nc = new Uint8Array(NC);
  // уменьшенные (×4) копии кадров в sRGB для сравнения цветов
  for (let ci = 0; ci < NI; ci++) {
    const cam = cams[ci], D = 4, sw = Math.floor(cam.w / D), sh = Math.floor(cam.h / D), sm = new Float32Array(sw * sh * 3), g = gains[ci];
    for (let y = 0; y < sh; y++) for (let x = 0; x < sw; x++) {
      let r = 0, gg = 0, b = 0;
      for (let yy = 0; yy < D; yy++) for (let xx = 0; xx < D; xx++) { const p = ((y * D + yy) * cam.w + x * D + xx) * 3; r += SRGB2LIN[cam.rgb[p]]; gg += SRGB2LIN[cam.rgb[p + 1]]; b += SRGB2LIN[cam.rgb[p + 2]]; }
      const o = (y * sw + x) * 3, n = D * D; sm[o] = Math.sqrt(r / n * g[0]); sm[o + 1] = Math.sqrt(gg / n * g[1]); sm[o + 2] = Math.sqrt(b / n * g[2]);
    }
    const W2C = m3T(cam.C2W), ctr = cam.center || [0, 0, 0], par = !!(depth && cam.center);
    for (let v = 0; v < LH; v++) {
      const lat = Math.PI / 2 - (v + 0.5) / LH * Math.PI, cl = Math.cos(lat), sl = Math.sin(lat);
      for (let u = 0; u < LW; u++) {
        const lon = (u + 0.5) / LW * 2 * Math.PI - Math.PI;
        let dx = cl * Math.sin(lon), dy = cl * Math.cos(lon), dz = sl;
        if (par) { const q = depthAt(depth, lon, lat); dx -= q * ctr[0]; dy -= q * ctr[1]; dz -= q * ctr[2]; }
        const cz = W2C[6] * dx + W2C[7] * dy + W2C[8] * dz; if (cz <= 1e-6) continue;
        const pd = distOff(cam, f * (W2C[0] * dx + W2C[1] * dy + W2C[2] * dz) / cz, f * (W2C[3] * dx + W2C[4] * dy + W2C[5] * dz) / cz);
        const x = pd[0] + cam.cx, y = pd[1] + cam.cy;
        if (x < 1 || y < 1 || x > cam.w - 2 || y > cam.h - 2) continue;
        if (cam.trimY && y > cam.trimY) continue;
        const cell = v * LW + u; if (nc[cell] >= K) continue;
        const wx = 1 - Math.abs((x - cam.cx) / (cam.cx + 0.5)), wy = 1 - Math.abs((y - cam.cy) / (cam.cy + 0.5));
        const sx = Math.min(sw - 1.001, Math.max(0, (x + 0.5) / D - 0.5)), sy = Math.min(sh - 1.001, Math.max(0, (y + 0.5) / D - 0.5));
        const x0 = sx | 0, y0 = sy | 0, fx = sx - x0, fy = sy - y0, k = cell * K + nc[cell];
        for (let c = 0; c < 3; c++) {
          const p = (y0 * sw + x0) * 3 + c;
          candC[k * 3 + c] = (sm[p] * (1 - fx) + sm[p + 3] * fx) * (1 - fy) + (sm[p + sw * 3] * (1 - fx) + sm[p + sw * 3 + 3] * fx) * fy;
        }
        // сила края в этой точке кадра: если у одного кадра здесь край, а у другого нет — края разъехались, шов тут виден
        {
          const lum = (xx, yy) => { xx = Math.min(sw - 1.001, Math.max(0, xx)); yy = Math.min(sh - 1.001, Math.max(0, yy)); const a = xx | 0, b = yy | 0, fa = xx - a, fb = yy - b, p = (b * sw + a) * 3, q = p + sw * 3;
            return ((sm[p] + sm[p + 1] + sm[p + 2]) * (1 - fa) + (sm[p + 3] + sm[p + 4] + sm[p + 5]) * fa) * (1 - fb) + ((sm[q] + sm[q + 1] + sm[q + 2]) * (1 - fa) + (sm[q + 3] + sm[q + 4] + sm[q + 5]) * fa) * fb; };
          candG[k] = Math.hypot(lum(sx + 1, sy) - lum(sx - 1, sy), lum(sx, sy + 1) - lum(sx, sy - 1)) / 3;
        }
        candI[k] = ci; candW[k] = wx * wy; nc[cell]++;
      }
    }
    if (report) report(0.3 * (ci + 1) / NI);
  }
  // данные: центр кадра + согласие с медианой
  const dataC = new Float32Array(NC * K);
  const med = [0, 0, 0], tmp = [[], [], []];
  for (let cell = 0; cell < NC; cell++) {
    const n = nc[cell]; if (!n) continue;
    let useMed = n >= 3;
    if (useMed) for (let c = 0; c < 3; c++) { const a = tmp[c]; a.length = 0; for (let k = 0; k < n; k++) a.push(candC[(cell * K + k) * 3 + c]); a.sort((x, y) => x - y); med[c] = a[n >> 1]; }
    for (let k = 0; k < n; k++) {
      const q = cell * K + k; let d = 0.03 * (1 - candW[q]);
      if (useMed) { const e = Math.abs(candC[q * 3] - med[0]) + Math.abs(candC[q * 3 + 1] - med[1]) + Math.abs(candC[q * 3 + 2] - med[2]); d += 1.2 * Math.max(0, e - (sopt.medTol != null ? sopt.medTol : 0.15)); }
      if (candW[q] < 0.08) d += 0.3 * (0.08 - candW[q]) / 0.08; // у самого края кадра — неохотно
      dataC[q] = d;
    }
  }
  const slot = (cell, img) => { const n = nc[cell]; for (let k = 0; k < n; k++) if (candI[cell * K + k] === img) return cell * K + k; return -1; };
  // начальная разметка — ближайший к центру кадр
  const lab = new Int16Array(NC).fill(-1);
  for (let cell = 0; cell < NC; cell++) { let best = -1, bw = -1; for (let k = 0; k < nc[cell]; k++) { const q = cell * K + k; if (candW[q] > bw) { bw = candW[q]; best = candI[q]; } } lab[cell] = best; }
  const BIG = 2.0, LAMS = sopt.lambda != null ? sopt.lambda : 1.0, GW8 = sopt.edgeW != null ? sopt.edgeW : 2.0;
  const colDiff = (cell, a, b) => {
    if (a === b) return 0;
    const qa = slot(cell, a), qb = slot(cell, b); if (qa < 0 || qb < 0) return BIG;
    return Math.abs(candC[qa * 3] - candC[qb * 3]) + Math.abs(candC[qa * 3 + 1] - candC[qb * 3 + 1]) + Math.abs(candC[qa * 3 + 2] - candC[qb * 3 + 2]) + GW8 * Math.abs(candG[qa] - candG[qb]);
  };
  const Spq = (p, q, a, b, wgt) => (a === b ? 0 : LAMS * wgt * (colDiff(p, a, b) + colDiff(q, a, b)));
  const dcost = (cell, img) => { const q = slot(cell, img); return q < 0 ? Infinity : dataC[q]; };
  const rowW = new Float32Array(LH); for (let v = 0; v < LH; v++) rowW[v] = Math.max(0.05, Math.cos(Math.PI / 2 - (v + 0.5) / LH * Math.PI));
  const nodeOf = new Int32Array(NC).fill(-1);
  for (let cycle = 0; cycle < 2; cycle++) {
    let changed = 0;
    for (let alpha = 0; alpha < NI; alpha++) {
      // узлы: ячейки, где виден кадр alpha и метка ещё не alpha
      const nodes = [];
      for (let cell = 0; cell < NC; cell++) { nodeOf[cell] = -1; if (lab[cell] !== alpha && lab[cell] >= 0 && slot(cell, alpha) >= 0) { nodeOf[cell] = nodes.length; nodes.push(cell); } }
      if (!nodes.length) continue;
      const S = nodes.length, T = S + 1, g = new MaxFlow(S + 2, S * 6);
      const U0 = new Float64Array(S), U1 = new Float64Array(S);
      for (let k = 0; k < S; k++) { const cell = nodes[k]; U1[k] = dcost(cell, alpha); U0[k] = dcost(cell, lab[cell]); }
      for (let k = 0; k < S; k++) {
        const p = nodes[k], v = (p / LW) | 0, u = p - v * LW;
        const nbs = [[v * LW + (u + 1) % LW, rowW[v]], [v < LH - 1 ? p + LW : -1, 1]];
        if (true) { // соседи слева/сверху, не являющиеся узлами — учитываем как унарные
          const left = v * LW + (u + LW - 1) % LW, up = v > 0 ? p - LW : -1;
          if (left >= 0 && nodeOf[left] < 0 && lab[left] >= 0) { U1[k] += Spq(p, left, alpha, lab[left], rowW[v]); U0[k] += Spq(p, left, lab[p], lab[left], rowW[v]); }
          if (up >= 0 && nodeOf[up] < 0 && lab[up] >= 0) { U1[k] += Spq(p, up, alpha, lab[up], 1); U0[k] += Spq(p, up, lab[p], lab[up], 1); }
        }
        for (const [q, wgt] of nbs) {
          if (q < 0 || lab[q] < 0) continue;
          const kq = nodeOf[q];
          if (kq < 0) { U1[k] += Spq(p, q, alpha, lab[q], wgt); U0[k] += Spq(p, q, lab[p], lab[q], wgt); continue; }
          const A = Spq(p, q, lab[p], lab[q], wgt), B = Spq(p, q, lab[p], alpha, wgt), Cc = Spq(p, q, alpha, lab[q], wgt), Dd = 0;
          // E = A + (Cc−A)·xp + (Dd−Cc)·xq + (B+Cc−A−Dd)·(1−xp)·xq
          const c1 = Cc - A, c2 = Dd - Cc;
          if (c1 >= 0) U1[k] += c1; else U0[k] -= c1;
          if (c2 >= 0) U1[kq] += c2; else U0[kq] -= c2;
          const w = B + Cc - A - Dd; if (w > 1e-9) g.edge(k, kq, w, 0);
        }
      }
      for (let k = 0; k < S; k++) {
        let a = U1[k], b = U0[k]; if (!isFinite(a)) a = 1e6; if (!isFinite(b)) b = 1e6;
        const m = Math.min(a, b); a -= m; b -= m;
        if (a > 0) g.edge(S, k, a, 0); if (b > 0) g.edge(k, T, b, 0);
      }
      const { sourceSide } = g.run(S, T);
      // x=1 (берём alpha) ⇔ вершина на стороне стока
      for (let k = 0; k < S; k++) if (!sourceSide[k]) { lab[nodes[k]] = alpha; changed++; }
      if (report) report(0.3 + 0.7 * (cycle * NI + alpha + 1) / (2 * NI));
    }
    if (!changed) break;
  }
  // карта риска: шов проходит там, где соседние кадры расходятся (двоение, разрыв), или кадр спорит с медианой (прохожий)
  const risk = new Float32Array(NC);
  for (let p = 0; p < NC; p++) {
    const a = lab[p]; if (a < 0) continue;
    const v = (p / LW) | 0, u = p - v * LW;
    for (const q of [v * LW + (u + 1) % LW, v < LH - 1 ? p + LW : -1]) {
      if (q < 0) continue; const b = lab[q]; if (b < 0 || b === a) continue;
      const qa = slot(p, b), qb = slot(q, a);
      const d = (qa >= 0 ? colDiff(p, a, b) : 0) + (qb >= 0 ? colDiff(q, a, b) : 0);
      if (d > risk[p]) risk[p] = d; if (d > risk[q]) risk[q] = d;
    }
    const sp = slot(p, a); if (sp >= 0 && nc[p] >= 3) { const dm = dataC[sp] / 1.2; if (dm > 0.5 && dm > risk[p]) risk[p] = dm; }
  }
  return { LW, LH, lab, risk };
}
// мягкие веса шва для кадра: размытый индикатор «эта ячейка взята из кадра i»
function seamWeights(seams, ci, sigma) {
  const { LW, LH, lab } = seams, a = new Float32Array(LW * LH), b = new Float32Array(LW * LH);
  let any = false; for (let k = 0; k < a.length; k++) if (lab[k] === ci) { a[k] = 1; any = true; }
  if (!any) return null;
  const r = Math.ceil(sigma * 2.5), ker = []; let ks = 0; for (let i = -r; i <= r; i++) { const v = Math.exp(-i * i / (2 * sigma * sigma)); ker.push(v); ks += v; }
  for (let i = 0; i < ker.length; i++) ker[i] /= ks;
  for (let v = 0; v < LH; v++) for (let u = 0; u < LW; u++) { let s = 0; for (let i = -r; i <= r; i++) s += ker[i + r] * a[v * LW + ((u + i) % LW + LW) % LW]; b[v * LW + u] = s; }
  for (let v = 0; v < LH; v++) for (let u = 0; u < LW; u++) { let s = 0; for (let i = -r; i <= r; i++) s += ker[i + r] * b[Math.max(0, Math.min(LH - 1, v + i)) * LW + u]; a[v * LW + u] = s; }
  return a;
}

// ───────────────────────── локальное выравнивание по швам (deghosting) ─────────────────────────
// Вдоль шва между кадрами A и B измеряем сдвиг t содержимого B относительно A (в касательной плоскости сферы).
// Каждый кадр получает гладкое поле смещений: на шве A — −t/2, B — +t/2, к середине кадра поправка затухает.
function projCam(cam, f, W2C, d) {
  const cz = W2C[6] * d[0] + W2C[7] * d[1] + W2C[8] * d[2]; if (cz <= 1e-6) return null;
  let x = f * (W2C[0] * d[0] + W2C[1] * d[1] + W2C[2] * d[2]) / cz, y = f * (W2C[3] * d[0] + W2C[4] * d[1] + W2C[5] * d[2]) / cz;
  if (Math.abs(x) > 1.4 * cam.cx || Math.abs(y) > 1.4 * cam.cy) return null;
  const k1 = cam.k1 || 0;
  if (k1) { const xu = x, yu = y, iR = 1 / (cam.R0 * cam.R0); for (let it = 0; it < 4; it++) { const s = 1 + k1 * (x * x + y * y) * iR; x = xu / s; y = yu / s; } }
  x += cam.cx; y += cam.cy;
  if (x < 1 || y < 1 || x > cam.w - 2 || y > cam.h - 2) return null;
  return [x, y];
}
function camGray(cam, x, y) {
  const w = cam.w, x0 = x | 0, y0 = y | 0, fx = x - x0, fy = y - y0, p = (y0 * w + x0) * 3, pr = p + w * 3, r = cam.rgb;
  const g = (q) => 0.299 * r[q] + 0.587 * r[q + 1] + 0.114 * r[q + 2];
  return (g(p) * (1 - fx) + g(p + 3) * fx) * (1 - fy) + (g(pr) * (1 - fx) + g(pr + 3) * fx) * fy;
}
// касательный базис в направлении d: восток, север
function tangentBasis(d) {
  const lon = Math.atan2(d[0], d[1]), lat = Math.asin(Math.max(-1, Math.min(1, d[2])));
  const e = [Math.cos(lon), -Math.sin(lon), 0], n = [-Math.sin(lat) * Math.sin(lon), -Math.sin(lat) * Math.cos(lon), Math.cos(lat)];
  return [e, n];
}
// патч size×size с шагом step (рад) вокруг d, смещённый на (ox, oy) пикселей патча
function renderPatch(cam, f, W2C, d, E, Nn, step, size, ox, oy) {
  const out = new Float32Array(size * size), c = (size - 1) / 2; let bad = 0;
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
    const a = (x - c + ox) * step, b = -(y - c + oy) * step;
    const v = [d[0] + a * E[0] + b * Nn[0], d[1] + a * E[1] + b * Nn[1], d[2] + a * E[2] + b * Nn[2]];
    const p = projCam(cam, f, W2C, v);
    if (!p) { out[y * size + x] = NaN; bad++; } else out[y * size + x] = camGray(cam, p[0], p[1]);
  }
  return bad > size * size * 0.2 ? null : out;
}
// структура патча: нормаль к преобладающему краю (восток, север) и отношение собственных чисел (0 — чистый край)
function patchEdge(A, size, inner) {
  const off = (size - inner) / 2; let jxx = 0, jxy = 0, jyy = 0;
  for (let y = off + 1; y < off + inner - 1; y++) for (let x = off + 1; x < off + inner - 1; x++) {
    const gx = A[y * size + x + 1] - A[y * size + x - 1], gy = A[(y + 1) * size + x] - A[(y - 1) * size + x];
    if (gx !== gx || gy !== gy) return null;
    jxx += gx * gx; jxy += gx * gy; jyy += gy * gy;
  }
  const tr = jxx + jyy; if (!(tr > 1e-6)) return null;
  const dd = Math.sqrt((jxx - jyy) * (jxx - jyy) / 4 + jxy * jxy), l1 = tr / 2 + dd, l2 = tr / 2 - dd;
  // собственный вектор при l1 (в координатах патча: x — восток, y — вниз)
  let vx = jxy, vy = l1 - jxx; if (Math.hypot(vx, vy) < 1e-9) { vx = 1; vy = 0; }
  const nl = Math.hypot(vx, vy);
  return { n: [vx / nl, -vy / nl], ratio: l2 / l1 };
}
function nccShift(A, B, size, inner, R) {
  // ищем сдвиг (sx, sy) такой, что B(x + s) ≈ A(x); A — центральный квадрат inner×inner, B — весь патч
  const off = (size - inner) / 2, N = inner * inner;
  let ma = 0, va = 0; for (let y = 0; y < inner; y++) for (let x = 0; x < inner; x++) { const v = A[(y + off) * size + x + off]; if (v !== v) return null; ma += v; va += v * v; }
  ma /= N; va = va - N * ma * ma; if (!(va > N * 9)) return null; // мало текстуры
  let best = -2, bx = 0, by = 0; const sc = new Float32Array((2 * R + 1) * (2 * R + 1));
  for (let sy = -R; sy <= R; sy++) for (let sx = -R; sx <= R; sx++) {
    let mb = 0, vb = 0, ab = 0, ok = true;
    for (let y = 0; y < inner && ok; y++) for (let x = 0; x < inner; x++) {
      const bv = B[(y + off + sy) * size + x + off + sx]; if (bv !== bv) { ok = false; break; }
      const av = A[(y + off) * size + x + off]; mb += bv; vb += bv * bv; ab += av * bv;
    }
    let s = -2;
    if (ok) { mb /= N; const vv = vb - N * mb * mb; if (vv > 1e-6) s = (ab - N * ma * mb) / Math.sqrt(va * vv); }
    sc[(sy + R) * (2 * R + 1) + sx + R] = s;
    if (s > best) { best = s; bx = sx; by = sy; }
  }
  // субпиксель
  const at = (x, y) => (Math.abs(x) <= R && Math.abs(y) <= R ? sc[(y + R) * (2 * R + 1) + x + R] : -2);
  let dx = 0, dy = 0;
  { const a = at(bx - 1, by), c = at(bx + 1, by), den = a - 2 * best + c; if (a > -1.5 && c > -1.5 && den < -1e-6) dx = Math.max(-0.5, Math.min(0.5, 0.5 * (a - c) / den)); }
  { const a = at(bx, by - 1), c = at(bx, by + 1), den = a - 2 * best + c; if (a > -1.5 && c > -1.5 && den < -1e-6) dy = Math.max(-0.5, Math.min(0.5, 0.5 * (a - c) / den)); }
  // однозначность: второй максимум вдали от первого
  let second = -2; for (let sy = -R; sy <= R; sy++) for (let sx = -R; sx <= R; sx++) if (Math.abs(sx - bx) > 2 || Math.abs(sy - by) > 2) second = Math.max(second, sc[(sy + R) * (2 * R + 1) + sx + R]);
  return { sx: bx + dx, sy: by + dy, score: best, second };
}
function seamAlign(cams, f, seams, W, opts, report) {
  opts = opts || {};
  const { LW, LH, lab } = seams, pix = 2 * Math.PI / W, N = cams.length;
  const W2C = cams.map(c => m3T(c.C2W));
  const cellDir = (u, v) => dirFromLonLat((u + 0.5) / LW * 2 * Math.PI - Math.PI, Math.PI / 2 - (v + 0.5) / LH * Math.PI);
  // точки на швах (прореживание: каждые ~3 клетки)
  const samples = [], seenKeys = new Set(), stepCells = opts.seamStep || 2;
  for (let v = 1; v < LH - 1; v++) for (let u = 0; u < LW; u++) {
    const p = v * LW + u, a = lab[p]; if (a < 0) continue;
    for (const q of [v * LW + (u + 1) % LW, p + LW]) {
      const b = lab[q]; if (b < 0 || b === a) continue;
      // не чаще одной точки на клетку stepCells×stepCells для каждой пары кадров
      // (раньше брались только клетки с чётными u и v — горизонтальный шов на нечётной строке не получал ни одной точки)
      const key = ((v / stepCells | 0) * LW + (u / stepCells | 0)) * 4096 + Math.min(a, b) * 64 + Math.max(a, b);
      if (seenKeys.has(key)) continue; seenKeys.add(key);
      samples.push({ u, v, a, b });
    }
  }
  const cons = []; // {frame, u, v, e, n} — смещение в радианах (восток, север)
  const S1 = 40, I1 = 24; // размер патча и внутренней части
  let done = 0; const fail = { patch1: 0, tex1: 0, low1: 0, patch2: 0, tex2: 0, low2: 0, amb: 0, big: 0, edge: 0 };
  for (const smp of samples) {
    done++; if (report && (done & 63) === 0) report(0.8 * done / samples.length);
    const d = cellDir(smp.u, smp.v), [E, Nn] = tangentBasis(d), A = cams[smp.a], B = cams[smp.b];
    // грубо: шаг 3 пикселя кадра, поиск ±10 (= ±30 px кадра)
    const fp = 1 / f, R1 = 10, I0 = 16, step1 = fp * 3;
    const pa1 = renderPatch(A, f, W2C[smp.a], d, E, Nn, step1, I0 + 2 * R1, 0, 0), pb1 = renderPatch(B, f, W2C[smp.b], d, E, Nn, step1, I0 + 2 * R1, 0, 0);
    if (!pa1 || !pb1) { fail.patch1++; continue; }
    const r1 = nccShift(pa1, pb1, I0 + 2 * R1, I0, R1); if (!r1) { fail.tex1++; continue; } if (r1.score < 0.6) { fail.low1++; continue; }
    // точно: шаг 1 пиксель кадра, поиск ±4 вокруг грубого
    const step2 = fp, sc = 3, R2 = 4;
    const pa2 = renderPatch(A, f, W2C[smp.a], d, E, Nn, step2, I1 + 2 * R2, 0, 0), pb2 = renderPatch(B, f, W2C[smp.b], d, E, Nn, step2, I1 + 2 * R2, r1.sx * sc, r1.sy * sc);
    if (!pa2 || !pb2) { fail.patch2++; continue; }
    const r2 = nccShift(pa2, pb2, I1 + 2 * R2, I1, R2); if (!r2) { fail.tex2++; continue; } if (r2.score < 0.8) { fail.low2++; continue; }
    // прямой край (рамка телевизора, косяк, угол стены): вдоль края сдвиг не определён, а поперёк — точно.
    // Такие места раньше отбрасывались как неоднозначные, и именно на них оставались «ступеньки».
    const st = patchEdge(pa2, I1 + 2 * R2, I1);
    let nrm = null;
    if (r2.second > r2.score - 0.04) { if (!st || st.ratio > 0.2) { fail.amb++; continue; } nrm = st.n; fail.edge++; }
    else if (st && st.ratio < 0.06) { nrm = st.n; fail.edge++; }
    let tx = (r1.sx * sc + r2.sx) * step2, ty = -(r1.sy * sc + r2.sy) * step2; // радианы (восток, север)
    // у края вдоль него сдвиг произвольный — оставляем только поперечную часть
    if (nrm) { const sn = tx * nrm[0] + ty * nrm[1]; tx = sn * nrm[0]; ty = sn * nrm[1]; }
    if (Math.hypot(tx, ty) > 40 * fp) { fail.big++; continue; }
    cons.push({ u: smp.u, v: smp.v, a: smp.a, b: smp.b, tx, ty, w: r2.score, ne: nrm ? nrm[0] : 0, nn: nrm ? nrm[1] : 0, one: !!nrm });
  }
  // отбраковка выбросов: сравнение с медианой соседей по тому же шву
  const good = cons.filter((c, i) => {
    const nb = cons.filter(o => o !== c && ((o.a === c.a && o.b === c.b) || (o.a === c.b && o.b === c.a)) && Math.abs(o.v - c.v) <= 10 && Math.min(Math.abs(o.u - c.u), LW - Math.abs(o.u - c.u)) <= 10);
    const fp = 1 / f;
    // для «краевых» ограничений сравниваем только поперечную к краю составляющую
    const dist = (mx, my) => c.one ? Math.abs((mx - c.tx) * c.ne + (my - c.ty) * c.nn) : Math.hypot(mx - c.tx, my - c.ty);
    // одиночная точка (край среди гладких стен) — берём, если совпадение очень уверенное и сдвиг умеренный
    if (!nb.length) return c.w >= 0.92 && Math.hypot(c.tx, c.ty) < 15 * fp;
    if (nb.length < 2) return dist(nb[0].tx * (nb[0].a === c.a ? 1 : -1), nb[0].ty * (nb[0].a === c.a ? 1 : -1)) < 8 * fp;
    const xs = nb.map(o => o.tx * (o.a === c.a ? 1 : -1)).sort((p, q) => p - q), ys = nb.map(o => o.ty * (o.a === c.a ? 1 : -1)).sort((p, q) => p - q);
    return dist(xs[xs.length >> 1], ys[ys.length >> 1]) < 6 * fp;
  });
  // поля смещений на грубой сетке
  const GW = LW >> 1, GH = LH >> 1, fields = new Array(N).fill(null);
  const per = Array.from({ length: N }, () => []);
  for (const c of good) { per[c.a].push([c.u >> 1, c.v >> 1, -c.tx / 2, -c.ty / 2, c.w, c.one, c.ne, c.nn]); per[c.b].push([c.u >> 1, c.v >> 1, c.tx / 2, c.ty / 2, c.w, c.one, c.ne, c.nn]); }
  for (let i = 0; i < N; i++) {
    if (per[i].length < 3) continue;
    // область кадра: клетки, видимые в кадре
    const vis = new Uint8Array(GW * GH);
    for (let v = 0; v < GH; v++) for (let u = 0; u < GW; u++) { const d = dirFromLonLat((u + 0.5) / GW * 2 * Math.PI - Math.PI, Math.PI / 2 - (v + 0.5) / GH * Math.PI); if (projCam(cams[i], f, W2C[i], d)) vis[v * GW + u] = 1; }
    // ограничения в клетке: Σ w·(d − t)² (полное) или Σ w·(n·d − n·t)² (только поперёк края) — матрица 2×2 и правая часть
    const axx = new Float32Array(GW * GH), axy = new Float32Array(GW * GH), ayy = new Float32Array(GW * GH), bx = new Float32Array(GW * GH), by = new Float32Array(GW * GH);
    for (const [u, v, e, n, w, one, ne, nn] of per[i]) {
      const k = v * GW + u;
      if (one) { const s = e * ne + n * nn; axx[k] += w * ne * ne; axy[k] += w * ne * nn; ayy[k] += w * nn * nn; bx[k] += w * s * ne; by[k] += w * s * nn; }
      else { axx[k] += w; ayy[k] += w; bx[k] += w * e; by[k] += w * n; }
    }
    let De = new Float32Array(GW * GH), Dn = new Float32Array(GW * GH);
    const wc = 20, eps = 0.002;
    for (let it = 0; it < (opts.alignIters || 600); it++) {
      for (let v = 0; v < GH; v++) for (let u = 0; u < GW; u++) {
        const k = v * GW + u; if (!vis[k]) continue;
        let se = 0, sn = 0, sw = 0;
        const nbs = [v * GW + (u + 1) % GW, v * GW + (u + GW - 1) % GW, v > 0 ? k - GW : -1, v < GH - 1 ? k + GW : -1];
        for (const q of nbs) { if (q < 0 || !vis[q]) continue; se += De[q]; sn += Dn[q]; sw += 1; }
        // ((sw + eps)·I + wc·A)·d = Σ соседей + wc·b
        const m00 = sw + eps + wc * axx[k], m01 = wc * axy[k], m11 = sw + eps + wc * ayy[k], r0 = se + wc * bx[k], r1 = sn + wc * by[k];
        const det = m00 * m11 - m01 * m01;
        De[k] = (m11 * r0 - m01 * r1) / det; Dn[k] = (m00 * r1 - m01 * r0) / det;
      }
    }
    fields[i] = { GW, GH, de: De, dn: Dn };
    if (report) report(0.8 + 0.2 * (i + 1) / N);
  }
  if (opts.debugAlign) console.log('align fail', JSON.stringify(fail));
  if (opts.debugAlignBox) { const [u0, v0, u1, v1] = opts.debugAlignBox; for (const c of cons) if (c.u >= u0 && c.u <= u1 && c.v >= v0 && c.v <= v1) console.log("con", c.u, c.v, c.a, c.b, (c.tx * f).toFixed(1), (c.ty * f).toFixed(1), c.one ? "edge" : "full", good.includes(c) ? "ok" : "rejected", c.w.toFixed(2)); }
  return { fields, constraints: good.length, tried: samples.length, raw: cons.length };
}
function fieldAt(F, u, v) { // u,v в долях (0..1)
  const x = u * F.GW - 0.5, y = Math.max(0, Math.min(F.GH - 1.001, v * F.GH - 0.5));
  const x0 = Math.floor(x), y0 = y | 0, fx = x - x0, fy = y - y0, xa = ((x0 % F.GW) + F.GW) % F.GW, xb = (xa + 1) % F.GW, G = F.GW;
  const bl = (A) => (A[y0 * G + xa] * (1 - fx) + A[y0 * G + xb] * fx) * (1 - fy) + (A[(y0 + 1) * G + xa] * (1 - fx) + A[(y0 + 1) * G + xb] * fx) * fy;
  return [bl(F.de), bl(F.dn)];
}

// ───────────────────────── сшивка в equirectangular ─────────────────────────
function lowBand(cam) {
  const D = 8, w = cam.w, h = cam.h, lw = Math.ceil(w / D), lh = Math.ceil(h / D);
  const a = new Float32Array(lw * lh * 3);
  for (let y = 0; y < lh; y++) for (let x = 0; x < lw; x++) {
    let r = 0, g = 0, b = 0, n = 0;
    for (let yy = y * D; yy < Math.min(h, y * D + D); yy++) for (let xx = x * D; xx < Math.min(w, x * D + D); xx++) { const p = (yy * w + xx) * 3; r += SRGB2LIN[cam.rgb[p]]; g += SRGB2LIN[cam.rgb[p + 1]]; b += SRGB2LIN[cam.rgb[p + 2]]; n++; }
    const q = (y * lw + x) * 3; a[q] = r / n; a[q + 1] = g / n; a[q + 2] = b / n;
  }
  // гаусс σ≈2.5 (≈20 px в полном разрешении), отражение на краях
  const sig = 2.5, rad = 7, k = []; let ks = 0;
  for (let i = -rad; i <= rad; i++) { const v = Math.exp(-i * i / (2 * sig * sig)); k.push(v); ks += v; }
  for (let i = 0; i < k.length; i++) k[i] /= ks;
  const tmp = new Float32Array(a.length);
  const cl = (v, n) => (v < 0 ? -v : v >= n ? 2 * n - 2 - v : v);
  for (let y = 0; y < lh; y++) for (let x = 0; x < lw; x++) for (let c = 0; c < 3; c++) {
    let s = 0; for (let i = -rad; i <= rad; i++) s += k[i + rad] * a[(y * lw + Math.max(0, Math.min(lw - 1, cl(x + i, lw)))) * 3 + c]; tmp[(y * lw + x) * 3 + c] = s;
  }
  for (let y = 0; y < lh; y++) for (let x = 0; x < lw; x++) for (let c = 0; c < 3; c++) {
    let s = 0; for (let i = -rad; i <= rad; i++) s += k[i + rad] * tmp[(Math.max(0, Math.min(lh - 1, cl(y + i, lh))) * lw + x) * 3 + c]; a[(y * lw + x) * 3 + c] = s;
  }
  return { lw, lh, D, a };
}

// Дополнительно размытая копия низкой полосы (σ в клетках сетки D): для очень низких частот (экспозиция).
function blurLow(lb, sig) {
  const { lw, lh } = lb, a = lb.a, rad = Math.ceil(sig * 2.5), k = []; let ks = 0;
  for (let i = -rad; i <= rad; i++) { const v = Math.exp(-i * i / (2 * sig * sig)); k.push(v); ks += v; }
  for (let i = 0; i < k.length; i++) k[i] /= ks;
  const tmp = new Float32Array(a.length), out = new Float32Array(a.length);
  const cl = (v, n) => Math.max(0, Math.min(n - 1, v));
  for (let y = 0; y < lh; y++) for (let x = 0; x < lw; x++) for (let c = 0; c < 3; c++) { let s = 0; for (let i = -rad; i <= rad; i++) s += k[i + rad] * a[(y * lw + cl(x + i, lw)) * 3 + c]; tmp[(y * lw + x) * 3 + c] = s; }
  for (let y = 0; y < lh; y++) for (let x = 0; x < lw; x++) for (let c = 0; c < 3; c++) { let s = 0; for (let i = -rad; i <= rad; i++) s += k[i + rad] * tmp[(cl(y + i, lh) * lw + x) * 3 + c]; out[(y * lw + x) * 3 + c] = s; }
  return { lw, lh, D: lb.D, a: out };
}
function compose(cams, f, gains, W, progress, depth, seams, opts, align) {
  opts = opts || {};
  const H = W / 2, rgba = new Uint8ClampedArray(W * H * 4), covered = new Uint8Array(W * H);
  const sinL = new Float64Array(W), cosL = new Float64Array(W);
  for (let u = 0; u < W; u++) { const lon = (u + 0.5) / W * 2 * Math.PI - Math.PI; sinL[u] = Math.sin(lon); cosL[u] = Math.cos(lon); }
  const iDmax = depth ? Math.max(...depth.iD) : 0;
  const info = cams.map(c => {
    const W2C = m3T(c.C2W);
    const ctr = c.center || [0, 0, 0], cn = Math.hypot(ctr[0], ctr[1], ctr[2]);
    const half = Math.atan(Math.hypot(c.cx + 1, c.cy + 1) / f) + (depth ? Math.min(0.6, Math.asin(Math.min(1, iDmax * cn)) * 1.2) : 0);
    const fw = camForward(c.C2W), ll = lonLatFromDir(fw);
    const ci = cams.indexOf(c);
    // ширина переходов на шве — в пикселях панорамы (детали ~8 px, средние частоты ~20 px), независимо от сетки швов
    const cellPx = seams ? W / seams.LW : 1;
    const sw = seams ? { hi: seamWeights(seams, ci, Math.max(0.5, (opts.seamHiPx || 8) / cellPx)), lo: seamWeights(seams, ci, Math.max(0.8, (opts.seamLoPx || 20) / cellPx)) } : null;
    if (sw) sw.any = true;
    const lb = c.low || (c.low = lowBand(c)), vb = c.vlow || (c.vlow = blurLow(lb, 10));
    return { W2C, ctr, sw, par: depth && cn > 1e-9, czMin: Math.cos(Math.min(half + 0.02, Math.PI / 2 - 0.01)), latC: ll[1], half: half + 0.03, lb, vb };
  });
  const BH0 = 64, iDband = depth ? new Float32Array(W * BH0) : null;
  const BH = BH0;
  // очень низкая полоса (общая яркость) — по всем кадрам на грубой сетке сферы; при швах её не берём из полного прохода
  let VS = null, VW = 0, VH = 0;
  if (seams && opts.vband) {
    VW = Math.max(256, W >> 3); VH = VW >> 1; VS = new Float32Array(VW * VH * 3);
    const vw = new Float32Array(VW * VH);
    cams.forEach((cam, ci) => {
      const M = info[ci].W2C, g = gains[ci], vb = info[ci].vb, k1c = cam.k1 || 0, iR0 = 1 / ((cam.R0 || 1) * (cam.R0 || 1));
      for (let v = 0; v < VH; v++) {
        const lat = Math.PI / 2 - (v + 0.5) / VH * Math.PI, cl = Math.cos(lat), sl = Math.sin(lat);
        for (let u = 0; u < VW; u++) {
          const lon = (u + 0.5) / VW * 2 * Math.PI - Math.PI, dx = cl * Math.sin(lon), dy = cl * Math.cos(lon), dz = sl;
          const cz = M[6] * dx + M[7] * dy + M[8] * dz; if (cz <= 1e-6) continue;
          let x = f * (M[0] * dx + M[1] * dy + M[2] * dz) / cz, y = f * (M[3] * dx + M[4] * dy + M[5] * dz) / cz;
          if (Math.abs(x) > 1.4 * cam.cx || Math.abs(y) > 1.4 * cam.cy) continue; // далеко за кадром: обратная дисторсия там не определена
          if (k1c) { const xu = x, yu = y; for (let it = 0; it < 4; it++) { const s2 = 1 + k1c * (x * x + y * y) * iR0; x = xu / s2; y = yu / s2; } }
          x += cam.cx; y += cam.cy; if (x < 0 || y < 0 || x > cam.w - 1 || y > cam.h - 1) continue;
          if (cam.trimY && y > cam.trimY) continue;
          const w = (1 - Math.abs((x - cam.cx) / (cam.cx + 0.5))) * (1 - Math.abs((y - cam.cy) / (cam.cy + 0.5))); if (w <= 0) continue;
          let lx = Math.max(0, Math.min(vb.lw - 1.001, (x + 0.5) / vb.D - 0.5)), ly = Math.max(0, Math.min(vb.lh - 1.001, (y + 0.5) / vb.D - 0.5));
          const x0 = lx | 0, y0 = ly | 0, fx = lx - x0, fy = ly - y0, q = (y0 * vb.lw + x0) * 3, qr = q + vb.lw * 3, a = vb.a, k = v * VW + u;
          for (let c = 0; c < 3; c++) VS[k * 3 + c] += w * g[c] * ((a[q + c] * (1 - fx) + a[q + 3 + c] * fx) * (1 - fy) + (a[qr + c] * (1 - fx) + a[qr + 3 + c] * fx) * fy);
          vw[k] += w;
        }
      }
    });
    for (let k = 0; k < VW * VH; k++) if (vw[k] > 0) for (let c = 0; c < 3; c++) VS[k * 3 + c] /= vw[k];
  }
  const vsAt = (u, v, out) => {
    let x = (u + 0.5) / W * VW - 0.5, y = Math.max(0, Math.min(VH - 1.001, (v + 0.5) / H * VH - 0.5));
    const x0 = Math.floor(x), y0 = y | 0, fx = x - x0, fy = y - y0, xa = ((x0 % VW) + VW) % VW, xb = (xa + 1) % VW;
    for (let c = 0; c < 3; c++) out[c] = (VS[(y0 * VW + xa) * 3 + c] * (1 - fx) + VS[(y0 * VW + xb) * 3 + c] * fx) * (1 - fy) + (VS[((y0 + 1) * VW + xa) * 3 + c] * (1 - fx) + VS[((y0 + 1) * VW + xb) * 3 + c] * fx) * fy;
  };
  const v3 = [0, 0, 0];
  const NA = 12, acc = new Float32Array(W * BH * NA);
  for (let v0 = 0; v0 < H; v0 += BH) {
    const v1 = Math.min(H, v0 + BH); acc.fill(0);
    // обратная глубина зависит только от направления — считаем один раз на полосу
    if (iDband) for (let v = v0; v < v1; v++) { const lat = Math.PI / 2 - (v + 0.5) / H * Math.PI; for (let u = 0; u < W; u++) iDband[(v - v0) * W + u] = depthAt(depth, (u + 0.5) / W * 2 * Math.PI - Math.PI, lat); }
    for (let ci = 0; ci < cams.length; ci++) {
      const cam = cams[ci], inf = info[ci], M = inf.W2C, g = gains[ci], lb = inf.lb, va = inf.vb.a, rgb = cam.rgb, cw = cam.w, ch = cam.h;
      const latTop = Math.PI / 2 - (v0 + 0.5) / H * Math.PI, latBot = Math.PI / 2 - (v1 - 0.5) / H * Math.PI;
      if (latBot > inf.latC + inf.half || latTop < inf.latC - inf.half) continue;
      const icx = 1 / (cam.cx + 0.5), icy = 1 / (cam.cy + 0.5), k1c = cam.k1 || 0, iR0 = 1 / ((cam.R0 || 1) * (cam.R0 || 1));
      for (let v = v0; v < v1; v++) {
        const lat = Math.PI / 2 - (v + 0.5) / H * Math.PI, cl = Math.cos(lat), sl = Math.sin(lat);
        const A = M[6] * cl, B = M[7] * cl, C = M[8] * sl, Rr = Math.hypot(A, B);
        const iDoff = (v - v0) * W;
        const pc = inf.ctr, par = inf.par, SW = inf.sw, AF = align && align.fields[ci];
        const slat = sl, clat = cl;
        let sRow0 = 0, sRow1 = 0, sFy = 0, LWs = 0;
        if (SW) { LWs = seams.LW; let sy = (v + 0.5) / H * seams.LH - 0.5; if (sy < 0) sy = 0; if (sy > seams.LH - 1) sy = seams.LH - 1; const y0 = Math.min(seams.LH - 2, Math.floor(sy)); sFy = sy - y0; sRow0 = y0 * LWs; sRow1 = sRow0 + LWs; }
        let uStart, uCount;
        if (Rr < 1e-9) { if (C > inf.czMin) { uStart = 0; uCount = W; } else continue; }
        else {
          const ratio = (inf.czMin - C) / Rr;
          if (ratio >= 1) continue;
          if (ratio <= -1) { uStart = 0; uCount = W; }
          else {
            const lam0 = Math.atan2(A, B), hw = Math.acos(ratio);
            const uLo = Math.ceil((lam0 - hw + Math.PI) / (2 * Math.PI) * W - 0.5), uHi = Math.floor((lam0 + hw + Math.PI) / (2 * Math.PI) * W - 0.5);
            uStart = uLo; uCount = Math.min(W, uHi - uLo + 1);
          }
        }
        const rowBase = (v - v0) * W;
        for (let kk = 0; kk < uCount; kk++) {
          let u = uStart + kk; u = ((u % W) + W) % W;
          let dx = cl * sinL[u], dy = cl * cosL[u], dz = sl;
          let hiS = 1, loS = 1;
          if (SW) {
            // вне своей области шва кадр не участвует — пропускаем дорогую выборку
            const sx = (u + 0.5) / W * LWs - 0.5, x0s = Math.floor(sx), fxs = sx - x0s, xa = ((x0s % LWs) + LWs) % LWs, xb = (xa + 1) % LWs, lo = SW.lo, hi = SW.hi;
            loS = lo ? (lo[sRow0 + xa] * (1 - fxs) + lo[sRow0 + xb] * fxs) * (1 - sFy) + (lo[sRow1 + xa] * (1 - fxs) + lo[sRow1 + xb] * fxs) * sFy : 0;
            hiS = hi ? (hi[sRow0 + xa] * (1 - fxs) + hi[sRow0 + xb] * fxs) * (1 - sFy) + (hi[sRow1 + xa] * (1 - fxs) + hi[sRow1 + xb] * fxs) * sFy : 0;
            if (loS < 0.002 && hiS < 0.002 && SW.any) continue;
          }
          if (par) { const q = iDband[iDoff + u]; dx -= q * pc[0]; dy -= q * pc[1]; dz -= q * pc[2]; }
          if (AF) { // локальное выравнивание: сдвиг направления в касательной плоскости
            const t = fieldAt(AF, (u + 0.5) / W, (v + 0.5) / H), te = t[0], tn = t[1];
            const sL = sinL[u], cL = cosL[u];
            dx += te * cL - tn * slat * sL; dy += -te * sL - tn * slat * cL; dz += tn * clat;
          }
          const cz = M[6] * dx + M[7] * dy + M[8] * dz; if (cz <= 1e-6) continue;
          const iz = f / cz;
          let x = (M[0] * dx + M[1] * dy + M[2] * dz) * iz, y = (M[3] * dx + M[4] * dy + M[5] * dz) * iz;
          if (Math.abs(x) > 1.4 * cam.cx || Math.abs(y) > 1.4 * cam.cy) continue;
          if (k1c) { const xu = x, yu = y; for (let it = 0; it < 4; it++) { const s2 = 1 + k1c * (x * x + y * y) * iR0; x = xu / s2; y = yu / s2; } }
          x += cam.cx; y += cam.cy;
          if (x < 0 || y < 0 || x > cw - 1 || y > ch - 1) continue;
          if (cam.trimY && y > cam.trimY) continue;
          const wx = 1 - Math.abs((x - cam.cx) * icx), wy = 1 - Math.abs((y - cam.cy) * icy);
          const w = wx * wy; if (w <= 0) continue;
          // полный кадр (линейный) билинейно
          let x0 = x | 0, y0 = y | 0; if (x0 >= cw - 1) x0 = cw - 2; if (y0 >= ch - 1) y0 = ch - 2;
          const fx = x - x0, fy = y - y0, p = (y0 * cw + x0) * 3, pr = p + cw * 3;
          const w00 = (1 - fx) * (1 - fy), w10 = fx * (1 - fy), w01 = (1 - fx) * fy, w11 = fx * fy;
          const r = SRGB2LIN[rgb[p]] * w00 + SRGB2LIN[rgb[p + 3]] * w10 + SRGB2LIN[rgb[pr]] * w01 + SRGB2LIN[rgb[pr + 3]] * w11;
          const gg = SRGB2LIN[rgb[p + 1]] * w00 + SRGB2LIN[rgb[p + 4]] * w10 + SRGB2LIN[rgb[pr + 1]] * w01 + SRGB2LIN[rgb[pr + 4]] * w11;
          const bb = SRGB2LIN[rgb[p + 2]] * w00 + SRGB2LIN[rgb[p + 5]] * w10 + SRGB2LIN[rgb[pr + 2]] * w01 + SRGB2LIN[rgb[pr + 5]] * w11;
          // низкие частоты
          let lx = (x + 0.5) / lb.D - 0.5, ly = (y + 0.5) / lb.D - 0.5;
          if (lx < 0) lx = 0; if (ly < 0) ly = 0; if (lx > lb.lw - 1) lx = lb.lw - 1; if (ly > lb.lh - 1) ly = lb.lh - 1;
          let lx0 = lx | 0, ly0 = ly | 0; if (lx0 >= lb.lw - 1) lx0 = Math.max(0, lb.lw - 2); if (ly0 >= lb.lh - 1) ly0 = Math.max(0, lb.lh - 2);
          const lfx = lx - lx0, lfy = ly - ly0, la = lb.a, q = (ly0 * lb.lw + lx0) * 3, qr = q + lb.lw * 3;
          const l00 = (1 - lfx) * (1 - lfy), l10 = lfx * (1 - lfy), l01 = (1 - lfx) * lfy, l11 = lfx * lfy;
          const Lr = la[q] * l00 + la[q + 3] * l10 + la[qr] * l01 + la[qr + 3] * l11;
          const Lg = la[q + 1] * l00 + la[q + 4] * l10 + la[qr + 1] * l01 + la[qr + 4] * l11;
          const Lb = la[q + 2] * l00 + la[q + 5] * l10 + la[qr + 2] * l01 + la[qr + 5] * l11;
          const Vr = va[q] * l00 + va[q + 3] * l10 + va[qr] * l01 + va[qr + 3] * l11;
          const Vg = va[q + 1] * l00 + va[q + 4] * l10 + va[qr + 1] * l01 + va[qr + 4] * l11;
          const Vb = va[q + 2] * l00 + va[q + 5] * l10 + va[qr + 2] * l01 + va[qr + 5] * l11;
          const o = (rowBase + u) * NA;
          // три полосы: очень низкая (экспозиция) — широкое растушёвывание; средняя — по размытому шву; детали — по шву
          const w2 = w * w, w4 = w2 * w2;
          let wv = w, wl = w2 * w + 1e-30, wh = w4 * w4 * w2 + 1e-30;
          if (SW) { const edge = Math.min(1, w * 12); wl = w * (loS + 1e-4) + 1e-30; wh = (hiS * edge + 1e-6 * wh) + 1e-30; }
          if (SW && !VS) {
            acc[o + 3] += wv; acc[o + 4] += wl * g[0] * Lr; acc[o + 5] += wl * g[1] * Lg; acc[o + 6] += wl * g[2] * Lb; acc[o + 7] += wl;
          } else {
            acc[o] += wv * g[0] * Vr; acc[o + 1] += wv * g[1] * Vg; acc[o + 2] += wv * g[2] * Vb; acc[o + 3] += wv;
            acc[o + 4] += wl * g[0] * (Lr - Vr); acc[o + 5] += wl * g[1] * (Lg - Vg); acc[o + 6] += wl * g[2] * (Lb - Vb); acc[o + 7] += wl;
          }
          acc[o + 8] += wh * g[0] * (r - Lr); acc[o + 9] += wh * g[1] * (gg - Lg); acc[o + 10] += wh * g[2] * (bb - Lb); acc[o + 11] += wh;
        }
      }
    }
    for (let v = v0; v < v1; v++) for (let u = 0; u < W; u++) {
      const o = ((v - v0) * W + u) * NA, px = v * W + u, d = px * 4;
      const vw = acc[o + 3];
      if (vw > 0) {
        const lw = acc[o + 7], hw = acc[o + 11];
        if (VS && compose.dbg === 'vs') { vsAt(u, v, v3); rgba[d] = lin2srgb(v3[0]); rgba[d + 1] = lin2srgb(v3[1]); rgba[d + 2] = lin2srgb(v3[2]); }
        else if (VS && compose.dbg === 'mid') { vsAt(u, v, v3); rgba[d] = lin2srgb(v3[0] + acc[o + 4] / lw); rgba[d + 1] = lin2srgb(v3[1] + acc[o + 5] / lw); rgba[d + 2] = lin2srgb(v3[2] + acc[o + 6] / lw); }
        else if (VS) { vsAt(u, v, v3); rgba[d] = lin2srgb(v3[0] + acc[o + 4] / lw + acc[o + 8] / hw); rgba[d + 1] = lin2srgb(v3[1] + acc[o + 5] / lw + acc[o + 9] / hw); rgba[d + 2] = lin2srgb(v3[2] + acc[o + 6] / lw + acc[o + 10] / hw); }
        else { rgba[d] = lin2srgb(acc[o] / vw + acc[o + 4] / lw + acc[o + 8] / hw); rgba[d + 1] = lin2srgb(acc[o + 1] / vw + acc[o + 5] / lw + acc[o + 9] / hw); rgba[d + 2] = lin2srgb(acc[o + 2] / vw + acc[o + 6] / lw + acc[o + 10] / hw); }
        covered[px] = 1;
      }
      rgba[d + 3] = 255;
    }
    if (progress) progress(v1 / H);
  }
  return { rgba, covered, W, H };
}

// ───────────────────────── заливка пустот (push–pull, с переходом через 180°) ─────────────────────────
function fillHoles(rgba, covered, W, H) {
  let holes = 0; for (let i = 0; i < covered.length; i++) if (!covered[i]) holes++;
  if (!holes) return 0;
  const sw = Math.min(W, 1024), s = W / sw, sh = sw / 2;
  // уровень 0 низкого разрешения: средние только по покрытым пикселям
  const lv = [{ w: sw, h: sh, c: new Float32Array(sw * sh * 3), wt: new Float32Array(sw * sh) }];
  const L0 = lv[0];
  for (let v = 0; v < H; v++) { const y = Math.min(sh - 1, (v / s) | 0); for (let u = 0; u < W; u++) { if (!covered[v * W + u]) continue; const x = Math.min(sw - 1, (u / s) | 0), q = y * sw + x, d = (v * W + u) * 4; L0.c[q * 3] += SRGB2LIN[rgba[d]]; L0.c[q * 3 + 1] += SRGB2LIN[rgba[d + 1]]; L0.c[q * 3 + 2] += SRGB2LIN[rgba[d + 2]]; L0.wt[q] += 1; } }
  const full = s * s;
  for (let q = 0; q < sw * sh; q++) { if (L0.wt[q] > 0) { for (let c = 0; c < 3; c++) L0.c[q * 3 + c] /= L0.wt[q]; } L0.wt[q] = Math.min(1, L0.wt[q] / full); }
  // размытая карта покрытия — для мягкого перехода от снятого к залитому
  const covB = new Float32Array(sw * sh), tmpB = new Float32Array(sw * sh), rb = 4;
  for (let y = 0; y < sh; y++) for (let x = 0; x < sw; x++) { let a = 0; for (let k = -rb; k <= rb; k++) a += L0.wt[y * sw + ((x + k + sw) % sw)]; tmpB[y * sw + x] = a / (2 * rb + 1); }
  for (let y = 0; y < sh; y++) for (let x = 0; x < sw; x++) { let a = 0; for (let k = -rb; k <= rb; k++) a += tmpB[Math.max(0, Math.min(sh - 1, y + k)) * sw + x]; covB[y * sw + x] = a / (2 * rb + 1); }
  while (lv[lv.length - 1].w > 2) {
    const a = lv[lv.length - 1], w = a.w >> 1, h = Math.max(1, a.h >> 1), b = { w, h, c: new Float32Array(w * h * 3), wt: new Float32Array(w * h) };
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      let ws = 0, cs = [0, 0, 0];
      for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) { const yy = Math.min(a.h - 1, 2 * y + dy), q = yy * a.w + 2 * x + dx, ww = a.wt[q]; ws += ww; for (let c = 0; c < 3; c++) cs[c] += a.c[q * 3 + c] * ww; }
      const q = y * w + x; b.wt[q] = Math.min(1, ws); if (ws > 0) for (let c = 0; c < 3; c++) b.c[q * 3 + c] = cs[c] / ws;
    }
    lv.push(b);
  }
  const sampleWrap = (L, x, y, c) => {
    x = x - 0.5; y = Math.max(0, Math.min(L.h - 1, y - 0.5));
    const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, y1 = Math.min(L.h - 1, y0 + 1);
    const xa = ((x0 % L.w) + L.w) % L.w, xb = (xa + 1) % L.w;
    return (L.c[(y0 * L.w + xa) * 3 + c] * (1 - fx) + L.c[(y0 * L.w + xb) * 3 + c] * fx) * (1 - fy) + (L.c[(y1 * L.w + xa) * 3 + c] * (1 - fx) + L.c[(y1 * L.w + xb) * 3 + c] * fx) * fy;
  };
  for (let l = lv.length - 2; l >= 0; l--) {
    const a = lv[l], b = lv[l + 1], sx = b.w / a.w, sy = b.h / a.h;
    for (let y = 0; y < a.h; y++) for (let x = 0; x < a.w; x++) {
      const q = y * a.w + x, ww = a.wt[q]; if (ww >= 1) continue;
      for (let c = 0; c < 3; c++) a.c[q * 3 + c] = ww * a.c[q * 3 + c] + (1 - ww) * sampleWrap(b, (x + 0.5) * sx, (y + 0.5) * sy, c);
      a.wt[q] = 1;
    }
  }
  const sampleCov = (x, y) => { x -= 0.5; y = Math.max(0, Math.min(sh - 1, y - 0.5)); const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0, y1 = Math.min(sh - 1, y0 + 1), xa = ((x0 % sw) + sw) % sw, xb = (xa + 1) % sw; return (covB[y0 * sw + xa] * (1 - fx) + covB[y0 * sw + xb] * fx) * (1 - fy) + (covB[y1 * sw + xa] * (1 - fx) + covB[y1 * sw + xb] * fx) * fy; };
  for (let v = 0; v < H; v++) for (let u = 0; u < W; u++) {
    const px = v * W + u, d = px * 4, x = (u + 0.5) / s, y = (v + 0.5) / s;
    if (covered[px]) {
      const cb = sampleCov(x, y); if (cb >= 0.98) continue;
      const a = Math.max(0, Math.min(1, (cb - 0.5) / 0.48)), al = a * a * (3 - 2 * a);
      if (al >= 0.999) continue;
      for (let c = 0; c < 3; c++) rgba[d + c] = lin2srgb(SRGB2LIN[rgba[d + c]] * (0.35 + 0.65 * al) + sampleWrap(L0, x, y, c) * 0.65 * (1 - al));
      continue;
    }
    rgba[d] = lin2srgb(sampleWrap(L0, x, y, 0)); rgba[d + 1] = lin2srgb(sampleWrap(L0, x, y, 1)); rgba[d + 2] = lin2srgb(sampleWrap(L0, x, y, 2)); rgba[d + 3] = 255;
  }
  return holes;
}

function downscaleRGBA(rgba, W, H, PW) {
  if (PW >= W) return { rgba: rgba, w: W, h: H };
  const PH = PW / 2, out = new Uint8ClampedArray(PW * PH * 4), s = W / PW;
  for (let y = 0; y < PH; y++) for (let x = 0; x < PW; x++) {
    let r = 0, g = 0, b = 0, n = 0;
    const ya = Math.floor(y * s), yb = Math.min(H, Math.floor((y + 1) * s)), xa = Math.floor(x * s), xb = Math.min(W, Math.floor((x + 1) * s));
    for (let yy = ya; yy < yb; yy++) for (let xx = xa; xx < xb; xx++) { const d = (yy * W + xx) * 4; r += SRGB2LIN[rgba[d]]; g += SRGB2LIN[rgba[d + 1]]; b += SRGB2LIN[rgba[d + 2]]; n++; }
    const o = (y * PW + x) * 4; out[o] = lin2srgb(r / n); out[o + 1] = lin2srgb(g / n); out[o + 2] = lin2srgb(b / n); out[o + 3] = 255;
  }
  return { rgba: out, w: PW, h: PH };
}
function downscaleMask(covered, W, H, PW) {
  // два канала: [доля «не снято» (залито или ИИ), доля «дорисовано ИИ»]
  const PH = PW / 2, out = new Uint8Array(PW * PH * 2), s = W / PW;
  for (let y = 0; y < PH; y++) for (let x = 0; x < PW; x++) {
    let real = 0, ai = 0, n = 0;
    const ya = Math.floor(y * s), yb = Math.min(H, Math.max(ya + 1, Math.floor((y + 1) * s))), xa = Math.floor(x * s), xb = Math.min(W, Math.max(xa + 1, Math.floor((x + 1) * s)));
    for (let yy = ya; yy < yb; yy++) for (let xx = xa; xx < xb; xx++) { const v = covered[yy * W + xx]; if (v === 1) real++; else if (v >= 2) ai++; n++; }
    out[(y * PW + x) * 2] = Math.round(255 * (n - real) / n); out[(y * PW + x) * 2 + 1] = Math.round(255 * ai / n);
  }
  return out;
}

// ───────────────────────── полный конвейер ─────────────────────────
// frames: [{w,h,rgb(Uint8Array RGB), C2W (гироскоп)}], opts: {f0, outW, maxW, previewW}
function stitchPanorama(frames, opts, report) {
  const t0 = Date.now();
  const say = (stage, frac, note) => report && report({ stage, frac, note });
  let f = opts.f0;
  const rand = rng(12345);
  const cams = frames.map((fr, idx) => ({
    idx, w: fr.w, h: fr.h, rgb: fr.rgb, cx: (fr.w - 1) / 2, cy: (fr.h - 1) / 2, R0: Math.hypot(fr.w, fr.h) / 2, k1: 0,
    C2W: m3copy(fr.C2W), C2Wprior: m3copy(fr.C2W), group: fr.group, omega: fr.omega && fr.omega.length === 3 ? Array.from(fr.omega) : null
  }));
  // 1. пирамиды и углы
  for (let i = 0; i < cams.length; i++) {
    const c = cams[i];
    c.pyr = buildPyramid(c.rgb, c.w, c.h, 4);
    const L0 = c.pyr.levels[0];
    const cell = Math.max(14, Math.round(Math.max(L0.w, L0.h) / 34));
    c.corners = detectCorners(L0, cell, 700).map(p => [(p[0] + 0.5) * c.pyr.k - 0.5, (p[1] + 0.5) * c.pyr.k - 0.5, p[2]]);
    say('features', (i + 1) / cams.length);
  }
  // 1б. фокус: калибровка по гироскопу (видеопоток телефонов часто обрезан, и паспортный угол обзора не подходит)
  let fCal = null;
  if (opts.calibrateFocal !== false && cams.length >= 8) {
    say('focal', 0);
    const fScan0 = f; fCal = calibrateFocal(cams, f, fr => say('focal', fr));
    if (fCal.pairs >= 4 && fCal.score > 0.02) f = fCal.f;
    if (opts.debugFocal) { const L = Math.max(cams[0].w, cams[0].h); console.log('focal scan', fCal.scan.map(([q, v]) => (q * fScan0 / L).toFixed(2) + ':' + v.toFixed(3)).join(' ')); if (opts.debugFocal === 'stop') throw new Error('stop'); }
    say('focal', 1, (f / Math.max(cams[0].w, cams[0].h)).toFixed(3));
  }
  // 2. кандидаты пар
  const cand = [];
  for (let i = 0; i < cams.length; i++) for (let j = i + 1; j < cams.length; j++) {
    const a = camForward(cams[i].C2W), b = camForward(cams[j].C2W);
    if (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] < Math.cos(100 * Math.PI / 180)) continue;
    const ov = Math.max(overlapFraction(cams[i], cams[j], f), overlapFraction(cams[j], cams[i], f));
    if (ov >= 0.06) cand.push({ i, j, ov });
  }
  // 3. грубое сопоставление (компенсация дрейфа гироскопа) → BA
  const runBA = (pairs, iters, cap) => {
    const all = []; for (const p of pairs) { const ms = p.matches, step = Math.max(1, ms.length / cap); for (let q = 0; q < ms.length; q += step) all.push([p.i, p.j, ms[q | 0]]); }
    const n = all.length, M = { n, i: new Int32Array(n), j: new Int32Array(n), xi: new Float64Array(n), yi: new Float64Array(n), xj: new Float64Array(n), yj: new Float64Array(n) };
    all.forEach(([i, j, m], k) => { M.i[k] = i; M.j[k] = j; M.xi[k] = m.xi - cams[i].cx; M.yi[k] = m.yi - cams[i].cy; M.xj[k] = m.xj - cams[j].cx; M.yj[k] = m.yj - cams[j].cy; });
    if (!n) return null;
    if (opts.pairWeights !== false) {
      // пара с сильным параллаксом (большой разброс относительно лучшего поворота) весит мало; число точек в паре почти не важно
      M.w = new Float64Array(n); M.hub = new Float64Array(n);
      const byPair = new Map(); all.forEach(([i, j], k) => { const key = i * 1000 + j; if (!byPair.has(key)) byPair.set(key, []); byPair.get(key).push(k); });
      for (const [key, ks] of byPair) {
        const i = M.i[ks[0]], j = M.j[ks[0]];
        const A = ks.map(k => camRay(cams[i], f, M.xi[k] + cams[i].cx, M.yi[k] + cams[i].cy)), B = ks.map(k => camRay(cams[j], f, M.xj[k] + cams[j].cx, M.yj[k] + cams[j].cy));
        const R = kabsch(A, B, A.map((_, q) => q));
        let s2 = 0; for (let q = 0; q < ks.length; q++) { const v = mv(R, A[q]), d = B[q]; s2 += (v[0] - d[0]) ** 2 + (v[1] - d[1]) ** 2 + (v[2] - d[2]) ** 2; }
        // пара = одно измерение относительного поворота с точностью σ (её собственный разброс); вклад ~ (Δ/σ)²
        const rmsPx = Math.sqrt(s2 / ks.length) * f, sig = Math.max(opts.pairSigMinPx || 2, rmsPx), wPair = opts.pairWeight || 2;
        for (const k of ks) { M.w[k] = wPair / ks.length / (sig * sig); M.hub[k] = 3 * sig; }
      }
    }
    // фокус откалиброван — держим его почти фиксированным (иначе связка может «убежать» в вырожденное f → ∞)
    const sigRel = opts.gyroRelDeg === 0 ? 0 : (opts.gyroRelDeg || 1) * Math.PI / 180;
    const sigUp = opts.gravityDeg === 0 ? 0 : (opts.gravityDeg || 1) * Math.PI / 180;
    const r = bundleAdjust(cams, M, f, iters, cams[0].k1, { sigF: opts.baSigF || 0.1, sigK: opts.baSigK || 1e-4, sigR: opts.baSigR || 0.2, sigRel, sigUp });
    if (opts.debugRel) { const rl = []; for (let c = 0; c + 1 < cams.length; c++) { const a = m3mul(m3T(cams[c].C2Wprior), cams[c + 1].C2Wprior), bb = m3mul(m3T(r.C2W[c]), r.C2W[c + 1]); rl.push((rotAngle(m3mul(bb, m3T(a))) * 57.3).toFixed(1)); } console.log('BA rel', sigRel.toFixed(3), rl.join(' ')); }
    const k1 = Math.abs(r.k1) < 0.004 ? 0 : r.k1;
    cams.forEach((c, k) => { c.C2W = r.C2W[k]; c.k1 = k1; }); f = r.f;
    return r;
  };
  let pairsA = [];
  const ptol = opts.parallaxTolerant !== false;
  cand.forEach((p, k) => { const r = matchPair(cams[p.i], cams[p.j], f, 'coarse', rand, ptol); if (r.matches.length) pairsA.push({ i: p.i, j: p.j, matches: r.matches }); say('match1', (k + 1) / cand.length); });
  say('ba1', 0); const ba1 = runBA(pairsA, 30, 40); say('ba1', 1, ba1 ? ba1.rms.toFixed(2) : '—');
  // 4. точное сопоставление от уточнённых поз → BA
  let pairsB = [];
  cand.forEach((p, k) => { const r = matchPair(cams[p.i], cams[p.j], f, 'fine', rand, ptol); if (r.matches.length) pairsB.push({ i: p.i, j: p.j, matches: r.matches }); say('match2', (k + 1) / cand.length); });
  if (pairsB.length < pairsA.length * 0.6) pairsB = pairsA.concat(pairsB.filter(p => !pairsA.some(q => q.i === p.i && q.j === p.j)));
  say('ba2', 0); let ba2 = runBA(pairsB, 40, 1e9);
  // отбрасываем выбросы по итоговой ошибке и повторяем
  if (ba2) {
    let k = 0; const thr = Math.max(2.5, 3 * ba2.rms);
    for (const p of pairsB) p.matches = p.matches.filter(() => ba2.per[k++] <= thr);
    pairsB = pairsB.filter(p => p.matches.length >= 6);
    ba2 = runBA(pairsB, 30, 70) || ba2;
  }
  say('ba2', 1, ba2 ? ba2.rms.toFixed(2) : '—');
  // 4а. бегущий затвор: только если кадры сняты в движении (есть угловая скорость)
  let rsTau = null;
  if (ba2 && opts.rollingShutter !== false && cams.some(c => c.omega && Math.hypot(c.omega[0], c.omega[1], c.omega[2]) > 0.05)) {
    say('rs', 0);
    for (const p of pairsB) for (const m of p.matches) { m.oxi = m.xi; m.oyi = m.yi; m.oxj = m.xj; m.oyj = m.yj; }
    const saveC = () => ({ C2W: cams.map(c => m3copy(c.C2W)), f });
    const setCoords = tau => { for (const p of pairsB) for (const m of p.matches) { const a = rsCorrectPoint(cams[p.i], f, m.oxi, m.oyi, tau), b = rsCorrectPoint(cams[p.j], f, m.oxj, m.oyj, tau); m.xi = a[0]; m.yi = a[1]; m.xj = b[0]; m.yj = b[1]; } };
    const rms0 = ba2.rms; let best = { tau: [0, 0], rms: rms0, st: saveC() }, tau = [0, 0];
    for (let it = 0; it < 8; it++) {
      const prev = tau; tau = estimateRS(cams, f, pairsB, tau); setCoords(tau);
      const r = runBA(pairsB, 15, 70); if (!r) break;
      if (r.rms < best.rms) best = { tau: tau.slice(), rms: r.rms, st: saveC() };
      say('rs', (it + 1) / 8);
      if (Math.hypot(tau[0] - prev[0], tau[1] - prev[1]) < 4e-7) break;
    }
    if (best.rms < 0.95 * rms0) {
      rsTau = best.tau; setCoords(rsTau); cams.forEach((c, k) => { c.C2W = best.st.C2W[k]; }); f = best.st.f;
      for (const c of cams) rsWarp(c, f, rsTau);
      ba2 = Object.assign({}, ba2, { rms: best.rms });
    } else { setCoords([0, 0]); cams.forEach((c, k) => { c.C2W = best.st.C2W[k]; }); f = best.st.f; }
    say('rs', 1, rsTau ? (Math.hypot(rsTau[0] * cams[0].w, rsTau[1] * cams[0].h) * 1000).toFixed(0) + ' мс' : 'не нужен');
    if (opts.debugRS) console.log('RS', JSON.stringify({ rms0, best: best.rms, tau: best.tau }));
  }
  if (opts.debugPairs) for (const p of pairsB) {
    // поворот по совпадениям этой пары (Кабш) против гироскопа
    const A = p.matches.map(m => camRay(cams[p.i], f, m.xi, m.yi)), B = p.matches.map(m => camRay(cams[p.j], f, m.xj, m.yj));
    const Rm = kabsch(A, B, A.map((_, k) => k)), Rg = m3mul(m3T(cams[p.j].C2Wprior), cams[p.i].C2Wprior);
    const dev = rotAngle(m3mul(Rm, m3T(Rg))) * 57.3;
    if (dev > 4 || opts.debugPairs === 'all') console.log('pair', p.i, p.j, 'n', p.matches.length, 'match-vs-gyro', dev.toFixed(1) + '°');
  }
  // 4б. параллакс: если телефон двигался (съёмка с рук), уточняем центры кадров и глубину сцены
  let pba = null, points = null;
  // съёмка «вокруг объектива» даёт ошибку поворотной модели ≲1 px — там параллакс-модель только добавляет свобод
  if (ba2 && opts.parallax === true) {
    const all = []; for (const p of pairsB) { const ms = p.matches, step = Math.max(1, ms.length / 120); for (let q = 0; q < ms.length; q += step) all.push([p.i, p.j, ms[q | 0]]); }
    const n = all.length, M = { n, i: new Int32Array(n), j: new Int32Array(n), xi: new Float64Array(n), yi: new Float64Array(n), xj: new Float64Array(n), yj: new Float64Array(n) };
    all.forEach(([i, j, m], k) => { const a = undistOff(cams[i], m.xi - cams[i].cx, m.yi - cams[i].cy), b = undistOff(cams[j], m.xj - cams[j].cx, m.yj - cams[j].cy); M.i[k] = i; M.j[k] = j; M.xi[k] = a[0]; M.yi[k] = a[1]; M.xj[k] = b[0]; M.yj[k] = b[1]; });
    say('parallax', 0);
    const r = bundleAdjustParallax(cams, M, f, 40, fr => say('parallax', fr));
    // считаем параллакс реальным, если ошибка заметно упала
    if (r.rms < 0.7 * ba2.rms) {
      pba = r; cams.forEach((c, k) => { c.C2W = r.C2W[k]; c.center = r.centers[k]; }); f = r.f;
      points = [];
      for (let k = 0; k < n; k++) {
        if (r.per[k] > Math.max(3, 3 * r.rms)) continue;
        const i = M.i[k], q = Math.exp(r.s[k]); if (q < 1e-4) continue;
        const w = rayW(cams[i].C2W, M.xi[k], M.yi[k], f), c = cams[i].center;
        points.push([c[0] + w[0] / q, c[1] + w[1] / q, c[2] + w[2] / q]);
      }
      // плотные точки вдоль эпиполярных линий: близкие предметы (витрины, колонны)
      say('depth', 0);
      const ep = epipolarPoints(cams, f, cand, fr => say('depth', fr));
      for (const p of ep) points.push(p);
      say('depth', 1, String(ep.length));
    }
    say('parallax', 1, (pba ? pba.rms : ba2.rms).toFixed(2));
  }
  // 4в. проверка по гироскопу: поворот между соседними по времени кадрами гироскоп знает точно (±1°).
  // Кадр, который разошёлся с обоими соседями больше чем на 5°, скорее всего сопоставлен ошибочно — убираем его пары.
  let suspicious = 0;
  {
    // относительный поворот в системе камеры (не зависит от общего дрейфа курса)
    const relDev = (a, b) => rotAngle(m3mul(m3mul(m3T(cams[a].C2W), cams[b].C2W), m3T(m3mul(m3T(cams[a].C2Wprior), cams[b].C2Wprior)))) * 180 / Math.PI;
    const bad = new Set();
    for (let k = 0; k < cams.length; k++) {
      const devs = []; if (k > 0) devs.push(relDev(k - 1, k)); if (k < cams.length - 1) devs.push(relDev(k, k + 1));
      if (devs.length && Math.min(...devs) > (opts.gyroCheckDeg || 5)) bad.add(k);
    }
    if (bad.size && bad.size < cams.length / 3) {
      suspicious = bad.size; if (opts.debugCheck) console.log("gyro-check bad", [...bad]);
      pairsB = pairsB.filter(p => !bad.has(p.i) && !bad.has(p.j));
    }
  }
  // 4г. кадры без пар (часто надир при съёмке с рук): дрейф гироскопа переносим с ближайших по времени связанных кадров
  {
    const lk = new Uint8Array(cams.length); for (const p of pairsB) { if (p.matches.length) { lk[p.i] = 1; lk[p.j] = 1; } }
    for (let k = 0; k < cams.length; k++) {
      if (lk[k]) continue;
      let a = k - 1; while (a >= 0 && !lk[a]) a--;
      let b = k + 1; while (b < cams.length && !lk[b]) b++;
      const cands = []; if (a >= 0) cands.push([a, k - a]); if (b < cams.length) cands.push([b, b - k]);
      if (!cands.length) continue;
      const M = new Float64Array(9);
      for (const [n, gap] of cands) { const corr = m3mul(cams[n].C2W, m3T(cams[n].C2Wprior)), wgt = 1 / gap; for (let q = 0; q < 9; q++) M[q] += corr[q] * wgt; }
      cams[k].C2W = m3mul(polar3(M), cams[k].C2Wprior);
    }
  }
  // 5. гравитация: поворачиваем решение к гироскопу (в среднем), затем ставим первый кадр в центр
  {
    const Msum = new Float64Array(9);
    for (const c of cams) { const P = m3mul(c.C2Wprior, m3T(c.C2W)); for (let k = 0; k < 9; k++) Msum[k] += P[k]; }
    const G = polar3(Msum);
    const lon0 = lonLatFromDir(camForward(m3mul(G, cams[0].C2W)))[0];
    const T = m3mul(rotZ(lon0), G);
    cams.forEach(c => { c.C2W = m3mul(T, c.C2W); if (c.center) c.center = mv(T, c.center); });
    if (points) points = points.map(p => mv(T, p));
  }
  const depth = points && points.length > 50 ? buildDepthField(points, opts.depthGW || 192, (opts.depthGW || 192) / 2, opts.depthSmooth) : null;
  // связность: кадры без единой пары остаются на гироскопе
  const linked = new Uint8Array(cams.length); for (const p of pairsB) { linked[p.i] = 1; linked[p.j] = 1; }
  // 5б. надир: нижняя часть кадра «вниз» смотрит на ноги снимающего — не используем её
  if (opts.nadirTrim !== false) for (const c of cams) {
    const fw = camForward(c.C2W);
    if (fw[2] < -Math.sin(70 * Math.PI / 180)) {
      // «низ» кадра в мире: куда смотрит ось +y камеры; тело снимающего — со стороны, противоположной верху телефона
      c.trimY = c.h * (1 - (opts.nadirTrimFrac || 0.42));
    }
  }
  // 6. экспозиция
  say('gains', 0); const gains = opts.gainsByMatches ? solveGains(cams, pairsB) : solveGainsOverlap(cams, f); say('gains', 1);
  // 7. размер развёртки
  const natW = 2 * Math.PI * f;
  let W = opts.outW && opts.outW > 0 ? opts.outW : Math.min(opts.maxW || 8192, Math.pow(2, Math.round(Math.log2(natW * 1.0))));
  W = Math.max(1024, Math.min(opts.maxW || 8192, W)); W -= W % 2;
  // освобождаем пирамиды
  cams.forEach(c => { c.pyr = null; c.corners = null; });
  say('compose', 0);
  let seams = null;
  if (opts.seams !== false) { say('seams', 0); seams = seamLabels(cams, f, gains, depth, opts.seamW || 512, fr => say('seams', fr), { lambda: opts.seamLambda, medTol: opts.seamMedTol != null ? opts.seamMedTol : 10 }); say('seams', 1); }
  let align = null;
  if (seams && opts.localAlign !== false) { say('align', 0); align = seamAlign(cams, f, seams, W, opts, fr => say('align', fr)); say('align', 1, align.constraints + '/' + align.raw + '/' + align.tried); }
  const out = compose(cams, f, gains, W, fr => say('compose', fr), depth, seams, opts, align);
  let cov = 0; for (let i = 0; i < out.covered.length; i++) cov += out.covered[i];
  let holes = out.W * out.H - cov;
  if (!opts.deferFill) { say('fill', 0); holes = fillHoles(out.rgba, out.covered, out.W, out.H); say('fill', 1); }
  const nMatches = pairsB.reduce((s, p) => s + p.matches.length, 0);
  const hfovMax = 2 * Math.atan(Math.max(...cams.map(c => Math.max(c.w, c.h))) / 2 / f) * 180 / Math.PI;
  return {
    rgba: out.rgba, covered: out.covered, W: out.W, H: out.H,
    stats: {
      frames: cams.length, pairs: pairsB.length, candidates: cand.length, matches: nMatches,
      aligned: align ? align.constraints : 0, suspicious, fCal: fCal ? +(fCal.f / Math.max(cams[0].w, cams[0].h)).toFixed(4) : null, k1: cams[0].k1, rms: pba ? pba.rms : ba2 ? ba2.rms : null, rmsRot: ba2 ? ba2.rms : null, rsMs: rsTau ? +(Math.hypot(rsTau[0] * cams[0].w, rsTau[1] * cams[0].h) * 1000).toFixed(1) : 0, rsDir: rsTau ? (Math.abs(rsTau[0]) > Math.abs(rsTau[1]) ? 'x' : 'y') : null, parallax: !!pba, armShift: pba ? +(Math.max(...cams.map(c => Math.hypot(...c.center))) / (depth ? 1 / depth.median : 1)).toFixed(3) : 0, f, f0: opts.f0, fovLong: hfovMax, linked: Array.from(linked).reduce((a, b) => a + b, 0),
      coverage: cov / (out.W * out.H), holes, gains: gains.map(g => g.map(v => +v.toFixed(3))), ms: Date.now() - t0
    },
    seams: seams ? { LW: seams.LW, LH: seams.LH, risk: seams.risk, lab: opts.debugSeams ? seams.lab : undefined } : null,
    cams: cams.map(c => Array.from(c.C2W)), centers: cams.map(c => c.center || [0, 0, 0]), depth, points: opts.debugPoints ? points : undefined
  };
}

// ───────────────────────── перспективный вид из сферы и обратно ─────────────────────────
// Билинейная выборка из equirectangular (RGBA) с переходом через ±180°.
function erpSample(rgba, W, H, d, out, o) {
  const lon = Math.atan2(d[0], d[1]), lat = Math.asin(Math.max(-1, Math.min(1, d[2])));
  let x = (lon / (2 * Math.PI) + 0.5) * W - 0.5, y = (0.5 - lat / Math.PI) * H - 0.5;
  if (y < 0) y = 0; if (y > H - 1) y = H - 1;
  const x0 = Math.floor(x), y0 = Math.min(H - 2, Math.floor(y)), fx = x - x0, fy = y - y0;
  const xa = ((x0 % W) + W) % W, xb = (xa + 1) % W;
  const p00 = (y0 * W + xa) * 4, p10 = (y0 * W + xb) * 4, p01 = ((y0 + 1) * W + xa) * 4, p11 = ((y0 + 1) * W + xb) * 4;
  for (let c = 0; c < 3; c++) out[o + c] = (rgba[p00 + c] * (1 - fx) + rgba[p10 + c] * fx) * (1 - fy) + (rgba[p01 + c] * (1 - fx) + rgba[p11 + c] * fx) * fy;
}
// квадратный перспективный кадр size×size, поле зрения fov (рад), ориентация C2W
function renderPerspective(rgba, W, H, C2W, fov, size) {
  const out = new Uint8ClampedArray(size * size * 4), t = Math.tan(fov / 2), c = (size - 1) / 2, d = [0, 0, 0];
  for (let y = 0; y < size; y++) for (let x = 0; x < size; x++) {
    const vx = (x - c) / c * t, vy = (y - c) / c * t, n = Math.sqrt(vx * vx + vy * vy + 1);
    d[0] = (C2W[0] * vx + C2W[1] * vy + C2W[2]) / n; d[1] = (C2W[3] * vx + C2W[4] * vy + C2W[5]) / n; d[2] = (C2W[6] * vx + C2W[7] * vy + C2W[8]) / n;
    const o = (y * size + x) * 4; erpSample(rgba, W, H, d, out, o); out[o + 3] = 255;
  }
  return out;
}
// обход пикселей сферы, попадающих в квадратный кадр (C2W, fov): cb(index, x, y) с координатами в кадре
function forEachErpInView(W, H, C2W, fov, size, cb) {
  const t = Math.tan(fov / 2), c = (size - 1) / 2, f = c / t;
  const half = Math.atan(t * Math.SQRT2) + 0.01, czMin = Math.cos(Math.min(half, Math.PI / 2 - 0.01));
  const fw = camForward(C2W), latC = Math.asin(Math.max(-1, Math.min(1, fw[2])));
  const v0 = Math.max(0, Math.floor((0.5 - (latC + half) / Math.PI) * H) - 1), v1 = Math.min(H - 1, Math.ceil((0.5 - (latC - half) / Math.PI) * H) + 1);
  for (let v = v0; v <= v1; v++) {
    const lat = Math.PI / 2 - (v + 0.5) / H * Math.PI, cl = Math.cos(lat), sl = Math.sin(lat);
    const A = C2W[2] * cl, B = C2W[5] * cl, C = C2W[8] * sl, Rr = Math.hypot(A, B);
    let uStart = 0, uCount = W;
    if (Rr < 1e-9) { if (C <= czMin) continue; }
    else {
      const ratio = (czMin - C) / Rr; if (ratio >= 1) continue;
      if (ratio > -1) { const lam0 = Math.atan2(A, B), hw = Math.acos(ratio); uStart = Math.ceil((lam0 - hw + Math.PI) / (2 * Math.PI) * W - 0.5); uCount = Math.min(W, Math.floor((lam0 + hw + Math.PI) / (2 * Math.PI) * W - 0.5) - uStart + 1); }
    }
    for (let k = 0; k < uCount; k++) {
      const u = (((uStart + k) % W) + W) % W, lon = (u + 0.5) / W * 2 * Math.PI - Math.PI;
      const dx = cl * Math.sin(lon), dy = cl * Math.cos(lon), dz = sl;
      const cz = C2W[2] * dx + C2W[5] * dy + C2W[8] * dz; if (cz <= 1e-6) continue;
      const x = f * (C2W[0] * dx + C2W[3] * dy + C2W[6] * dz) / cz + c, y = f * (C2W[1] * dx + C2W[4] * dy + C2W[7] * dz) / cz + c;
      if (x < 0 || y < 0 || x > size - 1 || y > size - 1) continue;
      cb(v * W + u, x, y);
    }
  }
}

// ───────────────────────── ИИ-дорисовка пустот ─────────────────────────
// Размер квадратного тайла с полем зрения fov, при котором его пиксель равен пикселю сферы шириной W.
function nativeTile(fov, W) { return Math.max(512, Math.min(2048, Math.round(2 * Math.tan(fov / 2) * W / (2 * Math.PI) / 8) * 8)); }
// covered: 0 — нет снимка, 1 — реальный кадр, 2 — дорисовано ИИ. run(tileRGBA, mask(0 — дорисовать), S) → Promise<RGBA S×S>.
// Мелкие пустоты — отдельным тайлом по их центру; большие (потолок/пол) — гранями куба, сначала те, где больше контекста.
async function aiFillHoles(rgba, covered, W, H, run, report, opts) {
  opts = opts || {};
  const LW = 256, LH = 128, hole = new Uint8Array(LW * LH);
  let any = 0;
  for (let v = 0; v < H; v++) { const y = Math.min(LH - 1, (v * LH / H) | 0); for (let u = 0; u < W; u += 2) if (!covered[v * W + u]) { const k = y * LW + Math.min(LW - 1, (u * LW / W) | 0); if (!hole[k]) { hole[k] = 1; any++; } } }
  if (!any) return 0;
  const cellDir = k => { const y = (k / LW) | 0, x = k - y * LW; return dirFromLonLat((x + 0.5) / LW * 2 * Math.PI - Math.PI, Math.PI / 2 - (y + 0.5) / LH * Math.PI); };
  // угловое расстояние каждой пустой ячейки до снятого (Дейкстра по сетке)
  const dist = new Float32Array(LW * LH).fill(Infinity), cellH = Math.PI / LH;
  const heap = [];
  const push = (k, d) => { heap.push([d, k]); let i = heap.length - 1; while (i > 0) { const p = (i - 1) >> 1; if (heap[p][0] <= heap[i][0]) break; [heap[p], heap[i]] = [heap[i], heap[p]]; i = p; } };
  const pop = () => { const top = heap[0], last = heap.pop(); if (heap.length) { heap[0] = last; let i = 0; for (;;) { const l = 2 * i + 1, r = l + 1; let m = i; if (l < heap.length && heap[l][0] < heap[m][0]) m = l; if (r < heap.length && heap[r][0] < heap[m][0]) m = r; if (m === i) break; [heap[m], heap[i]] = [heap[i], heap[m]]; i = m; } } return top; };
  for (let k = 0; k < LW * LH; k++) if (!hole[k]) { dist[k] = 0; push(k, 0); }
  while (heap.length) {
    const [d, k] = pop(); if (d > dist[k]) continue;
    const y = (k / LW) | 0, x = k - y * LW, cw = Math.max(0.02, Math.cos(Math.PI / 2 - (y + 0.5) / LH * Math.PI)) * 2 * Math.PI / LW;
    for (const [dx, dy, c] of [[1, 0, cw], [-1, 0, cw], [0, 1, cellH], [0, -1, cellH]]) {
      const yy = y + dy; if (yy < 0 || yy >= LH) continue; const kk = yy * LW + (x + dx + LW) % LW, nd = d + c;
      if (nd < dist[kk]) { dist[kk] = nd; push(kk, nd); }
    }
  }
  // компоненты связности (с переходом через ±180°)
  const comp = new Int32Array(LW * LH).fill(-1), comps = [];
  for (let k0 = 0; k0 < LW * LH; k0++) {
    if (!hole[k0] || comp[k0] >= 0) continue;
    const id = comps.length, stack = [k0], cells = []; comp[k0] = id;
    while (stack.length) {
      const k = stack.pop(); cells.push(k); const y = (k / LW) | 0, x = k - y * LW;
      for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) {
        const yy = y + dy; if (yy < 0 || yy >= LH) continue; const xx = (x + dx + LW) % LW, kk = yy * LW + xx;
        if (hole[kk] && comp[kk] < 0) { comp[kk] = id; stack.push(kk); }
      }
    }
    comps.push(cells);
  }
  const reach = (opts.reach || 15) * Math.PI / 180;
  const eligible = new Uint8Array(LW * LH); // ячейки, которые ИИ имеет право дорисовать
  const tiles = [];
  for (const cells of comps) {
    let sx = 0, sy = 0, sz = 0, wsum = 0, maxd = 0;
    for (const k of cells) { const d = cellDir(k), a = Math.max(0.05, Math.sqrt(1 - d[2] * d[2])); sx += d[0] * a; sy += d[1] * a; sz += d[2] * a; wsum += a; maxd = Math.max(maxd, dist[k]); }
    const n = Math.hypot(sx, sy, sz), c = [sx / n, sy / n, sz / n];
    let rad = 0; for (const k of cells) { const d = cellDir(k); rad = Math.max(rad, Math.acos(Math.max(-1, Math.min(1, d[0] * c[0] + d[1] * c[1] + d[2] * c[2])))); }
    rad += 1.5 * cellH;
    if (n / wsum > 0.5 && rad < 40 * Math.PI / 180) {
      // небольшая пустота — целиком одним тайлом с запасом контекста
      for (const k of cells) eligible[k] = 1;
      const ll = lonLatFromDir(c);
      tiles.push({ lon: ll[0], lat: ll[1], fov: Math.max((opts.minFov || 60) * Math.PI / 180, Math.min(110 * Math.PI / 180, (opts.fovScale || 4) * rad + 0.2)), area: cells.length });
    } else {
      // большая (потолок/пол у панорамы с телефона) — продолжаем текстуру полосой вдоль края снятого
      const todo = [];
      for (const k of cells) if (dist[k] <= reach) { eligible[k] = 1; if (dist[k] >= reach * 0.25 && dist[k] <= reach * 0.75) todo.push(k); }
      const F = 110 * Math.PI / 180, cover = 0.7 * F / 2, used = new Uint8Array(todo.length);
      for (let a0 = 0; a0 < todo.length; a0++) {
        if (used[a0]) continue;
        const d0 = cellDir(todo[a0]);
        // центр тайла — чуть в сторону снятого, чтобы в кадре был контекст
        for (let b0 = a0; b0 < todo.length; b0++) { if (used[b0]) continue; const d = cellDir(todo[b0]); if (d[0] * d0[0] + d[1] * d0[1] + d[2] * d0[2] > Math.cos(cover)) used[b0] = 1; }
        const ll = lonLatFromDir(d0), toward = ll[1] > 0 ? -1 : 1;
        tiles.push({ lon: ll[0], lat: ll[1] + toward * reach * 0.6, fov: F, area: 1e9 });
      }
    }
  }
  tiles.sort((a, b) => a.area - b.area);
  let filled = 0;
  const elig = (u, v) => eligible[Math.min(LH - 1, (v * LH / H) | 0) * LW + Math.min(LW - 1, (u * LW / W) | 0)];
  for (let t = 0; t < tiles.length; t++) {
    const T = tiles[t], C2W = c2wLookAt(T.lon, T.lat), S = opts.tile || nativeTile(T.fov, W);
    const img = renderPerspective(rgba, W, H, C2W, T.fov, S);
    // маска: 0 там, где на сфере пусто (с расширением на 2 px); недоступное ИИ — тоже маскируем, но не записываем
    const raw = new Uint8Array(S * S), tt = Math.tan(T.fov / 2), cc = (S - 1) / 2, d = [0, 0, 0];
    let nh = 0;
    for (let y = 0; y < S; y++) for (let x = 0; x < S; x++) {
      const vx = (x - cc) / cc * tt, vy = (y - cc) / cc * tt, nn = Math.sqrt(vx * vx + vy * vy + 1);
      d[0] = (C2W[0] * vx + C2W[1] * vy + C2W[2]) / nn; d[1] = (C2W[3] * vx + C2W[4] * vy + C2W[5]) / nn; d[2] = (C2W[6] * vx + C2W[7] * vy + C2W[8]) / nn;
      const lon = Math.atan2(d[0], d[1]), lat = Math.asin(Math.max(-1, Math.min(1, d[2])));
      const u = Math.min(W - 1, Math.max(0, Math.floor((lon / (2 * Math.PI) + 0.5) * W))), v = Math.min(H - 1, Math.max(0, Math.floor((0.5 - lat / Math.PI) * H)));
      if (!covered[v * W + u]) { raw[y * S + x] = 1; if (elig(u, v)) nh++; }
    }
    if (nh < 30) { if (report) report((t + 1) / tiles.length); continue; }
    const mask = new Uint8Array(S * S).fill(255), R = 2;
    for (let y = 0; y < S; y++) for (let x = 0; x < S; x++) {
      if (!raw[y * S + x]) continue;
      for (let dy = -R; dy <= R; dy++) for (let dx = -R; dx <= R; dx++) { const yy = y + dy, xx = x + dx; if (yy >= 0 && xx >= 0 && yy < S && xx < S) mask[yy * S + xx] = 0; }
    }
    const out = await run(img, mask, S);
    forEachErpInView(W, H, C2W, T.fov, S, (idx, x, y) => {
      if (covered[idx]) return;
      const v = (idx / W) | 0, u = idx - v * W; if (!elig(u, v)) return;
      const x0 = Math.min(S - 2, Math.floor(x)), y0 = Math.min(S - 2, Math.floor(y)), fx = x - x0, fy = y - y0, p = (y0 * S + x0) * 4, o = idx * 4;
      for (let c = 0; c < 3; c++) rgba[o + c] = (out[p + c] * (1 - fx) + out[p + 4 + c] * fx) * (1 - fy) + (out[p + S * 4 + c] * (1 - fx) + out[p + S * 4 + 4 + c] * fx) * fy;
      rgba[o + 3] = 255; covered[idx] = 2; filled++;
    });
    if (report) report((t + 1) / tiles.length);
  }
  return filled;
}
// ИИ лечит швы: кластеры ячеек, где соседние кадры заметно расходятся, перерисовываются нейросетью.
// Кластер — связная область рискованных ячеек (с расширением); тайл с запасом контекста, вклейка с мягким краем.
function aiSeamClusters(seams, opts) {
  opts = opts || {};
  const { LW, LH, risk } = seams, thr = opts.thr || 1.3;
  const hot = new Uint8Array(LW * LH);
  for (let k = 0; k < LW * LH; k++) if (risk[k] > thr) hot[k] = 1;
  const dil = new Uint8Array(LW * LH);
  for (let k = 0; k < LW * LH; k++) if (hot[k]) { const y = (k / LW) | 0, x = k - y * LW; for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) { const yy = y + dy; if (yy >= 0 && yy < LH) dil[yy * LW + (x + dx + LW) % LW] = 1; } }
  const comp = new Int32Array(LW * LH).fill(-1), comps = [];
  for (let k0 = 0; k0 < LW * LH; k0++) {
    if (!dil[k0] || comp[k0] >= 0) continue;
    const id = comps.length, st = [k0], cells = []; comp[k0] = id; let mass = 0;
    while (st.length) { const k = st.pop(); cells.push(k); mass += hot[k] ? risk[k] : 0; const y = (k / LW) | 0, x = k - y * LW; for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) { const yy = y + dy; if (yy < 0 || yy >= LH) continue; const kk = yy * LW + (x + dx + LW) % LW; if (dil[kk] && comp[kk] < 0) { comp[kk] = id; st.push(kk); } } }
    if (cells.length <= 400) comps.push({ cells, mass, id });
  }
  comps.sort((a, b) => b.mass - a.mass);
  return comps.filter(c => c.mass >= (opts.minMass || 6));
}
async function aiHealSeams(rgba, covered, W, H, seams, run, report, opts) {
  opts = opts || {};
  const { LW, LH } = seams, maxTiles = opts.maxTiles || 8;
  const cellDir = k => { const y = (k / LW) | 0, x = k - y * LW; return dirFromLonLat((x + 0.5) / LW * 2 * Math.PI - Math.PI, Math.PI / 2 - (y + 0.5) / LH * Math.PI); };
  let done = 0, tiles = 0;
  const undos = [];
  const todo = aiSeamClusters(seams, opts).slice(0, maxTiles);
  for (const c of todo) {
    let sx = 0, sy = 0, sz = 0; for (const k of c.cells) { const d = cellDir(k); sx += d[0]; sy += d[1]; sz += d[2]; }
    const n = Math.hypot(sx, sy, sz); if (n < 1e-6) continue; const cd = [sx / n, sy / n, sz / n];
    let rad = 0; for (const k of c.cells) { const d = cellDir(k); rad = Math.max(rad, Math.acos(Math.max(-1, Math.min(1, d[0] * cd[0] + d[1] * cd[1] + d[2] * cd[2])))); }
    rad += Math.PI / LH;
    if (rad > 12 * Math.PI / 180) continue; // крупное расхождение — не трогаем автоматически (лучше кистью)
    const fov = Math.min(100 * Math.PI / 180, Math.max(18 * Math.PI / 180, 3 * rad + 0.1)), ll = lonLatFromDir(cd), C2W = c2wLookAt(ll[0], ll[1]), S = opts.tile || nativeTile(fov, W);
    const img = renderPerspective(rgba, W, H, C2W, fov, S);
    const set = new Set(c.cells), mask = new Uint8Array(S * S).fill(255), tt = Math.tan(fov / 2), cc = (S - 1) / 2, d = [0, 0, 0];
    let nh = 0;
    for (let y = 0; y < S; y++) for (let x = 0; x < S; x++) {
      const vx = (x - cc) / cc * tt, vy = (y - cc) / cc * tt, nn = Math.sqrt(vx * vx + vy * vy + 1);
      d[0] = (C2W[0] * vx + C2W[1] * vy + C2W[2]) / nn; d[1] = (C2W[3] * vx + C2W[4] * vy + C2W[5]) / nn; d[2] = (C2W[6] * vx + C2W[7] * vy + C2W[8]) / nn;
      const lon = Math.atan2(d[0], d[1]), lat = Math.asin(Math.max(-1, Math.min(1, d[2])));
      const cu = Math.min(LW - 1, Math.max(0, Math.floor((lon / (2 * Math.PI) + 0.5) * LW))), cv = Math.min(LH - 1, Math.max(0, Math.floor((0.5 - lat / Math.PI) * LH)));
      if (set.has(cv * LW + cu)) { mask[y * S + x] = 0; nh++; }
    }
    if (nh < 30) continue;
    const alpha = new Float32Array(S * S);
    for (let y = 0; y < S; y++) for (let x = 0; x < S; x++) {
      if (!mask[y * S + x]) { alpha[y * S + x] = 1; continue; }
      let k = 0, m = 0; for (let dy = -2; dy <= 2; dy += 1) for (let dx = -2; dx <= 2; dx += 1) { const yy = y + dy, xx = x + dx; if (yy >= 0 && xx >= 0 && yy < S && xx < S) { k += mask[yy * S + xx] ? 0 : 1; m++; } }
      alpha[y * S + x] = 0.8 * k / m;
    }
    const out = await run(img, mask, S);
    const u = applyInpaintTile(rgba, covered, W, H, C2W, fov, S, out, alpha);
    done += u.idx.length; tiles++; undos.push(u);
    if (report) report(tiles / todo.length);
  }
  return { pixels: done, tiles, undos };
}

// Исправление кистью: тайл C2W/fov размером S, маска (0 — перерисовать) уже готова; out — ответ модели.
// Возвращает копию старых пикселей для отмены.
function applyInpaintTile(rgba, covered, W, H, C2W, fov, S, out, alpha) {
  const undo = { idx: [], px: [], cov: [] };
  forEachErpInView(W, H, C2W, fov, S, (idx, x, y) => {
    const x0 = Math.min(S - 2, Math.floor(x)), y0 = Math.min(S - 2, Math.floor(y)), fx = x - x0, fy = y - y0;
    const a = (alpha[y0 * S + x0] * (1 - fx) + alpha[y0 * S + x0 + 1] * fx) * (1 - fy) + (alpha[(y0 + 1) * S + x0] * (1 - fx) + alpha[(y0 + 1) * S + x0 + 1] * fx) * fy;
    if (a <= 0.002) return;
    const p = (y0 * S + x0) * 4, o = idx * 4;
    undo.idx.push(idx); undo.px.push(rgba[o], rgba[o + 1], rgba[o + 2]); undo.cov.push(covered[idx]);
    for (let c = 0; c < 3; c++) {
      const v = (out[p + c] * (1 - fx) + out[p + 4 + c] * fx) * (1 - fy) + (out[p + S * 4 + c] * (1 - fx) + out[p + S * 4 + 4 + c] * fx) * fy;
      rgba[o + c] = rgba[o + c] * (1 - a) + v * a;
    }
    if (a > 0.5) covered[idx] = 3;
  });
  return undo;
}

// ───────────────────────── панорама телефона → сфера ─────────────────────────
function parseGPano(bytes) {
  // ищем XMP в первых 256 КБ
  const n = Math.min(bytes.length, 262144); let s = '';
  for (let i = 0; i < n; i++) s += String.fromCharCode(bytes[i]);
  const k = s.indexOf('GPano'); if (k < 0) return null;
  const get = name => { const re = new RegExp('GPano:' + name + '(?:="([^"]*)"|>([^<]*)<)'); const m = s.match(re); return m ? (m[1] != null ? m[1] : m[2]) : null; };
  const num = name => { const v = get(name); return v == null ? null : parseFloat(v); };
  const g = {
    projection: get('ProjectionType'), fullW: num('FullPanoWidthPixels'), fullH: num('FullPanoHeightPixels'),
    cropW: num('CroppedAreaImageWidthPixels'), cropH: num('CroppedAreaImageHeightPixels'), left: num('CroppedAreaLeftPixels'), top: num('CroppedAreaTopPixels')
  };
  if (!g.fullW || !g.cropW) return null;
  return g;
}
// период 360° у полосы: ищем повтор левого края в правой части
function detectPeriod(rgb, w, h) {
  const k = Math.max(1, Math.round(h / 96)), g = grayDown(rgb, w, h, k);
  const sw = Math.max(6, Math.round(g.w * 0.035)), from = Math.round(g.w * 0.55), to = g.w - sw;
  if (to <= from) return null;
  const tpl = []; let ts = 0, tss = 0;
  for (let y = 0; y < g.h; y++) for (let x = 0; x < sw; x++) { const v = g.d[y * g.w + x]; tpl.push(v); ts += v; tss += v * v; }
  const N = tpl.length, tm = ts / N, tv = tss - ts * ts / N; if (tv < 1e-4) return null;
  let best = -1, bx = -1;
  for (let x0 = from; x0 <= to; x0++) {
    let s = 0, ss = 0, dot = 0, q = 0;
    for (let y = 0; y < g.h; y++) for (let x = 0; x < sw; x++, q++) { const v = g.d[y * g.w + x0 + x]; s += v; ss += v * v; dot += (tpl[q] - tm) * v; }
    const vv = ss - s * s / N; const sc = vv > 1e-6 ? dot / Math.sqrt(vv * tv) : -1;
    if (sc > best) { best = sc; bx = x0; }
  }
  if (best < 0.82) return null;
  return { period: bx * k, score: best };
}
// cyl: горизонтальный охват A (градусы), смещение горизонта (градусы). gpano: размещение по тегам.
function phonePanoToERP(src, opts, report) {
  const { w, h, rgb } = src, maxW = opts.maxW || 8192;
  let W, H, rgba, covered;
  if (opts.mode === 'gpano' && opts.gpano) {
    const g = opts.gpano;
    W = Math.min(maxW, Math.round(g.fullW * (w / g.cropW) / 2) * 2); H = W / 2;
    rgba = new Uint8ClampedArray(W * H * 4); covered = new Uint8Array(W * H);
    const left = g.left * W / g.fullW, top = (g.top || 0) * H / g.fullH, cw = g.cropW * W / g.fullW, chh = g.cropH * H / g.fullH;
    for (let v = 0; v < H; v++) {
      const yy = (v + 0.5 - top) / chh * h - 0.5; if (yy < 0 || yy > h - 1) continue;
      for (let u = 0; u < W; u++) {
        let du = u + 0.5 - left; if (du < 0) du += W; const xx = du / cw * w - 0.5; if (xx < 0 || xx > w - 1) continue;
        bilinRGB(rgb, w, h, xx, yy, rgba, (v * W + u) * 4); covered[v * W + u] = 1;
      }
      if (report && v % 64 === 0) report({ stage: 'compose', frac: v / H });
    }
  } else {
    const A = Math.max(30, Math.min(400, opts.hfov || 360)) * Math.PI / 180;
    const span = Math.min(A, 2 * Math.PI); // ≥360° — лишнее отрезаем
    const fc = w / A; // пикселей на радиан
    W = Math.min(maxW, Math.max(1024, Math.round(fc * 2 * Math.PI / 2) * 2)); H = W / 2;
    rgba = new Uint8ClampedArray(W * H * 4); covered = new Uint8Array(W * H);
    const off = (opts.vshift || 0) * Math.PI / 180;
    for (let v = 0; v < H; v++) {
      const lat = Math.PI / 2 - (v + 0.5) / H * Math.PI;
      if (Math.abs(lat - off) >= Math.PI / 2 - 0.01) continue;
      const yy = h / 2 - fc * Math.tan(lat - off) - 0.5; if (yy < 0 || yy > h - 1) continue;
      for (let u = 0; u < W; u++) {
        const lon = (u + 0.5) / W * 2 * Math.PI - Math.PI; // от −π до π
        const a = lon + span / 2; if (a < 0 || a > span) continue;
        const xx = a * fc - 0.5; if (xx < 0 || xx > w - 1) continue;
        bilinRGB(rgb, w, h, xx, yy, rgba, (v * W + u) * 4); covered[v * W + u] = 1;
      }
      if (report && v % 64 === 0) report({ stage: 'compose', frac: v / H });
    }
  }
  for (let i = 3; i < rgba.length; i += 4) rgba[i] = 255;
  let cov = 0; for (let i = 0; i < covered.length; i++) cov += covered[i];
  let holes = W * H - cov;
  if (!opts.deferFill) { if (report) report({ stage: 'fill', frac: 0 }); holes = fillHoles(rgba, covered, W, H); }
  return { rgba, covered, W, H, stats: { frames: 1, coverage: cov / (W * H), holes } };
}
function bilinRGB(rgb, w, h, x, y, out, o) {
  let x0 = x | 0, y0 = y | 0; if (x0 >= w - 1) x0 = w - 2; if (y0 >= h - 1) y0 = h - 2;
  const fx = x - x0, fy = y - y0, p = (y0 * w + x0) * 3, pr = p + w * 3;
  for (let c = 0; c < 3; c++) {
    const v = (SRGB2LIN[rgb[p + c]] * (1 - fx) + SRGB2LIN[rgb[p + 3 + c]] * fx) * (1 - fy) + (SRGB2LIN[rgb[pr + c]] * (1 - fx) + SRGB2LIN[rgb[pr + 3 + c]] * fx) * fy;
    out[o + c] = lin2srgb(v);
  }
}

// ───────────────────────── несколько панорам телефона → кадры для склейки ─────────────────────────
// Модель «щелевой» панорамы (так её собирает телефон из центральных полос кадров при повороте):
// столбец x — азимут ψ = x / F (F = f·cos θ — пикселей на радиан поворота), строка y — луч центральной
// вертикали камеры, наклонённой на θ: y = h/2 + f·tg(θ − e), где e — угол над горизонтом в плоскости столбца.
// При θ = 0 это обычная цилиндрическая панорама.
function slitToPix(P, d) {
  // мировое направление → пиксель панорамы P {w,h,f,F,theta,yaw,span}; null — вне полосы
  const hor = Math.hypot(d[0], d[1]); let az = Math.atan2(d[0], d[1]);
  for (let s = 0; s < 2; s++) {
    const hd = s ? -hor : hor, a = s ? az + Math.PI : az;
    const e = Math.atan2(d[2], hd), dl = P.theta - e;
    if (Math.abs(dl) > 1.35) continue;
    const y = P.h / 2 + P.f * Math.tan(dl) - 0.5;
    if (y < 0 || y > P.h - 1) continue;
    let psi = a - P.yaw; psi -= 2 * Math.PI * Math.floor(psi / (2 * Math.PI));
    let x = psi * P.F - 0.5;
    if (P.period && x > P.period) x -= P.period;
    if (x < 0 || x > P.w - 1) continue;
    return [x, y];
  }
  return null;
}
function slitSetup(P, k) {
  P.f = k * P.h; P.F = P.period ? P.period / (2 * Math.PI) : P.f * Math.cos(P.theta) * (P.fs || 1);
  P.span = Math.min(2 * Math.PI, P.w / P.F); P.full = P.span > 2 * Math.PI - 0.02;
}
// мелкая карта градиентов панорамы в развёртке (для поиска взаимного положения панорам).
// Отображение разделяется: строка развёртки → строка панорамы, столбец → столбец, поэтому считается быстро.
function slitErpGrad(P, small, W) {
  const H = W / 2, out = new Float32Array(W * H), m = new Uint8Array(W * H);
  const sc = small.s, f = P.f * sc, F = P.F * sc, per = P.period ? P.period * sc : 0, sw = small.w, shh = small.h;
  const xs = new Int32Array(W), xs2 = new Int32Array(W);
  for (let u = 0; u < W; u++) {
    const lon = (u + 0.5) / W * 2 * Math.PI - Math.PI;
    for (let s = 0; s < 2; s++) {
      let psi = lon + (s ? Math.PI : 0) - P.yaw; psi -= 2 * Math.PI * Math.floor(psi / (2 * Math.PI));
      let x = psi * F - 0.5; if (per && x > per) x -= per;
      (s ? xs2 : xs)[u] = x < 0 || x > sw - 1 ? -1 : Math.round(x);
    }
  }
  for (let v = 2; v < H - 2; v++) {
    const lat = Math.PI / 2 - (v + 0.5) / H * Math.PI;
    let yy = -1, X = xs;
    for (let s = 0; s < 2 && yy < 0; s++) {
      const e = s ? (lat > 0 ? Math.PI - lat : -Math.PI - lat) : lat, dl = P.theta - e;
      if (Math.abs(dl) > 1.35) continue;
      const y = shh / 2 + f * Math.tan(dl) - 0.5;
      if (y >= 0 && y <= shh - 1) { yy = Math.round(y); X = s ? xs2 : xs; }
    }
    if (yy < 0) continue;
    const row = yy * sw, o = v * W;
    for (let u = 0; u < W; u++) { const x = X[u]; if (x < 0) continue; out[o + u] = small.g[row + x]; m[o + u] = 1; }
  }
  return { g: out, m, W, H };
}
function slitSmall(src, H0) {
  // уменьшенная копия: модуль градиента сглаженной яркости
  const s = H0 / src.h, w = Math.max(8, Math.round(src.w * s)), h = H0, L = new Float32Array(w * h);
  const kx = src.w / w, ky = src.h / h;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
    let a = 0, n = 0;
    const x0 = Math.floor(x * kx), x1 = Math.max(x0 + 1, Math.floor((x + 1) * kx)), y0 = Math.floor(y * ky), y1 = Math.max(y0 + 1, Math.floor((y + 1) * ky));
    for (let yy = y0; yy < y1; yy += Math.max(1, (y1 - y0) >> 2)) for (let xx = x0; xx < x1; xx += Math.max(1, (x1 - x0) >> 2)) { const p = (yy * src.w + xx) * 3; a += 0.299 * src.rgb[p] + 0.587 * src.rgb[p + 1] + 0.114 * src.rgb[p + 2]; n++; }
    L[y * w + x] = a / n / 255;
  }
  const g = new Float32Array(w * h);
  for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) { const i = y * w + x; g[i] = Math.hypot(L[i + 1] - L[i - 1], L[i + w] - L[i - w]); }
  return { w, h, g, s };
}
// NCC двух карт по общей маске при сдвиге b по долготе на sh пикселей
function erpShiftNcc(A, B, sh, rows) {
  const W = A.W; let n = 0, sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
  for (const v of rows) {
    const o = v * W;
    for (let u = 0; u < W; u++) {
      const ia = o + u; if (!A.m[ia]) continue;
      let ub = u - sh; if (ub < 0) ub += W; else if (ub >= W) ub -= W;
      const ib = o + ub; if (!B.m[ib]) continue;
      const a = A.g[ia], b = B.g[ib]; n++; sa += a; sb += b; saa += a * a; sbb += b * b; sab += a * b;
    }
  }
  if (n < 150) return { s: -1, n };
  const va = saa - sa * sa / n, vb = sbb - sb * sb / n;
  return { s: va > 1e-9 && vb > 1e-9 ? (sab - sa * sb / n) / Math.sqrt(va * vb) : -1, n };
}
// panos: [{ rgb, w, h, tilt (° по подсказке) }] → кадры для stitchPanorama
function panosToFrames(panos, opts, report) {
  opts = opts || {};
  const say = (stage, frac, note) => report && report({ stage, frac, note });
  const D = Math.PI / 180, W = opts.alignW || 360, SH = 120;
  const Ps = panos.map(p => {
    const th = (p.tilt || 0) * D, per = detectPeriod(p.rgb, p.w, p.h);
    // замыкание круга принимаем, только если оно правдоподобно для фокуса телефона (0,68…1,05 высоты)
    let period = 0;
    if (per && per.score > 0.88 && Math.abs(th) < 10 * D) {
      const F = per.period / (2 * Math.PI), lo = 0.68 * p.h * Math.cos(Math.min(1.45, Math.abs(th) + 15 * D)), hi = 1.05 * p.h * Math.cos(Math.max(0, Math.abs(th) - 15 * D));
      if (F > lo && F < hi) period = per.period;
    }
    return { src: p, w: p.w, h: p.h, theta: th, theta0: th, auto: p.tilt == null, yaw: 0, period };
  });
  // порядок: от горизонта к полюсам; первая — опорная
  // опорная — первая «прямо»; панорамы с известным наклоном раньше «авто»
  let order = Ps.map((p, i) => i).sort((a, b) => (Ps[a].auto - Ps[b].auto) || Math.abs(Ps[a].theta0) - Math.abs(Ps[b].theta0) || a - b);
  const hz = Ps[order[0]]; if (hz.auto) { hz.auto = false; hz.theta0 = 0; }
  // фокус (доля высоты панорамы): если горизонтальная панорама замкнута по кругу, он известен точно
  let k = opts.k || 0.84, kFixed = !!opts.k;
  if (!opts.k && hz.period) { k = hz.period / (2 * Math.PI) / Math.cos(hz.theta0) / hz.h; kFixed = true; }
  Ps.forEach(P => { P.theta = P.theta0; P.yaw = 0; P.fs = 1; P.smC = slitSmall(P.src, 72); P.smF = slitSmall(P.src, 220); slitSetup(P, k); });
  const rowsOf = (comp, E) => { const r = [], W_ = E.W; for (let v = 0; v < E.H; v++) { let a = 0, c = 0; for (let u = 0; u < W_; u += 2) { a += comp.m[v * W_ + u]; c += E.m[v * W_ + u]; } if (a > 3 && c > 3) r.push(v); } return r; };
  const addTo = (comp, E) => { for (let i = 0; i < E.g.length; i++) if (E.m[i] && !comp.m[i]) { comp.g[i] = E.g[i]; comp.m[i] = 1; } };
  // 1. грубо: 2° на пиксель, наклон ±15° от указанного (или весь диапазон для «авто»), азимут — полный круг.
  //    Панораму, которой пока не с чем сравнить, откладываем в конец очереди.
  const Wc = 180;
  {
    const comp = slitErpGrad(hz, hz.smC, Wc), queue = order.slice(1), done = [order[0]], tries = new Map();
    while (queue.length) {
      const pi = queue.shift(), P = Ps[pi]; let bj = { s: -2, th: P.theta0, sh: 0, rows: 0 };
      // у крутых наклонов шаг F ∝ cos θ меняется быстро — там перебираем наклон мельче
      const ths = [], lo = P.auto ? -86 : P.theta0 / D - 15, hi = P.auto ? 86 : P.theta0 / D + 15;
      for (let t = lo; t <= hi + 1e-6; t += Math.abs(t) > 52 ? 1 : 3) if (Math.abs(t) <= 88) ths.push(t * D);
      for (const th of ths) {
        P.theta = th; P.yaw = 0; slitSetup(P, k);
        const E = slitErpGrad(P, P.smC, Wc), rows = rowsOf(comp, E); if (rows.length < 2) continue;
        for (let sh = 0; sh < Wc; sh++) { const r = erpShiftNcc(comp, E, sh, rows); if (r.s > bj.s) bj = { s: r.s, sh, th, rows: rows.length }; }
      }
      const t = tries.get(pi) || 0;
      if ((bj.s < 0.15 || bj.rows < 3) && t < 1 && queue.length) { tries.set(pi, t + 1); queue.push(pi); continue; }
      P.theta = bj.th; P.yaw = bj.sh / Wc * 2 * Math.PI; slitSetup(P, k); P.coarse = bj.s;
      addTo(comp, slitErpGrad(P, P.smC, Wc)); done.push(pi);
      say('panos', done.length / order.length * 0.5);
    }
    order = done;
  }
  // 2. точно: 0,5° на пиксель, координатный спуск по наклону, шагу F и азимуту каждой панорамы
  const Wf = 720;
  const refineAll = (kk, iters) => {
    Ps.forEach(P => slitSetup(P, kk));
    const comp = slitErpGrad(hz, hz.smF, Wf); let tot = 0, n = 0;
    for (let oi = 1; oi < order.length; oi++) {
      const P = Ps[order[oi]];
      const evalAt = (th, fs, yp) => {
        P.theta = th; P.fs = fs; P.yaw = 0; slitSetup(P, kk);
        const E = slitErpGrad(P, P.smF, Wf), rows = rowsOf(comp, E);
        let bs = { s: -2, sh: yp };
        for (let sh = Math.round(yp) - 6; sh <= Math.round(yp) + 6; sh++) { const s2 = ((sh % Wf) + Wf) % Wf, r = erpShiftNcc(comp, E, s2, rows); if (r.s > bs.s) bs = { s: r.s, sh: s2 }; }
        return bs;
      };
      let th = P.theta, fs = P.fs || 1, yp = P.yaw / (2 * Math.PI) * Wf, cur = evalAt(th, fs, yp); yp = cur.sh;
      for (let it = 0; it < iters; it++) {
        let moved = false;
        for (const dt of [-1.5, -0.5, 0.5, 1.5]) { const r = evalAt(th + dt * D, fs, yp); if (r.s > cur.s + 1e-4) { cur = r; th += dt * D; yp = r.sh; moved = true; } }
        if (!P.period) for (const df of [-0.03, -0.01, 0.01, 0.03]) { const r = evalAt(th, fs + df, yp); if (r.s > cur.s + 1e-4) { cur = r; fs += df; yp = r.sh; moved = true; } }
        if (!moved) break;
      }
      P.theta = th; P.fs = fs; P.yaw = yp / Wf * 2 * Math.PI; slitSetup(P, kk); P.score = cur.s;
      addTo(comp, slitErpGrad(P, P.smF, Wf));
      if (cur.s > -1) { tot += cur.s; n++; }
    }
    return n ? tot / n : 0;
  };
  const snap = () => Ps.map(P => [P.theta, P.yaw, P.fs]), restore = st => Ps.forEach((P, i) => { [P.theta, P.yaw, P.fs] = st[i]; });
  let bestS = order.length > 1 ? refineAll(k, 3) : 0, bestSt = snap();
  say('panos', 0.7);
  if (!kFixed && order.length > 1) {
    for (const dk of [-0.05, -0.025, 0.025, 0.05]) {
      restore(bestSt); const sc = refineAll(k + dk, 1);
      if (sc > bestS + 0.002) { bestS = sc; bestSt = snap(); k += dk; }
    }
    restore(bestSt); bestS = refineAll(k, 2); bestSt = snap();
  }
  restore(bestSt); Ps.forEach(P => slitSetup(P, k));
  // 3. панорамы, которые легли плохо (часто — крутой наклон к потолку или полу, где шаг F ∝ cos θ
  //    очень чувствителен), ищем заново по сетке «наклон × шаг» на фоне уже уложенных остальных
  const weak = order.slice(1).filter(i => !(Ps[i].score > 0.45));
  if (weak.length) {
    for (const pi of weak) {
      const P = Ps[pi], comp = { g: new Float32Array(Wc * Wc / 2), m: new Uint8Array(Wc * Wc / 2), W: Wc, H: Wc / 2 };
      order.forEach(j => { if (j !== pi && (j === order[0] || Ps[j].score > 0.45)) addTo(comp, slitErpGrad(Ps[j], Ps[j].smC, Wc)); });
      const ths = [];
      if (P.auto) { for (let t = -88; t <= 88; t += Math.abs(t) > 52 ? 1 : 2) ths.push(t * D); }
      else for (let t = P.theta0 / D - 14; t <= P.theta0 / D + 14; t += Math.abs(t) > 52 ? 1 : 2) if (Math.abs(t) <= 88) ths.push(t * D);
      let bj = { s: -2 };
      for (const th of ths) for (const fs of [0.88, 0.94, 1, 1.06, 1.12]) {
        P.theta = th; P.fs = fs; P.yaw = 0; slitSetup(P, k);
        const E = slitErpGrad(P, P.smC, Wc), rows = rowsOf(comp, E); if (rows.length < 3) continue;
        for (let sh = 0; sh < Wc; sh++) { const r = erpShiftNcc(comp, E, sh, rows); if (r.s > bj.s) bj = { s: r.s, sh, th, fs }; }
      }
      if (bj.s > (P.score || -1)) { P.theta = bj.th; P.fs = bj.fs; P.yaw = bj.sh / Wc * 2 * Math.PI; slitSetup(P, k); }
    }
    refineAll(k, 3);
  }
  say('panos', 1);
  const best = { k, score: bestS };
  // виртуальные кадры: обычные «снимки» центральной перспективной камерой внутри каждой полосы
  const fv = Ps[order[0]].f; // фокус виртуальной камеры = фокус панорамы (родное разрешение)
  const frames = [], info = [];
  Ps.forEach((P, pi) => {
    const halfV = Math.atan(P.h / 2 / P.f) - 3.5 * D;
    const hv = Math.round(2 * fv * Math.tan(halfV) / 2) * 2, wv = Math.round(hv * 0.75 / 2) * 2;
    const halfH = Math.atan(wv / 2 / fv);
    const step = Math.min(90 * D, (opts.stepDeg || 24) * D / Math.max(0.3, Math.cos(P.theta)));
    const lons = [];
    if (P.full) { const n = Math.max(3, Math.round(2 * Math.PI / step)); for (let i = 0; i < n; i++) lons.push(P.yaw + i * 2 * Math.PI / n); }
    else {
      const a0 = halfH / Math.max(0.3, Math.cos(P.theta)), a1 = P.span - a0;
      if (a1 <= a0) lons.push(P.yaw + P.span / 2);
      else { const n = Math.max(1, Math.ceil((a1 - a0) / step)); for (let i = 0; i <= n; i++) lons.push(P.yaw + a0 + (a1 - a0) * i / n); }
    }
    info.push({ tilt: (P.theta / D).toFixed(1), yaw: (P.yaw / D).toFixed(1), fs: (P.fs || 1).toFixed(2), span: (P.span / D).toFixed(0), full: P.full, frames: lons.length });
    for (const lon of lons) {
      const C2W = c2wLookAt(lon, P.theta), rgb = new Uint8Array(wv * hv * 3), cx = (wv - 1) / 2, cy = (hv - 1) / 2;
      let miss = 0;
      for (let y = 0; y < hv; y++) for (let x = 0; x < wv; x++) {
        const vx = (x - cx) / fv, vy = (y - cy) / fv;
        const d = [C2W[0] * vx + C2W[1] * vy + C2W[2], C2W[3] * vx + C2W[4] * vy + C2W[5], C2W[6] * vx + C2W[7] * vy + C2W[8]];
        const q = slitToPix(P, d), o = (y * wv + x) * 3;
        if (!q) { miss++; continue; }
        const xx = Math.min(P.w - 1.001, q[0]), yy = q[1], x0 = xx | 0, y0 = Math.min(P.h - 2, yy | 0), fx = xx - x0, fy = yy - y0, x1 = Math.min(P.w - 1, x0 + 1);
        const a = (y0 * P.w + x0) * 3, b = (y0 * P.w + x1) * 3, c = ((y0 + 1) * P.w + x0) * 3, e = ((y0 + 1) * P.w + x1) * 3, sr = P.src.rgb;
        for (let ch = 0; ch < 3; ch++) rgb[o + ch] = (sr[a + ch] * (1 - fx) + sr[b + ch] * fx) * (1 - fy) + (sr[c + ch] * (1 - fx) + sr[e + ch] * fx) * fy + 0.5;
      }
      if (miss > wv * hv * 0.02) continue; // кадр вылез за полосу — пропускаем
      frames.push({ w: wv, h: hv, rgb, C2W, group: pi });
    }
  });
  Ps.forEach(P => { P.smC = P.smF = null; });
  return { frames, f: fv, k: best.k, score: best.score, info };
}

// ───────────────────────── демо-зал (синтетическая сцена) ─────────────────────────
function hash2(x, y) { let h = (x * 374761393 + y * 668265263) | 0; h = (h ^ (h >>> 13)) * 1274126177 | 0; return ((h ^ (h >>> 16)) >>> 0) / 4294967296; }
function vnoise(x, y) {
  const x0 = Math.floor(x), y0 = Math.floor(y), fx = x - x0, fy = y - y0;
  const sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
  const a = hash2(x0, y0), b = hash2(x0 + 1, y0), c = hash2(x0, y0 + 1), d = hash2(x0 + 1, y0 + 1);
  return (a + (b - a) * sx) * (1 - sy) + (c + (d - c) * sx) * sy;
}
function fbm(x, y, oct) { let s = 0, a = 0.5, f = 1; for (let i = 0; i < oct; i++) { s += a * vnoise(x * f, y * f); f *= 2.03; a *= 0.5; } return s; }
const PAL = [[0.55, 0.12, 0.08], [0.08, 0.22, 0.45], [0.75, 0.55, 0.15], [0.12, 0.35, 0.2], [0.4, 0.15, 0.4], [0.85, 0.75, 0.6], [0.05, 0.08, 0.12]];
// Демо-зал: стены, паркет, кессонный потолок, картины, колонна, витрина с вазой, скамья, (опционально) прохожий.
// o — точка съёмки (м), d — единичное направление. Возвращает линейный RGB.
function rayBox(o, d, x0, x1, y0, y1, z0, z1) {
  let tn = -Infinity, tf = Infinity, ax = -1;
  const lo = [x0, y0, z0], hi = [x1, y1, z1];
  for (let k = 0; k < 3; k++) {
    if (Math.abs(d[k]) < 1e-12) { if (o[k] < lo[k] || o[k] > hi[k]) return null; continue; }
    let ta = (lo[k] - o[k]) / d[k], tb = (hi[k] - o[k]) / d[k]; if (ta > tb) { const s = ta; ta = tb; tb = s; }
    if (ta > tn) { tn = ta; ax = k; } if (tb < tf) tf = tb; if (tn > tf) return null;
  }
  return tn > 1e-6 ? { t: tn, ax } : null;
}
function raySphere(o, d, c, r) {
  const ox = o[0] - c[0], oy = o[1] - c[1], oz = o[2] - c[2];
  const b = ox * d[0] + oy * d[1] + oz * d[2], cc = ox * ox + oy * oy + oz * oz - r * r, disc = b * b - cc;
  if (disc < 0) return null; const t = -b - Math.sqrt(disc); return t > 1e-6 ? t : null;
}
function rayCyl(o, d, cx, cy, r, z0, z1) {
  const ox = o[0] - cx, oy = o[1] - cy, a = d[0] * d[0] + d[1] * d[1]; if (a < 1e-12) return null;
  const b = ox * d[0] + oy * d[1], c = ox * ox + oy * oy - r * r, disc = b * b - a * c; if (disc < 0) return null;
  const t = (-b - Math.sqrt(disc)) / a; if (t <= 1e-6) return null;
  const z = o[2] + t * d[2]; return z >= z0 && z <= z1 ? t : null;
}
function demoSceneRay(o, d, person) {
  const X = 5, Y = 6.5, Zf = -1.6, Zc = 3.4;
  let t = Infinity, face = -1;
  const pl = (k, v, f) => { if (Math.abs(d[k]) > 1e-9) { const tt = (v - o[k]) / d[k]; if (tt > 1e-6 && tt < t) { t = tt; face = f; } } };
  pl(0, X, 0); pl(0, -X, 1); pl(1, Y, 2); pl(1, -Y, 3); pl(2, Zc, 4); pl(2, Zf, 5);
  let obj = null;
  const hit = (tt, name, extra) => { if (tt != null && tt < t) { t = tt; obj = name; face = extra; } };
  hit(rayCyl(o, d, 2.2, 1.6, 0.25, Zf, Zc), 'column', 0);
  const pb = rayBox(o, d, -1.6, -1.0, 0.8, 1.4, Zf, -0.62); if (pb) hit(pb.t, 'pedestal', pb.ax);
  hit(raySphere(o, d, [-1.3, 1.1, -0.42], 0.2), 'vase', 0);
  const bb = rayBox(o, d, -0.9, 0.9, -2.45, -2.0, Zf, -1.15); if (bb) hit(bb.t, 'bench', bb.ax);
  if (person) {
    hit(rayCyl(o, d, person[0], person[1], 0.21, Zf, 0.05), 'person', 0);
    hit(raySphere(o, d, [person[0], person[1], 0.2], 0.12), 'head', 0);
  }
  const px = o[0] + d[0] * t, py = o[1] + d[1] * t, pz = o[2] + d[2] * t;
  if (obj === 'column') {
    const ang = Math.atan2(py - 1.6, px - 2.2);
    const v = Math.sin(ang * 3 + pz * 4 + 6 * fbm(ang * 2, pz * 1.5, 4)), k = 0.78 + 0.18 * v * v * v;
    const sh = 0.75 + 0.25 * Math.cos(ang - 2.2);
    return [0.82 * k * sh, 0.8 * k * sh, 0.76 * k * sh];
  }
  if (obj === 'pedestal') {
    const g = 0.7 + 0.5 * fbm((face === 0 ? py : px) * 30, pz * 3, 3), s = face === 2 ? 1.1 : 0.8;
    return [0.16 * g * s, 0.09 * g * s, 0.05 * g * s];
  }
  if (obj === 'vase') {
    const ang = Math.atan2(py - 1.1, px + 1.3), band = Math.sin((pz + 0.42) * 50) > 0.3 ? 1 : 0;
    const k = 0.6 + 0.4 * Math.max(0, (pz + 0.42) / 0.2 + 0.5);
    return band ? [0.85 * k, 0.75 * k, 0.4 * k] : [0.08 * k, 0.2 * k * (1 + 0.3 * Math.sin(ang * 6)), 0.55 * k];
  }
  if (obj === 'bench') {
    const g = 0.75 + 0.4 * fbm(px * 12, py * 12 + pz * 12, 3);
    return [0.3 * g, 0.12 * g, 0.06 * g];
  }
  if (obj === 'person') { const s = Math.sin(pz * 18) > 0 ? 1 : 0.85; return pz > -0.6 ? [0.55 * s, 0.08, 0.06] : [0.05, 0.06, 0.12 * s]; }
  if (obj === 'head') return [0.75, 0.52, 0.4];
  if (face === 5) { // паркет
    const col = Math.floor(px / 0.22), off = hash2(col, 7) * 1.4, vy = py + off, plank = Math.floor(vy / 1.3);
    const hb = hash2(col, plank), tone = 0.55 + 0.45 * hb;
    const grain = 0.75 + 0.5 * fbm(px * 40, vy * 3.5, 3);
    const ex = Math.min(px / 0.22 - col, 1 - (px / 0.22 - col)), ey = Math.min(vy / 1.3 - plank, 1 - (vy / 1.3 - plank)) * 6;
    const seam = Math.min(1, Math.min(ex, ey) * 14);
    const k = tone * grain * (0.35 + 0.65 * seam);
    return [0.32 * k, 0.17 * k, 0.07 * k];
  }
  if (face === 4) { // потолок: кессоны, лампы
    const gx = px / 1.5, gy = py / 1.5, fx = gx - Math.floor(gx), fy = gy - Math.floor(gy);
    const e = Math.min(fx, 1 - fx, fy, 1 - fy), seam = Math.min(1, e * 18);
    const lx = ((px % 3) + 3) % 3 - 1.5, ly = ((py % 3) + 3) % 3 - 1.5, lamp = Math.hypot(lx, ly) < 0.22;
    if (lamp) return [3.5, 3.2, 2.6];
    const sp = 0.85 + 0.3 * fbm(px * 22, py * 22, 2);
    const k = (0.45 + 0.4 * seam) * sp;
    return [0.72 * k, 0.72 * k, 0.7 * k];
  }
  // стены
  const s = (face === 0 || face === 1) ? py : px, z = pz, wid = face;
  let base = 0.85 + 0.25 * fbm(s * 6 + wid * 13, z * 6, 3);
  let c = [0.62 * base, 0.58 * base, 0.52 * base];
  if (z < -1.42) { const k = 0.6 + 0.4 * fbm(s * 20, z * 4, 2); return [0.1 * k, 0.07 * k, 0.05 * k]; }
  if (z > -0.72 && z < -0.64) { const k = 0.8 + 0.2 * vnoise(s * 30, wid); return [0.3 * k, 0.24 * k, 0.18 * k]; }
  const L = (face === 0 || face === 1) ? Y : X, slot = Math.floor((s + L) / 3.4), sc = slot * 3.4 - L + 1.7;
  const hs = hash2(slot, wid + 40), pw = 0.75 + 0.45 * hs, ph = 0.55 + 0.4 * hash2(slot, wid + 80), pzc = 0.55 + 0.25 * hash2(slot, wid + 120);
  const ds = s - sc, dz = z - pzc;
  if (Math.abs(sc) < L - 1 && Math.abs(ds) < pw + 0.09 && Math.abs(dz) < ph + 0.09) {
    if (Math.abs(ds) > pw || Math.abs(dz) > ph) { const k = 0.6 + 0.6 * fbm(ds * 50, dz * 50, 2); return [0.75 * k, 0.55 * k, 0.18 * k]; }
    const pa = PAL[(slot * 3 + wid * 5 + 70) % PAL.length], pb2 = PAL[(slot * 7 + wid + 71) % PAL.length], pc = PAL[(slot + wid * 2 + 3) % PAL.length];
    const n1 = fbm(ds * 3.2 + slot * 9.1, dz * 3.2 + wid * 3.7, 4), n2 = fbm(ds * 9 + 40, dz * 9 - slot, 3);
    const m = Math.max(0, Math.min(1, (n1 - 0.3) * 2.6));
    for (let q = 0; q < 3; q++) c[q] = pa[q] * (1 - m) + pb2[q] * m;
    if (n2 > 0.62) for (let q = 0; q < 3; q++) c[q] = c[q] * 0.4 + pc[q] * 0.6;
    const lit = 0.85 + 0.25 * (1 - Math.abs(dz) / ph);
    return [c[0] * lit, c[1] * lit, c[2] * lit];
  }
  if (Math.abs(sc) < L - 1 && Math.abs(ds - pw * 0.6) < 0.16 && Math.abs(z - (pzc - ph - 0.25)) < 0.07) {
    const line = Math.abs(z - (pzc - ph - 0.25)) < 0.045 && vnoise(ds * 60, slot) > 0.45 && Math.abs(z - (pzc - ph - 0.25)) > 0.008;
    return line ? [0.1, 0.1, 0.1] : [0.85, 0.83, 0.78];
  }
  return c;
}
function demoScene(d) { return demoSceneRay([0, 0, 0], d, null); }

// Кадры по плану с «ошибками» пользователя, дрейфом гироскопа, экспозицией и виньетированием.
// opts.handheld — радиус (м), на котором телефон вращается вокруг тела (0 — вокруг объектива);
// opts.person — прохожий в части кадров; opts.k1 — бочкообразная дисторсия объектива.
function renderDemoFrames(opts, report) {
  const w = opts.w || 600, h = opts.h || 800, fTrue = opts.fTrue || 628;
  const rand = rng(opts.seed || 7);
  const gauss = () => { let s = 0; for (let i = 0; i < 6; i++) s += rand(); return (s - 3) * 0.7; };
  const hf = 2 * Math.atan(w / 2 / fTrue) * 180 / Math.PI, vf = 2 * Math.atan(h / 2 / fTrue) * 180 / Math.PI;
  let plan = makePlan(hf, vf, 0.36);
  if (opts.skipNadir) plan = plan.filter(t => !(t.pole && t.pitch < 0));
  const d2r = Math.PI / 180, frames = [];
  const hh = opts.handheld || 0, k1 = opts.k1 || 0;
  let drift = 0;
  plan.forEach((t, k) => {
    drift += (opts.drift != null ? opts.drift : 0.12) + 0.25 * gauss();
    const yaw = (t.yaw + 1.8 * gauss()) * d2r, pitch = (t.pole ? t.pitch - Math.sign(t.pitch) * 1.5 : t.pitch + 1.5 * gauss()) * d2r, roll = 2.5 * gauss() * d2r;
    let C2W = m3mul(c2wLookAt(yaw, pitch), rotZ(roll));
    // центр съёмки: телефон в вытянутых руках вращается вокруг тела + дрожание
    const o = [hh * Math.sin(yaw) * Math.cos(pitch) + 0.012 * gauss(), hh * Math.cos(yaw) * Math.cos(pitch) + 0.012 * gauss(), hh * 0.5 * Math.sin(pitch) + 0.012 * gauss()];
    const person = opts.person && k >= 2 && k <= 6 ? [-2.6 + 1.1 * (k - 2), -3.4] : null;
    const noiseR = expSO3(0.35 * gauss() * d2r, 0.35 * gauss() * d2r, 0.2 * gauss() * d2r);
    const gyro = m3mul(m3mul(rotZ(drift * d2r), noiseR), C2W);
    const gain = 0.82 + 0.36 * rand(), wb = [1 + 0.04 * gauss(), 1, 1 + 0.04 * gauss()];
    const rgb = new Uint8Array(w * h * 3), cx = (w - 1) / 2, cy = (h - 1) / 2, R2 = cx * cx + cy * cy, rn2 = (w * w + h * h) / 4 / (fTrue * fTrue);
    for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) {
      const acc = [0, 0, 0];
      for (let ss = 0; ss < 2; ss++) {
        let vx = (x - cx + (ss - 0.5) * 0.5) / fTrue, vy = (y - cy + (0.5 - ss) * 0.5) / fTrue;
        if (k1) { const r2 = (vx * vx + vy * vy) / rn2, s = 1 + k1 * r2; vx *= s; vy *= s; }
        const n = Math.sqrt(vx * vx + vy * vy + 1);
        const dw = [(C2W[0] * vx + C2W[1] * vy + C2W[2]) / n, (C2W[3] * vx + C2W[4] * vy + C2W[5]) / n, (C2W[6] * vx + C2W[7] * vy + C2W[8]) / n];
        const c = demoSceneRay(o, dw, person); acc[0] += c[0]; acc[1] += c[1]; acc[2] += c[2];
      }
      const r2 = ((x - cx) * (x - cx) + (y - cy) * (y - cy)) / R2, vig = 1 - 0.18 * r2;
      const p = (y * w + x) * 3;
      for (let q = 0; q < 3; q++) rgb[p + q] = Math.max(0, Math.min(255, lin2srgb(acc[q] / 2 * gain * wb[q] * vig) + Math.round(gauss() * 1.2)));
    }
    frames.push({ w, h, rgb, C2W: gyro, C2Wtrue: C2W, center: o });
    if (report) report({ stage: 'demo', frac: (k + 1) / plan.length });
  });
  return { frames, plan, fTrue, f0: 0.75 * Math.max(w, h) };
}

if (typeof module !== 'undefined') module.exports = { panosToFrames, slitToPix, slitSetup, slitErpGrad, slitSmall, erpShiftNcc,
  m3mul, m3T, rotX, rotY, rotZ, expSO3, logSO3, rotAngle, polar3, gyroToC2W, dirFromLonLat, lonLatFromDir, camForward,
  compose, measureParallax, calibrateFocal, makePlan, c2wLookAt, stitchPanorama, depthAt, aiFillHoles, aiHealSeams, aiSeamClusters, nativeTile, applyInpaintTile, fillHoles, phonePanoToERP, parseGPano, detectPeriod, renderDemoFrames, demoScene, demoSceneRay, renderPerspective, forEachErpInView, erpSample, rng, downscaleRGBA, downscaleMask,
  lin2srgb, SRGB2LIN
};
