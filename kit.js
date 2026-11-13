// Facade kit: real 3D pieces on street-facing windows - sills, heads, surrounds, stone mullions - so fronts
// catch light and cast small shadows instead of reading as flat textured boxes. One instanced box does it all.
// Input rows (from tools/buildgeo.py): x, sill y, z, normal x, normal z, width, height, wall style, wall r, g, b.
import * as THREE from 'three';

const ROW = 11;
export function buildFacadeKit(scene, W) {
  const n = W.length / ROW;
  if (!n) return null;
  const boxes = [];   // [row index, along-wall offset, y offset from sill, size x, size y, depth, r, g, b]
  for (let i = 0; i < n; i++) {
    const o = i * ROW, w = W[o + 5], h = W[o + 6], st = W[o + 7], r = W[o + 8], g = W[o + 9], b = W[o + 10];
    const light = [236, 231, 222], stone = [Math.min(255, r * 1.06 + 8), Math.min(255, g * 1.06 + 8), Math.min(255, b * 1.06 + 8)];
    if (st === 4) {            // brick: stone sill, gauged-brick flat arch
      boxes.push([i, 0, -0.07, w + 0.24, 0.07, 0.11, ...light]);
      boxes.push([i, 0, h + 0.01, w + 0.22, 0.21, 0.035, 198, 132, 98]);
    } else if (st === 0) {     // render / painted: moulded surround and sill
      boxes.push([i, 0, -0.07, w + 0.30, 0.07, 0.12, ...light]);
      boxes.push([i, 0, h, w + 0.30, 0.12, 0.06, ...light]);
      boxes.push([i, -(w / 2 + 0.06), 0, 0.11, h, 0.045, ...light]);
      boxes.push([i, w / 2 + 0.06, 0, 0.11, h, 0.045, ...light]);
    } else {                   // ashlar: hood mould, jambs, central mullion and transom in the wall's own stone
      boxes.push([i, 0, -0.08, w + 0.20, 0.08, 0.10, ...stone]);
      boxes.push([i, 0, h, w + 0.30, 0.13, 0.09, ...stone]);
      boxes.push([i, -(w / 2 + 0.03), 0, 0.12, h, 0.06, ...stone]);
      boxes.push([i, w / 2 + 0.03, 0, 0.12, h, 0.06, ...stone]);
      boxes.push([i, 0, 0, 0.10, h, 0.07, ...stone]);
      boxes.push([i, 0, h * 0.56, w, 0.08, 0.06, ...stone]);
    }
  }
  const geo = new THREE.BoxGeometry(1, 1, 1); geo.translate(0, 0.5, 0.5);     // origin: bottom centre, back face
  const mesh = new THREE.InstancedMesh(geo, new THREE.MeshStandardMaterial({ roughness: 0.85 }), boxes.length);
  const m = new THREE.Matrix4(), c = new THREE.Color();
  boxes.forEach(([i, s, y, sx, sy, sz, r, g, b], k) => {
    const o = i * ROW, nx = W[o + 3], nz = W[o + 4], tx = -nz, tz = nx;
    m.set(tx * sx, 0, nx * sz, W[o] + tx * s + nx * 0.004,
          0, sy, 0, W[o + 1] + y,
          tz * sx, 0, nz * sz, W[o + 2] + tz * s + nz * 0.004,
          0, 0, 0, 1);
    mesh.setMatrixAt(k, m);
    mesh.setColorAt(k, c.setRGB(r / 255, g / 255, b / 255, THREE.SRGBColorSpace));
  });
  mesh.castShadow = mesh.receiveShadow = true;
  mesh.frustumCulled = false;
  scene.add(mesh);
  return mesh;
}
