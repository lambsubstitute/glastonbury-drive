// Baked ambient occlusion for ground-level surfaces (tools/bake_ao.py): one top-down map of the town tile,
// sampled by world position, so ground, roads and pavements all darken where buildings close in.
// ao.wrap(material) is a no-op until main.js calls ao.enable() (i.e. when no bake exists for this build).
import * as THREE from 'three';

export const ao = {
  wrap: (m) => m,
  enable(url, x0, z0, w) {
    const tex = new THREE.TextureLoader().load(url);
    tex.minFilter = THREE.LinearFilter; tex.generateMipmaps = false;
    const box = new THREE.Vector3(x0, z0, w);
    this.wrap = (m) => {
      const prev = m.onBeforeCompile;
      m.onBeforeCompile = (sh, r) => {
        if (prev) prev(sh, r);
        sh.uniforms.aoGround = { value: tex }; sh.uniforms.aoBox = { value: box };
        sh.vertexShader = sh.vertexShader
          .replace('#include <common>', '#include <common>\nvarying vec2 vAoW;')
          .replace('#include <project_vertex>', '#include <project_vertex>\nvAoW = (modelMatrix * vec4(transformed, 1.0)).xz;');
        sh.fragmentShader = sh.fragmentShader
          .replace('#include <common>', '#include <common>\nuniform sampler2D aoGround; uniform vec3 aoBox; varying vec2 vAoW;')
          .replace('#include <map_fragment>', `#include <map_fragment>
            { vec2 q = (vAoW - aoBox.xy) / aoBox.z;
              if (q.x > 0.0 && q.x < 1.0 && q.y > 0.0 && q.y < 1.0) diffuseColor.rgb *= texture2D(aoGround, vec2(q.x, 1.0 - q.y)).r; }`);
      };
      const key = m.customProgramCacheKey ? m.customProgramCacheKey() : '';
      m.customProgramCacheKey = () => key + '|groundAO';
      return m;
    };
  },
};
