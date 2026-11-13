"""Bake ambient occlusion with Blender (Cycles) for the town built by process.py.  Run after process.py:

  .venv/bin/python tools/bake_ao.py [--blender /Applications/Blender.app/Contents/MacOS/Blender]

Writes, next to the geometry in game/data/:
  walls<N>_ao.bin, roof_ao.bin   one byte per vertex (255 = open sky), multiplied into the vertex colours on load
  ground_ao.jpg                  top-down occlusion of the ground, sampled by world position for ground,
                                 roads and pavements
The game works without these files (no occlusion). Nothing here changes geometry or observed colours."""
import glob, json, os, shutil, subprocess, sys, tempfile
import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "game", "data")
BLENDER = sys.argv[sys.argv.index("--blender") + 1] if "--blender" in sys.argv else (
    shutil.which("blender") or "/Applications/Blender.app/Contents/MacOS/Blender")
FLOOR, GAMMA = 0.38, 0.85
WALL_FLOOR, WALL_GAMMA = 0.66, 0.45          # darkest multiplier, and a curve that keeps open walls near full brightness

T = json.load(open(os.path.join(OUT, "terrain.json")))
work = tempfile.mkdtemp(prefix="bake_ao_")

parts, tris, ups = [], [], []
for f in sorted(glob.glob(os.path.join(OUT, "walls*_pos.bin"))) + [os.path.join(OUT, "roof_pos.bin")]:
    P = np.fromfile(f, np.float32).reshape(-1, 3, 3)
    flip = np.zeros(len(P), bool)
    if f.endswith("roof_pos.bin"):   # the game turns downward-facing roof triangles over; bake them the same way
        u, v = P[:, 1] - P[:, 0], P[:, 2] - P[:, 0]
        flip = (u[:, 2] * v[:, 0] - u[:, 0] * v[:, 2]) < 0
        P = P.copy()
        P[flip] = P[flip][:, [0, 2, 1]]
    parts.append((f, len(P), flip))
    nrm = np.cross(P[:, 1] - P[:, 0], P[:, 2] - P[:, 0])
    ups.append(np.repeat(nrm[:, 1] / (np.linalg.norm(nrm, axis=1) + 1e-9), 3))
    tris.append(np.stack([P[..., 0], -P[..., 2], P[..., 1]], -1))      # game (x, y up, z south) -> Blender (x, y north, z up)
np.save(os.path.join(work, "tris.npy"), np.concatenate(tris).astype(np.float32))
n = T["n"]
H = np.fromfile(os.path.join(OUT, "terrain.bin"), np.uint16).reshape(n, n) / 100.0 + T["min"]
np.save(os.path.join(work, "terrain.npy"), H.astype(np.float32))
json.dump({"s": T["s"], "x0": T["origin"], "w": n * T["s"], "samples": 48, "distance": 12.0, "ground_px": 2048},
          open(os.path.join(work, "terrain.json"), "w"))

r = subprocess.run([BLENDER, "-b", "-P", os.path.join(ROOT, "tools", "bake_ao_blender.py"), "--", work],
                   capture_output=True, text=True)
if not os.path.exists(os.path.join(work, "ao_corners.npy")):
    sys.exit("Blender bake failed:\n" + r.stdout[-1500:] + r.stderr[-1500:])

raw = np.clip(np.load(os.path.join(work, "ao_corners.npy")), 0, 1)
# a wall on open flat ground still has the ground filling half its view, so its raw value tops out near 0.5
# (a flat roof near 1.0). Divide by what the surface would see with nothing around it, so only real
# enclosure (alleys, re-entrant corners, overhanging neighbours) darkens it.
open_sky = np.clip((1 + np.concatenate(ups)) / 2, 0.5, 1.0)
ao = np.clip(raw / open_sky, 0, 1)
# walls are sampled only at their corners, which sit where wall meets ground, roof or neighbour, so even an
# open wall reads half occluded and the big triangles smear that across the whole face. Walls therefore get
# a gentle curve and a high floor (shade in alleys, no general darkening); roofs keep the full range.
is_wall = np.concatenate(ups) < 0.3
ao = np.where(is_wall, WALL_FLOOR + (1 - WALL_FLOOR) * ao ** WALL_GAMMA, FLOOR + (1 - FLOOR) * ao ** GAMMA)
print("mean wall %.2f, mean roof %.2f" % (ao[is_wall].mean(), ao[~is_wall].mean()))
k = 0
for f, m, flip in parts:
    a = ao[k * 3:(k + m) * 3].reshape(-1, 3).copy()
    a[flip] = a[flip][:, [0, 2, 1]]                                    # back to the file's vertex order
    k += m
    np.round(a * 255).astype(np.uint8).tofile(f.replace("_pos.bin", "_ao.bin"))
g = np.asarray(Image.open(os.path.join(work, "ground_ao.png")).convert("L"), np.float32) / 255.0
g = FLOOR + (1 - FLOOR) * np.clip(g, 0, 1) ** 1.8      # stronger on the ground, where it reads as contact shade
Image.fromarray(np.round(g * 255).astype(np.uint8)).save(os.path.join(OUT, "ground_ao.jpg"), quality=88)
print("AO: %d triangles, mean wall/roof %.2f, mean ground %.2f" % (k, ao.mean(), g.mean()))
shutil.rmtree(work, ignore_errors=True)

# mark the bake as belonging to this build: the game ignores occlusion files from an older build
mp = os.path.join(OUT, "meta.json")
meta = json.load(open(mp))
meta["ao"] = meta.get("build", "")
json.dump(meta, open(mp, "w"), indent=0)
