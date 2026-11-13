"""Runs inside Blender (blender -b -P bake_ao_blender.py -- <work dir>): bakes ambient occlusion with Cycles.

Reads <work>/tris.npy (N,3,3 float32, Blender coordinates: x east, y north, z up) for walls + roofs and
<work>/terrain.npy (n,n heights) + <work>/terrain.json, writes <work>/ao_corners.npy (N*3 floats, 0..1, one
per triangle corner in input order) and <work>/ground_ao.png (top-down occlusion of the ground, north up)."""
import bpy, json, os, sys
import numpy as np

work = sys.argv[sys.argv.index("--") + 1]
cfg = json.load(open(os.path.join(work, "terrain.json")))
bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene
scene.render.engine = "CYCLES"
scene.cycles.device = "CPU"
scene.cycles.samples = cfg.get("samples", 48)
scene.world = bpy.data.worlds.new("w")
scene.world.light_settings.distance = cfg.get("distance", 12.0)     # how far occluders count, metres


def mesh_object(name, verts, faces):
    me = bpy.data.meshes.new(name)
    me.vertices.add(len(verts))
    me.vertices.foreach_set("co", verts.ravel())
    k = faces.shape[1]
    me.loops.add(faces.size)
    me.loops.foreach_set("vertex_index", faces.ravel())
    me.polygons.add(len(faces))
    me.polygons.foreach_set("loop_start", np.arange(0, faces.size, k))
    me.polygons.foreach_set("loop_total", np.full(len(faces), k))
    me.update(calc_edges=True)
    ob = bpy.data.objects.new(name, me)
    scene.collection.objects.link(ob)
    mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    me.materials.append(mat)
    return ob, me, mat


# buildings: one unshared vertex per corner, so corner order = input order
T = np.load(os.path.join(work, "tris.npy"))
bob, bme, bmat = mesh_object("buildings", T.reshape(-1, 3), np.arange(len(T) * 3).reshape(-1, 3))
att = bme.color_attributes.new("ao", "FLOAT_COLOR", "CORNER")
bme.color_attributes.active_color = att
bme.color_attributes.render_color_index = 0

# ground grid with a planar UV (u east, v north)
H = np.load(os.path.join(work, "terrain.npy"))
n, s, x0, w = H.shape[0], cfg["s"], cfg["x0"], cfg["w"]
ii, jj = np.meshgrid(np.arange(n), np.arange(n))                   # jj = row (south-going), ii = column
X = x0 + (ii + 0.5) * s
Y = -(x0 + (jj + 0.5) * s)
V = np.stack([X, Y, H], -1).reshape(-1, 3).astype(np.float32)
a = (jj[:-1, :-1] * n + ii[:-1, :-1]).ravel()
F = np.stack([a, a + n, a + n + 1, a + 1], 1)                      # counter-clockwise seen from above
gob, gme, gmat = mesh_object("ground", V, F)
uvl = gme.uv_layers.new(name="uv")
li = np.zeros(len(gme.loops), np.int32)
gme.loops.foreach_get("vertex_index", li)
uv = np.stack([(V[li, 0] - x0) / w, (V[li, 1] + x0 + w) / w], 1).astype(np.float32)
uvl.data.foreach_set("uv", uv.ravel())
R = cfg.get("ground_px", 2048)
img = bpy.data.images.new("ground_ao", R, R, alpha=False)
node = gmat.node_tree.nodes.new("ShaderNodeTexImage")
node.image = img
gmat.node_tree.nodes.active = node


def bake(ob, target):
    bpy.ops.object.select_all(action="DESELECT")
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob
    scene.render.bake.target = target
    scene.render.bake.margin = 2
    bpy.ops.object.bake(type="AO")


bake(bob, "VERTEX_COLORS")
col = np.zeros(len(bme.loops) * 4, np.float32)
bme.color_attributes["ao"].data.foreach_get("color", col)
np.save(os.path.join(work, "ao_corners.npy"), col.reshape(-1, 4)[:, 0])

bake(gob, "IMAGE_TEXTURES")
img.filepath_raw = os.path.join(work, "ground_ao.png")
img.file_format = "PNG"
img.save()
print("AO baked:", len(T), "triangles,", R, "px ground map")
