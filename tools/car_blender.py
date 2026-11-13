"""Runs inside Blender (blender -b -P car_blender.py -- <out.glb> [preview.png]): models the player's car.

A generic modern five-door hatchback (about 4.1 m x 1.75 m x 1.46 m), built from code so it can be re-run:
  - lower body: lofted cross-sections (plan width, side profile, tumblehome), subdivided, wheel arches cut
    with exact booleans (the cutter's black faces become the wheel wells), lower sill/bumper faces in black trim
  - greenhouse: lofted pillars and roof with flush side glass, a black B-pillar, curved windscreen and hatch glass
  - details projected onto the body surface by ray casting: lamps, grilles, UK plates, door shut lines, handles
  - wheels: lathed tyres, five-spoke alloys, discs and calipers

Glb layout (glTF: +z forward, +y up, x across; car origin on the ground, midway between the axles):
  Car > Body (everything that pitches/rolls) and Wheel_FL/FR/RL/RR (empties at the hubs; turn them to steer)
  each Wheel_* > <name>_Spin (rolls about x) and <name>_Brake (does not roll).
With an optional second path it also renders a Cycles preview of the car."""
import bpy, bmesh, math, sys
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree

argv = sys.argv[sys.argv.index("--") + 1:]
OUT = argv[0]
PREVIEW = argv[1] if len(argv) > 1 else None

bpy.ops.wm.read_factory_settings(use_empty=True)
scene = bpy.context.scene

WHEEL_R = 0.318          # 225/45 R17
TRACK = 0.765            # wheel centre plane, either side
AXLE_F, AXLE_R = 1.32, -1.24
L_FRONT, L_REAR = 2.13, -1.97


def P(lat, L, z):
    """car frame (lat across, L forward, z up) -> Blender (glTF export maps Blender -y to +z forward)"""
    return Vector((lat, -L, z))


def pchip(pts):
    xs = np.array([p[0] for p in pts], float); ys = np.array([p[1] for p in pts], float)
    o = np.argsort(xs); xs, ys = xs[o], ys[o]
    h = np.diff(xs); d = np.diff(ys) / h
    m = np.zeros_like(xs); m[0], m[-1] = d[0], d[-1]
    for k in range(1, len(xs) - 1):
        if d[k - 1] * d[k] > 0:
            w1, w2 = 2 * h[k] + h[k - 1], h[k] + 2 * h[k - 1]
            m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k])

    def f(x):
        k = int(np.clip(np.searchsorted(xs, x) - 1, 0, len(xs) - 2))
        t = (x - xs[k]) / h[k]
        t = min(max(t, 0.0), 1.0)
        return ((2 * t**3 - 3 * t**2 + 1) * ys[k] + (t**3 - 2 * t**2 + t) * h[k] * m[k]
                + (-2 * t**3 + 3 * t**2) * ys[k + 1] + (t**3 - t**2) * h[k] * m[k + 1])
    return f


# ------------------------------------------------------------------ materials
def mat(name, color, metal=0.0, rough=0.5, coat=0.0, emit=None, emit_k=0.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = (*color, 1)
    b.inputs["Metallic"].default_value = metal
    b.inputs["Roughness"].default_value = rough
    if coat:
        b.inputs["Coat Weight"].default_value = coat
        b.inputs["Coat Roughness"].default_value = 0.03
    if emit:
        b.inputs["Emission Color"].default_value = (*emit, 1)
        b.inputs["Emission Strength"].default_value = emit_k
    m.use_backface_culling = False
    return m


M = {
    "paint": mat("Paint", (0.42, 0.018, 0.013), metal=0.55, rough=0.32, coat=1.0),
    "glass": mat("Glass", (0.008, 0.010, 0.012), rough=0.03),
    "trim": mat("TrimMatte", (0.018, 0.018, 0.019), rough=0.72),
    "gloss": mat("TrimGloss", (0.006, 0.006, 0.007), rough=0.12),
    "chrome": mat("Chrome", (0.8, 0.8, 0.8), metal=1.0, rough=0.1),
    "lamp": mat("LampHousing", (0.55, 0.57, 0.6), metal=1.0, rough=0.16),
    "drl": mat("DRL", (0.9, 0.9, 0.9), rough=0.2, emit=(1.0, 0.97, 0.9), emit_k=4.0),
    "tail": mat("TailLamp", (0.32, 0.008, 0.008), rough=0.1, emit=(0.8, 0.02, 0.01), emit_k=0.8),
    "plate_w": mat("PlateWhite", (0.85, 0.85, 0.82), rough=0.35),
    "plate_y": mat("PlateYellow", (0.9, 0.62, 0.03), rough=0.35),
    "tyre": mat("Tyre", (0.011, 0.011, 0.011), rough=0.9),
    "rim": mat("Alloy", (0.72, 0.73, 0.75), metal=1.0, rough=0.26),
    "disc": mat("Disc", (0.28, 0.28, 0.28), metal=1.0, rough=0.45),
    "caliper": mat("Caliper", (0.03, 0.03, 0.035), rough=0.5),
    "well": mat("WheelWell", (0.012, 0.012, 0.012), rough=0.95),
}


def mesh_obj(name, verts, faces, mats, face_mat=None, parent=None, smooth=True):
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], faces)
    me.update(calc_edges=True)
    for m in mats:
        me.materials.append(M[m])
    if face_mat is not None:
        me.polygons.foreach_set("material_index", face_mat)
    me.polygons.foreach_set("use_smooth", [smooth] * len(me.polygons))
    ob = bpy.data.objects.new(name, me)
    scene.collection.objects.link(ob)
    if parent:
        ob.parent = parent
    return ob


def empty(name, loc=(0, 0, 0), parent=None):
    e = bpy.data.objects.new(name, None)
    e.location = loc
    scene.collection.objects.link(e)
    if parent:
        e.parent = parent
    return e


def activate(ob):
    for o in bpy.context.view_layer.objects:
        o.select_set(False)
    ob.select_set(True)
    bpy.context.view_layer.objects.active = ob


def recalc_normals(ob):
    bm = bmesh.new(); bm.from_mesh(ob.data)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(ob.data); bm.free()


car = empty("Car")
body = empty("Body", parent=car)

# ------------------------------------------------------------------ lower body (loft)
z_top = pchip([(2.15, 0.62), (2.10, 0.77), (2.02, 0.85), (1.85, 0.905), (1.4, 0.96), (1.05, 0.99), (0.0, 1.02),
               (-1.0, 1.04), (-1.6, 1.045), (-1.86, 1.03), (-1.93, 1.0), (-1.99, 0.92)])
z_bot = pchip([(2.15, 0.36), (2.0, 0.30), (1.6, 0.27), (0.0, 0.26), (-1.6, 0.28), (-1.9, 0.32), (-1.99, 0.36)])
half_w = pchip([(2.15, 0.60), (2.10, 0.74), (2.0, 0.82), (1.8, 0.862), (1.3, 0.875), (-1.3, 0.875), (-1.75, 0.865),
                (-1.9, 0.83), (-1.99, 0.74)])
# one side of the cross-section, bottom centre to top centre: (fraction of half width, height)
def side_ring(L, shrink=1.0):
    w, zb, zt = half_w(L) * shrink, z_bot(L), z_top(L)
    h = zt - zb
    if shrink < 1:      # the end-cap ring: pull heights toward the middle as well
        zm = zb + h * 0.5; zb, zt = zm - (zm - zb) * shrink * 1.6, zm + (zt - zm) * shrink * 1.6; h = zt - zb
    return [(0.0, zb), (0.78 * w, zb), (0.94 * w, zb + 0.035), (w, zb + 0.16 * h), (w, zb + 0.45 * h),
            (0.993 * w, zb + 0.70 * h), (0.965 * w, zt - 0.065), (0.90 * w, zt - 0.014), (0.6 * w, zt), (0.0, zt + 0.012)]


def full_ring(L, shrink=1.0):
    r = side_ring(L, shrink)
    return [(y, z) for y, z in r] + [(-y, z) for y, z in reversed(r[1:-1])]


u = np.linspace(0, 1, 46)
stations = list(L_REAR + (L_FRONT - L_REAR) * (0.5 - 0.5 * np.cos(np.pi * u)))
verts, faces = [], []
rings = []
for L in stations:
    ring = full_ring(L)
    rings.append(len(verts))
    verts += [P(y, L, z) for y, z in ring]
n = len(full_ring(0))
for i in range(len(stations) - 1):
    a, b = rings[i], rings[i + 1]
    for k in range(n):
        k2 = (k + 1) % n
        faces.append((a + k, a + k2, b + k2, b + k))
for end, L, step in ((0, L_REAR, -0.022), (len(stations) - 1, L_FRONT, 0.022)):
    cap = len(verts)
    verts += [P(y, L + step, z) for y, z in full_ring(L, 0.32)]
    a = rings[end]
    for k in range(n):
        k2 = (k + 1) % n
        faces.append((a + k, a + k2, cap + k2, cap + k))
    c = len(verts)
    ys = [v for v in full_ring(L, 0.32)]
    verts.append(P(0, L + step * 1.15, sum(z for _, z in ys) / len(ys)))
    for k in range(n):
        faces.append((cap + k, cap + (k + 1) % n, c))
lower = mesh_obj("BodyShell", verts, faces, ["paint", "well", "trim"], parent=body)
recalc_normals(lower)
sub = lower.modifiers.new("sub", "SUBSURF"); sub.levels = sub.render_levels = 2
activate(lower); bpy.ops.object.modifier_apply(modifier="sub")

# wheel arches: exact boolean with cylinders that carry the wheel-well material
for L in (AXLE_F, AXLE_R):
    for s in (1, -1):
        bpy.ops.mesh.primitive_cylinder_add(vertices=48, radius=0.37, depth=0.56,
                                            location=P(s * 0.84, L, WHEEL_R + 0.015), rotation=(0, math.pi / 2, 0))
        cut = bpy.context.active_object
        cut.data.materials.append(M["well"])
        bo = lower.modifiers.new("arch", "BOOLEAN"); bo.object = cut; bo.operation = "DIFFERENCE"; bo.solver = "EXACT"
        bo.material_mode = "TRANSFER"
        activate(lower); bpy.ops.object.modifier_apply(modifier="arch")
        bpy.data.objects.remove(cut, do_unlink=True)

# black plastic along the sills and the bottom of both bumpers
well_i = lower.data.materials.find("WheelWell"); trim_i = lower.data.materials.find("TrimMatte")
for p in lower.data.polygons:
    c = p.center
    if p.material_index != well_i and (c.z < 0.335 or (c.z < 0.40 and abs(c.y) > 1.85)):
        p.material_index = trim_i
activate(lower)
try:
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(40))
except Exception:
    bpy.ops.object.shade_smooth()

# ------------------------------------------------------------------ greenhouse
# One loft along the car from the windscreen foot to the hatch foot: each cross-section rises from the belt
# (on the body top) to the glasshouse top and over the crowned roof, so windscreen, roof and hatch are one
# smooth surface. Faces become glass, black pillar or paint by where they are.
GH_FRONT, GH_REAR = 1.10, -1.95
gh_top = pchip([(1.10, 0.975), (0.75, 1.13), (0.35, 1.29), (0.05, 1.395), (-0.2, 1.44), (-0.6, 1.457), (-1.1, 1.448),
                (-1.38, 1.42), (-1.55, 1.37), (-1.75, 1.22), (-1.95, 1.045)])
gh_w = pchip([(1.10, 0.79), (0.6, 0.73), (0.0, 0.672), (-1.0, 0.665), (-1.45, 0.645), (-1.75, 0.66), (-1.95, 0.70)])
S_SIDE = [0.0, 0.06, 0.5, 0.88, 1.0]
ROOF = [(-0.03, 0.028), (0.85, 0.045), (0.55, 0.055), (0.25, 0.059), (0.0, 0.06)]   # (fraction of roof half width, rise); first is absolute inset
WS_TOP, HATCH_TOP = -0.02, -1.50          # windscreen / hatch glass meet the roof here
WIN_F, B0, B1, WIN_R = 0.98, -0.40, -0.33, -1.30
L_B = 0.5 * (B0 + B1)                      # B-pillar: the shut line between the doors runs down from it


def gh_ring(L):
    zb, zt = z_top(L) - 0.02, gh_top(L)
    wb, wr = half_w(L) * 0.87, gh_w(L)
    one = [(wb + (wr - wb) * s + 0.022 * math.sin(math.pi * s) * min(1, (zt - zb) / 0.3), zb + (zt - zb) * s) for s in S_SIDE]
    one += [((wr + f) if i == 0 else wr * f, zt + dz) for i, (f, dz) in enumerate(ROOF)]
    return one + [(-y, z) for y, z in reversed(one[:-1])]


def gh_mat(L, band):
    """band: 0 seal, 1-2 side glass, 3 rail, 4 pillar edge, 5+ roof (windscreen/hatch at the ends)"""
    if band == 0:
        return "trim"
    if band in (1, 2):
        if B0 <= L <= B1:
            return "gloss"
        return "glass" if (B1 < L < WIN_F or WIN_R < L < B0) else "paint"
    if band >= 5 and (L > WS_TOP or L < HATCH_TOP):
        edge = band == 5 or L > 1.02 or L < -1.90 or abs(L - WS_TOP) < 0.06 or abs(L - HATCH_TOP) < 0.06
        return "gloss" if edge else "glass"
    if band == 4 and (L > WS_TOP + 0.04 or L < HATCH_TOP - 0.04):
        return "gloss"
    return "paint"


u = np.linspace(0, 1, 56)
GH_L = list(GH_REAR + (GH_FRONT - GH_REAR) * u)
for x in (WIN_F, B0, B1, WIN_R, WS_TOP, HATCH_TOP):     # section exactly on every material edge
    GH_L.append(x)
GH_L = sorted(set(round(x, 4) for x in GH_L))
verts, faces, fm = [], [], []
names = ["paint", "glass", "gloss", "trim"]
nr = len(gh_ring(0.0)); half = (nr + 1) // 2
for L in GH_L:
    verts += [P(y, L, z) for y, z in gh_ring(L)]
for i in range(len(GH_L) - 1):
    Lm = 0.5 * (GH_L[i] + GH_L[i + 1])
    for k in range(nr - 1):
        a, b = i * nr + k, (i + 1) * nr + k
        faces.append((a, b, b + 1, a + 1))
        band = k if k < half - 1 else nr - 2 - k     # mirror the band index for the far side
        fm.append(names.index(gh_mat(Lm, band)))
for i, flip in ((0, False), (len(GH_L) - 1, True)):   # close both ends where they sit on the body
    f = [i * nr + k for k in range(nr)]
    faces.append(tuple(f[::-1] if flip else f)); fm.append(names.index("gloss"))
gh = mesh_obj("Greenhouse", verts, faces, names, fm, parent=body)
recalc_normals(gh)
activate(gh)
try:
    bpy.ops.object.shade_smooth_by_angle(angle=math.radians(50))
except Exception:
    pass

# ------------------------------------------------------------------ details projected onto the body
dg = bpy.context.evaluated_depsgraph_get()
bvh = BVHTree.FromObject(lower, dg)


def decal(name, m, axis, v_range, u_fn, nv=6, nu=12, off=0.004):
    """grid in (u, v) projected onto the body along an axis. axis: 'front' (u=lat, v=z), 'rear' (u=lat, v=z),
    'right'/'left' (u=L, v=z). u_fn(v) -> (u0, u1). Faces whose corners miss the body are dropped."""
    verts, faces, idx = [], [], {}
    for j in range(nv + 1):
        v = v_range[0] + (v_range[1] - v_range[0]) * j / nv
        u0, u1 = u_fn(v)
        for i in range(nu + 1):
            uu = u0 + (u1 - u0) * i / nu
            if axis == "front":
                o, d = P(uu, 4, v), Vector((0, 1, 0))
            elif axis == "rear":
                o, d = P(uu, -4, v), Vector((0, -1, 0))
            elif axis == "right":
                o, d = P(3, uu, v), Vector((-1, 0, 0))
            else:
                o, d = P(-3, uu, v), Vector((1, 0, 0))
            hit, nrm, _, _ = bvh.ray_cast(o, d, 8)
            if hit is not None:
                idx[(i, j)] = len(verts)
                verts.append(hit + nrm * off)
    for j in range(nv):
        for i in range(nu):
            q = [(i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)]
            if all(k in idx for k in q):
                f = [idx[k] for k in q]
                faces.append(f if axis in ("front", "left") else f[::-1])
    return mesh_obj(name, verts, faces, [m], parent=body)


for s, side in ((1, "R"), (-1, "L")):
    lo = lambda a, b: (min(s * a, s * b), max(s * a, s * b))
    # headlamp: narrowing toward the grille, wrapping round the corner; a daytime running strip along its top
    decal(f"Headlamp{side}", "lamp", "front", (0.735, 0.82), lambda z: lo(0.42 + (z - 0.735) * 0.9, 0.82 - (z - 0.735) * 0.6), off=0.005)
    decal(f"DRL{side}", "drl", "front", (0.812, 0.826), lambda z: lo(0.48, 0.77), nv=1, off=0.008)
    # tail lamp on the tailgate corner and its wrap onto the flank
    decal(f"TailLamp{side}", "tail", "rear", (0.86, 0.99), lambda z: lo(0.48 + (0.99 - z) * 0.25, 0.80), off=0.005)
    # door shut lines (front door leading edge, B-pillar, rear door around the arch) and handles
    ax = "right" if s > 0 else "left"
    decal(f"ShutA{side}", "gloss", ax, (0.34, 0.985), lambda z: (0.96, 0.966), nv=12, nu=1, off=0.002)
    decal(f"ShutB{side}", "gloss", ax, (0.34, 1.0), lambda z: (L_B - 0.003, L_B + 0.003), nv=12, nu=1, off=0.002)
    decal(f"ShutC{side}", "gloss", ax, (0.62, 1.01), lambda z: (-1.03, -1.024), nv=10, nu=1, off=0.002)
    decal(f"HandleF{side}", "chrome", ax, (0.925, 0.948), lambda z: (0.22, 0.39), nv=1, nu=4, off=0.008)
    decal(f"HandleR{side}", "chrome", ax, (0.935, 0.958), lambda z: (-0.92, -0.75), nv=1, nu=4, off=0.008)

decal("Grille", "gloss", "front", (0.70, 0.785), lambda z: (-0.38 - (z - 0.7), 0.38 + (z - 0.7)), nv=3, off=0.004)
decal("Intake", "trim", "front", (0.33, 0.47), lambda z: (-0.62, 0.62), nv=4, nu=16, off=0.003)
decal("PlateF", "plate_w", "front", (0.495, 0.606), lambda z: (-0.26, 0.26), nv=2, nu=8, off=0.012)
decal("PlateR", "plate_y", "rear", (0.60, 0.711), lambda z: (-0.26, 0.26), nv=2, nu=8, off=0.008)
decal("Diffuser", "trim", "rear", (0.27, 0.42), lambda z: (-0.72, 0.72), nv=3, nu=16, off=0.003)
decal("HatchHandle", "chrome", "rear", (0.76, 0.79), lambda z: (-0.12, 0.12), nv=1, nu=4, off=0.008)

# door mirrors: rounded housing on a short stalk, glass on the back
for s, side in ((1, "R"), (-1, "L")):
    bpy.ops.mesh.primitive_cube_add(size=1, location=P(s * 0.955, 0.80, 1.075))
    m = bpy.context.active_object; m.name = f"Mirror{side}"
    m.scale = (0.16, 0.11, 0.11)
    m.data.materials.append(M["paint"])
    bv = m.modifiers.new("bv", "BEVEL"); bv.width = 0.35; bv.segments = 4; bv.limit_method = "NONE"
    activate(m); bpy.ops.object.transform_apply(scale=True); bpy.ops.object.modifier_apply(modifier="bv")
    bpy.ops.object.shade_smooth()
    m.parent = body
    bpy.ops.mesh.primitive_cube_add(size=1, location=P(s * 0.885, 0.82, 1.05))
    st = bpy.context.active_object; st.name = f"MirrorStalk{side}"; st.scale = (0.08, 0.06, 0.03)
    st.data.materials.append(M["trim"]); st.parent = body
    decal_pts = [P(s * (0.955 + dx), 0.80 - 0.058, 1.075 + dz) for dx, dz in ((-0.07, -0.042), (0.07, -0.042), (0.07, 0.042), (-0.07, 0.042))]
    mesh_obj(f"MirrorGlass{side}", decal_pts, [(0, 1, 2, 3)], ["chrome"], parent=body, smooth=False)


# ------------------------------------------------------------------ wheels
def lathe(profile, seg=48, closed=True):
    verts, faces = [], []
    for i in range(seg):
        a = 2 * math.pi * i / seg
        for r, x in profile:
            verts.append((x, r * math.cos(a), r * math.sin(a)))
    n = len(profile)
    for i in range(seg):
        i2 = (i + 1) % seg
        for k in range(n if closed else n - 1):
            k2 = (k + 1) % n
            faces.append((i * n + k, i * n + k2, i2 * n + k2, i2 * n + k))
    return verts, faces


def wheel_parts():
    """right-hand wheel, outer face toward +x, hub at the origin: (spin verts, faces, mats), (brake ...)"""
    parts = []   # (verts, faces, material name)
    tyre = [(0.214, -0.10), (0.25, -0.111), (0.285, -0.113), (0.305, -0.105), (0.314, -0.09), (0.318, -0.055),
            (WHEEL_R + 0.001, 0), (0.318, 0.055), (0.314, 0.09), (0.305, 0.105), (0.285, 0.113), (0.25, 0.111), (0.214, 0.10)]
    parts.append((*lathe(tyre), "tyre"))
    lip = [(0.207, -0.098), (0.215, -0.10), (0.221, -0.093), (0.221, 0.092), (0.224, 0.10), (0.215, 0.105), (0.205, 0.096),
           (0.199, 0.078), (0.199, -0.078)]
    parts.append((*lathe(lip), "rim"))
    hub = [(0.0, 0.088), (0.03, 0.088), (0.045, 0.085), (0.065, 0.075), (0.07, 0.05), (0.06, 0.03), (0.0, 0.03)]
    parts.append((*lathe(hub, 24, closed=False), "rim"))
    # ten spokes in five pairs, dished: hub end further out than the rim end
    sv, sf = [], []
    for k in range(5):
        for off in (-0.11, 0.11):
            a = 2 * math.pi * k / 5 + off
            ca, sa = math.cos(a), math.sin(a)
            box = []
            for r, half, x0, x1 in ((0.055, 0.016, 0.052, 0.078), (0.205, 0.011, 0.040, 0.066)):
                for side in (-1, 1):
                    for x in (x0, x1):
                        # point at radius r along direction a, offset sideways by half
                        py, pz = r * ca - side * half * sa, r * sa + side * half * ca
                        box.append((x, py, pz))
            b = len(sv); sv += box
            # box order: [inner r: (s-1,x0),(s-1,x1),(s+1,x0),(s+1,x1)], [outer r: same]
            for f in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (1, 3, 7, 5), (0, 4, 6, 2)):
                sf.append(tuple(b + i for i in f))
    parts.append((sv, sf, "rim"))
    barrel = [(0.199, 0.066), (0.199, -0.096), (0.12, -0.096)]
    parts.append((*lathe(barrel, 32, closed=False), "disc"))
    brake = []
    disc = [(0.165, -0.04), (0.165, -0.015), (0.07, -0.015), (0.07, -0.04)]
    brake.append((*lathe(disc, 32), "disc"))
    cv, cf = [], []
    for a0 in (0.95,):
        pts = []
        for r in (0.13, 0.182):
            for a in (a0 - 0.32, a0 + 0.32):
                for x in (-0.05, 0.0):
                    pts.append((x, r * math.cos(math.pi - a), r * math.sin(a)))
        cv += pts
        for f in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (1, 3, 7, 5), (0, 4, 6, 2)):
            cf.append(f)
    brake.append((cv, cf, "caliper"))
    return parts, brake


def build_group(name, parts, parent, mirror):
    verts, faces, fm, names = [], [], [], []
    for v, f, m in parts:
        if m not in names:
            names.append(m)
        b = len(verts)
        verts += [Vector((x * (-1 if mirror else 1), y, z)) for x, y, z in v]
        faces += [tuple(b + i for i in (ff[::-1] if mirror else ff)) for ff in f]
        fm += [names.index(m)] * len(f)
    ob = mesh_obj(name, verts, faces, names, fm, parent=parent)
    recalc_normals(ob)
    activate(ob)
    try:
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(35))
    except Exception:
        pass
    return ob


spin_parts, brake_parts = wheel_parts()
for nm, L, s in (("Wheel_FL", AXLE_F, -1), ("Wheel_FR", AXLE_F, 1), ("Wheel_RL", AXLE_R, -1), ("Wheel_RR", AXLE_R, 1)):
    w = empty(nm, P(s * TRACK, L, WHEEL_R), parent=car)
    # the caliper sits behind the hub on front wheels, ahead of it on rears; keep it simple: same everywhere
    build_group(nm + "_Spin", spin_parts, w, mirror=s < 0)
    build_group(nm + "_Brake", brake_parts, w, mirror=s < 0)

# ------------------------------------------------------------------ export
bpy.ops.object.select_all(action="DESELECT")
bpy.ops.export_scene.gltf(filepath=OUT, export_format="GLB", export_yup=True, export_apply=True)
tris = sum(sum(len(p.vertices) - 2 for p in o.data.polygons) for o in scene.objects if o.type == "MESH")
print(f"CAR_EXPORT {OUT} triangles={tris}")

# ------------------------------------------------------------------ optional preview render
if PREVIEW:
    scene.render.engine = "CYCLES"
    scene.cycles.samples = 64
    scene.render.resolution_x, scene.render.resolution_y = 1400, 800
    world = bpy.data.worlds.new("sky"); scene.world = world; world.use_nodes = True
    nt = world.node_tree
    sky = nt.nodes.new("ShaderNodeTexSky")
    try:
        sky.sky_type = "NISHITA"
    except Exception:
        pass
    sky.sun_elevation = math.radians(35); sky.sun_rotation = math.radians(140)
    nt.links.new(sky.outputs[0], nt.nodes["Background"].inputs[0])
    nt.nodes["Background"].inputs[1].default_value = 0.12
    scene.view_settings.exposure = -0.6
    bpy.ops.mesh.primitive_plane_add(size=40)
    g = bpy.context.active_object; g.data.materials.append(mat("Ground", (0.05, 0.05, 0.05), rough=0.8))
    sun = bpy.data.objects.new("Sun", bpy.data.lights.new("Sun", "SUN")); scene.collection.objects.link(sun)
    sun.data.energy = 2.5; sun.rotation_euler = (math.radians(50), 0, math.radians(140))
    cams = [("front", P(3.6, 5.6, 1.7), P(0, 0.3, 0.7)), ("rear", P(-4.2, -5.0, 2.0), P(0, -0.3, 0.75)),
            ("side", P(7.5, 0.0, 1.0), P(0, 0.1, 0.7))]
    for tag, pos, look in cams:
        cam = bpy.data.objects.new("Cam", bpy.data.cameras.new("Cam")); scene.collection.objects.link(cam)
        cam.location = pos
        cam.rotation_euler = (look - pos).to_track_quat("-Z", "Y").to_euler()
        cam.data.lens = 45
        scene.camera = cam
        scene.render.filepath = PREVIEW.replace(".png", f"_{tag}.png")
        bpy.ops.render.render(write_still=True)
