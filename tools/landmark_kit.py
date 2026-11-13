"""Primitives for hand-modelled landmarks (ruins, towers) that LIDAR roof fitting cannot describe.

A place puts its models in data/landmarks.py:

    EXCLUDE = [(tag test, shapely BNG polygon), ...]   outlines the generic builder must leave alone
    def build(K): ...                                   K is a Kit; call K.wall(...) etc.

Everything is a wall: a vertical slab between two BNG points with a thickness, a top profile (so tops can
be level, crenellated or broken) and openings with flat, round or pointed heads that are really cut through,
so you see sky and can drive through doorways. Slabs go into the ashlar wall mesh with a stone colour, and
their solid parts into footprints.json for collision, the minimap and labels."""
import math
import numpy as np
import shapely
from shapely.geometry import Polygon, box as sbox
from shapely.geometry.polygon import orient

KIND_PLAIN, STYLE_ASHLAR = 1, 5
SUNK = 0.6                       # how far slabs run below the lowest ground under them


def stepped(points, step=0.8, course=0.32, rough=0.0, seed=1):
    """Top profile [(s, h)] -> the same line broken into level courses (a ruined wall head)."""
    rng = np.random.default_rng(seed)
    s0, s1 = points[0][0], points[-1][0]
    xs = np.arange(s0, s1 + 1e-6, step)
    hs = np.interp(xs + step / 2, [p[0] for p in points], [p[1] for p in points])
    if rough:
        hs = hs + rng.uniform(-rough, rough, len(hs))
    hs = np.maximum(course, np.round(hs / course) * course)
    out = []
    for x, h in zip(xs, hs):
        out += [(float(x), float(h)), (float(min(x + step, s1)), float(h))]
    return out


def battlements(L, h, merlon=0.9, gap=0.7, up=0.9):
    """Level top at h with crenellations."""
    out, s, k = [], 0.0, 0
    while s < L - 1e-6:
        w = merlon if k % 2 == 0 else gap
        e = min(L, s + w)
        hh = h + up if k % 2 == 0 else h
        out += [(s, hh), (e, hh)]
        s, k = e, k + 1
    return out


def opening(s, w, y0, y1, head="pointed"):
    """Elevation polygon of an opening centred at s, w wide, from y0 to the top of its head at y1."""
    x0, x1 = s - w / 2, s + w / 2
    if head == "flat":
        return sbox(x0, y0, x1, y1)
    if head == "round":
        r = w / 2
        ys = y1 - r
        arc = [(s + r * math.cos(a), ys + r * math.sin(a)) for a in np.linspace(0, math.pi, 9)]
    else:                         # pointed: two arcs struck from the opposite jambs (equilateral arch)
        rise = min(w * 0.866, (y1 - y0) * 0.6)
        ys = y1 - rise
        right = [(x0 + w * math.cos(a), ys + rise / 0.866 * math.sin(a)) for a in np.linspace(0, math.pi / 3, 5)]
        left = [(x1 - w * math.cos(a), ys + rise / 0.866 * math.sin(a)) for a in np.linspace(math.pi / 3, 0, 5)]
        arc = right + left[1:]
    return Polygon([(x0, y0), (x1, y0)] + arc)


class Kit:
    def __init__(self, sink, foot_out, to_local, ground):
        self.sink, self.foot_out, self.to_local, self.ground = sink, foot_out, to_local, ground
        self.n_tris = 0

    def _tri(self, pts, uvs, col, want):
        a, b, c = (np.array(p) for p in pts)
        if np.dot(np.cross(b - a, c - a), want) < 0:
            pts, uvs = [pts[0], pts[2], pts[1]], [uvs[0], uvs[2], uvs[1]]
        W = self.sink.walls[STYLE_ASHLAR]
        for p, uv in zip(pts, uvs):
            W["pos"] += [float(v) for v in p]
            W["uv"] += [float(v) for v in uv]
            W["col"] += list(col)
            W["kind"].append(KIND_PLAIN)
        self.n_tris += 1

    def wall(self, p0, p1, thick, top, col, openings=(), name=None, grade=None, base=None):
        """Slab from BNG point p0 to p1. top: height (level) or [(s, h)] profile in metres along the wall.
        openings: [opening(...)] polygons in (s, y). base: ground level override (m OD)."""
        (e0, n0), (e1, n1) = p0, p1
        L = math.hypot(e1 - e0, n1 - n0)
        dE, dN = (e1 - e0) / L, (n1 - n0) / L
        nE, nN = dN, -dE                                        # slab normal (BNG)
        if base is None:
            ss = np.linspace(0, L, max(2, int(L) + 1))
            base = float(np.min(self.ground(e0 + dE * ss, n0 + dN * ss)))
        prof = [(0.0, top), (L, top)] if np.isscalar(top) else [(min(max(s, 0.0), L), h) for s, h in top]
        poly = Polygon([(0, -SUNK)] + [(s, h) for s, h in prof] + [(L, -SUNK)]).buffer(0)
        doors = []
        for o in openings:
            if o.bounds[1] <= 0.3:                              # reaches the ground: cut right through the footing
                o = shapely.union(o, sbox(o.bounds[0], -SUNK - 0.1, o.bounds[2], 0.4))
                doors.append((o.bounds[0], o.bounds[2]))
            poly = poly.difference(o)
        polys = [orient(g, 1) for g in (poly.geoms if hasattr(poly, "geoms") else [poly]) if g.area > 0.01]

        def P(s, y, off):                                       # (s along, y up, off across) -> local x, y, z
            x, z = self.to_local(e0 + dE * s + nE * off, n0 + dN * s + nN * off)
            return [x, base + y, z]
        lx0, lz0 = self.to_local(e0, n0)
        lx1, lz1 = self.to_local(e0 + nE, n0 + nN)
        nloc = np.array([lx1 - lx0, 0.0, lz1 - lz0])            # slab normal in local coords
        dloc = np.array(P(1, 0, 0)) - np.array(P(0, 0, 0))
        for g in polys:
            for t in shapely.constrained_delaunay_triangles(g).geoms:
                c = list(t.exterior.coords)[:3]
                for off, sgn in ((thick / 2, 1), (-thick / 2, -1)):
                    self._tri([P(s, y, off) for s, y in c], [(s / 3.2, y / 2.56) for s, y in c], col, nloc * sgn)
            for ring in [g.exterior] + list(g.interiors):
                rc = list(ring.coords)
                for (sa, ya), (sb, yb) in zip(rc[:-1], rc[1:]):
                    if max(ya, yb) <= -SUNK + 1e-6:
                        continue                                # underside, buried
                    ds, dy = sb - sa, yb - ya
                    out = dloc * dy + np.array([0, -ds, 0])     # right of travel = away from the stone
                    q = [P(sa, ya, -thick / 2), P(sb, yb, -thick / 2), P(sb, yb, thick / 2), P(sa, ya, thick / 2)]
                    w = math.hypot(ds, dy) / 3.2
                    uv = [(0, 0), (w, 0), (w, thick / 2.56), (0, thick / 2.56)]
                    self._tri([q[0], q[1], q[2]], [uv[0], uv[1], uv[2]], [int(v * 0.86) for v in col], out)
                    self._tri([q[0], q[2], q[3]], [uv[0], uv[2], uv[3]], [int(v * 0.86) for v in col], out)
        # collision / minimap: the solid stretches between doorways
        cuts, s = [], 0.0
        for a, b in sorted(doors):
            if a > s + 0.2:
                cuts.append((s, a))
            s = max(s, b)
        if s < L - 0.2:
            cuts.append((s, L))
        hmax = max(h for _, h in prof)
        for k, (a, b) in enumerate(cuts):
            ring = [P(a, 0, -thick / 2), P(b, 0, -thick / 2), P(b, 0, thick / 2), P(a, 0, thick / 2)]
            rec = {"p": [[round(q[0], 2), round(q[2], 2)] for q in ring + ring[:1]], "h": round(hmax, 1)}
            if name and k == 0:
                rec["n"] = name
                if grade:
                    rec["g"], rec["m"] = grade, "stone"
            self.foot_out.append(rec)

    def pier(self, centre, size, top, col, angle=0.0, name=None, grade=None, base=None):
        """Square-ish pier: size (along, across) metres, axis at `angle` radians from east."""
        ca, sa = math.cos(angle), math.sin(angle)
        h = size[0] / 2
        self.wall((centre[0] - ca * h, centre[1] - sa * h), (centre[0] + ca * h, centre[1] + sa * h),
                  size[1], top, col, name=name, grade=grade, base=base)
