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

KIND_WINDOW, KIND_PLAIN, STYLE_RENDER, STYLE_BRICK, STYLE_ASHLAR = 0, 1, 0, 4, 5
SASH_UV = (0.336, 0.664, 0.281, 0.797)      # the sash inside its texture cell (game/facades.js)
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
    def __init__(self, sink, foot_out, to_local, ground, shop_out=None):
        self.sink, self.foot_out, self.to_local, self.ground = sink, foot_out, to_local, ground
        self.shop_out = shop_out if shop_out is not None else []
        self.n_tris = 0

    def _tri(self, pts, uvs, col, want, style=STYLE_ASHLAR, kind=KIND_PLAIN):
        a, b, c = (np.array(p) for p in pts)
        if np.dot(np.cross(b - a, c - a), want) < 0:
            pts, uvs = [pts[0], pts[2], pts[1]], [uvs[0], uvs[2], uvs[1]]
        W = self.sink.walls[style]
        for p, uv in zip(pts, uvs):
            W["pos"] += [float(v) for v in p]
            W["uv"] += [float(v) for v in uv]
            W["col"] += [int(v) for v in col]
            W["kind"].append(kind)
        self.n_tris += 1

    def wall(self, p0, p1, thick, top, col, openings=(), name=None, grade=None, base=None, style=STYLE_ASHLAR,
             glaze=None, solid=True):
        """Slab from BNG point p0 to p1. top: height (level) or [(s, h)] profile in metres along the wall.
        openings: [opening(...)] polygons in (s, y). base: ground level override (m OD)."""
        (e0, n0), (e1, n1) = p0, p1
        L = math.hypot(e1 - e0, n1 - n0)
        dE, dN = (e1 - e0) / L, (n1 - n0) / L
        nE, nN = dN, -dE                                        # slab normal (BNG)
        if base is None:
            ss = np.linspace(0, L, max(2, int(L) + 1))
            base = float(np.min(self.ground(e0 + dE * ss, n0 + dN * ss)))
        self._base = base
        prof = [(0.0, top), (L, top)] if np.isscalar(top) else [(min(max(s, 0.0), L), h) for s, h in top]
        poly = Polygon([(0, -SUNK)] + [(s, h) for s, h in prof] + [(L, -SUNK)]).buffer(0)
        doors, glazed = [], []
        for o in openings:
            if glaze is not None and o.bounds[1] > 0.3:         # a sash set in the middle of the wall's thickness
                glazed.append(o.bounds)
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
                    self._tri([P(s, y, off) for s, y in c], [(s / 3.2, y / 2.56) for s, y in c], col, nloc * sgn, style)
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
                    self._tri([q[0], q[1], q[2]], [uv[0], uv[1], uv[2]], [int(v * 0.86) for v in col], out, style)
                    self._tri([q[0], q[2], q[3]], [uv[0], uv[2], uv[3]], [int(v * 0.86) for v in col], out, style)
        for (sa, ya, sb, yb) in glazed:
            q = [P(sa, ya, 0), P(sb, ya, 0), P(sb, yb, 0), P(sa, yb, 0)]
            u0, u1, v0, v1 = SASH_UV
            uv = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
            self._tri([q[0], q[1], q[2]], [uv[0], uv[1], uv[2]], glaze, nloc, STYLE_RENDER, KIND_WINDOW)
            self._tri([q[0], q[2], q[3]], [uv[0], uv[2], uv[3]], glaze, nloc, STYLE_RENDER, KIND_WINDOW)
        if not solid:
            return
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
                    rec["g"] = grade
                    rec["m"] = {0: "painted render", 4: "brick"}.get(style, "stone")
            self.foot_out.append(rec)

    def pier(self, centre, size, top, col, angle=0.0, name=None, grade=None, base=None):
        """Square-ish pier: size (along, across) metres, axis at `angle` radians from east."""
        ca, sa = math.cos(angle), math.sin(angle)
        h = size[0] / 2
        self.wall((centre[0] - ca * h, centre[1] - sa * h), (centre[0] + ca * h, centre[1] + sa * h),
                  size[1], top, col, name=name, grade=grade, base=base)

    def block(self, p0, p1, thick, y0, y1, col, base=None, style=STYLE_ASHLAR):
        """A slab that only exists between heights y0 and y1 (string course, quoin stone, lintel)."""
        L = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
        self.wall(p0, p1, thick, y1, col, [sbox(-1, -3, L + 1, y0)], base=base, style=style, solid=False)

    def _roof_quad(self, pts, col):
        """pts: four (E, N, y) corners in order; tiles run 2 m to the texture."""
        P = []
        for E, N, y in pts:
            x, z = self.to_local(E, N)
            P.append([float(x), float(y), float(z)])
        w = math.dist(P[0], P[1]) / 2.0
        h = math.dist(P[1], P[2]) / 2.0
        uv = [[0, 0], [w, 0], [w, h], [0, h]]
        for i, j, k in ((0, 1, 2), (0, 2, 3)):
            self.sink.roof_tri([P[i], P[j], P[k]], [uv[i], uv[j], uv[k]], [int(v) for v in col])
            self.n_tris += 1

    def house(self, a, b, depth, eave, ridge, col, roof_col, roof="parallel", style=STYLE_RENDER, side_col=None,
              side_style=None, windows=(), shops=(), shop_h=3.0, name=None, grade=None, quoins=None, barge=None,
              thick=0.35, over=0.25):
        """A building drawn from observation: front from BNG point a to b with the interior on the right of
        travel, `depth` back. roof: "parallel" (ridge along the street) or "gable_front" (gable to the street).
        eave / ridge: heights above the pavement (from LIDAR). windows: [opening(...)] on the front, in metres
        along the front and above the pavement. shops: [(s0, s1, name, paint hex, fascia hex)]."""
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        dE, dN = (b[0] - a[0]) / L, (b[1] - a[1]) / L
        iE, iN = dN, -dE                                         # inward
        at = lambda s_, d_: (a[0] + dE * s_ + iE * d_, a[1] + dN * s_ + iN * d_)
        ss = np.linspace(0, L, max(2, int(L) + 1))
        base = float(np.min(self.ground(a[0] + dE * ss, a[1] + dN * ss)))
        side_col, side_style = side_col or col, style if side_style is None else side_style
        gf = roof == "gable_front"
        peak_f = [(0, eave), (L / 2, ridge), (L, eave)] if gf else eave
        peak_s = eave if gf else [(0, eave), (depth / 2, ridge), (depth, eave)]
        h = thick / 2
        self.wall(at(0, h), at(L, h), thick, peak_f, col, list(windows), name=name, grade=grade, base=base, style=style,
                  glaze=(255, 255, 255))
        self.wall(at(0, depth - h), at(L, depth - h), thick, peak_f, side_col, base=base, style=side_style)
        self.wall(at(h, 0), at(h, depth), thick, peak_s, side_col, base=base, style=side_style)
        self.wall(at(L - h, 0), at(L - h, depth), thick, peak_s, side_col, base=base, style=side_style)
        ye, yr, o = base + eave, base + ridge, over
        drop = o * (ridge - eave) / ((L if gf else depth) / 2)   # eaves overhang carries the slope on down
        if gf:
            for s0, s1 in ((-o, L / 2), (L + o, L / 2)):
                self._roof_quad([(*at(s0, -o), ye - drop), (*at(s0, depth + o), ye - drop), (*at(s1, depth + o), yr), (*at(s1, -o), yr)], roof_col)
        else:
            for d0, d1 in ((-o, depth / 2), (depth + o, depth / 2)):
                self._roof_quad([(*at(-o, d0), ye - drop), (*at(L + o, d0), ye - drop), (*at(L + o, d1), yr), (*at(-o, d1), yr)], roof_col)
        if quoins:                                               # alternating long and short corner stones
            y, k = shop_h if shops else 0.0, 0
            while y < eave - 0.05:
                w = 0.52 if k % 2 == 0 else 0.32
                y1 = min(eave, y + 0.33)
                self.block(at(0, h), at(w, h), thick + 0.09, y, y1, quoins, base=base)
                self.block(at(L - w, h), at(L, h), thick + 0.09, y, y1, quoins, base=base)
                y, k = y1, k + 1
        if barge and gf:                                         # bargeboards following the gable
            under = Polygon([(-0.4, -3), (L + 0.4, -3), (L + 0.4, eave - 0.30 - 0.4 * (ridge - eave) / (L / 2)),
                             (L / 2, ridge - 0.30), (-0.4, eave - 0.30 - 0.4 * (ridge - eave) / (L / 2))])
            self.wall(at(-0.3, h - 0.12), at(L + 0.3, h - 0.12), thick + 0.24,
                      [(0, eave - 0.3 * (ridge - eave) / (L / 2) + 0.06), (L / 2 + 0.3, ridge + 0.06), (L + 0.6, eave - 0.3 * (ridge - eave) / (L / 2) + 0.06)],
                      barge, [under], base=base, solid=False)
        for s0, s1, nm, paint, fascia in shops:
            ax, az = self.to_local(*at(s0, 0))
            bx, bz = self.to_local(*at(s1, 0))
            u = {"a": [round(ax, 2), round(az, 2)], "b": [round(bx, 2), round(bz, 2)], "n": [round(-dN, 4), round(-dE, 4)],
                 "h": shop_h, "st": "trad", "c": "#" + paint, "sc": "#" + (fascia or paint), "d": "c", "y": round(base, 2)}
            if nm:
                u["name"] = nm
            self.shop_out.append(u)
