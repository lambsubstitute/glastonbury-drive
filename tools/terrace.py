"""Split block outlines into roof units before building them.

Town-centre blocks are often mapped as one outline covering a whole terrace plus rear wings and later
infill. One roof fitted over all of that is either too coarse or a patchwork. Here the LIDAR facets of the
block (roofseg) are grouped into roof units:

  joined   faces meeting at a ridge or hip at the same height (two slopes of one gable, the ends of a hip,
           one continuous range along the street)
  split    faces meeting at a valley (a rear wing against the main range), a height step over 0.7 m (houses
           of different heights), or a flat roof against a pitched one

and the outline is cut along the borders between units, so each house / range / wing is built (walls,
storeys, roof template) on its own. Parts under 20 m2 go back to the neighbour they share most wall with.
"""
import math
import numpy as np
import shapely
from shapely.geometry import Polygon
from scipy import ndimage
from scipy.spatial import cKDTree

import roofseg

FLAT = 0.12
MIN_PART = 20.0


def split(poly, rows, cols, Ex, Ny, Habs):
    """poly: block outline (BNG). rows/cols/Ex/Ny: its 1 m DSM pixels; Habs: DSM height there.
    Returns a list of polygons covering poly (just [poly] when it is one unit)."""
    if poly.area < 90 or len(Habs) < 60:
        return [poly]
    base = float(np.percentile(Habs, 2))
    seg = roofseg.segment(rows, cols, Habs - base)
    if seg is None:
        return [poly]
    labels, P, (r0, c0), _ = seg
    K = len(P)
    if K < 2:
        return [poly]
    # plane in pixel-local coords: x = col - c0, y = -(row - r0)
    X = (cols - c0).astype(float)
    Y = -(rows - r0).astype(float)
    cnt = np.bincount(labels[labels >= 0], minlength=K)
    zat = lambda k, x, y: P[k, 0] * x + P[k, 1] * y + P[k, 2]
    hgt, wid = rows.max() - r0 + 1, cols.max() - c0 + 1
    L = np.full((hgt, wid), -1, np.int32)
    L[rows - r0, cols - c0] = labels
    # adjacent facet pairs and where they touch
    touch = {}
    for dr, dc in ((0, 1), (1, 0)):
        A0, A1 = L[:hgt - dr, :wid - dc], L[dr:, dc:]
        m = (A0 >= 0) & (A1 >= 0) & (A0 != A1)
        rr, cc = np.nonzero(m)
        for r_, c_, i, j in zip(rr, cc, A0[m], A1[m]):
            touch.setdefault((min(i, j), max(i, j)), []).append((c_ + 0.5 * dc, -(r_ + 0.5 * dr)))
    parent = list(range(K))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for (i, j), pts in touch.items():
        if len(pts) < 3:
            continue
        pts = np.array(pts)
        gap = np.median(np.abs(zat(i, pts[:, 0], pts[:, 1]) - zat(j, pts[:, 0], pts[:, 1])))
        si, sj = math.hypot(P[i, 0], P[i, 1]), math.hypot(P[j, 0], P[j, 1])
        if gap > 0.7:
            continue                                  # height step: different houses / ranges
        if (si < FLAT) != (sj < FLAT):
            continue                                  # flat against pitched: an extension, not the same roof
        if si < FLAT and sj < FLAT:
            parent[find(j)] = find(i)                 # flat roofs at one level: one unit
            continue
        # ridge / hip (convex) joins; valley (concave) splits. Convex: each face is the lower one on its own
        # side, i.e. moving from the shared edge into face i, z_i - z_j drops.
        mi = labels == i
        mj = labels == j
        ci = (X[mi].mean(), Y[mi].mean())
        cj = (X[mj].mean(), Y[mj].mean())
        di = zat(i, *ci) - zat(j, *ci)
        dj = zat(i, *cj) - zat(j, *cj)
        if di < 0 < dj:
            parent[find(j)] = find(i)
    roots = sorted({find(k) for k in range(K)})
    if len(roots) < 2:
        return [poly]
    unit = np.array([roots.index(find(int(l))) if l >= 0 else -1 for l in labels])
    U = len(roots)
    # every point of the outline goes to the unit of its nearest labelled pixel (0.5 m grid), then the unit
    # regions are straightened as one coverage and cut from the outline
    ok = unit >= 0
    tree = cKDTree(np.c_[Ex[ok], Ny[ok]])
    uok = unit[ok]
    x0, y0, x1, y1 = poly.bounds
    G = 0.5
    gx, gy = np.meshgrid(np.arange(x0 - 1 + G / 2, x1 + 1, G), np.arange(y1 + 1 - G / 2, y0 - 1, -G))
    _, nn = tree.query(np.c_[gx.ravel(), gy.ravel()])
    img = uok[nn].reshape(gx.shape)
    for _ in range(3):                                 # majority filter: no ragged one-cell teeth
        votes = np.stack([ndimage.uniform_filter((img == u).astype(np.float32), 5) for u in range(U)])
        img = np.argmax(votes, axis=0)
    # unit areas on the grid (inside the outline only)
    inside = shapely.contains_xy(poly, gx, gy)
    img = np.where(inside, img, -1)
    live = [u for u in range(U) if (img == u).sum() * G * G >= MIN_PART * 0.5]
    if len(live) < 2:
        return [poly]
    # wall directions of the block (edges >= 3 m): party walls run along or across them
    co = np.array(poly.exterior.coords)
    dirs = []
    for p0, p1 in zip(co[:-1], co[1:]):
        d = p1 - p0
        if np.hypot(*d) >= 3.0:
            d = d / np.hypot(*d)
            dirs += [d, np.array([-d[1], d[0]])]
    # straight cut between every pair of touching units: a line fitted to their shared border cells,
    # turned onto the nearest wall direction (within 20 deg)
    cut = {}
    for dr, dc in ((0, 1), (1, 0)):
        A0, A1 = img[:img.shape[0] - dr, :img.shape[1] - dc], img[dr:, dc:]
        m = (A0 >= 0) & (A1 >= 0) & (A0 != A1)
        rr, cc = np.nonzero(m)
        for r_, c_, a_, b_ in zip(rr, cc, A0[m], A1[m]):
            if a_ in live and b_ in live:
                x_ = (gx[r_, c_] + gx[r_ + dr, c_ + dc]) / 2
                y_ = (gy[r_, c_] + gy[r_ + dr, c_ + dc]) / 2
                cut.setdefault((min(a_, b_), max(a_, b_)), []).append((x_, y_))
    R = math.hypot(x1 - x0, y1 - y0) + 10
    regs = {}
    for u in live:
        m = img == u
        sq = shapely.box(gx[m] - G / 2, gy[m] - G / 2, gx[m] + G / 2, gy[m] + G / 2)
        regs[u] = shapely.union_all(sq).buffer(2.5, join_style="mitre").intersection(poly)
    cen = {u: np.array([gx[img == u].mean(), gy[img == u].mean()]) for u in live}
    for (a_, b_), pts in cut.items():
        pts = np.array(pts)
        if len(pts) < 4:
            continue
        c = pts.mean(0)
        if len(pts) >= 2 and np.ptp(pts, axis=0).max() > 0.3:
            _, _, vt = np.linalg.svd(pts - c)
            d = vt[0]
        else:
            d = np.array([1.0, 0.0])
        if dirs:
            best = max(dirs, key=lambda q: abs(q @ d))
            if abs(best @ d) > math.cos(math.radians(20)):
                d = best
        n = np.array([-d[1], d[0]])
        if (cen[a_] - c) @ n < 0:
            n = -n                                     # n points into unit a
        for u, sg in ((a_, 1), (b_, -1)):
            nn = n * sg
            hp = shapely.Polygon([c - d * R, c + d * R, c + d * R + nn * R, c - d * R + nn * R])
            regs[u] = regs[u].intersection(hp)
    # overlaps go to the larger unit; gaps are filled by the nearest unit further down
    parts, taken = [], None
    for u in sorted(live, key=lambda u: -(img == u).sum()):
        g = regs[u] if taken is None else regs[u].difference(taken)
        g = shapely.make_valid(g)
        ps = [p for p in shapely.get_parts(g) if p.geom_type == "Polygon" and p.area > 0.5]
        if not ps:
            continue
        g = shapely.union_all(ps)
        taken = g if taken is None else shapely.union_all([taken, g])
        parts += [p for p in shapely.get_parts(g) if p.geom_type == "Polygon"]
    if len(parts) < 2:
        return [poly]
    # small parts merge into the neighbour they share most boundary with
    parts.sort(key=lambda p: -p.area)
    big = [p for p in parts if p.area >= MIN_PART]
    if len(big) < 2:
        return [poly]
    for p in parts:
        if p.area >= MIN_PART:
            continue
        i = max(range(len(big)), key=lambda k: big[k].buffer(0.05).intersection(p.buffer(0.05)).area)
        if big[i].buffer(0.05).intersects(p):
            m_ = shapely.union_all([big[i], p.buffer(0.01)]).buffer(-0.01)
            if m_.geom_type == "Polygon":
                big[i] = m_
    # whatever the pieces miss (slivers lost to simplification) goes to its nearest piece
    rest = poly.difference(shapely.union_all(big))
    for p in shapely.get_parts(rest):
        if p.geom_type != "Polygon" or p.area < 0.01:
            continue
        i = min(range(len(big)), key=lambda k: big[k].distance(p))
        m_ = shapely.union_all([big[i], p.buffer(0.01)]).buffer(-0.01)
        if m_.geom_type == "Polygon":
            big[i] = m_
    big = [shapely.make_valid(b) for b in big]
    out = []
    for b in big:
        for p in shapely.get_parts(b):
            if p.geom_type != "Polygon" or p.area < 6:
                continue
            # drop pinholes and repeated points left by the cutting
            p = Polygon(p.exterior, [r for r in p.interiors if Polygon(r).area > 2.0]).simplify(0.05)
            if p.geom_type == "Polygon" and p.is_valid and p.area >= 6:
                out.append(p)
    return out if len(out) >= 2 else [poly]
