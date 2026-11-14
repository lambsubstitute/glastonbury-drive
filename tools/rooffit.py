"""Clean roofs fitted to LIDAR.

The pixel-by-pixel facet segmentation (roofseg.py) follows every bump of the 1 m surface model, so a plain
pitched roof comes out as several slightly different planes with staircase edges and little vertical steps
between them. Here a roof is instead one of a few simple shapes whose parameters are fitted to the LIDAR:

  template(...)   rectangle-like footprints: flat, mono-pitch, gable or hip, ridge along either axis,
                  ridge position, height and pitch fitted by robust least squares; the shape with the
                  lowest error (plus a small penalty per extra face) wins. Faces meet exactly on straight
                  ridges and hips.
  regularise(...) any footprint: the facets roofseg found, with their slope directions snapped to the
                  building's own axes, near-identical planes merged, and each facet cut by the exact
                  intersection lines with its neighbours (ridges, hips, valleys) instead of pixel edges.

Both return planes as (a, b, c) with z = a * (E - ox) + b * (N - oy) + c (z above the building base) and
facets as [(plane index, polygon in BNG)] covering the roof area.
"""
import math
from collections import Counter
import numpy as np
import shapely
from shapely.geometry import Polygon, box
from shapely import affinity

REJECT = Counter()      # why regularise() gave up, for the build log
MIN_PITCH, MAX_PITCH = 0.18, 1.6        # tan: ~10 deg .. ~58 deg
FLAT_TOL = 0.12


def _frame(poly):
    """centre, unit long axis u, unit short axis v, half lengths (A along u, B along v) of the minimum
    rotated rectangle"""
    mrr = poly.minimum_rotated_rectangle
    co = np.array(mrr.exterior.coords)[:4]
    e0, e1 = co[1] - co[0], co[2] - co[1]
    if np.hypot(*e0) < np.hypot(*e1):
        e0, e1 = e1, e0
    u = e0 / (np.hypot(*e0) or 1)
    v = np.array([-u[1], u[0]])
    C = co.mean(0)
    return C, u, v, np.hypot(*e0) / 2, np.hypot(*e1) / 2, mrr


def _robust(G, z, iters=3):
    """least squares z ~ G @ p, refitted on the best 80 % of points"""
    keep = np.ones(len(z), bool)
    p = None
    for _ in range(iters):
        if keep.sum() < G.shape[1] + 2:
            return None, np.inf, 0
        p, *_ = np.linalg.lstsq(G[keep], z[keep], rcond=None)
        r = np.abs(G @ p - z)
        keep = r <= np.percentile(r, 80) + 0.05
    r = np.abs(G @ p - z)
    trim = np.sort(r)[: max(1, int(0.85 * len(r)))]
    return p, float(np.sqrt(np.mean(trim ** 2))), float(np.mean(r < 0.5))


def template(poly, roof_area, E, N, Z, hint=None):
    """fit a simple roof to a rectangle-like footprint. E, N: pixel centres (BNG), Z: heights above base.
    hint: OSM roof:shape if tagged. Returns dict or None when nothing fits well."""
    C, u, v, A, B, mrr = _frame(poly)
    if poly.area / mrr.area < 0.82 or len(Z) < 6 or B < 1.2:
        return None
    s = (E - C[0]) * u[0] + (N - C[1]) * u[1]
    t = (E - C[0]) * v[0] + (N - C[1]) * v[1]
    one = np.ones(len(Z))
    cands = []
    # flat
    p, err, frac = _robust(one[:, None], Z)
    if p is not None:
        cands.append((err + 0.0, "flat", None, p, frac))
    for axis in ("u", "v"):
        # ridge along `axis`; q = coordinate across the ridge, w = along it; Hq/Hw half extents
        q, w, Hq, Hw = (t, s, B, A) if axis == "u" else (s, t, A, B)
        # mono-pitch
        p, err, frac = _robust(np.c_[one, q], Z)
        if p is not None and MIN_PITCH <= abs(p[1]) <= MAX_PITCH:
            cands.append((err + 0.04, "shed", (axis,), p, frac))
        # gable, ridge offset q0 searched over a few positions
        for q0 in np.linspace(-0.25 * Hq, 0.25 * Hq, 7):
            G = np.c_[one, -np.abs(q - q0)]
            p, err, frac = _robust(G, Z)
            if p is not None and MIN_PITCH <= p[1] <= MAX_PITCH:
                cands.append((err + 0.05 + 0.02 * abs(q0) / Hq, "gable", (axis, q0), p, frac))
        # hip: equal pitch on all faces, ridge centred, half-length Hw - Hq (pyramid when <= 0)
        L = max(Hw - Hq, 0.0)
        G = np.c_[one, -np.maximum(np.abs(q), np.abs(w) - L)]
        p, err, frac = _robust(G, Z)
        if p is not None and MIN_PITCH <= p[1] <= MAX_PITCH:
            cands.append((err + 0.07, "hip", (axis, L), p, frac))
    if not cands:
        return None
    if hint in ("flat",):
        cands = [c for c in cands if c[1] == "flat"] or cands
    elif hint in ("gabled", "hipped", "skillion"):
        want = {"gabled": "gable", "hipped": "hip", "skillion": "shed"}[hint]
        pref = [(c[0] - 0.08, *c[1:]) if c[1] == want else c for c in cands]
        cands = pref
    cands.sort(key=lambda c: c[0])
    err, kind, prm, p, frac = cands[0]
    if err > 0.5 or frac < 0.62:
        return None
    flat_err = next((c[0] for c in cands if c[1] == "flat"), np.inf)
    if kind != "flat" and flat_err < err + 0.03:
        kind, prm, p = "flat", None, next(c[3] for c in cands if c[1] == "flat")
    return _build(kind, prm, p, C, u, v, A, B, roof_area, err)


def _halfplane(C, n, d, R):
    """polygon of points P with (P - C) . n >= d, big enough to cover radius R"""
    m = np.array([-n[1], n[0]])
    p0 = C + n * d
    pts = [p0 - m * R, p0 + m * R, p0 + m * R + n * 2 * R, p0 - m * R + n * 2 * R]
    return Polygon(pts)


def _build(kind, prm, p, C, u, v, A, B, roof_area, err):
    R = (A + B) * 3 + 20
    planes, facets = [], []
    if kind == "flat":
        planes = [(0.0, 0.0, float(p[0]))]
        facets = [(0, roof_area)]
    elif kind == "shed":
        axis, = prm
        n = v if axis == "u" else u
        planes = [(p[1] * n[0], p[1] * n[1], float(p[0]))]
        facets = [(0, roof_area)]
    elif kind == "gable":
        axis, q0 = prm
        n = v if axis == "u" else u
        h, k = float(p[0]), float(p[1])
        # z = h - k |q - q0|: side +n slopes down along +n
        planes = [(-k * n[0], -k * n[1], h + k * q0), (k * n[0], k * n[1], h - k * q0)]
        facets = [(0, roof_area.intersection(_halfplane(C, n, q0, R))),
                  (1, roof_area.intersection(_halfplane(C, -n, -q0, R)))]
    elif kind == "hip":
        axis, L = prm
        n, m = (v, u) if axis == "u" else (u, v)
        h, k = float(p[0]), float(p[1])
        # four faces: z = h - k * (q) for side +n, h - k * (-q), h - k * (w - L) for end +m, h - k * (-w - L)
        planes = [(-k * n[0], -k * n[1], h), (k * n[0], k * n[1], h),
                  (-k * m[0], -k * m[1], h + k * L), (k * m[0], k * m[1], h + k * L)]
        # face regions: the face whose plane is lowest at a point covers it (convex roof)
        dirs = [(n, 0.0), (-n, 0.0), (m, L), (-m, L)]
        for i, (di, Li) in enumerate(dirs):
            reg = roof_area
            for j, (dj, Lj) in enumerate(dirs):
                if i == j:
                    continue
                # face i is lower than face j where (P.di - Li) >= (P.dj - Lj)  ->  P.(di - dj) >= Li - Lj
                dd = np.array(di) - np.array(dj)
                nn = np.hypot(*dd)
                if nn < 1e-9:
                    continue
                reg = reg.intersection(_halfplane(C, dd / nn, (Li - Lj) / nn, R))
                if reg.is_empty:
                    break
            facets.append((i, reg))
    facets = [(k_, g_) for k_, g_ in facets if not g_.is_empty and g_.area > 0.05]
    return {"kind": kind, "origin": (float(C[0]), float(C[1])), "planes": np.array(planes, float),
            "facets": facets, "err": err}


# --------------------------------------------------------------------------------------------- regularise
def regularise(poly, roof_area, E, N, Z, planes_px, labels, to_bng_plane):
    """tidy a roofseg result. planes_px: roofseg planes; labels: facet per pixel (aligned with E, N, Z);
    to_bng_plane(a, b, c, ox, oy) -> (a, b, c) about origin (ox, oy). Returns dict or None."""
    if len(planes_px) < 2:
        # one face over an odd footprint: flat or a single pitch, snapped like any other
        if len(planes_px) == 0:
            REJECT["no plane"] += 1
            return None
        C, u, v, A, B, _ = _frame(poly)
        ox, oy = float(C[0]), float(C[1])
        a, b, c = to_bng_plane(*planes_px[0], ox, oy)
        if math.hypot(a, b) < FLAT_TOL:
            a = b = 0.0
        c = float(np.median(Z - a * (E - ox) - b * (N - oy)))
        return {"kind": "regular", "origin": (ox, oy), "planes": np.array([(a, b, c)]),
                "facets": [(0, roof_area)], "err": 0.0}
    C, u, v, A, B, _ = _frame(poly)
    # candidate slope directions: the normals of the footprint's own walls (>= 3 m), so each range of a
    # block along a curving street keeps its own axis
    axes = [u, -u, v, -v]
    co = np.array(poly.exterior.coords)
    for p0, p1 in zip(co[:-1], co[1:]):
        d = p1 - p0
        L_ = np.hypot(*d)
        if L_ >= 3.0:
            nrm = np.array([-d[1], d[0]]) / L_
            axes += [nrm, -nrm]
    ox, oy = float(C[0]), float(C[1])
    X, Y = E - ox, N - oy
    P = [to_bng_plane(*pl, ox, oy) for pl in planes_px]
    K = len(P)
    labels = np.asarray(labels).copy()
    # 0. tiny facets (< 8 px) go to whichever other plane fits their pixels best
    cnt = np.bincount(labels[labels >= 0], minlength=K)
    big = [k for k in range(K) if cnt[k] >= 8]
    if not big:
        REJECT["no big facet"] += 1
        return None
    for k in range(K):
        if cnt[k] and cnt[k] < 8:
            m = labels == k
            res = [np.median(np.abs(P[j][0] * X[m] + P[j][1] * Y[m] + P[j][2] - Z[m])) for j in big]
            labels[m] = big[int(np.argmin(res))]
    # 1. snap: a sloping facet's uphill direction goes to the nearest wall normal (within 22 deg); every
    #    pitched facet of the building takes one shared pitch (most roofs in a range are built to one pitch)
    slopes = []
    for k in big:
        sl = math.hypot(P[k][0], P[k][1])
        if sl >= FLAT_TOL:
            slopes += [sl] * int((labels == k).sum())
    pitch = float(np.median(slopes)) if slopes else None
    snapped = {}
    for k in big:
        a, b, c = P[k]
        m = labels == k
        sl = math.hypot(a, b)
        if sl < FLAT_TOL:
            snapped[k] = (0.0, 0.0, float(np.median(Z[m])))
            continue
        g = np.array([a, b]) / sl
        best = max(axes, key=lambda ax: g @ ax)
        if g @ best > math.cos(math.radians(22)):
            g = best
        if pitch is not None and abs(sl - pitch) < 0.35 * pitch:
            sl = pitch
        a, b = g * sl
        snapped[k] = (float(a), float(b), float(np.median(Z[m] - a * X[m] - b * Y[m])))
    # 2. merge facets sharing a plane (same slope vector, offset within 0.5 m)
    parent = {k: k for k in big}

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for ii, i in enumerate(big):
        for j in big[ii + 1:]:
            pi, pj = snapped[i], snapped[j]
            if abs(pi[0] - pj[0]) + abs(pi[1] - pj[1]) < 0.1 and abs(pi[2] - pj[2]) < 0.5:
                parent[find(j)] = find(i)
    roots = sorted({find(i) for i in big})
    rid = {r: n for n, r in enumerate(roots)}
    lab = np.array([rid[find(int(l))] if l >= 0 and int(l) in parent else -1 for l in labels])
    planes = []
    for n, r in enumerate(roots):
        m = lab == n
        a, b, _ = snapped[r]
        planes.append((a, b, float(np.median(Z[m] - a * X[m] - b * Y[m]))) if m.sum() >= 3 else snapped[r])
    planes = np.array(planes)
    K = len(planes)
    K = len(planes)
    # 3. face regions on a 0.5 m grid: each point takes the face (among those with pixels within 3 m) whose
    #    plane best matches the smoothed LIDAR there, so borders fall where the planes really cross
    from scipy.spatial import cKDTree
    from scipy import ndimage
    cnt = np.bincount(lab[lab >= 0], minlength=K)
    live = [k for k in range(K) if cnt[k] >= 6]
    if not live:
        REJECT["no live"] += 1
        return None
    tree = cKDTree(np.c_[E, N])
    # LIDAR smoothed over its 3 x 3 neighbourhood (median), at the pixels
    nb = tree.query_ball_point(np.c_[E, N], 1.5)
    Zs = np.array([np.median(Z[q]) for q in nb])
    x0, y0, x1, y1 = roof_area.bounds
    G = 0.5
    gx, gy = np.meshgrid(np.arange(x0 + G / 2, x1, G), np.arange(y1 - G / 2, y0, -G))
    inside = shapely.contains_xy(roof_area.buffer(G), gx, gy)
    px, py = gx[inside], gy[inside]
    d_, nn = tree.query(np.c_[px, py])
    zref = Zs[nn]
    score = np.full((len(px), len(live)), np.inf)
    for n_, k in enumerate(live):
        m = lab == k
        dk, _ = cKDTree(np.c_[E[m], N[m]]).query(np.c_[px, py], distance_upper_bound=3.0)
        z = planes[k][0] * (px - ox) + planes[k][1] * (py - oy) + planes[k][2]
        sc = np.abs(z - zref) + 0.15 * np.minimum(dk, 3.0)
        sc[~np.isfinite(dk)] = np.inf
        score[:, n_] = sc
    best = np.argmin(score, axis=1)
    best[~np.isfinite(score.min(1))] = -1
    img = np.full(gx.shape, -1, np.int32)
    img[inside] = best
    # majority clean-up (twice), then any unassigned grid point takes its nearest assigned neighbour
    for _ in range(2):
        votes = np.stack([ndimage.uniform_filter((img == n_).astype(np.float32), 3) for n_ in range(len(live))])
        img = np.where(inside, np.argmax(votes, axis=0), -1).astype(np.int32)
    # 4. polygons: one coverage of grid squares, simplified together so shared borders stay shared
    polys, ks = [], []
    for n_ in range(len(live)):
        m = img == n_
        if m.sum() < 8:
            continue
        sq = shapely.box(gx[m] - G / 2, gy[m] - G / 2, gx[m] + G / 2, gy[m] + G / 2)
        polys.append(shapely.union_all(sq))
        ks.append(live[n_])
    if not polys:
        REJECT["empty"] += 1
        return None
    cover = list(shapely.coverage_simplify(np.array(polys, dtype=object), 0.8))
    cover = [shapely.make_valid(g_) for g_ in cover]
    # 5. vertices shared by faces meeting at a ridge / hip / valley move onto the planes' intersection, so
    #    both faces have the same height there and no step wall is needed
    zp = lambda k, x, y: planes[k][0] * (x - ox) + planes[k][1] * (y - oy) + planes[k][2]
    owners = {}
    for idx_, g_ in enumerate(cover):
        for xy in shapely.get_coordinates(g_):
            owners.setdefault((round(xy[0], 3), round(xy[1], 3)), set()).add(idx_)
    moved = {}
    for key_, own in owners.items():
        if len(own) < 2:
            continue
        kk = [ks[i_] for i_ in own]
        x, y = key_
        hs = [zp(k, x, y) for k in kk]
        if max(hs) - min(hs) > 1.5:
            continue                                   # a real step (taller range): leave it
        A_, B_ = [], []
        for k in kk[1:]:
            A_.append([planes[kk[0]][0] - planes[k][0], planes[kk[0]][1] - planes[k][1]])
            B_.append(-(planes[kk[0]][2] - planes[k][2]))
        A_, B_ = np.array(A_), np.array(B_)
        # minimum-norm move from (x, y) onto the intersection: A (p + dp) = B  ->  dp = pinv(A) (B - A p)
        p_ = np.array([x - ox, y - oy])
        dp = np.linalg.pinv(A_) @ (B_ - A_ @ p_)
        if np.hypot(*dp) > 1.5:
            continue
        moved[key_] = (x + dp[0], y + dp[1])
    if moved:
        def _mv(arr):
            out_ = arr.copy()
            for i_ in range(len(arr)):
                m_ = moved.get((round(arr[i_, 0], 3), round(arr[i_, 1], 3)))
                if m_ is not None:
                    out_[i_] = m_
            return out_
        cover = [shapely.make_valid(shapely.transform(g_, _mv)) for g_ in cover]
    out = []
    for k, g_ in zip(ks, cover):
        g2 = g_.intersection(roof_area)
        g2 = shapely.union_all([q for q in shapely.get_parts(g2) if q.geom_type == "Polygon" and q.area > 0.05]) \
            if not g2.is_empty else g2
        if not g2.is_empty:
            out.append((k, g2))
    if not out:
        REJECT["empty"] += 1
        return None
    # fit check: the clean roof must still match the LIDAR
    zf = np.full(len(Z), np.nan)
    pts = shapely.points(E, N)
    for k, g in out:
        shapely.prepare(g)
        m = shapely.contains(g, pts) & np.isnan(zf)
        zf[m] = planes[k][0] * X[m] + planes[k][1] * Y[m] + planes[k][2]
    ok = ~np.isnan(zf)
    if ok.mean() < 0.9:
        REJECT["coverage"] += 1
        return None
    r = np.abs(zf[ok] - Z[ok])
    err = float(np.sqrt(np.mean(np.sort(r)[: max(1, int(0.85 * len(r)))] ** 2)))
    if err > 1.0 or np.mean(r < 0.8) < 0.55:
        REJECT["fit %.1f" % min(err, 2.0)] += 1
        return None
    # renumber planes to the ones used
    used = sorted({k for k, _ in out})
    remap = {k: n for n, k in enumerate(used)}
    # a face never rises above the highest LIDAR return on its own pixels: stops planes extended over
    # neighbouring ground from climbing into tall fins
    zmax = np.array([float(np.percentile(Z[lab == k], 98)) + 0.3 if (lab == k).any() else float(Z.max()) for k in used])
    return {"kind": "regular", "origin": (ox, oy), "planes": planes[used], "zmax": zmax,
            "facets": [(remap[k], g) for k, g in out], "err": err}
