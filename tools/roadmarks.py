"""Carriageway surface mesh and UK road markings, generated from the OSM road graph.

Called from process.py once the carriageway union (ROADS_U) and pavements exist.

  road_tris.bin   float32 (x, z, d) per vertex, 3 vertices per triangle, local coords. d = metres to the
                  nearest carriageway edge, so the game can lift the middle of the road (camber/crown).
                  One merged surface: junctions are a single shape, not overlapping strips.
  markings.bin    float32 (x, z, d, colour) per vertex, 3 per triangle; colour 0 white, 1 yellow.

Markings follow the Traffic Signs Regulations and General Directions (TSRGD) diagram sizes, simplified:
  centre line (1004)        100 mm, 4 m mark / 5 m gap; 6 m / 3 m on the last 25 m before a give-way
  lane line (1005)          100 mm, 2 m / 7 m on one-way roads with lanes >= 2
  edge line (1012.1)        150 mm solid on fast A/B roads outside built-up areas
  give way (1003)           two 200 mm broken lines, 600 mm mark / 300 mm gap, across the inbound half of
                            every minor arm of a junction (hierarchy from highway class, then through-way)
  stop line at signals      300 mm solid across the inbound half
  zebra (1048) + zig-zags   stripes across the full width, broken give-way line each side, 8 zig-zag marks
  signal crossing (1055)    stud lines + stop lines + zig-zags
  bus stop box (1025)       yellow outline, 13 m x 2.4 m (from the bus bays process.py places)
OSM gives the graph, crossings and signals; Mapillary zebra detections fill crossings OSM lacks.
"""
import math
import numpy as np
import shapely
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import substring

RANK = {"motorway": 8, "trunk": 7, "primary": 6, "secondary": 5, "tertiary": 4, "primary_link": 5,
        "secondary_link": 4, "tertiary_link": 3, "trunk_link": 6, "residential": 3, "unclassified": 3,
        "road": 3, "living_street": 2, "pedestrian": 2, "service": 1, "track": 0}
UNMARKED = {"service", "track", "pedestrian", "living_street"}
WHITE, YELLOW = 0, 1


def _parts(g):
    return [p for p in shapely.get_parts(g) if p.geom_type == "Polygon" and not p.is_empty]


def _rect(a, b, wid):
    """rectangle of width `wid` centred on segment a-b"""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    if L < 1e-3:
        return None
    nx, ny = -dy / L * wid / 2, dx / L * wid / 2
    return Polygon([(a[0] + nx, a[1] + ny), (b[0] + nx, b[1] + ny), (b[0] - nx, b[1] - ny), (a[0] - nx, a[1] - ny)])


def _edge_distance(geom):
    """f(points Nx2) -> distance to the boundary of `geom`, via a tree of boundary segments"""
    segs = []
    for ring in shapely.get_parts(shapely.boundary(geom)):
        c = shapely.get_coordinates(ring)
        if len(c) > 1:
            segs.append(np.stack([c[:-1], c[1:]], 1))
    segs = np.concatenate(segs)
    lines = shapely.linestrings(segs)
    tree = shapely.STRtree(lines)

    def f(pts):
        p = shapely.points(pts)
        _, d = tree.query_nearest(p, return_distance=True, all_matches=False)
        return d
    return f


def _triangulate(polys, seg=None):
    """polygons -> (T, 3, 2) triangle corner coordinates"""
    polys = [p for p in polys if p is not None and not p.is_empty]
    if not polys:
        return np.zeros((0, 3, 2))
    arr = np.array(polys, dtype=object)
    if seg:
        arr = shapely.segmentize(arr, seg)
    tris = shapely.get_parts(shapely.constrained_delaunay_triangles(arr))
    tris = tris[shapely.get_type_id(tris) == 3]
    co = shapely.get_coordinates(shapely.get_exterior_ring(tris)).reshape(-1, 4, 2)[:, :3]
    return co


def surface_mesh(surf, cell=3.0, tile=48.0):
    """triangulate the carriageway with a vertex at least every `cell` metres inside it (for camber and for
    following the terrain), cutting it first into `tile` m squares so each overlay is small"""
    x0, y0, x1, y1 = surf.bounds
    xs = np.arange(math.floor(x0 / tile) * tile, x1, tile)
    ys = np.arange(math.floor(y0 / tile) * tile, y1, tile)
    gx, gy = np.meshgrid(xs, ys)
    tiles = shapely.box(gx.ravel(), gy.ravel(), gx.ravel() + tile, gy.ravel() + tile)
    shapely.prepare(surf)
    tiles = tiles[shapely.intersects(surf, tiles)]
    k = int(round(tile / cell))
    out = []
    for tb in tiles:
        piece = shapely.intersection(tb, surf)
        if piece.is_empty:
            continue
        bx, by = tb.bounds[0], tb.bounds[1]
        cx, cy = np.meshgrid(bx + np.arange(k) * cell, by + np.arange(k) * cell)
        cells = shapely.box(cx.ravel(), cy.ravel(), cx.ravel() + cell, cy.ravel() + cell)
        shapely.prepare(piece)
        hit = cells[shapely.intersects(piece, cells)]
        inside = shapely.contains_properly(piece, hit)
        bits = list(hit[inside])
        for c_ in hit[~inside]:
            bits += _parts(shapely.intersection(c_, piece))
        out.append(_triangulate(bits))
    return np.concatenate(out) if out else np.zeros((0, 3, 2))


def build(*, osm, street_osm, mly_feats, furn, road_width, drive_w, proj, TF, to_local, E0, N0,
          roads_u, extra_tarmac, pave_u, all_b, tile, rural, out):
    import os

    # ------------------------------------------------------------ the road graph (OSM node ids)
    W = []
    bridge_surf = []
    for e in osm:
        t = e.get("tags", {})
        hw = t.get("highway")
        if e["type"] != "way" or hw not in drive_w or not e.get("geometry") or t.get("tunnel") not in (None, "no"):
            continue
        pts = proj(e["geometry"])
        if len(pts) < 2 or len(e.get("nodes", [])) != len(pts):
            continue
        w = road_width(t)
        line = LineString(pts)
        if t.get("bridge") not in (None, "no"):
            bridge_surf.append(line.buffer(w / 2, cap_style="flat"))
            continue
        ow = 1 if t.get("oneway") in ("yes", "true", "1") or t.get("junction") in ("roundabout", "circular") \
            else -1 if t.get("oneway") == "-1" else 0
        c = np.array(pts)
        cum = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(c, axis=0).T))])
        try:
            ms = float(str(t.get("maxspeed", "0")).split()[0])
        except ValueError:
            ms = 0
        W.append({"line": line, "nodes": e["nodes"], "cum": cum, "w": w, "hw": hw, "ow": ow, "name": t.get("name"),
                  "rb": t.get("junction") in ("roundabout", "circular"), "ms": ms, "lanes": t.get("lanes"), "id": e["id"]})

    inc = {}
    for wi, r in enumerate(W):
        for i, nid in enumerate(r["nodes"]):
            inc.setdefault(nid, []).append((wi, i))

    def arms_at(nid):
        A = []
        for wi, i in inc.get(nid, []):
            n = len(W[wi]["nodes"])
            if i > 0:
                A.append((wi, i, -1))
            if i < n - 1:
                A.append((wi, i, 1))
        return A

    def at(wi, s):
        """point and unit tangent (direction of increasing s) on way wi at distance s"""
        ln = W[wi]["line"]
        s = min(max(s, 0.0), ln.length)
        p = ln.interpolate(s)
        a, b = ln.interpolate(max(0, s - 0.6)), ln.interpolate(min(ln.length, s + 0.6))
        tx, ty = b.x - a.x, b.y - a.y
        tl = math.hypot(tx, ty) or 1
        return (p.x, p.y), (tx / tl, ty / tl)

    def arm_frame(wi, i, d, s):
        """point s metres out along an arm from its node; t points away from the node; n is the inbound half
        (traffic towards the node keeps left)"""
        c = W[wi]["cum"][i]
        P, (tx, ty) = at(wi, c + d * s)
        tx, ty = tx * d, ty * d
        return P, (tx, ty), (ty, -tx)

    def inbound(wi, d):
        """(o0, o1): offsets across an arm (along its n) that carry traffic towards the node, or None"""
        r = W[wi]
        if r["ow"] == 0:
            return (0.0, r["w"] / 2)
        return (-r["w"] / 2, r["w"] / 2) if r["ow"] == -d else None

    white, yellow = [], []

    def across(P, t, n, along, o0, o1, wid, dash=None, col=None):
        """line across the road at `along` m from P (in direction t), from offset o0 to o1 along n"""
        col = white if col is None else col
        ax, ay = P[0] + t[0] * along, P[1] + t[1] * along
        if dash is None:
            col.append(_rect((ax + n[0] * o0, ay + n[1] * o0), (ax + n[0] * o1, ay + n[1] * o1), wid))
            return
        mark, gap = dash
        o = o0 + 0.05
        while o < o1 - 0.05:
            e_ = min(o + mark, o1 - 0.05)
            col.append(_rect((ax + n[0] * o, ay + n[1] * o), (ax + n[0] * e_, ay + n[1] * e_), wid))
            o = e_ + gap

    def along_line(wi, s0, s1, off, wid, col=None):
        if s1 - s0 < 0.2:
            return
        seg = substring(W[wi]["line"], s0, s1)
        if seg.geom_type != "LineString" or seg.length < 0.2:
            return
        if off:
            seg = seg.offset_curve(off)
            if seg.is_empty or seg.geom_type != "LineString":
                return
        (white if col is None else col).append(seg.buffer(wid / 2, cap_style="flat"))

    # ------------------------------------------------------------ crossings and signals (OSM nodes on ways)
    sig_nodes, cross_nodes = set(), {}
    for e in street_osm:
        if e["type"] != "node":
            continue
        t = e.get("tags", {})
        hw, cr = t.get("highway"), t.get("crossing")
        if e["id"] not in inc:
            continue
        if cr == "traffic_signals" or (hw == "traffic_signals" and cr):
            cross_nodes[e["id"]] = "pelican"
        elif hw == "traffic_signals":
            sig_nodes.add(e["id"])
        elif hw == "crossing" and (cr in ("marked", "uncontrolled", "zebra") or t.get("crossing_ref") == "zebra"):
            cross_nodes[e["id"]] = "zebra"

    junctions = [nid for nid in inc if len(arms_at(nid)) >= 3]
    jset = set(junctions)
    excl = {wi: [] for wi in range(len(W))}      # (s0, s1) intervals with no centre line
    warn = {wi: [] for wi in range(len(W))}      # s positions of give-way lines on this way (hazard pattern)
    mouths = []
    n_gw = n_stop = 0

    def near_junction(wi, i, d, within):
        """first junction node along an arm within `within` m: (node index, distance) or None"""
        r = W[wi]
        c0 = r["cum"][i]
        j = i + d
        while 0 <= j < len(r["nodes"]):
            dist = abs(r["cum"][j] - c0)
            if dist > within:
                return None
            if r["nodes"][j] in jset:
                return j, dist
            j += d
        return None

    for nid in junctions:
        A = arms_at(nid)
        rank = lambda a: RANK.get(W[a[0]]["hw"], 2)
        rb = [a for a in A if W[a[0]]["rb"]]
        if rb:
            major = rb
        else:
            best, bs = None, -1e9
            for x in range(len(A)):
                for y in range(x + 1, len(A)):
                    a, b = A[x], A[y]
                    sc = (rank(a) + rank(b)) * 10 + (5 if a[0] == b[0] else 0) \
                        + (3 if W[a[0]]["name"] and W[a[0]]["name"] == W[b[0]]["name"] else 0) \
                        + min(W[a[0]]["w"], W[b[0]]["w"]) * 0.1
                    if sc > bs:
                        best, bs = (a, b), sc
            major = list(best)
        minor = [a for a in A if a not in major]
        maj_hw = max(W[a[0]]["w"] for a in major) / 2
        min_hw = max([W[a[0]]["w"] for a in minor] or [0]) / 2
        signals = nid in sig_nodes
        for a in A:
            wi, i, d = a
            r = W[wi]
            c = r["cum"][i]
            if a in major:
                trim = min_hw + 1.5
            else:
                s_line = (max(W[b[0]]["w"] for b in A if b != a) / 2 + 1.5) if signals else maj_hw + 0.3
                trim = s_line + 1.2
                io = inbound(wi, d)
                if io and r["line"].length > s_line + 1:
                    P, t, n = arm_frame(wi, i, d, s_line)
                    o0, o1 = io
                    if signals:
                        across(P, t, n, 0.15, o0, o1, 0.3)
                        n_stop += 1
                    elif r["w"] >= 3.5 and RANK.get(r["hw"], 2) >= 2 and not near_junction(wi, i, d, s_line + 8):
                        # no lines on driveways/service roads, nor on short links inside a complex junction
                        across(P, t, n, 0.1, o0, o1, 0.2, (0.6, 0.3))
                        across(P, t, n, 0.5, o0, o1, 0.2, (0.6, 0.3))
                        n_gw += 1
                        warn[wi].append(c + d * s_line)
                P, t, n = arm_frame(wi, i, d, 0)
                Q, _, _ = arm_frame(wi, i, d, 14)
                mouths.append(LineString([P, Q]).buffer(r["w"] / 2 + 2.5, cap_style="flat"))
            excl[wi].append((c - trim, c) if d < 0 else (c, c + trim))

    # traffic signals not on a junction node: an approach stop line (if a junction is near) or both ways
    for nid in sig_nodes - jset:
        for wi, i in inc[nid]:
            r = W[wi]
            hit = [(d, near_junction(wi, i, d, 40)) for d in (-1, 1)]
            hit = [(d, h) for d, h in hit if h]
            dirs = [-hit[0][0]] if len(hit) == 1 else [-1, 1]   # approach side: away from the junction
            for d in dirs:
                io = inbound(wi, d)
                if not io:
                    continue
                P, t, n = arm_frame(wi, i, d, 0.0 if hit else 3.0)
                across(P, t, n, 0, io[0], io[1], 0.3)
                n_stop += 1

    # ------------------------------------------------------------ crossings
    def zigzags(wi, c, d, start, two_way):
        r = W[wi]
        hw = r["w"] / 2
        lanes = [hw - 0.3, -(hw - 0.3)] + ([0.0] if two_way else [])
        for base in lanes:
            inward = -1 if base > 0 else 1
            pts = []
            for k in range(9):
                s = c + d * (start + 2.0 * k)
                if s < 0 or s > r["line"].length:
                    break
                P, (tx, ty) = at(wi, s)
                nx, ny = -ty, tx                       # left of increasing s
                # kerb zig-zags swing 0.3 m in from the edge; the centre one swings 0.15 m either side
                o = (0.15 if k % 2 else -0.15) if base == 0 else base + inward * 0.3 * (k % 2)
                pts.append((P[0] + nx * o, P[1] + ny * o))
            if len(pts) >= 2:
                white.append(LineString(pts).buffer(0.05, cap_style="flat", join_style="mitre", mitre_limit=4))

    crossings = []                                    # (wi, s) for de-duplication with Mapillary
    def crossing(kind, wi, c):
        r = W[wi]
        hw = r["w"] / 2
        P, t = at(wi, c)
        n = (-t[1], t[0])
        two_way = r["ow"] == 0
        crossings.append(Point(P))
        if kind == "zebra":
            k = max(5, round(r["w"] / 0.6)) | 1
            for q in range(0, k, 2):
                o0 = -hw + q * r["w"] / k
                across(P, t, n, 0, o0 + 0.02, o0 + r["w"] / k - 0.02, 3.0)
            for sg in (-1, 1):
                across(P, t, n, sg * 2.6, -hw, hw, 0.2, (0.5, 0.5))
            start = 3.2
        else:
            for sg in (-1, 1):
                across(P, t, n, sg * 1.3, -hw, hw, 0.12, (0.12, 0.38))
            for d in (-1, 1):
                io = inbound(wi, d)
                if io:
                    # n for the arm going in direction d is (t_d.y, -t_d.x) with t_d = d * t
                    nd = (d * t[1], -d * t[0])
                    across(P, (d * t[0], d * t[1]), nd, 3.0, io[0], io[1], 0.3)
            start = 3.6
        for d in (-1, 1):
            zigzags(wi, c, d, start, two_way)
        excl[wi].append((c - start - 17, c + start + 17))

    n_cross = {"zebra": 0, "pelican": 0, "mly": 0}
    for nid, kind in cross_nodes.items():
        if nid in jset:
            continue
        wi, i = inc[nid][0]
        crossing(kind, wi, W[wi]["cum"][i])
        n_cross[kind] += 1
    # Mapillary zebra detections where OSM has no crossing (photo-triangulated: a few metres off)
    WT = shapely.STRtree([r["line"] for r in W]) if W else None
    for f_ in mly_feats:
        if f_.get("object_value") != "marking--discrete--crosswalk-zebra" or WT is None:
            continue
        lon, lat = f_["geometry"]["coordinates"]
        pt = Point(*TF.transform(lon, lat))
        if not tile.contains(pt) or any(pt.distance(q) < 20 for q in crossings):
            continue
        wi = int(WT.nearest(pt))
        r = W[wi]
        if r["line"].distance(pt) > 6 or r["hw"] in UNMARKED:
            continue
        s = r["line"].project(pt)
        jn = [abs(r["cum"][i] - s) for i, nid in enumerate(r["nodes"]) if nid in jset]
        if jn and min(jn) < 8:
            continue
        crossing("zebra", wi, s)
        n_cross["mly"] += 1

    # ------------------------------------------------------------ longitudinal lines
    n_centre = 0
    for wi, r in enumerate(W):
        L = r["line"].length
        if r["hw"] in UNMARKED or r["rb"]:
            continue
        ex = sorted(excl[wi])
        def blocked(s):
            for a, b in ex:
                if a <= s < b:
                    return b
            return None
        def next_block(s):
            return min([a for a, b in ex if a > s] or [L])
        if r["ow"] == 0 and r["w"] >= 5.0:
            s = 0.0
            while s < L - 0.5:
                b = blocked(s)
                if b is not None:
                    s = b
                    continue
                near = any(abs(s - g) < 25 for g in warn[wi])
                mark, gap = (6.0, 3.0) if near else (4.0, 5.0)
                e_ = min(s + mark, next_block(s), L)
                along_line(wi, s, e_, 0, 0.1)
                s = e_ + gap
            n_centre += 1
        elif r["ow"] != 0 and r["lanes"]:
            try:
                nl = int(float(r["lanes"]))
            except ValueError:
                nl = 1
            for k in range(1, nl):
                off = -r["w"] / 2 + k * r["w"] / nl
                s = 0.0
                while s < L - 0.5:
                    b = blocked(s)
                    if b is not None:
                        s = b
                        continue
                    e_ = min(s + 2.0, next_block(s), L)
                    along_line(wi, s, e_, off, 0.1)
                    s = e_ + 7.0
    # edge lines on fast roads outside built-up areas
    MOUTHS = shapely.union_all(mouths) if mouths else None
    n_edge = 0
    for wi, r in enumerate(W):
        if r["hw"] not in ("trunk", "primary", "secondary", "trunk_link", "primary_link") or r["w"] < 5.5:
            continue
        if not (r["ms"] >= 40 or (r["ms"] == 0 and all_b.distance(r["line"]) > 30)):
            continue
        for sg in (-1, 1):
            g = r["line"].offset_curve(sg * (r["w"] / 2 - 0.3))
            if g.is_empty:
                continue
            band = g.buffer(0.075, cap_style="flat")
            if MOUTHS is not None:
                band = band.difference(MOUTHS)
            white += _parts(band)
        n_edge += 1

    # bus stop boxes (yellow), from the bays process.py placed at the kerb
    for f in furn:
        if f["t"] != "busbay":
            continue
        E, N = f["x"] + E0, N0 - f["z"]
        tx, ty = math.sin(f["r"]), -math.cos(f["r"])
        n = (ty, -tx)
        P = (E, N)
        Wd, Ld = f["w"], 13.0
        for o in (-Wd / 2, Wd / 2):
            a = (P[0] + n[0] * o - tx * Ld / 2, P[1] + n[1] * o - ty * Ld / 2)
            b = (P[0] + n[0] * o + tx * Ld / 2, P[1] + n[1] * o + ty * Ld / 2)
            yellow.append(_rect(a, b, 0.15))
        for s in (-Ld / 2, Ld / 2):
            across(P, (tx, ty), n, s, -Wd / 2, Wd / 2, 0.15, col=yellow)

    # ------------------------------------------------------------ surface
    BR = shapely.union_all(bridge_surf) if bridge_surf else None
    full = shapely.union_all([roads_u] + list(extra_tarmac))
    surf = full
    if BR is not None:
        surf = surf.difference(BR)
    surf = shapely.make_valid(surf.intersection(tile))
    surf = shapely.union_all(_parts(surf))
    dist = _edge_distance(full)
    T = surface_mesh(surf)
    if len(T):
        flat = T.reshape(-1, 2)
        d = dist(flat)
        E, N = flat[:, 0], flat[:, 1]
        rec = np.stack([E - E0, N0 - N, np.round(d, 2)], 1).astype(np.float32)
        rec.tofile(os.path.join(out, "road_tris.bin"))
    else:
        np.zeros(0, np.float32).tofile(os.path.join(out, "road_tris.bin"))
    print("road surface tris", len(T), "area m2", round(surf.area))

    # ------------------------------------------------------------ clip markings to the surface, triangulate
    shapely.prepare(surf)
    recs = []
    for col, polys in ((WHITE, white), (YELLOW, yellow)):
        polys = [p for p in polys if p is not None and not p.is_empty and p.is_valid]
        if not polys:
            continue
        arr = np.array(polys, dtype=object)
        inner = shapely.contains(surf, arr)
        keep = list(arr[inner])
        for p in arr[~inner]:
            loc = shapely.clip_by_rect(surf, *p.buffer(0.5).bounds)
            keep += _parts(shapely.intersection(p, shapely.make_valid(loc)))
        keep = [p for p in keep if p.area > 0.01]
        Tm = _triangulate(keep, seg=1.0)
        if len(Tm):
            flat = Tm.reshape(-1, 2)
            d = dist(flat)
            recs.append(np.stack([flat[:, 0] - E0, N0 - flat[:, 1], np.round(d, 2), np.full(len(flat), col)], 1))
    M = np.concatenate(recs).astype(np.float32) if recs else np.zeros((0, 4), np.float32)
    M.tofile(os.path.join(out, "markings.bin"))
    print("markings: centre-lined ways", n_centre, "give-ways", n_gw, "stop lines", n_stop, "edge-lined ways", n_edge,
          "crossings", n_cross, "tris", len(M) // 3)
