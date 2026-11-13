"""Lay out Mapillary crops of the building frontages along named streets, so wall materials of buildings
that have no listing photo can be read by eye.

Usage: facade_photos.py <sheet output dir> "High Street" "Market Place" ...
Run after tools/process.py. Every ~9 m along each street, on both sides, the nearest building wall within
18 m of the centre line is one "piece" of frontage. Pieces of buildings that already have a material from a
listing photo are skipped. Writes data/facade_index.json: sheet number -> BNG point 1.5 m inside the wall,
street, photo year. Classify into data/materials_by_eye.txt as "<E> <N> <code>" (codes as in
materials_by_photo.txt); process.py gives each wall section the material of its nearest point.
Photo crops go to data/mly_thumbs/ (CC BY-SA imagery, kept out of the repo).
"""
import json, math, os, sys, time, urllib.request
import pyproj
from pyproj import Transformer
from PIL import Image, ImageDraw, ImageFont
from shapely.geometry import LineString, Polygon, Point
from shapely.strtree import STRtree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mapillary as mly

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
D = os.path.join(ROOT, "data")
OUT_SHEETS, STREETS = sys.argv[1], sys.argv[2:]
THUMBS = os.path.join(D, "mly_thumbs")
os.makedirs(THUMBS, exist_ok=True)
os.makedirs(OUT_SHEETS, exist_ok=True)
E0, N0 = json.load(open(os.path.join(D, "place.json")))["origin"]
pyproj.network.set_network_enabled(True)
TF = Transformer.from_crs(4326, 27700, always_xy=True)
STEP, REACH = 9.0, 18.0

foots = json.load(open(os.path.join(ROOT, "game", "data", "footprints.json")))
polys = [Polygon(f["p"]).buffer(0) for f in foots]
tree = STRtree(polys)
roads = [r for r in json.load(open(os.path.join(ROOT, "game", "data", "roads.json"))) if r.get("n") in STREETS]

pieces, seen = [], []
for r in roads:
    line = LineString([p[:2] for p in r["pts"]])
    k = 0.0
    while k < line.length:
        a, b = line.interpolate(k), line.interpolate(min(k + 0.5, line.length))
        k += STEP
        tx, tz = b.x - a.x, b.y - a.y
        tl = math.hypot(tx, tz)
        if tl < 1e-6:
            continue
        for side in (1, -1):
            px, pz = -tz / tl * side, tx / tl * side            # unit vector from the road towards one side
            ray = LineString([(a.x + px * 2, a.y + pz * 2), (a.x + px * REACH, a.y + pz * REACH)])
            best = None
            for i in tree.query(ray, predicate="intersects"):
                d = polys[i].distance(a)
                if best is None or d < best[0]:
                    best = (d, i)
            if best is None:
                continue
            d, i = best
            if foots[i].get("m"):                               # already has a material from a listing photo
                continue
            wx, wz = a.x + px * d, a.y + pz * d                 # the wall point facing the road
            if any(math.hypot(wx - s[0], wz - s[1]) < STEP * 0.6 for s in seen):
                continue                                        # the other carriageway already covered it
            seen.append((wx, wz))
            pieces.append({"x": wx, "z": wz, "nx": -px, "nz": -pz, "street": r["n"],
                           "mx": wx + px * 1.5, "mz": wz + pz * 1.5})
print("frontage pieces", len(pieces))

cams = []
for im in json.load(open(os.path.join(D, "mapillary", "images.json"))):
    if im.get("compass_angle") is None:
        continue
    E, N = TF.transform(*im["geometry"]["coordinates"])
    year = int(1970 + im.get("captured_at", 0) / (1000 * 3600 * 24 * 365.25))
    cams.append((E - E0, N0 - N, im["compass_angle"], year, im["id"]))

HFOV = 70.0
picks = []
for pi, s in enumerate(pieces):
    best = None
    for (cx, cz, comp, year, iid) in cams:
        dx, dz = s["x"] - cx, s["z"] - cz
        dist = math.hypot(dx, dz)
        if not (5 < dist < 28):
            continue
        front = -(dx * s["nx"] + dz * s["nz"])
        if front < 2.5:
            continue
        bearing = math.degrees(math.atan2(dx, -dz)) % 360
        diff = (bearing - comp + 540) % 360 - 180
        if abs(diff) > HFOV / 2 - 6:
            continue
        score = abs(diff) / 30 + abs(dist - 12) / 12 + (1 - front / dist) * 1.5 + (2025 - year) * 0.1
        if best is None or score < best[0]:
            best = (score, iid, diff, year)
    if best:
        picks.append((pi, best))
print("with a usable photo", len(picks))

ids = sorted({b[1] for _, b in picks})
urls = {}
for k in range(0, len(ids), 50):
    need = [i for i in ids[k:k + 50] if not os.path.exists(os.path.join(THUMBS, f"{i}.jpg"))]
    if not need:
        continue
    d = mly.get("images", {"image_ids": ",".join(need), "fields": "id,thumb_1024_url"})
    for x in d.get("data", []):
        urls[x["id"]] = x.get("thumb_1024_url")
    time.sleep(0.2)

crops = []
for pi, (score, iid, diff, year) in picks:
    path = os.path.join(THUMBS, f"{iid}.jpg")
    if not os.path.exists(path):
        u = urls.get(iid)
        if not u:
            continue
        try:
            data = urllib.request.urlopen(urllib.request.Request(u, headers={"User-Agent": "real-place-drive/0.1"}), timeout=60).read()
            open(path, "wb").write(data)
        except Exception:
            continue
        time.sleep(0.15)
    im = Image.open(path).convert("RGB")
    W, H = im.size
    cx = W * (0.5 + diff / HFOV)
    half = W * 0.2
    x0 = int(max(0, min(W - 2 * half, cx - half)))
    crop = im.crop((x0, int(H * 0.08), int(x0 + 2 * half), int(H * 0.78)))
    # a tick at the top marks the column of the photo that the frontage piece sits in
    ImageDraw.Draw(crop).polygon([(cx - x0 - 9, 0), (cx - x0 + 9, 0), (cx - x0, 16)], fill=(255, 40, 40))
    crops.append((pi, crop, year))

C, R, TW, TH = 4, 3, 320, 330
font = ImageFont.load_default(size=13)
index = []
for s0 in range(0, len(crops), C * R):
    sheet = Image.new("RGB", (C * TW, R * TH), (30, 30, 30))
    d = ImageDraw.Draw(sheet)
    for k, (pi, crop, year) in enumerate(crops[s0:s0 + C * R]):
        x, y = (k % C) * TW, (k // C) * TH
        c = crop.copy()
        c.thumbnail((TW - 6, TH - 30))
        sheet.paste(c, (x + 3, y + 26))
        p = pieces[pi]
        n = len(index)
        d.rectangle([x, y, x + TW, y + 24], fill=(0, 0, 0))
        d.text((x + 4, y + 5), f"#{n} {p['street'][:22]} ({year})", fill=(255, 230, 120), font=font)
        index.append({"n": n, "E": round(E0 + p["mx"], 1), "N": round(N0 - p["mz"], 1), "street": p["street"], "year": year})
    sheet.save(os.path.join(OUT_SHEETS, f"facades_{s0 // (C * R):02d}.jpg"), quality=85)
json.dump(index, open(os.path.join(D, "facade_index.json"), "w"), indent=0)
print("crops", len(crops), "sheets", (len(crops) + C * R - 1) // (C * R))
