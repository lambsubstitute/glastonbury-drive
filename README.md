# Glastonbury Drive

Browser driving game set in a 3 km × 3 km square of real Glastonbury, Somerset (British National Grid E 348800–351800, N 137400–140400). The square is shifted east of the town centre so that Glastonbury Tor is inside it.

Live: https://lambsubstitute.github.io/glastonbury-drive/

## Play

```bash
python3 -m http.server 8765 -d game
```

Open http://localhost:8765. WASD/arrows drive, Space handbrake, C camera (chase / bonnet / high / drone), M big map, R back on road, V engine sound (off by default), 1–8 teleport (Market Cross, Abbey Gatehouse, St John's Church, Abbey Barn, Chalice Well, Tor foot, Tor summit, St Benedict's).

## Data

| Source | Used for |
|---|---|
| OpenStreetMap (Overpass) | roads + names, 1,936 buildings, water, land use, shops, street furniture, walls/fences/hedges |
| Environment Agency LIDAR Composite 1 m DTM, first- and last-return DSM | ground (including the Tor), building heights, roof facets, trees, roofs under trees |
| Historic England NHLE | 154 listed buildings: which are listed, grades, names |
| Wikidata + Wikimedia Commons | real wall materials of 19 listed buildings, read by eye from photos (`data/materials_by_photo.txt`) |
| OS OpenMap Local (square ST) | 1,195 building outlines where OSM has none (whole estates were missing from OSM) |
| Mapillary | street lights, bins, poles, signal heads and traffic signs detected from street photos; shopfront colours and frontage materials read by eye from its photos |

## What is real and what is not

- **Roofs and heights** come from LIDAR: area-weighted median roof error 0.29 m against the first-return surface; 13% of roof area is more than 1 m off (mostly edges and steps).
- **Sloping streets**: where OSM maps a whole street block as one outline (the south side of High Street is one 2,500 m2 polygon over ground that rises 5.7 m), the outline is cut into slices along the ground contours, one per metre of rise, so each slice has its own ground level and LIDAR eave. The cuts are straight lines, not the real party walls.
- **Building outlines**: OSM first, OS OpenMap Local where OSM has nothing. OS outlines are generalised (terraces are one block).
- **Wall materials**: 19 listed buildings are classified by eye from Commons photos (stone for the Abbey buildings, the churches, the Tribunal, the George & Pilgrims, the Town Hall; painted render for the Crown, 2 Market Place and others). Only 26 of the 154 listings had a photo linked on Wikidata. On Magdalene Street, Benedict Street, Northload Street and parts of High Street a further 113 pieces of frontage (41 buildings) are classified by eye from Mapillary photos taken 2020-2023 (`tools/facade_photos.py`, `data/materials_by_eye.txt`); photo positions are a few metres out, so pieces were skipped where the photo straddled two buildings, and most of High Street was skipped because its photos look along the street rather than at the fronts. Eight more High Street fronts (Crystals, White Rabbit, Dilliway & Dilliway, The Goddess and the Green Man, Chocolate Love Temple, The Mystic Garden Gallery, Sew Over the Moon, Hundred Monkeys) are read from 36 Wikimedia Commons photos (Category:High Street, Glastonbury, 2009-2022); those photos are sharp but old, and most shops in them have since changed name, so only fronts that could be tied to a current OSM shop were recorded. Ten more High Street fronts (nos. 15, 18, 20, 22, 35, 42, 44, 46, 50, 84) are placed by house number: NHLE listing names and OSM `addr:housenumber` give a number line along each side, and the front is read from a full 2020 Mapillary frame or a Commons photo, tied to a legible shop sign or to the building next to one. Where photos from different years disagree (several fronts were repainted between 2012 and 2020) the newest is used. Every other building has generated materials.
- **Shopfronts**: shop names and positions are from OSM. Paint and fascia colours of 11 shops are read by eye from Mapillary photos taken 2020-2023 (`data/shopfronts_from_photos.txt`), only where the shop's own name was legible; they may have been repainted since. The other 115 named shops have generated colours.
- **Street objects**: where Mapillary or OSM have them they are real (positions can be a few metres out, and are pushed off carriageways); lamp posts elsewhere are generated.
- **Glastonbury Abbey** is a ruin; it is modelled from its outline and LIDAR like any other building, so it reads as low roofless blocks rather than standing arches.
- **St Michael's Tower** on the Tor is on foot-only ground in reality; the game lets you teleport a car to the summit.
- **Lighting**: ambient occlusion is baked with Blender (Cycles) after each build: one value per wall and roof vertex, and a 2048 px top-down map for ground, roads and pavements. Wall triangles are large (median 3.5 m), so wall shading is coarse; it shows most from above and in yards and alleys. The game runs without the bake.
- Trees are placed from LIDAR canopy peaks; species and shapes are generic.

## Rebuild

```bash
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python tools/place.py fetch        # data/place.json is in the repo; LIDAR pieces are cached, rerun to resume
.venv/bin/python tools/place.py os           # OS OpenMap Local, 116 MB zip
.venv/bin/python tools/place.py mapillary    # needs ~/.mapillary_token
.venv/bin/python tools/process.py
.venv/bin/python tools/bake_ao.py          # optional, needs Blender: baked ambient occlusion (about 1 minute)
.venv/bin/python tools/check.py
.venv/bin/python tools/roof_check.py
```

## Publishing

`git subtree push --prefix game origin gh-pages`

Attribution: © OpenStreetMap contributors (ODbL); © Environment Agency (OGL); © Historic England (OGL); contains OS data © Crown copyright and database right 2026 (OGL); street objects © Mapillary contributors (CC BY-SA).

Known gaps: St Michael's Tower stands 13.5 m in the model (really about 21 m) because 1 m LIDAR barely resolves a 7 m roofless shell; distance fog hides most of the view from the Tor summit.
