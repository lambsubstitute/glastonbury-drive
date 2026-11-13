"""Hand-built models for Glastonbury: St Michael's Tower on the Tor and the standing ruins of the Abbey church.

Positions, lengths and heights are measured from the EA 1 m LIDAR (first/last return minus ground) and the
OSM ruin outlines. Openings, bay rhythm, turrets and battlements are drawn by hand from general knowledge of
the buildings and the Commons photos - they are the right kind of thing in the right place, not a survey.
"""
import math
from shapely.geometry import box, Polygon
from landmark_kit import opening, stepped, battlements, STYLE_RENDER, STYLE_BRICK, STYLE_ASHLAR

TOWER_STONE = (156, 148, 130)
ABBEY_STONE = (181, 170, 144)

# ---- St Michael's Tower: 7.6 m square, long axis from the OSM outline (about 27 degrees north of east) ----
TC = (351220.0, 138616.5)
TA = math.atan2(4.0, 8.0)
# ---- Abbey church: axis through the crossing, 2.9 degrees north of east (Lady Chapel centre -> crossing) ----
AC = (350081.0, 138850.0)
AA = math.atan(0.05)

EXCLUDE = [
    (lambda t: True, box(TC[0] - 8, TC[1] - 8, TC[0] + 8, TC[1] + 8)),                           # the tower's outline
    (lambda t: t.get("building") == "ruins", box(350020, 138795, 350160, 138890)),               # church ruins
]


def frame(c, a):
    ca, sa = math.cos(a), math.sin(a)
    return lambda u, v: (c[0] + u * ca - v * sa, c[1] + u * sa + v * ca)


def build_tower(K):
    T = frame(TC, TA)
    base = float(min(K.ground(TC[0], TC[1]))) - 0.4
    H, R, TH = 20.2, 3.8, 1.15
    faces = [  # (start, end) corners; the first two are the west and east faces, with the through-passage doors
        (T(-R + TH / 2, -R), T(-R + TH / 2, R), True), (T(R - TH / 2, -R), T(R - TH / 2, R), True),
        (T(-R, -R + TH / 2), T(R, -R + TH / 2), False), (T(-R, R - TH / 2), T(R, R - TH / 2), False)]
    L = 2 * R
    for k, (p0, p1, door) in enumerate(faces):
        ops = [opening(L / 2, 0.8, 8.4, 10.8), opening(L / 2 - 1.05, 0.75, 14.4, 18.3), opening(L / 2 + 1.05, 0.75, 14.4, 18.3)]
        if door:
            ops.append(opening(L / 2, 2.0, 0.0, 4.0))
        K.wall(p0, p1, TH, battlements(L, H), TOWER_STONE, ops, base=base,
               name="St Michael's Tower" if k == 0 else None, grade="I")
        # plinth and string courses: bands standing 0.15 m proud of the wall face
        K.wall(p0, p1, TH + 0.5, 1.0, TOWER_STONE, [opening(L / 2, 2.0, 0.0, 4.0)] if door else [], base=base)
        for y in (6.3, 13.7, 19.7):
            K.wall(p0, p1, TH + 0.3, y + 0.28, TOWER_STONE, [box(-1, -2, L + 1, y)], base=base)
    for su in (-1, 1):                                  # corner buttresses, stepping in at the belfry stage
        for sv in (-1, 1):
            c = T(su * (R + 0.1), sv * (R + 0.1))
            K.pier(c, (1.6, 1.6), 13.9, TOWER_STONE, TA, base=base)
            K.pier(c, (1.15, 1.15), 19.5, TOWER_STONE, TA, base=base)


def build_abbey(K):
    A = frame(AC, AA)
    S = ABBEY_STONE
    # --- Lady Chapel (west end): four bays, round-headed windows, north and south doors, corner turrets
    for sv, nm in ((-1, "Glastonbury Abbey: Lady Chapel"), (1, None)):
        ops = [opening(s, 1.7, 4.6 if s != 14.4 else 5.6, 9.4, "round") for s in (2.9, 8.6, 14.4, 20.1)]
        ops.append(opening(14.4, 2.0, 0.0, 3.9, "round"))
        K.wall(A(-107, sv * 6.2), A(-84, sv * 6.2), 1.4, stepped([(0, 11.6), (23, 11.2)], rough=0.35, seed=3 + sv), S,
               ops, name=nm, grade="I")
    K.wall(A(-107, -6.2), A(-107, 6.2), 1.4, stepped([(0, 11.5), (6.2, 13.4), (12.4, 11.5)], rough=0.3, seed=5), S,
           [opening(3.2, 1.2, 4.8, 9.2, "round"), opening(6.2, 1.3, 4.8, 10.2, "round"), opening(9.2, 1.2, 4.8, 9.2, "round")])
    for u, h in ((-107, 14.8), (-84, 13.2)):
        for sv in (-1, 1):
            K.pier(A(u, sv * 6.4), (2.3, 2.3), [(0, h), (1.15, h + 1.2), (2.3, h)], S, AA)
    K.wall(A(-84, -6.2), A(-84, 6.2), 1.3, stepped([(0, 10.4), (12.4, 9.0)], rough=0.4, seed=7), S,
           [opening(6.2, 6.4, 0.0, 9.0)])
    # --- Galilee, joining the Lady Chapel to the nave: lower walls, pointed windows, open east arch
    for sv in (-1, 1):
        K.wall(A(-84, sv * 6.2), A(-68, sv * 6.2), 1.3, stepped([(0, 8.6), (8, 7.4), (16, 6.2)], rough=0.5, seed=11 + sv), S,
               [opening(s, 1.6, 2.8, 6.0) for s in (2.7, 8.0, 13.3)])
    K.wall(A(-68, -6.2), A(-68, 6.2), 1.3, stepped([(0, 7.0), (6.2, 9.4), (12.4, 6.6)], rough=0.4, seed=13), S,
           [opening(6.2, 5.2, 0.0, 7.6)])
    # --- nave: the south aisle wall, 29 m, up to 12.7 m
    K.wall(A(-51, -13), A(-22, -13), 1.6, stepped([(0, 8.6), (6, 12.4), (20, 12.0), (29, 8.0)], rough=0.6, seed=17), S,
           [opening(s, 2.4, 4.0, 9.4 if 4 < s < 24 else 7.4) for s in (3.5, 9.0, 14.5, 20.0, 25.5)],
           name="Glastonbury Abbey: nave south aisle")
    # --- crossing: the two east piers (21.2 m south, 18.5 m north) with the springing of the great arch,
    #     and the tall arches into the transept chapels beside them
    for sv, h, nm in ((-1, 21.0, "Glastonbury Abbey: crossing piers"), (1, 18.4, None)):
        K.pier(A(1, sv * 7.5), (3.2, 3.2), stepped([(0, h), (3.2, h - 0.6)], step=0.8, rough=0.3, seed=19 + sv), S, AA,
               name=nm, grade="I")
        # arch springer leaning over the crossing: a slab with everything below the curve cut away
        cut = Polygon([(-0.1, -2), (4.7, -2), (4.7, h - 4.4), (3.3, h - 5.8), (2.0, h - 7.8), (1.1, h - 10.4), (0.5, h - 13), (-0.1, h - 13)])
        K.wall(A(1, sv * 6.0), A(1, sv * 1.5), 2.2, stepped([(0, h), (1.6, h - 0.4), (3.2, h - 2.2), (4.5, h - 4.0)], step=0.5, seed=23), S, [cut])
        K.wall(A(1, sv * 9.0), A(1, sv * 18.0), 1.8,
               stepped([(0, h - 2), (3, h - 3.5), (6, h - 8.5), (9, h - 13)], rough=0.5, seed=29 + sv), S,
               [opening(3.6, 3.4, 0.0, h - 9.5)])
    # --- choir aisle walls
    K.wall(A(4, -12.5), A(36, -12.5), 1.5, stepped([(0, 8.6), (8, 7.0), (20, 5.6), (32, 5.0)], rough=0.6, seed=31), S,
           [opening(s, 2.2, 1.8, 4.4) for s in (4, 10, 16, 22, 28)], name="Glastonbury Abbey: choir")
    K.wall(A(4, 12.0), A(18, 12.0), 1.5, stepped([(0, 6.0), (14, 3.4)], rough=0.5, seed=37), S)
    # --- north transept: east wall with two chapel arches, and part of the north end
    K.wall(A(3, 19.5), A(3, 31), 1.8, stepped([(0, 15.5), (4, 17.4), (8, 13.6), (11.5, 9.0)], rough=0.6, seed=41), S,
           [opening(3.2, 3.0, 0.0, 10.0), opening(8.4, 2.8, 0.0, 7.6)])
    K.wall(A(-11, 31), A(3, 31), 1.6, stepped([(0, 5.6), (8, 10.0), (14, 12.0)], rough=0.6, seed=43), S,
           [opening(8.5, 2.8, 3.0, 8.0)])
    # --- east end: the south-east corner of the retrochoir
    K.wall(A(66, -21), A(66, -12), 1.6, stepped([(0, 7.0), (5, 12.0), (9, 9.0)], rough=0.5, seed=47), S,
           [opening(5.0, 2.0, 3.0, 7.6)], name="Glastonbury Abbey: east end")
    K.wall(A(59, -12.5), A(66, -12.5), 1.5, stepped([(0, 5.0), (7, 9.0)], rough=0.5, seed=53), S)
    # --- wall running south from the nave (cloister / refectory range), low
    K.wall(A(-17, -24), A(-17, -50), 0.9, stepped([(0, 2.6), (26, 2.0)], rough=0.3, seed=59), S)


# ---- Market Place, east side: nos. 6-8 and the brick building south of them ---------------------------
# One OSM outline covers this whole block. Plot widths, eaves and ridges are read from the 1 m LIDAR along
# the frontage; fronts are read from a 2023 Mapillary frame (id 1333817450545843) and the Commons photo
# filed under listing 1345452. Window sizes and exact positions are by eye.
CUT = [box(349888.0, 138891.6, 349899.5, 138928.6)]
WHITE, CLAY, SLATE = (240, 238, 232), (150, 82, 60), (84, 86, 92)


def build_market_place(K):
    # no. 6: tall, narrow, white-painted gable front with black quoins; rubble stone flank; clay tiles
    K.house((349889.9, 138921.3), (349892.3, 138928.0), 6.5, 8.0, 10.9, WHITE, CLAY, roof="gable_front",
            side_col=(150, 140, 125), side_style=STYLE_ASHLAR, quoins=(28, 28, 30), shop_h=2.9,
            windows=[opening(3.55, 1.3, 3.9, 6.2, "round"), opening(3.55, 1.15, 6.9, 8.7, "flat")],
            shops=[(0.3, 6.8, None, "2b2f2c", "2b2f2c")], name="6, Market Place", grade="II")
    # low single-storey link carrying the shopfront across to no. 7
    K.house((349891.2, 138918.5), (349891.2, 138921.3), 5.0, 3.1, 3.7, WHITE, CLAY,
            shops=[(0.0, 2.8, None, "19a5c8", "19a5c8")])
    # no. 7: white gable to the street with dark bargeboards, wide first-floor window, small attic light
    K.house((349889.4, 138913.2), (349889.0, 138918.5), 11.0, 5.2, 8.1, WHITE, CLAY, roof="gable_front",
            barge=(34, 32, 30), windows=[opening(2.65, 2.2, 3.5, 4.9, "flat"), opening(2.65, 0.9, 5.6, 6.7, "flat")],
            shops=[(0.1, 5.2, "Ashok & Babali Good Karma", "19a5c8", "e8d44a")], name="7, Market Place", grade="II")
    # no. 8 and its neighbour: long low white range, roof along the street, four small sashes, three shops
    K.house((349890.2, 138902.5), (349889.4, 138913.2), 7.0, 5.8, 8.0, (238, 236, 230), (118, 96, 80),
            windows=[opening(s, 0.9, 3.7, 5.1, "flat") for s in (1.4, 4.0, 6.7, 9.3)],
            shops=[(0.1, 3.6, "Little Imps", "1c1c1e", "1c1c1e"), (3.6, 7.2, None, "28b8a8", "e8409a"),
                   (7.2, 10.6, "Elestial", "28b8a8", "e8409a")], name="8, Market Place", grade="II")
    # three-storey red brick with stone quoins, three bays, slate roof; stone shopfront
    K.house((349891.0, 138891.6), (349890.2, 138902.5), 8.5, 9.6, 12.2, (255, 232, 216), SLATE, style=STYLE_BRICK,
            quoins=(215, 205, 180), shop_h=3.6,
            windows=[opening(s, 1.1, y0, y1, "flat") for s in (1.9, 5.45, 9.0) for y0, y1 in ((4.4, 6.3), (7.2, 8.9))],
            shops=[(0.1, 10.8, "Man, Myth & Magik", "b8aa90", "dcdce0")])


def build(K):
    build_tower(K)
    build_abbey(K)
    build_market_place(K)
