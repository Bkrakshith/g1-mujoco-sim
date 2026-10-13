"""Extract collision boxes (world-frame AABBs) from the composed Isaac hospital stage.

Run (needs usd-core, not in .venv):
    uv run --no-project --with usd-core python g1-mujoco-sim/extract_scene_boxes.py    # from the hospital-sim root

Writes worlds/hospital_boxes.json, which build_world.py consumes (so building the MuJoCo world
does not need USD). Re-run after `hospital-sim-env/isaac/scene/build_hospital.py` changes the stage.

What is extracted (all world frame, metres, z-up, same frame as hospital-sim-env/isaac/usd/hospital_g1.usd):
  * building   - /World/Hospital children on the ground floor: walls (door openings cut at every door
                 frame found in the same wall), furniture, stairs. Floors, ceilings, trims, lights, door
                 leaves are skipped (doors are treated as open).
  * dressing   - /World/Wing/Dressing/<room>/<item> placed by hospital-sim-env/isaac/scene/layout.py, and
                 /World/Building/Dressing/<room>/<item> (the main-building rooms, layout.BUILDING_DRESSING)
  * suite      - /World/Wing/ORSuite/Props/* (active ones only; layout.SUITE_HIDE are inactive)
The wing's own walls are NOT extracted: build_world.py builds them analytically from layout.py.

Main-building changes of layout.py (final design, 2026-10-13), applied here and not only through the stage:
  * hidden prims   every name of hospital_fixtures.building_hide(layout, spec "full") is skipped, as a box and as a
                   door-frame marker, also when the stage was built before that prim was hidden. The tier only
                   decides which door leaves stay; leaves are never extracted, so the full list is right for all
                   tiers.
  * wall cuts      every rect of layout.BUILDING_OPENINGS (D07, D34, D13, D14) is cut out of every building wall box
                   of any thickness (the courtyard walls are 0.6 m thick). build_world.py closes D07/D34 again
                   for a tier without their exit doorsets (hospital_fixtures.building_openings()).
  * main entrance  the two sliding glass units (layout.BUILDING_UNINSTANCE_HIDE keys: GlassDoor_55, GlassDoor2_5,
                   x -5.64..3.16 on the south facade) are written as category "entrance_glass" (whole unit AABB, frame
                   included), never as "building". scene/build_hospital.py deactivates both units at tier2 and up (an
                   open entrance for the walk-in route) and keeps them below tier2. build_world.py does the same per
                   tier: it adds the entrance_glass boxes only below tier2. The units are read through a session-layer
                   override, so the extraction works on a stage built at any tier.
  * static partitions (screens, rebuilt walls, junction boxes) are not extracted: build_world.py adds them from
                   hospital_fixtures.static_partitions(layout).
  * static leaves  the asset's own door leaves that a building fire doorset replaces (BUILDING_FIRE_DOORS[*]["replaces"],
                   the 8 FD30S hazard-room doors; D07/D34 use their BUILDING_OPENINGS rect instead) are written to
                   "static_leaves" as oriented boxes (centre, half sizes, yaw) at the asset's pose. Isaac keeps these
                   leaves, with their colliders, at every tier that leaves the doorset out (building_hide), so
                   build_world.py adds them as closed_<door> boxes at those tiers. Some asset leaves stand ajar or
                   open; the boxes keep that pose. The leaves are read through a session-layer override, as the units.
"""
import importlib.util
import json
import math
import os
import re

from pxr import Gf, Usd, UsdGeom

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))   # hospital-sim root
OUT = os.path.join(HERE, "worlds", "hospital_boxes.json")
STAGE = os.path.join(REPO, "hospital-sim-env", "isaac", "usd", "hospital_g1.usd")

Z_MIN_MAX = 1.4      # skip things whose bottom is above this (ceiling-hung, wall signs above head)
MIN_HEIGHT = 0.12    # skip things flatter than this (rugs, trims)
MIN_SIZE = 0.06      # skip things smaller than this in both x and y

SKIP_BUILDING = re.compile(r"Light|Roof|Rooff|Ceiling|Floor|Trim|HandBase|Door_|SM_Door|CornerTrim|"
                           r"Window$|Window_|Plane|Pill|Bottle|Books|Intercom|TopTrim|Rug|Sign$")
OPENING = re.compile(r"DoorFrame|DobleDoor")
WALL = re.compile(r"Wall")


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def load_layout():
    L = _load("layout", os.path.join(REPO, "hospital-sim-env", "isaac", "scene", "layout.py"))
    F = _load("hospital_fixtures", os.path.join(REPO, "hospital-sim-env", "fixtures", "hospital_fixtures.py"))
    return L, F


def aabb(cache, prim):
    r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if r.IsEmpty():
        return None
    return [float(v) for v in r.GetMin()] + [float(v) for v in r.GetMax()]


def keep(b):
    if b is None:
        return False
    if b[2] > Z_MIN_MAX or b[5] < 0.05 or b[2] < -0.3:
        return False
    if b[5] - max(b[2], 0.0) < MIN_HEIGHT:
        return False
    if b[3] - b[0] < MIN_SIZE and b[4] - b[1] < MIN_SIZE:
        return False
    return True


def cut_openings(wall, openings):
    """Split a thin axis-aligned wall box at door-frame boxes that intersect it."""
    x0, y0, z0, x1, y1, z1 = wall
    along = 0 if (x1 - x0) >= (y1 - y0) else 1
    lo, hi = wall[along], wall[along + 3]
    holes = []
    for o in openings:
        if all(min(wall[i + 3], o[i + 3]) - max(wall[i], o[i]) > -0.05 for i in (0, 1)) and o[2] < 0.5:
            a, b = max(lo, o[along]), min(hi, o[along + 3])
            if b - a > 0.3:
                holes.append((a, b))
    if not holes:
        return [wall]
    holes.sort()
    pieces, cur = [], lo
    for a, b in holes:
        if a - cur > 0.05:
            pieces.append((cur, a))
        cur = max(cur, b)
    if hi - cur > 0.05:
        pieces.append((cur, hi))
    out = []
    for a, b in pieces:
        w = list(wall)
        w[along], w[along + 3] = a, b
        out.append(w)
    return out


def cut_rect(box, rect, min_piece=0.02):
    """Remove the xy rect (x0, y0, x1, y1), full height, from a box. Returns the remaining pieces (0 to 4 boxes)."""
    bx0, by0, bz0, bx1, by1, bz1 = box
    rx0, ry0, rx1, ry1 = rect
    if rx1 <= bx0 or rx0 >= bx1 or ry1 <= by0 or ry0 >= by1:
        return [box]
    out = []
    if rx0 - bx0 > min_piece:                                   # west of the rect
        out.append([bx0, by0, bz0, rx0, by1, bz1])
    if bx1 - rx1 > min_piece:                                   # east of the rect
        out.append([rx1, by0, bz0, bx1, by1, bz1])
    mx0, mx1 = max(bx0, rx0), min(bx1, rx1)
    if mx1 - mx0 > min_piece:
        if ry0 - by0 > min_piece:                               # south of the rect, inside its x span
            out.append([mx0, by0, bz0, mx1, ry0, bz1])
        if by1 - ry1 > min_piece:                               # north of the rect
            out.append([mx0, ry1, bz0, mx1, by1, bz1])
    return out


def unit_box(st, cache, path):
    """World AABB of a prim also when the stage has it inactive (activated in the session layer, not saved)."""
    p = st.GetPrimAtPath(path)
    if not p.IsValid():
        return None
    if not p.IsActive():
        with Usd.EditContext(st, st.GetSessionLayer()):
            p.SetActive(True)
        p = st.GetPrimAtPath(path)
        cache.Clear()
    return aabb(cache, p)


def unit_obb(st, cache, path):
    """Oriented world box of a prim's meshes (also when the prim is inactive): centre xyz, half sizes xyz, yaw (rad).
    The footprint is the minimum-area rectangle of the mesh points in xy (0.5 deg sweep): the asset's leaf meshes are
    not aligned with their prim frames, so neither the prim-frame box nor the world AABB fits an ajar leaf."""
    p = st.GetPrimAtPath(path)
    if not p.IsValid():
        return None
    if not p.IsActive():
        with Usd.EditContext(st, st.GetSessionLayer()):
            p.SetActive(True)
        p = st.GetPrimAtPath(path)
        cache.Clear()
    pts = []
    for q in Usd.PrimRange(p, Usd.TraverseInstanceProxies()):
        if q.IsA(UsdGeom.Mesh):
            P = UsdGeom.Mesh(q).GetPointsAttr().Get()
            if P:
                m = UsdGeom.Xformable(q).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                pts += [m.Transform(Gf.Vec3d(*v)) for v in P]
    if not pts:
        return None
    xs, ys, zs = [v[0] for v in pts], [v[1] for v in pts], [v[2] for v in pts]
    best = None
    for k in range(180):
        a = math.radians(k * 0.5)
        c, s_ = math.cos(a), math.sin(a)
        u = [x * c + y * s_ for x, y in zip(xs, ys)]
        w = [-x * s_ + y * c for x, y in zip(xs, ys)]
        area = (max(u) - min(u)) * (max(w) - min(w))
        if best is None or area < best[0]:
            best = (area, a, (min(u), max(u)), (min(w), max(w)))
    _, a, (u0, u1), (w0, w1) = best
    c, s_ = math.cos(a), math.sin(a)
    uc, wc = (u0 + u1) / 2, (w0 + w1) / 2
    centre = [uc * c - wc * s_, uc * s_ + wc * c, (min(zs) + max(zs)) / 2]
    half = [(u1 - u0) / 2, (w1 - w0) / 2, (max(zs) - min(zs)) / 2]
    if half[0] < half[1]:                       # yaw along the leaf's long side
        half[0], half[1] = half[1], half[0]
        a += math.pi / 2
    yaw = math.atan2(math.sin(a), math.cos(a))
    return [float(v) for v in centre], [float(v) for v in half], float(yaw)


def static_leaves(st, cache, L):
    """Asset leaves replaced by the building fire doorsets without a BUILDING_OPENINGS rect (see the docstring)."""
    out = []
    for door, d in getattr(L, "BUILDING_FIRE_DOORS", {}).items():
        if d.get("opening"):
            continue
        for prim in d.get("replaces", []):
            obb = unit_obb(st, cache, f"/World/Hospital/{prim}")
            if obb is None:
                raise SystemExit(f"static leaf /World/Hospital/{prim} of {door} is missing or has no mesh")
            c, half, yaw = obb
            out.append({"door": door, "prim": prim, "pos": [round(v, 4) for v in c],
                        "half": [round(v, 4) for v in half], "yaw": round(yaw, 6)})
    return out


def main():
    L, F = load_layout()
    st = Usd.Stage.Open(STAGE)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    boxes = []

    hidden = set(F.building_hide(L, F.build_spec(L, "full")))
    units = dict(getattr(L, "BUILDING_UNINSTANCE_HIDE", {}))      # main-entrance glass units: open doors
    rects = dict(getattr(L, "BUILDING_OPENINGS", {}))             # wall cuts (all tiers; build_world refills)

    entrance = []
    for u in units:
        b = unit_box(st, cache, f"/World/Hospital/{u}")
        if b is None:
            raise SystemExit(f"main-entrance unit /World/Hospital/{u} is not in the stage")
        entrance.append({"name": u, "cat": "entrance_glass", "box": b})
    hosp = st.GetPrimAtPath("/World/Hospital")
    raw, openings, n_hidden = [], [], 0
    for c in hosp.GetChildren():
        n = c.GetName()
        if n in hidden:
            n_hidden += 1
            continue
        if n in units:
            continue
        b = aabb(cache, c)
        if b is None:
            continue
        if OPENING.search(n):
            openings.append(b)
            continue
        if SKIP_BUILDING.search(n) or not keep(b):
            continue
        raw.append((n, b))
    cuts = {k: 0 for k in rects}
    for n, b in raw:
        thin = min(b[3] - b[0], b[4] - b[1]) < 0.35
        is_wall = bool(WALL.search(n))
        pieces = cut_openings(b, openings) if (is_wall and thin) else [b]
        if is_wall:     # BUILDING_OPENINGS: walls of any thickness
            for key, r in rects.items():
                nxt = []
                for pc in pieces:
                    cut = cut_rect(pc, r)
                    if cut != [pc]:
                        cuts[key] += 1
                    nxt += cut
                pieces = nxt
        if is_wall and (thin or len(pieces) != 1 or pieces[0] != b):
            for i, piece in enumerate(pieces):
                boxes.append({"name": f"{n}_{i}", "cat": "building_wall", "box": piece})
        elif pieces:
            boxes.append({"name": n, "cat": "building", "box": pieces[0]})
    boxes += entrance
    leaves = static_leaves(st, cache, L)

    n_dress = {}
    for root, tag in (("/World/Wing/Dressing", "wing"), ("/World/Building/Dressing", "building")):
        dres = st.GetPrimAtPath(root)
        n_dress[tag] = 0
        if not dres.IsValid():
            print(f"note: {root} is not in the stage (scene/build_hospital.py has not built it yet)")
            continue
        for room in dres.GetChildren():
            for it in room.GetChildren():
                b = aabb(cache, it)
                if keep(b):
                    boxes.append({"name": f"{room.GetName()}_{it.GetName()}", "cat": "dressing", "box": b})
                    n_dress[tag] += 1

    for p in st.GetPrimAtPath("/World/Wing/ORSuite/Props").GetChildren():
        b = aabb(cache, p)
        if keep(b):
            boxes.append({"name": p.GetName(), "cat": "suite", "box": b})

    for b in boxes:
        b["box"] = [round(v, 4) for v in b["box"]]
    names = [b["name"] for b in boxes]
    if len(names) != len(set(names)):
        raise SystemExit("duplicate box names in the extraction")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        json.dump({"source": "hospital-sim-env/isaac/usd/hospital_g1.usd", "frame": "world (Isaac), metres",
                   "hidden_skipped": n_hidden, "building_openings_cut": sorted(k for k, v in cuts.items() if v),
                   "entrance_glass": [b["name"] for b in entrance], "dressing": n_dress,
                   "static_leaves": leaves, "boxes": boxes}, f, indent=0)
    from collections import Counter
    print(Counter(b["cat"] for b in boxes), "openings", len(openings), "hidden skipped", n_hidden,
          "wall cuts", cuts, "entrance glass units", [b["name"] for b in entrance], "dressing", n_dress,
          "static leaves", len(leaves), "->", OUT)
    for k in (k for k, v in cuts.items() if not v):
        x0, y0, x1, y1 = rects[k]
        inside = [b["name"] for b in boxes if b["cat"].startswith("building")
                  and b["box"][0] < x1 and b["box"][3] > x0 and b["box"][1] < y1 and b["box"][4] > y0]
        print(f"note: BUILDING_OPENINGS {k} cut no wall box; "
              + (f"boxes still inside it: {inside}" if inside else "it is already open (a door frame cut it)"))


if __name__ == "__main__":
    main()
