"""Extract collision boxes (world-frame AABBs) from the composed Isaac hospital stage.

Run (needs usd-core, not in .venv):
    uv run --no-project --with usd-core python g1-mujoco-sim/extract_scene_boxes.py    # from the hospital-sim root

Writes worlds/hospital_boxes.json, which build_world.py consumes (so building the MuJoCo world
does not need USD). Re-run after `hospital-sim-env/isaac/scene/build_hospital.py` changes the stage.

What is extracted (all world frame, metres, z-up, same frame as hospital-sim-env/isaac/usd/hospital_g1.usd):
  * building   - /World/Hospital children on the ground floor: walls (door openings cut at every door
                 frame found in the same wall), furniture, stairs. Floors, ceilings, trims, lights, door
                 leaves are skipped (doors are treated as open).
  * dressing   - /World/Wing/Dressing/<room>/<item> placed by hospital-sim-env/isaac/scene/layout.py
  * suite      - /World/Wing/ORSuite/Props/* (active ones only; layout.SUITE_HIDE are inactive)
The wing's own walls are NOT extracted: build_world.py builds them analytically from layout.py.
"""
import json
import os
import re

from pxr import Usd, UsdGeom

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))   # hospital-sim root
OUT = os.path.join(HERE, "worlds", "hospital_boxes.json")

Z_MIN_MAX = 1.4      # skip things whose bottom is above this (ceiling-hung, wall signs above head)
MIN_HEIGHT = 0.12    # skip things flatter than this (rugs, trims)
MIN_SIZE = 0.06      # skip things smaller than this in both x and y

SKIP_BUILDING = re.compile(r"Light|Roof|Rooff|Ceiling|Floor|Trim|HandBase|Door_|SM_Door|CornerTrim|"
                           r"Window$|Window_|Plane|Pill|Bottle|Books|Intercom|TopTrim|Rug|Sign$")
OPENING = re.compile(r"DoorFrame|DobleDoor")
WALL = re.compile(r"Wall")


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


def main():
    st = Usd.Stage.Open(os.path.join(REPO, "hospital-sim-env", "isaac", "usd", "hospital_g1.usd"))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    boxes = []

    hosp = st.GetPrimAtPath("/World/Hospital")
    raw, openings = [], []
    for c in hosp.GetChildren():
        n = c.GetName()
        b = aabb(cache, c)
        if b is None:
            continue
        if OPENING.search(n):
            openings.append(b)
            continue
        if SKIP_BUILDING.search(n) or not keep(b):
            continue
        raw.append((n, b))
    for n, b in raw:
        thin = min(b[3] - b[0], b[4] - b[1]) < 0.35
        if WALL.search(n) and thin:
            for i, piece in enumerate(cut_openings(b, openings)):
                boxes.append({"name": f"{n}_{i}", "cat": "building_wall", "box": piece})
        else:
            boxes.append({"name": n, "cat": "building", "box": b})

    dres = st.GetPrimAtPath("/World/Wing/Dressing")
    for room in dres.GetChildren():
        for it in room.GetChildren():
            b = aabb(cache, it)
            if keep(b):
                boxes.append({"name": f"{room.GetName()}_{it.GetName()}", "cat": "dressing", "box": b})

    for p in st.GetPrimAtPath("/World/Wing/ORSuite/Props").GetChildren():
        b = aabb(cache, p)
        if keep(b):
            boxes.append({"name": p.GetName(), "cat": "suite", "box": b})

    for b in boxes:
        b["box"] = [round(v, 4) for v in b["box"]]
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w") as f:
        json.dump({"source": "hospital-sim-env/isaac/usd/hospital_g1.usd", "frame": "world (Isaac), metres",
                   "boxes": boxes}, f, indent=0)
    from collections import Counter
    print(Counter(b["cat"] for b in boxes), "openings", len(openings), "->", OUT)


if __name__ == "__main__":
    main()
