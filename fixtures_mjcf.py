"""Moving building fixtures in the MuJoCo hospital: fire-door leaves, bed-lift doors, lift buttons.

The geometry and joint names come from hospital-sim-env/fixtures/hospital_fixtures.py (`build_spec`), the same
module that drives the doors in Isaac Sim. build_world.py calls `add_fixtures()`; it writes
worlds/fixtures.json (the spec) next to the world, and the sim node runs `FixtureController` on it.

MJCF structure (all names start with "fx_", world frame = wing-local + WING_ORIGIN):
  body fx_<Door>_leaf<i>     at the hinge, hinge joint of the same name about +z (0 = closed, + = CCW),
                             range from the spec, box geom = leaf_box() (centre 0.005 + w/2 along closed_dir,
                             z = h/2 + 0.01), inertia of the leaf box; visual vision panel (group 1, no contact)
  body fx_<Lift>_door_{a,b}  at the closed panel centre, slide joint along the spec axis, 0..travel
  body fx_<Lift>_{Open,Close}, fx_<Call> (LobbyCall, MainCall)
                             button: slide joint along the press axis, 0..travel; box with the face
                             BUTTON_SIZE square, BUTTON_DEPTH deep, back face on the mounting surface (spec pos)
  geoms fx_<prefix>front_*, fx_<prefix>header_*, fx_<Lift>_car_*, _return_*, _sill
                             static lift fronts and car shells of every bank (worldbody, class world); the wing
                             prefix is "", the main bank's is "Main_"
Doors with 1 leaf (spec group "Doors1") and 2 leaves ("Doors") are built the same way, leaf by leaf; the building
doors' joint ranges are already clamped to their swing_limit_deg in the spec. The doorset side panels
(spec "side_panels") and the main-building screens are static partitions (build_world.py), not built here.
Moving geoms are in geom group 0 with contype = conaffinity = 1, like the walls: the robot, free objects and
the lidar see them. Contacts between a moving fixture and the static world (walls, floor, frames) and between
leaves of one doorset or panels of one lift are excluded: the joint ranges stop the parts, so no clearance
fight at rest or at the held-open angle. No joint stiffness and no joint damping: the controller applies the
closer, magnet, drive and button-spring efforts through qfrc_applied.
"""
import importlib.util
import json
import math
import os
import xml.etree.ElementTree as ET

BUTTON_DEPTH = 0.012     # m, button cap depth (protrudes this far from the mounting surface when out)
LEAF_ARMATURE = 0.01     # kg m2, numerical only (leaf inertia about the hinge is 7-14 kg m2)
PANEL_ARMATURE = 0.1     # kg, numerical only (panel mass 45 kg)
BUTTON_ARMATURE = 0.002  # kg, numerical only (button mass 0.02 kg)
BUTTON_SOLREFLIMIT = "0.004 1"  # stiff travel stops (2 x timestep): the 2.5 N preload sits 0.2 mm past the stop
MATERIALS = {"fx_leaf": "0.62 0.44 0.30 1", "fx_glass": "0.75 0.88 0.95 0.45", "fx_steel": "0.72 0.74 0.77 1",
             "fx_lift_frame": "0.55 0.57 0.60 1", "fx_car": "0.80 0.80 0.78 1", "fx_button": "0.95 0.85 0.20 1"}


def load_fixtures_module(repo):
    path = os.path.join(repo, "hospital-sim-env", "fixtures", "hospital_fixtures.py")
    spec = importlib.util.spec_from_file_location("hospital_fixtures", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _f(v):
    return " ".join(f"{x:.5f}" for x in v)


def _box_inertia(m, sx, sy, sz, yaw):
    """Full inertia (Ixx Iyy Izz Ixy Ixz Iyz) of a box of full sizes sx, sy, sz turned by yaw about z."""
    iu, iv, iz = m * (sy * sy + sz * sz) / 12, m * (sx * sx + sz * sz) / 12, m * (sx * sx + sy * sy) / 12
    c, s = math.cos(yaw), math.sin(yaw)
    return [c * c * iu + s * s * iv, s * s * iu + c * c * iv, iz, c * s * (iu - iv), 0.0, 0.0]


def add_fixtures(root, wb, L, F, worlds_dir, spec=None, spec_name="fixtures.json"):
    """Add fixture bodies, static lift parts and contact excludes to a world MJCF; write the spec to
    worlds_dir/spec_name. spec: hospital_fixtures.build_spec(L, tier); None = build_spec(L) ($HG1_FIXTURES tier).
    Every group of the spec is built: 2-leaf doors (Doors), 1-leaf doors (Doors1), the lifts of every bank and every
    call button. Returns counts."""
    if spec is None:
        spec = F.build_spec(L)
    ox, oy, oz = spec["wing_origin"]
    asset = root.find("asset")
    for name, rgba in MATERIALS.items():
        ET.SubElement(asset, "material", {"name": name, "rgba": rgba})
    contact = ET.SubElement(root, "contact")
    moving = []

    def W(x, y, z=0.0):
        return (x + ox, y + oy, z + oz)

    # fire-door leaves
    for d in spec["doors"]:
        bodies = []
        for lf in d["leaves"]:
            j = lf["joint"]
            (cx, cy, cz), yaw, (w, t, h) = F.leaf_box(lf, 0.0)
            hx, hy = lf["hinge"]
            rel = (cx - hx, cy - hy, cz)
            b = ET.SubElement(wb, "body", {"name": j, "pos": _f(W(hx, hy))})
            ET.SubElement(b, "inertial", {"pos": _f(rel), "mass": f"{lf['mass']:.3f}",
                                          "fullinertia": _f(_box_inertia(lf["mass"], w, t, h, yaw))})
            ET.SubElement(b, "joint", {"name": j, "type": "hinge", "axis": "0 0 1", "limited": "true",
                                       "range": _f(lf["range"]), "armature": str(LEAF_ARMATURE),
                                       "damping": "0", "stiffness": "0"})
            ET.SubElement(b, "geom", {"name": f"{j}_geom", "class": "world", "type": "box", "material": "fx_leaf",
                                      "pos": _f(rel), "euler": f"0 0 {yaw:.6f}",
                                      "size": _f((w / 2, t / 2, h / 2))})
            # vision panel (visual only): inset from the edges, bottom..top z (HTM 05-02 App C)
            inset, z0, z1 = lf["vision_panel"]
            ET.SubElement(b, "geom", {"name": f"{j}_vision", "type": "box", "material": "fx_glass", "group": "1",
                                      "contype": "0", "conaffinity": "0", "mass": "0",
                                      "pos": _f((rel[0], rel[1], (z0 + z1) / 2)), "euler": f"0 0 {yaw:.6f}",
                                      "size": _f(((w - 2 * inset) / 2, t / 2 + 0.002, (z1 - z0) / 2))})
            bodies.append(j)
        moving += bodies
        for i in range(len(bodies)):
            for k in range(i + 1, len(bodies)):
                ET.SubElement(contact, "exclude", {"body1": bodies[i], "body2": bodies[k]})

    # lift door panels
    for lift in spec["lifts"]:
        pnames = []
        for p in lift["panels"]:
            j = p["joint"]
            b = ET.SubElement(wb, "body", {"name": j, "pos": _f(W(*p["closed_center"]))})
            sx, sy, sz = p["size"]
            ET.SubElement(b, "inertial", {"pos": "0 0 0", "mass": f"{p['mass']:.3f}",
                                          "fullinertia": _f(_box_inertia(p["mass"], sx, sy, sz, 0.0))})
            ET.SubElement(b, "joint", {"name": j, "type": "slide", "axis": _f(p["axis"]), "limited": "true",
                                       "range": f"0 {p['travel']:.5f}", "armature": str(PANEL_ARMATURE),
                                       "damping": "0", "stiffness": "0"})
            ET.SubElement(b, "geom", {"name": f"{j}_geom", "class": "world", "type": "box", "material": "fx_steel",
                                      "size": _f((sx / 2, sy / 2, sz / 2))})
            pnames.append(j)
        moving += pnames
        ET.SubElement(contact, "exclude", {"body1": pnames[0], "body2": pnames[1]})

    # buttons: lift car panels and landing calls
    bsize = spec["button"]["size"]
    for bt in [b for l in spec["lifts"] for b in l["buttons"]] + list(spec["calls"]):
        j = bt["joint"]
        ax = bt["axis"]
        half = [bsize / 2] * 3
        for k in range(3):
            if abs(ax[k]) > 0.5:
                half[k] = BUTTON_DEPTH / 2
        centre = [-ax[k] * BUTTON_DEPTH / 2 for k in range(3)]  # back face on the mounting surface
        b = ET.SubElement(wb, "body", {"name": j, "pos": _f(W(*bt["pos"]))})
        ET.SubElement(b, "joint", {"name": j, "type": "slide", "axis": _f(ax), "limited": "true",
                                   "range": f"0 {spec['button']['travel']:.5f}", "armature": str(BUTTON_ARMATURE),
                                   "solreflimit": BUTTON_SOLREFLIMIT, "damping": "0", "stiffness": "0"})
        ET.SubElement(b, "geom", {"name": f"{j}_geom", "class": "world", "type": "box", "material": "fx_button",
                                  "pos": _f(centre), "size": _f(half), "mass": f"{spec['button']['mass']:.4f}"})
        moving.append(j)

    for j in moving:
        ET.SubElement(contact, "exclude", {"body1": "world", "body2": j})

    # static lift parts (fronts with openings, headers, car shells, sills)
    for name, lo, hi in spec["static_boxes"]:
        a, b = W(*lo), W(*hi)
        mat = "fx_lift_frame" if ("front" in name or "header" in name) else "fx_car"
        x0, y0, z0, x1, y1, z1 = a[0], a[1], a[2], b[0], b[1], b[2]
        if x1 - x0 < 1e-3 or y1 - y0 < 1e-3 or z1 - z0 < 1e-3:
            continue
        ET.SubElement(wb, "geom", {"name": name, "class": "world", "type": "box", "material": mat,
                                   "pos": _f(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)),
                                   "size": _f(((x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2))})

    with open(os.path.join(worlds_dir, spec_name), "w") as f:
        json.dump(spec, f, indent=1)
    return {"tier": spec["tier"], "doors": len(spec["groups"]["Doors"]), "doors1": len(spec["groups"]["Doors1"]),
            "lifts": len(spec["lifts"]), "calls": len(spec["calls"]),
            "leaves": sum(len(d["leaves"]) for d in spec["doors"]),
            "lift_panels": sum(len(l["panels"]) for l in spec["lifts"]),
            "buttons": sum(len(l["buttons"]) for l in spec["lifts"]) + len(spec["calls"]),
            "lift_static": len(spec["static_boxes"])}


def partition_walls(L, wall_h):
    """Interior wall lines (layout.PARTITIONS) minus DOORWAYS, WALL_T thick, centred on the line. Wing-local."""
    t = L.WALL_T
    boxes = []
    for i, (x0, y0, x1, y1) in enumerate(getattr(L, "PARTITIONS", [])):
        vertical = abs(x0 - x1) < 1e-9
        c = x0 if vertical else y0
        lo, hi = (min(y0, y1), max(y0, y1)) if vertical else (min(x0, x1), max(x0, x1))
        segs = [(lo, hi)]
        for dx0, dy0, dx1, dy1 in L.DOORWAYS.values():
            if vertical and abs(dx0 - dx1) < 1e-9 and abs(dx0 - c) < 1e-6:
                ha, hb = min(dy0, dy1), max(dy0, dy1)
            elif not vertical and abs(dy0 - dy1) < 1e-9 and abs(dy0 - c) < 1e-6:
                ha, hb = min(dx0, dx1), max(dx0, dx1)
            else:
                continue
            nxt = []
            for a, b in segs:
                if hb <= a or ha >= b:
                    nxt.append((a, b))
                    continue
                if ha > a:
                    nxt.append((a, ha))
                if hb < b:
                    nxt.append((hb, b))
            segs = nxt
        for a, b in segs:
            if b - a < 0.02:
                continue
            if vertical:
                boxes.append((f"Partition{i}_{a:g}", [c - t / 2, a, 0.0, c + t / 2, b, wall_h]))
            else:
                boxes.append((f"Partition{i}_{a:g}", [a, c - t / 2, 0.0, b, c + t / 2, wall_h]))
    return boxes


def doorway_zones(L):
    """Clear zones of every doorway, world frame (x0, y0, x1, y1): the wing's DOORWAYS (wing-local, shifted by
    WING_ORIGIN) and the main building's BUILDING_DOORWAYS (world frame), each the opening +- DOOR_CLEARANCE on both
    sides (the zone of the design check, design_final.md s.3 and check.py s.3)."""
    ox, oy, _ = L.WING_ORIGIN
    c = getattr(L, "DOOR_CLEARANCE", 1.0)
    segs = [(x0 + ox, y0 + oy, x1 + ox, y1 + oy) for x0, y0, x1, y1 in L.DOORWAYS.values()]
    segs += list(getattr(L, "BUILDING_DOORWAYS", {}).values())
    zones = []
    for x0, y0, x1, y1 in segs:
        if abs(y0 - y1) < 1e-9:
            zones.append((min(x0, x1), y0 - c, max(x0, x1), y0 + c))
        else:
            zones.append((x0 - c, min(y0, y1), x0 + c, max(y0, y1)))
    return zones


def doorway_blockers(extracted, L):
    """Names of extracted furniture boxes inside a doorway's clear zone (doorway_zones(), 0.08-1.8 m high), wing
    and main building. The Isaac dressing should keep these zones free; until the boxes are re-extracted,
    build_world drops them so the doorway stays walkable."""
    zones = doorway_zones(L)
    out = []
    for it in extracted:
        # walls and the building's stairs are structure, not furniture: a stair door opens onto the stair landing
        # (design_final.md s.4.4: landings 1.0 m W, 0.55 m E), so the flight is inside the door's 1 m zone
        if it["cat"] == "building_wall" or "Stairs" in it["name"]:
            continue
        X0, Y0, Z0, X1, Y1, Z1 = it["box"]
        if Z0 > 1.8 or Z1 < 0.08:
            continue
        for zone in zones:
            if X0 < zone[2] - 0.02 and X1 > zone[0] + 0.02 and Y0 < zone[3] - 0.02 and Y1 > zone[1] + 0.02:
                out.append(it["name"])
                break
    return out
