"""Scrub-nurse set for the operating theatre: instrument table, tool rack with instruments, surgeon stand-in.

Used by build_world.py. Everything is placed relative to one pose, STATION: where the G1's pelvis stands when it is
lined up at the instrument table (world frame = Isaac world frame; the theatre is wing-local x 0..10, y -20..-10,
see hospital-sim-env/isaac/scene/layout.py). The east half of the theatre is free floor between the OR table group and the east wall.

    world (x, y) of a station-local point (fx, ly) = STATION + fx * forward + ly * left
    forward = (cos yaw, sin yaw), left = (-sin yaw, cos yaw)

Assets
  * STEP-1 PRIMITIVE STAND-INS (clearly marked, "standin": true in theatre_setup.json): every instrument, the rack,
    the table and the surgeon are built from boxes/capsules here. When the `hospital-sim-env/objects/mjcf/` assets
    land, `objects_assets()` picks them up per name (see OBJECTS_DIR) and the stand-in is dropped.
  * Instruments are free bodies (realistic mass, steel-on-glove friction 0.8, condim 4) standing tips-down in the
    rack, handle up. Object frame: origin at the working-end tip, +z along the instrument towards the handle.
    Sites: <name>_grasp (where the hand closes), <name>_handle (handle end, what the surgeon receives).
  * The tool rack is a free body too (1.2 kg): the robot can knock it, and the sim can re-place it with offsets.
  * The surgeon stand-in is static (lidar-visible, geom group 0) except its hand/forearm (visual only, so the
    instrument can be presented into it). Site `surgeon_hand` = where the handle should arrive. Each instrument has
    an inactive weld `grip_<name>` to body `surgeon_hand`: the sim's simulated "surgeon grip" (a CHEAT, see
    g1_mujoco_sim) activates it when the handle has dwelt at the hand.
"""
import glob
import json
import math
import os
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))   # hospital-sim root
OBJECTS_DIR = os.path.join(REPO, "hospital-sim-env", "objects", "mjcf")      # owned by hospital-sim-env

# Pelvis pose at the instrument table (world). Facing south (-y), towards the table; the surgeon is on its right.
STATION = {"x": 26.20, "y": -19.20, "yaw": -math.pi / 2}

TABLE = {"center": (0.60, -0.05), "half": (0.20, 0.40), "top_z": 0.75, "thickness": 0.025}  # station-local (fwd, left)
RACK = {"center": (0.45, -0.07), "pitch": 0.09, "plate_half": (0.05, 0.20, 0.006), "bar_h": 0.035,
        "slot_half": (0.0075, 0.0045), "wall": 0.002, "mass": 1.2}
# Surgeon stand-in: body across the table corner on the robot's right, facing the robot's side.
SURGEON = {"body": (0.40, -0.85), "yaw_face": math.pi / 2, "hand": (0.32, -0.42, 1.00)}   # hand site (fwd, left, z)

STEEL = "0.72 0.74 0.78 1"
# name: parts [(geom type, size, z-centre, rgba)], mass kg, length, grasp z, handle z, label colour.
# All in the object frame (tip at z=0). Foot = the part that sits in the rack slot (same for all: 6 x 12 mm).
FOOT = ("box", (0.003, 0.006, 0.02), 0.02)
INSTRUMENTS = {
    "scalpel_handle": {"parts": [FOOT, ("box", (0.004, 0.007, 0.06), 0.10)], "mass": 0.035, "length": 0.16,
                       "band": "0.9 0.2 0.2 1", "desc": "No. 3 scalpel handle with blade (stand-in)"},
    "forceps": {"parts": [FOOT, ("box", (0.0045, 0.008, 0.06), 0.10)], "mass": 0.04, "length": 0.16,
                "band": "0.2 0.7 0.3 1", "desc": "tissue forceps (stand-in)"},
    "needle_driver": {"parts": [FOOT, ("box", (0.004, 0.005, 0.05), 0.09), ("box", (0.004, 0.022, 0.02), 0.16)],
                      "mass": 0.06, "length": 0.18, "band": "0.2 0.4 0.9 1", "desc": "Mayo-Hegar needle driver (stand-in)"},
    "scissors": {"parts": [FOOT, ("box", (0.004, 0.006, 0.045), 0.085), ("box", (0.004, 0.022, 0.02), 0.15)],
                 "mass": 0.07, "length": 0.17, "band": "0.9 0.7 0.1 1", "desc": "Mayo scissors (stand-in)"},
}
GRASP_Z = 0.09           # object frame: where the hand's grasp centre goes (above the rack bars + hand half-width)
RACK_ORDER = ["scalpel_handle", "forceps", "needle_driver", "scissors"]   # slot 0 = robot's right (right hand side)
FRICTION = "0.8 0.02 0.001"


def to_world(fx, ly):
    c, s = math.cos(STATION["yaw"]), math.sin(STATION["yaw"])
    return STATION["x"] + fx * c - ly * s, STATION["y"] + fx * s + ly * c


def quat_z(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def fmt(v):
    return " ".join(f"{x:.5g}" for x in v)


def objects_assets():
    """MJCF assets available in hospital-sim-env/objects/mjcf/ (name -> path). Empty until they land."""
    if not os.path.isdir(OBJECTS_DIR):
        return {}
    return {os.path.splitext(os.path.basename(p))[0]: p for p in sorted(glob.glob(os.path.join(OBJECTS_DIR, "*.xml")))}


def rack_slot_local(i):
    """Rack-frame (x along the row, robot's left = +) position of slot i."""
    n = len(RACK_ORDER)
    return (i - (n - 1) / 2) * RACK["pitch"]


def layout():
    """World-frame poses of everything (also written to worlds/theatre_setup.json)."""
    yaw = STATION["yaw"]
    rack_yaw = yaw + math.pi / 2          # rack x axis = robot's left
    rc = to_world(*RACK["center"])
    plate_top = TABLE["top_z"] + 2 * RACK["plate_half"][2]
    inst = {}
    for i, name in enumerate(RACK_ORDER):
        ly = RACK["center"][1] + rack_slot_local(i)
        x, y = to_world(RACK["center"][0], ly)
        spec = INSTRUMENTS[name]
        # instrument frame: x = robot's left (thin side along the row), y = robot's forward... wide side faces the robot
        inst[name] = {"pos": [x, y, plate_top], "yaw": rack_yaw, "quat": list(quat_z(rack_yaw)), "slot": i,
                      "length": spec["length"], "grasp_z": GRASP_Z, "handle_z": spec["length"], "mass": spec["mass"],
                      "desc": spec["desc"], "standin": True, "approach_axis": "y"}
    hx, hy = to_world(SURGEON["hand"][0], SURGEON["hand"][1])
    bx, by = to_world(*SURGEON["body"])
    tx, ty = to_world(*TABLE["center"])
    return {
        "source": "standin",
        "station": dict(STATION),
        "handover_station": dict(STATION),
        "table": {"pos": [tx, ty, TABLE["top_z"]], "yaw": yaw, "half": list(TABLE["half"]), "standin": True},
        "rack": {"pos": [rc[0], rc[1], TABLE["top_z"]], "yaw": rack_yaw, "pitch": RACK["pitch"], "standin": True},
        "instruments": inst,
        "surgeon": {"body": [bx, by], "yaw": yaw + SURGEON["yaw_face"], "hand": [hx, hy, SURGEON["hand"][2]],
                    "hand_body": "surgeon_hand", "standin": True},
    }


def _geom(parent, **kw):
    return ET.SubElement(parent, "geom", {k: (fmt(v) if isinstance(v, (tuple, list)) else str(v)) for k, v in kw.items()})


def build(wb, asset, equality, L=None):
    """Append the set to a world MJCF (worldbody `wb`, `asset`, `equality` elements). Returns the layout dict."""
    L = L or layout()
    for name, rgba in (("inst_steel", STEEL), ("table_steel", "0.80 0.82 0.85 1"), ("rack_mat", "0.55 0.58 0.62 1"),
                       ("scrubs", "0.25 0.55 0.55 1"), ("skin_glove", "0.85 0.80 0.55 1")):
        ET.SubElement(asset, "material", {"name": name, "rgba": rgba})
    ET.SubElement(asset, "texture", {"name": "drape_tex", "type": "2d", "builtin": "flat", "rgb1": "0.25 0.45 0.65",
                                     "width": "16", "height": "16"})
    yaw = STATION["yaw"]
    # --- instrument table (static, lidar-visible) -------------------------------------------------------------
    t = L["table"]
    tb = ET.SubElement(wb, "body", {"name": "instrument_table", "pos": fmt((t["pos"][0], t["pos"][1], 0)),
                                    "quat": fmt(quat_z(yaw))})
    hx, hy = TABLE["half"]
    th = TABLE["thickness"]
    _geom(tb, name="instrument_table_top", type="box", size=(hx, hy, th / 2), pos=(0, 0, TABLE["top_z"] - th / 2),
          material="table_steel", group=0, friction="0.6 0.01 0.001")
    for sx in (-1, 1):
        for sy in (-1, 1):
            _geom(tb, name=f"instrument_table_leg_{'n' if sx > 0 else 's'}{'l' if sy > 0 else 'r'}", type="cylinder",
                  size=(0.015, (TABLE["top_z"] - th) / 2), pos=(sx * (hx - 0.03), sy * (hy - 0.03), (TABLE["top_z"] - th) / 2),
                  material="table_steel", group=0)
    ET.SubElement(tb, "site", {"name": "instrument_table_top", "pos": fmt((0, 0, TABLE["top_z"])), "size": "0.01",
                               "group": "5"})
    # --- tool rack (free body) ---------------------------------------------------------------------------------
    r = L["rack"]
    rb = ET.SubElement(wb, "body", {"name": "tool_rack", "pos": fmt(r["pos"]), "quat": fmt(quat_z(r["yaw"]))})
    ET.SubElement(rb, "freejoint", {"name": "tool_rack_free"})
    px, py, pz = RACK["plate_half"][1], RACK["plate_half"][0], RACK["plate_half"][2]   # rack x = along the row
    bh = RACK["bar_h"]
    sxh, syh = RACK["slot_half"][1], RACK["slot_half"][0]          # slot half along row (x), across (y)
    w = RACK["wall"]
    # geoms: plate + front/back bars + two side walls per slot; masses from density of steel-ish (total ~RACK mass)
    parts = [("plate", (px, py, pz), (0, 0, pz))]
    for sgn, nm in ((1, "back"), (-1, "front")):
        parts.append((f"bar_{nm}", (px, w, bh / 2), (0, sgn * (syh + w), 2 * pz + bh / 2)))
    for i in range(len(RACK_ORDER)):
        x0 = rack_slot_local(i)
        for sgn in (-1, 1):
            parts.append((f"wall_{i}_{'l' if sgn > 0 else 'r'}", (w, syh, bh / 2), (x0 + sgn * (sxh + w), 0, 2 * pz + bh / 2)))
    vol = sum(8 * s[0] * s[1] * s[2] for _, s, _ in parts)
    for nm, s, p in parts:
        _geom(rb, name=f"tool_rack_{nm}", type="box", size=s, pos=p, material="rack_mat", group=1,
              mass=RACK["mass"] * 8 * s[0] * s[1] * s[2] / vol, friction="0.6 0.01 0.001", condim=4)
    for i in range(len(RACK_ORDER)):
        ET.SubElement(rb, "site", {"name": f"tool_rack_slot{i}", "pos": fmt((rack_slot_local(i), 0, 2 * pz)),
                                   "size": "0.004", "group": "5"})
    # --- instruments (free bodies) -----------------------------------------------------------------------------
    for name, info in L["instruments"].items():
        spec = INSTRUMENTS[name]
        ib = ET.SubElement(wb, "body", {"name": name, "pos": fmt(info["pos"]), "quat": fmt(quat_z(info["yaw"]))})
        ET.SubElement(ib, "freejoint", {"name": f"{name}_free"})
        vols = [8 * s[0] * s[1] * s[2] for _, s, _ in spec["parts"]]
        for k, ((gt, s, zc), v) in enumerate(zip(spec["parts"], vols)):
            _geom(ib, name=f"{name}_g{k}", type=gt, size=s, pos=(0, 0, zc), material="inst_steel", group=1,
                  mass=spec["mass"] * v / sum(vols), friction=FRICTION, condim=4, solimp="0.95 0.99 0.001")
        # colour band just below the grasp zone (visual only) = the label a camera / human uses
        _geom(ib, name=f"{name}_band", type="box", size=(0.0048, 0.0085, 0.006), pos=(0, 0, 0.042), rgba=spec["band"],
              contype=0, conaffinity=0, group=1, mass=0)
        ET.SubElement(ib, "site", {"name": f"{name}_grasp", "pos": fmt((0, 0, info["grasp_z"])), "size": "0.004", "group": "5"})
        ET.SubElement(ib, "site", {"name": f"{name}_handle", "pos": fmt((0, 0, info["handle_z"])), "size": "0.004",
                                   "group": "5"})
    # --- surgeon stand-in (static) -----------------------------------------------------------------------------
    s = L["surgeon"]
    sb = ET.SubElement(wb, "body", {"name": "surgeon_standin", "pos": fmt((s["body"][0], s["body"][1], 0)),
                                    "quat": fmt(quat_z(s["yaw"]))})
    _geom(sb, name="surgeon_legs", type="box", size=(0.11, 0.17, 0.45), pos=(0, 0, 0.45), material="scrubs", group=0)
    _geom(sb, name="surgeon_torso", type="capsule", size=(0.16, 0.20), pos=(0, 0, 1.18), material="scrubs", group=0)
    _geom(sb, name="surgeon_head", type="sphere", size=(0.10,), pos=(0, 0, 1.68), rgba="0.3 0.6 0.6 1", group=0)
    # right upper arm hanging, forearm reaching towards the presented handle (visual only)
    hand_w = s["hand"]
    c, sn = math.cos(-s["yaw"]), math.sin(-s["yaw"])
    dx, dy = hand_w[0] - s["body"][0], hand_w[1] - s["body"][1]
    hand_b = (c * dx - sn * dy, sn * dx + c * dy, hand_w[2])        # hand in surgeon-body frame
    elbow = (0.05, hand_b[1] * 0.6, 1.05)
    _geom(sb, name="surgeon_forearm", type="capsule", fromto=(*elbow, *hand_b), size=(0.035,), material="scrubs",
          contype=0, conaffinity=0, group=1)
    _geom(sb, name="surgeon_upperarm", type="capsule", fromto=(0.0, hand_b[1] * 0.6, 1.38, *elbow), size=(0.04,),
          material="scrubs", contype=0, conaffinity=0, group=1)
    hb = ET.SubElement(wb, "body", {"name": "surgeon_hand", "pos": fmt(hand_w), "quat": fmt(quat_z(s["yaw"]))})
    _geom(hb, name="surgeon_hand_palm", type="box", size=(0.045, 0.012, 0.04), pos=(0.02, 0, -0.035),
          material="skin_glove", contype=0, conaffinity=0, group=1)
    ET.SubElement(hb, "site", {"name": "surgeon_hand", "pos": "0 0 0", "size": "0.015", "rgba": "1 0.3 0.3 0.5",
                               "group": "5"})
    # --- simulated surgeon grip: one inactive weld per instrument ----------------------------------------------
    for name in L["instruments"]:
        ET.SubElement(equality, "weld", {"name": f"grip_{name}", "body1": "surgeon_hand", "body2": name,
                                         "active": "false", "solref": "0.004 1", "solimp": "0.99 0.999 0.001"})
    return L


def look_at(eye, target):
    """MuJoCo camera xyaxes for a camera at `eye` looking at `target` (world, z up)."""
    import numpy as np
    f = np.array(target, float) - np.array(eye, float)
    f /= np.linalg.norm(f)
    x = np.cross(f, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    return fmt(list(x) + list(y))


def add_cameras(wb):
    """Fixed viewing cameras on the scrub station (videos, dashboard)."""
    views = {"theatre_station": ((1.35, 0.75, 1.75), (0.30, -0.30, 0.95), 60),   # front-left, whole set
             "theatre_closeup": ((1.15, -0.30, 1.45), (0.35, -0.30, 0.95), 55)}   # across the table, rack + handover
    for name, (e, t, fovy) in views.items():
        ex, ey = to_world(e[0], e[1])
        tx, ty = to_world(t[0], t[1])
        ET.SubElement(wb, "camera", {"name": name, "pos": fmt((ex, ey, e[2])),
                                     "xyaxes": look_at((ex, ey, e[2]), (tx, ty, t[2])), "fovy": str(fovy)})


def write_setup(path, L=None):
    L = L or layout()
    L = dict(L)
    L["note"] = ("Generated by g1-mujoco-sim/build_world.py (theatre.py). World frame = Isaac world. source=standin: "
                 "primitive stand-ins of this file; source=objects: hospital-sim-env/objects assets at its catalog.json poses. "
                 "Instrument frame: tip at origin, +z to the handle; grasp_z / handle_z along +z; approach_axis = "
                 "the instrument axis whose side faces the robot in the rack.")
    with open(path, "w") as f:
        json.dump(L, f, indent=1)
    return L


# ================================================================================================================
# objects mode: the hospital-sim-env/objects assets (mjcf/*.xml, catalog.json, placements.yaml)
# ================================================================================================================
OBJ_ROOT = os.path.join(REPO, "hospital-sim-env", "objects")
# Which instruments the right hand serves from the table stance (rack slots 0-3; slots 4-6 need the left hand or a
# sideways step, see hospital-sim-env/objects/README.md "Reach"). All seven are placed in the world regardless.
OBJ_STANCE_BACK = 0.40      # m: pelvis to rack row at the table stance (objects README suggests 0.33; with the policy's
                            # default arms, fingertips reach 0.42 m forward, so 0.33 m collides with the rings)
OBJ_STANCE_X = 25.58        # m: pelvis x at the table stance (right shoulder centred on slots 0-3)
REMOVE_BOXES = ("sm_surgicalchair_a01_01", "sm_surgicalchair_a01_02")   # hospital-sim-env/objects/placements.yaml remove_scene_props


def objects_available():
    return (os.path.exists(os.path.join(OBJ_ROOT, "catalog.json"))
            and os.path.exists(os.path.join(OBJ_ROOT, "mjcf", "surgeon.xml"))
            and os.path.exists(os.path.join(OBJ_ROOT, "mjcf", "tool_rack.xml")))


def _yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def objects_layout():
    """Layout dict (same schema as layout()) from hospital-sim-env/objects/catalog.json."""
    cat = json.load(open(os.path.join(OBJ_ROOT, "catalog.json")))
    wp = cat["world_poses"]
    guide_top = cat["rack"]["guide_top"] - cat["rack"]["base_top"]       # guide plate top above the tip pockets
    inst = {}
    rack = wp["tool_rack"]
    rack_yaw = _yaw_of(rack["quat"])
    for name, e in cat["instruments"].items():
        w = wp[name]
        lo, hi = e["grasp"]["z_range"]
        # power-grasp centre: the hand spans about +-45 mm along the instrument; keep its lower edge above the rack
        # guide plate (tip at z=0 sits 8.5 mm above the rack base) and stay inside the grasp region if possible
        g = max(e["grasp"]["site"][2], lo + 0.03, guide_top + 0.05)
        g = min(g, hi - 0.005) if hi - 0.005 > guide_top + 0.045 else g
        slot = cat["rack"]["slots"][name]["index"]
        inst[name] = {"pos": list(w["pos"]), "quat": list(w["quat"]), "yaw": _yaw_of(w["quat"]), "slot": slot,
                      "length": e["bbox_max"][2], "grasp_z": round(g, 4), "handle_z": round(e["bbox_max"][2], 4),
                      "grasp_range": [lo, hi], "mass": e["mass_kg"], "desc": e["title"], "standin": e["stand_in"],
                      "approach_axis": "x"}
    st = cat["robot_stances"]["theatre_instrument_table"]
    rack_y = rack["pos"][1]
    station = {"x": OBJ_STANCE_X, "y": round(rack_y + OBJ_STANCE_BACK, 4), "yaw": st[2]}
    ss = cat["robot_stances"]["theatre_surgeon_side"]
    tb, sg = wp["instrument_table"], wp["surgeon"]
    return {
        "source": "objects",
        "station": station,
        "handover_station": {"x": ss[0], "y": ss[1], "yaw": ss[2]},
        "table": {"pos": tb["pos"], "yaw": _yaw_of(tb["quat"]), "top_z": cat["table"]["top"], "standin": False},
        "rack": {"pos": rack["pos"], "yaw": rack_yaw, "pitch": cat["rack"]["pitch"], "static": True, "standin": False},
        "instruments": inst,
        "surgeon": {"body": sg["pos"][:2], "yaw": _yaw_of(sg["quat"]), "hand": wp["surgeon_hand"]["pos"],
                    "hand_body": "surgeon", "standin": False},
        "remove_boxes": list(REMOVE_BOXES),
        "catalog_generated_by": cat.get("generated_by", ""),
    }


def objects_model_assets(scene_dir):
    """<model name file> entries for hospital-sim-env/objects/mjcf/*.xml, paths relative to the MAIN scene file's directory. They go
    in the scene file itself: <model file> paths inside an <include>d file resolved inconsistently (depending on
    the working directory) with MuJoCo 3.4."""
    L = objects_layout()
    names = ["instrument_table", "tool_rack", "surgeon"] + list(L["instruments"])
    return [(n, os.path.relpath(os.path.join(OBJ_ROOT, "mjcf", f"{n}.xml"), scene_dir)) for n in names]


def build_objects(root, wb, asset, equality, scene_dir=None):
    """Attach hospital-sim-env/objects/mjcf/<name>.xml at their catalog world poses (MuJoCo <attach>; the <model> assets are declared
    in the scene file, see objects_model_assets), add the simulated grip welds (surgeon body <-> each instrument)
    and return the layout."""
    L = objects_layout()
    cat = json.load(open(os.path.join(OBJ_ROOT, "catalog.json")))
    names = ["instrument_table", "tool_rack", "surgeon"] + list(L["instruments"])
    for n in names:
        w = cat["world_poses"][n]
        fr = ET.SubElement(wb, "frame", {"pos": fmt(w["pos"]), "quat": fmt(w["quat"])})
        ET.SubElement(fr, "attach", {"model": n, "body": n, "prefix": ""})
    for n in L["instruments"]:
        ET.SubElement(equality, "weld", {"name": f"grip_{n}", "body1": "surgeon", "body2": n, "active": "false",
                                         "solref": "0.004 1", "solimp": "0.99 0.999 0.001"})
    return L


def add_cameras_at(wb, station, handover):
    """Fixed viewing cameras for the objects layout: the table stance and the handover stance."""
    def at(st, fwd, left, z):
        c, s = math.cos(st["yaw"]), math.sin(st["yaw"])
        return st["x"] + fwd * c - left * s, st["y"] + fwd * s + left * c, z
    views = {"theatre_station": (at(station, 1.35, 0.75, 1.75), at(station, 0.30, -0.10, 0.95), 60),
             "theatre_closeup": (at(station, 1.20, -0.25, 1.45), at(station, 0.35, -0.15, 0.95), 55),
             "theatre_handover": (at(handover, 0.25, -1.00, 2.00), at(handover, 0.40, -0.12, 1.00), 60)}
    for name, (e, t, fovy) in views.items():
        ET.SubElement(wb, "camera", {"name": name, "pos": fmt(e), "xyaxes": look_at(e, t), "fovy": str(fovy)})
