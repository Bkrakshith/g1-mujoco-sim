"""Generate the MuJoCo hospital world (and a flat-ground world) for the G1.

    python g1-mujoco-sim/build_world.py            # any python with numpy; no MuJoCo/USD needed to write XML
    python g1-mujoco-sim/build_world.py --check    # also compile both scenes with mujoco and print stats
    (run from the hospital-sim root, which holds the sibling repos hospital-sim-env and g1-brainco-description)

Inputs
  hospital-sim-env/isaac/scene/layout.py   wing rooms, doorways, WING_ORIGIN (walls are built analytically)
  worlds/hospital_boxes.json         building walls + furniture AABBs extracted from the Isaac stage
                                     (refresh with extract_scene_boxes.py after the USD scene changes)
  robot MJCF                         g1-brainco-description/g1_29dof_brainco/mjcf/g1_29dof_brainco.xml, or the marked
                                     fallback (unitree_rl_gym g1_29dof_rev_1_0.xml) while it does not exist

Outputs (g1-mujoco-sim/)
  worlds/hospital_world.xml   world-only MJCF (floor, walls with door openings, furniture boxes)
  worlds/flat_world.xml       world-only MJCF (checker floor)
  worlds/spawns.json          named spawn poses (world frame) per world
  worlds/theatre_setup.json   scrub-nurse set in the theatre (theatre.py): instrument table, tool rack, instruments,
                              surgeon stand-in; world poses + instrument metadata (read by g1_mission/scripts)
  g1_robot.xml               cleaned copy of the robot MJCF (generated; robot_source.txt says which)
  hospital_g1.xml, flat_g1.xml  scenes = robot + world (load these)

Frame: identical to the Isaac scene (hospital-sim-env/isaac/usd/hospital_g1.usd) world frame; the wing origin is
layout.WING_ORIGIN = (17.93, -4.45, 0). Static world geoms are in geom group 0 (the simulated
lidar only sees group 0; the robot MJCF uses groups 2 visual, 3 collision, 4 sites).
"""
import argparse
import importlib.util
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))   # hospital-sim root (sibling repos live next to this one)
WORLDS = os.path.join(HERE, "worlds")
sys.path.insert(0, HERE)
import theatre  # noqa: E402

ROBOT_MJCF = os.path.join(REPO, "g1-brainco-description", "g1_29dof_brainco", "mjcf", "g1_29dof_brainco.xml")
# Optional fallback: a checkout of unitreerobotics/unitree_rl_gym, given by UNITREE_RL_GYM.
FALLBACK_MJCF = os.path.join(
    os.environ.get("UNITREE_RL_GYM", os.path.expanduser("~/unitree_rl_gym")),
    "resources", "robots", "g1_description", "g1_29dof_rev_1_0.xml")

WALL_H = 3.0
WALL_GROUP = 0


def load_layout():
    spec = importlib.util.spec_from_file_location("layout", os.path.join(REPO, "hospital-sim-env", "isaac", "scene", "layout.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _subtract(intervals, holes):
    out = []
    for a, b in intervals:
        segs = [(a, b)]
        for ha, hb in holes:
            nxt = []
            for s0, s1 in segs:
                if hb <= s0 or ha >= s1:
                    nxt.append((s0, s1))
                    continue
                if ha - s0 > 1e-6:
                    nxt.append((s0, ha))
                if s1 - hb > 1e-6:
                    nxt.append((hb, s1))
            segs = nxt
        out += segs
    return [s for s in out if s[1] - s[0] > 0.02]


def wing_walls(L):
    """Kit walls: WALL_T thick, inside each room's cell, with every doorway cut out. Wing-local boxes."""
    rooms = dict(L.KIT_ROOMS)
    rooms.update(L.SUITE_ROOMS)
    doors = list(L.DOORWAYS.values()) + list(L.OPENINGS)
    t = L.WALL_T
    boxes = []
    for name, (x0, y0, x1, y1) in rooms.items():
        # (fixed coordinate, axis, lo, hi, inward sign)
        edges = [("S", y0, "x", x0, x1, +1), ("N", y1, "x", x0, x1, -1),
                 ("W", x0, "y", y0, y1, +1), ("E", x1, "y", y0, y1, -1)]
        for side, c, axis, lo, hi, sgn in edges:
            holes = []
            for dx0, dy0, dx1, dy1 in doors:
                if axis == "x" and dy0 == dy1 and abs(dy0 - c) < 1e-6:
                    holes.append((min(dx0, dx1), max(dx0, dx1)))
                if axis == "y" and dx0 == dx1 and abs(dx0 - c) < 1e-6:
                    holes.append((min(dy0, dy1), max(dy0, dy1)))
            for a, b in _subtract([(lo, hi)], holes):
                c0, c1 = sorted((c, c + sgn * t))
                if axis == "x":
                    boxes.append((f"{name}_{side}_{a:g}", [a, c0, 0.0, b, c1, WALL_H]))
                else:
                    boxes.append((f"{name}_{side}_{a:g}", [c0, a, 0.0, c1, b, WALL_H]))
    return boxes


def cut_doorways(box, L, ox, oy):
    """Remove the clear door zone of every wing doorway from an (extracted) building wall box."""
    out = [box]
    for dx0, dy0, dx1, dy1 in L.DOORWAYS.values():
        if dy0 == dy1:
            zone = (min(dx0, dx1) + ox, dy0 + oy - 0.3, max(dx0, dx1) + ox, dy0 + oy + 0.3)
        else:
            zone = (dx0 + ox - 0.3, min(dy0, dy1) + oy, dx0 + ox + 0.3, max(dy0, dy1) + oy)
        nxt = []
        for b in out:
            if not (b[0] < zone[2] and b[3] > zone[0] and b[1] < zone[3] and b[4] > zone[1]):
                nxt.append(b)
                continue
            along = 0 if (b[3] - b[0]) >= (b[4] - b[1]) else 1
            for a, c in _subtract([(b[along], b[along + 3])], [(zone[along], zone[along + 2])]):
                nb = list(b)
                nb[along], nb[along + 3] = a, c
                nxt.append(nb)
        out = nxt
    return out


def box_geom(parent, name, b, material, cls="world"):
    x0, y0, z0, x1, y1, z1 = b
    z0 = max(z0, 0.0)
    if x1 - x0 < 1e-3 or y1 - y0 < 1e-3 or z1 - z0 < 1e-3:
        return
    ET.SubElement(parent, "geom", {
        "name": name, "class": cls, "type": "box", "material": material,
        "pos": f"{(x0 + x1) / 2:.4f} {(y0 + y1) / 2:.4f} {(z0 + z1) / 2:.4f}",
        "size": f"{(x1 - x0) / 2:.4f} {(y1 - y0) / 2:.4f} {(z1 - z0) / 2:.4f}"})


def world_root(model):
    root = ET.Element("mujoco", {"model": model})
    ET.SubElement(root, "compiler", {"angle": "radian"})
    default = ET.SubElement(root, "default")
    w = ET.SubElement(default, "default", {"class": "world"})
    ET.SubElement(w, "geom", {"group": str(WALL_GROUP), "contype": "1", "conaffinity": "1",
                              "condim": "3", "friction": "1 0.005 0.0001"})
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", {"type": "skybox", "builtin": "gradient", "rgb1": "0.3 0.5 0.7",
                                     "rgb2": "0 0 0", "width": "512", "height": "3072"})
    ET.SubElement(asset, "texture", {"type": "2d", "name": "floortex", "builtin": "checker",
                                     "rgb1": "0.80 0.80 0.76", "rgb2": "0.72 0.72 0.68",
                                     "width": "512", "height": "512"})
    ET.SubElement(asset, "material", {"name": "floor", "texture": "floortex", "texuniform": "true",
                                      "texrepeat": "1 1", "reflectance": "0", "specular": "0", "shininess": "0"})
    for name, rgba in (("wing_wall", "0.88 0.90 0.92 1"), ("building_wall", "0.86 0.82 0.74 1"),
                       ("furniture", "0.35 0.55 0.75 1"), ("suite", "0.45 0.70 0.60 1"),
                       ("building_obj", "0.65 0.55 0.45 1")):
        ET.SubElement(asset, "material", {"name": name, "rgba": rgba})
    wb = ET.SubElement(root, "worldbody")
    return root, wb


def write(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="unicode")


def theatre_mode(req):
    if req == "auto":
        return "objects" if theatre.objects_available() else "standin"
    if req == "objects" and not theatre.objects_available():
        raise SystemExit("--theatre objects: hospital-sim-env/objects/mjcf/*.xml + catalog.json not found (run hospital-sim-env/objects/build.sh)")
    return req


def build_hospital(L, mode="standin"):
    ox, oy, _ = L.WING_ORIGIN
    root, wb = world_root("hospital_world")
    ET.SubElement(wb, "light", {"name": "sun", "pos": f"{ox} {oy} 30", "dir": "0 0.2 -1",
                                "directional": "true", "diffuse": "0.7 0.7 0.7", "castshadow": "false"})
    ET.SubElement(wb, "geom", {"name": "world_floor", "class": "world", "type": "plane", "size": "0 0 0.05",
                               "material": "floor"})
    n = {"wing_wall": 0, "building_wall": 0, "furniture": 0}
    for name, b in wing_walls(L):
        b = [b[0] + ox, b[1] + oy, b[2], b[3] + ox, b[4] + oy, b[5]]
        box_geom(wb, f"wing_{name}", b, "wing_wall")
        n["wing_wall"] += 1
    with open(os.path.join(WORLDS, "hospital_boxes.json")) as f:
        extracted = json.load(f)["boxes"]
    skip = set(theatre.REMOVE_BOXES) if mode == "objects" else set()
    for i, it in enumerate(extracted):
        cat = it["cat"]
        if it["name"] in skip:          # props replaced by the theatre set (hospital-sim-env/objects/placements.yaml)
            continue
        if cat == "building_wall":
            for j, piece in enumerate(cut_doorways(it["box"], L, ox, oy)):
                box_geom(wb, f"b{i}_{j}_{it['name']}", piece, "building_wall")
                n["building_wall"] += 1
        else:
            mat = {"building": "building_obj", "dressing": "furniture", "suite": "suite"}[cat]
            box_geom(wb, f"f{i}_{it['name']}", it["box"], mat)
            n["furniture"] += 1
    # scrub-nurse set in the theatre (instrument table, tool rack + instruments, surgeon)
    asset = root.find("asset")
    equality = ET.SubElement(root, "equality")
    if mode == "objects":            # the perception agent's assets (hospital-sim-env/objects/), at its catalog.json poses
        TL = theatre.build_objects(root, wb, asset, equality)
        theatre.add_cameras_at(wb, TL["station"], TL["handover_station"])
    elif mode == "standin":          # primitive stand-ins of theatre.py
        TL = theatre.build(wb, asset, equality)
        theatre.add_cameras(wb)
    else:
        TL = None
    if TL is not None:
        theatre.write_setup(os.path.join(WORLDS, "theatre_setup.json"), TL)
        n["theatre_set"] = f"{mode}: {len(TL['instruments'])} instruments"
    write(root, os.path.join(WORLDS, "hospital_world.xml"))
    return n


def build_flat():
    root, wb = world_root("flat_world")
    ET.SubElement(wb, "light", {"name": "sun", "pos": "0 0 20", "dir": "0 0.2 -1", "directional": "true",
                                "diffuse": "0.7 0.7 0.7", "castshadow": "false"})
    ET.SubElement(wb, "geom", {"name": "world_floor", "class": "world", "type": "plane", "size": "0 0 0.05",
                               "material": "floor"})
    write(root, os.path.join(WORLDS, "flat_world.xml"))


def spawns(L):
    ox, oy, _ = L.WING_ORIGIN

    def w(x, y, yaw_deg):
        return {"x": round(x + ox, 4), "y": round(y + oy, 4), "yaw": round(math.radians(yaw_deg), 6)}
    return {
        "hospital": {
            "default": "corridor_west",
            # Surgical corridor, west end, facing east: 20+ m of 4 m-wide corridor ahead (best for demos).
            "corridor_west": w(-9.0, -8.0, 0),
            # Link corridor just inside the SE door, facing south into the wing.
            "link": w(1.0, -1.2, -90),
            # Hospital SE lobby, 2.5 m north of the SE door, facing the door (south).
            "lobby": w(1.0, 2.5, -90),
            # Operating theatre, lined up at the instrument table (fixed_base arm work); filled in by main()
            # from worlds/theatre_setup.json "station" (stand-in or hospital-sim-env/objects layout).
            "theatre_table": {"x": theatre.STATION["x"], "y": theatre.STATION["y"],
                              "yaw": round(theatre.STATION["yaw"], 6)},
        },
        "flat": {"default": "origin", "origin": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
    }


def root_rel(path):
    """Path relative to the hospital-sim root for files inside it (what generated files record); else unchanged."""
    rel = os.path.relpath(path, REPO)
    return path if rel.startswith("..") else rel


def robot_path(explicit=None):
    if explicit:
        return os.path.abspath(explicit), False
    if os.path.exists(ROBOT_MJCF):
        return ROBOT_MJCF, False
    return FALLBACK_MJCF, True


def robot_copy(robot, fallback):
    """Write g1_robot.xml: the robot MJCF with world-level geoms/lights and skybox removed (robot files often
    ship their own floor), and relative asset paths re-based on g1-mujoco-sim/. Regenerated on every build."""
    tree = ET.parse(robot)
    r = tree.getroot()
    rdir = os.path.dirname(robot)
    comp = {}
    for c in r.findall("compiler"):
        comp.update(c.attrib)
        r.remove(c)
    new_comp = {k: v for k, v in comp.items() if k not in ("meshdir", "texturedir", "assetdir")}
    for key in ("meshdir", "texturedir", "assetdir"):
        if key in comp:
            p = comp[key] if os.path.isabs(comp[key]) else os.path.join(rdir, comp[key])
            new_comp[key] = os.path.relpath(p, HERE)
    if not any(k in comp for k in ("meshdir", "assetdir")):
        new_comp["meshdir"] = os.path.relpath(rdir, HERE)
    r.insert(0, ET.Element("compiler", new_comp))
    for inc in r.iter("include"):
        if not os.path.isabs(inc.get("file")):
            inc.set("file", os.path.relpath(os.path.join(rdir, inc.get("file")), HERE))
    for wb in r.findall("worldbody"):
        for el in list(wb):
            if el.tag in ("geom", "light"):
                wb.remove(el)
    for asset in r.findall("asset"):
        for el in list(asset):
            if el.tag == "texture" and el.get("type") == "skybox":
                asset.remove(el)
    for tag in ("visual", "statistic"):
        for el in r.findall(tag):
            r.remove(el)
    r.insert(0, ET.Comment(f" GENERATED by build_world.py from {root_rel(robot)} -- do not edit. "
                           + ("FALLBACK ROBOT (unitree_rl_gym, no BrainCo hands): re-run build_world.py once "
                              "g1-brainco-description/g1_29dof_brainco/mjcf/g1_29dof_brainco.xml exists. " if fallback else "")))
    write(r, os.path.join(HERE, "g1_robot.xml"))


def scene(world_xml, out_name, models=()):
    root = ET.Element("mujoco", {"model": out_name.replace(".xml", "")})
    if models:        # hospital-sim-env/objects assets attached by the world file (theatre.objects_model_assets)
        asset = ET.SubElement(root, "asset")
        for name, path in models:
            ET.SubElement(asset, "model", {"name": name, "file": path})
    ET.SubElement(root, "include", {"file": "g1_robot.xml"})
    ET.SubElement(root, "include", {"file": os.path.relpath(world_xml, HERE)})
    # elliptic friction cones + impratio 10: stable finger grasps of 16-85 g instruments (measured: instrument
    # jitter in the hand 4.6 -> 1.6 cm/s RMS); hospital-sim-env/objects/README.md asks for the same
    ET.SubElement(root, "option", {"timestep": "0.002", "integrator": "implicitfast", "cone": "elliptic",
                                   "impratio": "10"})
    ET.SubElement(root, "size", {"memory": "64M"})
    vis = ET.SubElement(root, "visual")
    ET.SubElement(vis, "headlight", {"diffuse": "0.6 0.6 0.6", "ambient": "0.3 0.3 0.3", "specular": "0 0 0"})
    ET.SubElement(vis, "global", {"offwidth": "1280", "offheight": "720", "azimuth": "-120", "elevation": "-20"})
    ET.SubElement(vis, "quality", {"shadowsize": "0"})
    ET.SubElement(vis, "map", {"znear": "0.002", "zfar": "5"})
    ET.SubElement(root, "statistic", {"extent": "20", "center": "0 0 1"})
    # fixed_base (sim param): pelvis welded to the world at its spawn/stand pose, for arm-skill development while
    # the placeholder policy cannot take large reaches. The sim node fills in the pose and activates it.
    eq = ET.SubElement(root, "equality")
    ET.SubElement(eq, "weld", {"name": "fixed_base", "body1": "pelvis", "active": "false", "solref": "0.005 1"})
    write(root, os.path.join(HERE, out_name))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", help="robot MJCF (default: g1-brainco-description canonical model, else fallback)")
    ap.add_argument("--check", action="store_true", help="compile scenes with mujoco")
    ap.add_argument("--theatre", default="auto", choices=["auto", "objects", "standin", "none"],
                    help="scrub-nurse set: hospital-sim-env/objects assets (auto when present), theatre.py stand-ins, or none")
    a = ap.parse_args()
    L = load_layout()
    os.makedirs(WORLDS, exist_ok=True)
    mode = theatre_mode(a.theatre)
    n = build_hospital(L, mode)
    build_flat()
    sp = spawns(L)
    setup_path = os.path.join(WORLDS, "theatre_setup.json")
    if mode != "none" and os.path.exists(setup_path):
        TL = json.load(open(setup_path))
        # fixed-base spawn 0.12 m behind the table stance: with the policy's default arms (fingertips 0.42 m ahead)
        # the robot would otherwise stand up with its fingers in the rack; the mission raises the arms and moves in
        st = TL["station"]
        sp["hospital"]["theatre_table"] = {"x": round(st["x"] - 0.12 * math.cos(st["yaw"]), 4),
                                           "y": round(st["y"] - 0.12 * math.sin(st["yaw"]), 4),
                                           "yaw": round(st["yaw"], 6)}
        sp["hospital"]["theatre_handover"] = {k: round(TL["handover_station"][k], 6) for k in ("x", "y", "yaw")}
    with open(os.path.join(WORLDS, "spawns.json"), "w") as f:
        json.dump(sp, f, indent=1)
    robot, fallback = robot_path(a.robot)
    robot_copy(robot, fallback)
    scene(os.path.join(WORLDS, "hospital_world.xml"), "hospital_g1.xml",
          theatre.objects_model_assets(HERE) if mode == "objects" else ())
    scene(os.path.join(WORLDS, "flat_world.xml"), "flat_g1.xml")
    with open(os.path.join(HERE, "robot_source.txt"), "w") as f:
        f.write(f"{root_rel(robot)}\nfallback={fallback}\n")
    print(f"world geoms: {n}; robot: {root_rel(robot)}{'  (FALLBACK)' if fallback else ''}")
    if a.check:
        import mujoco
        for s in ("hospital_g1.xml", "flat_g1.xml"):
            m = mujoco.MjModel.from_xml_path(os.path.join(HERE, s))
            print(f"{s}: nq={m.nq} nv={m.nv} nu={m.nu} ngeom={m.ngeom} nbody={m.nbody}")


if __name__ == "__main__":
    main()
