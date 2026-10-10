# g1-mujoco-sim: MuJoCo hospital world for the G1

The MuJoCo version of the Isaac hospital (`hospital-sim-env/isaac/usd/hospital_g1.usd`), **in the same world frame**, with the G1 29-DoF
and BrainCo Revo2 hands from `g1-brainco-description`. `g1_mujoco_sim` (in `hospital-sim-ros2`) runs it behind the real robot's
low-level interface.

Two sibling repos have to be checked out next to this one: `hospital-sim-env` for the room layout, the
extracted furniture boxes and the surgical objects, and `g1-brainco-description` for the robot MJCF.
`hospital-sim-ros2` is the consumer. The master repo `hospital-sim` arranges all of that and has the `env.sh`
that activates the `.venv` with mujoco. Paths inside this repo are relative to its root; the commands below
run from the master checkout.

## Files

| File | What |
|---|---|
| `build_world.py` | Generates every XML below from `hospital-sim-env/isaac/scene/layout.py` and `worlds/hospital_boxes.json`. `--check` compiles the scenes. `--theatre auto\|objects\|standin\|none` picks the scrub-nurse set (below) |
| `theatre.py` | The scrub-nurse set in the operating theatre: the `hospital-sim-env/objects` assets at their `catalog.json` poses, or primitive stand-ins |
| `worlds/theatre_setup.json` | Generated: the set's world poses, stances (`station` = instrument table, `handover_station` = surgeon side), and per-instrument grasp / handle / approach data. `g1_mission/scripts/make_locations.py` turns it into the mission config |
| `extract_scene_boxes.py` | Pulls world-frame AABBs of the Isaac building walls, furniture and OR-suite props out of the USD stage into `worlds/hospital_boxes.json`. Needs `usd-core`, so run it with `uv` |
| `hospital_g1.xml`, `flat_g1.xml` | **The scenes to load**: robot plus world, `timestep 0.002`, `implicitfast` |
| `g1_robot.xml` | Generated copy of the robot MJCF: world-level geoms/lights stripped, mesh paths rebased. `robot_source.txt` records which robot file it came from |
| `worlds/hospital_world.xml`, `worlds/flat_world.xml` | World-only MJCF, included by the scenes |
| `worlds/spawns.json` | Named spawn poses (world frame) |
| `render_replay.py` | Renders a recorded run (`sim.launch.py record:=run.npz`) offscreen to MP4 |
| `media/g1_hospital_walk.mp4` | Stand-up, 12 s keyboard walk down the surgical corridor, stop, MANIP arms/hands, blend-out, e-stop (0.4 MB) |
| `media/e2e_report_hospital.json` | The end-to-end check report for that run |
| `media/g1_scrub_nurse_fixed_base.mp4` | Fixed-base scrub-nurse run through the full ROS stack. Pick from the rack, carry, fixed-base glide to the surgeon, handle-first handover, back to the table |

## Build

```bash
source env.sh
python g1-mujoco-sim/build_world.py --check                                        # regenerate XML (seconds)
uv run --no-project --with usd-core python g1-mujoco-sim/extract_scene_boxes.py    # only after the USD scene changes
```

`build_world.py` uses `g1-brainco-description/g1_29dof_brainco/mjcf/g1_29dof_brainco.xml`. If that file is missing, it falls back to
unitree_rl_gym's `g1_29dof_rev_1_0.xml`, which has no hands, and marks the generated files as a fallback. Re-run
it whenever the robot MJCF changes.

## What is in the world

- Wing (from `layout.py`, exact): every room of `KIT_ROOMS` and `SUITE_ROOMS` gets 0.10 m walls inside its
  cell, like the kit, 3 m high. Every opening in `layout.DOORWAYS` / `OPENINGS` is cut out (2 m clear).
- Isaac hospital building (from the USD, approximate): ground-floor walls, cut at every door frame, plus
  furniture and stairs, as axis-aligned boxes. The building is grid-aligned, so boxes are close. Door leaves are
  left out (doors are open). Floors, ceilings, trims and lights are skipped.
- Furniture: layout dressing (87 items) and OR-suite props (104). Items are skipped when they are smaller than
  6 cm, flatter than 12 cm, or have their bottom above 1.4 m (ceiling lights, high wall signs).
- Floor: one plane at z = 0.

Totals: 41 wing walls, 355 building walls, 439 furniture boxes. All static world geoms are in **geom group 0**.
The robot MJCF uses 2 (visual), 3 (collision) and 4 (sites). The simulated lidar only sees group 0.

## Scrub-nurse set (theatre)

`build_world.py --theatre auto` (the default) uses the `hospital-sim-env/objects` assets when its `mjcf/*.xml` and
`catalog.json` exist. Otherwise it falls back to the primitive stand-ins in `theatre.py`.

**objects mode** (current):

- The objects are attached with MuJoCo `<model>`/`<attach>` at the `hospital-sim-env/objects/catalog.json` world poses, which come
  from `hospital-sim-env/objects/placements.yaml`:
  - 7 instruments as free bodies
  - the static 7-slot tool rack on a Mayo-style instrument table
  - the static surgeon mannequin with the `surgeon_hand` site
- The two OR-suite surgical stools the surgeon replaces are left out of the box world.
- The `<model>` assets are declared in `hospital_g1.xml` itself. With MuJoCo 3.4, `<model file>` paths inside an
  `<include>`d file resolved differently depending on the working directory.

**Stances** (mission `config/locations.yaml`):

- `theatre_instrument_table` is (25.58, -20.12, facing south), 0.40 m from the rack row. hospital-sim-env/objects suggests
  0.33 m, but with the policy's default arms the fingertips reach 0.42 m forward and would sit in the rack.
- `theatre_surgeon_side` is the hospital-sim-env/objects stance (23.80, -18.25, facing south). The surgeon's hand is 0.45 m ahead
  and 0.20 m to the right.
- The two stances are 2.6 m apart, so the mission carries the instrument between them. In fixed-base mode it
  glides (below).
- Spawn `theatre_table` (fixed base) is 0.12 m behind the table stance. The robot stands up there, raises its
  arms, and FineAlign moves it in.

**stand-in mode**:

- A table, a free 4-slot rack and 4 box instruments (`scalpel_handle`, `forceps`, `needle_driver`, `scissors`).
- A surgeon stand-in whose hand is reachable from the table stance.
- Use `--theatre standin`.

**Sim-only extras** (all in `g1_mujoco_sim`, **ground truth or cheats**, see `g1_mission/README.md`):

- `/sim/objects` publishes JSON at 10 Hz: `{name: {"pose": [x,y,z,qw,qx,qy,qz], "type": "instrument" | "rack" |
  "surgeon_hand", ...}}`. It covers every free object, the rack (`"static": true` if it is static) and the
  surgeon's hand. Instruments also carry `"held_by": "surgeon" | null`, and the hand carries
  `"holding": <name> | null`.
- `/sim/reset_objects` (`std_srvs/Trigger`) puts every object back. The parameter `object_offset`
  `[dx, dy, dyaw]` shifts the rack and its instruments rigidly for trials. A static rack moves through
  `body_pos`.
- Simulated surgeon grip: a handle end within 4.5 cm of `surgeon_hand` that moves slower than 8 cm/s (0.3 s
  low-pass) for 0.6 s activates the weld `grip_<name>` (solref 0.004). The surgeon holds one instrument at a time:
  once another instrument that is out of its rack slot comes within 0.40 m of `surgeon_hand` (the robot arriving
  with the next one), the held one goes back to its slot. Until 2026-09-30 it went back only when the next one was
  gripped. It still lay welded along the previous handover axis, so in sessions without `/sim/reset_objects`
  (the iPad app) the next instrument jammed against it. Example: angled_tweezers after needle_driver -> scissors
  stopped 5-9 cm from the hand, and "surgeon did not take the instrument".
- The mannequin's hand and forearm geoms within 0.22 m of the site are made contact-free. A rigid hand cannot
  yield, and the weld does the gripping.
- `fixed_base:=true` welds the pelvis at the spawn pose, 0.78 m high, and turns off the harness. The weld point
  is the pelvis origin; at the world origin, a milliradian of yaw error drifted the pelvis 5 cm.
- `/sim/fixed_base_goal` (`geometry_msgs/PoseStamped`) glides the welded base in a straight line to another
  stance. It moves at 0.3 m/s, lifts 3 cm mid-way and does no collision checking. It stands in for walking in
  fixed-base mode, and whatever the hands hold travels with it.
- Physics option `cone="elliptic" impratio="10"`, which hospital-sim-env/objects also asks for. Instrument jitter in a closed
  Revo2 hand dropped from 4.6 to 1.6 cm/s RMS.

**Head RGB-D camera** (MJCF camera `head_d435` at `d435_link`, `fovy` 42°):

- Topics `/g1/head_camera/image_raw` (rgb8, 640x360, 16:9), `/depth` (16UC1 mm, aligned) and `/camera_info`.
  The intrinsics match the Isaac backend: fovy 42°, fx = fy = 468.9, cx 320, cy 180. Frames are in
  `d435_color_optical_frame` (parameter `camera_frame`), with a static TF from `d435_link`. Isaac stamps its
  optical-convention images with `d435_link`, so a consumer that uses TF must know which one it has.
- The RTF and rate numbers below were measured at 640x480.
- It renders only while someone subscribes, in a **separate process** (`head_camera.py`) fed with qpos through
  shared memory. A render thread inside the sim process starved the 500 Hz Python physics loop through the GIL
  (RTF 0.60, 8.8 fps).
- rosidl validates `uint8[]` byte by byte, about 50 ms per 640x480 frame. It is bypassed by assigning an
  `array('B')`, and the same fix went into the lidar cloud.
- Measured with the fixed-base theatre stack (sim + controller + manager + mission): RTF 1.00 and step mean
  about 1.2 ms both with and without a camera subscriber. The camera runs at 14.7-15 Hz, with 8 ms render +
  publish for RGB + depth (RTX 3070, EGL).

## Frame and spawns

World frame = Isaac world frame. The wing origin (SE door, west edge of the link corridor) is at
`(17.93, -4.45, 0)`. Wing-local (x, y) maps to world (x + 17.93, y - 4.45).

| Spawn | World (x, y, yaw) | Where |
|---|---|---|
| `corridor_west` (**default**) | (8.93, -12.45, 0) | Surgical corridor, west end, facing east. More than 20 m of 4 m-wide corridor ahead, so it is the best place to walk |
| `link` | (18.93, -5.65, -pi/2) | Link corridor just inside the SE door, facing south into the wing (2 m wide) |
| `lobby` | (18.93, -1.95, -pi/2) | Hospital SE lobby, 2.5 m north of the SE door, facing the door |
| `theatre_table` | (25.58, -20.00, -pi/2) | Operating theatre, 0.12 m behind the instrument-table stance (fixed-base mission spawn) |
| `theatre_handover` | (23.80, -18.25, -pi/2) | Operating theatre, handover stance beside the surgeon |

Pass `spawn:=link` or `spawn:="x,y,yaw"` to `sim.launch.py`. The `odom` frame starts at the spawn pose, like a
real odometry source. `/sim/ground_truth` gives the pose in the world frame.

## Run

```bash
source env.sh
ros2 launch g1_bringup sim.launch.py                    # headless hospital, placeholder policy
ros2 launch g1_bringup sim.launch.py viewer:=true world:=flat
ros2 run g1_teleop_keyboard keyboard_node               # own terminal: r, l, w/a/s/d/q/e, p, m, n, x
ros2 launch g1_bringup sim.launch.py fixed_base:=true spawn:=theatre_table   # arm-skill development
ros2 launch g1_bringup mission.launch.py                # scrub-nurse mission (see ros2_ws/src/hospital-sim-ros2/g1_mission)
```

Standalone inspection without ROS: `python -m mujoco.viewer --mjcf g1-mujoco-sim/hospital_g1.xml`.

## Measured on this machine (RTX 3070 box, 32 threads)

- `mj_step`: ~110 µs with the 980-geom hospital. The MID-360 slice (200 rays per step, 10 000 points per 10 Hz
  frame) costs ~360 µs. The 720-beam `/scan` costs ~0.76 ms, once every 50 steps.
- Whole sim loop including ROS publishing: 1.35-1.5 ms mean per 2 ms step. RTF is **0.99-1.00** with the full
  stack (sim, controller, manager, robot_state_publisher). It dropped to about 0.91 in one run while Isaac Sim and
  an MJX training job were loading the CPU at the same time.
- `/lowstate` and `/lowcmd`: 500 Hz. Policy: 50 Hz.

## Fixed 2026-09-30: /clock went backwards

`g1_mujoco_sim` stamped t = 6.9999999999 as 6.0 s. That value is the float sum of 0.002 s steps.
`int(t)` + `round(frac) % 1e9` sent `/clock` back by 1 s once every few sim seconds. Every C++ tf2 buffer
(AMCL, Nav2) then "detected a jump back in time" and cleared itself, and robot_state_publisher warned "Moved
backwards in time" every second. Stamps are now rounded once in integer nanoseconds; a test in
`g1_mujoco_sim/test` covers it. Earlier Nav2 runs on the sim were affected.

## Known limits

- Hospital building geometry is AABB-approximate. The wing is exact to `layout.py`.
- The ceiling is left out, so it is open from above in the viewer.
- Lidar intensity is constant. There is no noise model yet on the IMU or lidar.
