# Development notes

Measurements and fix history from the development workstation. Nothing here is needed to install or run
the simulator; see the README for that.

## Measurements (development workstation: RTX 3070, 32 threads)

- The call `mj_step` took about 110 us with the 980-geom hospital of that time. The MID-360 slice (200 rays per
  step, 10 000 points per 10 Hz frame) costs about 360 us. The 720-beam `/scan` costs about 0.76 ms, once
  every 50 steps.
- The whole simulator loop, including ROS publishing, takes 1.35 ms to 1.5 ms mean per 2 ms step. The RTF is
  0.99 to 1.00 with the full stack (simulator, controller, manager, robot_state_publisher). It dropped to about
  0.91 in one run while Isaac Sim and an MJX training job loaded the CPU at the same time.
- The topics `/lowstate` and `/lowcmd` run at 500 Hz. The policy runs at 50 Hz.

## Fix of 2026-09-30: /clock stamps

The node `g1_mujoco_sim` stamped t = 6.9999999999 as 6.0 s. That value is the float sum of 0.002 s steps. The
expression `int(t)` + `round(frac) % 1e9` sent `/clock` back by 1 s once every few simulated seconds. Every
C++ tf2 buffer (AMCL, Nav2) then "detected a jump back in time" and cleared itself. The node
robot_state_publisher warned "Moved backwards in time" every second.

The node now rounds the stamps once in integer nanoseconds (`_stamp()` in `sim_node.py`). A test in
`g1_mujoco_sim/test` covers it. Earlier Nav2 runs on the simulator were affected.
