"""Render a recorded sim run (qpos .npz from `sim.launch.py record:=<file>`) to MP4 offscreen.

    MUJOCO_GL=egl python g1-mujoco-sim/render_replay.py run.npz out.mp4 [--t0 10 --t1 45] [--scene hospital_g1.xml]
        [--size 640x360] [--fps 25] [--follow] [--camera theatre_station,theatre_handover] [--label "text"]
--camera renders named MJCF cameras instead of the follow camera (several = side by side).

Rendering happens after the run, so recording never costs the real-time sim anything.
"""
import argparse
import os

import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("npz")
    ap.add_argument("out")
    ap.add_argument("--scene", default=os.path.join(HERE, "hospital_g1.xml"))
    ap.add_argument("--t0", type=float, default=0.0)
    ap.add_argument("--t1", type=float, default=1e9)
    ap.add_argument("--size", default="640x352")
    ap.add_argument("--fps", type=float, default=25.0)
    ap.add_argument("--distance", type=float, default=3.0)
    ap.add_argument("--azimuth", type=float, default=25.0, help="deg, world frame (with this default the camera is ahead of a robot heading +x, facing it)")
    ap.add_argument("--elevation", type=float, default=-18.0)
    ap.add_argument("--crf", type=int, default=30)
    ap.add_argument("--camera", default="", help="named MJCF camera(s), comma separated -> side by side")
    ap.add_argument("--label", default="", help="caption burnt into the top-left corner")
    a = ap.parse_args()
    import imageio.v2 as iio
    z = np.load(a.npz)
    t, qpos = z["t"], z["qpos"]
    sel = (t >= a.t0) & (t <= a.t1)
    t, qpos = t[sel], qpos[sel]
    w, h = (int(v) for v in a.size.split("x"))
    m = mujoco.MjModel.from_xml_path(a.scene)
    m.vis.global_.offwidth = max(m.vis.global_.offwidth, w)
    m.vis.global_.offheight = max(m.vis.global_.offheight, h)
    d = mujoco.MjData(m)
    r = mujoco.Renderer(m, h, w)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.azimuth, cam.elevation = a.distance, a.azimuth, a.elevation
    opt = mujoco.MjvOption()
    writer = iio.get_writer(a.out, fps=a.fps, codec="libx264", quality=None,
                            ffmpeg_params=["-crf", str(a.crf), "-preset", "slow", "-pix_fmt", "yuv420p"])
    frame_t = np.arange(t[0], t[-1], 1.0 / a.fps)
    look = None
    cams = [c for c in a.camera.split(",") if c]
    for ft in frame_t:
        i = min(len(t) - 1, int(np.searchsorted(t, ft)))
        d.qpos[:] = qpos[i]
        mujoco.mj_forward(m, d)
        if cams:
            imgs = []
            for c in cams:
                r.update_scene(d, camera=c, scene_option=opt)
                imgs.append(r.render())
            img = np.concatenate(imgs, 1)
        else:
            target = d.qpos[0:3].copy()
            target[2] = 0.8
            look = target if look is None else 0.9 * look + 0.1 * target  # smooth follow
            cam.lookat[:] = look
            r.update_scene(d, cam, opt)
            img = r.render()
        if a.label:
            import cv2
            img = np.ascontiguousarray(img)
            txt = f"{a.label}   t={ft - frame_t[0]:5.1f} s"
            cv2.rectangle(img, (0, 0), (12 + 9 * len(txt), 26), (20, 20, 20), -1)
            cv2.putText(img, txt, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        writer.append_data(img)
    writer.close()
    print(f"{len(frame_t)} frames, {t[0]:.1f}..{t[-1]:.1f} s sim -> {a.out} "
          f"({os.path.getsize(a.out) / 1e6:.2f} MB)")


if __name__ == "__main__":
    main()
