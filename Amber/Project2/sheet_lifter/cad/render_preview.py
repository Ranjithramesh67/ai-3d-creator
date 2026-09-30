"""Offline preview renders (matplotlib + trimesh) from the exported STLs."""

import json
import os
import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STL = os.path.join(ROOT, "models", "stl")
OUT = os.path.join(ROOT, "docs", "renders")
os.makedirs(OUT, exist_ok=True)

scene = json.load(open(os.path.join(ROOT, "models", "scene.json")))
colors = scene["colors"]


def load(name):
    return trimesh.load(os.path.join(STL, f"{name}.stl"), force="mesh")


cache = {}


def render(azim, elev, path, title="", only=None):
    fig = plt.figure(figsize=(11, 9), dpi=110)
    ax = fig.add_subplot(111, projection="3d")
    for it in scene["instances"]:
        name = it["part"]
        if only and name not in only:
            continue
        if name not in cache:
            cache[name] = load(name)
        m = cache[name].copy()
        rx, ry, rz = [np.radians(a) for a in it["rot"]]
        R = (trimesh.transformations.rotation_matrix(rz, [0, 0, 1])[:3, :3]
             @ trimesh.transformations.rotation_matrix(ry, [0, 1, 0])[:3, :3]
             @ trimesh.transformations.rotation_matrix(rx, [1, 0, 0])[:3, :3])
        m.vertices = m.vertices @ R.T + np.array(it["pos"])
        c = colors.get(name, "#999999")
        pc = Poly3DCollection(m.triangles, facecolor=c, edgecolor="none",
                              linewidth=0.05)
        pc.set_alpha(1.0)
        ax.add_collection3d(pc)
    ax.set_xlim(-900, 1150)
    ax.set_ylim(-800, 800)
    ax.set_zlim(-200, 1800)
    ax.set_box_aspect((2050, 1600, 2000))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    ax.set_title(title)
    plt.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)
    print("wrote", os.path.basename(path))


render(-60, 22, os.path.join(OUT, "iso.png"), "Sheet lifter - iso")
render(-90, 5, os.path.join(OUT, "front.png"), "Sheet lifter - front")
render(0, 88, os.path.join(OUT, "top.png"), "Sheet lifter - top")
render(-60, 15, os.path.join(OUT, "drivetrain.png"), "Drivetrain",
       only={"bevel_gearbox", "electric_motor", "disc_brake", "jaw_coupling",
             "drive_shaft", "worm_gear_jack", "trapez_screw", "base_frame",
             "motor_mount"})
render(-60, 20, os.path.join(OUT, "bed.png"), "Bed + T-slot adjustable guides",
       only={"bed_ribs", "tslot_track", "track_riser", "side_guide",
             "back_guide", "sheet_stack", "hgw35_block", "wiper_seal",
             "hgr35_rail", "rail_pad"})
render(-55, 20, os.path.join(OUT, "frame.png"), "Vertical-post rail support frame",
       only={"base_frame", "rail_support", "rail_post", "rail_pad",
             "hgr35_rail", "sensor_mast"})
render(-55, 12, os.path.join(OUT, "mast.png"), "Side sensor mast",
       only={"sensor_mast", "laser_sensor", "base_frame", "bed_ribs",
             "sheet_stack"})
print("done")
