"""Render axonometric / orthographic previews of the assembly to PNG.

Uses the per-part STL files plus the scene manifest so the output matches the
web viewer exactly.  matplotlib's painter algorithm is plenty for a design
sanity check.
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL = os.path.join(ROOT, "models", "stl")
OUT = os.path.join(ROOT, "docs", "renders")
os.makedirs(OUT, exist_ok=True)

scene = json.load(open(os.path.join(ROOT, "models", "scene.json")))

cache = {}


def get_mesh(part):
    if part not in cache:
        cache[part] = trimesh.load(os.path.join(STL, part + ".stl"))
    return cache[part]


def placed_tris(inst, lift=0.0):
    m = get_mesh(inst["part"]).copy()
    rx, ry, rz = [np.radians(a) for a in inst["rot"]]
    m.apply_transform(trimesh.transformations.euler_matrix(rx, ry, rz))
    p = list(inst["pos"])
    if inst["group"] == "carriage":
        p[2] += lift
    m.apply_translation(p)
    return m


def render(name, elev, azim, view_axis=None, groups=None, size=(9, 9), lift=0.0,
           exclude=()):
    fig = plt.figure(figsize=size, dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    for inst in scene["instances"]:
        if groups and inst["group"] not in groups:
            continue
        if inst["part"] in exclude:
            continue
        m = placed_tris(inst, lift)
        col = scene["colors"].get(inst["part"], "#888888")
        pc = Poly3DCollection(m.triangles, facecolors=col, edgecolors="none", alpha=1.0)
        ax.add_collection3d(pc)
    if view_axis == "z":
        ax.view_init(elev=90, azim=-90)
    elif view_axis == "y":
        ax.view_init(elev=0, azim=-90)
    elif view_axis == "x":
        ax.view_init(elev=0, azim=0)
    else:
        ax.view_init(elev=elev, azim=azim)
    ax.set_xlim(-140, 140); ax.set_ylim(-110, 110); ax.set_zlim(0, 260)
    try:
        ax.set_box_aspect((280, 220, 260))
    except Exception:
        pass
    ax.set_axis_off()
    ax.set_facecolor("#0e1116")
    fig.patch.set_facecolor("#0e1116")
    fig.savefig(os.path.join(OUT, name), bbox_inches="tight", facecolor="#0e1116")
    plt.close(fig)
    print("wrote", name)


render("iso.png", 26, -55)
render("front.png", 0, -90, view_axis=None)
render("top.png", 0, 0, view_axis="z")
render("iso_carriage_only.png", 26, -55, groups={"carriage"})
render("iso_bed_up.png", 26, -55, lift=scene["meta"]["travel_mm"])
render("base_iso.png", 35, -58, exclude=("top_frame", "sensor_bracket", "carriage_bed"),
       groups=None)
print("done")
