"""Compact parametric mesh for the Tr60x9 lifting screw (web viewer asset).

OCC's STL mesher over-tessellates a swept helix (megabytes per part), so the
web asset is generated directly from the thread mathematics: a lathed core plus
a genuine helical trapezoidal rib.  Geometry matches screw_thread_seg() in
parts.py (same pitch, root and crest diameters).
"""

import numpy as np
import trimesh

import params as PP

P = PP.P


def build_screw_mesh(length=None, pitch=None, core_d=None, crest_d=None,
                     seg_per_turn=12, radial=48):
    length = P.SCREW_LEN if length is None else length
    pitch = P.SCREW_PITCH if pitch is None else pitch
    core_d = P.SCREW_CORE_D if core_d is None else core_d
    crest_d = P.SCREW_D if crest_d is None else crest_d

    r_core, r_crest = core_d / 2.0, crest_d / 2.0
    depth = r_crest - r_core

    core = trimesh.creation.cylinder(radius=r_core, height=length,
                                     sections=radial)
    core.apply_translation([0.0, 0.0, length / 2.0])

    turns = length / pitch
    n = max(2, int(round(turns * seg_per_turn)))
    t = np.linspace(0.0, turns, n)
    theta, z = 2.0 * np.pi * t, pitch * t

    hr, hc = pitch * 0.29, pitch * 0.145
    profile = [(-1.0, -hr), (depth, -hc), (depth, hc), (-1.0, hr)]
    ct, st = np.cos(theta), np.sin(theta)

    verts = np.empty((n, 4, 3), dtype=np.float64)
    for k, (dr, dz) in enumerate(profile):
        r = r_core + dr
        verts[:, k, 0] = r * ct
        verts[:, k, 1] = r * st
        verts[:, k, 2] = z + dz
    verts = verts.reshape(-1, 3)

    faces = []
    for i in range(n - 1):
        a, b = i * 4, (i + 1) * 4
        for e in range(4):
            e2 = (e + 1) % 4
            faces.append([a + e, a + e2, b + e2])
            faces.append([a + e, b + e2, b + e])
    faces.append([0, 1, 2])
    faces.append([0, 2, 3])
    last = (n - 1) * 4
    faces.append([last + 3, last + 2, last + 1])
    faces.append([last + 3, last + 1, last])

    rib = trimesh.Trimesh(vertices=verts, faces=np.asarray(faces),
                          process=False)
    trimesh.repair.fix_normals(rib)
    return trimesh.util.concatenate([core, rib])


def write_screw_stl(path):
    mesh = build_screw_mesh()
    mesh.export(path, file_type="stl")
    return len(mesh.faces)


if __name__ == "__main__":
    import os
    out = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "models", "stl", "trapez_screw.stl")
    nf = write_screw_stl(out)
    print(f"wrote {out}: {nf} faces, {os.path.getsize(out)/1e6:.3f} MB")
