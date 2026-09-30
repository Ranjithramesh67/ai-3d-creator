"""Reusable 2D profiles and primitive hardware solids (build123d)."""

from build123d import (
    Align, Box, Circle, Cylinder, Polygon, Pos, Rectangle, extrude,
    make_face, Axis, Plane, Rot,
)
import params as PP

P = PP.P


def at(shape, x=0.0, y=0.0, z=0.0):
    return Pos(x, y, z) * shape


def hole(parent, r, z0, z1, x=0.0, y=0.0):
    """Subtract a vertical clearance cylinder between z0 and z1."""
    return parent - at(Cylinder(r, z1 - z0), x, y, (z0 + z1) / 2)


def ismc_profile(h, b, tw, tf):
    """ISMC C-channel cross-section: web at x=0 (height h), flanges toward +x."""
    pts = [
        (0.0, 0.0), (0.0, h), (b, h), (b, h - tf), (tw, h - tf),
        (tw, tf), (b, tf), (b, 0.0),
    ]
    return make_face(Polygon(*pts, align=None))


def ismc(length, h, b, tw, tf):
    """Channel extruded along +Z from z=0 to z=length."""
    return extrude(ismc_profile(h, b, tw, tf), amount=length)


def hex_prism(across_flats, height):
    """Hex prism centred on Z, across-flats given."""
    import math
    r = across_flats / math.cos(math.pi / 6) / 2.0
    pts = [(r * math.cos(math.pi / 3 * i + math.pi / 6),
            r * math.sin(math.pi / 3 * i + math.pi / 6)) for i in range(6)]
    return extrude(make_face(Polygon(*pts, align=None)), amount=height, both=False)


def bolt(shank_d, length, head_af=None, head_h=None):
    """Hex-head bolt along +Z: head at z=0..head_h, shank in -Z."""
    head_af = head_af or shank_d * 1.6
    head_h = head_h or shank_d * 0.7
    head = at(hex_prism(head_af, head_h), 0, 0, head_h / 2)
    shank = at(Cylinder(shank_d / 2, length), 0, 0, -length / 2)
    return head + shank


def nut(across_flats, h, bore):
    n = hex_prism(across_flats, h)
    n = n - at(Cylinder(bore / 2, h + 2), 0, 0, h / 2)
    return n


def washer(od, id_, t):
    return (at(Cylinder(od / 2, t), 0, 0, t / 2)
            - at(Cylinder(id_ / 2, t + 2), 0, 0, t / 2))


def disc(d, t):
    return Cylinder(d / 2, t)


def pipe(od, id_, length):
    return (at(Cylinder(od / 2, length), 0, 0, length / 2)
            - at(Cylinder(id_ / 2, length + 2), 0, 0, length / 2))


def box(x, y, z):
    return Box(x, y, z)
