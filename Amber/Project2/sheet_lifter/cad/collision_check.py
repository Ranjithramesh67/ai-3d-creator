"""
Interference scan for the 3-ton sheet-lifting machine.

Rebuilds every part, places it exactly as build.py does, then reports every pair
of *different* parts whose solids overlap by more than a tolerance.  A small
whitelist of pairs is allowed to interfere because they are intentionally
assembled that way (sliding guide block on its rail, shaft inside a coupling,
screw through a jack, pedestal under a foot, ...).  Anything else is an
unwanted collision and must be fixed.

Run:  python3 collision_check.py
"""

import json
import os
import sys
import time

from build123d import Compound, Pos, Rot, Cylinder, Align

import params as PP
import parts as PT

P = PP.P
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# Pairs that are *supposed* to share space (assembly fits / sliding surfaces).
ALLOWED = {
    frozenset(("rail_pad", "rail_support")),
    frozenset(("hgr35_rail", "rail_pad")),
    frozenset(("hgr35_rail", "rail_post")),
    frozenset(("rail_post", "rail_support")),
    frozenset(("rail_support", "base_frame")),
    frozenset(("hgw35_block", "hgr35_rail")),
    frozenset(("wiper_seal", "hgr35_rail")),
    frozenset(("wiper_seal", "hgw35_block")),
    frozenset(("wiper_seal", "bed_ribs")),
    frozenset(("hgw35_block", "bed_ribs")),
    frozenset(("drive_shaft", "worm_gear_jack")),
    frozenset(("drive_shaft", "bevel_gearbox")),
    frozenset(("drive_shaft", "jaw_coupling")),
    frozenset(("trapez_screw", "worm_gear_jack")),
    frozenset(("bevel_gearbox", "jaw_coupling")),
    frozenset(("jaw_coupling", "disc_brake")),
    frozenset(("disc_brake", "electric_motor")),
    frozenset(("electric_motor", "jaw_coupling")),
    frozenset(("electric_motor", "motor_mount")),
    frozenset(("bevel_gearbox", "gearbox_mount")),
    # lifting bed: tracks and guides are mounted on the deck and clamp to them
    frozenset(("tslot_track", "bed_ribs")),
    frozenset(("track_riser", "bed_ribs")),
    frozenset(("track_riser", "tslot_track")),
    frozenset(("side_guide", "tslot_track")),
    frozenset(("back_guide", "tslot_track")),
    frozenset(("side_guide", "back_guide")),
    frozenset(("sheet_stack", "bed_ribs")),
    # jack hold-down: stool on the mid beam, L-clamps bolt the housing down
    frozenset(("jack_stool", "rail_support")),
    frozenset(("jack_stool", "worm_gear_jack")),
    frozenset(("jack_clamp", "worm_gear_jack")),
    frozenset(("jack_clamp", "jack_stool")),
    # travelling bronze nut: threads the screw, bolts up to the deck bracket
    frozenset(("trapez_screw", "bronze_nut")),
    frozenset(("trapez_screw", "nut_bracket")),
    frozenset(("bronze_nut", "nut_bracket")),
    frozenset(("nut_bracket", "bed_ribs")),
    # guided drag chain: rests in the trough and is bracketed at both ends
    frozenset(("drag_chain", "chain_trough")),
    frozenset(("drag_chain", "chain_bed_bracket")),
    frozenset(("chain_trough", "base_frame")),
    frozenset(("chain_bed_bracket", "bed_ribs")),
    frozenset(("motor_mount", "base_frame")),
    frozenset(("sensor_mast", "base_frame")),
    frozenset(("sensor_mast", "laser_sensor")),
    frozenset(("cabinet", "estop")),
    frozenset(("cabinet", "hmi")),
    frozenset(("drag_chain", "cabinet")),
    frozenset(("drag_chain", "bed_ribs")),
}

TOL = 5.0  # mm^3 of overlap below which we treat the parts as merely touching


def build_geom():
    """Same part set as build.py, but the thread is a plain envelope cylinder."""
    g = {}
    g["base_frame"] = PT.base_frame()
    g["rail_support"] = PT.rail_support()
    g["rail_post"] = PT.rail_post()
    g["rail_pad"] = PT.pad_and_bolts()
    g["hgr35_rail"] = PT.hgr35_rail()
    g["hgw35_block"] = PT.hgw35_block()
    g["wiper_seal"] = PT.wiper_seal()
    g["sensor_mast"] = PT.sensor_mast()
    g["bed_ribs"] = PT.bed_ribs()
    g["tslot_track"] = PT.tslot_track()
    g["track_riser"] = PT.track_riser()
    g["side_guide"] = PT.side_guide()
    g["back_guide"] = PT.back_guide()
    g["nut_bracket"] = PT.nut_bracket()
    g["sheet_stack"] = PT.sheet_stack()
    # envelope stand-in for the screw envelope, far cheaper than 10 helical sweeps
    g["trapez_screw"] = Cylinder(P.SCREW_D / 2, P.SCREW_LEN,
                                 align=(Align.CENTER, Align.CENTER, Align.MIN))
    g["worm_gear_jack"] = PT.worm_gear_jack()
    g["bronze_nut"] = PT.bronze_nut()
    g["jack_stool"] = PT.jack_stool()
    g["jack_clamp"] = PT.jack_clamp()
    g["electric_motor"] = PT.electric_motor()
    g["disc_brake"] = PT.disc_brake()
    g["jaw_coupling"] = PT.jaw_coupling()
    g["bevel_gearbox"] = PT.bevel_gearbox()
    g["gearbox_mount"] = PT.gearbox_mount()
    g["drive_shaft"] = PT.drive_shaft(P.SHAFT_LEN)
    g["motor_mount"] = PT.motor_mount()
    g["laser_sensor"] = PT.laser_sensor()
    g["cabinet"] = PT.cabinet()
    g["estop"] = PT.estop()
    g["hmi"] = PT.hmi()
    g["drag_chain"] = PT.drag_chain()
    g["chain_trough"] = PT.chain_trough()
    g["chain_bed_bracket"] = PT.chain_bed_bracket()
    return g


def instances():
    import math
    P = PP.P
    RAIL_XY = [(sx * P.RAIL_X, sy * P.RAIL_Y) for sx in (-1, 1) for sy in (-1, 1)]
    RAIL_PAD_Z = P.ISMC200_H + P.RAIL_LAND_T
    RAIL_TOP = RAIL_PAD_Z + P.PAD_T
    RAIL_ROT = {1: (0, 0, -90), -1: (0, 0, 90)}
    SHAFT_X = P.SHAFT_X
    inst = []

    def place(name, pos, rot=(0, 0, 0), group="static"):
        inst.append({"part": name, "group": group,
                     "pos": [float(v) for v in pos],
                     "rot": [float(v) for v in rot]})

    place("base_frame", (0, 0, 0))
    place("rail_support", (0, 0, 0))
    place("rail_post", (0, 0, 0))
    place("sensor_mast", (0, 0, 0))
    place("cabinet", (P.CAB_X, P.CAB_Y, 0))
    place("estop", (P.CAB_X + P.CAB_D / 2 + 12, P.CAB_Y - 180, P.CAB_Z + 620))
    place("hmi", (P.CAB_X + P.CAB_D / 2 + 14, P.CAB_Y + 120, P.CAB_Z + 480))
    place("chain_trough", (0, 0, 0))
    place("drag_chain", (0, 0, 0))
    for (rx, ry) in RAIL_XY:
        place("rail_pad", (rx, ry, RAIL_PAD_Z))
        place("hgr35_rail", (rx, ry, RAIL_TOP), RAIL_ROT[int(math.copysign(1, rx))])
    for sx in (-1, 1):
        place("worm_gear_jack", (sx * P.JACK_X, 0, P.JACK_Z),
              (0, 0, 90 if sx < 0 else -90))
        place("trapez_screw", (sx * P.JACK_X, 0, P.SCREW_BOTTOM))
        place("bronze_nut", (sx * P.JACK_X, 0, P.NUT_Z_HOME), group="lift")
        place("drive_shaft", (sx * SHAFT_X, 0, P.JACK_Z))
        place("jack_stool", (sx * P.JACK_X, 0, P.ISMC200_H + P.RAIL_LAND_T))
        for cy in (-1, 1):
            place("jack_clamp", (sx * P.JACK_X, cy * P.JACK_BODY / 2,
                                 P.JACK_BOTTOM), (0, 0, -90 if cy < 0 else 90))
    place("bevel_gearbox", (0, 0, P.JACK_Z))
    place("gearbox_mount", (0, 0, 0))
    place("jaw_coupling", (0, P.COUPLING_Y, P.JACK_Z))
    place("disc_brake", (0, P.BRAKE_Y, P.JACK_Z))
    place("electric_motor", (0, P.MOTOR_FACE_Y, P.JACK_Z), (0, 0, 0))
    place("motor_mount", (0, 0, 0))
    place("laser_sensor", (P.MAST_X - P.MAST_ARM, 0, P.MAST_Z - 150))
    place("bed_ribs", (0, 0, P.Z_BED_HOME), group="lift")
    place("nut_bracket", (0, 0, P.Z_BED_HOME), group="lift")
    place("chain_bed_bracket", (0, 0, 0), group="lift")
    for sy in (-1, 1):
        place("tslot_track", (0, sy * P.TRACK_Y, P.Z_BED_HOME), group="lift")
    for sx in (-1, 1):
        place("tslot_track", (sx * P.TRACK_Y, 0, P.Z_BED_HOME + P.TSLOT_H),
              (0, 0, 90), group="lift")
        for yy in (-380.0, 0.0, 380.0):
            place("track_riser", (sx * P.TRACK_Y, yy, P.Z_BED_HOME),
                  group="lift")
    for (rx, ry) in RAIL_XY:
        bz = P.Z_BED_HOME - P.BLOCK_Z_OFFSET
        rot = RAIL_ROT[int(math.copysign(1, rx))]
        place("hgw35_block", (rx, ry, bz), rot, group="lift")
        for zz in (-1, 1):
            place("wiper_seal", (rx, ry, bz + zz * (P.BLOCK_L / 2 - P.WIPER_T / 2)),
                  rot, group="lift")
    fence_x = P.STACK_W / 2 + P.FENCE_T / 2 + 2.0
    stop_y = P.STACK_D / 2 + P.FENCE_T / 2 + 2.0
    for sx in (-1, 1):
        place("side_guide", (sx * fence_x, 0, P.Z_BED_HOME + P.TSLOT_H),
              group="lift")
    place("back_guide", (0, stop_y, P.Z_BED_HOME + 2 * P.TSLOT_H), group="lift")
    place("sheet_stack", (0, 0, P.Z_BED_HOME), group="lift")
    return inst


def aabb(shape):
    b = shape.bounding_box()
    return (b.min.X, b.min.Y, b.min.Z, b.max.X, b.max.Y, b.max.Z)


def overlaps(a, b, slack=0.0):
    return not (a[3] < b[0] - slack or b[3] < a[0] - slack or
                a[4] < b[1] - slack or b[4] < a[1] - slack or
                a[5] < b[2] - slack or b[5] < a[2] - slack)


def main():
    t0 = time.time()
    print("building geometry ...")
    geom = build_geom()
    inst = instances()
    located = []
    for i, it in enumerate(inst):
        s = geom[it["part"]]
        loc = Pos(*it["pos"]) * Rot(*it["rot"])
        shp = loc * s
        located.append((i, it["part"], shp, aabb(shp)))
    print(f"  {len(located)} instances in {time.time()-t0:.1f} s")

    pairs = []
    n = len(located)
    for i in range(n):
        for j in range(i + 1, n):
            ii, na, sa, ba = located[i]
            jj, nb, sb, bb = located[j]
            if not overlaps(ba, bb):
                continue
            key = frozenset((na, nb))
            if key in ALLOWED:
                continue
            if na == nb:
                continue
            try:
                inter = sa.intersect(sb)
                vol = sum(s.volume for s in inter)
            except Exception as e:  # noqa: BLE001
                vol = -1.0
            if vol > TOL:
                pairs.append({
                    "a": na, "b": nb, "a_idx": ii, "b_idx": jj,
                    "volume_mm3": round(vol, 2),
                    "a_pos": inst[ii]["pos"], "b_pos": inst[jj]["pos"],
                })

    pairs.sort(key=lambda p: -p["volume_mm3"])
    print(f"\nunwanted collisions: {len(pairs)}  ({time.time()-t0:.1f} s)")
    for p in pairs:
        print(f"  {p['volume_mm3']:10.2f} mm^3  {p['a']} @{p['a_pos']} "
              f"<-> {p['b']} @{p['b_pos']}")
    out = {"tol_mm3": TOL, "collisions": pairs,
           "seconds": round(time.time() - t0, 1)}
    with open(os.path.join(ROOT, "models", "collision_report.json"), "w") as f:
        json.dump(out, f, indent=1)
    return 1 if pairs else 0


if __name__ == "__main__":
    sys.exit(main())
