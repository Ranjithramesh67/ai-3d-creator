"""build123d part generators for the dual-screw lift table."""

import math
from build123d import (
    Align, Axis, Box, Circle, Compound, Cylinder, Plane, Polygon, Pos, Rectangle,
    Rot, SlotOverall, extrude, make_hull, offset, export_step, export_stl,
)
from params import P, PI, rad


# --------------------------------------------------------------------------- helpers
def at(shape, x=0.0, y=0.0, z=0.0, rz=0.0):
    return Pos(x, y, z) * Rot(0, 0, rz) * shape


def hole(shape, r, z0, z1, x=0.0, y=0.0):
    """Subtract a vertical cylinder centred on (x, y) between z0 and z1."""
    return shape - at(Cylinder(r, z1 - z0), x, y, (z0 + z1) / 2)


def box_at(l, w, h, x, y, z, rz=0.0):
    return at(Box(l, w, h), x, y, z, rz)


# --------------------------------------------------------------------------- rails
def _profile_slots(length, axis):
    """Return the 4T-slot + centre-bore cutter solids for a 20x20 rail."""
    a = P.EXT / 2
    slots = []
    if axis == "X":
        slots.append(Box(length, 6.2, 6.0))          # +Y face
        slots.append(Box(length, 6.2, 6.0))
        slots.append(Box(length, 6.0, 6.2))          # +Z / -Z
        slots.append(Box(length, 6.0, 6.2))
        for s, off in zip(slots, [(0, a - 3, 0), (0, -(a - 3), 0),
                                  (0, 0, a - 3), (0, 0, -(a - 3))]):
            pass
    return slots


def rail(length, axis, cx, cy, cz=None):
    """A simplified 2020 extrusion bar, length along `axis`, top at z=20."""
    zc = P.Z_RAIL_TOP / 2 if cz is None else cz
    if axis == "X":
        s = at(Box(length, P.EXT, P.EXT), cx, cy, zc)
        a = P.EXT / 2
        c = 3.0
        for oy, oz, sw, sh in [(a - c, 0, 6.2, 6.0), (-(a - c), 0, 6.2, 6.0),
                               (0, a - c, 6.0, 6.2), (0, -(a - c), 6.0, 6.2)]:
            s -= at(Box(length, sw, sh), cx, cy + oy, zc + oz)
        s -= at(Rot(0, 90, 0) * Cylinder(2.1, length), cx, cy, zc)
    else:
        s = at(Box(P.EXT, length, P.EXT), cx, cy, zc)
        a = P.EXT / 2
        c = 3.0
        for ox, oz, sw, sh in [(a - c, 0, 6.2, 6.0), (-(a - c), 0, 6.2, 6.0),
                               (0, a - c, 6.0, 6.2), (0, -(a - c), 6.0, 6.2)]:
            s -= at(Box(sw, length, sh), cx + ox, cy, zc + oz)
        s -= at(Rot(90, 0, 0) * Cylinder(2.1, length), cx, cy, zc)
    return s


# --------------------------------------------------------------------------- corner bracket
def corner_bracket(sx, sy):
    """Locks two extrusions at 90 deg and anchors the bottom of a guide rod."""
    cx, cy = sx * P.ROD_X, sy * P.ROD_Y
    z0, z1 = P.CB_Z0, P.CB_Z1
    s = at(Box(P.CB_SIZE, P.CB_SIZE, z1 - z0), cx, cy, (z0 + z1) / 2)
    # locating lips that drop into the T-slots of both rails
    lip_h = 4.0
    s += at(Box(P.CB_SIZE - 8, 4.0, lip_h), cx, sy * P.RAIL_Y, z0 - lip_h / 2)
    s += at(Box(4.0, 20.0, lip_h), sx * P.RAIL_X, cy - sy * 7.0, z0 - lip_h / 2)
    # guide-rod blind hole (8.2 x 15 deep)
    s -= at(Cylinder(P.ROD_HOLE_D / 2, P.ROD_HOLE_DEPTH),
            cx, cy, z1 - P.ROD_HOLE_DEPTH / 2)
    # 3 x M3 into the two rails (heads sit on top of the bracket)
    bolts = [(sx * P.RAIL_X, sy * (P.ROD_Y - 8.0)),
             (sx * (P.ROD_X - 8.0), sy * P.RAIL_Y),
             (sx * (P.ROD_X + 8.0), sy * P.RAIL_Y)]
    for bx, by in bolts:
        s = hole(s, P.M3_CLEAR / 2, 0, z1, bx, by)
    # anchor point for the wire mast (only materially used on the rear-right corner)
    s = hole(s, P.M3_CLEAR / 2, 0, z1, sx * (P.ROD_X + 11.0), sy * P.ROD_Y)
    return s


# --------------------------------------------------------------------------- screw anchor
def screw_anchor(sx):
    """Sits on a side rail and carries the bottom 608ZZ thrust bearing of a screw."""
    cx = sx * P.SCREW_X
    z0, z1 = P.Z_RAIL_TOP, P.Z_RAIL_TOP + 16.0
    xa, xb = min(cx - 15, sx * (P.RAIL_X + 4)), max(cx + 15, sx * (P.RAIL_X + 4))
    s = at(Box(xb - xa, 30.0, z1 - z0), (xa + xb) / 2, 0, (z0 + z1) / 2)
    # gusset rib outboard so the 40 mm reach is stiff in bending
    rx0, rx1 = min(cx + 14, sx * P.RAIL_X), max(cx + 14, sx * P.RAIL_X)
    s += at(Box(rx1 - rx0, 6.0, 10.0), (rx0 + rx1) / 2, 0, z1 + 5.0)
    # 608ZZ pocket: 22.4 slip bore x 8 deep, floor ledge, then 9 clearance
    s -= at(Cylinder(P.B608_OD / 2 + 0.2, 8.0), cx, 0, z1 - 4.0)
    s -= at(Cylinder(4.5, 12.0), cx, 0, z1 - 8.0 - 6.0)
    # 2 x M3 down into the side rail top slot
    for by in (-10.0, 10.0):
        s = hole(s, P.M3_CLEAR / 2, z0 - 1, z1 + 11, sx * P.RAIL_X, by)
    return s


# --------------------------------------------------------------------------- motor mount
def motor_mount():
    """Z-axis style bracket bridging the rear rail; motor shaft points up."""
    base = at(Box(60, 26, 6), 0, 92, P.Z_RAIL_TOP + 3)
    top = at(Box(52, 46, 6), 0, P.MOTOR_Y - 1, P.Z_MOTOR_FACE + 3)
    # two side walls beside the motor (motor body is x = +/-21.15) tie the
    # cantilevered motor plate back to the base with no overhang
    walls = at(Box(6, 66, 28), 25, 67, P.Z_RAIL_TOP + 14)
    walls += at(Box(6, 66, 28), -25, 67, P.Z_RAIL_TOP + 14)
    s = base + top + walls
    # 2 x M3 into the rear rail top slot
    for bx in (-20.0, 20.0):
        s = hole(s, P.M3_CLEAR / 2, 0, P.Z_RAIL_TOP + 6, bx, P.RAIL_Y)
    # NEMA17 pilot + shaft clearance and 4 x M3 mounting holes
    s = hole(s, 12.0, P.Z_MOTOR_FACE - 1, P.Z_MOTOR_FACE + 7, 0, P.MOTOR_Y)
    for dx in (-P.NEMA_BOLT / 2, P.NEMA_BOLT / 2):
        for dy in (-P.NEMA_BOLT / 2, P.NEMA_BOLT / 2):
            s = hole(s, P.M3_CLEAR / 2, P.Z_MOTOR_FACE - 1, P.Z_MOTOR_FACE + 7,
                     0 + dx, P.MOTOR_Y + dy)
    return s


# --------------------------------------------------------------------------- tensioner
def tensioner_bracket():
    """Slot-adjustable idler mount that tensions the front belt span."""
    y_idler = -34.0
    base = at(Box(60, 20, 6), 0, -90, P.Z_RAIL_TOP + 3)
    column = at(Box(12, 16, 18), 0, y_idler, 35)
    lower = at(Box(24, 18, 6), 0, y_idler, 47)
    upper = at(Box(24, 18, 6), 0, y_idler, 65)
    web = at(Box(12, 6, 24), 0, y_idler - 8, 56)
    s = base + column + lower + upper + web
    # slotted axle hole (8.4 wide x 12 long in Y) through the whole clevis
    s -= at(Box(8.4, 12.0, 46), 0, y_idler, 56)
    # 2 x M3 into the front rail top slot
    for bx in (-20.0, 20.0):
        s = hole(s, P.M3_CLEAR / 2, 0, P.Z_RAIL_TOP + 6, bx, -P.RAIL_Y)
    return s


def wire_mast():
    """Post outside the carriage envelope that carries the sensor cable down
    alongside the rear-right guide rod (never touched by the moving bed)."""
    bar = at(Box(8, 10, 184), 124, P.ROD_Y, 132)
    foot_lo = at(Box(16, 12, 8), 116, P.ROD_Y, 42)
    foot_hi = at(Box(16, 12, 8), 116, P.ROD_Y, 226)
    s = bar + foot_lo + foot_hi
    # open wire groove down the outboard face
    s -= at(Box(3.0, 4.0, 184), 127.0, P.ROD_Y, 132)
    for zz in (42.0, 226.0):
        s = hole(s, P.M3_CLEAR / 2, zz - 4, zz + 4, 116, P.ROD_Y)
    return s


# --------------------------------------------------------------------------- carriage
def carriage_bed():
    """Ribbed payload plate with integrated LM8UU housings and T8 nut flanges."""
    z0 = 0.0                      # local; assembly translate sets Z_BED
    zt = z0 + P.BED_T
    zr = zt + P.RIB_T
    # ---- base plate
    s = at(Box(2 * P.BED_X, 2 * P.BED_Y, P.BED_T), 0, 0, z0 + P.BED_T / 2)
    # ---- rim + rib grid on the top face
    rim = Rectangle(2 * P.BED_X, 2 * P.BED_Y) - Rectangle(
        2 * P.BED_X - 2 * P.RIB_W, 2 * P.BED_Y - 2 * P.RIB_W)
    s += at(extrude(rim, P.RIB_T), 0, 0, zt)
    for rx in (-60.0, -20.0, 20.0, 60.0):
        s += at(Box(P.RIB_W, 2 * P.BED_Y - 2 * P.RIB_W, P.RIB_T), rx, 0, zt + P.RIB_T / 2)
    for ry in (-40.0, 0.0, 40.0):
        s += at(Box(2 * P.BED_X - 2 * P.RIB_W, P.RIB_W, P.RIB_T), 0, ry, zt + P.RIB_T / 2)

    # ---- LM8UU corner housings
    boss_h = P.LM8_UP + P.LM8_DOWN
    boss_cz = z0 + P.LM8_UP - boss_h / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * P.ROD_X, sy * P.ROD_Y
            s += at(Cylinder(P.LM8_BOSS_OD / 2, boss_h), cx, cy, boss_cz)
            # gussets tying the boss to the plate
            s += at(Box(20, 5, P.RIB_T), cx - sx * 13, cy, zt + P.RIB_T / 2)
            s += at(Box(5, 20, P.RIB_T), cx, cy - sy * 13, zt + P.RIB_T / 2)
    # ---- nut pads
    for sx in (-1, 1):
        s += at(Cylinder(P.NUT_FLANGE_D / 2 + 5, P.RIB_T), sx * P.SCREW_X, 0, zt + P.RIB_T / 2)

    # ---- cleaning: LM8UU bores, retaining lip, zip-tie slots, relief notch
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx, cy = sx * P.ROD_X, sy * P.ROD_Y
            s = hole(s, P.LM8_BORE / 2, z0 - P.LM8_DOWN + P.LM8_RETAIN, z0 + P.LM8_UP, cx, cy)
            s = hole(s, P.LM8_RETAIN_D / 2, z0 - P.LM8_DOWN, z0 - P.LM8_DOWN + P.LM8_RETAIN,
                     cx, cy)
            # two horizontal grooves for zip ties
            for gz in (z0 + 4.0, z0 - 6.0):
                ring = at(Cylinder(P.LM8_BOSS_OD / 2 + 1, P.ZIP_GROOVE_W), cx, cy, gz)
                ring -= at(Cylinder(P.LM8_BOSS_OD / 2 - P.ZIP_GROOVE_D, P.ZIP_GROOVE_W),
                           cx, cy, gz)
                s -= ring
            # vertical relief notch on the inboard side
            ang = math.degrees(math.atan2(-cy, -cx))
            s -= at(Box(P.RELIEF_W, 34, boss_h),
                    cx - sx * (P.LM8_BOSS_OD / 2) * 0.55,
                    cy - sy * (P.LM8_BOSS_OD / 2) * 0.55,
                    boss_cz, ang - 90)

    # ---- T8 anti-backlash nut mounts (slotted on X)
    for sx in (-1, 1):
        cx = sx * P.SCREW_X
        s = hole(s, P.NUT_CLEAR_D / 2, z0 - 2, zr, cx, 0)
        for dx in (-P.NUT_HOLE_SQ / 2, P.NUT_HOLE_SQ / 2):
            for dy in (-P.NUT_HOLE_SQ / 2, P.NUT_HOLE_SQ / 2):
                s -= at(Box(P.NUT_SLOT_LEN, P.M3_CLEAR, zr + 4), cx + dx, dy, zt)
    return s


# --------------------------------------------------------------------------- top frame
def top_frame():
    """Ties the 4 rods + 2 screws together and carries the top bearings."""
    z0, z1 = P.Z_TOP_BOT, P.Z_TOP_TOP
    s = at(Box(2 * P.BED_X, 2 * P.BED_Y, z1 - z0), 0, 0, (z0 + z1) / 2)
    # lightening windows (a solid centre spine is kept for the wire channel)
    for sx in (-1, 1):
        for sy in (-1, 1):
            s -= at(Box(84, 42, 20), sx * 66, sy * 44, (z0 + z1) / 2)
    # bosses at the rod columns
    for sx in (-1, 1):
        for sy in (-1, 1):
            s += at(Cylinder(11, z1 - z0), sx * P.ROD_X, sy * P.ROD_Y, (z0 + z1) / 2)
    # bosses + pockets for the top 608 bearings
    for sx in (-1, 1):
        cx = sx * P.SCREW_X
        s += at(Cylinder(P.B608_OD / 2 + 5, z1 - z0), cx, 0, (z0 + z1) / 2)
    # rod through-holes
    for sx in (-1, 1):
        for sy in (-1, 1):
            s = hole(s, P.ROD_HOLE_D / 2, z0 - 1, z1 + 1, sx * P.ROD_X, sy * P.ROD_Y)
    # top 608 pockets
    for sx in (-1, 1):
        cx = sx * P.SCREW_X
        s = hole(s, P.B608_OD / 2 + 0.2, z1 - 7.0, z1 + 1, cx, 0)
        s = hole(s, P.ROD_HOLE_D / 2 + 0.1, z0 - 1, z1 - 7.0, cx, 0)
    # sensor-bracket bolt holes
    for bx in (-18.0, 18.0):
        s = hole(s, P.M3_CLEAR / 2, z0 - 1, z1 + 1, bx, 0)
    # wire-mast anchor holes at the four corners
    for sx in (-1, 1):
        for sy in (-1, 1):
            s = hole(s, P.M3_CLEAR / 2, z0 - 1, z1 + 1,
                     sx * (P.ROD_X + 11.0), sy * P.ROD_Y)
    # wire channel on the underside: centre -> (0,68) -> (116,68) -> mast
    ch = at(Box(P.WIRE_CH_W, 71, P.WIRE_CH_D), 0, 35.5, z0 + P.WIRE_CH_D / 2)
    ch += at(Box(116, P.WIRE_CH_W, P.WIRE_CH_D), 58, 71, z0 + P.WIRE_CH_D / 2)
    ch += at(Box(P.WIRE_CH_W, 10, P.WIRE_CH_D), 116, 73, z0 + P.WIRE_CH_D / 2)
    s -= ch
    return s


# --------------------------------------------------------------------------- sensor bracket
def sensor_bracket():
    """Rigid downward bracket bridging the centre of the top frame (TCRT5000)."""
    bridge = at(Box(44, 16, 8), 0, 0, P.Z_TOP_BOT - 4)
    post = at(Box(16, 16, 22), 0, 0, 211)
    pad = at(Box(36, 18, 6), 0, 0, P.SENSOR_PAD_Z + 3)
    s = bridge + post + pad
    for bx in (-18.0, 18.0):
        s = hole(s, P.M3_CLEAR / 2, P.Z_TOP_BOT - 8, P.Z_TOP_BOT, bx, 0)
    # optical window + module mounting holes
    s = hole(s, 5.0, P.SENSOR_PAD_Z - 1, P.SENSOR_PAD_Z + 7, 0, 0)
    for dx in (-P.SENSOR_HOLES / 2, P.SENSOR_HOLES / 2):
        s = hole(s, P.M3_CLEAR / 2, P.SENSOR_PAD_Z - 1, P.SENSOR_PAD_Z + 7, dx, 0)
    # wire channel up the rear face of the post, continuing into the top frame
    s -= at(Box(P.WIRE_CH_W, P.WIRE_CH_D, 38), 0, 8.5, 213)
    return s


def wire_clip():
    """Legacy helper kept for reference; the wire mast replaces rod clips."""
    zc = 0.0
    s = at(Cylinder(6, 8), 0, 0, zc)
    s -= at(Cylinder(P.ROD_HOLE_D / 2 + 0.3, 10), 0, 0, zc)
    s += at(Box(14, 5, 4), 8, 0, zc)
    s -= at(Box(9, 7, 2.4), 11, 0, zc)
    return s


def axle_m8(length=26.0):
    head = at(Cylinder(7.0, 4.0), 0, 0, -2.0)
    shank = at(Cylinder(4.0, length), 0, 0, length / 2 - 4.0)
    nut = at(Cylinder(6.5, 5.0), 0, 0, length - 6.5)
    return head + shank + nut


# --------------------------------------------------------------------------- purchased parts
def bearing_608():
    s = at(Cylinder(P.B608_OD / 2, P.B608_H), 0, 0, 0)
    s -= at(Cylinder(P.B608_OD / 2 - 2.5, P.B608_H), 0, 0, 0)
    s += at(Cylinder(P.B608_OD / 2 - 2.6, P.B608_H - 1.6), 0, 0, 0)
    s -= at(Cylinder(P.B608_ID / 2, P.B608_H + 1), 0, 0, 0)
    return s


def bearing_lm8uu():
    s = at(Cylinder(P.LM8_OD / 2, P.LM8_L), 0, 0, 0)
    s -= at(Cylinder(P.LM8_ID / 2, P.LM8_L + 1), 0, 0, 0)
    s -= at(Cylinder(P.LM8_OD / 2 - 1.0, 3.0), 0, 0, 0)
    s -= at(Cylinder(P.LM8_OD / 2 - 1.0, 3.0), 0, 0, P.LM8_L / 2 - 1.5)
    s -= at(Cylinder(P.LM8_OD / 2 - 1.0, 3.0), 0, 0, -(P.LM8_L / 2 - 1.5))
    return s


def gt2_pulley(bore):
    s = at(Cylinder(P.GT2_OD / 2, P.PULLEY_H), 0, 0, 0)
    for zz in (P.PULLEY_H / 2 - 0.6, -(P.PULLEY_H / 2 - 0.6)):
        s += at(Cylinder(P.GT2_FLANGE_D / 2, 1.2), 0, 0, zz)
    s -= at(Cylinder(bore / 2, P.PULLEY_H + 4), 0, 0, 0)
    s -= at(Cylinder(bore / 2 + 1.6, 6), 0, 0, 8)
    return s


def guide_rod(z0, z1):
    return at(Cylinder(P.ROD_D / 2, z1 - z0), 0, 0, (z0 + z1) / 2)


def leadscrew(z0, z1):
    return at(Cylinder(P.SCREW_D / 2, z1 - z0), 0, 0, (z0 + z1) / 2)


def t8_nut():
    s = at(Cylinder(P.NUT_BODY_D / 2, P.NUT_BODY_L), 0, 0, -P.NUT_BODY_L / 2)
    s += at(Cylinder(P.NUT_FLANGE_D / 2, P.NUT_FLANGE_T), 0, 0, P.NUT_FLANGE_T / 2)
    s -= at(Cylinder(P.SCREW_D / 2 - 0.6, P.NUT_BODY_L + P.NUT_FLANGE_T + 2), 0, 0, 0)
    for dx in (-P.NUT_HOLE_SQ / 2, P.NUT_HOLE_SQ / 2):
        for dy in (-P.NUT_HOLE_SQ / 2, P.NUT_HOLE_SQ / 2):
            s -= at(Cylinder(P.M3_CLEAR / 2, P.NUT_FLANGE_T + 2), dx, dy, 1)
    return s


def nema17_motor():
    body = at(Box(P.NEMA_BODY, P.NEMA_BODY, P.MOTOR_BODY_H), 0, 0, -P.MOTOR_BODY_H / 2)
    boss = at(Cylinder(P.NEMA_BOSS_D / 2, 2), 0, 0, 1)
    shaft = at(Cylinder(P.MOTOR_SHAFT_D / 2, P.MOTOR_SHAFT_L), 0, 0, P.MOTOR_SHAFT_L / 2)
    s = body + boss + shaft
    for dx in (-P.NEMA_BOLT / 2, P.NEMA_BOLT / 2):
        for dy in (-P.NEMA_BOLT / 2, P.NEMA_BOLT / 2):
            s -= at(Cylinder(1.5, 8), dx, dy, 0)
    return s


def m3_bolt(length=16.0):
    """Origin is the seating face under the head; the shank runs in -Z."""
    head = at(Cylinder(2.75, 3.0), 0, 0, 1.5)
    shank = at(Cylinder(1.5, length), 0, 0, -length / 2)
    return head + shank


def gt2_belt():
    """Closed belt loop around the motor, both screw pulleys and the tensioner
    idler (the idler pushes the front span outward to take up slack)."""
    circles = [
        (P.MOTOR_X, P.MOTOR_Y, P.GT2_RP),
        (-P.SCREW_X, P.SCREW_Y, P.GT2_RP),
        (P.SCREW_X, P.SCREW_Y, P.GT2_RP),
        (0.0, -34.0, 11.0),                 # 608ZZ idler
    ]
    hull = make_hull([(Pos(cx, cy) * Circle(r)).edge() for cx, cy, r in circles])
    outer = offset(hull, amount=P.BELT_T / 2)
    inner = offset(hull, amount=-P.BELT_T / 2)
    return extrude(outer - inner, P.BELT_W)
