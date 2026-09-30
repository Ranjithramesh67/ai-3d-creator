"""
Parametric definition for a miniature, fully synchronized dual-screw vertical lift table.

Coordinate system
-----------------
  X : right (screw-to-screw direction)
  Y : back / rear (motor end is +Y)
  Z : up

All dimensions in millimetres.  Every value below is a documented design decision;
see README.md for the rationale and the corresponding commercial hardware.
"""


class P:
    # ------------------------------------------------------------------ frame
    EXT = 20.0                      # 2020 aluminium extrusion, 20 x 20 mm
    FX = 260.0                      # frame outer envelope, X
    FY = 200.0                      # frame outer envelope, Y
    RAIL_X = FX / 2 - EXT / 2       # side-rail centreline  = 120
    RAIL_Y = FY / 2 - EXT / 2       # front/rear centreline = 90
    RAIL_XLEN = FX                  # front & rear rails run the full X width
    RAIL_YLEN = FY - 2 * EXT        # side rails butt between them
    Z_RAIL_TOP = EXT                # rails lie flat: z = 0 .. 20

    # ------------------------------------------------------------ vertical shafts
    ROD_D = 8.0                     # 8 mm hardened smooth rod
    ROD_X = 105.0                   # guide-rod columns (outer corners)
    ROD_Y = 75.0
    ROD_HOLE_D = 8.2                # blind-hole for the rod (spec)
    ROD_HOLE_DEPTH = 15.0           # spec

    SCREW_X = 80.0                  # T8 lead-screw columns (left/right centrelines)
    SCREW_Y = 0.0
    SCREW_D = 8.0

    # ---------------------------------------------------------------- corner block
    CB_SIZE = 36.0
    CB_Z0 = Z_RAIL_TOP              # 20
    CB_Z1 = Z_RAIL_TOP + 18.0       # 38  -> top of the exposed guide rod

    # ---------------------------------------------------------------- motor / belt
    MOTOR_X = 0.0
    MOTOR_Y = 56.0                  # motor body clears the rear rail (y >= 80)
    NEMA_BODY = 42.3                # NEMA17 square body
    NEMA_BOLT = 31.0                # NEMA17 bolt pattern (31 mm)
    NEMA_BOSS_D = 22.0              # NEMA17 pilot boss
    MOTOR_BODY_H = 40.0
    MOTOR_SHAFT_D = 5.0
    MOTOR_SHAFT_L = 24.0
    Z_MOTOR_FACE = 42.0             # mounting face / top of the motor body
    GT2_TEETH = 20                  # 20T GT2 everywhere -> 1:1
    GT2_PITCH = 2.0
    GT2_RP = GT2_TEETH * GT2_PITCH / (2 * 3.141592653589793)   # pitch radius 6.366
    GT2_OD = 14.0
    GT2_FLANGE_D = 16.0
    PULLEY_H = 11.0
    BELT_T = 1.5                    # belt thickness
    BELT_W = 6.0                    # belt width
    Z_BELT = 56.0                   # belt / pulley mid-plane

    # ---------------------------------------------------------------- carriage
    BED_X = 118.0                   # half-size X (236 total)
    BED_Y = 88.0                    # half-size Y (176 total)
    BED_T = 6.0                     # minimum plate thickness (spec)
    RIB_T = 3.0                     # rib layer on top -> 9 mm total
    RIB_W = 4.0
    LM8_BORE = 15.15                # CRITICAL clearance (spec)
    LM8_BOSS_OD = 24.0
    LM8_UP = 9.0                    # boss above plate top
    LM8_DOWN = 16.0                 # boss below plate bottom
    LM8_RETAIN = 1.0                # bottom lip that carries the bearing
    LM8_RETAIN_D = 13.0
    ZIP_GROOVE_W = 3.0              # two horizontal zip-tie slots (spec)
    ZIP_GROOVE_D = 1.4
    RELIEF_W = 1.6                  # vertical relief notch so the clamp can flex

    NUT_BODY_D = 16.0               # T8 anti-backlash nut (assumed standard)
    NUT_BODY_L = 22.0
    NUT_FLANGE_D = 30.0
    NUT_FLANGE_T = 5.0
    NUT_HOLE_SQ = 20.0              # 4 x M3 on a 20 mm square
    NUT_CLEAR_D = 17.0
    NUT_SLOT_LEN = 5.2              # 3.2 hole + 1 mm adjustment each way

    # ---------------------------------------------------------------- top frame
    Z_TOP_BOT = 230.0
    Z_TOP_TOP = 240.0
    SENSOR_PAD_Z = 194.0            # underside of the sensor pad
    SENSOR_HOLES = 27.0             # TCRT5000 module bolt spacing (assumed)
    WIRE_CH_W = 6.0
    WIRE_CH_D = 3.0

    # ---------------------------------------------------------------- hardware
    M3_CLEAR = 3.2                  # every M3 through hole (spec)
    NUT_SLOT_W = 5.6                # M3 square-nut capture slot (spec)
    NUT_SLOT_H = 2.4                # M3 square-nut capture slot (spec)
    B608_OD = 22.0
    B608_ID = 8.0
    B608_H = 7.0
    LM8_OD = 15.0
    LM8_ID = 8.0
    LM8_L = 24.0

    # ---------------------------------------------------------------- kinematics
    T8_LEAD = 8.0                   # T8x8, 4-start trapezoidal, 2 mm pitch
    MICROSTEPS = 16                 # typical
    MOTOR_STEPS = 200               # 1.8 deg

    # ---------------------------------------------------------------- travel
    Z_BED_MIN = 82.0
    Z_BED_MAX = 180.0


PI = 3.141592653589793


def deg(a):
    return a * 180.0 / PI


def rad(a):
    return a * PI / 180.0
