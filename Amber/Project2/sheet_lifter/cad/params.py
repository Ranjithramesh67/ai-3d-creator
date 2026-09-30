"""
Parametric definition for a 3-ton automatic sheet-metal stack lifting machine.

Coordinate system
-----------------
  X : machine width  (left jack at -X, right jack at +X)
  Y : machine depth  (front is -Y, sensor-mast / operator side)
  Z : up             (floor at z = 0)

All dimensions in millimetres.  Structural sections follow IS 808 (ISMC) and
HIWIN HGR35 catalogue values; fasteners follow IS 1367.  Every value here is a
documented design decision and is consumed by parts.py / build.py and by the web
cinematic viewer (scene.json).
"""

import math


class P:
    # ============================================================== base frame
    BASE_OX = 1400.0                # outer envelope, X
    BASE_OY = 1200.0                # outer envelope, Y

    # ISMC 200 channel (IS 808): depth x flange, web, flange
    ISMC200_H = 200.0
    ISMC200_B = 75.0
    ISMC200_TW = 6.1
    ISMC200_TF = 9.0

    # ISMC 150 channel (IS 808)
    ISMC150_H = 150.0
    ISMC150_B = 75.0
    ISMC150_TW = 5.0
    ISMC150_TF = 7.5

    PAD_T = 20.0                    # precision-milled rail mounting pads
    PAD_SIZE = 140.0                # rail pad along the rail axis (X)
    PAD_SIZE_Y = 74.0               # rail pad across, sized into the post web
    ANCHOR_HOLE_D = 18.0            # M16 floor anchor bolts
    WELD_R = 6.0                    # representative MIG fillet bead radius

    # rigid rail support substructure (carries the 4 vertical HGR35 rails)
    RAIL_SUPPORT_W = 110.0          # RHS cross-beam width (Y)
    RAIL_SUPPORT_T = 8.0            # RHS wall thickness
    RAIL_LAND_T = 6.0               # machined rail mounting land on the beam
    POST_W = 90.0                   # vertical RHS support column behind each rail
    POST_T = 8.0                    # column wall thickness
    POST_GAP = 3.0                  # clearance between rail flange and column face
    POST_SHIFT = 15.0               # column shifted outboard to clear the pad
    MID_BEAM_W = 200.0              # mid cross-member carrying the jack stools

    # =========================================================== linear guides
    RAIL_X = 460.0                  # HGR35 columns, rail centreline
    RAIL_Y = 460.0
    DECK_FRAME = 410.0              # moving-deck tie frame, inboard of the rails
    TRACK_Y = 425.0                 # T-slot track centreline, on the deck frame
    HGR35_W = 34.0                  # rail width
    HGR35_H = 28.0                  # rail height above pad
    HGR35_LEN = 0.0                 # set below: rail top = sheet-stack top at home
    BLOCK_W = 100.0                 # HGW35CC flange block
    BLOCK_H = 48.0
    BLOCK_L = 112.0
    WIPER_T = 6.0                   # red polyurethane end wipers

    # ============================================================ lifting bed
    BED = 1000.0                    # 1000 x 1000 platform
    BED_T = 16.0                    # machined top plate
    RIB_H = 150.0                   # ISMC 150 rib depth
    RIB_COUNT = 4                   # each direction (crossed grid)
    Z_BED_HOME = 620.0              # bed top plate underside at home
    MICRO_STEP = 0.6                # drive resolution per step

    # adjustable sheet guides (T-slot track + cam-lever slider)
    FENCE_T = 12.0
    FENCE_H = 120.0

    # ================================================ sheet size adjustability
    SHEET_W_MIN = 400.0             # adjustable side-guide sheet width range (X)
    SHEET_W_MAX = 800.0
    SHEET_D_MIN = 400.0             # adjustable rear-stop sheet depth range (Y)
    SHEET_D_MAX = 750.0
    SIZE_STEP = 50.0                # engraved size graduation
    STACK_W = 800.0                 # nominal sheet size used for the demo stack
    STACK_D = 750.0

    # aluminium T-slot track the guides ride on (30-series profile)
    TSLOT_W = 30.0                  # track width across (Y)
    TSLOT_H = 24.0                  # track height
    TSLOT_LEN = 940.0               # track length along the deck member
    TSLOT_MOUTH = 12.0              # slot opening at the top
    TSLOT_BORE = 20.0               # undercut slot width (T head)
    TSLOT_LIP = 8.0                 # slot depth to the undercut
    TSLOT_GRAD = 50.0               # engraved size graduation pitch
    TSLOT_BACK_X = 425.0            # back-stop slider x (rides the Y tracks)
    RISER_S = 40.0                  # riser block under the raised Y tracks

    # cam-lever slider that clamps the guide to the track
    SLIDER_W = 66.0                 # slider body (along track)
    SLIDER_D = 40.0                 # slider body (across track)
    SLIDER_H = 20.0                 # slider body height above the track
    TSLOT_TNUT_D = 19.0             # T-nut across flats
    TSLOT_TNUT_H = 14.0

    # ======================================================= pick indexing
    PICK_SHEETS_PER_STEP = 5        # bed re-indexes after every 5 sheets

    # ===================================================== screw jacks / screws
    JACK_X = 500.0                  # both jacks at the bed side midpoints
    SCREW_D = 60.0                  # Tr60x9 trapezoidal screw (major dia)
    SCREW_PITCH = 9.0
    SCREW_CORE_D = 50.5             # Tr60x9 root diameter (thread depth 4.75)
    SCREW_THREAD_TURNS_SEG = 10     # whole turns per instanced thread segment
    # Rotating leadscrew: the screw is axially retained by thrust bearings in the
    # jack and only turns; the load is carried by a travelling bronze nut bolted
    # to the moving deck, so the screw must span the nut's whole index stroke.
    SCREW_BOTTOM = 100.0            # lower end (thrust/journal below the jack)
    SCREW_TOP = 980.0               # upper end (clears the nut at full index)
    SCREW_LEN = SCREW_TOP - SCREW_BOTTOM
    JACK_Z = 335.0                  # jack input centreline: set so the motor foot
    #                                 lands on the rear frame / support beam and
    #                                 the motor shell clears the frame below it
    JACK_BODY = 190.0               # rotating-screw worm gear jack housing
    JACK_RATIO = 24.0               # worm ratio (i)
    # jack hold-down stool (bolts the jack housing down to a mid beam)
    JACK_STOOL_W = 220.0            # stool plate along X (clears the sensor mast)
    JACK_STOOL_W_Y = 320.0          # stool plate across Y (hosts the L-clamp feet)
    JACK_STOOL_T = 16.0             # plate thickness
    JACK_CLAMP_T = 14.0             # L-clamp angle thickness
    MID_BEAM_Y = 0.0                # mid cross-member under the two jacks
    MID_BEAM_GAP = 430.0            # central opening clears the gearbox pedestal

    # travelling bronze leadscrew nut, bolted to a bracket on the moving deck.
    # It rides the rotating Tr60x9 screw and carries the 3 t bed.
    NUT_W = 170.0                   # nut block along X
    NUT_D = 140.0                   # nut block along Y
    NUT_H = 100.0                   # nut block along Z
    NUT_BORE_D = 50.5               # threaded bore root diameter (Tr60x9)
    NUT_Z_OFF = 71.0                # nut centre below the deck-top plane at home
    NUT_BOLT_D = 13.0               # M12 clearance bolts up into the bracket
    NUT_BRACKET_T = 16.0            # bracket plate thickness
    NUT_BRACKET_W = 220.0           # bracket shoe along Y at the frame face

    # guided vertical drag-chain trough.  It carries a self-supporting C-loop
    # energy chain between a fixed bracket on the base frame and a moving bracket
    # on the deck, so no part of the chain hangs in mid-air.
    TROUGH_X = 575.0                # trough centre in X (deck frame .. base side)
    TROUGH_DEPTH = 62.0             # trough internal depth (X)
    TROUGH_W = 200.0                # trough internal width (Y)
    TROUGH_T = 12.0                 # channel wall thickness
    TROUGH_Y = -275.0               # trough centre in Y (clear of the +X posts)
    TROUGH_Z0 = 150.0               # trough bottom
    TROUGH_Z1 = 950.0               # trough top (above the full index position)
    CHAIN_RUN_DY = 45.0             # half separation of the two vertical runs
    CHAIN_R = 45.0                  # chain bend radius
    CHAIN_LINK = 45.0               # chain link pitch
    CHAIN_W = 55.0                  # chain link width across (X)
    CHAIN_T = 30.0                  # chain link thickness (normal to the run)
    CHAIN_FIXED_Z = 450.0           # fixed end bracket height on the trough

    # ============================================================== powertrain
    MOTOR_KW = 3.7
    MOTOR_D = 230.0                 # 3.7 kW IE3 frame (132 frame, stubby body)
    MOTOR_L = 330.0                 # finned stator shell length (compact, real frame)
    MOTOR_SHAFT_D = 38.0
    MOTOR_SHAFT_L = 110.0           # output shaft length (enters the coupling)
    CLUTCH_D = 120.0                # zero-backlash elastomer jaw coupling
    CLUTCH_L = 140.0
    BRAKE_D = 180.0                 # rear electromagnetic disc brake
    BRAKE_T = 60.0

    GEARBOX = 260.0                 # 1:1 T-type spiral bevel gearbox
    SHAFT_D = 30.0                  # Ø30 keyed horizontal drive shafts
    KEY_W = 8.0
    KEY_H = 7.0

    # ======================================================== sensor mast
    MAST_X = 660.0                  # on the +X base member (|x| 625-700)
    MAST_D = 80.0
    MAST_Z = 1600.0                 # mast top (above the tallest stack)
    MAST_ARM = 192.0                # cantilever arm reaching over the deck edge
    LASER_D = 70.0                  # industrial laser distance sensor
    LASER_L = 120.0

    # ====================================================== electrical cabinet
    CAB_W = 600.0                   # 600 wide (Y)
    CAB_D = 250.0                   # 250 deep (X)
    CAB_H = 800.0
    CAB_Z = 100.0                   # plinth height
    E_STOP_D = 60.0
    HMI_W = 180.0                   # 7-inch panel PC
    HMI_H = 130.0
    CAB_X = 1000.0                  # adjacent to the base
    CAB_Y = -100.0

    # =============================================================== payload
    SHEET_T = 0.65                  # 0.6 - 0.7 mm galvanised sheet
    STACK_H = 300.0                 # ~460 sheets

    # ================================================================ hardware
    M16_CLEAR = 18.0
    M12_CLEAR = 13.0
    M12_BOLT = 12.0
    M10_CLEAR = 11.0

    # =============================================================== kinematics
    START_TIME = 0.0                # 15 s cinematic timeline
    DURATION = 15.0


PI = 3.141592653589793


# ---------------------------------------------------------------------------
# Derived guide-rail sizing.
#
# The HGR35 rails are deliberately cut down so their tops finish level with the
# top of the sheet stack at the bed-home position -- the guides are never taller
# than the sheets they carry, instead of towering ~1.5 m over an open deck.
# The carriage (HGW35 block) is then offset below the bed top so that it stays
# fully engaged on the rail over the entire index stroke and never runs off the
# rail top at the end of the stack.
# ---------------------------------------------------------------------------
P.MAX_BED_INDEX = (math.ceil((P.STACK_H / P.SHEET_T) / P.PICK_SHEETS_PER_STEP)
                   * P.SHEET_T * P.PICK_SHEETS_PER_STEP)
P.HGR35_LEN = (P.Z_BED_HOME + P.STACK_H
               - (P.ISMC200_H + P.RAIL_LAND_T + P.PAD_T))
P.BLOCK_Z_OFFSET = P.BLOCK_L / 2 + P.MAX_BED_INDEX - P.STACK_H


# ---------------------------------------------------------------------------
# Derived powertrain layout.  The 1:1 T-type bevel gearbox outputs along +/-X to
# the two worm-gear jacks; its input train stacks coaxially along +Y off the
# gearbox input boss (gearbox -> jaw coupling -> disc brake -> motor face) so the
# motor shaft always enters the coupling and nothing passes through the motor.
# Kept here (not in build.py) so parts.py can place the gearbox/motor pedestals.
# ---------------------------------------------------------------------------
P.GB_OUT_END = P.GEARBOX / 2 + 30 + 45                  # end of each output boss
P.JACK_IN_END = P.JACK_X - (P.JACK_BODY / 2 + 40 + 65)  # jack input face
P.SHAFT_LEN = P.JACK_IN_END - P.GB_OUT_END
P.SHAFT_X = (P.GB_OUT_END + P.JACK_IN_END) / 2
P.GB_IN_END = P.GEARBOX * 0.4 + 30 + 45                 # gearbox input boss end
P.COUPLING_Y = P.GB_IN_END + P.CLUTCH_L / 2             # jaw coupling centre
P.BRAKE_Y = P.COUPLING_Y + P.CLUTCH_L / 2 + P.BRAKE_T / 2   # disc brake centre
P.MOTOR_FACE_Y = P.BRAKE_Y + P.BRAKE_T / 2              # motor mounting face
P.MOTOR_FOOT_Z = P.JACK_Z - P.MOTOR_D / 2 - 20        # underside of motor foot
P.MOTOR_FOOT_Y = P.MOTOR_FACE_Y + P.MOTOR_L / 2       # motor foot centre
P.GB_FOOT_Z = P.JACK_Z - P.GEARBOX * 0.4 - 8 - 12     # underside of gearbox feet
P.JACK_BOTTOM = P.JACK_Z - P.JACK_BODY * 0.95 / 2     # underside of jack housing
P.JACK_STOOL_H = P.JACK_BOTTOM - (P.ISMC200_H + P.RAIL_LAND_T)   # mid-beam to jack

# motor foot pedestal: two saddles straddle the rear ISMC200 member (whose top is
# flush with the foot at z = ISMC200_H), so the front and rear overhang are both
# taken to the floor without a solid block passing through the frame member.
P.MOTOR_SUP_X = 120.0                                 # saddle x, both sides
P.MOTOR_SUP_W = 70.0                                  # saddle square section
P.MOTOR_SUP_FRONT_Y = P.MOTOR_FACE_Y + 60.0           # front saddle under the foot
P.MOTOR_SUP_REAR_Y = P.MOTOR_FOOT_Y + 116.0           # rear saddle beyond the frame

# travelling bronze nut: everything below is expressed in the deck-local frame
# (origin at the deck top plane, so world z = Z_BED_HOME + local z).
P.NUT_Z_HOME = P.Z_BED_HOME - P.NUT_Z_OFF             # nut centre at home
P.NUT_TOP_LOCAL = -P.NUT_Z_OFF + P.NUT_H / 2          # nut top face (deck-local)
P.NUT_BOT_LOCAL = -P.NUT_Z_OFF - P.NUT_H / 2          # nut bottom face
P.NUT_TRAVEL = P.MAX_BED_INDEX                         # nut travel over the stroke

# drag-chain C-loop: fixed end on the trough, moving end on the deck.  The bend
# rises as the deck rises so the loop length stays constant.
P.CHAIN_Z_M_HOME = P.Z_BED_HOME                        # moving bracket at home
P.CHAIN_ZB_HOME = P.CHAIN_FIXED_Z - 200.0              # bend apex at home


def deg(a):
    return a * 180.0 / PI


def rad(a):
    return a * PI / 180.0
