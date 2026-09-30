"""
Industrial hardware part generators for the 3-ton sheet-lifting machine.

Parts are built in world coordinates at the machine *home* position.  build.py
unions them, tags them static/lift for the viewer, validates the kinematics and
exports STEP/STL plus a scene.json manifest.
"""

from build123d import (
    Align, Box, Circle, Compound, Cylinder, Helix, Polygon, Pos, Rectangle,
    Rot, Plane, Torus, Axis, extrude, make_face, sweep,
)
import math
import params as PP
from profiles import at, bolt, nut, washer, hex_prism, disc, pipe

P = PP.P


def cyl(d, length, axis="Z"):
    c = Cylinder(d / 2, length)
    if axis == "X":
        c = Rot(0, 90, 0) * c
    elif axis == "Y":
        c = Rot(-90, 0, 0) * c
    return c


def bbox(shape):
    b = shape.bounding_box()
    return (b.max.X - b.min.X, b.max.Y - b.min.Y, b.max.Z - b.min.Z)


# =============================================================== base frame
def channel_x(length, back_y, open_dir):
    """ISMC 200 running along X. Web at back_y, flanges open toward +open_dir*Y."""
    h, b, tw, tf = P.ISMC200_H, P.ISMC200_B, P.ISMC200_TW, P.ISMC200_TF
    s = at(Box(length, tw, h), 0, back_y + open_dir * tw / 2, h / 2)
    s += at(Box(length, b, tf), 0, back_y + open_dir * b / 2, h - tf / 2)
    s += at(Box(length, b, tf), 0, back_y + open_dir * b / 2, tf / 2)
    return s


def channel_y(length, back_x, open_dir):
    """ISMC 200 running along Y. Web at back_x, flanges open toward +open_dir*X."""
    h, b, tw, tf = P.ISMC200_H, P.ISMC200_B, P.ISMC200_TW, P.ISMC200_TF
    s = at(Box(tw, length, h), back_x + open_dir * tw / 2, 0, h / 2)
    s += at(Box(b, length, tf), back_x + open_dir * b / 2, 0, h - tf / 2)
    s += at(Box(b, length, tf), back_x + open_dir * b / 2, 0, tf / 2)
    return s


def base_frame():
    """Welded ISMC 200 perimeter frame with lapped, gusseted corner joints.

    The side members run the full depth so every corner is a solid lap joint
    (no open butt gap), reinforced by a top and bottom gusset plate welded over
    each corner and MIG fillet beads along the seams.
    """
    hx, hy = P.BASE_OX / 2, P.BASE_OY / 2
    s = channel_x(P.BASE_OX, -hy, +1)          # front
    s += channel_x(P.BASE_OX, +hy, -1)         # rear
    s = s + channel_y(P.BASE_OY, -hx, +1)      # left  (full length -> lap)
    s = s + channel_y(P.BASE_OY, +hx, -1)      # right (full length -> lap)

    g = 150.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            gx = sx * (hx - g / 2)
            gy = sy * (hy - g / 2)
            # top and bottom corner gusset plates bridging both members
            s += at(Box(g, g, 10), gx, gy, P.ISMC200_TF + 5)
            s += at(Box(g, g, 10), gx, gy, P.ISMC200_H - P.ISMC200_TF - 5)
            # MIG fillet beads along the vertical seam of the lap
            for z0 in (P.ISMC200_TF + 22, P.ISMC200_H - P.ISMC200_TF - 22):
                s += at(Cylinder(P.WELD_R, 30), gx, gy, z0)
    return s


def hgr35_rail():
    """HIWIN HGR35 profile rail, vertical, origin at the rail base centre.

    The 34 mm ball head faces -Y and a 48 mm mounting flange with a row of
    Ø9 bolt holes faces +Y, so the rail bolts back onto a machined column face
    rather than sitting on free pads.
    """
    w, L = P.HGR35_W, P.HGR35_LEN
    s = at(Box(w, 20, L), 0, 0, L / 2)                 # head
    s -= at(Box(w * 0.5, 9, L + 2), 0, -8, L / 2)      # running ball groove
    s += at(Box(48, 14, L), 0, 17, L / 2)              # base mounting flange (+Y)
    for i in range(int(L // 125) + 1):                 # mounting bolt holes
        z = 62 + i * 125
        if z > L - 40:
            break
        s -= at(cyl(9, 26, "Y"), 0, 17, z)
    return s


def rail_support():
    """Rigid substructure carrying the four HGR35 rails and the two jack stools.

    Two 200 mm deep RHS cross-beams span the machine between the ISMC200 side
    members, flush with the frame top, each closed by bolted end plates and
    carrying precision-machined rail mounting lands.  A third mid cross-member at
    y=0 carries the worm-gear jack stools; it is split by a central opening so
    the gearbox pedestal can drop to the floor, and is drilled for the two
    screws to pass through.
    """
    L = P.BASE_OX - 2 * P.ISMC200_B                    # 1250 between side webs
    W, T, H = P.RAIL_SUPPORT_W, P.RAIL_SUPPORT_T, P.ISMC200_H
    s = None
    for sy in (-1, 1):
        y = sy * P.RAIL_Y
        beam = at(Box(L, W, H), 0, y, H / 2) - at(Box(L - 2 * T, W - 2 * T,
                                                       H - 2 * T), 0, y, H / 2)
        # end plates bolted to the side-member webs
        for sx in (-1, 1):
            beam += at(Box(16, W + 36, H + 36), sx * (L / 2 + 8), y, H / 2)
            for dz in (-1, 1):
                beam -= at(cyl(11, 20, "Y"), sx * (L / 2 + 8),
                           y + dz * (W / 2 - 12), H / 2)
        # machined rail land under each pad
        for sx in (-1, 1):
            beam += at(Box(P.PAD_SIZE, W, P.RAIL_LAND_T), sx * P.RAIL_X, y,
                       H + P.RAIL_LAND_T / 2)
        s = beam if s is None else s + beam

    # mid cross-member at y=0 under the two jack stools
    mw = P.MID_BEAM_W
    mid = at(Box(L, mw, H), 0, P.MID_BEAM_Y, H / 2) \
        - at(Box(L - 2 * T, mw - 2 * T, H - 2 * T), 0, P.MID_BEAM_Y, H / 2)
    mid -= at(Box(P.MID_BEAM_GAP, mw + 40, H + 40), 0, P.MID_BEAM_Y, H / 2)
    for sx in (-1, 1):
        mid += at(Box(16, mw + 36, H), sx * (L / 2 + 8), P.MID_BEAM_Y,
                  H / 2)
        for dz in (-1, 1):
            mid -= at(cyl(11, 20, "Y"), sx * (L / 2 + 8),
                      P.MID_BEAM_Y + dz * (mw / 2 - 12), H / 2)
    for sx in (-1, 1):                                   # stool bolt pads
        mid += at(Box(P.JACK_STOOL_W, P.JACK_STOOL_W_Y, P.RAIL_LAND_T),
                  sx * P.JACK_X, P.MID_BEAM_Y, H + P.RAIL_LAND_T / 2)
    for sx in (-1, 1):                                   # screw clearance through
        mid -= at(cyl(70, H + 60, "Z"), sx * P.JACK_X, P.MID_BEAM_Y, H / 2)
    s = s + mid
    return s


def rail_post():
    """Four vertical RHS support columns that back the HGR35 guide rails.

    Replaces the earlier tilted diagonal buttress struts.  Each rail is rotated
    so its 48 mm mounting flange faces outboard (+/-X); the flange bolts to the
    machined inner face of a plumb column whose foot lands on the rail-support
    cross-beam.  Nothing in the machine is tilted, and every rail is backed over
    its full height with the carriage block free to run between the columns.
    """
    rail_face = P.HGR35_W / 2 + 7.0            # centreline -> flange outer face
    base = P.ISMC200_H
    top = P.ISMC200_H + P.RAIL_LAND_T + P.PAD_T + P.HGR35_LEN
    depth = top - base
    W, T = P.POST_W, P.POST_T
    s = None
    for sx in (-1, 1):
        x = sx * (P.RAIL_X + rail_face + P.POST_GAP + P.POST_SHIFT + W / 2)
        for sy in (-1, 1):
            y = sy * P.RAIL_Y
            col = at(Box(W, W, depth), x, y, base + depth / 2)
            col -= at(Box(W - 2 * T, W - 2 * T, depth + 2), x, y,
                      base + depth / 2)
            col += at(Box(W + 50, W + 50, 16), x, y, base + 8)   # foot plate
            col += at(Box(W + 40, W + 40, 14), x, y, top - 7)    # cap plate
            # clearance pocket so the precision rail pad and land pass the
            # column base without touching (column stays full height above)
            col -= at(Box(P.PAD_SIZE + 20, P.PAD_SIZE + 90, 110),
                      sx * P.RAIL_X, y, P.RAIL_LAND_T + P.ISMC200_H + 24)
            s = col if s is None else s + col
    return s


def hgw35_block():
    """HGW35CC flanged heavy-load block with red PU wiper seals. Origin at rail axis."""
    w, hh, L = P.BLOCK_W, P.BLOCK_H, P.BLOCK_L
    s = at(Box(w, hh, L), 0, 0, 0)
    s -= at(Box(P.HGR35_W + 0.4, hh + 2, L + 2), 0, -hh / 2 + P.HGR35_H / 2 + 0.5, 0)
    # open the back so the rail mounting flange and bolt row pass through
    s -= at(Box(56, 80, L + 2), 0, hh / 2 + 12, 0)
    for zz in (-1, 1):
        s += at(Box(w + 4, hh + 4, P.WIPER_T), 0, 0, zz * (L / 2 - P.WIPER_T / 2))
    return s


def pad_and_bolts():
    t = P.PAD_T
    s = at(Box(P.PAD_SIZE, P.PAD_SIZE_Y, t), 0, 0, t / 2)
    for dx in (-1, 1):
        for dy in (-1, 1):
            s -= at(Cylinder(P.M12_CLEAR / 2, t + 2), dx * 45, dy * 22, t / 2)
    return s


# ================================================================== lifting bed
def bed_plate():
    """1000 x 1000 x 16 machined top plate (origin at plate underside centre)."""
    return at(Box(P.BED, P.BED, P.BED_T), 0, 0, P.BED_T / 2)


def bed_ribs():
    """Crossed ISMC 150 rib network under the bed (origin at plate underside)."""
    h, b, tw, tf = P.ISMC150_H, P.ISMC150_B, P.ISMC150_TW, P.ISMC150_TF
    zc = -h / 2
    s = None
    n = P.RIB_COUNT
    span = P.BED - 2 * 40
    for i in range(n):
        x = -P.BED / 2 + 40 + (i + 0.5) * (span / n)
        seg = at(Box(tw, span, h), x, 0, zc)
        seg += at(Box(b, span, tf), x, 0, zc - h / 2 + tf / 2)
        seg += at(Box(b, span, tf), x, 0, zc + h / 2 - tf / 2)
        s = seg if s is None else s + seg
    for j in range(n):
        y = -P.BED / 2 + 40 + (j + 0.5) * (span / n)
        seg = at(Box(span, tw, h), 0, y, zc)
        seg += at(Box(span, b, tf), 0, y, zc - h / 2 + tf / 2)
        seg += at(Box(span, b, tf), 0, y, zc + h / 2 - tf / 2)
        s = s + seg
    # perimeter deck frame: ties the four HGW35CC carriage blocks together.
    # It sits inboard of the rail centrelines so the moving deck never runs into
    # the static rails; only the carriage blocks touch the frame.
    fr = P.DECK_FRAME
    for sy in (-1, 1):
        seg = at(Box(2 * fr, tw, h), 0, sy * fr, zc)
        seg += at(Box(2 * fr, b, tf), 0, sy * fr, zc - h / 2 + tf / 2)
        seg += at(Box(2 * fr, b, tf), 0, sy * fr, zc + h / 2 - tf / 2)
        s = s + seg
    for sx in (-1, 1):
        seg = at(Box(tw, 2 * fr, h), sx * fr, 0, zc)
        seg += at(Box(b, 2 * fr, tf), sx * fr, 0, zc - h / 2 + tf / 2)
        seg += at(Box(b, 2 * fr, tf), sx * fr, 0, zc + h / 2 - tf / 2)
        s = s + seg
    return s


def size_stations(lo, hi, step):
    """Discrete quick-set size stations from lo to hi inclusive."""
    n = int(round((hi - lo) / step))
    return [lo + i * step for i in range(n + 1)]


def tslot_track(length=None):
    """30-series aluminium T-slot track the adjustable guides ride on.

    Replaces the row of discrete quick-set index plates: one continuous track
    per deck member, with a narrow mouth over a wider undercut so a guide slider
    can be locked anywhere along it with a cam lever.  Shallow grooves on the top
    face are engraved size graduations.
    """
    L = P.TSLOT_LEN if length is None else length
    W, H = P.TSLOT_W, P.TSLOT_H
    s = at(Box(L, W, H), 0, 0, H / 2)
    # top slot: narrow mouth, then the wider T-nut undercut beneath it
    s -= at(Box(L + 2, P.TSLOT_MOUTH, P.TSLOT_LIP), 0, 0, H - P.TSLOT_LIP / 2)
    s -= at(Box(L + 2, P.TSLOT_BORE, P.TSLOT_LIP), 0, 0,
            H - P.TSLOT_LIP - P.TSLOT_LIP / 2)
    n = int((L / 2) / P.TSLOT_GRAD)
    for k in range(-n, n + 1):                       # engraved graduations
        s -= at(Box(1.5, 4, 1.2), k * P.TSLOT_GRAD, W / 2 - 4, H - 0.6)
    for x in (-L / 2 + 90, 0, L / 2 - 90):           # mounting holes to the deck
        s -= at(cyl(9, 14, "Z"), x, 0, 8)
    return s


def track_riser():
    """Square riser block lifting the rear/left T-slot tracks clear of the
    perpendicular tracks they cross at the deck corners."""
    S = P.RISER_S
    return at(Box(S, S, P.TSLOT_H), 0, 0, P.TSLOT_H / 2)


def guide_slider():
    """Cam-lever slider clamping a guide to the T-slot track.

    Origin at the track's top surface: the body sits on the track, a T-nut drops
    through the slot, and a knurled cam knob locks it anywhere along the track --
    no loose bolts and no discrete index plates.
    """
    W, D, H = P.SLIDER_W, P.SLIDER_D, P.SLIDER_H
    s = at(Box(W, D, H), 0, 0, H / 2)                       # body on the track
    s += at(Box(P.TSLOT_TNUT_D, 12, 8), 0, 0,
            -(P.TSLOT_H - P.TSLOT_LIP - 4))                 # T-nut head in the bore
    s += at(Box(P.TSLOT_MOUTH - 1, 10, 8), 0, 0, -4)        # stem through the mouth
    s += at(cyl(18, 24, "Z"), 0, 0, H + 12)                 # cam knob
    return s


def side_guide():
    """Adjustable centring side-guide on the +/-X side of the deck.

    A vertical blade spanning the working depth is carried by two cam-lever
    sliders riding the front/rear T-slot tracks, so the guide slides in X to any
    sheet width and locks with two flip levers.  The blade starts at the track top
    so it never runs into the tracks it is mounted on.
    """
    blade = 2 * P.TRACK_Y
    s = at(Box(P.FENCE_T, blade, P.FENCE_H), 0, 0, P.FENCE_H / 2)
    for sy in (-1, 1):
        s += at(guide_slider(), 0, sy * P.TRACK_Y, 0)
    return s


def back_guide():
    """Adjustable rear depth stop on the deck ribs / T-slot tracks.

    Slides in Y to set the sheet depth and locks with two cam levers.
    """
    blade = 2 * P.TRACK_Y
    s = at(Box(blade, P.FENCE_T, P.FENCE_H), 0, 0, P.FENCE_H / 2)
    for sx in (-1, 1):
        # sliders sit behind the blade so their bodies never overhang the sheet
        s += at(guide_slider(), sx * P.TSLOT_BACK_X,
                P.SLIDER_D / 2 - P.FENCE_T / 2, 0)
    return s


def sheet_stack():
    """Galvanised stack of ~460 x 0.65 mm sheets (origin at stack base)."""
    s = at(Box(P.STACK_W, P.STACK_D, P.STACK_H), 0, 0, P.STACK_H / 2)
    for i in range(1, 14):
        s += at(Box(P.STACK_W + 2, P.STACK_D + 2, 0.6), 0, 0, P.STACK_H * i / 14)
    return s


# ========================================================= screw jacks / screws
def screw_thread_seg(turns=None):
    """One whole-turn block of real Tr60x9 trapezoidal thread, origin at bottom.

    The thread is a true trapezoidal profile swept along a genuine helix with
    pitch 9 mm.  Each block spans a whole number of turns so identical blocks
    phase-align exactly when stacked end to end.
    """
    turns = P.SCREW_THREAD_TURNS_SEG if turns is None else turns
    pitch = P.SCREW_PITCH
    L = turns * pitch
    r_core = P.SCREW_CORE_D / 2
    r_crest = P.SCREW_D / 2
    depth = r_crest - r_core

    core = Cylinder(r_core, L, align=(Align.CENTER, Align.CENTER, Align.MIN))
    helix = Helix(pitch=pitch, height=L, radius=r_core)
    plane = Plane(origin=helix @ 0, x_dir=(1, 0, 0), z_dir=helix % 0)
    hr, hc = pitch * 0.29, pitch * 0.145          # root / crest half-widths
    profile = plane * Polygon(                      # local X = radial, Y = axial
        (-1.0, -hr), (depth, -hc), (depth, hc), (-1.0, hr), align=None)
    return core + sweep(profile, path=helix)


def trapez_screw():
    """Full 1500 mm Tr60x9 screw: a stack of whole-turn threaded blocks plus a
    plain unthreaded tip.  Returned as one compound part so it instances once
    per jack while keeping the exact thread representation cheap to rebuild."""
    pitch = P.SCREW_PITCH
    seg_len = P.SCREW_THREAD_TURNS_SEG * pitch
    n = int(P.SCREW_LEN // seg_len)
    core_r = P.SCREW_CORE_D / 2
    kids = [Pos(0, 0, i * seg_len) * screw_thread_seg() for i in range(n)]
    rem = P.SCREW_LEN - n * seg_len
    if rem > 1e-6:
        kids.append(Pos(0, 0, n * seg_len)
                    * Cylinder(core_r, rem, align=(Align.CENTER, Align.CENTER,
                                                   Align.MIN)))
    return Compound(children=kids)


def worm_gear_jack():
    """Rotating-screw 5 t worm gear jack (cast iron, origin mid-body).

    The housing is fixed to the stool and holds the leadscrew in upper and lower
    thrust bearings, so the screw only turns while a travelling bronze nut on the
    deck carries the load.  The input barrel faces the gearbox.
    """
    J = P.JACK_BODY
    s = at(Box(J, J, J * 0.95), 0, 0, 0)
    # input shaft barrel toward -Y
    s += at(cyl(90, 130, "Y"), 0, -(J / 2 + 40), 0)
    # upper / lower thrust-bearing bearing covers the screw runs in
    for dz in (-1, 1):
        s += at(cyl(J * 0.62, 22, "Z"), 0, 0, dz * (J * 0.475 + 5))
    # corner tie bolts
    for dx in (-1, 1):
        for dy in (-1, 1):
            s += at(bolt(12, 22, 19, 8), dx * (J / 2 - 22), dy * (J / 2 - 22),
                    J * 0.475 + 12)
    return s


def bronze_nut():
    """Travelling bronze leadscrew nut bolted to the moving deck.

    Origin at the block centre.  A threaded Tr60x9 bore runs right through it and
    four M12 capscrews clamp the block up to the deck bracket.  The screw turns
    inside it, so the nut travels and carries the 3 t bed.
    """
    W, D, H = P.NUT_W, P.NUT_D, P.NUT_H
    s = at(Box(W, D, H), 0, 0, 0)
    s -= at(cyl(P.NUT_BORE_D, H + 2, "Z"), 0, 0, 0)      # threaded bore
    for zz in (-1, 1):                                    # lead-in chamfers
        s -= at(Cylinder(P.NUT_BORE_D / 2 + 7, 8), 0, 0,
                zz * (H / 2 + 1))
    for dx in (-1, 1):                                    # bracket bolt holes
        for dy in (-1, 1):
            s -= at(cyl(P.NUT_BOLT_D, H + 2, "Z"), dx * (W / 2 - 28),
                    dy * (D / 2 - 28), 0)
    return s


def nut_bracket():
    """Gusseted bracket tying each travelling bronze nut to the deck tie frame.

    Built in the deck-local frame (origin on the deck top plane).  A vertical web
    bolts to the outboard face of the deck side member and a top plate reaches
    out over the jack to carry the bronze nut; a central gusset stiffens the
    cantilever.  Both sides are modelled in one part.
    """
    face = P.DECK_FRAME + P.ISMC150_B / 2      # outboard face of the side member
    T = P.NUT_BRACKET_T
    top = P.NUT_TOP_LOCAL                      # nut top -> plate underside
    s = None
    for sx in (-1, 1):
        x0 = face
        x1 = P.JACK_X + P.NUT_W / 2 + 15
        web = at(Box(T, P.NUT_BRACKET_W, 140), sx * (face + T / 2), 0, top - 70)
        plate = at(Box(x1 - x0, P.NUT_W - 40, T), sx * (x0 + x1) / 2, 0,
                   top + T / 2)
        gus = at(Box(x1 - x0 - 30, 14, 96), sx * (x0 + x1) / 2, 0, top - 48)
        br = web + plate + gus
        for dz in (-1, 1):                     # bolt holes into the member face
            br -= at(cyl(P.M12_CLEAR, T + 4, "X"), sx * (face + T / 2),
                     dz * 70, top - 70 + dz * 35)
        for dx in (-1, 1):                     # nut capscrew clearance
            for dy in (-1, 1):
                br -= at(cyl(P.NUT_BOLT_D, T + 4, "Z"),
                         sx * (P.JACK_X + dx * (P.NUT_W / 2 - 28)),
                         dy * (P.NUT_D / 2 - 28), top + T / 2)
        s = br if s is None else s + br
    return s


def jack_stool():
    """Machined stool that takes the jack housing down onto the mid beam.

    Origin at the stool underside (on the beam's bolt pads).  The screw passes
    through a clearance bore so the translating screw never touches the stool.
    """
    W, Wy, h = P.JACK_STOOL_W, P.JACK_STOOL_W_Y, P.JACK_STOOL_H
    s = at(Box(W, Wy, h), 0, 0, h / 2)
    s -= at(cyl(120, h + 2, "Z"), 0, 0, h / 2)          # screw clearance bore
    for sx in (-1, 1):                                   # clamp bolt holes
        s -= at(cyl(13, h + 2, "Z"), sx * 120, 0, h / 2)
    return s


def jack_clamp():
    """Cast hold-down L-clamp bolting one face of the jack housing to the stool.

    Origin at the inner-bottom corner: the vertical web bears on the housing
    face, the horizontal foot lands on the stool and is bolted down.
    """
    T = P.JACK_CLAMP_T
    L, H, F = 120.0, 90.0, 60.0
    s = at(Box(T, L, H), T / 2, 0, H / 2)                # vertical web on housing
    s += at(Box(F, L, T), F / 2, 0, T / 2)               # horizontal foot on stool
    s -= at(cyl(13, T + 2, "Z"), F * 0.62, 0, T / 2)     # foot bolt clearance
    return s


# ================================================================= powertrain
def electric_motor():
    """3.7 kW IE3 TEFC motor, axis along Y, origin at mounting face (front, +Y face).

    Compact finned stator shell with radial cooling ribs that stand proud of the
    frame, cast end bells, a vented fan cowl and a foot, so the body reads as a
    real motor instead of a stretched tube.
    """
    D, L = P.MOTOR_D, P.MOTOR_L
    s = at(cyl(D, L, "Y"), 0, L / 2, 0)                 # stator shell
    # radial cooling fins: short plates on the shell surface, not through the axis
    nfin, rib_h, rib_t = 28, 18.0, 7.0
    for i in range(nfin):
        a = 360.0 * i / nfin
        rib = Rot(0, a, 0) * at(Box(rib_h, L - 50, rib_t),
                                0, 0, D / 2 + rib_h / 2)
        s += at(rib, 0, L / 2, 0)
    s += at(cyl(D + 8, 34, "Y"), 0, 17, 0)              # front end bell
    s += at(cyl(D + 8, 34, "Y"), 0, L - 17, 0)          # rear end bell
    # vented fan cowl (stepped, with radial vent slots)
    cowl = at(cyl(D - 16, 8, "Y"), 0, L + 4, 0)
    cowl += at(cyl(D - 30, 48, "Y"), 0, L + 32, 0)
    for k in range(12):
        slot = Rot(0, 360.0 * k / 12, 0) * at(Box(34, 10, 26),
                                              0, 0, D / 2 - 20)
        cowl -= at(slot, 0, L + 24, 0)
    s += cowl
    s += at(cyl(P.MOTOR_SHAFT_D, P.MOTOR_SHAFT_L, "Y"), 0,
            -P.MOTOR_SHAFT_L / 2, 0)                    # output shaft
    # terminal box
    s += at(Box(130, 150, 72), 0, L / 2, D / 2 + 16)
    # foot
    s += at(Box(D + 24, L - 60, 22), 0, L / 2, -D / 2 - 9)
    return s


def disc_brake():
    """Rear-mounted electromagnetic disc brake, axis Y."""
    s = at(cyl(P.BRAKE_D, P.BRAKE_T, "Y"), 0, 0, 0)
    s -= at(cyl(P.MOTOR_SHAFT_D, P.BRAKE_T + 4, "Y"), 0, 0, 0)
    return s


def jaw_coupling():
    """Zero-backlash elastomer jaw coupling, axis Y."""
    s = at(cyl(P.CLUTCH_D, P.CLUTCH_L / 2, "Y"), 0, -P.CLUTCH_L / 4, 0)
    s += at(cyl(P.CLUTCH_D, P.CLUTCH_L / 2, "Y"), 0, P.CLUTCH_L / 4, 0)
    s += at(cyl(P.CLUTCH_D + 6, P.CLUTCH_L * 0.3, "Y"), 0, 0, 0)
    return s


def bevel_gearbox():
    """1:1 T-type spiral bevel gearbox; input along Y, outputs along +/-X."""
    G = P.GEARBOX
    s = at(Box(G, G * 0.8, G * 0.8), 0, 0, 0)
    s += at(cyl(120, 90, "Y"), 0, G * 0.4 + 30, 0)
    for sx in (-1, 1):
        s += at(cyl(110, 90, "X"), sx * (G / 2 + 30), 0, 0)
    # four cast mounting feet
    for sx in (-1, 1):
        for sy in (-1, 1):
            s += at(Box(72, 72, 24), sx * (G / 2 - 45), sy * (G * 0.4 - 40),
                    -G * 0.4 - 8)
    return s


def gearbox_mount():
    """Welded pedestal carrying the central bevel gearbox down to the floor.

    The base frame is an open perimeter ring, so without this the gearbox hung in
    mid-air between the two jack input shafts.  The cast feet bolt onto a machined
    top plate; the pedestal lands on a floor base plate.
    """
    G, top = P.GEARBOX, P.GB_FOOT_Z
    s = at(Box(380, 320, 24), 0, 0, 12)                 # floor base plate
    s += at(Box(280, 230, top - 24 - 16), 0, 0, 24 + (top - 24 - 16) / 2)
    s += at(Box(340, 300, 16), 0, 0, top - 8)           # machined top plate
    for sx in (-1, 1):
        for sy in (-1, 1):
            s += at(Box(46, 46, 8), sx * 150, sy * 130, top + 4)   # bolt pads
    return s


def drive_shaft(length):
    """Ø30 keyed horizontal drive shaft along X, centred on origin.

    The key runs along the shaft axis (X) and sits proud of the surface on top
    (+Z), so it engages the jacks' input keyways instead of cutting across them.
    """
    s = at(cyl(P.SHAFT_D, length, "X"), 0, 0, 0)
    s += at(Box(length * 0.5, P.KEY_W, P.KEY_H), 0, 0, P.SHAFT_D / 2)
    return s


# =============================================================== sensor mast
def sensor_mast():
    """Side sensor mast on the +X base member with a cantilever arm over the
    deck edge, keeping the top of the machine clear for the pick frame."""
    x = P.MAST_X
    s = at(Box(P.MAST_D, P.MAST_D, P.MAST_Z - P.ISMC200_H),
           x, 0, (P.MAST_Z + P.ISMC200_H) / 2)
    s += at(Box(100, 180, 24), x, 0, P.ISMC200_H + 12)   # bolted base plate
    # base gussets bracing the column against the side member it is bolted to
    for gy in (-1, 1):
        s += at(Box(70, 12, 110), x - 10, gy * (P.MAST_D / 2 + 6),
                P.ISMC200_H + 70)
    # cantilever arm reaching -X over the deck edge
    arm_x = x - P.MAST_ARM / 2
    s += at(Box(P.MAST_ARM, 100, 60), arm_x, 0, P.MAST_Z - 40)
    s += at(Box(80, 100, 90), x - P.MAST_ARM, 0, P.MAST_Z - 105)   # sensor bracket
    return s


def motor_mount():
    """Rear floor saddle carrying the overhanging end of the motor foot.

    The front of the foot already lands on the rail-support cross-beam (its top
    is flush with the foot at z = ISMC200_H), and the rear frame member carries
    the middle.  Only the part of the foot past the frame needs taking to the
    floor, so the pedestal is a single two-leg saddle that straddles the member
    instead of one solid block passing through it.
    """
    W = P.MOTOR_SUP_W
    h = P.ISMC200_H
    y = P.MOTOR_SUP_REAR_Y
    s = at(Box(2 * P.MOTOR_SUP_X + W, W, 18), 0, y, h - 9)        # top crossbar
    for sx in (-1, 1):
        s += at(Box(W, W, h - 18), sx * P.MOTOR_SUP_X, y, (h - 18) / 2)
        s += at(Box(W + 40, W + 40, 12), sx * P.MOTOR_SUP_X, y, h - 6)
    return s


def wiper_seal():
    """Red polyurethane wiper/end seal for an HGW35CC block."""
    return at(Box(P.BLOCK_W + 4, P.BLOCK_H + 4, P.WIPER_T), 0, 0, 0)


def laser_sensor():
    """Downward-facing industrial laser distance sensor + bracket."""
    s = at(Box(160, 160, 20), 0, 0, 0)
    s += at(cyl(P.LASER_L, P.LASER_D, "Z"), 0, 0, -(P.LASER_L / 2 + 10))
    return s


# ============================================================= cabinet / drag
def cabinet():
    """IP55 floor-standing enclosure 800(H) x 600(W) x 250(D) on a plinth."""
    W, D, H = P.CAB_W, P.CAB_D, P.CAB_H
    s = at(Box(D, W, 60), 0, 0, 30)
    s += at(Box(D, W, H), 0, 0, 60 + H / 2)
    # raised door edge + hinges
    s += at(Box(20, W - 30, H - 30), D / 2, 0, 60 + H / 2)
    for dz in (-1, 1):
        s += at(cyl(24, 90, "Z"), D / 2 + 4, dz * (W / 2 - 40), 60 + H / 2 + dz * H * 0.28)
    # roof fan cowl
    s += at(Box(D - 40, W - 60, 30), 0, 0, 60 + H + 15)
    return s


def estop():
    """Emergency-stop mushroom button, axis +X (door face)."""
    s = at(cyl(40, 16, "X"), 0, 0, 0)
    s += at(cyl(P.E_STOP_D, 12, "X"), 6, 0, 0)
    return s


def hmi():
    s = at(Box(20, P.HMI_W, P.HMI_H), 0, 0, 0)
    s += at(Box(6, P.HMI_W - 30, P.HMI_H - 30), 10, 0, 0)
    return s


def _chain_links(z_m):
    """Sample the C-loop energy chain into link poses (y, z, theta) for the
    moving-run top at height z_m.  The bend rises as the deck rises so the total
    loop length is constant -- the chain never hangs free."""
    R = P.CHAIN_R
    y_move = P.TROUGH_Y + P.CHAIN_RUN_DY
    y_fix = P.TROUGH_Y - P.CHAIN_RUN_DY
    z_f = P.CHAIN_FIXED_Z
    pi = math.pi
    L = ((P.CHAIN_Z_M_HOME - P.CHAIN_ZB_HOME) + (z_f - P.CHAIN_ZB_HOME)
         + pi * R)
    z_b = (z_m + z_f - (L - pi * R)) / 2.0
    m = 80
    poly = [(y_move, z_m + (z_b - z_m) * i / m) for i in range(m + 1)]
    for i in range(1, m + 1):                      # 180 deg bend at the bottom
        phi = -pi * i / m
        poly.append((P.TROUGH_Y + R * math.cos(phi), z_b + R * math.sin(phi)))
    poly += [(y_fix, z_b + (z_f - z_b) * i / m) for i in range(1, m + 1)]
    dist = [0.0]
    for i in range(1, len(poly)):
        dist.append(dist[-1] + math.hypot(poly[i][0] - poly[i - 1][0],
                                          poly[i][1] - poly[i - 1][1]))
    total = dist[-1]
    n = max(2, int(total / P.CHAIN_LINK))
    step = total / n
    out, j = [], 0
    for k in range(n):
        target = (k + 0.5) * step
        while j < len(poly) - 2 and dist[j + 1] < target:
            j += 1
        span = dist[j + 1] - dist[j]
        f = (target - dist[j]) / span if span > 1e-9 else 0.0
        y = poly[j][0] + (poly[j + 1][0] - poly[j][0]) * f
        z = poly[j][1] + (poly[j + 1][1] - poly[j][1]) * f
        theta = math.atan2(-(poly[j + 1][0] - poly[j][0]),
                           poly[j + 1][1] - poly[j][1])
        out.append((y, z, theta))
    return out


def drag_chain(z_m=None):
    """Self-supporting cable drag chain: one C-loop, fixed at the trough and
    moving with the deck, resting inside the guided trough (nothing suspended)."""
    z_m = P.CHAIN_Z_M_HOME if z_m is None else z_m
    s = None
    for (y, z, th) in _chain_links(z_m):
        link = Rot(math.degrees(th), 0, 0) * Box(P.CHAIN_W, P.CHAIN_T,
                                                 P.CHAIN_LINK * 0.86)
        link = Pos(P.TROUGH_X, y, z) * link
        s = link if s is None else s + link
    return s


def chain_trough():
    """Guided vertical drag-chain trough with its base foot and the fixed-end
    anchor bracket (static, world coordinates)."""
    X, Y = P.TROUGH_X, P.TROUGH_Y
    depth, w, t = P.TROUGH_DEPTH, P.TROUGH_W, P.TROUGH_T
    H = P.TROUGH_Z1 - P.TROUGH_Z0
    zc = (P.TROUGH_Z0 + P.TROUGH_Z1) / 2
    s = at(Box(t, w + 2 * t, H), X + depth / 2 + t / 2, Y, zc)
    for sy in (-1, 1):
        s += at(Box(depth + t, t, H), X - t / 2, Y + sy * (w / 2 + t / 2), zc)
    s += at(Box(170, 160, 16), 630, Y, P.ISMC200_H + 8)          # base foot
    s += at(Box(16, 160, 60), X + depth / 2 + t, Y, P.ISMC200_H - 20)
    y_fix = Y - P.CHAIN_RUN_DY
    s += at(Box(120, 90, 16), X - 10, y_fix, P.CHAIN_FIXED_Z)    # fixed anchor
    s += at(Box(16, 90, 120), X + depth / 2 + t + 6, y_fix,
            P.CHAIN_FIXED_Z - 60)
    for dz in (-1, 1):
        s -= at(cyl(P.M12_CLEAR, 20, "X"), X - 10, y_fix + dz * 30,
                P.CHAIN_FIXED_Z)
    return s


def chain_bed_bracket():
    """Moving chain-end bracket: carries the chain's moving run from the deck
    side member (world coordinates at home, lift group)."""
    x0 = P.DECK_FRAME + P.ISMC150_B / 2
    y = P.TROUGH_Y + P.CHAIN_RUN_DY
    z = P.Z_BED_HOME
    s = at(Box(P.TROUGH_X - x0, 90, 16), (x0 + P.TROUGH_X) / 2, y, z)
    s += at(Box(P.TROUGH_X - x0, 16, 90), (x0 + P.TROUGH_X) / 2, y, z - 45)
    for dz in (-1, 1):
        s -= at(cyl(P.M12_CLEAR, 20, "X"), x0 - 4, y + dz * 30, z)
    return s
