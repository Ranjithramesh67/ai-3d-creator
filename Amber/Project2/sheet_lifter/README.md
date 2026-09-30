# 3-Ton Automatic Sheet-Metal Stack Lifting Machine

A parametric industrial CAD model of a dual-column screw-jack sheet lifter that
raises a 3 t stack of 0.6-0.7 mm galvanised sheets in 0.6 mm increments so a
de-stacker/press feed can peel one sheet per cycle. Two Tr60x9 trapezoidal screws,
driven by a single 3.7 kW IE3 motor through a 1:1 T-type spiral bevel gearbox,
lift a 1000 x 1000 mm platform on four HIWIN HGR35 profile rails.

Everything is generated from code with [build123d](https://build123d.readthedocs.io/),
so any dimension can be changed in `cad/params.py` and the whole model (geometry,
validation and web viewer manifest) is rebuilt.

## Specifications

| Item | Value |
| --- | --- |
| Base frame | Welded ISMC 200 channel (IS 808), lapped + gusseted corners, 1400 x 1200 mm |
| Overall envelope | 1400 x 1200 x 1600 mm (side sensor mast is the tallest point) |
| Platform | Open ISMC 150 crossed rib deck + perimeter frame (no top plate; a robotic suction frame picks sheets from above) |
| Sheet size range | Width 400-800 mm, depth 400-750 mm (continuous T-slot tracks + cam-lever clamps) |
| Payload | 3 t (approx. 460 sheets @ 0.65 mm, 300 mm stack) |
| Vertical guides | 4 x HIWIN HGR35 rail, 694 mm, at +/-460 / +/-460 mm, on supported cross-beams; rail tops finish level with the 300 mm sheet stack (never taller) |
| Rail support | 2 x 200x110 RHS cross-beams between the ISMC 200 side members + 4 x 90x90 vertical RHS posts, one plumb column behind every rail |
| Carriage blocks | 8 x HGW35CC flange blocks + polyurethane end wipers |
| Lifting screws | 2 x Tr60x9 trapezoidal with real helical thread, threaded over 880 mm (z = 100-980), at x = +/-500 mm |
| Screw jacks | 2 x worm-gear jack, i = 24, 5 t each, thrust-bearing-held **rotating** screws, L-clamped to the mid beam |
| Load transfer | Travelling bronze leadscrew nuts (170 x 140 x 100 mm) bolted to gusseted deck brackets, so each nut carries the deck |
| Powertrain | 3.7 kW IE3 motor + EM disc brake + zero-backlash jaw coupling |
| Transmission | 1:1 T-type spiral bevel gearbox, 2 x Ø30 keyed shafts, one to each jack |
| Resolution | 0.6 mm per step = 1.6 motor revolutions |
| Indexing | After every 5 sheets only the two screws turn to raise the bed 3.25 mm, then the axis holds until the next 5-sheet batch |
| Sensing | Downward laser distance sensor on a side mast (top of machine stays clear) |
| Control | Standalone cabinet, E-stop, 7-inch HMI panel PC |

## Kinematics

The motor output passes through an electromagnetic disc brake and a zero-backlash
elastomer jaw coupling into a 1:1 T-type spiral bevel gearbox. The gearbox splits
the rotation left and right into two horizontal Ø30 keyed shafts that feed the
input barrels of the two worm-gear jacks (both barrels face the centreline). This
is a true two-sided gearbox drive; there is no belt or chain anywhere in the
machine.

Each jack has a worm ratio of i = 24 driving a Tr60x9 screw (9 mm lead), so one
motor revolution raises the platform by 9 / 24 = 0.375 mm. A single command step
of 0.6 mm (one sheet thickness) therefore equals 1.6 motor revolutions, giving
smooth, repeatable sheet-indexing motion.

Both screw jacks are identical but the left unit is mirrored 180 degrees about Z.
Because the two screws share one gearbox, the platform stays level without any
electronic levelling loop. The four corner HGW35CC blocks carry the eccentric
tipping moment, so the screws see axial load only. Each jack holds its screw in
**upper and lower thrust bearings**, so the screw **only rotates** and cannot walk
axially; the jack housing is bolted firmly down onto a machined stool on the mid
cross-member by two cast L-clamps. A **bronze leadscrew nut** travels up and down
the turning screw and is bolted through a gusseted bracket to the deck side
members, so the nut - not a translating screw - carries the platform.

Cable and signal power reach the moving deck through a **guided vertical drag
chain**: one self-supporting C-loop (R45 bend) runs inside a U-channel trough
bolted to the base, with its fixed end anchored to the frame at z = 450 mm and its
moving end carried by a bracket on the deck. The chain is under positive guidance
over the whole stroke, so nothing hangs on a free span.

The bed is deliberately an **open deck**: the 1000 x 1000 top plate is removed and
the sheet stack rests directly on the ISMC 150 crossed rib grid, which is closed
by a perimeter frame that ties the four carriage blocks together. With the top
open, an external robotic suction frame with four vacuum pads reaches down and
lifts the top sheet off the stack each cycle.

### Sheet-size adjustability

Two side guides and one rear depth stop ride on **continuous aluminium T-slot
tracks**. A 30 x 24 mm 30-series track runs along each side of the open deck
(the cross pair is raised on square riser blocks so the two directions never
collide at the corners). Each guide is carried by two cam-lever sliders whose
T-nuts drop into the slot and clamp anywhere along the track, so the size is set
steplessly and locked with a flip of two levers - no loose bolts and no discrete
index plates. Engraved graduations every 50 mm give a datum for the common sizes
(400-800 mm width, 400-750 mm depth). In the web viewer the same motion is driven
live by the **width / depth aligner sliders**, and an **Auto size** mode cycles
the guides, the stack footprint and the suction frame through a sequence of blank
sizes.

### Automatic re-indexing

The laser sensor on the side mast watches the top of the stack. The cycle runs in
batches of five sheets: the suction frame removes five sheets, then **only the two
Tr60x9 screws rotate** to index the bed up by 5 x 0.65 mm = 3.25 mm (a whole number
of screw steps), and then the axis **holds** until the next batch of five is ready.
This keeps the top of the stack at a constant height for the suction frame while
the bed ratchets steadily upward until the whole 300 mm stack has been consumed.
A **speed control** in the viewer and the **Pick cycle** interval input slow the
time-lapse down so each sheet pick-up (minimum 5 s), the screw-only index and the
dwell are clearly visible.

## Design rationale

- **Real catalogue hardware, no stylised geometry.** Sections follow IS 808
  (ISMC 200 / ISMC 150), rails and blocks follow HIWIN HGR35 / HGW35CC values,
  fasteners follow IS 1367, and the drive string is real industrial equipment
  (IE3 motor, EM brake, jaw coupling, bevel gearbox, worm-gear jacks).
- **Screws lift, rails guide.** The trapezoidal screws are sized for the full
  axial load with a large safety factor, while the profile rails and carriage
  blocks react bending and the tipping moment from an off-centre stack. This keeps
  the screws in pure tension instead of buckling.
- **Rails on a braced substructure.** The four vertical HGR35 rails are no longer
  standing on free pads: each rail's 48 mm mounting flange is rotated outboard and
  bolted to the machined face of a plumb 90 x 90 RHS column over its full height,
  and each column's foot lands on a 200 x 110 RHS cross-beam welded between the
  ISMC 200 side members. Nothing is tilted, the top of the machine stays open for
  the pick frame, and the carriage block runs freely between the columns.
- **Guide height matched to the sheets.** The HGR35 rails are cut down to 694 mm
  so their tops finish level with the top of the 300 mm sheet stack at the
  bed-home position, instead of towering ~1.5 m over an open deck. The HGW35CC
  carriage block is offset below the bed top by `MAX_BED_INDEX + BLOCK_L/2 - STACK_H`
  so it stays fully engaged on the rail through the whole 302.25 mm index stroke
  and never runs off the rail top (`carriage_on_rail` check).
- **1:1 bevel split, two-sided drive.** A single T-type bevel gearbox drives both
  jacks from one motor through a Ø30 keyed shaft on each side, so the two screws
  cannot drift out of phase. There is no belt, chain, separate slave axis or
  levelling controller. Each shaft carries a key that runs **along its axis** and
  sits proud of the surface on top (+Z), engaging the jack input keyway instead of
  cutting across it.
- **Realistic, compact motor.** The 3.7 kW IE3 motor is a 200 mm-diameter TEFC
  frame with radial cooling fins that stand proud of the shell, cast end bells, a
  vented fan cowl and a terminal box (400 mm stator, not a stretched tube). Its
  output shaft projects inboard into the jaw coupling; nothing passes through the
  motor, and the overhanging foot is carried by a welded pedestal.
- **Gearbox on its own pedestal.** Because the base frame is an open perimeter
  ring, the central bevel gearbox used to hang between the two jack shafts. It now
  bolts onto a welded pedestal with a machined top plate that lands on the floor,
  so the gearbox rest on a solid base (`gearbox_mount`).
- **Real thread geometry.** Each screw carries a true helical Tr60x9 trapezoidal
  thread (pitch 9 mm) swept along a genuine helix. The CAD builds it as whole-turn
  segments so identical blocks phase-align when stacked; the web asset is a compact
  parametric helix mesh (0.8 MB) instead of OCC's over-tessellated sweep.
- **Fail-safe holding.** The electromagnetic disc brake sits between the motor and
  the coupling, so power loss clamps the drivetrain and the raised bed cannot fall.
- **Stiff, light, open bed.** The ISMC 150 crossed rib grid is closed by a perimeter
  deck frame that ties the four carriage blocks together. The 16 mm top plate is
  omitted so a robotic suction arm can pick each sheet off the top of the stack; the
  analytic strip-deflection check stays under 1 mm at full 3 t load.
- **Open, unobstructed top.** There is no gantry over the machine: the laser sensor
  sits on a bolted side mast, so the entire top opening is free for the robotic
  suction frame to descend onto the stack.
- **"I"-shaped suction head, clear of the sensor.** The robotic pick head is an
  I-frame, not a closed square: two cross beams along X (front and back) joined by a
  single central spine along Y. The whole `+X` side is open at `y = 0`, so the head
  passes the side-mounted sensor and its laser beam, and the four vacuum cups hang
  directly under the cross-beam ends (inside the sheet outline for every blank
  size). The robot column and its carriage are bolted down onto a hub on the spine,
  so the column, carriage, frame and cups read and move as one rigid attached head.
- **Stated geometry checks.** `build.py` runs twenty-four automated checks (symmetry,
  rail parallelism, lead-per-revolution, micro-step consistency, jack capacity, rail
  height cap, carriage on rail, moment sharing, supported rails, plumb rail posts,
  two-sided drivetrain, coupled motor shaft, real thread, bed deflection, sensor
  geometry, frame corner joints, sheet-size adjustability, continuous guide tracks,
  jack hold-down, motor support, pick indexing, nut carries bed, screw axially
  retained and guided drag chain) and writes them to
  `models/build_report.json`.

## Automated checks

`python3 build.py` prints and records:

```
jack_symmetry              jacks at x=+/-500 mm, y=0 (left mirrored)
rail_vertical_parallel     4 x HGR35 at x=+/-460, y=+/-460, axis = Z
lead_per_motor_rev         Tr60x9 / i=24 -> 0.375 mm per motor revolution
micro_step_consistency     0.6 mm step = 1.60 motor revolutions
jacking_capacity           2 x 5 t = 10 t vs 3 t payload (SF 3.33)
rail_height_capped         rail tops level with the 300 mm stack at home
carriage_on_rail           HGW35 block captive over the 302.25 mm index stroke
guide_rails_share_moment   4 corner blocks react tipping; screws axial only
rails_supported            rails backed by plumb RHS posts on RHS cross-beams
rail_posts_plumb           4 x 90x90 columns vertical (axis = Z), no tilt
drivetrain_two_sided       1:1 gearbox -> 2 x Ø30 shafts into both jacks
motor_shaft_coupled        motor shaft enters the jaw coupling; no pass-through
screw_thread_real          Tr60x9 trapezoidal helix, 10-turn segments
analytic_bed_deflection    ~0.435 mm strip estimate under 3 t (target <1 mm)
laser_sensor_geometry      on the +X side mast, clear of the sheet edge
frame_corner_joints        lapped ISMC corners + gussets + fillet beads
sheet_size_adjustable      X 400-800 mm, Y 400-750 mm on continuous T-slot tracks
guide_track_continuous     one 940 mm T-slot track per side, clamp anywhere
jack_held_down             jack housing on a stool + 2 L-clamps (no floating jack)
motor_foot_supported       foot on cross-beam + rear member + two-leg saddle
pick_indexing              screws only: bed up 3.25 mm every 5 sheets, then hold
nut_carries_bed            2 x bronze leadscrew nut bolt to the deck bracket
screw_axially_retained     screw held in jack thrust bearings, z=100-980
drag_chain_guided          C-loop (R45) inside a vertical trough, both ends held
```

## Files

```
params.py            all dimensions (class P); edit here to re-parameterise
profiles.py          2D profiles and primitive hardware (ISMC, bolts, nuts)
parts.py             every part generator (34 parts)
screwmesh.py         compact parametric Tr60x9 helix mesh for the web asset
build.py             assemble + checks + STEP/STL + scene.json export
render_preview.py    PNG previews to docs/renders/
models/step/       34 per-part STEP files + full assembly STEP
models/stl/          34 per-part STL files (used by the web viewer)
models/scene.json    instance manifest (colors, placement, metadata)
models/build_report.json   machine-readable check results
index.html           cinematic web viewer shell
web/materials.js     procedural PBR materials + generated HDRI environment
web/machine.js       STL loading, assembly hierarchy, suction pick frame, vectors
web/app.js           timeline, camera keyframes, kinematics, HUD, overlays
vendor/              three.js, STLLoader, OrbitControls (offline)
docs/renders/        iso / front / top / drivetrain / bed / frame / mast PNG previews
```

## Build

```bash
cd cad
python3 build.py
```

This regenerates every STEP/STL, the assembly, `models/scene.json` and
`models/build_report.json` (under two minutes on 2 cores).

To regenerate the static PNG previews:

```bash
cd cad
python3 render_preview.py
```

## Web viewer

The interactive cinematic viewer is a self-contained three.js scene (three.js is
vendored locally, so it runs fully offline).

```bash
# from the sheet_lifter directory
python3 -m http.server 6800
```

Then open `http://localhost:6800/index.html`.

The sequence follows the de-stacking cycle: the stack sits ready, the four-pad
suction frame descends and picks the top sheet (each pick-up lasts a configurable
**interval of at least 5 seconds**), and after each batch of five sheets only the
screws turn to raise the bed, then the axis holds. The timeline spans the whole
job, so the stack shrinks and the bed indexes upward (3.25 mm every five sheets)
until the stack is consumed. The total length is `intro + sheets x interval +
batches x (index + hold) + tail`, recomputed live whenever the interval changes. A
timeline scrubber, speed control (0.1x / 0.25x / 0.5x / 1x / 2x, starts at 0.5x),
FEA stress overlay, load-vector overlay, wireframe and frame-sync controls are in
the HUD, along with the index-batch and sheet counters. A sheet-size panel carries
width and depth aligner sliders and an Auto size mode that animates the machine
through a sequence of blank sizes. A **Pick cycle** numeric input sets the
seconds per sheet pick-up (minimum 5 s, default 5 s); raising it slows the
simulation and stretches the timeline to fit every sheet in the stack.

The **Parts** button opens a component panel: every part is listed in its group
(Structure, Linear guides, Bed & sheets, Drivetrain, Pick head, Electrical) with a
colour swatch and a checkbox, so any component can be shown or hidden on its own
(All / None shortcuts included). **Click or long-press any component** in the 3D
scene to read its name and a short description of what it does, in the info card
above the timeline.

Debug URL parameters (useful for stills and QA):

```
?t=<seconds>&pause=1        freeze the timeline at a time
?orbit                      start in free-orbit mode
?eye=x,y,z&look=x,y,z       explicit camera placement
?solo=part1,part2           show only the named parts
?parts=1                    open the component show/hide panel
?hide=part1,part2           hide named components on load
?inspect=<part>             show the info card for a part
?fea=1&vec=1&wire=1&frame=1 enable overlays directly
?speed=0.25                 simulation speed multiplier
?interval=8                 seconds per sheet pick-up (min 5)
?size=650,600               start at a sheet size (width,depth)
?auto=1                     start the Auto size animation
```

## Model hierarchy

The scene is authored Z-up (X = width, Y = depth, Z = up, floor at z = 0) and uses
millimetres throughout. `scene.json` lists 67 instances of 34 unique parts, split
into a `static` group (base frame, RHS rail-support beams and plumb rail posts,
rails, side sensor mast, cabinet, motor saddle, jack stools/clamps, drag-chain
trough, powertrain) and a `lift` group (rib deck, continuous T-slot guide tracks and
risers, adjustable guides, carriage blocks, deck-side guides, bronze leadscrew nuts,
nut brackets, the moving chain end, rotating threaded screws, sheet stack) that
moves with the platform. The manifest carries `stack_base_z` (deck top), the
sheet-size range, `size_step_mm`, `track_y_mm`, `pick_index_mm` and a `chain` block
(bend radius, run spacing, fixed height), so the viewer places the stack on the open
deck, drives the 5-sheet re-indexing animation, flexes the drag chain with the deck
and resizes the guides for any blank size.
