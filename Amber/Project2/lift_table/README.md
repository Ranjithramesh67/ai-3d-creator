# Miniature Synchronized Dual-Screw Lift Table

A parametric, 3D-printable vertical lift table driven by a single NEMA17 through a
closed GT2 belt loop that turns two T8 lead screws in perfect sync. The carriage
rides four 8 mm guide rods on LM8UU bearings and travels 98 mm.

Everything is generated from code with [build123d](https://build123d.readthedocs.io/),
so any dimension can be changed in `cad/params.py` and the whole model (geometry,
validation and web viewer manifest) is rebuilt.

## Specifications

| Item | Value |
| --- | --- |
| Frame | 2020 aluminium extrusion, 260 x 200 x 20 mm base |
| Overall envelope | 260 x 200 x 240 mm (plus motor) |
| Usable stroke | 98 mm (bed Z 82 -> 180 mm) |
| Vertical guide rods | 4 x 8 mm hardened rod, 217 mm long |
| Lead screws | 2 x T8x8 (8 mm lead, 4-start) |
| Bearings (thrust) | 5 x 608ZZ (22/8/7) |
| Bearings (linear) | 4 x LM8UU (15/8/24) |
| Transmission | 20T : 20T GT2, closed loop, pitch length 413.1 mm |
| Resolution | 2.5 um/step @ 200 steps, 16 microsteps |
| Motor | NEMA17, 42.3 mm body, 31 mm bolt pattern |

## Kinematics

One 20T pulley on the motor drives a single closed belt that wraps the two 20T
screw pulleys, so both lead screws rotate together. Both screws turn the same
direction; because the anti-backlash nuts are mounted on opposite sides of the
carriage centreline, the two nuts move in the same vertical direction and the bed
rises as a rigid, level platform. Gear ratio is 1:1, giving 8 mm of travel per
motor revolution.

The belt path is a triangle formed by the two screw pulleys and one smooth idler
on the rear side. The idler sits in a slotted bracket so belt tension is set by
sliding the bracket before locking it down.

## Design rationale

- **Extruded frame, printed adapters.** Loads and bending moments are carried by
  2020 aluminium extrusion; only the interfaces are printed. The four blue corner
  brackets have a vertical 8.2 mm x 15 mm blind hole that captures the base of each
  guide rod, and their lips drop into the rail T-slots so they self-align.
- **Thrust is taken by real bearings.** The bottom 608ZZ in each blue screw anchor
  block carries the screw's axial load against the block's floor ledge; the top
  608ZZ in the top frame only centres the screw. Printed plastic never acts as a
  bearing race.
- **1:1 belt, no differential.** Identical 20T pulleys on motor and screws keep the
  left and right screws phase-locked without any couplings or separate drivers.
- **LM8UU housings with a specified clearance.** The critical LM8UU bore is
  15.15 mm (see `LM8_BORE`) rather than 15.0 mm, so the printed clamp can actually
  slip over the bearing after printing shrinkage. Two horizontal zip-tie grooves
  clamp each bearing, and the bottom lip carries it so it cannot fall out.
- **Anti-backlash nuts slotted in X.** Each nut flange hole is slotted 1 mm in X
  (`NUT_SLOT_LEN = 5.2` for a 3.2 mm screw) so the nut can be preloaded against the
  screw and still find the carriage's hole pattern.
- **Wire routing outside the envelope.** Sensor and motor wiring runs down a
  dedicated mast at x = 124 mm, clear of the full carriage travel.
- **Manifold validation.** `build.py` runs dynamic clearance checks by boolean
  intersection at the bottom and top of travel, plus reach checks between moving
  and static parts.

## Validation summary

All checks pass (see `models/build_report.json` for detail):

- Dynamic clearance at bed Z = 82 and Z = 180: 0.000 mm^3 interference.
- Nut body clears the screw pulley (nut bottom 69.0 > pulley top 61.5).
- LM8 housing clears the corner bracket (66.0 > 38.0).
- Bed clears the downward sensor (189.0 < 194.0) and the top frame (189.0 < 230.0).
- Belt pitch length 413.1 mm (order a closed loop of that length, or a 410 mm loop
  plus the idler slot take-up).

Printed parts are designed for PLA/PETG with no support: overhangs stay at or below
45 degrees and blind pockets open downward or upward.

## Bill of materials

Printed (PLA or PETG):

- 1x motor mount, 1x idler tensioner bracket, 2x screw anchor block
- 4x corner bracket, 1x carriage bed with LM8UU bosses and nut pads
- 1x top frame, 1x sensor bracket, 1x wire mast

Purchased:

- 2020 extrusion: 2x 260 mm, 2x 160 mm
- 4x 8 mm x 217 mm guide rod, 2x T8x8 215 mm lead screw
- 5x 608ZZ, 4x LM8UU, 2x T8 anti-backlash nut
- 3x 20T GT2 pulley (5 mm bore for motor, 8 mm bore for screws)
- 1x 413 mm closed GT2 belt, 1x 8 mm x 26 mm idler axle
- 1x NEMA17 stepper, 1x TCRT5000 reflective sensor
- M3 bolts: 34 total (see instance list in `models/build_report.json`)

## Files

- `cad/params.py` - every dimension, documented (class `P`).
- `cad/parts.py` - part generator functions.
- `cad/build.py` - assembly, validation, STEP/STL export, `scene.json` manifest.
- `cad/render_preview.py` - offline PNG previews to `docs/renders/`.
- `index.html` - interactive three.js viewer (orbit, per-part visibility, lift
  slider, play, explode, wireframe, frame-hide).
- `vendor/` - vendored three.js r128 so the viewer works offline.
- `models/step/`, `models/stl/` - per-part geometry exports.
- `models/lift_table_assembly.step` - full assembly.
- `models/scene.json`, `models/build_report.json` - viewer manifest and report.

## Build and view

Rebuild geometry and re-run validation:

```
python3 cad/build.py
```

Render preview images:

```
python3 cad/render_preview.py
```

Serve the interactive viewer:

```
python3 -m http.server 8000 --directory .
```

Then open `http://localhost:8000/index.html`.
