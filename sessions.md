# Sessions / Handoff Notes

_Last updated: 2026-09-30_

Repo: https://github.com/Ranjithramesh67/ai-3d-creator

Notes for the next AI (or human) picking this up. Read this first.

## What this repo is

Two fully parametric, **code-generated** 3D machines with interactive three.js
web viewers. Nothing is modelled by hand: every dimension lives in a Python
`params.py`, and geometry, validation and the web-viewer manifest are all
regenerated from source.

```
Amber/Project2/
  sheet_lifter/   # 3-ton automatic sheet-metal stack lifter (active)
  lift_table/     # scissor/belt lift table (earlier, finished)
```

Both projects keep their own `README.md` with the full design write-up.

## sheet_lifter (the active project)

**What it is.** A dual-column screw-jack lifter that raises a 3 t stack of
0.65 mm galvanised steel sheets in 3.25 mm batches so a de-stacker can peel one
sheet at a time. Single 3.7 kW motor -> EM brake -> jaw coupling -> 1:1 bevel
gearbox -> two Tr60x9 screws -> identical left/right jacks.

**Key kinematic facts (batch-3, current):**

- The jack housings are fixed; the **screw rotates** in upper/lower thrust
  bearings and cannot move axially.
- Load is carried by a **travelling bronze leadscrew nut** (170x140x100 mm),
  bolted through a gusseted bracket to the deck side members.
- A **guided drag chain** loops through a vertical U-channel trough; fixed end
  anchored to the base at z=450 mm, moving end on the deck. No free span.
- 4 x HIWIN HGR35 rails on plumb 90x90 RHS posts; rails deliberately capped so
  their tops finish level with the 300 mm stack at home.
- Open rib deck (no top plate) so a robotic 4-cup suction "I"-frame can pick
  from above; laser sensor is on a **side mast**, not over the top.
- Leads: 9 mm screw lead / i=24 -> **0.375 mm per motor rev**; 0.6 mm micro-step
  = 1.6 revs.

**Status:** green and deployed.

- `models/build_report.json`: **24/24 checks PASS**
- `models/collision_report.json`: **0 unwanted collisions**
- 34 unique parts, 67 instances in `models/scene.json`

### Batch history (most recent last)

1. Initial parametric model
2. T-slot size index, stationary worm jacks, two-leg motor saddle
3. **Rotating screws + bronze leadscrew nuts + guided drag-chain trough**
   (commit `6ca7d34` in the original workspace history)

### Key files

| File | Purpose |
| --- | --- |
| `cad/params.py` | Every dimension (class `P`) + derived values. Edit here first. |
| `cad/profiles.py` | 2D profiles & primitive hardware (I-sections, bolts, nuts, `at()`). |
| `cad/parts.py` | One generator function per part. |
| `cad/screwmesh.py` | Compact parametric Tr60x9 helix mesh for the web asset. |
| `cad/build.py` | Assembles everything, runs the geometry checks, writes STEP/STL + `models/scene.json` + `models/build_report.json`. |
| `cad/collision_check.py` | AABB prefilter + real `.intersect()` volume scan; `ALLOWED` whitelist for intentional mating pairs. |
| `cad/render_preview.py` | Static PNG previews into `docs/renders/`. |
| `web/materials.js` | Procedural PBR materials + generated HDRI env. |
| `web/machine.js` | STL loading, assembly hierarchy, suction frame, procedural flexing drag chain. |
| `web/app.js` | Timeline, camera keyframes, kinematics, HUD, overlays, parts panel. |
| `index.html` | Viewer shell + UI. |
| `vendor/` | three.js r128 + STLLoader + OrbitControls (offline, no CDN). |

### Rebuild / verify

```bash
cd cad
python3 build.py           # ~40-60 s on 2 cores: geometry + checks + STEP/STL + scene.json
python3 collision_check.py # interference scan -> models/collision_report.json
python3 render_preview.py  # PNG previews -> docs/renders/
```

Serve the viewer locally (it must be served over HTTP, not opened as a file):

```bash
# from the sheet_lifter directory
python3 -m http.server 6800
```

## lift_table (finished)

A smaller belt-driven lift table. 23 unique parts, 73 instances, `models/`
manifest + a per-part validity report. Same tooling and structure as
`sheet_lifter`. Its `models/scene.json` meta carries `travel_mm`,
`bed_min_z`/`bed_max_z`, `lead_mm_per_rev` and `belt_pitch_length_mm`.

## Conventions & gotchas

- **Units are millimetres; the scene is Z-up** (X = width, Y = depth, Z = up,
  floor at z = 0). In three.js the camera uses `camera.up.set(0, 0, 1)` and
  Y-up primitives go through `cylY2Z()`.
- `scene.json` keys: `meta`, `colors`, `instances`. The viewer loads one STL per
  entry in `colors` from `models/stl/`.
- **Regenerated STEP files churn on every build** (export headers), so a diff
  after `build.py` touches many `models/step/*.step` files even for a small
  change. That is expected.
- When you add a part or a new moving interface, update **both**
  `cad/collision_check.py` (geometry + placements + `ALLOWED` whitelist) and the
  web layer: `materials.js` `SPEC`, `app.js` `PART_INFO`/`PART_GROUPS`, and
  `machine.js` if it animates.
- The web viewer is offline-first: three.js is vendored in `vendor/`. Do not
  switch to a CDN.
- Big regenerated artifacts (`*_full.zip`, and the giant per-part STEP files) are
  intentionally **not** committed to keep the repo lean; rebuild with
  `python3 build.py`.

## Open items / next steps

- Deployment is done by rsync to a remote host and served by pm2; **credentials
  and the host address are kept out of this repo and shared out of band.**
- The in-session `request_preview` tool fails in this environment
  ("relay tunnel is not connected"); use the deployed public URL or the local
  `http.server` instead.
- Possible future work: export a GLB/glTF alongside STL for the web viewer,
  add animated pick-cycle timing to `lift_table`, and add CI to run
  `build.py` + `collision_check.py` on push.

## Security

This repo is public. **Never commit deployment credentials, SSH passwords or
API tokens.** Machine parameters are safe to commit; secrets are not.
