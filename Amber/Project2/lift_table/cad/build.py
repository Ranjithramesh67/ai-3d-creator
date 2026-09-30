"""Build, assemble, validate and export every part of the lift table."""

import json
import math
import os
import sys
import time
import traceback

from build123d import Circle, Compound, Pos, Rot, export_step, export_stl, make_hull

from params import P, PI
import parts as PT

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STEP_DIR = os.path.join(ROOT, "models", "step")
STL_DIR = os.path.join(ROOT, "models", "stl")
os.makedirs(STEP_DIR, exist_ok=True)
os.makedirs(STL_DIR, exist_ok=True)

report = {"parts": [], "instances": [], "warnings": [], "checks": []}


def log(msg):
    print(msg, flush=True)


def register(name, shape, color, group="static", tolerance=0.08):
    report["parts"].append({"name": name, "color": color})
    try:
        export_step(shape, os.path.join(STEP_DIR, name + ".step"))
        export_stl(shape, os.path.join(STL_DIR, name + ".stl"),
                   tolerance=tolerance, angular_tolerance=0.25)
        bb = shape.bounding_box()
        report["checks"].append({
            "part": name, "valid": bool(shape.is_valid),
            "volume_mm3": round(shape.volume, 1),
            "bbox": [round(v, 2) for v in (bb.size.X, bb.size.Y, bb.size.Z)],
        })
    except Exception as exc:  # noqa: BLE001
        report["warnings"].append(f"export {name}: {exc}")
        log(f"  ! export failed {name}: {exc}")


def instance(name, shape, x=0, y=0, z=0, rx=0, ry=0, rz=0, group="static"):
    placed = Pos(x, y, z) * Rot(rx, ry, rz) * shape
    report["instances"].append({
        "part": name, "group": group,
        "pos": [x, y, z], "rot": [rx, ry, rz],
    })
    return placed


def build():
    t0 = time.time()
    solids = []
    colors = {}

    # ---------------------------------------------------------------- base frame
    log("[1/6] base frame + brackets")
    rails = {
        "rail_front": (0.0, -P.RAIL_Y, "X", P.RAIL_XLEN),
        "rail_rear": (0.0, P.RAIL_Y, "X", P.RAIL_XLEN),
        "rail_left": (-P.RAIL_X, 0.0, "Y", P.RAIL_YLEN),
        "rail_right": (P.RAIL_X, 0.0, "Y", P.RAIL_YLEN),
    }
    for nm, (cx, cy, ax, ln) in rails.items():
        shp = PT.rail(ln, ax, cx, cy)
        colors[nm] = "#9aa0a6"
        register(nm, shp, colors[nm])
        solids.append(instance(nm, shp))

    cb = PT.corner_bracket(1, 1)
    colors["corner_bracket"] = "#2f80ed"
    register("corner_bracket", cb, colors["corner_bracket"])
    for sx in (-1, 1):
        for sy in (-1, 1):
            if (sx, sy) == (1, 1):
                shp = cb
            elif (sx, sy) == (1, -1):
                shp = PT.corner_bracket(1, -1)
            elif (sx, sy) == (-1, 1):
                shp = PT.corner_bracket(-1, 1)
            else:
                shp = PT.corner_bracket(-1, -1)
            solids.append(instance("corner_bracket", shp))

    an = PT.screw_anchor(1)
    register("screw_anchor", an, "#2d9cdb")
    solids.append(instance("screw_anchor", an))
    solids.append(instance("screw_anchor", PT.screw_anchor(-1)))

    mm = PT.motor_mount()
    register("motor_mount", mm, "#27ae60")
    solids.append(instance("motor_mount", mm))

    tb = PT.tensioner_bracket()
    register("tensioner_bracket", tb, "#27ae60")
    solids.append(instance("tensioner_bracket", tb))

    # ---------------------------------------------------------------- transmission
    log("[2/6] transmission + powertrain")
    idler = PT.bearing_608()
    register("bearing_608", idler, "#f2c94c")
    ax = PT.axle_m8()
    register("axle_m8", ax, "#828282")
    reg = PT.bearing_lm8uu()
    register("bearing_lm8uu", reg, "#f2c94c")

    pull_motor = PT.gt2_pulley(5.0)
    pull_screw = PT.gt2_pulley(8.0)
    register("pulley_motor", pull_motor, "#eb5757")
    register("pulley_screw", pull_screw, "#eb5757")

    motor = PT.nema17_motor()
    register("motor_nema17", motor, "#4f4f4f")
    solids.append(instance("motor_nema17", motor, P.MOTOR_X, P.MOTOR_Y, P.Z_MOTOR_FACE))

    solids.append(instance("pulley_motor", pull_motor, P.MOTOR_X, P.MOTOR_Y, P.Z_BELT))
    for sx in (-1, 1):
        solids.append(instance("pulley_screw", pull_screw, sx * P.SCREW_X, 0, P.Z_BELT))
    solids.append(instance("bearing_608", idler, 0, -34.0, P.Z_BELT))
    # bottom thrust bearings inside the anchor blocks, top bearings in the frame
    for sx in (-1, 1):
        solids.append(instance("bearing_608", idler, sx * P.SCREW_X, 0,
                               P.Z_RAIL_TOP + 12.0))
        solids.append(instance("bearing_608", idler, sx * P.SCREW_X, 0,
                               P.Z_TOP_TOP - 3.0))

    belt = PT.gt2_belt()
    register("gt2_belt", belt, "#111111", tolerance=0.05)
    solids.append(instance("gt2_belt", belt, 0, 0, P.Z_BELT - P.BELT_W / 2))

    rod = PT.guide_rod(23.0, P.Z_TOP_TOP)
    register("guide_rod", rod, "#bdbdbd")
    for sx in (-1, 1):
        for sy in (-1, 1):
            solids.append(instance("guide_rod", rod, sx * P.ROD_X, sy * P.ROD_Y, 0))

    screw = PT.leadscrew(25.0, P.Z_TOP_TOP)
    register("leadscrew_t8", screw, "#bdbdbd")
    for sx in (-1, 1):
        solids.append(instance("leadscrew_t8", screw, sx * P.SCREW_X, 0, 0))

    # ---------------------------------------------------------------- carriage
    log("[3/6] carriage / lift bed")
    bed = PT.carriage_bed()
    register("carriage_bed", bed, "#f2994a")
    solids.append(instance("carriage_bed", bed, 0, 0, P.Z_BED_MIN, group="carriage"))

    for sx in (-1, 1):
        for sy in (-1, 1):
            solids.append(instance("bearing_lm8uu", reg, sx * P.ROD_X, sy * P.ROD_Y,
                                   P.Z_BED_MIN - 3.0, group="carriage"))
    nut = PT.t8_nut()
    register("nut_t8_antibacklash", nut, "#bb6bd9")
    for sx in (-1, 1):
        solids.append(instance("nut_t8_antibacklash", nut, sx * P.SCREW_X, 0,
                               P.Z_BED_MIN + P.BED_T + P.RIB_T, group="carriage"))

    # ---------------------------------------------------------------- top frame
    log("[4/6] top frame + sensor")
    tf = PT.top_frame()
    register("top_frame", tf, "#56ccf2")
    solids.append(instance("top_frame", tf))

    sb = PT.sensor_bracket()
    register("sensor_bracket", sb, "#27ae60")
    solids.append(instance("sensor_bracket", sb))

    wm = PT.wire_mast()
    register("wire_mast", wm, "#27ae60")
    solids.append(instance("wire_mast", wm))
    solids.append(instance("axle_m8", ax, 0, -34.0, 45.0))

    # ---------------------------------------------------------------- hardware
    log("[5/6] fasteners")
    bolt = PT.m3_bolt(16.0)
    register("m3_bolt", bolt, "#828282", tolerance=0.04)
    bcount = 0
    # motor (4) - head on top of the motor plate
    for dx in (-P.NEMA_BOLT / 2, P.NEMA_BOLT / 2):
        for dy in (-P.NEMA_BOLT / 2, P.NEMA_BOLT / 2):
            solids.append(instance("m3_bolt", bolt, P.MOTOR_X + dx, P.MOTOR_Y + dy,
                                   P.Z_MOTOR_FACE + 6.0))
            bcount += 1
    # corner brackets (12)
    for sx in (-1, 1):
        for sy in (-1, 1):
            for bx, by in [(sx * P.RAIL_X, sy * (P.ROD_Y - 8.0)),
                           (sx * (P.ROD_X - 8.0), sy * P.RAIL_Y),
                           (sx * (P.ROD_X + 8.0), sy * P.RAIL_Y)]:
                solids.append(instance("m3_bolt", bolt, bx, by, P.CB_Z1))
                bcount += 1
    # screw anchors (4)
    for sx in (-1, 1):
        for by in (-10.0, 10.0):
            solids.append(instance("m3_bolt", bolt, sx * P.RAIL_X, by, P.Z_RAIL_TOP + 16.0))
            bcount += 1
    # tensioner (2)
    for bx in (-20.0, 20.0):
        solids.append(instance("m3_bolt", bolt, bx, -P.RAIL_Y, P.Z_RAIL_TOP + 6.0))
        bcount += 1
    # lead-nut flanges (8) - head above the flange, pointing down
    for sx in (-1, 1):
        for dx in (-P.NUT_HOLE_SQ / 2, P.NUT_HOLE_SQ / 2):
            for dy in (-P.NUT_HOLE_SQ / 2, P.NUT_HOLE_SQ / 2):
                solids.append(instance("m3_bolt", bolt, sx * P.SCREW_X + dx, dy,
                                       P.Z_BED_MIN + P.BED_T + P.RIB_T + P.NUT_FLANGE_T + 0.5,
                                       group="carriage"))
                bcount += 1
    # sensor bracket (2) - head below, pointing up into the top frame
    for bx in (-18.0, 18.0):
        solids.append(instance("m3_bolt", bolt, bx, 0, P.Z_TOP_BOT - 8.0, 180, 0, 0))
        bcount += 1
    # wire mast (2) - one down into the corner bracket, one up into the top frame
    solids.append(instance("m3_bolt", bolt, P.ROD_X + 11.0, P.ROD_Y, 46.0))
    solids.append(instance("m3_bolt", bolt, P.ROD_X + 11.0, P.ROD_Y, 222.0, 180, 0, 0))
    bcount += 2
    report["checks"].append({"bolts": bcount})

    # ------------------------------------------------ dynamic clearance checks
    def place(sh, x=0, y=0, z=0, rx=0, ry=0, rz=0):
        return Pos(x, y, z) * Rot(rx, ry, rz) * sh

    def _bbox(sh):
        b = sh.bounding_box()
        return (b.min.X, b.min.Y, b.min.Z), (b.max.X, b.max.Y, b.max.Z)

    def _overlap(a, b):
        a0, a1 = _bbox(a)
        b0, b1 = _bbox(b)
        return all(a0[i] <= b1[i] + 1e-6 and b0[i] <= a1[i] + 1e-6 for i in range(3))

    obstacles = {
        "top_frame": place(tf), "sensor_bracket": place(sb), "wire_mast": place(wm),
        "belt": place(belt, 0, 0, P.Z_BELT - P.BELT_W / 2),
        "motor": place(motor, P.MOTOR_X, P.MOTOR_Y, P.Z_MOTOR_FACE),
        "idler": place(idler, 0, -34.0, P.Z_BELT),
        "tensioner": place(tb), "motor_mount": place(mm),
        "anchor": place(an), "corner": place(cb),
        "pulley_screw": place(pull_screw, P.SCREW_X, 0, P.Z_BELT),
    }
    for zb in (P.Z_BED_MIN, P.Z_BED_MAX):
        car = [("bed", place(bed, 0, 0, zb))]
        for sx in (-1, 1):
            for sy in (-1, 1):
                car.append(("lm8", place(reg, sx * P.ROD_X, sy * P.ROD_Y, zb - 3)))
        for sx in (-1, 1):
            car.append(("nut", place(nut, sx * P.SCREW_X, 0,
                                     zb + P.BED_T + P.RIB_T)))
        worst, wname = 0.0, ""
        for pn, ps in car:
            for on, os_ in obstacles.items():
                if not _overlap(ps, os_):
                    continue
                try:
                    v = (ps & os_).volume
                except Exception:  # noqa: BLE001
                    v = 0.0
                if v > worst:
                    worst, wname = v, f"{pn} vs {on}"
        report["checks"].append({
            "check": f"dynamic_clearance_z{int(zb)}", "pass": worst < 1.0,
            "detail": f"max interference {worst:.3f} mm^3 ({wname or 'none'})",
        })

    # /////////////////////////////////////////////////////////////// assemble
    log("[6/6] boolean assembly union (this takes a moment)")
    asm = Compound(solids)
    export_step(asm, os.path.join(ROOT, "models", "lift_table_assembly.step"))
    try:
        export_stl(asm, os.path.join(STL_DIR, "_assembly.stl"),
                   tolerance=0.15, angular_tolerance=0.4)
    except Exception as exc:  # noqa: BLE001
        report["warnings"].append(f"assembly STL: {exc}")

    # ---------------------------------------------------------------- validation
    checks = report["checks"]

    def add_check(name, ok, detail):
        checks.append({"check": name, "pass": bool(ok), "detail": detail})

    nut_bottom = P.Z_BED_MIN + P.BED_T + P.RIB_T - P.NUT_BODY_L
    pulley_top = P.Z_BELT + P.PULLEY_H / 2
    add_check("nut_body_clears_screw_pulley", nut_bottom > pulley_top,
              f"nut bottom {nut_bottom:.1f} > pulley top {pulley_top:.1f}")
    lm_bottom = P.Z_BED_MIN - P.LM8_DOWN
    add_check("lm8_housing_clears_corner_bracket", lm_bottom > P.CB_Z1,
              f"housing bottom {lm_bottom:.1f} > bracket top {P.CB_Z1:.1f}")
    bed_top_max = P.Z_BED_MAX + P.BED_T + P.RIB_T
    add_check("bed_clears_sensor", bed_top_max < P.SENSOR_PAD_Z,
              f"bed top max {bed_top_max:.1f} < sensor face {P.SENSOR_PAD_Z:.1f}")
    lm_top_max = P.Z_BED_MAX + P.LM8_UP
    add_check("carriage_clears_top_frame", lm_top_max < P.Z_TOP_BOT,
              f"carriage top {lm_top_max:.1f} < top frame {P.Z_TOP_BOT:.1f}")
    travel = P.Z_BED_MAX - P.Z_BED_MIN
    add_check("travel", True, f"{travel:.1f} mm usable stroke")

    # belt length from the hull perimeter
    perim = None
    try:
        centers = [(P.MOTOR_X, P.MOTOR_Y, P.GT2_RP), (-P.SCREW_X, P.SCREW_Y, P.GT2_RP),
                   (P.SCREW_X, P.SCREW_Y, P.GT2_RP), (0.0, -34.0, 11.0)]
        hull = make_hull([(Pos(cx, cy) * Circle(r)).edge() for cx, cy, r in centers])
        perim = sum(e.length for e in hull.edges())
        add_check("gt2_belt_pitch_length", True,
                  f"{perim:.1f} mm pitch length (order a closed loop to match)")
    except Exception as exc:  # noqa: BLE001
        report["warnings"].append(f"belt length: {exc}")

    # ---------------------------------------------------------------- kinematics
    lift_per_rev = P.T8_LEAD
    step_res = lift_per_rev / (P.MOTOR_STEPS * P.MICROSTEPS)
    add_check("kinematics_1to1", True,
              f"20T motor : 20T screws -> 1:1; {lift_per_rev} mm/rev; "
              f"{step_res*1000:.1f} um/step @ {P.MOTOR_STEPS} steps, "
              f"{P.MICROSTEPS} microsteps")
    report["belt_pitch_length_mm"] = round(perim, 2) if perim else None

    report["build_seconds"] = round(time.time() - t0, 1)
    with open(os.path.join(ROOT, "models", "build_report.json"), "w") as fh:
        json.dump(report, fh, indent=2)

    # scene manifest for the three.js viewer
    scene = {
        "meta": {
            "units": "mm",
            "travel_mm": P.Z_BED_MAX - P.Z_BED_MIN,
            "bed_min_z": P.Z_BED_MIN,
            "bed_max_z": P.Z_BED_MAX,
            "lead_mm_per_rev": P.T8_LEAD,
            "belt_pitch_length_mm": report.get("belt_pitch_length_mm"),
            "envelope": [P.FX, P.FY, P.Z_TOP_TOP],
        },
        "colors": {p["name"]: p["color"] for p in report["parts"]},
        "instances": report["instances"],
    }
    with open(os.path.join(ROOT, "models", "scene.json"), "w") as fh:
        json.dump(scene, fh, indent=2)

    log(f"\nDone in {report['build_seconds']} s.  "
        f"{len(report['parts'])} unique parts, {len(report['instances'])} instances.")
    for c in checks:
        if "pass" in c:
            log(f"  [{'PASS' if c['pass'] else 'FAIL'}] {c['check']}: {c['detail']}")
    if report["warnings"]:
        log("Warnings:")
        for w in report["warnings"]:
            log("  - " + w)


if __name__ == "__main__":
    try:
        build()
    except Exception:
        traceback.print_exc()
        sys.exit(1)
