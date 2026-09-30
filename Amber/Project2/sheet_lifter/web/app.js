/* Cinematic driver: 15 s / 60 fps timeline, camera choreography, mechanism
   kinematics, FEA overlay and load vectors. */
(function () {
  "use strict";
  const THREE = window.THREE;
  const P = window.P || {};

  const MM_PER_REV = 0.6 / 1.6;   // = Tr60x9 lead / jack ratio = 0.375 mm/rev
  const MICRO = 0.6;

  // ---- cycle timing: every sheet pick-up lasts `pickInterval` seconds ----
  const INTRO = 1.5, TAIL = 1.5;       // lead-in and tail of the sequence
  const INDEX_DUR = 2.0;               // screws-only index after every batch
  const HOLD_DUR = 1.2;                // dwell before the next batch
  const CAM_PERIOD = 15.0;             // camera choreography loop (s)
  const MIN_PICK = 5.0;                // sheet pick-up can never be faster
  let pickInterval = MIN_PICK;         // seconds per sheet pick-up (user input)
  let dur = CAM_PERIOD;                // full sequence length (recomputed on load)
  let pendingTime = null;              // ?t= value, applied once dur is known

  const keys = [
    { t: 0.0, pos: [2600, -3000, 2100], tgt: [0, 0, 1000] },
    { t: 2.4, pos: [1150, -1750, 1560], tgt: [-60, 0, 940] },
    { t: 5.4, pos: [2000, -1500, 1180], tgt: [-80, 0, 820] },
    { t: 8.6, pos: [-1900, -1750, 1300], tgt: [-40, 0, 840] },
    { t: 11.6, pos: [2400, -2400, 1600], tgt: [0, 0, 900] },
    { t: 15.0, pos: [3100, -3100, 2050], tgt: [0, 0, 1080] }
  ];

  const root = document.getElementById("app");
  const loadEl = document.getElementById("load");
  const $ = (id) => document.getElementById(id);

  let renderer, scene, camera, controls, mats, half = new THREE.Vector2();
  let time = 0, playing = true, clock;
  let cine = true;
  let snapCam = false;
  const camTarget = new THREE.Vector3(40, 0, 1000);

  // simulation speed (button cycles through this list)
  const SPEEDS = [0.1, 0.25, 0.5, 1, 2];
  let speedIdx = 2, speed = SPEEDS[speedIdx];   // start at 0.5x so picking reads

  // sheet-size presets (width, depth) for the "Auto size" animation
  const SIZE_PRESETS = [[850, 750], [650, 600], [450, 450], [600, 550]];
  let sizeAuto = false, sizeTime = 0, curSize = null;

  // ------------- component catalogue (show/hide + click to inspect) ---------
  const PART_INFO = {
    base_frame: { label: "Welded base frame", desc: "ISMC 200 channel perimeter ring with lapped, gusseted corners and M16 floor anchors. Carries every other sub-assembly and reacts the total tipping moment." },
    rail_support: { label: "Rail support cross-beams", desc: "Two 200 mm deep RHS beams span between the side members and carry the machined rail mounting lands, so the guide rails never stand on free pads." },
    rail_post: { label: "Vertical rail columns", desc: "Four plumb 90x90 RHS columns back the HGR35 rails over their full height; the rail flanges bolt to their faces. Nothing is tilted." },
    rail_pad: { label: "Precision rail pads", desc: "140 mm milled M16-bolted pads that datum the rail lands onto the cross-beams." },
    hgr35_rail: { label: "HIWIN HGR35 rail", desc: "Profile linear guide rail, cut to 694 mm so its top finishes level with the 300 mm sheet stack at the home position." },
    hgw35_block: { label: "HGW35CC carriage block", desc: "Heavy-load flanged recirculating-ball block; four of them carry the bed and react eccentric load, fully engaged over the whole index stroke." },
    wiper_seal: { label: "Wiper / end seal", desc: "Red polyurethane wipers keep swarf out of the carriage ball circuits." },
    bed_ribs: { label: "Open rib deck", desc: "ISMC 150 crossed-rib grid closed by a perimeter frame that ties the four carriage blocks together. No top plate, so the suction head can pick sheets off the stack." },
    tslot_track: { label: "T-slot guide track", desc: "Continuous 30x24 T-slot aluminium track on each deck side. Guides clamp anywhere along it with cam levers - no discrete size plates." },
    track_riser: { label: "Track riser block", desc: "Square risers that lift the cross tracks clear of the perpendicular tracks they cross at the deck corners." },
    side_guide: { label: "Adjustable side guide", desc: "Centring blade for the +/-X sheet edge, carried by two cam-lever sliders on the front/rear T-slot tracks; sets sheet width 400-800 mm." },
    back_guide: { label: "Adjustable rear stop", desc: "Depth stop riding two cam-lever sliders on the left/right T-slot tracks; sets sheet depth 400-750 mm, lockable anywhere." },
    jack_stool: { label: "Jack hold-down stool", desc: "Machined stool on the mid cross-member that receives the stationary worm-gear jack housing and its bolted L-clamps." },
    jack_clamp: { label: "Jack hold-down L-clamp", desc: "Cast L-clamp: vertical web bears on the jack housing, bolted foot lands on the stool, holding each jack firmly down." },
    sheet_stack: { label: "Sheet stack (payload)", desc: "Up to 3 t (~462 sheets of 0.65 mm galvanised steel) at 800x750 mm - the rated 300 mm column the machine lifts and indexes." },
    trapez_screw: { label: "Tr60x9 trapezoidal screw", desc: "Real helical trapezoidal thread, threaded over 880 mm (z=100-980). It rotates in the jack's thrust bearings; i=24 with the worm gear gives 0.375 mm of lift per motor revolution." },
    worm_gear_jack: { label: "5 t worm-gear screw jack", desc: "Cast-iron jack with upper and lower thrust bearings that axially retain the rotating Tr60x9 screw. The screw only turns - the travelling bronze nut does the lifting. Two give 10 t (SF 3.3)." },
    bronze_nut: { label: "Bronze leadscrew nut", desc: "170x140x100 bronze nut threaded onto the rotating Tr60x9 screw; it travels the full 302 mm index stroke and is bolted to the deck bracket, so it carries the bed." },
    nut_bracket: { label: "Leadscrew nut bracket", desc: "Gusseted steel bracket that bolts the bronze nut up to the deck side members, tying the deck to the screw at two points - the deck can no longer sit free of the screws." },
    electric_motor: { label: "3.7 kW IE3 motor", desc: "Compact 132-frame TEFC induction motor with radial cooling fins and a vented fan cowl. Its shaft enters the jaw coupling and drives the gearbox input - nothing passes through the motor." },
    motor_mount: { label: "Motor pedestal", desc: "Two-leg saddle that takes the overhanging rear of the motor foot to the floor, straddling the base-frame member instead of passing through it." },
    disc_brake: { label: "Electromagnetic disc brake", desc: "Fail-safe brake between motor and coupling; power loss clamps the drivetrain so the raised bed cannot fall." },
    jaw_coupling: { label: "Zero-backlash jaw coupling", desc: "Elastomer spider coupling that transmits motor torque to the gearbox input boss while absorbing misalignment." },
    bevel_gearbox: { label: "1:1 spiral bevel gearbox", desc: "T-type gearbox, input along Y and outputs along +/-X, splitting one motor into two in-phase jack shafts - no belt, chain or slave axis." },
    gearbox_mount: { label: "Gearbox pedestal", desc: "Welded pedestal with a machined top plate that the gearbox cast feet bolt onto, landing on a floor base plate instead of floating between the shafts." },
    drive_shaft: { label: "Keyed drive shaft", desc: "O30 horizontal shaft with an axial key that transmits torque from the bevel gearbox to each worm-gear jack input." },
    sensor_mast: { label: "Side sensor mast", desc: "Bolted, gusseted mast on the +X base member with a cantilever arm, so the laser sits to the side and the machine top stays open." },
    laser_sensor: { label: "Laser distance sensor", desc: "Downward-facing industrial sensor that confirms sheet presence and stack height; it sits clear of the sheet edge." },
    cabinet: { label: "IP55 control cabinet", desc: "Floor-standing enclosure on a plinth, housing the VFD, PLC and safety contactors." },
    estop: { label: "Emergency stop", desc: "Red mushroom E-stop on the cabinet door; drops the drive and applies the brake." },
    hmi: { label: "HMI panel PC", desc: "7-inch touch panel for sheet size, stack recipe and diagnostics." },
    drag_chain: { label: "Guided drag chain", desc: "Segmented energy chain that loops through a vertical trough beside the +X jack; the fixed end anchors to the base at z=450 and the moving end rides the deck, so the chain never hangs on a free span." },
    chain_trough: { label: "Drag-chain trough", desc: "Vertical U-channel on the base that guides and supports the drag chain over its whole travel; bolted foot on the base-frame member with a fixed-end anchor bracket." },
    chain_bed_bracket: { label: "Moving chain bracket", desc: "Bracket on the deck that carries the chain's moving run inside the trough, keeping the loop under positive guidance as the bed indexes." },
    suction_pick_frame: { label: "Suction pick head", desc: "Robotic I-frame (two cross beams + a central spine) carrying four vacuum cups. It descends, picks 5 sheets, then transfers them; the open +X side clears the sensor." },
    robot_column: { label: "Pick-head column", desc: "Vertical Z column of the external robot, bolted onto the pick-head hub so the column, carriage and frame move as one attached head." },
    robot_carriage: { label: "Pick-head carriage", desc: "Robot carriage block that carries the column and pick head and travels with them as a single assembly." }
  };

  const PART_GROUPS = [
    ["Structure", ["base_frame", "rail_support", "rail_post", "rail_pad", "sensor_mast", "motor_mount", "gearbox_mount"]],
    ["Linear guides", ["hgr35_rail", "hgw35_block", "wiper_seal"]],
    ["Bed & sheets", ["bed_ribs", "tslot_track", "track_riser", "side_guide", "back_guide", "sheet_stack"]],
    ["Drivetrain", ["electric_motor", "disc_brake", "jaw_coupling", "bevel_gearbox", "drive_shaft", "trapez_screw", "worm_gear_jack", "jack_stool", "jack_clamp", "bronze_nut", "nut_bracket"]],
    ["Pick head", ["suction_pick_frame", "robot_column", "robot_carriage"]],
    ["Electrical", ["cabinet", "estop", "hmi", "drag_chain", "chain_trough", "chain_bed_bracket", "laser_sensor"]]
  ];

  // ---------------- FEA heat-map material ----------------
  const feaMat = new THREE.ShaderMaterial({
    uniforms: { uTime: { value: 0 } },
    vertexShader: [
      "varying vec3 vN; varying vec3 vW;",
      "void main(){ vN = normalize(mat3(modelMatrix)*normal);",
      "vec4 w = modelMatrix*vec4(position,1.0); vW = w.xyz;",
      "gl_Position = projectionMatrix*viewMatrix*w; }"
    ].join("\n"),
    fragmentShader: [
      "varying vec3 vN; varying vec3 vW; uniform float uTime;",
      "vec3 ramp(float t){",
      " t=clamp(t,0.0,1.0);",
      " vec3 a=vec3(0.10,0.31,0.84), b=vec3(0.12,0.64,0.78);",
      " vec3 c=vec3(0.16,0.78,0.44), d=vec3(0.96,0.77,0.27);",
      " vec3 e=vec3(0.91,0.27,0.17);",
      " if(t<0.4) return mix(a,b,t/0.4);",
      " if(t<0.7) return mix(b,c,(t-0.4)/0.3);",
      " if(t<0.9) return mix(c,d,(t-0.7)/0.2);",
      " return mix(d,e,(t-0.9)/0.1); }",
      "void main(){",
      " float r = length(vW.xy - vec3(0.0).xy)/700.0;",
      " float scan = 0.5+0.5*sin(vW.x*0.01+vW.y*0.012-uTime*2.0);",
      " float t = clamp(r*1.1 + (scan-0.5)*0.10, 0.0, 1.0);",
      " vec3 col = ramp(t);",
      " vec3 L=normalize(vec3(0.4,0.5,0.8));",
      " float df=0.55+0.45*max(dot(normalize(vN),L),0.0);",
      " gl_FragColor=vec4(col*df,1.0); }"
    ].join("\n")
  });

  function init() {
    const q = new URLSearchParams(location.search);
    if (q.has("t")) { pendingTime = Math.max(0, parseFloat(q.get("t"))); snapCam = true; }
    if (q.has("pause")) { playing = false; cine = true; }
    if (q.has("orbit")) cine = false;
    if (q.has("eye")) {
      const e = q.get("eye").split(",").map(Number);
      const l = q.get("look").split(",").map(Number);
      cine = false;
      window.__dbg = { eye: e, look: l };
    }
    renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.setSize(innerWidth, innerHeight);
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.05;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    root.appendChild(renderer.domElement);

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0b0d10);
    scene.fog = new THREE.Fog(0x0b0d10, 4200, 9000);

    camera = new THREE.PerspectiveCamera(38, innerWidth / innerHeight, 5, 20000);
    camera.up.set(0, 0, 1);
    camera.position.set(...keys[0].pos);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.06;
    controls.target.set(...keys[0].tgt);
    controls.maxPolarAngle = Math.PI * 0.495;
    controls.minDistance = 200;
    controls.maxDistance = 6000;
    controls.enabled = false;
    if (window.__dbg) {
      camera.position.set(window.__dbg.eye[0], window.__dbg.eye[1], window.__dbg.eye[2]);
      camTarget.set(window.__dbg.look[0], window.__dbg.look[1], window.__dbg.look[2]);
      controls.target.copy(camTarget);
      camera.lookAt(camTarget);
    }

    addLights();
    MAT.buildEnvironment(renderer, scene);

    fetch("models/scene.json").then((r) => r.json()).then((sc) => {
      mats = MAT.buildMaterials(sc.colors, MAT.makeMaps());
      MACHINE.loadAll(sc, mats, function () {
        MACHINE.build(sc, mats);
        scene.add(sc.root);
        window.__sc = sc;
        buildFEA(sc);
        buildPartsPanel(sc);
        applyOverlayParams();
        refreshDuration();
        loadEl.style.display = "none";
      });
    });

    clock = new THREE.Clock();
    addEventListener("resize", onResize);
    bindUI();
    animate();
  }

  function addLights() {
    scene.add(new THREE.HemisphereLight(0xbfd0e2, 0x2a2d31, 0.25));
    scene.add(new THREE.AmbientLight(0xffffff, 0.08));

    const specs = [
      { p: [-1400, -1500, 2600], i: 3.2, c: 0xfff4e0, s: 2400 },
      { p: [1600, -900, 2500], i: 2.3, c: 0xeaf0ff, s: 2200 },
      { p: [300, 1700, 2400], i: 1.6, c: 0xffffff, s: 2000 },
      { p: [2200, 1200, 1900], i: 1.1, c: 0xfff0d8, s: 1600 }
    ];
    specs.forEach(function (s) {
      const l = new THREE.DirectionalLight(s.c, s.i);
      l.position.set(...s.p);
      l.castShadow = true;
      l.shadow.mapSize.set(2048, 2048);
      l.shadow.camera.near = 200;
      l.shadow.camera.far = 8000;
      l.shadow.camera.left = -s.s;
      l.shadow.camera.right = s.s;
      l.shadow.camera.top = s.s;
      l.shadow.camera.bottom = -s.s;
      l.shadow.bias = -0.0004;
      l.shadow.normalBias = 2.0;
      scene.add(l);
      scene.add(l.target);
    });
  }

  // ------------------------------ FEA override ------------------------------
  let feaOn = false, feaTargets = [];
  function buildFEA(sc) {
    ["bed_plate", "bed_ribs"].forEach(function (n) {
      (sc.nodes[n] || []).forEach(function (m) {
        m.userData.solid = m.material;
        feaTargets.push(m);
      });
    });
    // displace the screws' look to show "axial only" when FEA is on
    (sc.nodes.trapez_screw || []).forEach(function (m) { m.userData.solid = m.material; });
  }
  function setFEA(on) {
    feaOn = on;
    $("legend").style.display = on ? "block" : "none";
    feaTargets.forEach(function (m) { m.material = on ? feaMat : m.userData.solid; });
    (window.__sc.nodes.trapez_screw || []).forEach(function (m) {
      if (on) { m.material = m.userData.solid.clone(); m.material.color.setHex(0x2f3338);
        m.material.emissive = new THREE.Color(0x101418); }
      else { m.material = m.userData.solid; }
    });
  }

  // ------------------------------- timeline --------------------------------
  function ss(x) { x = Math.min(1, Math.max(0, x)); return x * x * (3 - 2 * x); }
  function seg(t, a, b) { return ss((t - a) / (b - a)); }

  // ---- sequence timing derived from the per-sheet pick interval ----
  function cycleInfo() {
    const meta = (window.__sc && window.__sc.meta) || {};
    const sheets = Math.max(1, Math.round((meta.stack_height_mm || 300) /
      (meta.sheet_thickness_mm || 0.65)));
    const batch = meta.pick_sheets_per_step || 5;
    const indexMM = meta.pick_index_mm || batch * 0.65;
    const nb = Math.ceil(sheets / batch);
    const dur = INTRO + sheets * pickInterval +
                nb * (INDEX_DUR + HOLD_DUR) + TAIL;
    return { sheets, batch, indexMM, nb, dur };
  }

  function refreshDuration() {
    const c = cycleInfo();
    dur = c.dur;
    if (pendingTime !== null) { time = Math.min(pendingTime, dur); pendingTime = null; snapCam = true; }
    if (time > dur) time = 0;
    const s = $("scrub");
    if (s) { s.max = dur; s.step = Math.max(0.1, dur / 2000); }
    if ($("dur_txt")) $("dur_txt").textContent = fmtTime(dur);
    if ($("pi_txt")) $("pi_txt").textContent = pickInterval.toFixed(1) + " s";
    if ($("pint") && document.activeElement !== $("pint")) {
      $("pint").value = (Math.round(pickInterval * 10) / 10);
    }
  }

  function fmtTime(x) {
    x = Math.max(0, x);
    const m = Math.floor(x / 60), s = x - m * 60;
    return m > 0 ? (m + " min " + s.toFixed(1) + " s") : (s.toFixed(1) + " s");
  }

  function motion(t) {
    const c = cycleInfo();
    const base = { sheets: c.sheets, batch: c.batch, indexMM: c.indexMM,
                   idxTotal: c.nb, consumed: 0, batches: 0, bedIndex: 0,
                   stackScale: 1, done: false, sub: "ready", php: 0,
                   bedFrac: 0, sheetInBatch: 0 };
    if (t < INTRO) return base;

    const batchPeriod = c.batch * pickInterval + INDEX_DUR + HOLD_DUR;
    const tb = t - INTRO;
    const bi = Math.floor(tb / batchPeriod);

    // tail: whole sequence finished
    if (bi >= c.nb) {
      return Object.assign({}, base, { consumed: c.sheets, batches: c.nb,
        bedIndex: c.nb * c.indexMM, stackScale: 0, done: true, sub: "done" });
    }

    const within = tb - bi * batchPeriod;
    const pickRegion = c.batch * pickInterval;
    let sub, sheetInBatch = 0, php = 0, bedFrac = 0;
    if (within < pickRegion) {
      sub = "pick";
      sheetInBatch = Math.min(c.batch - 1, Math.floor(within / pickInterval));
      php = (within - sheetInBatch * pickInterval) / pickInterval;
    } else if (within < pickRegion + INDEX_DUR) {
      sub = "index"; sheetInBatch = c.batch; php = 1;
      bedFrac = (within - pickRegion) / INDEX_DUR;
    } else {
      sub = "hold"; sheetInBatch = c.batch; php = 1; bedFrac = 1;
    }

    const consumedIn = (sub === "pick")
      ? (sheetInBatch + (php > 0.5 ? 1 : 0))
      : c.batch;
    const consumed = Math.min(c.sheets, bi * c.batch + consumedIn);
    const bedIndex = (bi + bedFrac) * c.indexMM;
    const stackScale = Math.max(0, 1 - consumed / c.sheets);
    return { sheets: c.sheets, batch: c.batch, indexMM: c.indexMM,
             idxTotal: c.nb, consumed, batches: bi, bedIndex, stackScale,
             done: false, sub, php, bedFrac, sheetInBatch };
  }

  let feaScan = 0;
  function apply(t, dt) {
    const sc = window.__sc;
    if (!sc) return;
    const m = motion(t);

    // bed indexes up in discrete steps; the stack shrinks so its top stays level
    sc.liftG.position.z = m.bedIndex;
    if (sc.__updateChain) sc.__updateChain(m.bedIndex);
    if (sc.__stack) {
      sc.__stack.scale.z = Math.max(1e-3, m.stackScale);
      sc.__stack.visible = m.stackScale > 0.02;
    }
    const stackTop = sc.__stackBase + m.bedIndex + m.stackScale * sc.__stackH;
    sc.__stackTop = stackTop;

    // screws / shafts turn with the rise (Tr60x9, i=24 -> 0.375 mm/rev)
    const revs = m.bedIndex / MM_PER_REV;
    (sc.nodes.trapez_screw || []).forEach(function (o) {
      o.rotation.z = revs * Math.PI * 2;
    });
    (sc.nodes.drive_shaft || []).forEach(function (o) {
      o.rotation.x = revs * Math.PI * 2;
    });
    (sc.nodes.jaw_coupling || []).forEach(function (o) {
      o.rotation.y = revs * Math.PI * 2;
    });

    // ---- 4-leg suction pick cycle: pick 5 sheets, then the screws index ----
    const ph = m.done ? 1 : (m.sub === "pick" ? m.php : 1);
    const approach = seg(ph, 0.00, 0.28);
    const lift = seg(ph, 0.40, 0.64);
    const trav = seg(ph, 0.64, 0.84);
    const ret = seg(ph, 0.84, 1.00);
    const hover = 460;
    const padDrop = sc.__padDrop || 232;
    const pf = sc.__pickFrame;
    if (pf) {
      pf.visible = !m.done;
      const travel = m.done ? 0 : (-trav * 1600 + ret * 1600);
      pf.position.set(travel, 0,
        stackTop + padDrop + hover - approach * hover + lift * 240);
      const held = sc.__heldSheet;
      if (held) held.visible = !m.done && m.sub === "pick" && ph > 0.40 && ph < 0.92;
    }

    // ---- laser beam from the side mast ----
    const beam = sc.__beam, spot = sc.__spot;
    const on = !m.done && t > 0.9;
    beam.visible = on; spot.visible = on;
    if (on) {
      const len = Math.max(1, sc.__laserTop - stackTop);
      beam.position.set(sc.__laserX, 0, stackTop + len / 2);
      beam.scale.y = len;
      spot.position.set(sc.__laserX, 0, stackTop + 2);
    }

    // ---- reaction vectors ----
    if (sc.__vecG.visible) {
      const pulse = 0.6 + 0.4 * Math.abs(Math.sin(t * 2.2));
      sc.__rails.forEach(function (r) {
        r.up.userData.setLen(340 * pulse);
        r.down.userData.setLen(300 * pulse);
      });
    }

    feaScan += dt;
    feaMat.uniforms.uTime.value = feaScan;
  }

  function updateCamera(t, dt) {
    const ct = t % CAM_PERIOD;         // camera choreography runs its own loop
    let i = 0;
    while (i < keys.length - 2 && ct > keys[i + 1].t) i++;
    const a = keys[i], b = keys[i + 1];
    const f = ss((ct - a.t) / Math.max(1e-3, b.t - a.t));
    const p = new THREE.Vector3().fromArray(a.pos).lerp(new THREE.Vector3().fromArray(b.pos), f);
    const g = new THREE.Vector3().fromArray(a.tgt).lerp(new THREE.Vector3().fromArray(b.tgt), f);
    p.x += Math.sin(ct * 0.6) * 8; p.y += Math.cos(ct * 0.5) * 6;
    if (!cine) return;
    if (snapCam) {
      camera.position.copy(p); camTarget.copy(g); camera.lookAt(camTarget);
      snapCam = false; return;
    }
    camera.position.lerp(p, Math.min(1, dt * 6));
    camTarget.lerp(g, Math.min(1, dt * 6));
    camera.lookAt(camTarget);
  }

  function phaseName(t) {
    const sc = window.__sc;
    if (!sc) return "Initialising";
    const m = motion(t);
    if (m.sub === "ready") return "Stack Ready";
    if (m.done) return "Stack Consumed";
    if (m.sub === "pick") return "Vacuum Pick - sheet " + (m.consumed + 1);
    if (m.sub === "index") return "Screw Index - bed rising";
    return "Hold - next batch";
  }

  function updateHud(t) {
    const m = motion(t);
    $("tval").textContent = fmtTime(t);
    $("tcode").textContent = fmtTime(t) + " / " + fmtTime(dur);
    $("dz").textContent = m.bedIndex.toFixed(3) + " mm";
    $("rpm").textContent = (m.sub === "index" && !m.done) ? "1500 rpm" : "0 rpm";
    if ($("batch")) $("batch").textContent = Math.min(m.batches, m.idxTotal) + " / " + m.idxTotal;
    if ($("sheets")) {
      $("sheets").textContent = Math.ceil(m.sheets - m.consumed) + " / " + m.sheets;
    }
    if ($("pi_txt")) $("pi_txt").textContent = pickInterval.toFixed(1) + " s";
    $("phase").textContent = phaseName(t);
    const active = !m.done && m.sub !== "ready";
    $("dot").className = "dot " + (active ? "on" : "off");
    $("sens_txt").textContent = m.done ? "Laser: stack empty"
      : (!active ? "Laser: idle" : "Laser: sheet present");
    const s = $("scrub");
    if (document.activeElement !== s) s.value = t;
  }

  function animate() {
    requestAnimationFrame(animate);
    const dt = Math.min(0.05, clock.getDelta());
    if (playing) { time += dt * speed; if (time >= dur) time = 0; }
    if (sizeAuto) updateAutoSize(dt);
    updateCamera(time, dt);
    apply(time, dt);
    updateHud(time);
    if (!cine) controls.update();
    renderer.render(scene, camera);
  }

  function setSize(w, d) {
    const sc = window.__sc;
    if (!sc || !sc.__setSize) return;
    sc.__setSize(w, d);
    curSize = { w: w, d: d };
    if ($("sw")) $("sw").value = w;
    if ($("sd")) $("sd").value = d;
    if ($("sw_txt")) $("sw_txt").textContent = Math.round(w) + " mm";
    if ($("sd_txt")) $("sd_txt").textContent = Math.round(d) + " mm";
  }

  function updateAutoSize(dt) {
    sizeTime += dt;
    const n = SIZE_PRESETS.length, period = 3.0;
    const u = (sizeTime / period) % n;
    const k = Math.floor(u), f = ss(u - k);
    const A = SIZE_PRESETS[k], B = SIZE_PRESETS[(k + 1) % n];
    setSize(A[0] + (B[0] - A[0]) * f, A[1] + (B[1] - A[1]) * f);
  }

  function manualSize(w, d) {
    sizeAuto = false;
    if ($("bauto")) $("bauto").classList.remove("active");
    setSize(w, d);
  }

  // ---------------------- component show/hide panel -------------------------
  function partTargets(name) {
    const sc = window.__sc;
    if (!sc) return [];
    if (name === "suction_pick_frame") return sc.__frameG ? [sc.__frameG] : [];
    if (name === "robot_column") return sc.__robotCol ? [sc.__robotCol] : [];
    if (name === "robot_carriage") return sc.__robotCar ? [sc.__robotCar] : [];
    return sc.nodes[name] || [];
  }

  function partRow(name, targets) {
    const info = PART_INFO[name];
    const label = document.createElement("label");
    const cb = document.createElement("input");
    cb.type = "checkbox"; cb.checked = true;
    const sw = document.createElement("span");
    sw.className = "sw";
    const sc = window.__sc;
    sw.style.background = (sc && sc.colors && sc.colors[name]) || "#888";
    const txt = document.createElement("span");
    txt.textContent = info ? info.label : name;
    cb.onchange = function () {
      targets.forEach(function (t) { t.visible = cb.checked; });
    };
    label.appendChild(cb); label.appendChild(sw); label.appendChild(txt);
    label.dataset.part = name;
    return label;
  }

  function buildPartsPanel(sc) {
    const host = $("parts_list");
    if (!host) return;
    host.innerHTML = "";
    const seen = {};
    PART_GROUPS.forEach(function (g) {
      const wrap = document.createElement("div");
      const h = document.createElement("div");
      h.className = "grp"; h.textContent = g[0];
      wrap.appendChild(h);
      let n = 0;
      g[1].forEach(function (name) {
        const targets = partTargets(name);
        if (!targets.length) return;
        seen[name] = true; n++;
        wrap.appendChild(partRow(name, targets));
      });
      if (n) host.appendChild(wrap);
    });
    const extra = [];
    Object.keys(sc.nodes || {}).forEach(function (nm) {
      if (!seen[nm] && nm !== "bed_plate") extra.push(nm);
    });
    if (extra.length) {
      const wrap = document.createElement("div");
      const h = document.createElement("div");
      h.className = "grp"; h.textContent = "Other";
      wrap.appendChild(h);
      extra.forEach(function (nm) { wrap.appendChild(partRow(nm, sc.nodes[nm])); });
      host.appendChild(wrap);
    }
  }

  function setAllParts(on) {
    const list = document.querySelectorAll("#parts_list input[type=checkbox]");
    Array.prototype.forEach.call(list, function (cb) {
      if (cb.checked !== on) { cb.checked = on; cb.onchange(); }
    });
  }

  // ------------------------- click / long-press inspect ---------------------
  function isShown(o) {
    let n = o;
    while (n) { if (n.visible === false) return false; n = n.parent; }
    return true;
  }

  function showInfo(part) {
    const info = PART_INFO[part] || { label: part, desc: "Machine component." };
    $("info_title").textContent = info.label;
    $("info_sub").textContent = part;
    $("info_desc").textContent = info.desc;
    $("info").style.display = "block";
  }
  function hideInfo() { const el = $("info"); if (el) el.style.display = "none"; }

  function pickInfo(cx, cy) {
    const sc = window.__sc;
    if (!sc || !sc.root) return;
    const r = renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((cx - r.left) / r.width) * 2 - 1,
      -((cy - r.top) / r.height) * 2 + 1);
    const ray = new THREE.Raycaster();
    ray.setFromCamera(ndc, camera);
    const hits = ray.intersectObject(sc.root, true);
    for (let i = 0; i < hits.length; i++) {
      const o = hits[i].object;
      if (!o.userData || !o.userData.part) continue;
      if (!isShown(o)) continue;
      showInfo(o.userData.part);
      return;
    }
    hideInfo();
  }

  function onResize() {
    camera.aspect = innerWidth / innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(innerWidth, innerHeight);
  }

  function takeOver() {
    if (!cine) return;
    cine = false;
    controls.enabled = true;
    controls.target.copy(camTarget);
  }

  function applyOverlayParams() {
    const q = new URLSearchParams(location.search);
    if (q.get("parts") === "1" && $("bparts")) $("bparts").click();
    if (q.has("inspect")) showInfo(q.get("inspect"));
    if (q.has("hide")) {
      q.get("hide").split(",").forEach(function (nm) {
        const cb = document.querySelector(
          '#parts_list label[data-part="' + nm + '"] input');
        if (cb && cb.checked) { cb.checked = false; cb.onchange(); }
      });
    }
    if (q.get("fea") === "1") $("bfea").click();
    if (q.get("vec") === "1") $("bvec").click();
    if (q.get("wire") === "1") $("bwire").click();
    if (q.get("frame") === "1") $("bframe").click();
    if (q.has("speed")) {
      const s = parseFloat(q.get("speed"));
      const i = SPEEDS.indexOf(s);
      if (i >= 0) { speedIdx = i; speed = SPEEDS[i]; $("bspeed").textContent = speed + "x"; }
    }
    if (q.has("interval")) {
      const v = parseFloat(q.get("interval"));
      if (isFinite(v) && v >= MIN_PICK) pickInterval = v;
    }
    const sc = window.__sc;
    const sz = q.get("size");
    if (sz) {
      const a = sz.split(",").map(Number);
      if (a.length === 2) setSize(a[0], a[1]);
    } else if (sc && sc.__baseSize) {
      setSize(sc.__baseSize.w, sc.__baseSize.d);
    }
    if (q.get("auto") === "1" && $("bauto")) $("bauto").click();
  }

  function bindUI() {
    // tap / click / long-press a component to read its name and role; drag to orbit
    let dn = null, dnT = 0, lp = null;
    function endPress() { if (lp) { clearTimeout(lp); lp = null; } }
    renderer.domElement.addEventListener("pointerdown", function (e) {
      if (e.button !== undefined && e.button !== 0) return;
      dn = { x: e.clientX, y: e.clientY }; dnT = performance.now();
      endPress();
      lp = setTimeout(function () {
        lp = null; dn = null; pickInfo(e.clientX, e.clientY);
      }, 480);
    });
    renderer.domElement.addEventListener("pointermove", function (e) {
      if (!dn) return;
      if (Math.hypot(e.clientX - dn.x, e.clientY - dn.y) > 8) { endPress(); dn = null; }
    });
    renderer.domElement.addEventListener("pointerup", function (e) {
      const wasTap = dn && performance.now() - dnT < 450;
      endPress();
      if (wasTap) pickInfo(e.clientX, e.clientY);
      dn = null;
    });
    renderer.domElement.addEventListener("pointercancel", function () {
      endPress(); dn = null;
    });

    renderer.domElement.addEventListener("pointerdown", takeOver);
    $("play").onclick = function () {
      playing = !playing;
      this.textContent = playing ? "Pause" : "Play";
    };
    $("restart").onclick = function () {
      time = 0; playing = true; cine = true; controls.enabled = false;
      $("play").textContent = "Pause";
    };
    $("scrub").oninput = function () { time = parseFloat(this.value); snapCam = true; };
    $("bspeed").onclick = function () {
      speedIdx = (speedIdx + 1) % SPEEDS.length;
      speed = SPEEDS[speedIdx];
      this.textContent = speed + "x";
    };
    if ($("bauto")) {
      $("bauto").onclick = function () {
        this.classList.toggle("active");
        sizeAuto = this.classList.contains("active");
        if (sizeAuto) sizeTime = 0;
      };
    }
    if ($("sw")) {
      $("sw").oninput = function () {
        manualSize(parseFloat(this.value), curSize ? curSize.d : 750);
      };
    }
    if ($("sd")) {
      $("sd").oninput = function () {
        manualSize(curSize ? curSize.w : 850, parseFloat(this.value));
      };
    }
    if ($("pint")) {
      $("pint").oninput = function () {
        const v = parseFloat(this.value);
        if (isFinite(v) && v >= MIN_PICK) { pickInterval = v; refreshDuration(); }
      };
      $("pint").onchange = function () {
        const v = Math.max(MIN_PICK, parseFloat(this.value) || MIN_PICK);
        this.value = v; pickInterval = v; refreshDuration();
      };
    }
    $("bfea").onclick = function () {
      this.classList.toggle("active");
      setFEA(this.classList.contains("active"));
    };
    $("bvec").onclick = function () {
      this.classList.toggle("active");
      if (window.__sc) window.__sc.__vecG.visible = this.classList.contains("active");
    };
    $("bwire").onclick = function () {
      this.classList.toggle("active");
      const on = this.classList.contains("active");
      if (!window.__sc) return;
      window.__sc.root.traverse(function (o) {
        if (o.isMesh && o.material && "wireframe" in o.material) o.material.wireframe = on;
      });
    };
    $("bframe").onclick = function () {
      this.classList.toggle("active");
      const hide = this.classList.contains("active");
      if (!window.__sc) return;
      ["base_frame", "sensor_mast"].forEach(function (n) {
        (window.__sc.nodes[n] || []).forEach(function (m) { m.visible = !hide; });
      });
    };
    if ($("bparts")) {
      $("bparts").onclick = function () {
        this.classList.toggle("active");
        $("parts").style.display = this.classList.contains("active")
          ? "flex" : "none";
      };
    }
    if ($("parts_all")) $("parts_all").onclick = function () { setAllParts(true); };
    if ($("parts_none")) $("parts_none").onclick = function () { setAllParts(false); };
    if ($("info_x")) $("info_x").onclick = hideInfo;
    addEventListener("keydown", function (e) {
      if (e.code === "Space") { e.preventDefault(); $("play").click(); }
    });
  }

  init();
})();
