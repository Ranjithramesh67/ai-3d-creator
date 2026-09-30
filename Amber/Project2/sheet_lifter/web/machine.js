/* Loads exported STLs + scene.json, builds the machine hierarchy, adds the
   cinematic props (top sheet, suction arm, laser beam, load vectors, ground). */
(function (global) {
  "use strict";
  const THREE = global.THREE;

  function planarUV(geo) {
    geo.computeBoundingBox();
    const bb = geo.boundingBox;
    const ex = Math.max(1e-3, bb.max.x - bb.min.x);
    const ez = Math.max(1e-3, bb.max.z - bb.min.z);
    const pos = geo.attributes.position;
    const uv = new Float32Array(pos.count * 2);
    for (let i = 0; i < pos.count; i++) {
      uv[i * 2] = (pos.getX(i) - bb.min.x) / ex;
      uv[i * 2 + 1] = (pos.getZ(i) - bb.min.z) / ez;
    }
    geo.setAttribute("uv", new THREE.BufferAttribute(uv, 2));
  }

  function makeArrow(dir, len, color) {
    const mat = new THREE.MeshBasicMaterial({ color: color, toneMapped: false });
    const g = new THREE.Group();
    const headLen = len * 0.22, headW = len * 0.08, shaftLen = len - headLen;
    const shaft = new THREE.Mesh(
      new THREE.CylinderGeometry(len * 0.019, len * 0.019, shaftLen, 12), mat);
    shaft.position.y = shaftLen / 2;
    const head = new THREE.Mesh(new THREE.ConeGeometry(headW, headLen, 18), mat);
    head.position.y = shaftLen + headLen / 2;
    g.add(shaft, head);
    g.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0),
      dir.clone().normalize());
    g.userData.base = len;
    g.userData.setLen = function (L) { g.scale.y = L / len; };
    return g;
  }

  function loadAll(scene, mats, onDone) {
    const loader = new THREE.STLLoader();
    const names = Object.keys(scene.colors);
    const geoCache = {};
    let pending = names.length;
    if (!pending) return onDone();
    names.forEach(function (name) {
      loader.load("models/stl/" + name + ".stl", function (g) {
        g.computeVertexNormals();
        planarUV(g);
        g.computeBoundingSphere();
        geoCache[name] = g;
        if (--pending === 0) onDone();
      }, undefined, function () { if (--pending === 0) onDone(); });
    });
    geoCache.__names = names;
    scene.__cache = geoCache;
  }

  function build(scene, mats) {
    const root = new THREE.Group();
    const staticG = new THREE.Group();
    const liftG = new THREE.Group();
    root.add(staticG, liftG);

    const cache = scene.__cache;
    const nodes = {};
    const soloQ = new URLSearchParams(location.search).get("solo");
    const solo = soloQ ? soloQ.split(",") : null;
    const rotNodes = { drive_shaft: "x", jaw_coupling: "y", trapez_screw: "z",
                       electric_fan: "y" };

    scene.instances.forEach(function (it) {
      const g = cache[it.part];
      if (!g) return;
      if (solo && solo.indexOf(it.part) < 0) return;
      const m = new THREE.Mesh(g, mats[it.part] || mats.sheet_stack);
      m.position.set(it.pos[0], it.pos[1], it.pos[2]);
      m.rotation.set(
        THREE.MathUtils.degToRad(it.rot[0]),
        THREE.MathUtils.degToRad(it.rot[1]),
        THREE.MathUtils.degToRad(it.rot[2]));
      m.castShadow = true; m.receiveShadow = true;
      m.userData.part = it.part;
      (it.group === "lift" ? liftG : staticG).add(m);
      (nodes[it.part] = nodes[it.part] || []).push(m);
    });

    // ---------------- ground + grid ----------------
    const ground = new THREE.Mesh(
      new THREE.PlaneGeometry(12000, 12000),
      new THREE.MeshStandardMaterial({ color: 0x2b2f34, metalness: 0.1,
        roughness: 0.95 }));
    ground.receiveShadow = true;
    root.add(ground);
    const grid = new THREE.GridHelper(8000, 40, 0x3a424b, 0x30363d);
    grid.rotation.x = Math.PI / 2;
    grid.position.z = 0.4;
    root.add(grid);

    // ---------------- payload stack reference ----------------
    const meta = scene.meta;
    const stackBase = meta.stack_base_z !== undefined ? meta.stack_base_z
                                                      : meta.home_bed_z + 16;
    scene.__stackBase = stackBase;
    scene.__stackH = meta.stack_height_mm;
    scene.__sheetT = meta.sheet_thickness_mm;
    scene.__stack = (nodes.sheet_stack || [])[0] || null;

    const baseW = meta.stack_width_mm || 850;
    const baseD = meta.stack_depth_mm || 750;
    const FENCE_T = meta.fence_t || 12;
    const GUIDE_GAP = 2;

    // ---------------- 4-leg suction pick frame (external robot) ----------------
    const spineMat = new THREE.MeshStandardMaterial({ color: 0x323840,
      metalness: 0.85, roughness: 0.32 });
    const steelMat = new THREE.MeshStandardMaterial({ color: 0x9aa0a6,
      metalness: 0.9, roughness: 0.28 });
    const pickFrame = new THREE.Group();
    const frameG = new THREE.Group();          // rebuildable, sized to the sheet
    pickFrame.add(frameG);
    scene.__padDrop = 232;

    // CylinderGeometry is built along +Y; the scene is Z-up, so every round
    // member (stems, cups, column) must be rotated onto Z or it lies horizontal.
    function cylY2Z(rt, rb, h, seg) {
      const g = new THREE.CylinderGeometry(rt, rb, h, seg);
      g.rotateX(Math.PI / 2);
      return g;
    }

    function buildPick(w, d) {
      while (frameG.children.length) {
        const c = frameG.children[0];
        frameG.remove(c);
        if (c.geometry) c.geometry.dispose();
      }
      const barH = 70, barW = 70;
      // pad lines stay inside the sheet so every cup lands on the panel; the
      // "I" opens the whole +X side dead-centre, clear of the sensor head.
      const padX = Math.max(120, w / 2 - 90);
      const padY = Math.max(120, d / 2 - 80);
      const spanX = padX + barW / 2;    // half length of each cross beam
      function bar(bw, bd, bh, x, y, z) {
        const m = new THREE.Mesh(new THREE.BoxGeometry(bw, bd, bh), spineMat);
        m.position.set(x, y, z); m.castShadow = true;
        m.userData.part = "suction_pick_frame";
        frameG.add(m);
      }
      // two cross beams along X = the top and bottom strokes of the "I"
      bar(spanX * 2, barW, barH, 0, padY, 0);
      bar(spanX * 2, barW, barH, 0, -padY, 0);
      // central spine along Y = the stem of the "I"
      bar(barW, padY * 2, barH, 0, 0, 0);
      // centre hub the robot column bolts down onto
      bar(220, 220, 24, 0, 0, barH / 2 + 12);
      // four suction cups hung directly under the cross-beam ends
      [[1, 1], [1, -1], [-1, 1], [-1, -1]].forEach(function (c) {
        const x = c[0] * padX, y = c[1] * padY;
        const stem = new THREE.Mesh(cylY2Z(24, 24, 160, 16), steelMat);
        stem.position.set(x, y, -115); stem.castShadow = true;
        stem.userData.part = "suction_pick_frame";
        const pad = new THREE.Mesh(cylY2Z(58, 72, 34, 28), mats.suction_cup);
        pad.position.set(x, y, -212); pad.castShadow = true;
        pad.userData.part = "suction_pick_frame";
        frameG.add(stem, pad);
      });
      const held = new THREE.Mesh(
        new THREE.BoxGeometry(w - 10, d - 10, meta.sheet_thickness_mm),
        mats.sheet_top);
      held.position.set(0, 0, -229 - meta.sheet_thickness_mm / 2);
      held.castShadow = true; held.visible = false;
      held.userData.part = "sheet_top";
      frameG.add(held);
      scene.__heldSheet = held;
    }
    buildPick(baseW, baseD);
    // overhead column + carriage of the external robot, bolted onto the frame
    // hub so the whole pick head reads (and moves) as one attached assembly.
    const colH = 900, hubTop = 70 / 2 + 24;
    const col = new THREE.Mesh(cylY2Z(70, 70, colH, 24), steelMat);
    col.position.set(0, 0, hubTop + colH / 2); col.castShadow = true;
    col.userData.part = "robot_column";
    const car = new THREE.Mesh(new THREE.BoxGeometry(420, 300, 160), spineMat);
    car.position.set(0, 0, hubTop + colH + 80); car.castShadow = true;
    car.userData.part = "robot_carriage";
    pickFrame.add(col, car);
    scene.__pickFrame = pickFrame;
    scene.__frameG = frameG;
    scene.__robotCol = col;
    scene.__robotCar = car;
    scene.__baseSize = { w: baseW, d: baseD };

    // reposition the adjustable guides / stack / pick frame for any sheet size
    scene.__setSize = function (w, d, applyStack) {
      if (scene.__size && Math.abs(scene.__size.w - w) < 0.5 &&
          Math.abs(scene.__size.d - d) < 0.5) return;
      const fx = w / 2 + FENCE_T / 2 + GUIDE_GAP;
      (nodes.side_guide || []).forEach(function (m) {
        m.position.x = (m.position.x < 0 ? -1 : 1) * fx;
      });
      const by = d / 2 + FENCE_T / 2 + GUIDE_GAP;
      (nodes.back_guide || []).forEach(function (m) { m.position.y = by; });
      if (scene.__stack && applyStack !== false) {
        scene.__stack.scale.x = w / baseW;
        scene.__stack.scale.y = d / baseD;
      }
      buildPick(w, d);
      scene.__size = { w: w, d: d };
    };
    root.add(pickFrame);

    // ---------------- laser beam + spot (from the side mast arm) ----------------
    const beamMat = new THREE.MeshBasicMaterial({ color: 0xff1a0a,
      transparent: true, opacity: 0.7, toneMapped: false,
      blending: THREE.AdditiveBlending, depthWrite: false });
    const beam = new THREE.Mesh(new THREE.CylinderGeometry(4, 4, 1, 12),
      beamMat);
    beam.rotation.x = Math.PI / 2;
    root.add(beam);
    const spot = new THREE.Mesh(new THREE.CircleGeometry(26, 32),
      new THREE.MeshBasicMaterial({ color: 0xff2a1a, transparent: true,
        opacity: 0.95, toneMapped: false }));
    root.add(spot);
    scene.__beam = beam; scene.__spot = spot;
    scene.__laserX = (meta.mast_x || 660) - (meta.mast_arm || 192);
    scene.__laserTop = (meta.mast_z || 1600) - 150;

    // ---------------- load / reaction vectors ----------------
    const vecG = new THREE.Group();
    vecG.visible = false;
    root.add(vecG);
    const railXY = [[460, 460], [460, -460], [-460, 460], [-460, -460]];
    scene.__rails = [];
    railXY.forEach(function (p) {
      const ox = p[0] + Math.sign(p[0]) * 95;
      const oy = p[1] + Math.sign(p[1]) * 95;
      const up = makeArrow(new THREE.Vector3(0, 0, 1), 340, 0x2ee68a);
      up.position.set(ox, oy, meta.home_bed_z + 40);
      const down = makeArrow(new THREE.Vector3(0, 0, -1), 300, 0x46d6ff);
      down.position.set(ox, oy,
        meta.home_bed_z + (meta.stack_height_mm || 300) + 40);
      vecG.add(up, down);
      scene.__rails.push({ up: up, down: down, x: ox, y: oy });
    });
    scene.__vecG = vecG;

    // ------------- guided drag chain (procedural, flexes with the deck) ------
    const chainMeta = meta.chain;
    if (chainMeta) {
      (nodes.drag_chain || []).forEach(function (m) {
        staticG.remove(m); liftG.remove(m);
      });
      const chainG = new THREE.Group();
      root.add(chainG);
      const linkMat = mats.drag_chain || new THREE.MeshStandardMaterial(
        { color: 0x111111, metalness: 0.45, roughness: 0.55 });
      const linkGeo = new THREE.BoxGeometry(chainMeta.width, chainMeta.thick,
                                            chainMeta.link * 0.86);
      const linkCache = [];
      function linkPoses(zm) {
        const R = chainMeta.radius, rdy = chainMeta.run_dy;
        const yMove = chainMeta.y + rdy, yFix = chainMeta.y - rdy;
        const zf = chainMeta.fixed_z;
        const L = (chainMeta.z_m_home - chainMeta.z_b_home)
                + (zf - chainMeta.z_b_home) + Math.PI * R;
        const zb = (zm + zf - (L - Math.PI * R)) / 2;
        const m = 80, poly = [];
        for (let i = 0; i <= m; i++) poly.push([yMove, zm + (zb - zm) * i / m]);
        for (let i = 1; i <= m; i++) {
          const phi = -Math.PI * i / m;
          poly.push([chainMeta.y + R * Math.cos(phi), zb + R * Math.sin(phi)]);
        }
        for (let i = 1; i <= m; i++) poly.push([yFix, zb + (zf - zb) * i / m]);
        const dist = [0];
        for (let i = 1; i < poly.length; i++)
          dist.push(dist[i - 1] + Math.hypot(poly[i][0] - poly[i - 1][0],
                                             poly[i][1] - poly[i - 1][1]));
        const total = dist[dist.length - 1];
        const n = Math.max(2, Math.floor(total / chainMeta.link));
        const step = total / n, out = [];
        let j = 0;
        for (let k = 0; k < n; k++) {
          const target = (k + 0.5) * step;
          while (j < poly.length - 2 && dist[j + 1] < target) j++;
          const span = dist[j + 1] - dist[j];
          const f = span > 1e-9 ? (target - dist[j]) / span : 0;
          const y = poly[j][0] + (poly[j + 1][0] - poly[j][0]) * f;
          const z = poly[j][1] + (poly[j + 1][1] - poly[j][1]) * f;
          const th = Math.atan2(-(poly[j + 1][0] - poly[j][0]),
                                poly[j + 1][1] - poly[j][1]);
          out.push([y, z, th]);
        }
        return out;
      }
      function updateChain(bedZ) {
        const poses = linkPoses(chainMeta.z_m_home + bedZ);
        while (linkCache.length < poses.length) {
          const l = new THREE.Mesh(linkGeo, linkMat);
          l.castShadow = true; l.userData.part = "drag_chain";
          chainG.add(l); linkCache.push(l);
        }
        linkCache.forEach(function (l, k) {
          if (k < poses.length) {
            const p = poses[k];
            l.visible = true;
            l.position.set(chainMeta.x, p[0], p[1]);
            l.rotation.set(p[2], 0, 0);
          } else l.visible = false;
        });
      }
      updateChain(0);
      nodes.drag_chain = linkCache;
      scene.__updateChain = updateChain;
    }

    scene.root = root;
    scene.staticG = staticG;
    scene.liftG = liftG;
    scene.nodes = nodes;
    scene.rotNodes = rotNodes;
    return root;
  }

  global.MACHINE = { loadAll, build, planarUV };
})(window);
