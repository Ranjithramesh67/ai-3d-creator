/* Procedural PBR materials + HDRI-style environment for the sheet-lifter scene.
   STL geometry has no UVs, so machine.js generates a planar UV set; the maps
   below are generated procedurally on canvas (no external assets required). */
(function (global) {
  "use strict";
  const THREE = global.THREE;

  function canvas(w, h) {
    const c = document.createElement("canvas");
    c.width = w; c.height = h;
    return c;
  }

  // deterministic value noise
  function rng(seed) {
    let s = seed >>> 0;
    return function () {
      s = (s * 1664525 + 1013904223) >>> 0;
      return s / 4294967296;
    };
  }
  function valueNoise(w, h, cell, seed) {
    const r = rng(seed);
    const gw = Math.ceil(w / cell) + 1, gh = Math.ceil(h / cell) + 1;
    const g = new Float32Array(gw * gh);
    for (let i = 0; i < g.length; i++) g[i] = r();
    const sm = (t) => t * t * (3 - 2 * t);
    const out = new Float32Array(w * h);
    for (let y = 0; y < h; y++) {
      const fy = y / cell, y0 = Math.floor(fy), ty = sm(fy - y0);
      for (let x = 0; x < w; x++) {
        const fx = x / cell, x0 = Math.floor(fx), tx = sm(fx - x0);
        const a = g[y0 * gw + x0], b = g[y0 * gw + x0 + 1];
        const c = g[(y0 + 1) * gw + x0], d = g[(y0 + 1) * gw + x0 + 1];
        out[y * w + x] = (a * (1 - tx) + b * tx) * (1 - ty) +
                         (c * (1 - tx) + d * tx) * ty;
      }
    }
    return out;
  }
  function fbm(w, h, cell, seed, oct) {
    let acc = new Float32Array(w * h), amp = 1, tot = 0, c = cell;
    for (let o = 0; o < oct; o++) {
      const n = valueNoise(w, h, c, seed + o * 97);
      for (let i = 0; i < n.length; i++) acc[i] += n[i] * amp;
      tot += amp; amp *= 0.5; c = Math.max(2, c * 0.5);
    }
    for (let i = 0; i < acc.length; i++) acc[i] /= tot;
    return acc;
  }

  function texFrom(noise, w, h, lo, hi, repeat) {
    const c = canvas(w, h), ctx = c.getContext("2d");
    const img = ctx.createImageData(w, h);
    for (let i = 0; i < noise.length; i++) {
      const v = lo + (hi - lo) * noise[i];
      img.data[i * 4] = img.data[i * 4 + 1] = img.data[i * 4 + 2] = v * 255;
      img.data[i * 4 + 3] = 255;
    }
    ctx.putImageData(img, 0, 0);
    const t = new THREE.CanvasTexture(c);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    if (repeat) t.repeat.set(repeat, repeat);
    return t;
  }

  function normalFromHeight(noise, w, h, strength, repeat) {
    const c = canvas(w, h), ctx = c.getContext("2d");
    const img = ctx.createImageData(w, h);
    const s = strength;
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = y * w + x;
        const xl = noise[y * w + ((x - 1 + w) % w)];
        const xr = noise[y * w + ((x + 1) % w)];
        const yu = noise[((y - 1 + h) % h) * w + x];
        const yd = noise[((y + 1) % h) * w + x];
        let nx = (xl - xr) * s, ny = (yu - yd) * s, nz = 1;
        const l = Math.hypot(nx, ny, nz);
        img.data[i * 4] = (nx / l * 0.5 + 0.5) * 255;
        img.data[i * 4 + 1] = (ny / l * 0.5 + 0.5) * 255;
        img.data[i * 4 + 2] = (nz / l * 0.5 + 0.5) * 255;
        img.data[i * 4 + 3] = 255;
      }
    }
    ctx.putImageData(img, 0, 0);
    const t = new THREE.CanvasTexture(c);
    t.wrapS = t.wrapT = THREE.RepeatWrapping;
    if (repeat) t.repeat.set(repeat, repeat);
    return t;
  }

  function stripes(w, h, period, jitter, seed, strength, vertical) {
    const n = new Float32Array(w * h);
    const r = rng(seed);
    for (let i = 0; i < (vertical ? w : h); i++) {
      const v = 0.5 + (r() - 0.5) * jitter;
      for (let j = 0; j < (vertical ? h : w); j++) {
        const idx = vertical ? j * w + i : i * w + j;
        n[idx] = 0.5 + 0.5 * Math.sin((i % period) / period * Math.PI * 2) * 0.6 +
                 (v - 0.5) * 0.4;
      }
    }
    return normalFromHeight(n, w, h, strength, null);
  }

  // ---------- environment (bright overcast factory + high-bay rectangles) ----
  function buildEnvironment(renderer, scene) {
    const w = 1024, h = 512;
    const c = canvas(w, h), ctx = c.getContext("2d");
    const g = ctx.createLinearGradient(0, 0, 0, h);
    g.addColorStop(0.0, "#9fb2c8");
    g.addColorStop(0.45, "#c8d3de");
    g.addColorStop(0.5, "#b9c3cd");
    g.addColorStop(0.55, "#6d737a");
    g.addColorStop(1.0, "#2c3034");
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, w, h);
    // ceiling high-bay lamps
    ctx.fillStyle = "#ffffff";
    for (let i = 0; i < 7; i++) {
      const x = (i + 0.5) * (w / 7);
      ctx.fillRect(x - 42, 60, 84, 26);
      ctx.fillRect(x - 30, 92, 60, 10);
    }
    // warm floor bounce
    ctx.fillStyle = "rgba(120,110,95,0.25)";
    ctx.fillRect(0, h * 0.62, w, h * 0.38);
    const tex = new THREE.CanvasTexture(c);
    tex.mapping = THREE.EquirectangularReflectionMapping;
    tex.encoding = THREE.sRGBEncoding;
    const pmrem = new THREE.PMREMGenerator(renderer);
    pmrem.compileEquirectangularShader();
    const env = pmrem.fromEquirectangular(tex).texture;
    pmrem.dispose();
    tex.dispose();
    scene.environment = env;
    return env;
  }

  // ------------------------------- material set -----------------------------
  function makeMaps() {
    const N = 256;
    const M = {};
    M.powderRough = texFrom(fbm(N, N, 28, 11, 5), N, N, 0.42, 0.72, 3);
    const pc = fbm(N, N, 20, 7, 4);
    M.powderColor = texFrom(pc, N, N, 0.86, 1.0, 3);
    M.brush = stripes(256, 256, 128, 0.8, 5, 1.4, false);
    M.thread = stripes(128, 256, 10, 0.6, 21, 2.6, false);
    M.zinc = texFrom(fbm(N, N, 16, 31, 4), N, N, 0.5, 0.95, 2);
    M.zincRough = texFrom(fbm(N, N, 10, 41, 4), N, N, 0.18, 0.55, 2);
    M.castRough = texFrom(fbm(N, N, 24, 53, 5), N, N, 0.5, 0.85, 2);
    return M;
  }

  const SPEC = {
    base_frame:     { cat: "powder" },
    sensor_mast:    { cat: "powder" },
    bed_ribs:       { cat: "powder" },
    side_guide:     { cat: "powder" },
    back_guide:     { cat: "powder" },
    rail_support:   { cat: "cast", color: 0x5c6b78 },
    rail_post:      { cat: "cast", color: 0x5c6b78 },
    motor_mount:    { cat: "cast", color: 0x5c6b78 },
    gearbox_mount:  { cat: "cast", color: 0x5c6b78 },
    jack_stool:     { cat: "cast", color: 0x5c6b78 },
    jack_clamp:     { cat: "painted", color: 0x005a5b },
    nut_bracket:    { cat: "cast", color: 0x5c6b78 },
    bronze_nut:     { cat: "bronze" },
    chain_trough:   { cat: "cast", color: 0x5c6b78 },
    chain_bed_bracket: { cat: "cast", color: 0x5c6b78 },
    tslot_track:    { cat: "machined" },
    track_riser:    { cat: "cast", color: 0x5c6b78 },
    rail_pad:       { cat: "machined" },
    drive_shaft:    { cat: "brushed" },
    hgr35_rail:     { cat: "chrome" },
    hgw35_block:    { cat: "cast", color: 0x1a1a1a },
    wiper_seal:     { cat: "rubber", color: 0xc0392b },
    trapez_screw:   { cat: "thread" },
    worm_gear_jack: { cat: "painted" },
    bevel_gearbox:  { cat: "painted" },
    electric_motor: { cat: "motor" },
    disc_brake:     { cat: "cast", color: 0x181818 },
    jaw_coupling:   { cat: "rubber", color: 0xd35400 },
    laser_sensor:   { cat: "device" },
    cabinet:        { cat: "panel" },
    estop:          { cat: "rubber", color: 0xc0392b },
    hmi:            { cat: "device" },
    drag_chain:     { cat: "cable" },
    sheet_stack:    { cat: "zinc" },
    sheet_top:      { cat: "zinc" },
    suction_cup:    { cat: "rubber", color: 0x2a2a2a }
  };

  function buildMaterials(colors, maps) {
    const out = {};
    const col = (name) => new THREE.Color(colors[name] || "#888888");
    for (const name in SPEC) {
      const s = SPEC[name];
      const p = { name: name, color: s.color !== undefined ? s.color : col(name) };
      switch (s.cat) {
        case "powder":
          Object.assign(p, { metalness: 0.12, roughness: 0.55,
            roughnessMap: maps.powderRough, map: maps.powderColor,
            envMapIntensity: 0.8 }); break;
        case "machined":
          Object.assign(p, { metalness: 0.85, roughness: 0.35,
            roughnessMap: maps.powderRough, envMapIntensity: 1.0 }); break;
        case "brushed":
          Object.assign(p, { metalness: 0.95, roughness: 0.28,
            normalMap: maps.brush, envMapIntensity: 1.2 }); break;
        case "chrome":
          Object.assign(p, { metalness: 1.0, roughness: 0.13,
            normalMap: maps.brush, envMapIntensity: 1.6 }); break;
        case "cast":
          Object.assign(p, { metalness: 0.35, roughness: 0.7,
            roughnessMap: maps.castRough, envMapIntensity: 0.9 }); break;
        case "bronze":
          Object.assign(p, { metalness: 0.85, roughness: 0.42,
            roughnessMap: maps.castRough, envMapIntensity: 1.1 }); break;
        case "rubber":
          Object.assign(p, { metalness: 0.0, roughness: 0.62,
            envMapIntensity: 0.6 }); break;
        case "thread":
          Object.assign(p, { metalness: 0.9, roughness: 0.34,
            normalMap: maps.thread, envMapIntensity: 1.1 }); break;
        case "painted":
          Object.assign(p, { metalness: 0.25, roughness: 0.42,
            roughnessMap: maps.castRough, envMapIntensity: 1.0 }); break;
        case "motor":
          Object.assign(p, { metalness: 0.2, roughness: 0.5,
            roughnessMap: maps.castRough, envMapIntensity: 0.9 }); break;
        case "device":
          Object.assign(p, { metalness: 0.4, roughness: 0.4,
            envMapIntensity: 1.0 }); break;
        case "panel":
          Object.assign(p, { metalness: 0.1, roughness: 0.62,
            roughnessMap: maps.powderRough, envMapIntensity: 0.8 }); break;
        case "cable":
          Object.assign(p, { metalness: 0.0, roughness: 0.78,
            envMapIntensity: 0.5 }); break;
        case "zinc":
          Object.assign(p, { metalness: 0.9, roughness: 0.38,
            roughnessMap: maps.zincRough, map: maps.zinc,
            envMapIntensity: 1.3 }); break;
        default:
          Object.assign(p, { metalness: 0.4, roughness: 0.6 });
      }
      const m = new THREE.MeshStandardMaterial(p);
      m.userData = { base: { ...p } };
      out[name] = m;
    }
    // FEA heat-map material (bed) - replaced at runtime
    out.__fea = new THREE.MeshStandardMaterial({
      color: 0x1b4fd6, metalness: 0.2, roughness: 0.5,
      emissive: 0x0a2a6b, emissiveIntensity: 0.35
    });
    return out;
  }

  global.MAT = { buildEnvironment, makeMaps, buildMaterials, rng };
})(window);
