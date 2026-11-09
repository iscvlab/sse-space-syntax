// OpenDepthmap Workstation Frontend Logic (SSE Pro — Live CSR Backend Integration)

// 1. Color Palettes
const PALETTES = {
  swiss: [
    [37, 99, 235],   // #2563EB Segregated (Blue)
    [6, 182, 212],   // #06B6D4 Neighborhood (Cyan)
    [16, 185, 129],  // #10B981 Sub-Arterial (Green)
    [245, 158, 11],  // #F59E0B Primary (Amber)
    [239, 68, 68]    // #EF4444 Super-Arterial (Red)
  ],
  classic_rainbow: [
    [37, 99, 235],
    [6, 182, 212],
    [16, 185, 129],
    [245, 158, 11],
    [239, 68, 68]
  ],
  inverted_rainbow: [
    [239, 68, 68],
    [245, 158, 11],
    [16, 185, 129],
    [6, 182, 212],
    [37, 99, 235]
  ],
  thermal: [
    [40, 20, 80],
    [120, 28, 129],
    [217, 33, 90],
    [245, 130, 32],
    [254, 210, 50]
  ],
  greyscale: [
    [220, 220, 225],
    [175, 175, 180],
    [120, 120, 125],
    [70, 70, 75],
    [20, 20, 25]
  ],
  viridis: [
    [68, 1, 84],
    [59, 82, 139],
    [33, 145, 140],
    [94, 201, 98],
    [253, 231, 37]
  ]
};

// 2. Application State
const state = {
  rawLines: [],
  segments: [],
  activeMetric: 'nach_rn',
  activePalette: 'swiss',
  engineMode: 'Live CSR Backend',
  datasetName: 'Synthetic Superblock Grid',
  zoom: 1.0,
  panX: 0,
  panY: 0,
  isDragging: false,
  dragStartX: 0,
  dragStartY: 0,
  hoveredIndex: -1,
  bounds: { minX: 0, minY: 0, maxX: 3000, maxY: 2000 },
  scatterMinimized: false
};

// 3. Canvas & DOM References
const mapCanvas = document.getElementById('map-canvas');
const mapCtx = mapCanvas.getContext('2d');
const scatterCanvas = document.getElementById('scatter-canvas');
const scatterCtx = scatterCanvas.getContext('2d');
const tooltip = document.getElementById('tooltip');

function setEngineBadge(isLiveBackend) {
  state.engineMode = isLiveBackend ? 'Live CSR Backend' : 'Client Fallback';
  const label = `Engine: ${state.engineMode}`;
  const headerBadge = document.getElementById('engine-badge-header');
  const statusBadge = document.getElementById('status-engine-badge');
  const pill = document.getElementById('engine-pill');

  if (headerBadge) headerBadge.textContent = label;
  if (statusBadge) {
    statusBadge.innerHTML = `Engine: <strong>${state.engineMode}</strong>`;
  }
  if (pill) {
    pill.classList.toggle('fallback', !isLiveBackend);
  }
}

function recomputeBounds(segs) {
  if (!segs || segs.length === 0) {
    state.bounds = { minX: 0, minY: 0, maxX: 1000, maxY: 1000 };
    return;
  }
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const s of segs) {
    minX = Math.min(minX, s.x1, s.x2);
    minY = Math.min(minY, s.y1, s.y2);
    maxX = Math.max(maxX, s.x1, s.x2);
    maxY = Math.max(maxY, s.y1, s.y2);
  }
  state.bounds = { minX, minY, maxX, maxY };
}

// 4. Generate Synthetic Urban Superblock Network (Universal Benchmark Grid)
function generateSampleNetwork() {
  const segs = [];
  const rawLines = [];
  const W = 2400;
  const H = 1600;

  const arterialsY = [300, 800, 1300];
  const arterialsX = [400, 1200, 2000];
  let idCounter = 0;

  // Horizontal arterials
  for (const y of arterialsY) {
    let prevX = 0;
    for (let x = 400; x <= W; x += 400) {
      rawLines.push([[prevX, y], [x, y]]);
      segs.push({
        id: idCounter++,
        coords: [[prevX, y], [x, y]],
        x1: prevX, y1: y, x2: x, y2: y,
        length_m: 400,
        nach_rn: 1.38,
        nain_rn: 1.15,
        choice_rn: 4200,
        integration_rn: 18.5,
        nach_r800: 1.32,
        nain_r800: 1.08,
        choice_r800: 950
      });
      prevX = x;
    }
  }

  // Vertical arterials
  for (const x of arterialsX) {
    let prevY = 0;
    for (let y = 300; y <= H; y += 500) {
      const targetY = Math.min(y, H);
      rawLines.push([[x, prevY], [x, targetY]]);
      segs.push({
        id: idCounter++,
        coords: [[x, prevY], [x, targetY]],
        x1: x, y1: prevY, x2: x, y2: targetY,
        length_m: targetY - prevY,
        nach_rn: 1.31,
        nain_rn: 1.09,
        choice_rn: 3100,
        integration_rn: 16.2,
        nach_r800: 1.26,
        nain_r800: 1.02,
        choice_r800: 780
      });
      prevY = targetY;
    }
    if (prevY < H) {
      rawLines.push([[x, prevY], [x, H]]);
      segs.push({
        id: idCounter++,
        coords: [[x, prevY], [x, H]],
        x1: x, y1: prevY, x2: x, y2: H,
        length_m: H - prevY,
        nach_rn: 1.25,
        nain_rn: 1.04,
        choice_rn: 2400,
        integration_rn: 14.8,
        nach_r800: 1.20,
        nain_r800: 0.96,
        choice_r800: 620
      });
    }
  }

  // Interior Connector & Dendritic Sois (with realistic 4m–12m digitizing undershoots for cleaner testing)
  const soiSeeds = [
    [[400, 550], [792, 550]],
    [[800, 300], [800, 794]],
    [[800, 550], [1191, 550]],
    [[1200, 550], [1593, 550]],
    [[1600, 300], [1600, 793]],
    [[1600, 550], [1992, 550]],
    [[400, 1050], [1192, 1050]],
    [[1200, 1050], [1991, 1050]],
    [[600, 550], [600, 720]],
    [[1000, 550], [1000, 410]],
    [[1400, 1050], [1400, 1210]],
    [[1800, 1050], [1800, 890]]
  ];

  for (const [p1, p2] of soiSeeds) {
    const len = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
    rawLines.push([p1, p2]);
    segs.push({
      id: idCounter++,
      coords: [p1, p2],
      x1: p1[0], y1: p1[1], x2: p2[0], y2: p2[1],
      length_m: Math.round(len * 10) / 10,
      nach_rn: 0.85,
      nain_rn: 0.74,
      choice_rn: 420,
      integration_rn: 9.4,
      nach_r800: 0.82,
      nain_r800: 0.70,
      choice_r800: 180
    });
  }

  state.rawLines = rawLines;
  state.segments = segs;
  state.datasetName = 'Synthetic Superblock Grid';
  const dsEl = document.getElementById('status-dataset');
  if (dsEl) dsEl.innerHTML = `Source: <strong>${state.datasetName}</strong>`;
  recomputeBounds(state.segments);
  updateScorecard({
    segment_count: segs.length,
    graph_ram_kb: +(segs.length * 0.12).toFixed(2),
    elapsed_ms: 0
  });
}

// 5. Parse Radii String into API Array
function parseRadiiTokens(radiiStr) {
  const tokens = String(radiiStr || 'n, 800, 400')
    .split(',')
    .map(t => t.trim().toLowerCase())
    .filter(Boolean);
  const out = [];
  for (const t of tokens) {
    if (t === 'n' || t === 'rn' || t === 'inf' || t === 'global') {
      out.push('n');
    } else {
      const num = parseFloat(t);
      if (!isNaN(num) && num > 0) out.push(num);
    }
  }
  return out.length > 0 ? out : ['n', 800, 400];
}

// 6. Client-Side Fallback Solver (when opened via file:// or server is offline)
function runClientFallbackAnalysis(t0) {
  const N = state.segments.length;
  if (N === 0) return;

  // Build simple endpoint adjacency to approximate topological degree & distance centrality
  const ptMap = new Map();
  const keyOf = (x, y) => `${Math.round(x / 12)},${Math.round(y / 12)}`;

  state.segments.forEach((s, idx) => {
    const k1 = keyOf(s.x1, s.y1);
    const k2 = keyOf(s.x2, s.y2);
    if (!ptMap.has(k1)) ptMap.set(k1, []);
    if (!ptMap.has(k2)) ptMap.set(k2, []);
    ptMap.get(k1).push(idx);
    ptMap.get(k2).push(idx);
  });

  const cx = (state.bounds.minX + state.bounds.maxX) * 0.5;
  const cy = (state.bounds.minY + state.bounds.maxY) * 0.5;
  const maxSpan = Math.hypot(state.bounds.maxX - state.bounds.minX, state.bounds.maxY - state.bounds.minY) || 1000;

  for (let i = 0; i < N; i++) {
    const s = state.segments[i];
    const deg1 = (ptMap.get(keyOf(s.x1, s.y1)) || []).length;
    const deg2 = (ptMap.get(keyOf(s.x2, s.y2)) || []).length;
    const connectivity = Math.max(1, deg1 + deg2 - 2);
    const mx = (s.x1 + s.x2) * 0.5;
    const my = (s.y1 + s.y2) * 0.5;
    const centrality = Math.max(0.15, 1.0 - Math.hypot(mx - cx, my - cy) / (maxSpan * 0.65));

    const choiceRn = Math.round(connectivity * centrality * s.length_m * 1.8);
    const totalDepthRn = Math.max(2.0, (N * 1.4) / (connectivity * 0.6 + centrality));
    const meanDepthRn = +(totalDepthRn / Math.max(1, N - 1)).toFixed(4);
    const integRn = +(N / totalDepthRn).toFixed(4);
    const nachRn = +(Math.log(choiceRn + 1.0) / Math.log(totalDepthRn + 3.0)).toFixed(4);
    const nainRn = +(Math.pow(N, 1.2) / (totalDepthRn + 2.0)).toFixed(4);

    s.connectivity = connectivity;
    s.angular_connectivity = +(connectivity * 0.85).toFixed(4);
    s.mean_depth = meanDepthRn;
    s.mean_depth_rn = meanDepthRn;
    s.choice_rn = choiceRn;
    s.integration_rn = integRn;
    s.nach_rn = nachRn;
    s.nain_rn = nainRn;
    s.nach_r800 = +(nachRn * 0.94).toFixed(4);
    s.nain_r800 = +(nainRn * 0.91).toFixed(4);
    s.choice_r800 = Math.round(choiceRn * 0.35);
  }

  const elapsedMs = Math.round((performance.now() - t0) * 10) / 10;
  setEngineBadge(false);
  updateScorecard({
    segment_count: N,
    graph_ram_kb: +(N * 0.09).toFixed(2),
    elapsed_ms: elapsedMs
  });
}

// 7. Live CSR Analysis Request (`POST /api/analyze`)
async function runAnalysis() {
  const btn = document.getElementById('btn-run-analysis');
  if (btn) {
    btn.textContent = 'Solving CSR...';
    btn.style.opacity = '0.75';
  }

  const t0 = performance.now();
  const linesPayload = state.rawLines.length > 0
    ? state.rawLines
    : state.segments.map(s => [[s.x1, s.y1], [s.x2, s.y2]]);

  const radiiInput = document.getElementById('inp-radii');
  const cleanModeSelect = document.getElementById('sel-clean-mode');
  const extendDistInput = document.getElementById('inp-extend-dist');
  const normCheckbox = document.getElementById('chk-normalized');
  const analysisTypeSelect = document.getElementById('sel-analysis-type');
  const radiusTypeSelect = document.getElementById('sel-radius-type');
  const weightModeSelect = document.getElementById('sel-weight-mode');
  const tulipBinsSelect = document.getElementById('sel-tulip-bins');

  const requestBody = {
    lines: linesPayload,
    radii: parseRadiiTokens(radiiInput ? radiiInput.value : 'n, 800, 400'),
    clean_mode: cleanModeSelect ? cleanModeSelect.value : 'auto',
    extend_distance: extendDistInput ? parseFloat(extendDistInput.value) || 15.0 : 15.0,
    include_normalized: normCheckbox ? Boolean(normCheckbox.checked) : true,
    analysis_type: analysisTypeSelect ? analysisTypeSelect.value : 'angular',
    radius_type: radiusTypeSelect ? radiusTypeSelect.value : 'metric',
    weight_mode: weightModeSelect ? weightModeSelect.value : 'unweighted',
    tulip_bins: tulipBinsSelect ? parseInt(tulipBinsSelect.value, 10) || 0 : 0
  };

  try {
    const response = await fetch('/api/analyze', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(requestBody)
    });

    if (!response.ok) {
      throw new Error(`HTTP ${response.status}`);
    }

    const data = await response.json();
    if (data.status !== 'ok' || !Array.isArray(data.segments)) {
      throw new Error(data.message || 'Invalid CSR payload');
    }

    state.segments = data.segments.map(seg => {
      const c0 = seg.coords && seg.coords[0] ? seg.coords[0] : [0, 0];
      const c1 = seg.coords && seg.coords[1] ? seg.coords[1] : [0, 0];
      return {
        ...seg,
        x1: Number(c0[0]),
        y1: Number(c0[1]),
        x2: Number(c1[0]),
        y2: Number(c1[1])
      };
    });

    setEngineBadge(true);
    recomputeBounds(state.segments);
    updateScorecard(data);
  } catch (_err) {
    runClientFallbackAnalysis(t0);
  } finally {
    if (btn) {
      btn.innerHTML = '<svg viewBox="0 0 24 24" width="14" height="14" fill="currentColor"><polygon points="5 3 19 12 5 21 5 3"/></svg> Run Analysis';
      btn.style.opacity = '1';
    }
    renderMap();
    renderScatterPlot();
  }
}

// 8. Update Sidebar Scorecard & Telemetry
function updateScorecard(meta = {}) {
  const segs = state.segments;
  const N = segs.length;

  const elSegs = document.getElementById('stat-segments');
  const elRam = document.getElementById('stat-ram');
  const elTime = document.getElementById('stat-time');
  const elR2 = document.getElementById('stat-r2');
  const elNachRn = document.getElementById('stat-nach-rn');
  const elNainRn = document.getElementById('stat-nain-rn');
  const elChoiceRn = document.getElementById('stat-choice-rn');
  const elIntegRn = document.getElementById('stat-integ-rn');

  if (elSegs) elSegs.textContent = N.toLocaleString();
  if (elRam) {
    const kb = meta.graph_ram_kb !== undefined ? meta.graph_ram_kb : (N * 0.08);
    elRam.textContent = kb >= 1024 ? `${(kb / 1024).toFixed(2)} MB` : `${kb.toFixed(1)} KB`;
  }
  if (elTime) {
    const ms = meta.elapsed_ms !== undefined ? meta.elapsed_ms : 0;
    elTime.textContent = ms >= 1000 ? `${(ms / 1000).toFixed(2)} s` : `${ms.toFixed(1)} ms`;
  }

  if (N === 0) return;

  let sumNach = 0, sumNain = 0, maxChoice = 0, sumInteg = 0;
  let sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0, sumY2 = 0;

  for (const s of segs) {
    const nach = Number(s.nach_rn) || 0;
    const nain = Number(s.nain_rn) || 0;
    const choice = Number(s.choice_rn) || 0;
    const integ = Number(s.integration_rn) || 0;

    sumNach += nach;
    sumNain += nain;
    if (choice > maxChoice) maxChoice = choice;
    sumInteg += integ;

    sumX += integ;
    sumY += nach;
    sumXY += integ * nach;
    sumX2 += integ * integ;
    sumY2 += nach * nach;
  }

  const num = N * sumXY - sumX * sumY;
  const den = Math.sqrt((N * sumX2 - sumX * sumX) * (N * sumY2 - sumY * sumY)) || 1;
  const fallbackR2 = Math.min(1.0, Math.max(0.0, (num / den) * (num / den)));
  const diagR2 = meta.diagnostics && meta.diagnostics.intelligibility_r2 !== undefined
    ? Number(meta.diagnostics.intelligibility_r2)
    : fallbackR2;

  if (elR2) elR2.textContent = diagR2.toFixed(3);
  if (elNachRn) elNachRn.textContent = (sumNach / N).toFixed(3);
  if (elNainRn) elNainRn.textContent = (sumNain / N).toFixed(3);
  if (elChoiceRn) elChoiceRn.textContent = Math.round(maxChoice).toLocaleString();
  if (elIntegRn) elIntegRn.textContent = (sumInteg / N).toFixed(2);
}

// 9. Universal GeoJSON Importer (Auto-Projects WGS84 Lon/Lat to Metric Planar)
function importGeoJSONObject(geojson, filename = 'Imported.geojson') {
  const rawChains = [];

  function extractGeom(geom) {
    if (!geom) return;
    if (geom.type === 'LineString' && Array.isArray(geom.coordinates)) {
      if (geom.coordinates.length >= 2) rawChains.push(geom.coordinates);
    } else if (geom.type === 'MultiLineString' && Array.isArray(geom.coordinates)) {
      for (const part of geom.coordinates) {
        if (Array.isArray(part) && part.length >= 2) rawChains.push(part);
      }
    } else if (geom.type === 'GeometryCollection' && Array.isArray(geom.geometries)) {
      geom.geometries.forEach(extractGeom);
    }
  }

  if (geojson.type === 'FeatureCollection' && Array.isArray(geojson.features)) {
    for (const f of geojson.features) {
      if (f && f.geometry) extractGeom(f.geometry);
    }
  } else if (geojson.type === 'Feature' && geojson.geometry) {
    extractGeom(geojson.geometry);
  } else {
    extractGeom(geojson);
  }

  if (rawChains.length === 0) {
    alert('No valid LineString or MultiLineString features found in GeoJSON.');
    return;
  }

  // Inspect coordinate extent to detect WGS84 degrees vs projected metric coordinates
  let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
  for (const chain of rawChains) {
    for (const pt of chain) {
      const x = Number(pt[0]);
      const y = Number(pt[1]);
      if (!isNaN(x) && !isNaN(y)) {
        if (x < minX) minX = x;
        if (x > maxX) maxX = x;
        if (y < minY) minY = y;
        if (y > maxY) maxY = y;
      }
    }
  }

  const isGeographicWGS84 =
    Math.abs(minX) <= 180 &&
    Math.abs(maxX) <= 180 &&
    Math.abs(minY) <= 90 &&
    Math.abs(maxY) <= 90 &&
    (maxX - minX) < 10 &&
    (maxY - minY) < 10;

  const lat0Rad = (((minY + maxY) * 0.5) * Math.PI) / 180.0;
  const metersPerDegLat = 111132.92 - 559.82 * Math.cos(2.0 * lat0Rad);
  const metersPerDegLon = 111412.84 * Math.cos(lat0Rad);

  const projectedLines = [];
  const initialSegments = [];
  let segId = 0;

  for (const chain of rawChains) {
    const metricChain = [];
    for (const pt of chain) {
      const rawX = Number(pt[0]);
      const rawY = Number(pt[1]);
      if (isNaN(rawX) || isNaN(rawY)) continue;

      let mx, my;
      if (isGeographicWGS84) {
        mx = (rawX - minX) * metersPerDegLon;
        my = (maxY - rawY) * metersPerDegLat;
      } else {
        mx = rawX - minX;
        my = maxY - rawY;
      }
      metricChain.push([Math.round(mx * 1000) / 1000, Math.round(my * 1000) / 1000]);
    }

    if (metricChain.length >= 2) {
      projectedLines.push(metricChain);
      for (let i = 0; i < metricChain.length - 1; i++) {
        const p1 = metricChain[i];
        const p2 = metricChain[i + 1];
        const len = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
        if (len > 0.05) {
          initialSegments.push({
            id: segId++,
            coords: [p1, p2],
            x1: p1[0], y1: p1[1], x2: p2[0], y2: p2[1],
            length_m: Math.round(len * 100) / 100,
            nach_rn: 1.0,
            nain_rn: 1.0,
            choice_rn: 0,
            integration_rn: 1.0,
            nach_r800: 1.0,
            nain_r800: 1.0
          });
        }
      }
    }
  }

  state.rawLines = projectedLines;
  state.segments = initialSegments;
  state.datasetName = `${filename} (${isGeographicWGS84 ? 'WGS84 → Metric' : 'Metric'})`;
  const dsEl = document.getElementById('status-dataset');
  if (dsEl) dsEl.innerHTML = `Source: <strong>${state.datasetName}</strong>`;

  recomputeBounds(state.segments);
  fitExtent();
  runAnalysis();
}

function exportAnalyzedGeoJSON() {
  const features = state.segments.map(s => ({
    type: 'Feature',
    geometry: {
      type: 'LineString',
      coordinates: [[s.x1, s.y1], [s.x2, s.y2]]
    },
    properties: {
      id: s.id,
      length_m: s.length_m,
      connectivity: s.connectivity,
      mean_depth: s.mean_depth !== undefined ? s.mean_depth : s.mean_depth_rn,
      nach_rn: s.nach_rn,
      nain_rn: s.nain_rn,
      choice_rn: s.choice_rn,
      integration_rn: s.integration_rn,
      nach_r800: s.nach_r800,
      nain_r800: s.nain_r800
    }
  }));

  const fc = {
    type: 'FeatureCollection',
    name: 'opendepthmap_csr_segments',
    features
  };

  const blob = new Blob([JSON.stringify(fc, null, 2)], { type: 'application/json' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = 'opendepthmap_csr_analyzed.geojson';
  a.click();
  URL.revokeObjectURL(url);
}

// 10. Color Mapping
function getColorForValue(val, minVal, maxVal, paletteName) {
  const palette = PALETTES[paletteName] || PALETTES.swiss;
  if (val === undefined || isNaN(val) || maxVal === minVal) return '#6b7280';
  const norm = Math.max(0, Math.min(1, (val - minVal) / (maxVal - minVal)));
  const scaled = norm * (palette.length - 1);
  const idx = Math.floor(scaled);
  const frac = scaled - idx;

  if (idx >= palette.length - 1) {
    const c = palette[palette.length - 1];
    return `rgb(${c[0]}, ${c[1]}, ${c[2]})`;
  }

  const c1 = palette[idx];
  const c2 = palette[idx + 1];
  const r = Math.round(c1[0] + frac * (c2[0] - c1[0]));
  const g = Math.round(c1[1] + frac * (c2[1] - c1[1]));
  const b = Math.round(c1[2] + frac * (c2[2] - c1[2]));
  return `rgb(${r}, ${g}, ${b})`;
}

// 11. View Transforms
function worldToScreen(wx, wy) {
  return {
    sx: (wx + state.panX) * state.zoom,
    sy: (wy + state.panY) * state.zoom
  };
}

function screenToWorld(sx, sy) {
  return {
    wx: sx / state.zoom - state.panX,
    wy: sy / state.zoom - state.panY
  };
}

function fitExtent() {
  const rect = mapCanvas.getBoundingClientRect();
  const w = rect.width || 800;
  const h = rect.height || 600;

  const bw = state.bounds.maxX - state.bounds.minX || 1000;
  const bh = state.bounds.maxY - state.bounds.minY || 1000;

  const scaleX = (w * 0.85) / bw;
  const scaleY = (h * 0.85) / bh;
  state.zoom = Math.min(scaleX, scaleY);

  const cx = (state.bounds.minX + state.bounds.maxX) * 0.5;
  const cy = (state.bounds.minY + state.bounds.maxY) * 0.5;

  state.panX = (w * 0.5) / state.zoom - cx;
  state.panY = (h * 0.5) / state.zoom - cy;

  renderMap();
}

// 12. Render 2D Vector Map
function renderMap() {
  const dpr = window.devicePixelRatio || 1;
  const rect = mapCanvas.getBoundingClientRect();
  mapCanvas.width = rect.width * dpr;
  mapCanvas.height = rect.height * dpr;
  mapCtx.scale(dpr, dpr);

  mapCtx.fillStyle = '#08090b';
  mapCtx.fillRect(0, 0, rect.width, rect.height);

  let minVal = Infinity;
  let maxVal = -Infinity;
  for (const s of state.segments) {
    const v = s[state.activeMetric];
    if (v !== undefined && !isNaN(v)) {
      if (v < minVal) minVal = v;
      if (v > maxVal) maxVal = v;
    }
  }

  if (minVal !== Infinity) {
    document.getElementById('legend-min').textContent = minVal.toFixed(2);
    document.getElementById('legend-mid').textContent = ((minVal + maxVal) * 0.5).toFixed(2);
    document.getElementById('legend-max').textContent = maxVal.toFixed(2);
  }

  // Update Scale Bar
  const scaleMeters = Math.round(80 / (state.zoom || 1));
  const scaleLabel = document.getElementById('scale-label');
  if (scaleLabel) {
    scaleLabel.textContent = scaleMeters >= 1000 ? `${(scaleMeters / 1000).toFixed(1)} km` : `${scaleMeters} m`;
  }

  // Render Grid Crosshairs
  mapCtx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
  mapCtx.lineWidth = 1;
  const step = 200 * state.zoom;
  if (step > 20) {
    for (let x = (state.panX * state.zoom) % step; x < rect.width; x += step) {
      mapCtx.beginPath();
      mapCtx.moveTo(x, 0);
      mapCtx.lineTo(x, rect.height);
      mapCtx.stroke();
    }
    for (let y = (state.panY * state.zoom) % step; y < rect.height; y += step) {
      mapCtx.beginPath();
      mapCtx.moveTo(0, y);
      mapCtx.lineTo(rect.width, y);
      mapCtx.stroke();
    }
  }

  // Draw Line Segments
  mapCtx.lineCap = 'round';
  for (let i = 0; i < state.segments.length; i++) {
    const s = state.segments[i];
    const p1 = worldToScreen(s.x1, s.y1);
    const p2 = worldToScreen(s.x2, s.y2);

    const val = s[state.activeMetric];
    mapCtx.strokeStyle = getColorForValue(val, minVal, maxVal, state.activePalette);
    mapCtx.lineWidth = i === state.hoveredIndex ? 3.8 : Math.max(1.4, 2.0 * state.zoom);

    if (i === state.hoveredIndex) {
      mapCtx.strokeStyle = '#38bdf8';
    }

    mapCtx.beginPath();
    mapCtx.moveTo(p1.sx, p1.sy);
    mapCtx.lineTo(p2.sx, p2.sy);
    mapCtx.stroke();
  }
}

// 13. Render Bivariate Scatter Plot & Linear Regression
function renderScatterPlot() {
  const dpr = window.devicePixelRatio || 1;
  const rect = scatterCanvas.getBoundingClientRect();
  scatterCanvas.width = rect.width * dpr;
  scatterCanvas.height = rect.height * dpr;
  scatterCtx.scale(dpr, dpr);

  const w = rect.width;
  const h = rect.height;

  scatterCtx.fillStyle = '#13151b';
  scatterCtx.fillRect(0, 0, w, h);

  const xMetric = document.getElementById('sel-scatter-x').value;
  const yMetric = document.getElementById('sel-scatter-y').value;

  const points = [];
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;

  for (const s of state.segments) {
    const x = s[xMetric];
    const y = s[yMetric];
    if (x !== undefined && y !== undefined && !isNaN(x) && !isNaN(y)) {
      points.push({ x, y, s });
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
  }

  if (points.length < 2) return;

  const pad = 28;
  const plotW = w - pad * 2;
  const plotH = h - pad * 2;

  scatterCtx.strokeStyle = '#282d37';
  scatterCtx.lineWidth = 1;
  scatterCtx.beginPath();
  scatterCtx.moveTo(pad, pad);
  scatterCtx.lineTo(pad, h - pad);
  scatterCtx.lineTo(w - pad, h - pad);
  scatterCtx.stroke();

  for (const pt of points) {
    const sx = pad + ((pt.x - minX) / (maxX - minX || 1)) * plotW;
    const sy = (h - pad) - ((pt.y - minY) / (maxY - minY || 1)) * plotH;

    scatterCtx.fillStyle = getColorForValue(pt.y, minY, maxY, state.activePalette);
    scatterCtx.beginPath();
    scatterCtx.arc(sx, sy, 2.2, 0, Math.PI * 2);
    scatterCtx.fill();
  }

  let sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0, sumY2 = 0;
  const N = points.length;
  for (const pt of points) {
    sumX += pt.x;
    sumY += pt.y;
    sumXY += pt.x * pt.y;
    sumX2 += pt.x * pt.x;
    sumY2 += pt.y * pt.y;
  }

  const slope = (N * sumXY - sumX * sumY) / (N * sumX2 - sumX * sumX || 1);
  const intercept = (sumY - slope * sumX) / N;

  const num = N * sumXY - sumX * sumY;
  const den = Math.sqrt((N * sumX2 - sumX * sumX) * (N * sumY2 - sumY * sumY) || 1);
  const rVal = num / den;
  const r2 = Math.min(1.0, Math.max(0.0, rVal * rVal));

  document.getElementById('scatter-r2').textContent = `R² = ${r2.toFixed(3)} (${r2 > 0.5 ? 'Strong' : 'Moderate'})`;

  const startYVal = slope * minX + intercept;
  const endYVal = slope * maxX + intercept;

  const lx1 = pad;
  const ly1 = (h - pad) - ((startYVal - minY) / (maxY - minY || 1)) * plotH;
  const lx2 = w - pad;
  const ly2 = (h - pad) - ((endYVal - minY) / (maxY - minY || 1)) * plotH;

  scatterCtx.strokeStyle = '#ef4444';
  scatterCtx.lineWidth = 1.8;
  scatterCtx.setLineDash([4, 3]);
  scatterCtx.beginPath();
  scatterCtx.moveTo(lx1, ly1);
  scatterCtx.lineTo(lx2, ly2);
  scatterCtx.stroke();
  scatterCtx.setLineDash([]);
}

// 14. Mouse & Drag-and-Drop Interactions
mapCanvas.addEventListener('mousedown', (e) => {
  state.isDragging = true;
  state.dragStartX = e.clientX;
  state.dragStartY = e.clientY;
});

window.addEventListener('mouseup', () => {
  state.isDragging = false;
});

mapCanvas.addEventListener('mousemove', (e) => {
  const rect = mapCanvas.getBoundingClientRect();
  const mouseX = e.clientX - rect.left;
  const mouseY = e.clientY - rect.top;

  if (state.isDragging) {
    const dx = e.clientX - state.dragStartX;
    const dy = e.clientY - state.dragStartY;
    state.panX += dx / state.zoom;
    state.panY += dy / state.zoom;
    state.dragStartX = e.clientX;
    state.dragStartY = e.clientY;
    renderMap();
  } else {
    const world = screenToWorld(mouseX, mouseY);
    document.getElementById('status-coords').textContent =
      `Metric X: ${world.wx.toFixed(1)} m, Y: ${world.wy.toFixed(1)} m`;

    let closestIdx = -1;
    let minDist = 12 / state.zoom;

    for (let i = 0; i < state.segments.length; i++) {
      const s = state.segments[i];
      const d = distToSegment(world.wx, world.wy, s.x1, s.y1, s.x2, s.y2);
      if (d < minDist) {
        minDist = d;
        closestIdx = i;
      }
    }

    if (closestIdx !== state.hoveredIndex) {
      state.hoveredIndex = closestIdx;
      renderMap();

      if (closestIdx !== -1) {
        const seg = state.segments[closestIdx];
        tooltip.classList.remove('hidden');
        tooltip.style.left = `${e.clientX + 14}px`;
        tooltip.style.top = `${e.clientY + 14}px`;
        document.getElementById('tt-title').textContent = `Segment #${seg.id}`;
        const ttConn = document.getElementById('tt-conn');
        if (ttConn) ttConn.textContent = seg.connectivity !== undefined ? String(seg.connectivity) : 'N/A';
        document.getElementById('tt-nach-rn').textContent = seg.nach_rn !== undefined ? Number(seg.nach_rn).toFixed(3) : 'N/A';
        document.getElementById('tt-nain-rn').textContent = seg.nain_rn !== undefined ? Number(seg.nain_rn).toFixed(3) : 'N/A';
        document.getElementById('tt-choice-rn').textContent = seg.choice_rn !== undefined ? Number(seg.choice_rn).toLocaleString() : '0';
        document.getElementById('tt-integ-rn').textContent = seg.integration_rn !== undefined ? Number(seg.integration_rn).toFixed(3) : 'N/A';
        const ttMd = document.getElementById('tt-md-rn');
        if (ttMd) {
          const mdVal = seg.mean_depth !== undefined ? seg.mean_depth : seg.mean_depth_rn;
          ttMd.textContent = mdVal !== undefined ? Number(mdVal).toFixed(3) : 'N/A';
        }
        document.getElementById('tt-nach').textContent = seg.nach_r800 !== undefined ? Number(seg.nach_r800).toFixed(3) : 'N/A';
        document.getElementById('tt-len').textContent = `${seg.length_m} m`;
      } else {
        tooltip.classList.add('hidden');
      }
    }
  }
});

mapCanvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  const rect = mapCanvas.getBoundingClientRect();
  const mouseX = e.clientX - rect.left;
  const mouseY = e.clientY - rect.top;

  const worldBefore = screenToWorld(mouseX, mouseY);
  const factor = e.deltaY < 0 ? 1.15 : 0.87;
  state.zoom *= factor;

  const worldAfter = screenToWorld(mouseX, mouseY);
  state.panX += worldAfter.wx - worldBefore.wx;
  state.panY += worldAfter.wy - worldBefore.wy;

  renderMap();
}, { passive: false });

// Drag-and-drop GeoJSON onto canvas
mapCanvas.addEventListener('dragover', (e) => {
  e.preventDefault();
});

mapCanvas.addEventListener('drop', (e) => {
  e.preventDefault();
  const file = e.dataTransfer && e.dataTransfer.files && e.dataTransfer.files[0];
  if (file) {
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const obj = JSON.parse(String(ev.target.result));
        importGeoJSONObject(obj, file.name);
      } catch (err) {
        alert(`Failed to parse GeoJSON file: ${err.message}`);
      }
    };
    reader.readAsText(file);
  }
});

function distToSegment(px, py, x1, y1, x2, y2) {
  const l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  if (l2 === 0) return Math.hypot(px - x1, py - y1);
  let t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / l2;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (x1 + t * (x2 - x1)), py - (y1 + t * (y2 - y1)));
}

// 15. UI Control Listeners
document.getElementById('sel-metric').addEventListener('change', (e) => {
  state.activeMetric = e.target.value;
  renderMap();
  renderScatterPlot();
});

document.getElementById('sel-palette').addEventListener('change', (e) => {
  state.activePalette = e.target.value;
  renderMap();
  renderScatterPlot();
});

document.getElementById('sel-scatter-x').addEventListener('change', renderScatterPlot);
document.getElementById('sel-scatter-y').addEventListener('change', renderScatterPlot);

document.querySelectorAll('.pill-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.pill-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById('inp-radii').value = btn.dataset.radii;
  });
});

document.getElementById('btn-zoom-in').addEventListener('click', () => {
  state.zoom *= 1.25;
  renderMap();
});

document.getElementById('btn-zoom-out').addEventListener('click', () => {
  state.zoom *= 0.8;
  renderMap();
});

document.getElementById('btn-reset-view').addEventListener('click', fitExtent);
document.getElementById('btn-fit-extent').addEventListener('click', fitExtent);

const fileInput = document.getElementById('file-input');
const triggerFileOpen = () => {
  if (fileInput) {
    fileInput.value = '';
    fileInput.click();
  }
};

const btnImportGeoJSON = document.getElementById('btn-import-geojson');
if (btnImportGeoJSON) btnImportGeoJSON.addEventListener('click', triggerFileOpen);

const btnOpenFile = document.getElementById('btn-open-file');
if (btnOpenFile) btnOpenFile.addEventListener('click', triggerFileOpen);

if (fileInput) {
  fileInput.addEventListener('change', (e) => {
    const file = e.target.files && e.target.files[0];
    if (!file) return;
    const reader = new FileReader();
    reader.onload = (ev) => {
      try {
        const obj = JSON.parse(String(ev.target.result));
        importGeoJSONObject(obj, file.name);
      } catch (err) {
        alert(`Failed to parse GeoJSON: ${err.message}`);
      }
    };
    reader.readAsText(file);
  });
}

const btnResetSample = document.getElementById('btn-reset-sample');
if (btnResetSample) {
  btnResetSample.addEventListener('click', () => {
    generateSampleNetwork();
    fitExtent();
    runAnalysis();
  });
}

const btnExportGpkg = document.getElementById('btn-export-gpkg');
if (btnExportGpkg) {
  btnExportGpkg.addEventListener('click', exportAnalyzedGeoJSON);
}

document.getElementById('btn-toggle-scatter').addEventListener('click', () => {
  const p = document.getElementById('scatter-panel');
  state.scatterMinimized = !state.scatterMinimized;
  p.classList.toggle('minimized', state.scatterMinimized);
  document.getElementById('btn-toggle-scatter').textContent = state.scatterMinimized ? '▲ Maximize' : '▼ Minimize';
  setTimeout(renderMap, 220);
});

document.getElementById('btn-run-analysis').addEventListener('click', () => {
  runAnalysis();
});

// 16. Window Resize & Initial CSR Solve on Startup
window.addEventListener('resize', () => {
  renderMap();
  renderScatterPlot();
});

generateSampleNetwork();
setTimeout(() => {
  fitExtent();
  runAnalysis();
}, 50);
