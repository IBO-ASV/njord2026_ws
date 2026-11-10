const FIX_TOPIC = "/sensor/vehicle_gnss/fix/raw";
const SPEED_TOPIC = "/gui/ground_speed_mps";
const BATTERY_PERCENT_TOPIC = "/gui/battery_percent";
const TF_TOPIC = "/tf";
const TF_STATIC_TOPIC = "/tf_static";
const GROUND_WAYPOINT_MARKERS_TOPIC = "/ground_waypoint_markers";
const WORLD_FRAME = "odom";
const VESSEL_FRAME = "base_link";

const PANEL_CSS = `
.gnss-map-root{height:100%;position:relative;overflow:hidden;background:#15202b}
.gnss-map-canvas{height:100%;width:100%;font-family:system-ui,sans-serif}
.leaflet-pane,.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow,.leaflet-tile-container,
.leaflet-pane>svg,.leaflet-pane>canvas,.leaflet-zoom-box,.leaflet-image-layer,.leaflet-layer{position:absolute;left:0;top:0}
.leaflet-container{overflow:hidden;background:#d9d9d9;outline:0;-webkit-tap-highlight-color:transparent}
.leaflet-tile,.leaflet-marker-icon,.leaflet-marker-shadow{user-select:none;-webkit-user-drag:none}
.leaflet-container .leaflet-tile-pane img,.leaflet-container .leaflet-tile{max-width:none!important;max-height:none!important;width:auto;padding:0}
.leaflet-container.leaflet-touch-zoom{touch-action:pan-x pan-y}
.leaflet-container.leaflet-touch-drag{touch-action:none;touch-action:pinch-zoom}
.leaflet-container.leaflet-touch-drag.leaflet-touch-zoom{touch-action:none}
.leaflet-tile{filter:inherit;visibility:hidden}.leaflet-tile-loaded{visibility:inherit}
.leaflet-pane{z-index:400}.leaflet-tile-pane{z-index:200}.leaflet-overlay-pane{z-index:400}
.leaflet-shadow-pane{z-index:500}.leaflet-marker-pane{z-index:600}.leaflet-tooltip-pane{z-index:650}.leaflet-popup-pane{z-index:700}
.leaflet-map-pane canvas{z-index:100}.leaflet-map-pane svg{z-index:200}
.leaflet-control{position:relative;z-index:800;pointer-events:auto}.leaflet-top,.leaflet-bottom{position:absolute;z-index:1000;pointer-events:none}
.leaflet-top{top:0}.leaflet-right{right:0}.leaflet-bottom{bottom:0}.leaflet-left{left:0}
.leaflet-control{float:left;clear:both}.leaflet-right .leaflet-control{float:right}.leaflet-top .leaflet-control{margin-top:10px}
.leaflet-bottom .leaflet-control{margin-bottom:10px}.leaflet-left .leaflet-control{margin-left:10px}.leaflet-right .leaflet-control{margin-right:10px}
.leaflet-zoom-animated{transform-origin:0 0}.leaflet-zoom-anim .leaflet-zoom-animated{transition:transform .25s cubic-bezier(0,0,.25,1)}
.leaflet-zoom-anim .leaflet-tile,.leaflet-pan-anim .leaflet-tile{transition:none}.leaflet-zoom-anim .leaflet-zoom-hide{visibility:hidden}
.leaflet-grab{cursor:grab}.leaflet-dragging .leaflet-grab{cursor:grabbing}
.leaflet-marker-icon,.leaflet-marker-shadow,.leaflet-image-layer,.leaflet-tile-container{pointer-events:none}
.leaflet-container{font:12px/1.5 "Helvetica Neue",Arial,Helvetica,sans-serif}.leaflet-container a{color:#0078a8}
.leaflet-bar{box-shadow:0 1px 5px rgba(0,0,0,.65);border-radius:4px}.leaflet-bar a{background:#fff;border-bottom:1px solid #ccc;width:30px;height:30px;line-height:30px;display:block;text-align:center;text-decoration:none;color:#111;font:bold 20px/30px monospace}
.leaflet-bar a:hover{background:#f4f4f4}.leaflet-bar a:first-child{border-radius:4px 4px 0 0}.leaflet-bar a:last-child{border-radius:0 0 4px 4px;border-bottom:0}
.leaflet-container .leaflet-control-attribution{background:rgba(255,255,255,.85);margin:0;padding:0 5px;color:#333;line-height:1.4}.leaflet-control-attribution a{text-decoration:none}
.gnss-telemetry{position:absolute;right:12px;top:12px;z-index:1100;min-width:225px;padding:10px 12px;border:1px solid #526375;border-radius:6px;background:rgba(12,18,28,.92);color:#f4f7fb;font:14px/1.5 system-ui,sans-serif;pointer-events:none}
.gnss-telemetry-title{color:#a9c7e8;font-size:12px;font-weight:700;letter-spacing:.06em}.gnss-telemetry-separator{border-top:1px solid #526375;margin-top:5px;padding-top:5px}
.gnss-map-error{display:none;position:absolute;left:50%;bottom:34px;z-index:1100;transform:translateX(-50%);padding:7px 10px;border-radius:4px;background:rgba(137,28,28,.92);color:#fff;font:13px system-ui,sans-serif;pointer-events:none}.gnss-map-error.visible{display:block}
.waypoint-pin{position:relative;width:24px;height:24px;border:2px solid #003b4a;border-radius:50%;background:#00e5ff;color:#00303b;font:800 11px/20px system-ui,sans-serif;text-align:center;box-sizing:border-box;box-shadow:0 1px 3px rgba(0,0,0,.85)}
.waypoint-pin-leader{position:absolute;left:21px;top:10px;width:10px;border-top:2px solid #00e5ff;filter:drop-shadow(0 1px 1px rgba(0,0,0,.85))}
.waypoint-pin-label{position:absolute;left:31px;top:0;padding:2px 5px;border:1px solid #00e5ff;border-radius:3px;background:rgba(5,25,35,.92);color:#fff;font:600 11px/1.3 system-ui,sans-serif;box-shadow:0 1px 3px rgba(0,0,0,.85);white-space:nowrap}
	.vessel-icon{height:40px;width:40px;filter:drop-shadow(0 1px 2px rgba(0,0,0,.8))}.vessel-arrow{height:40px;width:40px;transform-origin:20px 20px}.vessel-body{stroke:#063946;stroke-width:1.8;stroke-linejoin:round;stroke-linecap:round;fill-rule:evenodd}
	.vessel-label{position:absolute;left:50%;top:42px;transform:translateX(-50%);padding:1px 4px;border:1px solid #00e5ff;border-radius:3px;background:rgba(5,25,35,.92);color:#fff;font:700 10px/1.35 system-ui,sans-serif;letter-spacing:.05em;white-space:nowrap}
	.vessel-dot{display:none;position:absolute;left:13px;top:13px;width:14px;height:14px;border:3px solid #063946;border-radius:50%;background:#00cceb;box-sizing:border-box}.vessel-icon.no-heading .vessel-arrow{display:none}.vessel-icon.no-heading .vessel-dot{display:block}
	`;

const BATTERY_PANEL_CSS = `
.battery-root{height:100%;min-height:150px;display:flex;align-items:center;justify-content:center;background:#111827;color:#f9fafb;font-family:system-ui,sans-serif;overflow:hidden}
.battery-card{width:min(92%,360px);display:flex;flex-direction:column;align-items:center;gap:12px}
.battery-title{font-size:13px;font-weight:700;letter-spacing:.08em;color:#cbd5e1}
.battery-shell{position:relative;width:min(82%,250px);height:82px;border:5px solid #e5edf5;border-radius:8px;background:#1f2937;box-sizing:border-box;box-shadow:0 8px 20px rgba(0,0,0,.35)}
.battery-shell::after{content:"";position:absolute;right:-17px;top:24px;width:12px;height:26px;border-radius:0 5px 5px 0;background:#e5edf5}
.battery-fill{position:absolute;left:6px;top:6px;bottom:6px;width:0%;border-radius:4px;background:#22c55e;transition:width .2s ease,background-color .2s ease}
.battery-percent{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;font-size:30px;font-weight:800;color:#ffffff;text-shadow:0 1px 3px rgba(0,0,0,.65)}
.battery-meta{font-size:13px;color:#cbd5e1}
`;

function format(value, digits, suffix) {
  return typeof value === "number" && Number.isFinite(value) ? `${value.toFixed(digits)}${suffix}` : "--";
}

function clamp(value, min, max) {
  return Math.min(max, Math.max(min, value));
}

function batteryColor(percent) {
  if (!Number.isFinite(percent)) return "#64748b";
  if (percent <= 20) return "#ef4444";
  if (percent <= 50) return "#eab308";
  return "#22c55e";
}

function normalizeFrame(frame) {
  return typeof frame === "string" ? frame.replace(/^\/+/, "") : "";
}

function normalizeQuaternion(rotation) {
  if (!rotation) return undefined;
  const values = [rotation.x, rotation.y, rotation.z, rotation.w];
  if (!values.every((value) => typeof value === "number" && Number.isFinite(value))) return undefined;
  const magnitude = Math.hypot(...values);
  if (magnitude < 1e-12) return undefined;
  return {x: rotation.x / magnitude, y: rotation.y / magnitude, z: rotation.z / magnitude, w: rotation.w / magnitude};
}

function multiplyQuaternions(a, b) {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}

function resolveOrientation(transforms, worldFrame, vesselFrame) {
  const rotations = [];
  const visited = new Set();
  let current = normalizeFrame(vesselFrame);
  const world = normalizeFrame(worldFrame);
  while (current !== world) {
    if (!current || visited.has(current)) return undefined;
    visited.add(current);
    const transform = transforms.get(current);
    if (!transform) return undefined;
    rotations.push(transform.rotation);
    current = transform.parent;
  }
  let result = {x: 0, y: 0, z: 0, w: 1};
  for (let index = rotations.length - 1; index >= 0; index -= 1) {
    result = multiplyQuaternions(result, rotations[index]);
  }
  return normalizeQuaternion(result);
}

function quaternionToBearingDegrees(rotation) {
  if (!rotation) return undefined;
  // map is ENU. Rotate base_link +X into map, then convert east/north to a compass bearing.
  const east = 1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z);
  const north = 2 * (rotation.x * rotation.y + rotation.w * rotation.z);
  if (!Number.isFinite(east) || !Number.isFinite(north) || Math.hypot(east, north) < 1e-9) return undefined;
  return (Math.atan2(east, north) * 180 / Math.PI + 360) % 360;
}

function vesselIcon() {
  // Existing catamaran silhouette: its bow is base_link +X. Ground PC code
  // rotates it from the received map -> base_link TF.
  const points = [
    [11, 3], [7, 11], [7, 30], [11, 37], [15, 30], [15, 22],
    [25, 22], [25, 30], [29, 37], [33, 30], [33, 11], [29, 3],
    [25, 11], [25, 18], [15, 18], [15, 11],
  ];
  const bodyPath = `${points.map(([x, y], index) => `${index === 0 ? "M" : "L"}${x} ${y}`).join(" ")} Z`;
  return L.divIcon({
    className: "",
    iconSize: [40, 40],
    iconAnchor: [20, 20],
    html: `<div class="vessel-icon no-heading"><svg class="vessel-arrow" viewBox="0 0 40 40" aria-label="Catamaran vessel heading"><defs><linearGradient id="vessel-hull-gradient" x1="0" y1="0" x2="0" y2="1"><stop offset="0%" stop-color="#00e5ff"/><stop offset="48%" stop-color="#008ca5"/><stop offset="100%" stop-color="#022f3a"/></linearGradient></defs><path class="vessel-body" fill="url(#vessel-hull-gradient)" d="${bodyPath}"/></svg><div class="vessel-dot"></div><span class="vessel-label">SELF</span></div>`,
  });
}

function initGnssMapTelemetry(context) {
  const root = context.panelElement;
  root.replaceChildren();
  root.className = "gnss-map-root";

  const style = document.createElement("style");
  style.textContent = PANEL_CSS;
  root.appendChild(style);

  const mapElement = document.createElement("div");
  mapElement.className = "gnss-map-canvas";
  root.appendChild(mapElement);

  const legend = document.createElement("div");
  legend.className = "gnss-telemetry";
  root.appendChild(legend);

  const mapError = document.createElement("div");
  mapError.className = "gnss-map-error";
  mapError.textContent = "OpenStreetMapタイルを読み込めません";
  root.appendChild(mapError);

  const map = L.map(mapElement, {zoomControl: true, attributionControl: true}).setView([20, 0], 2);
  const tileLayer = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  }).addTo(map);
  tileLayer.on("tileerror", () => mapError.classList.add("visible"));
  tileLayer.on("load", () => mapError.classList.remove("visible"));

  const state = {};
  const transforms = new Map();
  const waypointMarkers = new Map();
  let marker;
  let waypointRoute;
  let activeWaypointTask = "";
  let centered = false;
  let waypointSignature = "";

  function updateTransforms(message) {
    for (const stamped of message?.transforms || []) {
      const parent = normalizeFrame(stamped.header?.frame_id);
      const child = normalizeFrame(stamped.child_frame_id);
      const rotation = normalizeQuaternion(stamped.transform?.rotation);
      const translation = stamped.transform?.translation;
      if (parent && child && rotation && Number.isFinite(translation?.x) && Number.isFinite(translation?.y)) {
        transforms.set(child, {parent, rotation, translation});
      }
    }
  }

  function updateWaypointMarkers(message) {
    for (const waypoint of message?.markers || []) {
      if (waypoint.action === 3) {
        for (const marker of waypointMarkers.values()) map.removeLayer(marker);
        waypointMarkers.clear();
        if (waypointRoute) map.removeLayer(waypointRoute);
        waypointRoute = undefined;
        activeWaypointTask = "";
        continue;
      }
      if (!waypoint.ns?.startsWith("ground_waypoint_wgs84") || !Number.isFinite(waypoint.pose?.position?.x) || !Number.isFinite(waypoint.pose?.position?.y)) continue;
      const selectedTask = waypoint.ns.split("/", 2)[1];
      if (selectedTask) activeWaypointTask = selectedTask;
      // The Ground PC publisher deliberately stores longitude in x and
      // latitude in y, so this panel can render YAML waypoints without
      // receiving the vessel's map-frame MarkerArray over Zenoh.
      const position = [waypoint.pose.position.y, waypoint.pose.position.x];
      const id = String(waypoint.id);
      const order = String(Number(waypoint.id) + 1).padStart(2, "0");
      const name = waypoint.text || "WP";
      let waypointMarker = waypointMarkers.get(id);
      if (!waypointMarker) {
        waypointMarker = L.marker(position, {
          icon: L.divIcon({
            className: "",
            iconSize: [220, 24],
            iconAnchor: [12, 12],
            html: `<div class="waypoint-pin">${order}<span class="waypoint-pin-leader"></span><span class="waypoint-pin-label">${order} · ${name}</span></div>`,
          }),
          interactive: false,
        });
        waypointMarker.addTo(map);
        waypointMarkers.set(id, waypointMarker);
      } else {
        waypointMarker.setLatLng(position);
        waypointMarker.setIcon(L.divIcon({
          className: "",
          iconSize: [220, 24],
          iconAnchor: [12, 12],
          html: `<div class="waypoint-pin">${order}<span class="waypoint-pin-leader"></span><span class="waypoint-pin-label">${order} · ${name}</span></div>`,
        }));
      }
    }
    const orderedWaypoints = [...waypointMarkers.entries()]
      .sort(([left], [right]) => Number(left) - Number(right));
    if (waypointRoute) map.removeLayer(waypointRoute);
    if (orderedWaypoints.length > 1) {
      waypointRoute = L.polyline(
        orderedWaypoints.map(([, item]) => item.getLatLng()),
        {color: "#00e5ff", weight: 2, opacity: 0.8, dashArray: "6 5", interactive: false},
      ).addTo(map);
    }
    const nextWaypointSignature = [...waypointMarkers.entries()].map(([id, item]) => {
      const position = item.getLatLng();
      return `${id}:${position.lat.toFixed(8)}:${position.lng.toFixed(8)}`;
    }).join("|");
    // The source republishes DELETEALL followed by the complete identical
    // route every second. Refit only when that actual route changes.
    if (nextWaypointSignature && nextWaypointSignature !== waypointSignature) {
      // A waypoint-only view has no vessel fix; center on the route itself.
      const positions = [...waypointMarkers.values()].map((item) => item.getLatLng());
      if (marker) positions.unshift(marker.getLatLng());
      map.fitBounds(L.latLngBounds(positions), {padding: [40, 40], maxZoom: 17, animate: false});
    }
    waypointSignature = nextWaypointSignature;
  }

  function updateMarker() {
    if (!Number.isFinite(state.latitude) || !Number.isFinite(state.longitude)) return;
    const position = [state.latitude, state.longitude];
    if (!marker) marker = L.marker(position, {icon: vesselIcon(), interactive: false}).addTo(map);
    else marker.setLatLng(position);
    if (!centered) {
      map.setView(position, 16, {animate: false});
      centered = true;
    }

    const orientation = resolveOrientation(transforms, WORLD_FRAME, VESSEL_FRAME);
    state.headingDegrees = quaternionToBearingDegrees(orientation);
    const element = marker.getElement()?.querySelector(".vessel-icon");
    const arrow = element?.querySelector(".vessel-arrow");
    if (element && arrow && Number.isFinite(state.headingDegrees)) {
      element.classList.remove("no-heading");
      arrow.style.transform = `rotate(${state.headingDegrees}deg)`;
    } else if (element) {
      element.classList.add("no-heading");
    }
  }

  function renderLegend() {
    legend.innerHTML = [
      '<div class="gnss-telemetry-title">VESSEL TELEMETRY</div>',
      `<div>LAT&nbsp;&nbsp; ${format(state.latitude, 6, "°")}</div>`,
      `<div>LON&nbsp; ${format(state.longitude, 6, "°")}</div>`,
      `<div>HDG&nbsp; ${format(state.headingDegrees, 1, "°")}</div>`,
      '<div class="gnss-telemetry-separator">',
      `SOG&nbsp;&nbsp; ${format(state.speedMps, 2, " m/s")}</div>`,
      `BAT&nbsp;&nbsp; ${format(state.batteryPercent, 0, " %")}</div>`,
      '<div class="gnss-telemetry-separator">',
      `TASK&nbsp; ${activeWaypointTask || "--"}</div>`,
      `WP&nbsp;&nbsp;&nbsp; ${waypointMarkers.size}</div>`,
    ].join("");
  }

  context.subscribe([
    {topic: FIX_TOPIC}, {topic: SPEED_TOPIC}, {topic: BATTERY_PERCENT_TOPIC},
    {topic: TF_TOPIC}, {topic: TF_STATIC_TOPIC}, {topic: GROUND_WAYPOINT_MARKERS_TOPIC},
  ]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    try {
      for (const event of renderState.currentFrame || []) {
        const message = event.message || {};
        if (event.topic === FIX_TOPIC) {
          if (typeof message.latitude === "number") state.latitude = message.latitude;
          if (typeof message.longitude === "number") state.longitude = message.longitude;
        } else if (event.topic === SPEED_TOPIC && typeof message.data === "number") {
          state.speedMps = message.data;
        } else if (event.topic === BATTERY_PERCENT_TOPIC && typeof message.data === "number") {
          state.batteryPercent = message.data;
        } else if (event.topic === TF_TOPIC || event.topic === TF_STATIC_TOPIC) {
          updateTransforms(message);
        } else if (event.topic === GROUND_WAYPOINT_MARKERS_TOPIC) {
          updateWaypointMarkers(message);
        }
      }
      updateMarker();
      // MarkerArray is transient-local, but re-applying it also handles a
      // GNSS fix arriving after the ground-side waypoint list.
      for (const event of renderState.currentFrame || []) {
        if (event.topic === GROUND_WAYPOINT_MARKERS_TOPIC) updateWaypointMarkers(event.message || {});
      }
      renderLegend();
    } finally {
      done();
    }
  };

  const resizeObserver = new ResizeObserver(() => map.invalidateSize({animate: false}));
  resizeObserver.observe(root);
  renderLegend();

  return () => {
    resizeObserver.disconnect();
    map.remove();
    root.replaceChildren();
  };
}

function initBatteryStatus(context) {
  const root = context.panelElement;
  root.replaceChildren();
  root.className = "battery-root";

  const style = document.createElement("style");
  style.textContent = BATTERY_PANEL_CSS;
  root.appendChild(style);

  const card = document.createElement("div");
  card.className = "battery-card";
  card.innerHTML = [
    '<div class="battery-title">BATTERY REMAINING</div>',
    '<div class="battery-shell"><div class="battery-fill"></div><div class="battery-percent">--%</div></div>',
    '<div class="battery-meta">/gui/battery_percent</div>',
  ].join("");
  root.appendChild(card);

  const fill = card.querySelector(".battery-fill");
  const percentText = card.querySelector(".battery-percent");
  let batteryPercent;

  function renderBattery() {
    if (!Number.isFinite(batteryPercent)) {
      fill.style.width = "0%";
      fill.style.backgroundColor = batteryColor(undefined);
      percentText.textContent = "--%";
      return;
    }
    const percent = clamp(batteryPercent, 0, 100);
    fill.style.width = `${percent}%`;
    fill.style.backgroundColor = batteryColor(percent);
    percentText.textContent = `${Math.round(percent)}%`;
  }

  context.subscribe([{topic: BATTERY_PERCENT_TOPIC}]);
  context.watch("currentFrame");
  context.onRender = (renderState, done) => {
    try {
      for (const event of renderState.currentFrame || []) {
        const message = event.message || {};
        if (event.topic === BATTERY_PERCENT_TOPIC && typeof message.data === "number") {
          batteryPercent = message.data;
        }
      }
      renderBattery();
    } finally {
      done();
    }
  };

  renderBattery();
  return () => root.replaceChildren();
}

function initLidarMissionSplat(context) {
  const defaults = {map: "/gui/livox/splat_map", waypoints: "/mission/waypoint_markers", actual: "/actual_path_marker", tf: "/tf", tfStatic: "/tf_static", status: "/mission/status", fix: FIX_TOPIC, speed: SPEED_TOPIC, battery: BATTERY_PERCENT_TOPIC};
  const topics = {...defaults, ...(context.initialState?.topics || {})};
  const root = context.panelElement;
  root.replaceChildren();
  root.style.cssText = "height:100%;position:relative;overflow:hidden;background:#071018;color:#dcecff;font:12px system-ui";
  const canvas = document.createElement("canvas"); canvas.style.cssText = "width:100%;height:100%;touch-action:none"; root.appendChild(canvas);
  const card = document.createElement("div"); card.style.cssText = "position:absolute;right:12px;top:12px;background:#0c1620df;border:1px solid #526375;border-radius:6px;padding:9px;min-width:190px;line-height:1.55"; root.appendChild(card);
  const settings = document.createElement("details"); settings.style.cssText = "position:absolute;left:12px;top:12px;background:#0c1620df;padding:7px;border-radius:5px";
  settings.innerHTML = "<summary style='cursor:pointer'>⚙ Topics</summary><form></form>"; root.appendChild(settings);
  const form = settings.querySelector("form");
  for (const [name, value] of Object.entries(topics)) form.insertAdjacentHTML("beforeend", `<label style="display:block;margin-top:5px">${name}<input name="${name}" value="${value}" style="display:block;width:180px"></label>`);
  form.insertAdjacentHTML("beforeend", "<button style='margin-top:7px'>Apply</button>");
  form.onsubmit = event => { event.preventDefault(); for (const input of form.querySelectorAll("input")) topics[input.name] = input.value.trim(); context.saveState({topics}); subscribe(); settings.open = false; };
  const gl = canvas.getContext("webgl2", {antialias: true});
  if (!gl) { card.textContent = "WebGL2 is unavailable. LiDAR Mission Splat requires WebGL2."; return () => root.replaceChildren(); }
  const vertex = `#version 300 es
  in vec3 p; in float i; uniform mat4 m; uniform float size; out float v; void main(){gl_Position=m*vec4(p,1.);gl_PointSize=size;v=i;}`;
  const fragment = `#version 300 es
  precision mediump float; in float v; out vec4 c; void main(){vec2 q=gl_PointCoord*2.-1.;float a=exp(-dot(q,q)*3.);c=vec4(clamp(v/255.,0.,1.),.85,1.-clamp(v/255.,0.,1.),a);}`;
  function shader(type, source) { const s = gl.createShader(type); gl.shaderSource(s, source); gl.compileShader(s); return s; }
  const program = gl.createProgram(); gl.attachShader(program, shader(gl.VERTEX_SHADER, vertex)); gl.attachShader(program, shader(gl.FRAGMENT_SHADER, fragment)); gl.linkProgram(program);
  const buffer = gl.createBuffer(); const position = gl.getAttribLocation(program, "p"), intensity = gl.getAttribLocation(program, "i");
  let points = new Float32Array(), lastCloud = 0, resolution = .75, yaw = .6, pitch = .9, zoom = 1, panX = 0, panY = 0, dragging, telemetry = {}, path = [], waypoints = [], vessel;
  function decode(message) {
    const fields = Object.fromEntries((message.fields || []).map(f => [f.name, f.offset])); const data = message.data;
    if (!(data && fields.x !== undefined && fields.y !== undefined && fields.z !== undefined && message.point_step >= 12)) return undefined;
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data.buffer || data); const out = new Float32Array(Math.floor(bytes.byteLength / message.point_step) * 4); const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength); let n = 0;
    for (let o = 0; o + message.point_step <= bytes.byteLength; o += message.point_step) { const x=view.getFloat32(o+fields.x,true), y=view.getFloat32(o+fields.y,true), z=view.getFloat32(o+fields.z,true), i=fields.intensity===undefined?0:view.getFloat32(o+fields.intensity,true); if ([x,y,z,i].every(Number.isFinite)) out.set([x,y,z,i],n+=4); }
    return out.slice(0,n);
  }
  function markerPoints(message) { return (message?.markers || []).flatMap(m => (m.points || []).map(p => [p.x,p.y,p.z || 0])); }
  function matrix() { const c=Math.cos(yaw),s=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch), z=zoom/80; return new Float32Array([c*z,s*sp*z,s*cp*z,0,-s*z,c*sp*z,c*cp*z,0,0,-cp*z,sp*z,0,panX,panY,0,1]); }
  function draw() { const rect=canvas.getBoundingClientRect(), w=Math.max(1,rect.width*devicePixelRatio),h=Math.max(1,rect.height*devicePixelRatio); if(canvas.width!==w||canvas.height!==h){canvas.width=w;canvas.height=h;} gl.viewport(0,0,w,h);gl.clearColor(.027,.063,.094,1);gl.clear(gl.COLOR_BUFFER_BIT); if(points.length && Date.now()-lastCloud <= 3000){const overlay=[...path,...waypoints];if(vessel)overlay.push([vessel.translation.x,vessel.translation.y,vessel.translation.z||0]);const drawPoints=new Float32Array(points.length+overlay.length*4);drawPoints.set(points);overlay.forEach((p,n)=>drawPoints.set([p[0],p[1],p[2],255],points.length+n*4));gl.useProgram(program);gl.bindBuffer(gl.ARRAY_BUFFER,buffer);gl.bufferData(gl.ARRAY_BUFFER,drawPoints,gl.DYNAMIC_DRAW);gl.enableVertexAttribArray(position);gl.vertexAttribPointer(position,3,gl.FLOAT,false,16,0);gl.enableVertexAttribArray(intensity);gl.vertexAttribPointer(intensity,1,gl.FLOAT,false,16,12);gl.uniformMatrix4fv(gl.getUniformLocation(program,"m"),false,matrix());gl.uniform1f(gl.getUniformLocation(program,"size"),Math.max(2,7*zoom));gl.enable(gl.BLEND);gl.blendFunc(gl.SRC_ALPHA,gl.ONE);gl.drawArrays(gl.POINTS,0,drawPoints.length/4);} requestAnimationFrame(draw); }
  function subscribe() { context.subscribe(Object.values(topics).filter(Boolean).map(topic => ({topic}))); }
  context.watch("currentFrame"); subscribe(); draw();
  context.onRender = (state, done) => { try { for(const event of state.currentFrame || []) { const m=event.message || {}; if(event.topic===topics.map){const decoded=decode(m);if(decoded){points=decoded;lastCloud=Date.now();}} else if(event.topic===topics.actual) path=markerPoints(m); else if(event.topic===topics.waypoints) waypoints=markerPoints(m); else if(event.topic===topics.fix) {telemetry.lat=m.latitude; telemetry.lon=m.longitude;} else if(event.topic===topics.speed) telemetry.speed=m.data; else if(event.topic===topics.battery) telemetry.battery=m.data; else if(event.topic===topics.status) telemetry.task=m.task_id; else if(event.topic===topics.tf || event.topic===topics.tfStatic) for(const t of m.transforms||[]) if(normalizeFrame(t.child_frame_id)==="base_link"&&normalizeFrame(t.header?.frame_id)==="odom") vessel=t.transform; } const age=(Date.now()-lastCloud)/1000, lidar=lastCloud ? (age > 3 ? `STALE (${age.toFixed(1)}s)` : `${age.toFixed(1)}s ago`) : "waiting"; card.innerHTML=`<b>LiDAR MISSION SPLAT</b><br>PTS ${points.length/4} · VOX ${resolution.toFixed(2)} m<br>LiDAR ${lidar}<br>TF ${vessel?"odom ✓":"waiting"}<br>LAT ${format(telemetry.lat,6,"")}<br>LON ${format(telemetry.lon,6,"")}<br>HDG ${vessel?quaternionToBearingDegrees(vessel.rotation)?.toFixed(1)+"°":"--"}<br>SOG ${format(telemetry.speed,2," m/s")}<br>BAT ${format(telemetry.battery,0,"%")}<br>TASK ${telemetry.task||"--"}`; } finally {done();} };
  canvas.onpointerdown=e=>{dragging=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)}; canvas.onpointermove=e=>{if(!dragging)return; yaw+=(e.clientX-dragging[0])*.01; pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-dragging[1])*.01));dragging=[e.clientX,e.clientY]}; canvas.onpointerup=()=>dragging=undefined; canvas.onwheel=e=>{zoom=Math.max(.2,Math.min(8,zoom*(e.deltaY>0?.9:1.1)));e.preventDefault()};
  return () => root.replaceChildren();
}

function activate(extensionContext) {
  extensionContext.registerPanel({name: "gnss-map-telemetry", initPanel: initGnssMapTelemetry});
  extensionContext.registerPanel({name: "battery-status", initPanel: initBatteryStatus});
  extensionContext.registerPanel({name: "lidar-mission-splat", initPanel: initLidarMissionSplat});
}

module.exports = {
  activate,
  multiplyQuaternions,
  normalizeQuaternion,
  quaternionToBearingDegrees,
  resolveOrientation,
};
