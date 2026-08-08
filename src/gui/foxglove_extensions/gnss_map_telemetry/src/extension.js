const FIX_TOPIC = "/sensor/vehicle_gnss/fix/raw";
const SPEED_TOPIC = "/gui/ground_speed_mps";
const BATTERY_PERCENT_TOPIC = "/gui/battery_percent";
const TF_TOPIC = "/tf";
const TF_STATIC_TOPIC = "/tf_static";
const WORLD_FRAME = "map";
const VESSEL_FRAME = "base_link";
const WAYPOINT_TOPICS = Object.freeze({
  "/visualization/norway_waypoints/start": {label: "Start", color: "#ffffff"},
  ...Object.fromEntries(Array.from({length: 10}, (_, index) => {
    const id = index + 1;
    return [`/visualization/norway_waypoints/task1/wp_1_${id}`, {label: `T1 1.${id}`, color: "#ffd54f"}];
  })),
  "/visualization/norway_waypoints/task1/gps1": {label: "T1 GPS 1", color: "#ffd54f"},
  "/visualization/norway_waypoints/task1/gps2": {label: "T1 GPS 2", color: "#ffd54f"},
  "/visualization/norway_waypoints/task1/gps3": {label: "T1 GPS 3", color: "#ffd54f"},
  "/visualization/norway_waypoints/task1/wp_3_1_pass_south": {label: "T1 3.1 S", color: "#ffd54f"},
  "/visualization/norway_waypoints/task1/wp_3_2_pass_north": {label: "T1 3.2 N", color: "#ffd54f"},
  "/visualization/norway_waypoints/task1/wp_3_3_pass_south": {label: "T1 3.3 S", color: "#ffd54f"},
  "/visualization/norway_waypoints/task1/gps4": {label: "T1 GPS 4", color: "#ffd54f"},
  "/visualization/norway_waypoints/task2/gps5": {label: "T2 GPS 5", color: "#ff5252"},
  "/visualization/norway_waypoints/task2/gate1_red": {label: "T2 gate 1 red", color: "#ff5252"},
  "/visualization/norway_waypoints/task2/gate1_center": {label: "T2 gate 1", color: "#ff5252"},
  "/visualization/norway_waypoints/task2/gate1_green": {label: "T2 gate 1 green", color: "#ff5252"},
  "/visualization/norway_waypoints/task2/gate2_red": {label: "T2 gate 2 red", color: "#ff5252"},
  "/visualization/norway_waypoints/task2/gate2_center": {label: "T2 gate 2", color: "#ff5252"},
  "/visualization/norway_waypoints/task2/gate2_green": {label: "T2 gate 2 green", color: "#ff5252"},
  "/visualization/norway_waypoints/task2/gps6": {label: "T2 GPS 6", color: "#ff5252"},
  "/visualization/norway_waypoints/task3_1/gps7": {label: "T3.1 GPS 7", color: "#4fc3f7"},
  "/visualization/norway_waypoints/task3_1/corridor_gate": {label: "T3.1 corridor", color: "#4fc3f7"},
  "/visualization/norway_waypoints/task3_1/gps8_gate": {label: "T3.1 GPS 8", color: "#4fc3f7"},
  "/visualization/norway_waypoints/task3_1/berth_approach": {label: "T3.1 berth approach", color: "#4fc3f7"},
  "/visualization/norway_waypoints/task3_1/berth": {label: "T3.1 berth", color: "#4fc3f7"},
  "/visualization/norway_waypoints/task3_1/undock_exit": {label: "T3.1 undock", color: "#4fc3f7"},
  "/visualization/norway_waypoints/task3_2/gps9": {label: "T3.2 GPS 9", color: "#ab47bc"},
  "/visualization/norway_waypoints/task3_2/corridor_gate": {label: "T3.2 corridor", color: "#ab47bc"},
  "/visualization/norway_waypoints/task3_2/gps9_gate": {label: "T3.2 gate", color: "#ab47bc"},
  "/visualization/norway_waypoints/task3_2/berth_approach": {label: "T3.2 berth approach", color: "#ab47bc"},
  "/visualization/norway_waypoints/task3_2/berth": {label: "T3.2 berth", color: "#ab47bc"},
  "/visualization/norway_waypoints/task3_2/gps10_finish": {label: "T3.2 GPS 10", color: "#ab47bc"},
});

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
	.vessel-icon{height:40px;width:40px;filter:drop-shadow(0 1px 2px rgba(0,0,0,.8))}.vessel-arrow{height:40px;width:40px;transform-origin:20px 20px}.vessel-body{fill:#00cceb;stroke:#063946;stroke-width:1.8;stroke-linejoin:round;stroke-linecap:round;fill-rule:evenodd}
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
    html: `<div class="vessel-icon no-heading"><svg class="vessel-arrow" viewBox="0 0 40 40" aria-label="Catamaran vessel heading"><path class="vessel-body" d="${bodyPath}"/></svg><div class="vessel-dot"></div></div>`,
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
  let centered = false;
  let waypointsCentered = false;
  let waypointFitTimer;

  function updateWaypoint(topic, message) {
    if (!Number.isFinite(message.latitude) || !Number.isFinite(message.longitude)) return;
    const waypoint = WAYPOINT_TOPICS[topic];
    const position = [message.latitude, message.longitude];
    let waypointMarker = waypointMarkers.get(topic);
    if (!waypointMarker) {
      waypointMarker = L.circleMarker(position, {
        radius: 7, color: waypoint.color, fillColor: waypoint.color, fillOpacity: 0.9, weight: 2,
      }).bindTooltip(waypoint.label, {direction: "top", offset: [0, -7]});
      waypointMarker.addTo(map);
      waypointMarkers.set(topic, waypointMarker);
      scheduleWaypointFit();
    } else {
      waypointMarker.setLatLng(position);
    }
  }

  function scheduleWaypointFit() {
    if (waypointsCentered || waypointFitTimer) return;
    waypointFitTimer = setTimeout(() => {
      waypointFitTimer = undefined;
      const positions = [...waypointMarkers.values()].map((item) => item.getLatLng());
      if (positions.length > 1) map.fitBounds(L.latLngBounds(positions), {padding: [40, 40], maxZoom: 17, animate: false});
      else if (positions.length === 1) map.setView(positions[0], 16, {animate: false});
      waypointsCentered = positions.length > 0;
    }, 200);
  }

  function updateTransforms(message) {
    for (const stamped of message?.transforms || []) {
      const parent = normalizeFrame(stamped.header?.frame_id);
      const child = normalizeFrame(stamped.child_frame_id);
      const rotation = normalizeQuaternion(stamped.transform?.rotation);
      if (parent && child && rotation) transforms.set(child, {parent, rotation});
    }
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
      `WP&nbsp;&nbsp;&nbsp; ${waypointMarkers.size} / ${Object.keys(WAYPOINT_TOPICS).length}</div>`,
    ].join("");
  }

  context.subscribe([
    {topic: FIX_TOPIC}, {topic: SPEED_TOPIC}, {topic: BATTERY_PERCENT_TOPIC},
    {topic: TF_TOPIC}, {topic: TF_STATIC_TOPIC},
    ...Object.keys(WAYPOINT_TOPICS).map((topic) => ({topic})),
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
        } else if (WAYPOINT_TOPICS[event.topic]) {
          updateWaypoint(event.topic, message);
        }
      }
      updateMarker();
      renderLegend();
    } finally {
      done();
    }
  };

  const resizeObserver = new ResizeObserver(() => map.invalidateSize({animate: false}));
  resizeObserver.observe(root);
  renderLegend();

  return () => {
    if (waypointFitTimer) clearTimeout(waypointFitTimer);
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

function activate(extensionContext) {
  extensionContext.registerPanel({name: "gnss-map-telemetry", initPanel: initGnssMapTelemetry});
  extensionContext.registerPanel({name: "battery-status", initPanel: initBatteryStatus});
}

module.exports = {
  activate,
  multiplyQuaternions,
  normalizeQuaternion,
  quaternionToBearingDegrees,
  resolveOrientation,
};
