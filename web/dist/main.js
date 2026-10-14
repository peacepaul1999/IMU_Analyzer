"use strict";
(() => {
  // ts/ui/logger.ts
  function element(id) {
    const node = document.getElementById(id);
    if (!node) throw new Error(`Missing element: ${id}`);
    return node;
  }
  function setConnectionStatus(connected) {
    const connection = element("connection");
    connection.textContent = connected ? "Connected" : "Disconnected";
    connection.classList.toggle("online", connected);
  }
  async function loadFirmwareVersion() {
    const version = element("firmwareVersion");
    try {
      const response = await fetch("/version", { cache: "no-store" });
      if (!response.ok) {
        throw new Error(`Version request failed with status ${response.status}`);
      }
      const firmwareVersion = (await response.text()).trim();
      if (!firmwareVersion) {
        throw new Error("Version request returned an empty response");
      }
      version.textContent = `Firmware ${firmwareVersion}`;
    } catch (error) {
      version.textContent = "Firmware unavailable";
      console.error("Failed to load firmware version:", error);
    }
  }

  // ts/imu/calibration.ts
  function updateCalibration(sample) {
    element("calState").textContent = sample.calibrating ? "Calibrating" : "Idle";
    element("northReady").textContent = sample.northReady ? "Yes" : "No";
  }

  // ts/utils/units.ts
  var RAD_TO_DEG = 180 / Math.PI;
  var UNITS = {
    acc: { unit: "g", factor: 1 },
    gyro: { unit: "\xB0/s", factor: RAD_TO_DEG },
    mag: { unit: "\xB5T", factor: 1 }
  };
  function toDisplayUnits(axis, values) {
    const { factor } = UNITS[axis];
    return [values[0] * factor, values[1] * factor, values[2] * factor];
  }

  // ts/imu/csvLogger.ts
  var MAX_ROWS = 5e4;
  var HEADER = [
    "device_ms",
    "pc_time",
    "yaw_deg",
    "roll_deg",
    "pitch_deg",
    "acc_x_g",
    "acc_y_g",
    "acc_z_g",
    "gyro_x_dps",
    "gyro_y_dps",
    "gyro_z_dps",
    "mag_x_uT",
    "mag_y_uT",
    "mag_z_uT"
  ].join(",");
  var rows = [];
  var recording = false;
  function recordCsvSample(sample) {
    if (!recording) return;
    if (rows.length >= MAX_ROWS) {
      setRecording(false);
      return;
    }
    const gyro = sample.gyro.map((value) => value * UNITS.gyro.factor);
    const fields = [
      sample.t,
      (/* @__PURE__ */ new Date()).toISOString(),
      ...[sample.yaw, sample.roll, sample.pitch].map((value) => value.toFixed(3)),
      ...sample.acc.map((value) => value.toFixed(4)),
      ...gyro.map((value) => value.toFixed(3)),
      ...sample.mag.map((value) => value.toFixed(3))
    ];
    rows.push(fields.join(","));
    updateStatus();
  }
  function setRecording(next) {
    recording = next;
    element("logToggle").textContent = recording ? "Stop Recording" : "Start Recording";
    updateStatus();
  }
  function updateStatus() {
    element("logStatus").textContent = `${recording ? "Recording" : "Stopped"} \xB7 ${rows.length} rows`;
  }
  function saveCsv() {
    if (rows.length === 0) return;
    const blob = new Blob([`${HEADER}
${rows.join("\n")}
`], { type: "text/csv" });
    const link = document.createElement("a");
    const stamp = (/* @__PURE__ */ new Date()).toISOString().replace(/[:.]/g, "-");
    link.href = URL.createObjectURL(blob);
    link.download = `imu_log_${stamp}.csv`;
    link.click();
    URL.revokeObjectURL(link.href);
  }
  function bindCsvButtons() {
    element("logToggle").addEventListener("click", () => setRecording(!recording));
    element("logSave").addEventListener("click", saveCsv);
    element("logClear").addEventListener("click", () => {
      rows.length = 0;
      updateStatus();
    });
    updateStatus();
  }

  // ts/imu/dataRate.ts
  var WINDOW = 20;
  var arrivals = [];
  function updateDataRates(sample) {
    arrivals.push(performance.now());
    if (arrivals.length > WINDOW) arrivals.shift();
    let measured = "\u2014";
    if (arrivals.length >= 2) {
      const elapsed = (arrivals[arrivals.length - 1] - arrivals[0]) / 1e3;
      if (elapsed > 0) measured = ((arrivals.length - 1) / elapsed).toFixed(1);
    }
    element("rates").textContent = `IMU: ${sample.imuHz} Hz \xB7 WebSocket: ${sample.wsHz} Hz (measured ${measured} Hz)`;
  }
  function clearDataRates() {
    arrivals.length = 0;
    element("rates").textContent = "IMU: \u2014 \xB7 WebSocket: \u2014";
  }

  // ts/imu/imuState.ts
  function updateImuState(sample) {
    for (const key of ["yaw", "roll", "pitch"]) {
      element(key).textContent = `${sample[key].toFixed(2)}\xB0`;
    }
  }

  // ts/utils/math.ts
  function maxAbsolute(values) {
    return Math.max(1e-5, ...values.map((value) => Math.abs(value)));
  }
  function isFiniteTriplet(value) {
    return Array.isArray(value) && value.length >= 3 && value.slice(0, 3).every((entry) => typeof entry === "number" && Number.isFinite(entry));
  }

  // ts/imu/parser.ts
  function parseSample(message) {
    try {
      const value = JSON.parse(message);
      if (!isSample(value)) return null;
      return value;
    } catch {
      return null;
    }
  }
  function isSample(value) {
    if (!value || typeof value !== "object") return false;
    const sample = value;
    return typeof sample.yaw === "number" && typeof sample.roll === "number" && typeof sample.pitch === "number" && isFiniteTriplet(sample.acc) && isFiniteTriplet(sample.gyro) && isFiniteTriplet(sample.mag) && typeof sample.ready === "boolean" && typeof sample.calibrating === "boolean" && typeof sample.northReady === "boolean" && typeof sample.t === "number" && typeof sample.imuHz === "number" && typeof sample.wsHz === "number";
  }

  // ts/charts/chartState.ts
  var HISTORY_LENGTH = 150;
  var histories = {
    acc: [[], [], []],
    gyro: [[], [], []],
    mag: [[], [], []]
  };
  function recordValues(axis, values) {
    values.forEach((value, index) => {
      const history = histories[axis][index];
      history.push(value);
      if (history.length > HISTORY_LENGTH) history.shift();
    });
  }

  // ts/charts/sensorCharts.ts
  var axes = ["acc", "gyro", "mag"];
  var lineColors = ["#ff3c00", "#0011ff", "#33ff00"];
  function updateSensorCharts(sample) {
    for (const axis of axes) {
      recordValues(axis, toDisplayUnits(axis, sample[axis]));
      drawSensorChart(axis);
    }
  }
  function resizeSensorCharts() {
    axes.forEach(drawSensorChart);
  }
  function drawSensorChart(axis) {
    const canvas = element(`${axis}Chart`);
    const rectangle = canvas.getBoundingClientRect();
    const devicePixelRatio = window.devicePixelRatio || 1;
    const width = Math.max(1, Math.floor(rectangle.width * devicePixelRatio));
    const height = Math.max(1, Math.floor(rectangle.height * devicePixelRatio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    const context = canvas.getContext("2d");
    if (!context) return;
    context.clearRect(0, 0, width, height);
    drawGrid(context, width, height, devicePixelRatio);
    const values = histories[axis];
    const range = maxAbsolute(values.flat()) * 1.15;
    values.forEach((history, index) => {
      context.beginPath();
      context.strokeStyle = lineColors[index];
      context.lineWidth = 1.8 * devicePixelRatio;
      history.forEach((value, position) => {
        const x = position / (HISTORY_LENGTH - 1) * width;
        const y = height / 2 - value / range * height / 2;
        if (position === 0) context.moveTo(x, y);
        else context.lineTo(x, y);
      });
      context.stroke();
    });
    drawScaleLabels(context, width, height, devicePixelRatio, range, UNITS[axis].unit);
  }
  function formatTick(value) {
    const magnitude = Math.abs(value);
    return value.toFixed(magnitude >= 100 ? 0 : magnitude >= 10 ? 1 : 2);
  }
  function drawScaleLabels(context, width, height, devicePixelRatio, range, unit) {
    context.fillStyle = "#a8a8a8";
    context.font = `${11 * devicePixelRatio}px Segoe UI, Arial, sans-serif`;
    context.textAlign = "left";
    const pad = 4 * devicePixelRatio;
    context.textBaseline = "top";
    context.fillText(`${formatTick(range)} ${unit}`, pad, pad);
    context.textBaseline = "middle";
    context.fillText(`0 ${unit}`, pad, height / 2);
    context.textBaseline = "bottom";
    context.fillText(`${formatTick(-range)} ${unit}`, pad, height - pad);
  }
  function drawGrid(context, width, height, devicePixelRatio) {
    context.strokeStyle = "#ffffff20";
    context.lineWidth = devicePixelRatio;
    for (let index = 1; index < 4; index += 1) {
      const y = height * index / 4;
      context.beginPath();
      context.moveTo(0, y);
      context.lineTo(width, y);
      context.stroke();
    }
  }

  // ts/charts/view3d.ts
  var MAG_POINTS = 600;
  var magPoints = [];
  var orientView = { yaw: 0.6, pitch: 0.5 };
  var magView = { yaw: 0.6, pitch: 0.5 };
  var latest = null;
  var AXIS_COLORS = ["#ff3c00", "#0011ff", "#33ff00"];
  var CUBE_VERTICES = [
    [-1, -0.6, -0.15],
    [1, -0.6, -0.15],
    [1, 0.6, -0.15],
    [-1, 0.6, -0.15],
    [-1, -0.6, 0.15],
    [1, -0.6, 0.15],
    [1, 0.6, 0.15],
    [-1, 0.6, 0.15]
  ];
  var CUBE_EDGES = [
    [0, 1],
    [1, 2],
    [2, 3],
    [3, 0],
    [4, 5],
    [5, 6],
    [6, 7],
    [7, 4],
    [0, 4],
    [1, 5],
    [2, 6],
    [3, 7]
  ];
  function rotateX(v, a) {
    const c = Math.cos(a);
    const s = Math.sin(a);
    return [v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c];
  }
  function rotateY(v, a) {
    const c = Math.cos(a);
    const s = Math.sin(a);
    return [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c];
  }
  function rotateZ(v, a) {
    const c = Math.cos(a);
    const s = Math.sin(a);
    return [v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]];
  }
  function project(v, view, scale, width, height) {
    const camera = rotateX(rotateZ(v, view.yaw), view.pitch - Math.PI / 2);
    const depth = 1 / (1 - camera[1] * 0.12);
    return {
      x: width / 2 + camera[0] * scale * depth,
      y: height / 2 - camera[2] * scale * depth,
      depth: camera[1]
    };
  }
  function prepareCanvas(id) {
    const canvas = element(id);
    const ratio = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(1, Math.floor(rect.width * ratio));
    const height = Math.max(1, Math.floor(rect.height * ratio));
    if (canvas.width !== width || canvas.height !== height) {
      canvas.width = width;
      canvas.height = height;
    }
    const context = canvas.getContext("2d");
    if (!context) return null;
    context.clearRect(0, 0, width, height);
    return { context, width, height, ratio };
  }
  function drawAxes(context, view, length, scale, width, height, ratio, labels) {
    const origin = project([0, 0, 0], view, scale, width, height);
    const tips = [[length, 0, 0], [0, length, 0], [0, 0, length]];
    context.font = `${12 * ratio}px Segoe UI, Arial, sans-serif`;
    tips.forEach((tip, index) => {
      const end = project(tip, view, scale, width, height);
      context.strokeStyle = AXIS_COLORS[index];
      context.fillStyle = AXIS_COLORS[index];
      context.lineWidth = 2 * ratio;
      context.beginPath();
      context.moveTo(origin.x, origin.y);
      context.lineTo(end.x, end.y);
      context.stroke();
      context.fillText(labels[index], end.x + 4 * ratio, end.y);
    });
  }
  function drawOrientation() {
    const surface = prepareCanvas("orientCanvas");
    if (!surface) return;
    const { context, width, height, ratio } = surface;
    const scale = Math.min(width, height) * 0.26;
    const roll = (latest?.roll ?? 0) * Math.PI / 180;
    const pitch = (latest?.pitch ?? 0) * Math.PI / 180;
    const heading = -(latest?.yaw ?? 0) * Math.PI / 180;
    const body = (v) => rotateZ(rotateY(rotateX(v, roll), pitch), heading);
    const points = CUBE_VERTICES.map((vertex) => project(body(vertex), orientView, scale, width, height));
    context.strokeStyle = "#e0e0e0";
    context.lineWidth = 1.6 * ratio;
    for (const [from, to] of CUBE_EDGES) {
      context.beginPath();
      context.moveTo(points[from].x, points[from].y);
      context.lineTo(points[to].x, points[to].y);
      context.stroke();
    }
    const origin = project([0, 0, 0], orientView, scale, width, height);
    const bodyAxes = [[1.5, 0, 0], [0, 1.5, 0], [0, 0, 1.5]];
    const names = ["X", "Y", "Z"];
    context.font = `${12 * ratio}px Segoe UI, Arial, sans-serif`;
    bodyAxes.forEach((axis, index) => {
      const end = project(body(axis), orientView, scale, width, height);
      context.strokeStyle = AXIS_COLORS[index];
      context.fillStyle = AXIS_COLORS[index];
      context.lineWidth = 2.5 * ratio;
      context.beginPath();
      context.moveTo(origin.x, origin.y);
      context.lineTo(end.x, end.y);
      context.stroke();
      context.fillText(names[index], end.x + 4 * ratio, end.y);
    });
  }
  function drawMagCloud() {
    const surface = prepareCanvas("mag3dCanvas");
    if (!surface) return;
    const { context, width, height, ratio } = surface;
    const radius = Math.max(1e-3, ...magPoints.map((p) => Math.hypot(p[0], p[1], p[2])));
    const scale = Math.min(width, height) * 0.42 / radius;
    drawAxes(context, magView, radius, scale, width, height, ratio, ["X \xB5T", "Y \xB5T", "Z \xB5T"]);
    magPoints.forEach((point, index) => {
      const p = project(point, magView, scale, width, height);
      const alpha = 0.25 + 0.75 * (index / magPoints.length);
      context.fillStyle = `rgba(255, 255, 255, ${alpha.toFixed(2)})`;
      context.beginPath();
      context.arc(p.x, p.y, 2 * ratio, 0, Math.PI * 2);
      context.fill();
    });
    context.fillStyle = "#a8a8a8";
    context.font = `${11 * ratio}px Segoe UI, Arial, sans-serif`;
    context.textBaseline = "top";
    context.fillText(`max |B| ${radius.toFixed(1)} \xB5T`, 4 * ratio, 4 * ratio);
  }
  function updateView3d(sample) {
    latest = sample;
    const mag = sample.mag;
    magPoints.push([mag[0], mag[1], mag[2]]);
    if (magPoints.length > MAG_POINTS) magPoints.shift();
    redraw3d();
  }
  function redraw3d() {
    drawOrientation();
    drawMagCloud();
  }
  function enableDragRotate(id, view) {
    const canvas = element(id);
    let dragging = false;
    let lastX = 0;
    let lastY = 0;
    canvas.addEventListener("pointerdown", (event) => {
      dragging = true;
      lastX = event.clientX;
      lastY = event.clientY;
      canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener("pointerup", () => {
      dragging = false;
    });
    canvas.addEventListener("pointermove", (event) => {
      if (!dragging) return;
      view.yaw += (event.clientX - lastX) * 0.01;
      view.pitch = Math.min(Math.PI, Math.max(0, view.pitch + (event.clientY - lastY) * 0.01));
      lastX = event.clientX;
      lastY = event.clientY;
      redraw3d();
    });
  }
  function bindView3d() {
    enableDragRotate("orientCanvas", orientView);
    enableDragRotate("mag3dCanvas", magView);
    element("mag3dClear").addEventListener("click", () => {
      magPoints.length = 0;
      redraw3d();
    });
    redraw3d();
  }

  // ts/ui/buttons.ts
  function bindCalibrationButtons(send) {
    const buttons = [
      ["calStart", "CAL_START"],
      ["calStop", "CAL_STOP"],
      ["setNorth", "SET_NORTH"]
    ];
    for (const [id, command] of buttons) {
      element(id).addEventListener("click", () => send(command));
    }
  }

  // ts/websocket/websocket.ts
  var ImuWebSocket = class {
    constructor(handlers) {
      this.handlers = handlers;
      this.socket = null;
    }
    connect() {
      this.socket = new WebSocket(`ws://${location.host}/ws`);
      this.socket.onopen = this.handlers.onOpen;
      this.socket.onclose = () => {
        this.handlers.onClose();
        window.setTimeout(() => this.connect(), 1800);
      };
      this.socket.onerror = () => this.socket?.close();
      this.socket.onmessage = (event) => this.handlers.onMessage(event.data);
    }
    send(command) {
      if (this.socket?.readyState === WebSocket.OPEN) this.socket.send(command);
    }
  };

  // ts/main.ts
  var socket = new ImuWebSocket({
    onOpen: () => setConnectionStatus(true),
    onClose: () => {
      setConnectionStatus(false);
      clearDataRates();
    },
    onMessage: (message) => {
      const sample = parseSample(message);
      if (!sample) return;
      updateImuState(sample);
      updateSensorCharts(sample);
      updateView3d(sample);
      updateDataRates(sample);
      updateCalibration(sample);
      recordCsvSample(sample);
    }
  });
  bindCalibrationButtons((command) => socket.send(command));
  bindCsvButtons();
  bindView3d();
  void loadFirmwareVersion();
  window.addEventListener("resize", () => {
    resizeSensorCharts();
    redraw3d();
  });
  socket.connect();
})();
