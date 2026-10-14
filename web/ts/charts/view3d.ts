import type { ImuSample, SensorValues } from "../types";
import { element } from "../ui/logger";

type Vec3 = [number, number, number];

type View = { yaw: number; pitch: number };

const MAG_POINTS = 600;
const magPoints: Vec3[] = [];

const orientView: View = { yaw: 0.6, pitch: 0.5 };
const magView: View = { yaw: 0.6, pitch: 0.5 };

let latest: ImuSample | null = null;

const AXIS_COLORS = ["#ff3c00", "#0011ff", "#33ff00"];

const CUBE_VERTICES: Vec3[] = [
  [-1, -0.6, -0.15],
  [1, -0.6, -0.15],
  [1, 0.6, -0.15],
  [-1, 0.6, -0.15],
  [-1, -0.6, 0.15],
  [1, -0.6, 0.15],
  [1, 0.6, 0.15],
  [-1, 0.6, 0.15],
];

const CUBE_EDGES: ReadonlyArray<[number, number]> = [
  [0, 1], [1, 2], [2, 3], [3, 0],
  [4, 5], [5, 6], [6, 7], [7, 4],
  [0, 4], [1, 5], [2, 6], [3, 7],
];

function rotateX(v: Vec3, a: number): Vec3 {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [v[0], v[1] * c - v[2] * s, v[1] * s + v[2] * c];
}

function rotateY(v: Vec3, a: number): Vec3 {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c];
}

function rotateZ(v: Vec3, a: number): Vec3 {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]];
}

/** Rotates the world (Z up) into the camera frame and projects to 2D. */
function project(v: Vec3, view: View, scale: number, width: number, height: number) {
  const camera = rotateX(rotateZ(v, view.yaw), view.pitch - Math.PI / 2);
  const depth = 1 / (1 - camera[1] * 0.12);
  return {
    x: width / 2 + camera[0] * scale * depth,
    y: height / 2 - camera[2] * scale * depth,
    depth: camera[1],
  };
}

function prepareCanvas(id: string) {
  const canvas = element(id) as HTMLCanvasElement;
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

function drawAxes(
  context: CanvasRenderingContext2D,
  view: View,
  length: number,
  scale: number,
  width: number,
  height: number,
  ratio: number,
  labels: readonly string[],
): void {
  const origin = project([0, 0, 0], view, scale, width, height);
  const tips: Vec3[] = [[length, 0, 0], [0, length, 0], [0, 0, length]];
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

function drawOrientation(): void {
  const surface = prepareCanvas("orientCanvas");
  if (!surface) return;
  const { context, width, height, ratio } = surface;
  const scale = Math.min(width, height) * 0.26;

  const roll = ((latest?.roll ?? 0) * Math.PI) / 180;
  const pitch = ((latest?.pitch ?? 0) * Math.PI) / 180;
  // Yaw is a compass heading (clockwise), so it is negated for a Z-up frame.
  const heading = (-(latest?.yaw ?? 0) * Math.PI) / 180;

  const body = (v: Vec3): Vec3 => rotateZ(rotateY(rotateX(v, roll), pitch), heading);
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
  const bodyAxes: Vec3[] = [[1.5, 0, 0], [0, 1.5, 0], [0, 0, 1.5]];
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

function drawMagCloud(): void {
  const surface = prepareCanvas("mag3dCanvas");
  if (!surface) return;
  const { context, width, height, ratio } = surface;

  const radius = Math.max(1e-3, ...magPoints.map((p) => Math.hypot(p[0], p[1], p[2])));
  const scale = (Math.min(width, height) * 0.42) / radius;

  drawAxes(context, magView, radius, scale, width, height, ratio, ["X µT", "Y µT", "Z µT"]);

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
  context.fillText(`max |B| ${radius.toFixed(1)} µT`, 4 * ratio, 4 * ratio);
}

export function updateView3d(sample: ImuSample): void {
  latest = sample;
  const mag: SensorValues = sample.mag;
  magPoints.push([mag[0], mag[1], mag[2]]);
  if (magPoints.length > MAG_POINTS) magPoints.shift();
  redraw3d();
}

export function redraw3d(): void {
  drawOrientation();
  drawMagCloud();
}

function enableDragRotate(id: string, view: View): void {
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

export function bindView3d(): void {
  enableDragRotate("orientCanvas", orientView);
  enableDragRotate("mag3dCanvas", magView);
  element("mag3dClear").addEventListener("click", () => {
    magPoints.length = 0;
    redraw3d();
  });
  redraw3d();
}
