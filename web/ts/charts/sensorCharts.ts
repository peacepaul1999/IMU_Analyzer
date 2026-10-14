import type { Axis, ImuSample } from "../types";
import { maxAbsolute } from "../utils/math";
import { UNITS, toDisplayUnits } from "../utils/units";
import { element } from "../ui/logger";
import { histories, HISTORY_LENGTH, recordValues } from "./chartState";

const axes: readonly Axis[] = ["acc", "gyro", "mag"];
const lineColors = ["#ff3c00", "#0011ff", "#33ff00"];

export function updateSensorCharts(sample: ImuSample): void {
  for (const axis of axes) {
    recordValues(axis, toDisplayUnits(axis, sample[axis]));
    drawSensorChart(axis);
  }
}

export function resizeSensorCharts(): void {
  axes.forEach(drawSensorChart);
}

function drawSensorChart(axis: Axis): void {
  const canvas = element(`${axis}Chart`) as HTMLCanvasElement;
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
      const x = (position / (HISTORY_LENGTH - 1)) * width;
      const y = height / 2 - ((value / range) * height) / 2;
      if (position === 0) context.moveTo(x, y);
      else context.lineTo(x, y);
    });
    context.stroke();
  });

  drawScaleLabels(context, width, height, devicePixelRatio, range, UNITS[axis].unit);
}

function formatTick(value: number): string {
  const magnitude = Math.abs(value);
  return value.toFixed(magnitude >= 100 ? 0 : magnitude >= 10 ? 1 : 2);
}

function drawScaleLabels(
  context: CanvasRenderingContext2D,
  width: number,
  height: number,
  devicePixelRatio: number,
  range: number,
  unit: string,
): void {
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

function drawGrid(
  context: CanvasRenderingContext2D,
  width: number,
  height: number,
  devicePixelRatio: number,
): void {
  context.strokeStyle = "#ffffff20";
  context.lineWidth = devicePixelRatio;
  for (let index = 1; index < 4; index += 1) {
    const y = (height * index) / 4;
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(width, y);
    context.stroke();
  }
}
