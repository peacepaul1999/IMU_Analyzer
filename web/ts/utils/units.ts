import type { Axis, SensorValues } from "../types";

const RAD_TO_DEG = 180 / Math.PI;

/** Display unit per sensor, plus the factor from the firmware unit. */
export const UNITS: Record<Axis, { unit: string; factor: number }> = {
  acc: { unit: "g", factor: 1 },
  gyro: { unit: "°/s", factor: RAD_TO_DEG },
  mag: { unit: "µT", factor: 1 },
};

export function toDisplayUnits(axis: Axis, values: SensorValues): SensorValues {
  const { factor } = UNITS[axis];
  return [values[0] * factor, values[1] * factor, values[2] * factor];
}
