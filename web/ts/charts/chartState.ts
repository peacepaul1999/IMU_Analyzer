import type { Axis, SensorValues } from "../types";

export const HISTORY_LENGTH = 150;

export const histories: Record<Axis, number[][]> = {
  acc: [[], [], []],
  gyro: [[], [], []],
  mag: [[], [], []],
};

export function recordValues(axis: Axis, values: SensorValues): void {
  values.forEach((value, index) => {
    const history = histories[axis][index];
    history.push(value);
    if (history.length > HISTORY_LENGTH) history.shift();
  });
}
