import type { ImuSample } from "../types";
import { element } from "../ui/logger";

const WINDOW = 20;
const arrivals: number[] = [];

export function updateDataRates(sample: ImuSample): void {
  arrivals.push(performance.now());
  if (arrivals.length > WINDOW) arrivals.shift();

  let measured = "—";
  if (arrivals.length >= 2) {
    const elapsed = (arrivals[arrivals.length - 1] - arrivals[0]) / 1000;
    if (elapsed > 0) measured = ((arrivals.length - 1) / elapsed).toFixed(1);
  }

  element("rates").textContent =
    `IMU: ${sample.imuHz} Hz · WebSocket: ${sample.wsHz} Hz (measured ${measured} Hz)`;
}

export function clearDataRates(): void {
  arrivals.length = 0;
  element("rates").textContent = "IMU: — · WebSocket: —";
}
