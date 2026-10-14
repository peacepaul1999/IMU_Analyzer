import type { ImuSample } from "../types";
import { element } from "../ui/logger";

export function updateCalibration(sample: ImuSample): void {
  element("calState").textContent = sample.calibrating ? "Calibrating" : "Idle";
  element("northReady").textContent = sample.northReady ? "Yes" : "No";
}
