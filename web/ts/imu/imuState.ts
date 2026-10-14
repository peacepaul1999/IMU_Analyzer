import type { ImuSample } from "../types";
import { element } from "../ui/logger";

export function updateImuState(sample: ImuSample): void {
  for (const key of ["yaw", "roll", "pitch"] as const) {
    element(key).textContent = `${sample[key].toFixed(2)}°`;
  }
}
