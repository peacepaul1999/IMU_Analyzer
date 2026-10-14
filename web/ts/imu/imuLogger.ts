import type { ImuSample } from "../types";
import { formatTime } from "../utils/time";

export function logSample(sample: ImuSample): void {
  console.debug(`[${formatTime()}] IMU sample`, sample);
}
