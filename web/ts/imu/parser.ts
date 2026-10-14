import type { ImuSample } from "../types";
import { isFiniteTriplet } from "../utils/math";

export function parseSample(message: string): ImuSample | null {
  try {
    const value: unknown = JSON.parse(message);
    if (!isSample(value)) return null;
    return value;
  } catch {
    return null;
  }
}

function isSample(value: unknown): value is ImuSample {
  if (!value || typeof value !== "object") return false;

  const sample = value as Record<string, unknown>;
  return (
    typeof sample.yaw === "number" &&
    typeof sample.roll === "number" &&
    typeof sample.pitch === "number" &&
    isFiniteTriplet(sample.acc) &&
    isFiniteTriplet(sample.gyro) &&
    isFiniteTriplet(sample.mag) &&
    typeof sample.ready === "boolean" &&
    typeof sample.calibrating === "boolean" &&
    typeof sample.northReady === "boolean" &&
    typeof sample.t === "number" &&
    typeof sample.imuHz === "number" &&
    typeof sample.wsHz === "number"
  );
}
