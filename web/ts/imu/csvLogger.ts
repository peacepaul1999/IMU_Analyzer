import type { ImuSample } from "../types";
import { element } from "../ui/logger";
import { UNITS } from "../utils/units";

const MAX_ROWS = 50000;

const HEADER = [
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
  "mag_z_uT",
].join(",");

const rows: string[] = [];
let recording = false;

export function recordCsvSample(sample: ImuSample): void {
  if (!recording) return;
  if (rows.length >= MAX_ROWS) {
    setRecording(false);
    return;
  }

  const gyro = sample.gyro.map((value) => value * UNITS.gyro.factor);
  const fields = [
    sample.t,
    new Date().toISOString(),
    ...[sample.yaw, sample.roll, sample.pitch].map((value) => value.toFixed(3)),
    ...sample.acc.map((value) => value.toFixed(4)),
    ...gyro.map((value) => value.toFixed(3)),
    ...sample.mag.map((value) => value.toFixed(3)),
  ];
  rows.push(fields.join(","));
  updateStatus();
}

function setRecording(next: boolean): void {
  recording = next;
  element("logToggle").textContent = recording ? "Stop Recording" : "Start Recording";
  updateStatus();
}

function updateStatus(): void {
  element("logStatus").textContent = `${recording ? "Recording" : "Stopped"} · ${rows.length} rows`;
}

function saveCsv(): void {
  if (rows.length === 0) return;
  const blob = new Blob([`${HEADER}\n${rows.join("\n")}\n`], { type: "text/csv" });
  const link = document.createElement("a");
  const stamp = new Date().toISOString().replace(/[:.]/g, "-");
  link.href = URL.createObjectURL(blob);
  link.download = `imu_log_${stamp}.csv`;
  link.click();
  URL.revokeObjectURL(link.href);
}

export function bindCsvButtons(): void {
  element("logToggle").addEventListener("click", () => setRecording(!recording));
  element("logSave").addEventListener("click", saveCsv);
  element("logClear").addEventListener("click", () => {
    rows.length = 0;
    updateStatus();
  });
  updateStatus();
}
