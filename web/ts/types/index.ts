export type Axis = "acc" | "gyro" | "mag";

export type SensorValues = [number, number, number];

/** Units as sent by the firmware: acc [g], gyro [rad/s], mag [µT], angles [deg]. */
export type ImuSample = {
  yaw: number;
  roll: number;
  pitch: number;
  acc: SensorValues;
  gyro: SensorValues;
  mag: SensorValues;
  ready: boolean;
  calibrating: boolean;
  northReady: boolean;
  /** Device uptime in ms when the frame was built. */
  t: number;
  /** IMU sampling rate configured in firmware [Hz]. */
  imuHz: number;
  /** WebSocket publish rate configured in firmware [Hz]. */
  wsHz: number;
};

export type CalibrationCommand = "CAL_START" | "CAL_STOP" | "SET_NORTH";
