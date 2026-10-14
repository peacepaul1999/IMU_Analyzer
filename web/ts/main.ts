import { updateCalibration } from "./imu/calibration";
import { bindCsvButtons, recordCsvSample } from "./imu/csvLogger";
import { clearDataRates, updateDataRates } from "./imu/dataRate";
import { updateImuState } from "./imu/imuState";
import { parseSample } from "./imu/parser";
import { resizeSensorCharts, updateSensorCharts } from "./charts/sensorCharts";
import { bindView3d, redraw3d, updateView3d } from "./charts/view3d";
import { bindCalibrationButtons } from "./ui/buttons";
import { loadFirmwareVersion, setConnectionStatus } from "./ui/logger";
import { ImuWebSocket } from "./websocket/websocket";

const socket = new ImuWebSocket({
  onOpen: () => setConnectionStatus(true),
  onClose: () => {
    setConnectionStatus(false);
    clearDataRates();
  },
  onMessage: (message) => {
    const sample = parseSample(message);
    if (!sample) return;

    updateImuState(sample);
    updateSensorCharts(sample);
    updateView3d(sample);
    updateDataRates(sample);
    updateCalibration(sample);
    recordCsvSample(sample);
  },
});

bindCalibrationButtons((command) => socket.send(command));
bindCsvButtons();
bindView3d();
void loadFirmwareVersion();
window.addEventListener("resize", () => {
  resizeSensorCharts();
  redraw3d();
});
socket.connect();
