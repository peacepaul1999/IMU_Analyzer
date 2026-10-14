import type { CalibrationCommand } from "../types";
import { element } from "./logger";

export function bindCalibrationButtons(send: (command: CalibrationCommand) => void): void {
  const buttons: ReadonlyArray<[string, CalibrationCommand]> = [
    ["calStart", "CAL_START"],
    ["calStop", "CAL_STOP"],
    ["setNorth", "SET_NORTH"],
  ];

  for (const [id, command] of buttons) {
    element(id).addEventListener("click", () => send(command));
  }
}
