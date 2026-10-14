function element(id: string): HTMLElement {
  const node = document.getElementById(id);
  if (!node) throw new Error(`Missing element: ${id}`);
  return node;
}

export function setConnectionStatus(connected: boolean): void {
  const connection = element("connection");
  connection.textContent = connected ? "Connected" : "Disconnected";
  connection.classList.toggle("online", connected);
}

export async function loadFirmwareVersion(): Promise<void> {
  const version = element("firmwareVersion");

  try {
    const response = await fetch("/version", { cache: "no-store" });
    if (!response.ok) {
      throw new Error(`Version request failed with status ${response.status}`);
    }

    const firmwareVersion = (await response.text()).trim();
    if (!firmwareVersion) {
      throw new Error("Version request returned an empty response");
    }

    version.textContent = `Firmware ${firmwareVersion}`;
  } catch (error) {
    version.textContent = "Firmware unavailable";
    console.error("Failed to load firmware version:", error);
  }
}

export { element };
