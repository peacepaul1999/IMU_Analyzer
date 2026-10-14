export function drawSphereChart(canvas: HTMLCanvasElement): void {
  const context = canvas.getContext("2d");
  if (!context) return;

  const radius = Math.min(canvas.width, canvas.height) / 2;
  context.beginPath();
  context.arc(canvas.width / 2, canvas.height / 2, radius, 0, Math.PI * 2);
  context.strokeStyle = "#808080";
  context.stroke();
}
