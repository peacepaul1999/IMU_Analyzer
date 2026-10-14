export function maxAbsolute(values: readonly number[]): number {
  return Math.max(1e-5, ...values.map((value) => Math.abs(value)));
}

export function isFiniteTriplet(value: unknown): value is [number, number, number] {
  return (
    Array.isArray(value) &&
    value.length >= 3 &&
    value.slice(0, 3).every((entry) => typeof entry === "number" && Number.isFinite(entry))
  );
}
