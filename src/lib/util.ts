// Direct port of odyssey/util.cpp

import { Pose } from './pose.ts';

/** force a turn direction, or AUTO for the shortest path */
export type AngularDirection = 'auto' | 'cw' | 'ccw';

export function sgn(value: number): number {
  return value < 0 ? -1 : 1;
}

export function radToDeg(rad: number): number {
  return (rad * 180) / Math.PI;
}

export function degToRad(deg: number): number {
  return (deg * Math.PI) / 180;
}

export function sanitizeAngle(angle: number, radians = true): number {
  const max = radians ? 2 * Math.PI : 360;
  angle = angle % max;
  if (angle < 0) angle += max;
  return angle;
}

/** IEEE remainder, like C++ std::remainder */
function remainder(a: number, b: number): number {
  const q = Math.round(a / b);
  return a - q * b;
}

export function angleError(
  target: number,
  position: number,
  radians = true,
  direction: AngularDirection = 'auto',
): number {
  const max = radians ? 2 * Math.PI : 360;
  const rawError = sanitizeAngle(target, radians) - sanitizeAngle(position, radians);
  switch (direction) {
    case 'cw':
      // force a positive (clockwise, in the compass frame) error
      return rawError < 0 ? rawError + max : rawError;
    case 'ccw':
      // force a negative (counterclockwise, in the compass frame) error
      return rawError > 0 ? rawError - max : rawError;
    default:
      // shortest path
      return remainder(rawError, max);
  }
}

export function avg(values: number[]): number {
  if (values.length === 0) return 0;
  let sum = 0;
  for (const value of values) sum += value;
  return sum / values.length;
}

export function ema(current: number, previous: number, smooth: number): number {
  return current * smooth + previous * (1 - smooth);
}

export function slew(target: number, current: number, maxChange: number): number {
  if (maxChange === 0) return target;
  let change = target - current;
  if (change > maxChange) change = maxChange;
  else if (change < -maxChange) change = -maxChange;
  return current + change;
}

export function getCurvature(pose: Pose, other: Pose): number {
  const d = pose.distance(other);
  if (d === 0) return 0;
  // angle from the robot's heading to the chord, CCW positive
  const beta = angleError(pose.angle(other), pose.theta, true);
  // curvature of the arc tangent to the heading through the other point
  return (2 * Math.sin(beta)) / d;
}

export function clamp(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}
