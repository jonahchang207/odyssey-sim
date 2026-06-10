// Direct port of odyssey/pid.cpp

import { sgn } from './util.ts';

export class PID {
  kP: number;
  kI: number;
  kD: number;
  windupRange: number;
  signFlipReset: boolean;

  private integral = 0;
  private prevError = 0;

  constructor(kP: number, kI: number, kD: number, windupRange = 0, signFlipReset = false) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.windupRange = windupRange;
    this.signFlipReset = signFlipReset;
  }

  update(error: number): number {
    this.integral += error;
    // kill the integral when the error crosses zero to prevent oscillation
    if (this.signFlipReset && sgn(error) !== sgn(this.prevError)) this.integral = 0;
    // anti-windup: only integrate near the target
    if (this.windupRange !== 0 && Math.abs(error) > this.windupRange) this.integral = 0;

    const derivative = error - this.prevError;
    this.prevError = error;

    return error * this.kP + this.integral * this.kI + derivative * this.kD;
  }

  reset(): void {
    this.integral = 0;
    this.prevError = 0;
  }
}
