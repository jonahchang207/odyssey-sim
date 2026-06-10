// Direct port of odyssey/drivecurve.cpp

import { sgn } from './util.ts';

export interface DriveCurve {
  curve(input: number): number;
}

/**
 * Exponential drive curve.
 * - deadband: inputs smaller than this are ignored (fixes stick drift)
 * - minOutput: minimum output once outside the deadband (overcomes friction)
 * - curve: exponential gain. 1 = linear. Typical values are 1.01 - 1.05
 */
export class ExpoDriveCurve implements DriveCurve {
  private deadband: number;
  private minOutput: number;
  private curveGain: number;

  constructor(deadband = 0, minOutput = 0, curve = 1) {
    this.deadband = deadband;
    this.minOutput = minOutput;
    this.curveGain = curve;
  }

  curve(input: number): number {
    // ignore input inside the deadzone
    if (Math.abs(input) <= this.deadband) return 0;
    // g(x) = |x| - deadband, remapped so g spans (0, 127 - deadband]
    const g = Math.abs(input) - this.deadband;
    const g127 = 127 - this.deadband;
    // exponential curve, normalized so full deflection still gives 127
    const i = Math.pow(this.curveGain, g - 127) * g;
    const i127 = Math.pow(this.curveGain, g127 - 127) * g127;
    return ((127 - this.minOutput) / 127) * i * (127 / i127) * sgn(input) + this.minOutput * sgn(input);
  }
}
