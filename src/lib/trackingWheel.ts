// Direct port of odyssey/trackingwheel.cpp

import { SimMotorGroup, SimRotation } from '../sim/devices.ts';

/**
 * Real-world diameters of common VEX omni wheels, in inches.
 * Wheels are rarely the size printed on the box.
 */
export const Omniwheel = {
  NEW_2: 2.125,
  NEW_275: 2.75,
  OLD_275: 2.75,
  NEW_325: 3.25,
  NEW_4: 4.0,
  OLD_4: 4.18,
} as const;

/**
 * A tracking wheel: a wheel + encoder used to measure distance traveled.
 *
 * Offset sign conventions (distance from the tracking center, inches):
 *  - vertical wheels: negative = left of center, positive = right
 *  - horizontal wheels: negative = behind center, positive = in front
 */
export class TrackingWheel {
  private rotation: SimRotation | null = null;
  private motors: SimMotorGroup[] | null = null;
  private diameter: number;
  private offset: number;
  private gearRatio: number;
  private rpm = 0;

  constructor(
    encoder: SimRotation | SimMotorGroup[],
    wheelDiameter: number,
    offset: number,
    gearRatioOrRpm = 1,
  ) {
    if (encoder instanceof SimRotation) {
      this.rotation = encoder;
      this.gearRatio = gearRatioOrRpm;
    } else {
      this.motors = encoder;
      this.gearRatio = 1;
      this.rpm = gearRatioOrRpm;
    }
    this.diameter = wheelDiameter;
    this.offset = offset;
  }

  reset(): void {
    if (this.rotation) this.rotation.reset_position();
    if (this.motors) for (const m of this.motors) m.tare_position_all();
  }

  getDistanceTraveled(): number {
    const circumference = this.diameter * Math.PI;
    if (this.rotation) {
      // get_position returns centidegrees
      return (this.rotation.get_position() / 36000) * this.gearRatio * circumference;
    }
    if (this.motors) {
      // average all motors in the group, accounting for each cartridge
      let total = 0;
      let count = 0;
      for (const group of this.motors) {
        const positions = group.get_position_all();
        const cartridges = group.get_gearing_rpm_all();
        for (let i = 0; i < positions.length; i++) {
          // wheel revolutions = motor revolutions * external ratio
          total += positions[i] * (this.rpm / cartridges[i]) * circumference;
          count++;
        }
      }
      return count > 0 ? total / count : 0;
    }
    return 0;
  }

  getOffset(): number {
    return this.offset;
  }
}
