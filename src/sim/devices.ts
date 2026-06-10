// Virtual V5 devices. These expose the same readings the PROS devices give
// the library on a real robot; the physics model in robot.ts drives them.

export type BrakeMode = 'coast' | 'brake' | 'hold';

/** pros::MotorGroup stand-in for one side of the drivetrain */
export class SimMotorGroup {
  /** last commanded power, -127..127, or null when braking */
  command: number | null = 0;
  brakeMode: BrakeMode = 'coast';
  /** motor shaft position, rotations (PROS MotorUnits::rotations) */
  position = 0;
  /** cartridge rpm, used by the motor-encoder tracking wheel fallback */
  cartridgeRpm = 600;

  move(power: number): void {
    this.command = Math.max(-127, Math.min(127, power));
  }

  brake(): void {
    this.command = null;
  }

  set_brake_mode_all(mode: BrakeMode): void {
    this.brakeMode = mode;
  }

  get_brake_mode(): BrakeMode {
    return this.brakeMode;
  }

  tare_position_all(): void {
    this.position = 0;
  }

  get_position_all(): number[] {
    return [this.position];
  }

  get_gearing_rpm_all(): number[] {
    return [this.cartridgeRpm];
  }
}

/** pros::Rotation stand-in. Position in centidegrees, like the real sensor */
export class SimRotation {
  private centidegrees = 0;

  /** physics pushes wheel travel in as an angle, quantized like hardware */
  addRotation(deg: number): void {
    this.centidegrees += deg * 100;
  }

  get_position(): number {
    // the real sensor reports whole centidegrees
    return Math.round(this.centidegrees);
  }

  reset_position(): void {
    this.centidegrees = 0;
  }
}

/** pros::Imu stand-in. Rotation in degrees, clockwise positive, unbounded */
export class SimImu {
  private rotation = 0;
  /** additive drift + gaussian noise, injected by the physics model */
  addRotation(deg: number): void {
    this.rotation += deg;
  }

  get_rotation(): number {
    return this.rotation;
  }

  reset(): void {
    this.rotation = 0;
  }
}

/** deterministic RNG (mulberry32) so noisy runs are reproducible */
export function makeRng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a += 0x6d2b79f5;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** Box-Muller gaussian from a uniform RNG */
export function gaussian(rng: () => number): number {
  let u = 0;
  let v = 0;
  while (u === 0) u = rng();
  while (v === 0) v = rng();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}
