// Differential-drive physics + virtual sensor wiring.
//
// The robot's TRUE pose lives here, in math radians like the odometry
// internals (0 = +x, CCW positive). Each 10ms tick the physics integrates
// motor commands into motion, then feeds the virtual encoders and IMU the
// readings a real robot would produce — including, optionally, the slip and
// drift that make odometry estimates diverge from reality.

import { SimImu, SimMotorGroup, SimRotation, gaussian, makeRng } from './devices.ts';

export interface MountedWheel {
  encoder: SimRotation;
  /** wheel diameter, inches */
  diameter: number;
  /** signed offset from the tracking center, inches */
  offset: number;
  orientation: 'vertical' | 'horizontal';
}

export interface NoiseConfig {
  enabled: boolean;
  /** stddev of wheel slip, as a fraction of each distance increment */
  wheelSlip: number;
  /** IMU drift, degrees per second */
  imuDrift: number;
  /** IMU white noise, degrees (stddev per tick) */
  imuNoise: number;
}

export interface RobotConfig {
  /** distance between left and right wheels, inches */
  trackWidth: number;
  /** drive wheel diameter, inches */
  wheelDiameter: number;
  /** wheel rpm after external gearing */
  rpm: number;
  /** motor cartridge rpm (600 = blue) */
  cartridgeRpm: number;
  /** first-order motor response time constant, seconds */
  motorTau: number;
  /** robot footprint for drawing, inches */
  width: number;
  length: number;
  noise: NoiseConfig;
}

export const defaultNoise: NoiseConfig = {
  enabled: false,
  wheelSlip: 0.01,
  imuDrift: 0.05,
  imuNoise: 0.02,
};

export class Robot {
  config: RobotConfig;
  leftMotors = new SimMotorGroup();
  rightMotors = new SimMotorGroup();
  imu = new SimImu();
  wheels: MountedWheel[] = [];

  /** true pose, math radians */
  x = 0;
  y = 0;
  theta = Math.PI / 2;

  /** wheel surface speeds, inches per second */
  vLeft = 0;
  vRight = 0;

  private rng = makeRng(0x0d75537);

  constructor(config: RobotConfig) {
    this.config = config;
    this.leftMotors.cartridgeRpm = config.cartridgeRpm;
    this.rightMotors.cartridgeRpm = config.cartridgeRpm;
  }

  /** teleport the true robot (used by setPose: on a real robot the robot
   *  doesn't move, the coordinate system does — same thing here) */
  setTruePose(x: number, y: number, thetaMathRad: number): void {
    this.x = x;
    this.y = y;
    this.theta = thetaMathRad;
  }

  private sideStep(group: SimMotorGroup, v: number, dt: number): number {
    const vmax = (this.config.rpm / 60) * Math.PI * this.config.wheelDiameter;
    let target: number;
    let tau = this.config.motorTau;
    if (group.command === null) {
      target = 0;
      // hold/brake stop harder than coast
      if (group.brakeMode === 'coast') tau *= 3;
    } else {
      target = (group.command / 127) * vmax;
    }
    const alpha = 1 - Math.exp(-dt / tau);
    v += (target - v) * alpha;
    if (group.command === null && group.brakeMode !== 'coast' && Math.abs(v) < 1) v = 0;
    return v;
  }

  step(dt: number): void {
    const cfg = this.config;
    this.vLeft = this.sideStep(this.leftMotors, this.vLeft, dt);
    this.vRight = this.sideStep(this.rightMotors, this.vRight, dt);

    // differential drive kinematics, math frame (right faster = CCW positive)
    const v = (this.vLeft + this.vRight) / 2;
    const w = (this.vRight - this.vLeft) / cfg.trackWidth;
    const midTheta = this.theta + (w * dt) / 2;
    this.x += v * Math.cos(midTheta) * dt;
    this.y += v * Math.sin(midTheta) * dt;
    this.theta += w * dt;

    const noise = cfg.noise;
    const slip = (d: number) =>
      noise.enabled && d !== 0 ? d + gaussian(this.rng) * Math.abs(d) * noise.wheelSlip : d;

    // drive motor encoders (motor shaft rotations)
    const circumference = Math.PI * cfg.wheelDiameter;
    const motorPerWheel = cfg.cartridgeRpm / cfg.rpm;
    this.leftMotors.position += (slip(this.vLeft * dt) / circumference) * motorPerWheel;
    this.rightMotors.position += (slip(this.vRight * dt) / circumference) * motorPerWheel;

    // tracking wheels. Readings invert the odometry equations exactly:
    //   vertical reading   = forward travel + offset * dTheta
    //   horizontal reading = rightward travel - offset * dTheta
    // (an ideal diff drive has no rightward travel; rotation still spins
    // a horizontal wheel through its offset)
    const dTheta = w * dt;
    for (const wheel of this.wheels) {
      const travel =
        wheel.orientation === 'vertical'
          ? v * dt + wheel.offset * dTheta
          : -wheel.offset * dTheta;
      const deg = (slip(travel) / (Math.PI * wheel.diameter)) * 360;
      wheel.encoder.addRotation(deg);
    }

    // IMU: clockwise-positive degrees, with optional drift + white noise
    let imuDelta = -((dTheta * 180) / Math.PI);
    if (noise.enabled) {
      imuDelta += noise.imuDrift * dt + gaussian(this.rng) * noise.imuNoise;
    }
    this.imu.addRotation(imuDelta);
  }
}
