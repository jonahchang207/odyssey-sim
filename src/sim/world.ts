// Wires a Robot (physics + virtual sensors) to a Chassis (the ported
// library) and steps them together, 10 simulated milliseconds at a time.

import { Chassis, type ControllerSettings } from '../lib/chassis.ts';
import { Pose } from '../lib/pose.ts';
import { TrackingWheel } from '../lib/trackingWheel.ts';
import { degToRad } from '../lib/util.ts';
import { SimClock, flushTasks } from './clock.ts';
import { SimRotation } from './devices.ts';
import { Robot, defaultNoise, type NoiseConfig } from './robot.ts';

export interface WheelConfig {
  enabled: boolean;
  /** wheel diameter, inches */
  diameter: number;
  /** signed offset from tracking center, inches */
  offset: number;
}

export interface WorldConfig {
  trackWidth: number;
  wheelDiameter: number;
  rpm: number;
  cartridgeRpm: number;
  horizontalDrift: number;
  /** motor response time constant, seconds */
  motorTau: number;
  /** robot footprint, inches */
  robotWidth: number;
  robotLength: number;
  vertical: WheelConfig;
  horizontal: WheelConfig;
  useImu: boolean;
  noise: NoiseConfig;
  lateral: ControllerSettings;
  angular: ControllerSettings;
}

/** mirrors the example robot config in the library's src/main.cpp */
export function defaultConfig(): WorldConfig {
  return {
    trackWidth: 11.5,
    wheelDiameter: 3.25,
    rpm: 450,
    cartridgeRpm: 600,
    horizontalDrift: 2,
    motorTau: 0.18,
    robotWidth: 14,
    robotLength: 14,
    vertical: { enabled: true, diameter: 2.125, offset: -1.25 },
    horizontal: { enabled: true, diameter: 2.125, offset: -2.5 },
    useImu: true,
    noise: { ...defaultNoise },
    lateral: {
      kP: 10, kI: 0, kD: 3, windupRange: 3,
      smallError: 1, smallErrorTimeout: 100,
      largeError: 3, largeErrorTimeout: 500,
      slew: 20,
    },
    angular: {
      kP: 2, kI: 0, kD: 10, windupRange: 3,
      smallError: 1, smallErrorTimeout: 100,
      largeError: 3, largeErrorTimeout: 500,
      slew: 0,
    },
  };
}

export interface TrailPoint {
  x: number;
  y: number;
}

const TICK_MS = 10;
const MAX_TRAIL = 6000; // one minute of sim time

export class World {
  clock = new SimClock();
  robot: Robot;
  chassis: Chassis;
  config: WorldConfig;

  trueTrail: TrailPoint[] = [];
  odomTrail: TrailPoint[] = [];

  /** called once per tick, after physics (driver control hooks in here) */
  onTick: (() => void) | null = null;

  constructor(config: WorldConfig) {
    this.config = config;
    this.robot = new Robot({
      trackWidth: config.trackWidth,
      wheelDiameter: config.wheelDiameter,
      rpm: config.rpm,
      cartridgeRpm: config.cartridgeRpm,
      motorTau: config.motorTau,
      width: config.robotWidth,
      length: config.robotLength,
      noise: config.noise,
    });

    let vertical1: TrackingWheel | null = null;
    let horizontal1: TrackingWheel | null = null;
    if (config.vertical.enabled) {
      const encoder = new SimRotation();
      this.robot.wheels.push({
        encoder,
        diameter: config.vertical.diameter,
        offset: config.vertical.offset,
        orientation: 'vertical',
      });
      vertical1 = new TrackingWheel(encoder, config.vertical.diameter, config.vertical.offset);
    }
    if (config.horizontal.enabled) {
      const encoder = new SimRotation();
      this.robot.wheels.push({
        encoder,
        diameter: config.horizontal.diameter,
        offset: config.horizontal.offset,
        orientation: 'horizontal',
      });
      horizontal1 = new TrackingWheel(encoder, config.horizontal.diameter, config.horizontal.offset);
    }

    this.chassis = new Chassis(
      this.clock,
      {
        leftMotors: this.robot.leftMotors,
        rightMotors: this.robot.rightMotors,
        trackWidth: config.trackWidth,
        wheelDiameter: config.wheelDiameter,
        rpm: config.rpm,
        horizontalDrift: config.horizontalDrift,
      },
      config.lateral,
      config.angular,
      {
        vertical1,
        vertical2: null,
        horizontal1,
        horizontal2: null,
        imu: config.useImu ? this.robot.imu : null,
      },
    );

    // in the sim, setting the pose also teleports the true robot: on a real
    // robot the coordinate system moves instead, which is the same thing
    const libSetPose = this.chassis.setPose.bind(this.chassis);
    this.chassis.setPose = (x: number, y: number, theta: number, radians = false) => {
      libSetPose(x, y, theta, radians);
      const mathTheta = radians ? theta : degToRad(90 - theta);
      this.robot.setTruePose(x, y, mathTheta);
      this.trueTrail.length = 0;
      this.odomTrail.length = 0;
    };

    this.chassis.calibrate();
  }

  /** advance the simulation by one 10ms tick */
  async step(): Promise<void> {
    // wake every task sleeping on the clock (odometry, motions, programs)
    // and let each of them run to its next delay before physics moves on
    this.clock.advance(TICK_MS);
    await flushTasks();
    this.robot.step(TICK_MS / 1000);
    this.onTick?.();

    this.trueTrail.push({ x: this.robot.x, y: this.robot.y });
    const odom = this.chassis.getPose(true);
    this.odomTrail.push({ x: odom.x, y: odom.y });
    if (this.trueTrail.length > MAX_TRAIL) this.trueTrail.shift();
    if (this.odomTrail.length > MAX_TRAIL) this.odomTrail.shift();
  }

  /** odometry's estimate, compass degrees */
  getOdomPose(): Pose {
    return this.chassis.getPose();
  }

  /** ground truth, compass degrees */
  getTruePose(): Pose {
    return new Pose(this.robot.x, this.robot.y, 90 - (this.robot.theta * 180) / Math.PI);
  }

  dispose(): void {
    this.clock.stop();
  }
}
