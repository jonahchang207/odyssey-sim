// Direct port of odyssey/chassis.cpp, motions.cpp and purepursuit.cpp.
//
// Sign conventions (identical to the C++ library):
//  - turn/swing functions work in the compass frame (degrees, clockwise
//    positive), so a positive angular output drives the left side forward
//  - drive motions work in the math frame (radians, counterclockwise
//    positive), so a positive angular output drives the right side faster

import { Mutex, spawnTask, type SimClock } from '../sim/clock.ts';
import type { SimMotorGroup, BrakeMode } from '../sim/devices.ts';
import { ExitCondition } from './exitCondition.ts';
import { Odometry, type OdomSensors } from './odometry.ts';
import { PID } from './pid.ts';
import { Pose } from './pose.ts';
import { TrackingWheel } from './trackingWheel.ts';
import {
  angleError,
  clamp,
  degToRad,
  getCurvature,
  radToDeg,
  sanitizeAngle,
  sgn,
  slew,
  type AngularDirection,
} from './util.ts';
import { ExpoDriveCurve, type DriveCurve } from './driveCurve.ts';

// distance from the target where moveToPoint/moveToPose switch into the
// "close" state: heading correction is relaxed and speed is no longer
// allowed to increase, which prevents spinning near the target
const CLOSE_RANGE = 7.5; // inches
// minimum speed cap while close, so the robot can still settle
const CLOSE_MIN_CAP = 60;

export interface Drivetrain {
  leftMotors: SimMotorGroup;
  rightMotors: SimMotorGroup;
  /** distance between the left and right wheels, inches */
  trackWidth: number;
  /** diameter of the drive wheels, inches */
  wheelDiameter: number;
  /** output rpm of the drive wheels (after external gearing) */
  rpm: number;
  /** 2 = all omni wheels (default), 8 = center traction wheels */
  horizontalDrift: number;
}

/** PID gains + settling behavior for a controller (lateral or angular) */
export interface ControllerSettings {
  kP: number;
  kI: number;
  kD: number;
  /** integral anti-windup range. 0 disables the integral limit */
  windupRange: number;
  /** error range for the fast exit (inches or degrees) */
  smallError: number;
  /** time (ms) the error must stay within smallError to exit */
  smallErrorTimeout: number;
  /** error range for the slow "give up" exit */
  largeError: number;
  /** time (ms) the error must stay within largeError to exit */
  largeErrorTimeout: number;
  /** maximum output change per 10ms tick. 0 disables slew */
  slew: number;
}

export interface TurnToHeadingParams {
  direction?: AngularDirection;
  maxSpeed?: number;
  minSpeed?: number;
  /** degrees */
  earlyExitRange?: number;
}

export interface TurnToPointParams extends TurnToHeadingParams {
  forwards?: boolean;
}

export type SwingToHeadingParams = TurnToHeadingParams;

export interface MoveToPointParams {
  forwards?: boolean;
  maxSpeed?: number;
  minSpeed?: number;
  /** inches */
  earlyExitRange?: number;
}

export interface MoveToPoseParams extends MoveToPointParams {
  /** carrot point aggressiveness, 0 to 1. Larger = wider approach arc */
  lead?: number;
}

export type DriveSide = 'left' | 'right';

export class Waypoint extends Pose {
  /** target speed at this point, -127 to 127 */
  speed: number;

  constructor(x = 0, y = 0, speed = 0) {
    super(x, y, 0);
    this.speed = speed;
  }
}

/** index of the path point closest to the robot, never moving backwards */
function findClosest(pose: Pose, path: Waypoint[], startIndex: number): number {
  let closest = startIndex;
  let minDist = pose.distance(path[startIndex]);
  for (let i = startIndex + 1; i < path.length; i++) {
    const dist = pose.distance(path[i]);
    if (dist < minDist) {
      minDist = dist;
      closest = i;
    }
  }
  return closest;
}

/**
 * where the lookahead circle intersects the segment p1 -> p2.
 * Returns t in [0, 1] (the intersection furthest along), or -1 if none.
 */
function circleIntersect(p1: Pose, p2: Pose, center: Pose, radius: number): number {
  const d = p2.sub(p1);
  const f = p1.sub(center);
  const a = d.dot(d);
  const b = 2 * f.dot(d);
  const c = f.dot(f) - radius * radius;
  let discriminant = b * b - 4 * a * c;

  if (discriminant >= 0 && a !== 0) {
    discriminant = Math.sqrt(discriminant);
    const t1 = (-b - discriminant) / (2 * a);
    const t2 = (-b + discriminant) / (2 * a);
    // prefer the intersection furthest along the segment
    if (t2 >= 0 && t2 <= 1) return t2;
    if (t1 >= 0 && t1 <= 1) return t1;
  }
  return -1;
}

export class Chassis {
  drivetrain: Drivetrain;
  lateralSettings: ControllerSettings;
  angularSettings: ControllerSettings;
  sensors: OdomSensors;

  lateralPID: PID;
  angularPID: PID;

  private lateralSmallExit: ExitCondition;
  private lateralLargeExit: ExitCondition;
  private angularSmallExit: ExitCondition;
  private angularLargeExit: ExitCondition;

  private throttleCurve: DriveCurve;
  private steerCurve: DriveCurve;

  private odometry: Odometry;
  private clock: SimClock;

  /** distance traveled by the current motion (inches for drives, degrees
   *  for turns). -1 when no motion is running */
  distTraveled = -1;

  private motionRunning = false;
  private motionQueued = false;
  private calibrated = false;
  private motionMutex = new Mutex();

  constructor(
    clock: SimClock,
    drivetrain: Drivetrain,
    lateralSettings: ControllerSettings,
    angularSettings: ControllerSettings,
    sensors: OdomSensors,
    throttleCurve: DriveCurve | null = null,
    steerCurve: DriveCurve | null = null,
  ) {
    this.clock = clock;
    this.drivetrain = drivetrain;
    this.lateralSettings = lateralSettings;
    this.angularSettings = angularSettings;
    this.sensors = sensors;
    this.lateralPID = new PID(
      lateralSettings.kP, lateralSettings.kI, lateralSettings.kD,
      lateralSettings.windupRange, true,
    );
    this.angularPID = new PID(
      angularSettings.kP, angularSettings.kI, angularSettings.kD,
      angularSettings.windupRange, true,
    );
    this.lateralSmallExit = new ExitCondition(lateralSettings.smallError, lateralSettings.smallErrorTimeout, clock);
    this.lateralLargeExit = new ExitCondition(lateralSettings.largeError, lateralSettings.largeErrorTimeout, clock);
    this.angularSmallExit = new ExitCondition(angularSettings.smallError, angularSettings.smallErrorTimeout, clock);
    this.angularLargeExit = new ExitCondition(angularSettings.largeError, angularSettings.largeErrorTimeout, clock);
    this.throttleCurve = throttleCurve ?? new ExpoDriveCurve(0, 0, 1);
    this.steerCurve = steerCurve ?? new ExpoDriveCurve(0, 0, 1);
    if (this.drivetrain.horizontalDrift === 0) this.drivetrain.horizontalDrift = 2;
    this.odometry = new Odometry(clock);
  }

  calibrate(): void {
    if (this.calibrated) return;
    this.calibrated = true;

    // fall back to drive motor encoders if no vertical tracking wheels exist
    if (this.sensors.vertical1 === null && this.sensors.vertical2 === null) {
      this.sensors.vertical1 = new TrackingWheel(
        [this.drivetrain.leftMotors], this.drivetrain.wheelDiameter,
        -this.drivetrain.trackWidth / 2, this.drivetrain.rpm,
      );
      this.sensors.vertical2 = new TrackingWheel(
        [this.drivetrain.rightMotors], this.drivetrain.wheelDiameter,
        this.drivetrain.trackWidth / 2, this.drivetrain.rpm,
      );
    }

    // zero everything and start the tracking task
    this.sensors.vertical1?.reset();
    this.sensors.vertical2?.reset();
    this.sensors.horizontal1?.reset();
    this.sensors.horizontal2?.reset();
    this.drivetrain.leftMotors.tare_position_all();
    this.drivetrain.rightMotors.tare_position_all();

    this.odometry.setSensors(this.sensors);
    this.odometry.setPose(new Pose(0, 0, 0), false);
    this.odometry.init();
  }

  setPose(x: number, y: number, theta: number, radians = false): void {
    this.odometry.setPose(new Pose(x, y, theta), radians);
  }

  getPose(radians = false): Pose {
    return this.odometry.getPose(radians);
  }

  // --------------------------------------------------------------------
  // motion management
  // --------------------------------------------------------------------

  private async requestMotionStart(): Promise<void> {
    if (this.isInMotion()) this.motionQueued = true;
    else this.motionRunning = true;

    // blocks until the previous motion gives the mutex back
    await this.motionMutex.take();

    this.motionRunning = true;
    this.motionQueued = false;
  }

  private endMotion(): void {
    // if another motion is queued, it keeps the "running" flag alive
    this.motionRunning = this.motionQueued;
    this.motionMutex.give();
  }

  async cancelMotion(): Promise<void> {
    this.motionRunning = false;
    await this.clock.delay(10); // give the motion loop a tick to notice
  }

  async cancelAllMotions(): Promise<void> {
    this.motionRunning = false;
    this.motionQueued = false;
    await this.clock.delay(10);
  }

  isInMotion(): boolean {
    return this.motionRunning;
  }

  /** block until the current motion passes dist (inches for drives,
   *  degrees for turns), or until it ends */
  async waitUntil(dist: number): Promise<void> {
    do {
      await this.clock.delay(10);
    } while (this.distTraveled <= dist && this.distTraveled !== -1);
  }

  /** block until the current motion finishes */
  async waitUntilDone(): Promise<void> {
    do {
      await this.clock.delay(10);
    } while (this.distTraveled !== -1);
  }

  private stopDrive(): void {
    this.drivetrain.leftMotors.brake();
    this.drivetrain.rightMotors.brake();
  }

  setBrakeMode(mode: BrakeMode): void {
    this.drivetrain.leftMotors.set_brake_mode_all(mode);
    this.drivetrain.rightMotors.set_brake_mode_all(mode);
  }

  // --------------------------------------------------------------------
  // motions
  // --------------------------------------------------------------------

  /** Turn in place to face a heading (compass degrees) */
  async turnToHeading(
    theta: number, timeout: number, params: TurnToHeadingParams = {}, async = true,
  ): Promise<void> {
    await this.requestMotionStart();
    if (!this.motionRunning) return;
    // re-call this function in a new task if the motion is async
    if (async) {
      spawnTask(() => this.turnToHeading(theta, timeout, params, false));
      this.endMotion();
      await this.clock.delay(10);
      return;
    }

    const maxSpeed = params.maxSpeed ?? 127;
    const minSpeed = params.minSpeed ?? 0;
    const earlyExitRange = params.earlyExitRange ?? 0;
    let direction: AngularDirection = params.direction ?? 'auto';

    this.angularPID.reset();
    this.angularSmallExit.reset();
    this.angularLargeExit.reset();

    const targetTheta = sanitizeAngle(theta, false);
    const startTheta = this.getPose().theta;
    const startTime = this.clock.millis();
    this.distTraveled = 0;

    while (
      this.clock.millis() - startTime < timeout &&
      !this.angularSmallExit.getExit() && !this.angularLargeExit.getExit() &&
      this.motionRunning
    ) {
      const currentTheta = this.getPose().theta; // compass degrees
      this.distTraveled = Math.abs(angleError(currentTheta, startTheta, false));

      let error = angleError(targetTheta, currentTheta, false, direction);
      // once a forced-direction turn gets near the target, release the
      // direction lock so an overshoot doesn't cause a full extra rotation
      if (direction !== 'auto' && Math.abs(error) < 45) direction = 'auto';

      this.angularSmallExit.update(error);
      this.angularLargeExit.update(error);

      let power = this.angularPID.update(error);
      power = clamp(power, -maxSpeed, maxSpeed);
      if (minSpeed !== 0 && Math.abs(power) < minSpeed) power = minSpeed * sgn(power);
      if (minSpeed !== 0 && Math.abs(error) < earlyExitRange) break;

      // positive error = clockwise = left forward, right backward
      this.drivetrain.leftMotors.move(power);
      this.drivetrain.rightMotors.move(-power);
      await this.clock.delay(10);
    }

    this.stopDrive();
    this.distTraveled = -1;
    this.endMotion();
  }

  /** Turn in place to face a point */
  async turnToPoint(
    x: number, y: number, timeout: number, params: TurnToPointParams = {}, async = true,
  ): Promise<void> {
    await this.requestMotionStart();
    if (!this.motionRunning) return;
    if (async) {
      spawnTask(() => this.turnToPoint(x, y, timeout, params, false));
      this.endMotion();
      await this.clock.delay(10);
      return;
    }

    const forwards = params.forwards ?? true;
    const maxSpeed = params.maxSpeed ?? 127;
    const minSpeed = params.minSpeed ?? 0;
    const earlyExitRange = params.earlyExitRange ?? 0;
    let direction: AngularDirection = params.direction ?? 'auto';

    this.angularPID.reset();
    this.angularSmallExit.reset();
    this.angularLargeExit.reset();

    const target = new Pose(x, y);
    const startTheta = this.getPose().theta;
    const startTime = this.clock.millis();
    this.distTraveled = 0;

    while (
      this.clock.millis() - startTime < timeout &&
      !this.angularSmallExit.getExit() && !this.angularLargeExit.getExit() &&
      this.motionRunning
    ) {
      const pose = this.getPose(true); // math radians
      this.distTraveled = Math.abs(angleError(this.getPose().theta, startTheta, false));

      // recompute the target heading every tick in case the robot drifts
      let targetTheta = 90 - radToDeg(pose.angle(target));
      if (!forwards) targetTheta += 180;

      let error = angleError(targetTheta, this.getPose().theta, false, direction);
      if (direction !== 'auto' && Math.abs(error) < 45) direction = 'auto';

      this.angularSmallExit.update(error);
      this.angularLargeExit.update(error);

      let power = this.angularPID.update(error);
      power = clamp(power, -maxSpeed, maxSpeed);
      if (minSpeed !== 0 && Math.abs(power) < minSpeed) power = minSpeed * sgn(power);
      if (minSpeed !== 0 && Math.abs(error) < earlyExitRange) break;

      this.drivetrain.leftMotors.move(power);
      this.drivetrain.rightMotors.move(-power);
      await this.clock.delay(10);
    }

    this.stopDrive();
    this.distTraveled = -1;
    this.endMotion();
  }

  /** Turn to a heading with one side of the drive locked (swing turn) */
  async swingToHeading(
    theta: number, lockedSide: DriveSide, timeout: number,
    params: SwingToHeadingParams = {}, async = true,
  ): Promise<void> {
    await this.requestMotionStart();
    if (!this.motionRunning) return;
    if (async) {
      spawnTask(() => this.swingToHeading(theta, lockedSide, timeout, params, false));
      this.endMotion();
      await this.clock.delay(10);
      return;
    }

    const maxSpeed = params.maxSpeed ?? 127;
    const minSpeed = params.minSpeed ?? 0;
    const earlyExitRange = params.earlyExitRange ?? 0;
    let direction: AngularDirection = params.direction ?? 'auto';

    this.angularPID.reset();
    this.angularSmallExit.reset();
    this.angularLargeExit.reset();

    const targetTheta = sanitizeAngle(theta, false);
    const startTheta = this.getPose().theta;
    const startTime = this.clock.millis();
    this.distTraveled = 0;

    // hold the locked side still while the other side swings
    const lockedGroup =
      lockedSide === 'left' ? this.drivetrain.leftMotors : this.drivetrain.rightMotors;
    const prevMode = lockedGroup.get_brake_mode();
    lockedGroup.set_brake_mode_all('hold');

    while (
      this.clock.millis() - startTime < timeout &&
      !this.angularSmallExit.getExit() && !this.angularLargeExit.getExit() &&
      this.motionRunning
    ) {
      const currentTheta = this.getPose().theta;
      this.distTraveled = Math.abs(angleError(currentTheta, startTheta, false));

      let error = angleError(targetTheta, currentTheta, false, direction);
      if (direction !== 'auto' && Math.abs(error) < 45) direction = 'auto';

      this.angularSmallExit.update(error);
      this.angularLargeExit.update(error);

      let power = this.angularPID.update(error);
      power = clamp(power, -maxSpeed, maxSpeed);
      if (minSpeed !== 0 && Math.abs(power) < minSpeed) power = minSpeed * sgn(power);
      if (minSpeed !== 0 && Math.abs(error) < earlyExitRange) break;

      // positive error = clockwise. With the left side locked, clockwise
      // means the right side drives backward, and vice versa
      if (lockedSide === 'left') {
        this.drivetrain.leftMotors.brake();
        this.drivetrain.rightMotors.move(-power);
      } else {
        this.drivetrain.rightMotors.brake();
        this.drivetrain.leftMotors.move(power);
      }
      await this.clock.delay(10);
    }

    this.stopDrive();
    lockedGroup.set_brake_mode_all(prevMode);
    this.distTraveled = -1;
    this.endMotion();
  }

  /** Drive to a point. Heading at arrival is not controlled */
  async moveToPoint(
    x: number, y: number, timeout: number, params: MoveToPointParams = {}, async = true,
  ): Promise<void> {
    await this.requestMotionStart();
    if (!this.motionRunning) return;
    if (async) {
      spawnTask(() => this.moveToPoint(x, y, timeout, params, false));
      this.endMotion();
      await this.clock.delay(10);
      return;
    }

    const forwards = params.forwards ?? true;
    const minSpeed = params.minSpeed ?? 0;
    const earlyExitRange = params.earlyExitRange ?? 0;
    let maxSpeed = params.maxSpeed ?? 127;

    this.lateralPID.reset();
    this.angularPID.reset();
    this.lateralSmallExit.reset();
    this.lateralLargeExit.reset();

    const target = new Pose(x, y);
    const startTime = this.clock.millis();
    let lastPose = this.getPose();
    let prevLateralOut = 0;
    let close = false;
    this.distTraveled = 0;

    while (
      this.clock.millis() - startTime < timeout &&
      !this.lateralSmallExit.getExit() && !this.lateralLargeExit.getExit() &&
      this.motionRunning
    ) {
      const pose = this.getPose(true); // math radians
      this.distTraveled += pose.distance(lastPose);
      lastPose = pose;

      const distToTarget = pose.distance(target);
      if (!close && distToTarget < CLOSE_RANGE) {
        close = true;
        maxSpeed = Math.max(Math.abs(prevLateralOut), CLOSE_MIN_CAP);
      }

      // heading error to the target, CCW positive. Flip the robot's
      // heading when driving backwards
      const headingTheta = forwards ? pose.theta : pose.theta + Math.PI;
      const angularError = angleError(pose.angle(target), headingTheta, true);

      // project the distance onto the robot's heading so driving past the
      // target produces a negative (corrective) error
      let lateralError = distToTarget * Math.cos(angularError);
      if (!forwards) lateralError = -lateralError;

      this.lateralSmallExit.update(lateralError);
      this.lateralLargeExit.update(lateralError);

      let lateralOut = this.lateralPID.update(lateralError);
      lateralOut = clamp(lateralOut, -maxSpeed, maxSpeed);
      lateralOut = slew(lateralOut, prevLateralOut, this.lateralSettings.slew);
      if (!close && minSpeed !== 0 && Math.abs(lateralOut) < minSpeed)
        lateralOut = minSpeed * sgn(lateralOut);
      prevLateralOut = lateralOut;

      if (minSpeed !== 0 && distToTarget < earlyExitRange) break;

      // stop correcting heading when close so the robot doesn't spin on
      // top of the target point
      const angularOut = close ? 0 : this.angularPID.update(radToDeg(angularError));

      // math frame: positive (CCW) angular error -> right side faster
      let leftPower = lateralOut - angularOut;
      let rightPower = lateralOut + angularOut;
      const ratio = Math.max(Math.abs(leftPower), Math.abs(rightPower)) / maxSpeed;
      if (ratio > 1) {
        leftPower /= ratio;
        rightPower /= ratio;
      }
      this.drivetrain.leftMotors.move(leftPower);
      this.drivetrain.rightMotors.move(rightPower);
      await this.clock.delay(10);
    }

    this.stopDrive();
    this.distTraveled = -1;
    this.endMotion();
  }

  /** Drive to a pose (position AND final heading), boomerang controller */
  async moveToPose(
    x: number, y: number, theta: number, timeout: number,
    params: MoveToPoseParams = {}, async = true,
  ): Promise<void> {
    await this.requestMotionStart();
    if (!this.motionRunning) return;
    if (async) {
      spawnTask(() => this.moveToPose(x, y, theta, timeout, params, false));
      this.endMotion();
      await this.clock.delay(10);
      return;
    }

    const forwards = params.forwards ?? true;
    const lead = params.lead ?? 0.6;
    const minSpeed = params.minSpeed ?? 0;
    const earlyExitRange = params.earlyExitRange ?? 0;
    let maxSpeed = params.maxSpeed ?? 127;

    this.lateralPID.reset();
    this.angularPID.reset();
    this.lateralSmallExit.reset();
    this.lateralLargeExit.reset();

    // target heading in math radians. When driving backwards, flip both the
    // robot's heading and the target heading by 180 degrees so the math is
    // identical to the forwards case, then negate the output at the end
    let targetTheta = degToRad(90 - theta);
    if (!forwards) targetTheta += Math.PI;
    const target = new Pose(x, y, targetTheta);

    const startTime = this.clock.millis();
    let lastPose = this.getPose();
    let prevLateralOut = 0;
    let close = false;
    this.distTraveled = 0;

    while (
      this.clock.millis() - startTime < timeout &&
      !this.lateralSmallExit.getExit() && !this.lateralLargeExit.getExit() &&
      this.motionRunning
    ) {
      const pose = this.getPose(true);
      if (!forwards) pose.theta += Math.PI;
      this.distTraveled += pose.distance(lastPose);
      lastPose = this.getPose(true);

      const distToTarget = pose.distance(target);
      if (!close && distToTarget < CLOSE_RANGE) {
        close = true;
        maxSpeed = Math.max(Math.abs(prevLateralOut), CLOSE_MIN_CAP);
      }

      // the carrot point sits behind the target along the target heading,
      // pulling the robot into an arc that arrives at the right angle.
      // Once close, chase the target itself
      const carrot = close
        ? target
        : target.sub(
            new Pose(Math.cos(targetTheta), Math.sin(targetTheta)).scale(lead * distToTarget),
          );

      // far away: aim at the carrot. Close: settle into the target heading
      const angularError = close
        ? angleError(targetTheta, pose.theta, true)
        : angleError(pose.angle(carrot), pose.theta, true);
      const lateralError =
        pose.distance(carrot) * Math.cos(angleError(pose.angle(carrot), pose.theta, true));

      this.lateralSmallExit.update(lateralError);
      this.lateralLargeExit.update(lateralError);

      let lateralOut = this.lateralPID.update(lateralError);
      lateralOut = clamp(lateralOut, -maxSpeed, maxSpeed);
      lateralOut = slew(lateralOut, prevLateralOut, this.lateralSettings.slew);

      // limit speed through tight arcs so the drivetrain doesn't drift
      if (!close) {
        const curvature = Math.abs(getCurvature(pose, carrot));
        if (curvature !== 0) {
          const radius = 1 / curvature;
          const maxSlipSpeed = Math.sqrt(this.drivetrain.horizontalDrift * radius * 9.8);
          lateralOut = clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);
        }
      }
      if (!close && minSpeed !== 0 && Math.abs(lateralOut) < minSpeed)
        lateralOut = minSpeed * sgn(lateralOut);
      prevLateralOut = lateralOut;

      if (minSpeed !== 0 && distToTarget < earlyExitRange) break;

      let angularOut = this.angularPID.update(radToDeg(angularError));
      angularOut = clamp(angularOut, -maxSpeed, maxSpeed);

      // undo the 180 degree flip for backwards motion
      const lateralCmd = forwards ? lateralOut : -lateralOut;

      let leftPower = lateralCmd - angularOut;
      let rightPower = lateralCmd + angularOut;
      const ratio = Math.max(Math.abs(leftPower), Math.abs(rightPower)) / maxSpeed;
      if (ratio > 1) {
        leftPower /= ratio;
        rightPower /= ratio;
      }
      this.drivetrain.leftMotors.move(leftPower);
      this.drivetrain.rightMotors.move(rightPower);
      await this.clock.delay(10);
    }

    this.stopDrive();
    this.distTraveled = -1;
    this.endMotion();
  }

  /** Follow a path with the pure pursuit algorithm */
  async follow(
    path: Waypoint[], lookahead: number, timeout: number, forwards = true, async = true,
  ): Promise<void> {
    if (path.length < 2) return;

    await this.requestMotionStart();
    if (!this.motionRunning) return;
    if (async) {
      spawnTask(() => this.follow(path, lookahead, timeout, forwards, false));
      this.endMotion();
      await this.clock.delay(10);
      return;
    }

    const startTime = this.clock.millis();
    let lastPose = this.getPose();
    let lookaheadPoint = path[0];
    let lookaheadIndex = 0;
    let closestIndex = 0;
    this.distTraveled = 0;

    while (this.clock.millis() - startTime < timeout && this.motionRunning) {
      const pose = this.getPose(true); // math radians
      if (!forwards) pose.theta += Math.PI;
      this.distTraveled += pose.distance(lastPose);
      lastPose = this.getPose(true);

      // end the motion once the nearest path point is the final one
      closestIndex = findClosest(pose, path, closestIndex);
      if (closestIndex === path.length - 1) break;

      // search forward along the path for the furthest lookahead
      // intersection. If the circle misses (sharp corner), keep the
      // previous lookahead point so the robot keeps moving
      for (let i = lookaheadIndex; i < path.length - 1; i++) {
        const t = circleIntersect(path[i], path[i + 1], pose, lookahead);
        if (t !== -1) {
          lookaheadPoint = new Waypoint(
            path[i].x + (path[i + 1].x - path[i].x) * t,
            path[i].y + (path[i + 1].y - path[i].y) * t,
            path[i + 1].speed,
          );
          lookaheadIndex = i;
        }
      }

      // signed curvature of the arc to the lookahead point (CCW positive)
      const curvature = getCurvature(pose, lookaheadPoint);

      // speed comes from the path, so deceleration is baked into the file
      let targetVel = path[closestIndex].speed;
      if (!forwards) targetVel = -targetVel;

      // arc kinematics: v_left/right = v * (2 -/+ curvature * trackWidth) / 2
      let leftPower = (targetVel * (2 - curvature * this.drivetrain.trackWidth)) / 2;
      let rightPower = (targetVel * (2 + curvature * this.drivetrain.trackWidth)) / 2;
      const ratio = Math.max(Math.abs(leftPower), Math.abs(rightPower)) / 127;
      if (ratio > 1) {
        leftPower /= ratio;
        rightPower /= ratio;
      }
      this.drivetrain.leftMotors.move(leftPower);
      this.drivetrain.rightMotors.move(rightPower);
      await this.clock.delay(10);
    }

    this.stopDrive();
    this.distTraveled = -1;
    this.endMotion();
  }

  // --------------------------------------------------------------------
  // driver control
  // --------------------------------------------------------------------

  /** Tank drive: independent left/right inputs, -127 to 127 */
  tank(left: number, right: number, disableDriveCurve = false): void {
    const leftPower = disableDriveCurve ? left : this.throttleCurve.curve(left);
    const rightPower = disableDriveCurve ? right : this.throttleCurve.curve(right);
    this.drivetrain.leftMotors.move(leftPower);
    this.drivetrain.rightMotors.move(rightPower);
  }

  /** Arcade drive: throttle + turn (positive = right) */
  arcade(throttle: number, turn: number, disableDriveCurve = false): void {
    const t = disableDriveCurve ? throttle : this.throttleCurve.curve(throttle);
    const s = disableDriveCurve ? turn : this.steerCurve.curve(turn);

    let leftPower = t + s;
    let rightPower = t - s;
    // desaturate so turning authority is preserved at full throttle
    const ratio = Math.max(Math.abs(leftPower), Math.abs(rightPower)) / 127;
    if (ratio > 1) {
      leftPower /= ratio;
      rightPower /= ratio;
    }
    this.drivetrain.leftMotors.move(leftPower);
    this.drivetrain.rightMotors.move(rightPower);
  }

  /** Curvature drive: the turn input bends the path instead of setting a
   *  turn rate, so a given stick deflection feels the same at any speed */
  curvature(throttle: number, turn: number, disableDriveCurve = false): void {
    const t = disableDriveCurve ? throttle : this.throttleCurve.curve(throttle);
    const s = disableDriveCurve ? turn : this.steerCurve.curve(turn);

    // at zero throttle, fall back to turning in place
    if (t === 0) {
      this.drivetrain.leftMotors.move(s);
      this.drivetrain.rightMotors.move(-s);
      return;
    }

    let leftPower = t + (Math.abs(t) * s) / 127;
    let rightPower = t - (Math.abs(t) * s) / 127;
    const ratio = Math.max(Math.abs(leftPower), Math.abs(rightPower)) / 127;
    if (ratio > 1) {
      leftPower /= ratio;
      rightPower /= ratio;
    }
    this.drivetrain.leftMotors.move(leftPower);
    this.drivetrain.rightMotors.move(rightPower);
  }
}
