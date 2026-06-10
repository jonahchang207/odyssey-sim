// Direct port of odyssey/odometry.cpp
//
// Arc-based odometry, as described in the 5225A "Introduction to Position
// Tracking" document and used (in spirit) by LemLib.
//
// Internal conventions:
//  - theta is in standard math radians: 0 = +x, counterclockwise positive
//  - user-facing theta is compass degrees: 0 = +y, clockwise positive
//  - conversion: theta_compass = 90 - deg(theta_math)

import { Pose } from './pose.ts';
import { avg, degToRad, radToDeg } from './util.ts';
import type { TrackingWheel } from './trackingWheel.ts';
import type { SimImu } from '../sim/devices.ts';
import { spawnTask, type SimClock } from '../sim/clock.ts';

export interface OdomSensors {
  vertical1: TrackingWheel | null;
  vertical2: TrackingWheel | null;
  horizontal1: TrackingWheel | null;
  horizontal2: TrackingWheel | null;
  imu: SimImu | null;
}

export class Odometry {
  private sensors: OdomSensors = {
    vertical1: null,
    vertical2: null,
    horizontal1: null,
    horizontal2: null,
    imu: null,
  };
  // default pose: origin, facing compass 0 (+y), which is pi/2 in math radians
  private pose = new Pose(0, 0, Math.PI / 2);

  private prevVertical1 = 0;
  private prevVertical2 = 0;
  private prevHorizontal1 = 0;
  private prevHorizontal2 = 0;
  private prevImuRotation = 0;

  private clock: SimClock;
  private running = false;

  constructor(clock: SimClock) {
    this.clock = clock;
  }

  setSensors(sensors: OdomSensors): void {
    this.sensors = sensors;
  }

  getPose(radians = false): Pose {
    const pose = this.pose;
    if (radians) return pose.clone();
    return new Pose(pose.x, pose.y, 90 - radToDeg(pose.theta));
  }

  setPose(pose: Pose, radians = false): void {
    const theta = radians ? pose.theta : degToRad(90 - pose.theta);
    this.pose = new Pose(pose.x, pose.y, theta);
  }

  /** re-read all sensors so the next update starts from a clean baseline */
  private resetBaselines(): void {
    const s = this.sensors;
    this.prevVertical1 = s.vertical1 ? s.vertical1.getDistanceTraveled() : 0;
    this.prevVertical2 = s.vertical2 ? s.vertical2.getDistanceTraveled() : 0;
    this.prevHorizontal1 = s.horizontal1 ? s.horizontal1.getDistanceTraveled() : 0;
    this.prevHorizontal2 = s.horizontal2 ? s.horizontal2.getDistanceTraveled() : 0;
    if (s.imu) {
      const rotation = s.imu.get_rotation();
      this.prevImuRotation = Number.isFinite(rotation) ? rotation : 0;
    }
  }

  update(): void {
    const s = this.sensors;
    // 1. read sensors and compute deltas
    const vertical1 = s.vertical1 ? s.vertical1.getDistanceTraveled() : 0;
    const vertical2 = s.vertical2 ? s.vertical2.getDistanceTraveled() : 0;
    const horizontal1 = s.horizontal1 ? s.horizontal1.getDistanceTraveled() : 0;
    const horizontal2 = s.horizontal2 ? s.horizontal2.getDistanceTraveled() : 0;

    const deltaVertical1 = vertical1 - this.prevVertical1;
    const deltaVertical2 = vertical2 - this.prevVertical2;
    const deltaHorizontal1 = horizontal1 - this.prevHorizontal1;
    const deltaHorizontal2 = horizontal2 - this.prevHorizontal2;
    this.prevVertical1 = vertical1;
    this.prevVertical2 = vertical2;
    this.prevHorizontal1 = horizontal1;
    this.prevHorizontal2 = horizontal2;

    // 2. heading change. IMU preferred; parallel wheel difference as fallback
    let deltaTheta = 0;
    let headingFound = false;
    if (s.imu) {
      const imuRotation = s.imu.get_rotation();
      if (Number.isFinite(imuRotation)) {
        // IMU rotation is clockwise positive; internal theta is CCW positive
        deltaTheta = -degToRad(imuRotation - this.prevImuRotation);
        this.prevImuRotation = imuRotation;
        headingFound = true;
      }
    }
    if (!headingFound && s.vertical1 && s.vertical2) {
      const offsetDiff = s.vertical1.getOffset() - s.vertical2.getOffset();
      if (offsetDiff !== 0) {
        deltaTheta = (deltaVertical1 - deltaVertical2) / offsetDiff;
        headingFound = true;
      }
    }
    if (!headingFound && s.horizontal1 && s.horizontal2) {
      const offsetDiff = s.horizontal1.getOffset() - s.horizontal2.getOffset();
      if (offsetDiff !== 0) {
        deltaTheta = (deltaHorizontal2 - deltaHorizontal1) / offsetDiff;
        headingFound = true;
      }
    }

    // 3. local translation arcs. A wheel's reading includes the distance it
    // swept while the robot rotated, so subtract the rotation component
    const forwardEstimates: number[] = [];
    if (s.vertical1)
      forwardEstimates.push(deltaVertical1 - s.vertical1.getOffset() * deltaTheta);
    if (s.vertical2)
      forwardEstimates.push(deltaVertical2 - s.vertical2.getOffset() * deltaTheta);
    const forwardArc = avg(forwardEstimates);

    const rightwardEstimates: number[] = [];
    if (s.horizontal1)
      rightwardEstimates.push(deltaHorizontal1 + s.horizontal1.getOffset() * deltaTheta);
    if (s.horizontal2)
      rightwardEstimates.push(deltaHorizontal2 + s.horizontal2.getOffset() * deltaTheta);
    const rightwardArc = avg(rightwardEstimates);

    // convert the arcs to straight-line chords
    let localY: number; // forward
    let localX: number; // rightward
    if (deltaTheta === 0) {
      localY = forwardArc;
      localX = rightwardArc;
    } else {
      const chordFactor = 2 * Math.sin(deltaTheta / 2);
      localY = chordFactor * (forwardArc / deltaTheta);
      localX = chordFactor * (rightwardArc / deltaTheta);
    }

    // 4. rotate the local chord into the field frame at the tick's average
    // heading and accumulate
    const avgTheta = this.pose.theta + deltaTheta / 2;
    this.pose.x += localY * Math.cos(avgTheta) + localX * Math.sin(avgTheta);
    this.pose.y += localY * Math.sin(avgTheta) - localX * Math.cos(avgTheta);
    this.pose.theta += deltaTheta;
  }

  init(): void {
    this.resetBaselines();
    if (!this.running) {
      this.running = true;
      spawnTask(async () => {
        while (true) {
          this.update();
          await this.clock.delay(10);
        }
      });
    }
  }
}
