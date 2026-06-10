// Direct port of odyssey/pose.cpp

export class Pose {
  x: number;
  y: number;
  theta: number;

  constructor(x = 0, y = 0, theta = 0) {
    this.x = x;
    this.y = y;
    this.theta = theta;
  }

  add(other: Pose): Pose {
    return new Pose(this.x + other.x, this.y + other.y, this.theta);
  }

  sub(other: Pose): Pose {
    return new Pose(this.x - other.x, this.y - other.y, this.theta);
  }

  /** dot product */
  dot(other: Pose): number {
    return this.x * other.x + this.y * other.y;
  }

  scale(scalar: number): Pose {
    return new Pose(this.x * scalar, this.y * scalar, this.theta);
  }

  lerp(other: Pose, t: number): Pose {
    return new Pose(this.x + (other.x - this.x) * t, this.y + (other.y - this.y) * t, this.theta);
  }

  distance(other: { x: number; y: number }): number {
    return Math.hypot(other.x - this.x, other.y - this.y);
  }

  /** angle to another point, math radians */
  angle(other: { x: number; y: number }): number {
    return Math.atan2(other.y - this.y, other.x - this.x);
  }

  rotate(angle: number): Pose {
    const cosA = Math.cos(angle);
    const sinA = Math.sin(angle);
    return new Pose(this.x * cosA - this.y * sinA, this.x * sinA + this.y * cosA, this.theta);
  }

  clone(): Pose {
    return new Pose(this.x, this.y, this.theta);
  }
}
