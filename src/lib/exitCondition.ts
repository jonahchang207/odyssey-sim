// Direct port of odyssey/exitcondition.cpp

import type { SimClock } from '../sim/clock.ts';

export class ExitCondition {
  range: number;
  time: number;

  private startTime = -1;
  private done = false;
  private clock: SimClock;

  constructor(range: number, time: number, clock: SimClock) {
    this.range = range;
    this.time = time;
    this.clock = clock;
  }

  getExit(): boolean {
    return this.done;
  }

  update(input: number): boolean {
    const currentTime = this.clock.millis();
    if (Math.abs(input) > this.range) {
      // outside the range: restart the timer
      this.startTime = -1;
    } else if (this.startTime === -1) {
      // just entered the range: start the timer
      this.startTime = currentTime;
    } else if (currentTime >= this.startTime + this.time) {
      // stayed in the range long enough
      this.done = true;
    }
    return this.done;
  }

  reset(): void {
    this.startTime = -1;
    this.done = false;
  }
}
