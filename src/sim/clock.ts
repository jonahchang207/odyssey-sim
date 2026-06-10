// Simulated time, replacing pros::millis / pros::delay / pros::Task.
//
// The engine advances sim time in 10ms ticks. Library code (odometry task,
// motion loops, user programs) awaits clock.delay(10) exactly where the C++
// calls pros::delay(10), so the control flow matches the real RTOS: every
// sleeping "task" wakes once per tick, runs to its next delay, and yields.

/** thrown into every pending delay when the simulation is stopped/reset */
export class ProgramStopped extends Error {
  constructor() {
    super('simulation stopped');
    this.name = 'ProgramStopped';
  }
}

interface PendingDelay {
  wakeAt: number;
  seq: number;
  resolve: () => void;
  reject: (err: Error) => void;
}

const flushQueue: (() => void)[] = [];
const channel = typeof MessageChannel !== 'undefined' ? new MessageChannel() : null;
if (channel) channel.port1.onmessage = () => flushQueue.shift()?.();

/**
 * Yield one macrotask so every microtask chain (i.e. every woken task) runs
 * to completion before the next physics step. MessageChannel avoids the
 * 4ms setTimeout clamp so fast-forward stays fast.
 */
export function flushTasks(): Promise<void> {
  return new Promise((resolve) => {
    if (channel) {
      flushQueue.push(resolve);
      channel.port2.postMessage(null);
    } else {
      setTimeout(resolve, 0);
    }
  });
}

export class SimClock {
  private now = 0;
  private seq = 0;
  private pending: PendingDelay[] = [];
  private stopped = false;

  millis(): number {
    return this.now;
  }

  /** resolves when sim time has advanced by ms */
  delay(ms: number): Promise<void> {
    if (this.stopped) return Promise.reject(new ProgramStopped());
    return new Promise((resolve, reject) => {
      this.pending.push({ wakeAt: this.now + ms, seq: this.seq++, resolve, reject });
    });
  }

  /** advance sim time and wake every delay that has come due */
  advance(ms: number): void {
    this.now += ms;
    const due = this.pending.filter((p) => p.wakeAt <= this.now);
    if (due.length === 0) return;
    this.pending = this.pending.filter((p) => p.wakeAt > this.now);
    // FIFO within a tick, like the RTOS round-robin
    due.sort((a, b) => a.wakeAt - b.wakeAt || a.seq - b.seq);
    for (const p of due) p.resolve();
  }

  /** reject everything that is sleeping so all task loops unwind */
  stop(): void {
    this.stopped = true;
    const all = this.pending;
    this.pending = [];
    for (const p of all) p.reject(new ProgramStopped());
  }
}

/**
 * Launch a background task (the pros::Task equivalent). ProgramStopped is
 * the normal way tasks die on reset, so it is swallowed here.
 */
export function spawnTask(fn: () => Promise<void>): void {
  fn().catch((err) => {
    if (!(err instanceof ProgramStopped)) throw err;
  });
}

/** pros::Mutex equivalent: FIFO, explicitly taken and given */
export class Mutex {
  private locked = false;
  private waiters: (() => void)[] = [];

  take(): Promise<void> {
    if (!this.locked) {
      this.locked = true;
      return Promise.resolve();
    }
    return new Promise((resolve) => this.waiters.push(resolve));
  }

  give(): void {
    const next = this.waiters.shift();
    if (next) next();
    else this.locked = false;
  }
}
