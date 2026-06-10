// Headless verification that the ported library drives the simulated robot
// to its targets. Run with: node tests/smoke.ts

import { Waypoint } from '../src/lib/chassis.ts';
import { World, defaultConfig } from '../src/sim/world.ts';

let failures = 0;

function check(name: string, actual: number, expected: number, tolerance: number): void {
  const ok = Math.abs(actual - expected) <= tolerance;
  if (!ok) failures++;
  console.log(
    `${ok ? 'PASS' : 'FAIL'}  ${name}: ${actual.toFixed(2)} (want ${expected} ± ${tolerance})`,
  );
}

function angleDiff(a: number, b: number): number {
  let d = (a - b) % 360;
  if (d > 180) d -= 360;
  if (d < -180) d += 360;
  return d;
}

/** run an autonomous program, ticking the world until it finishes */
async function run(world: World, program: () => Promise<void>): Promise<number> {
  let done = false;
  let error: unknown = null;
  program().then(
    () => (done = true),
    (e) => {
      error = e;
      done = true;
    },
  );
  let ticks = 0;
  while (!done && ticks < 30000) {
    await world.step();
    ticks++;
  }
  if (error) throw error;
  if (!done) throw new Error('program did not finish within 5 sim minutes');
  return ticks;
}

// ---------------------------------------------------------------------------
// ideal sensors: odometry should match ground truth almost exactly
// ---------------------------------------------------------------------------
{
  const world = new World(defaultConfig());
  const chassis = world.chassis;

  await run(world, async () => {
    chassis.setPose(0, 0, 0);

    await chassis.moveToPoint(0, 24, 4000, {}, false);
    check('moveToPoint y', chassis.getPose().y, 24, 1.5);
    check('moveToPoint x', chassis.getPose().x, 0, 1.5);

    await chassis.turnToHeading(90, 3000, {}, false);
    check('turnToHeading 90', angleDiff(chassis.getPose().theta, 90), 0, 2);

    await chassis.moveToPose(24, 48, 0, 6000, { lead: 0.6 }, false);
    check('moveToPose x', chassis.getPose().x, 24, 2.5);
    check('moveToPose y', chassis.getPose().y, 48, 2.5);
    check('moveToPose heading', angleDiff(chassis.getPose().theta, 0), 0, 8);

    await chassis.swingToHeading(90, 'left', 3000, {}, false);
    check('swingToHeading 90', angleDiff(chassis.getPose().theta, 90), 0, 3);

    await chassis.turnToPoint(0, 0, 3000, {}, false);
    const toOrigin = 90 - (Math.atan2(0 - chassis.getPose().y, 0 - chassis.getPose().x) * 180) / Math.PI;
    check('turnToPoint heading', angleDiff(chassis.getPose().theta, toOrigin), 0, 3);
  });

  // odometry vs ground truth with ideal sensors (centidegree quantization only)
  const odom = world.getOdomPose();
  const truth = world.getTruePose();
  check('odom x error (ideal)', odom.x - truth.x, 0, 0.5);
  check('odom y error (ideal)', odom.y - truth.y, 0, 0.5);
  check('odom heading error (ideal)', angleDiff(odom.theta, truth.theta), 0, 1);
}

// ---------------------------------------------------------------------------
// async motions + waitUntil/waitUntilDone, the C++ usage idiom
// ---------------------------------------------------------------------------
{
  const world = new World(defaultConfig());
  const chassis = world.chassis;

  await run(world, async () => {
    chassis.setPose(0, 0, 0);
    await chassis.moveToPoint(0, 36, 5000); // async: returns once started
    await chassis.waitUntil(18);
    const midway = chassis.getPose().y;
    check('waitUntil fires midway', midway > 12 && midway < 30 ? 1 : 0, 1, 0);
    await chassis.waitUntilDone();
    check('async moveToPoint y', chassis.getPose().y, 36, 1.5);
  });
}

// ---------------------------------------------------------------------------
// pure pursuit
// ---------------------------------------------------------------------------
{
  const world = new World(defaultConfig());
  const chassis = world.chassis;

  await run(world, async () => {
    chassis.setPose(0, 0, 0);
    const path = [
      new Waypoint(0, 0, 80),
      new Waypoint(0, 18, 80),
      new Waypoint(8, 32, 70),
      new Waypoint(24, 40, 60),
      new Waypoint(32, 40, 50),
      new Waypoint(38, 40, 40),
      new Waypoint(40, 40, 30),
    ];
    await chassis.follow(path, 12, 10000, true, false);
    check('pure pursuit end x', chassis.getPose().x, 40, 6);
    check('pure pursuit end y', chassis.getPose().y, 40, 6);
  });
}

// ---------------------------------------------------------------------------
// noisy sensors: odometry should drift, but only a little
// ---------------------------------------------------------------------------
{
  const config = defaultConfig();
  config.noise.enabled = true;
  const world = new World(config);
  const chassis = world.chassis;

  await run(world, async () => {
    chassis.setPose(0, 0, 0);
    await chassis.moveToPoint(0, 36, 5000, {}, false);
    await chassis.turnToHeading(90, 3000, {}, false);
    await chassis.moveToPoint(36, 36, 5000, {}, false);
  });

  const odom = world.getOdomPose();
  const truth = world.getTruePose();
  const drift = Math.hypot(odom.x - truth.x, odom.y - truth.y);
  console.log(`INFO  odom drift with noise: ${drift.toFixed(2)}"`);
  check('noisy odom drift bounded', drift, 0, 4);
}

console.log(failures === 0 ? '\nall checks passed' : `\n${failures} check(s) FAILED`);
process.exit(failures === 0 ? 0 : 1);
