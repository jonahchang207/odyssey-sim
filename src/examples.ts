// Example autonomous programs. The API mirrors the C++ library: motions are
// async by default, so either `await chassis.waitUntilDone()` like you would
// on the robot, or pass `false` as the last argument to block directly.

export interface Example {
  name: string;
  code: string;
}

export const examples: Example[] = [
  {
    name: 'First motions',
    code: `// Drive a square, turning in place at each corner.
// Coordinates are inches; headings are compass degrees
// (0 = up/+y, clockwise positive), exactly like the C++ API.
chassis.setPose(-24, -24, 0);

for (let i = 0; i < 4; i++) {
  chassis.moveToPoint(
    [-24, 24, 24, -24][i],
    [24, 24, -24, -24][i],
    4000
  );
  await chassis.waitUntilDone();

  chassis.turnToHeading(90 * (i + 1), 2000);
  await chassis.waitUntilDone();
}

console.log('square complete at', chassis.getPose().x.toFixed(1),
            chassis.getPose().y.toFixed(1));
`,
  },
  {
    name: 'Boomerang (moveToPose)',
    code: `// moveToPose controls the arrival heading with a "carrot point"
// that pulls the robot into an arc. Play with lead: larger = wider,
// smoother approach; smaller = more direct.
chassis.setPose(-48, -48, 45);

chassis.moveToPose(0, 0, 90, 5000, { lead: 0.6 });
await chassis.waitUntilDone();

chassis.moveToPose(48, 36, 0, 5000, { lead: 0.4 });
await chassis.waitUntilDone();

// drive backwards into a pose
chassis.moveToPose(24, -24, 180, 5000, { forwards: false, lead: 0.6 });
await chassis.waitUntilDone();
`,
  },
  {
    name: 'Pure pursuit',
    code: `// Follow a waypoint path. Each waypoint is (x, y, speed): speed
// shapes deceleration into tight sections. The lookahead (12 here)
// trades tracking tightness against smoothness.
chassis.setPose(-60, -60, 0);

const path = [
  new odyssey.Waypoint(-60, -60, 90),
  new odyssey.Waypoint(-60, -24, 90),
  new odyssey.Waypoint(-48, 0, 80),
  new odyssey.Waypoint(-24, 12, 70),
  new odyssey.Waypoint(0, 12, 70),
  new odyssey.Waypoint(24, 24, 70),
  new odyssey.Waypoint(48, 48, 60),
  new odyssey.Waypoint(56, 56, 40),
  new odyssey.Waypoint(60, 60, 30),
];

chassis.follow(path, 12, 15000);
await chassis.waitUntilDone();

console.log('path complete');
`,
  },
  {
    name: 'Chained motions (minSpeed)',
    code: `// minSpeed + earlyExitRange let motions flow into each other
// without settling, like a real skills route.
chassis.setPose(-48, -60, 0);

chassis.moveToPoint(-48, 0, 3000, { minSpeed: 70, earlyExitRange: 8 });
chassis.moveToPoint(0, 24, 3000, { minSpeed: 70, earlyExitRange: 8 });
chassis.moveToPose(48, 48, 90, 4000, { lead: 0.5 });
await chassis.waitUntilDone();

// swing turn: pivot on the locked right side
chassis.swingToHeading(180, 'right', 2000);
await chassis.waitUntilDone();

chassis.moveToPoint(48, 0, 3000);
await chassis.waitUntilDone();
`,
  },
  {
    name: 'waitUntil mid-motion',
    code: `// waitUntil blocks until the motion has traveled a distance —
// on a real robot this is where you'd raise an intake or score.
chassis.setPose(0, -60, 0);

chassis.moveToPoint(0, 48, 6000, { maxSpeed: 100 });

await chassis.waitUntil(36);
console.log('36" traveled — deploy intake here!');

await chassis.waitUntil(72);
console.log('72" traveled');

await chassis.waitUntilDone();
console.log('done at y =', chassis.getPose().y.toFixed(1));
`,
  },
];
