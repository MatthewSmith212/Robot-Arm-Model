import test from 'node:test';
import assert from 'node:assert/strict';
import {
  ARM,
  ELBOW_TO_TIP,
  JOINTS,
  PLANES,
  REACH,
  jacobian,
  reachOnPlane,
  solveForward,
  solveInverse,
  solveOnPlane,
  touchingPlane,
  wrapDegrees,
} from '../src/spatial-kinematics.js';

const near = (actual, expected, tolerance = 1e-9) => {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} is not within ${tolerance} of ${expected}`);
};
const positions = ({ tip }) => [tip.x, tip.y, tip.z];

test('three joints: two at the shoulder, one at the elbow', () => {
  assert.equal(JOINTS.length, 3);
  assert.match(JOINTS[0].axis, /VERTICAL/);
  assert.match(JOINTS[1].axis, /HORIZONTAL/);
  assert.match(JOINTS[2].axis, /HORIZONTAL/);
  JOINTS.forEach(({ min, max, home }) => {
    assert.ok(min < max);
    assert.ok(home >= min && home <= max);
  });
});

test('the lower plane sits below the mounting plane and the shoulder', () => {
  assert.ok(PLANES.mount.height > PLANES.lower.height);
  assert.ok(ARM.shoulderZ > PLANES.mount.height);
  near(ARM.shoulderZ - PLANES.mount.height, ARM.pedestal);
});

test('zero pitch and elbow hang the arm straight down from the shoulder', () => {
  const straightUp = solveForward([0, 90, 0]);
  near(straightUp.tip.x, 0, 1e-12);
  near(straightUp.tip.z, ARM.shoulderZ + ARM.upperArm + ELBOW_TO_TIP);

  const hanging = solveForward([0, 0, 90]);
  near(hanging.elbow.z, ARM.shoulderZ);
  near(hanging.tip.z, ARM.shoulderZ - ELBOW_TO_TIP, 1e-12);
  near(hanging.forearmElevationDeg, -90);
});

test('shoulder yaw turns the arm about the vertical axis without changing height', () => {
  const base = solveForward([0, 43, 110]);
  const turned = solveForward([90, 43, 110]);
  near(Math.hypot(base.tip.x, base.tip.y), Math.hypot(turned.tip.x, turned.tip.y), 1e-12);
  near(turned.tip.x, 0, 1e-12);
  near(turned.tip.z, base.tip.z, 1e-12);
  near(turned.elbow.z, base.elbow.z, 1e-12);
});

test('the home pose throws the elbow into the air above the shoulder', () => {
  const home = JOINTS.map(({ home: h }) => h);
  const pose = solveForward(home);
  assert.ok(pose.elbowRise > 0.1, 'elbow must sit above the shoulder pitch axis');
  assert.ok(pose.elbowZ > PLANES.mount.height, 'elbow must clear the mounting plane');
  assert.ok(pose.tip.z < PLANES.mount.height, 'the tool hangs below the mounting plane');
  assert.equal(touchingPlane(pose.tip.z)?.key, 'lower', 'the home tool rests on the lower plane');
});

test('pitch and elbow work together to reach down', () => {
  const raised = solveForward([30, 60, 120]);
  const lowered = solveForward([30, 25, 60]);
  assert.ok(raised.elbowRise > lowered.elbowRise, 'more shoulder pitch throws the elbow higher');
  assert.ok(raised.tip.z < lowered.tip.z, 'with the elbow high, the elbow joint aims the tool lower');
  assert.ok(raised.elbowZ > PLANES.mount.height);
});

test('the analytic Jacobian matches finite differences of the tip position', () => {
  const angles = [-32, 58, 96];
  const { matrix } = jacobian(angles);
  const step = 1e-6;
  for (let joint = 0; joint < 3; joint += 1) {
    const bumped = angles.slice();
    bumped[joint] += step;
    const moved = positions(solveForward(bumped));
    const base = positions(solveForward(angles));
    for (let axis = 0; axis < 3; axis += 1) {
      const slope = (moved[axis] - base[axis]) / (step * Math.PI / 180);
      near(matrix[axis][joint], slope, 1e-4);
    }
  }
});

test('the Jacobian collapses when the arm is fully stretched', () => {
  const stretched = solveForward([0, 90, 0]);
  near(jacobian([0, 90, 0]).determinant, 0, 1e-9);
  assert.ok(jacobian([-56, 43, 110]).dexterity > 0.01, 'the working pose stays dexterous');
  assert.ok(stretched.reach > REACH.max - 1e-9);
});

test('inverse kinematics recovers a random forward pose', () => {
  const angles = [-64, 52, 118];
  const pose = solveForward(angles);
  const solution = solveInverse(pose.tip, { elbow: 'up' });
  assert.equal(solution.ok, true);
  near(solution.tip.x, pose.tip.x, 1e-9);
  near(solution.tip.y, pose.tip.y, 1e-9);
  near(solution.tip.z, pose.tip.z, 1e-9);
  near(solution.angles[0], angles[0], 1e-7);
  near(solution.angles[1], angles[1], 1e-7);
  near(solution.angles[2], angles[2], 1e-7);
});

test('inverse kinematics reaches a tool target on the lower plane', () => {
  const goal = { x: 0.42, y: -0.12, z: PLANES.lower.height };
  const solution = solveInverse(goal, { elbow: 'up' });
  assert.equal(solution.ok, true);
  assert.equal(solution.elbow, 'up');
  near(solution.tip.x, goal.x, 1e-9);
  near(solution.tip.y, goal.y, 1e-9);
  near(solution.tip.z, goal.z, 1e-9);
  assert.ok(solution.elbowZ > PLANES.mount.height, 'elbow stays above the mounting plane');
});

test('plane solving targets the plane height and reports contact', () => {
  for (const key of ['mount', 'lower']) {
    const solution = solveOnPlane(key, 0.5, 0.2);
    assert.equal(solution.ok, true);
    assert.equal(touchingPlane(solution.tip.z)?.key, key);
  }
});

test('targets outside the workspace are rejected or clamped on request', () => {
  const far = solveInverse({ x: 5, y: 0, z: 0 });
  assert.equal(far.ok, false);
  assert.match(far.reason, /beyond the arm’s reach/);

  const clamped = solveInverse({ x: 5, y: 0, z: 0 }, { clamp: true });
  assert.equal(clamped.ok, true);
  assert.equal(clamped.clamped, true);
  near(clamped.reach, REACH.max, 1e-9);

  const tooClose = solveInverse({ x: 0, y: 0, z: ARM.shoulderZ });
  assert.equal(tooClose.ok, false);
  assert.match(tooClose.reason, /closer than the arm can fold/);
});

test('each plane has its own reachable annulus', () => {
  const mount = reachOnPlane(PLANES.mount.height);
  const lower = reachOnPlane(PLANES.lower.height);
  assert.ok(mount.outer > 0 && lower.outer > 0);
  // The mounting plane is only just below the shoulder, so it reaches furthest;
  // the lower plane is further down and loses horizontal reach, but the arm can
  // hang past the yaw axis, so its inner limit closes to zero.
  assert.ok(mount.outer > lower.outer);
  assert.ok(mount.inner > 0);
  assert.equal(lower.inner, 0);
});

test('invalid requests are rejected', () => {
  assert.throws(() => solveForward([0, 0]), /Expected 3 joint angles/);
  assert.throws(() => solveForward([0, 0, Number.NaN]), /finite numbers/);
  assert.throws(() => solveInverse({ x: 0, y: 0, z: Number.POSITIVE_INFINITY }), /finite/);
  assert.throws(() => solveOnPlane('ceiling', 0, 0), /Unknown plane/);
  near(wrapDegrees(190), -170);
});
