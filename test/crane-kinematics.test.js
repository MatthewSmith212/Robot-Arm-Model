import test from 'node:test';
import assert from 'node:assert/strict';
import {
  ARM,
  BENCH,
  JOINTS,
  REACH,
  SHOULDER_Z,
  WORK_BAND,
  clampAngles,
  jacobian,
  onBench,
  overBench,
  reachOnBench,
  solveForward,
  solveInverse,
  solveOnBench,
  withinLimits,
  wrapDegrees,
} from '../src/crane-kinematics.js';

const near = (actual, expected, tolerance = 1e-9) => {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} is not within ${tolerance} of ${expected}`);
};
const positions = ({ tip }) => [tip.x, tip.y, tip.z];
const home = () => JOINTS.map(({ home: h }) => h);

test('three joints: one vertical yaw axis and two parallel horizontal hinges', () => {
  assert.equal(JOINTS.length, 3);
  assert.match(JOINTS[0].axis, /VERTICAL/);
  assert.match(JOINTS[1].axis, /HORIZONTAL/);
  assert.match(JOINTS[2].axis, /HORIZONTAL/);
  JOINTS.forEach(({ min, max, home: h }) => {
    assert.ok(min < max);
    assert.ok(h >= min && h <= max);
  });
});

test('the shoulder hinge stands above the bench on a short mast', () => {
  near(SHOULDER_Z, BENCH.height + ARM.mast);
  assert.ok(ARM.mast > 0.1 && ARM.mast < 0.3, 'the mast is what makes the sketch buildable');
  assert.ok(ARM.upperArm < ARM.forearm, 'the sketch draws a shorter upper arm than forearm');
  near(REACH.max, ARM.upperArm + ARM.forearm);
  near(REACH.min, ARM.forearm - ARM.upperArm);
});

test('the home pose is the A-frame from the sketch, with the tool on the bench', () => {
  const pose = solveForward(home());
  assert.ok(onBench(pose.tip.z), 'the tool point rests on the bench top');
  assert.ok(pose.elbowAboveBench > 0.2, 'the elbow is thrown well above the bench');
  assert.ok(pose.forearmElevationDeg < -30, 'the forearm comes back down to the surface');
  assert.ok(pose.forearmElevationDeg > -90, 'and is still angled down a slope, not vertical');
});

test('yaw turns the arm about the vertical axis without changing height or radius', () => {
  const base = solveForward([0, 40, 90]);
  const turned = solveForward([90, 40, 90]);
  near(turned.radial, base.radial, 1e-12);
  near(turned.tip.z, base.tip.z, 1e-12);
  near(turned.elbow.z, base.elbow.z, 1e-12);
  near(turned.tip.x, 0, 1e-12);
  near(turned.tip.y, base.radial, 1e-12);
});

test('the two hinges fold the arm from stretched to fully folded', () => {
  const straightOut = solveForward([0, 0, 0]);
  near(straightOut.tip.x, ARM.upperArm + ARM.forearm, 1e-12);
  near(straightOut.tip.z, SHOULDER_Z, 1e-12);

  const straightUp = solveForward([0, 90, 0]);
  near(straightUp.tip.z, SHOULDER_Z + REACH.max, 1e-12);
  near(straightUp.radial, 0, 1e-12);

  const folded = solveForward([0, 0, 180]);
  near(folded.tip.x, ARM.upperArm - ARM.forearm, 1e-12);
  near(Math.abs(folded.forearmElevationDeg), 180, 1e-9);
});

test('the shoulder hinge raises the elbow and the elbow hinge decides where the tool ends up', () => {
  const low = solveForward([0, 20, 60]);
  const high = solveForward([0, 60, 120]);
  assert.ok(high.elbowAboveBench > low.elbowAboveBench, 'more shoulder pitch throws the elbow up');
  // The forearm sits at θ₂ − θ₃ from horizontal, so it is the elbow hinge that
  // decides whether the tool points up, out, or down at the bench.
  const toolHeights = [0, 60, 120, 170].map((t3) => solveForward([0, 60, t3]).tip.z);
  for (let i = 1; i < toolHeights.length; i += 1) {
    assert.ok(toolHeights[i] < toolHeights[i - 1], `elbow pitch ${[0, 60, 120, 170][i]}° should lower the tool`);
  }
  assert.ok(toolHeights.at(-1) < BENCH.height, 'a folded elbow puts the tool under the bench');
});

test('the analytic Jacobian matches finite differences of the tool point', () => {
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

test('the Jacobian collapses when the arm is fully stretched or folded', () => {
  near(jacobian([0, 0, 0]).determinant, 0, 1e-9);
  near(jacobian([0, 90, 0]).determinant, 0, 1e-9);
  assert.ok(jacobian(home()).dexterity > 0.01, 'the working pose stays dexterous');
});

test('inverse kinematics recovers a random forward pose on the bench', () => {
  const angles = [-64, 52, 118];
  const pose = solveForward(angles);
  const solution = solveInverse(pose.tip);
  assert.equal(solution.ok, true);
  near(solution.tip.x, pose.tip.x, 1e-9);
  near(solution.tip.y, pose.tip.y, 1e-9);
  near(solution.tip.z, pose.tip.z, 1e-9);
  near(solution.angles[0], angles[0], 1e-7);
  near(solution.angles[1], angles[1], 1e-7);
  near(solution.angles[2], angles[2], 1e-7);
});

test('every goal on the bench band is solvable and lands on the bench top', () => {
  for (const radius of [0.25, 0.4, 0.55, 0.7, 0.82]) {
    for (const bearing of [-120, -40, 0, 35, 90, 145]) {
      const goal = { x: radius * Math.cos(bearing * Math.PI / 180), y: radius * Math.sin(bearing * Math.PI / 180) };
      const solution = solveOnBench(goal.x, goal.y);
      assert.equal(solution.ok, true, `goal at ${radius} m / ${bearing}° should solve`);
      assert.equal(solution.clamped, false, `goal at ${radius} m / ${bearing}° should be reachable as asked`);
      near(solution.tip.x, goal.x, 1e-9);
      near(solution.tip.y, goal.y, 1e-9);
      near(solution.tip.z, BENCH.height, 1e-9);
      assert.ok(solution.elbowZ > SHOULDER_Z - 0.05, 'the elbow stays up in the crane configuration');
    }
  }
});

test('goals outside the bench or beyond the arm are clamped, not refused', () => {
  const far = solveInverse({ x: 5, y: 0, z: BENCH.height });
  assert.equal(far.ok, false);
  assert.match(far.reason, /beyond the arm’s reach|travel/);

  const clamped = solveInverse({ x: 5, y: 0, z: BENCH.height }, { clamp: true });
  assert.equal(clamped.ok, true);
  assert.equal(clamped.clamped, true);
  assert.ok(clamped.reach <= REACH.max + 1e-9);

  // Beyond the yaw travel the base stops at its limit.
  const behind = solveOnBench(-0.5, -0.088, { clamp: true });
  assert.equal(behind.ok, true);
  assert.equal(behind.clamped, true);
  assert.ok(withinLimits(behind.angles));
  assert.ok(behind.angles[0] >= JOINTS[0].min && behind.angles[0] <= JOINTS[0].max);

  // Inside the mast the tool would collide with the turret, so it is pushed out.
  const inside = solveOnBench(0.05, 0, { clamp: true });
  assert.equal(inside.ok, true);
  assert.equal(inside.clamped, true);
  near(inside.radial, WORK_BAND.inner, 1e-6);
});

test('a surface above the shoulder or below the stretched arm is reported as unreachable', () => {
  assert.equal(reachOnBench(SHOULDER_Z + 2).reachable, false);
  assert.match(reachOnBench(SHOULDER_Z + 2).reason, /higher/);
  const below = reachOnBench(0.02);
  assert.equal(below.reachable, false);
  assert.match(below.reason, /lower/);
  assert.equal(reachOnBench(BENCH.height).reachable, true);

  const band = reachOnBench(BENCH.height);
  assert.ok(band.outer > 0.8 && band.outer < REACH.max);
  assert.equal(band.inner, 0, 'geometrically the arm can fold the tool onto the axis');
  // Just under the shoulder never needs the tool past the joint travel.
  for (const z of [BENCH.height, 0.8, 0.86, 0.9, 0.94]) {
    const solution = solveInverse({ x: 0.4, y: 0.1, z }, { clamp: true });
    assert.equal(solution.ok, true, `z ${z} should be solvable when clamped`);
    assert.equal(solution.clamped, false, `z ${z} should not need clamping`);
    near(solution.tip.z, z, 1e-9);
  }
});

test('joint clamping and bench helpers behave', () => {
  const limited = clampAngles([200, -90, 400]);
  assert.deepEqual(limited.angles, [JOINTS[0].max, JOINTS[1].min, JOINTS[2].max]);
  assert.equal(limited.clamped, true);

  assert.equal(onBench(BENCH.height + 0.002), true);
  assert.equal(onBench(BENCH.height + 0.05), false);
  assert.equal(overBench(BENCH.center.x, BENCH.center.y), true);
  assert.equal(overBench(10, 10), false);
  near(wrapDegrees(190), -170);
});

test('invalid requests are rejected', () => {
  assert.throws(() => solveForward([0, 0]), /Expected 3 joint angles/);
  assert.throws(() => solveForward([0, 0, Number.NaN]), /finite numbers/);
  assert.throws(() => solveInverse({ x: 0, y: 0, z: Number.POSITIVE_INFINITY }), /finite/);
});
