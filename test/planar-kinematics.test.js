import test from 'node:test';
import assert from 'node:assert/strict';
import {
  PLANAR_LINKS,
  PLANAR_LIMITS,
  TIP_WORKSPACE,
  WRIST_WORKSPACE,
  planarJacobian,
  planarPose,
  shortestAngleDelta,
  solvePlanarForward,
  solvePlanarInverse,
  solvePlanarInverseBoth,
  wrapDegrees,
} from '../src/planar-kinematics.js';

const near = (actual, expected, tolerance = 1e-9) => {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} is not within ${tolerance} of ${expected}`);
};
const [L1, L2, L3] = PLANAR_LINKS.map((link) => link.length);

test('the planar arm has three revolute degrees of freedom with usable limits', () => {
  assert.equal(PLANAR_LINKS.length, 3);
  assert.equal(PLANAR_LIMITS.length, 3);
  PLANAR_LIMITS.forEach(({ min, max, home }) => {
    assert.ok(min < max);
    assert.ok(home >= min && home <= max);
  });
});

test('a straight arm puts the tip on the +X axis at full reach', () => {
  const { tip, points, orientationDeg, reach } = solvePlanarForward([0, 0, 0]);
  near(tip.x, L1 + L2 + L3);
  near(tip.y, 0);
  near(points[1].x, L1);
  near(points[2].x, L1 + L2);
  near(reach, TIP_WORKSPACE.outerRadius);
  near(orientationDeg, 0);
});

test('folding the elbow 180° doubles the arm back toward the base', () => {
  const { tip, points } = solvePlanarForward([0, 180, 0]);
  near(points[2].x, L1 - L2);
  near(tip.x, L1 - L2 - L3);
  near(tip.y, 0, 1e-12);
});

test('a quarter turn of the base joint aims the whole arm straight up +Y', () => {
  const { tip, points } = solvePlanarForward([90, 0, 0]);
  near(tip.x, 0, 1e-12);
  near(tip.y, L1 + L2 + L3);
  near(points[1].y, L1);
});

test('the wrist joint sweeps the tool about a fixed wrist center', () => {
  const withoutWrist = solvePlanarForward([15, -40, 0]);
  const withWrist = solvePlanarForward([15, -40, 75]);
  const wrist = withoutWrist.points[2];
  near(withWrist.points[2].x, wrist.x, 1e-12);
  near(withWrist.points[2].y, wrist.y, 1e-12);
  near(withWrist.orientationDeg, 15 - 40 + 75);
  near(
    Math.hypot(withWrist.tip.x - wrist.x, withWrist.tip.y - wrist.y),
    PLANAR_LINKS[2].length,
    1e-12,
  );
});

test('the tip never leaves the mounting plane (it is a 2D model)', () => {
  const pose = planarPose([37, -63, 118]);
  assert.deepEqual(Object.keys(pose).sort(), ['phiDeg', 'reach', 'x', 'y']);
  near(Math.hypot(pose.x, pose.y), pose.reach);
});

test('absolute link angles are wrapped and callers keep sane numbers', () => {
  const { absoluteDegrees, orientationDeg } = solvePlanarForward([170, 170, 170]);
  near(absoluteDegrees[2], wrapDegrees(510));
  near(absoluteDegrees[2], 150);
  near(orientationDeg, 150);
  near(wrapDegrees(180), -180);
  near(wrapDegrees(-190), 170);
  near(wrapDegrees(360), 0);
});

test('the analytic Jacobian matches finite differences of the tip pose', () => {
  const angles = [27, -58, 39];
  const { matrix } = planarJacobian(angles);
  const step = 1e-6;
  for (let joint = 0; joint < 3; joint += 1) {
    const bumped = angles.slice();
    bumped[joint] += step;
    const displacement = planarPose(bumped);
    near(matrix[0][joint], (displacement.x - planarPose(angles).x) / (step * Math.PI / 180), 1e-4);
    near(matrix[1][joint], (displacement.y - planarPose(angles).y) / (step * Math.PI / 180), 1e-4);
    near(
      matrix[2][joint],
      (displacement.phiDeg - planarPose(angles).phiDeg) * Math.PI / 180 / (step * Math.PI / 180),
      1e-4,
    );
  }
});

test('the determinant vanishes at the fully stretched and fully folded poses', () => {
  near(planarJacobian([0, 0, 0]).determinant, 0, 1e-12);
  near(planarJacobian([0, 180, 0]).determinant, 0, 1e-12);
  assert.ok(planarJacobian([0, 90, 0]).dexterity > 0.2);
});

test('inverse kinematics recovers a random forward pose exactly', () => {
  const angles = [64, -71, 27];
  const target = planarPose(angles);
  const solution = solvePlanarInverse(target, { elbow: angles[1] >= 0 ? 'up' : 'down' });
  assert.equal(solution.ok, true);
  solution.anglesDeg.forEach((angle, index) => near(angle, angles[index], 1e-7));
  near(solution.tip.x, target.x, 1e-9);
  near(solution.tip.y, target.y, 1e-9);
  near(solution.orientationDeg, target.phiDeg, 1e-9);
});

test('both mirror solutions reach the same tip and orientation', () => {
  const target = { x: 0.42, y: 0.31, phiDeg: 35 };
  const solutions = solvePlanarInverseBoth(target);
  assert.equal(solutions.length, 2);
  for (const solution of solutions) {
    assert.equal(solution.ok, true);
    near(solution.tip.x, target.x, 1e-9);
    near(solution.tip.y, target.y, 1e-9);
    near(solution.orientationDeg, target.phiDeg, 1e-9);
  }
  assert.notEqual(solutions[0].elbow, solutions[1].elbow);
});

test('the wrist center must stay inside the two-link annulus', () => {
  assert.equal(WRIST_WORKSPACE.minReach, Math.abs(L1 - L2));
  assert.equal(WRIST_WORKSPACE.maxReach, L1 + L2);
  const beyond = solvePlanarInverse({ x: 3, y: 0, phiDeg: 0 });
  assert.equal(beyond.ok, false);
  assert.match(beyond.reason, /beyond the arm’s reach/);

  const inside = solvePlanarInverse({ x: 0.25, y: 0, phiDeg: 0 });
  assert.equal(inside.ok, false);
  assert.match(inside.reason, /inner limit/);
});

test('clamped inverse kinematics lands on the workspace boundary', () => {
  const solution = solvePlanarInverse({ x: 5, y: 0, phiDeg: 0 }, { clamp: true });
  assert.equal(solution.ok, true);
  assert.equal(solution.clamped, true);
  near(Math.hypot(solution.wristCenter.x, solution.wristCenter.y), WRIST_WORKSPACE.maxReach, 1e-9);
  near(solution.tip.x, WRIST_WORKSPACE.maxReach + L3, 1e-6);
});

test('invalid planar requests are rejected', () => {
  assert.throws(() => solvePlanarForward([0, 0]), /Expected 3 planar joint angles/);
  assert.throws(() => solvePlanarForward([0, 0, Number.NaN]), /finite numbers/);
  assert.throws(() => solvePlanarInverse({ x: Number.NaN, y: 0, phiDeg: 0 }), /finite/);
});

test('shortestAngleDelta takes the short way around the joint circle', () => {
  near(shortestAngleDelta(170, -170), 20);
  near(shortestAngleDelta(-170, 170), -20);
  near(shortestAngleDelta(10, 10), 0);
});
