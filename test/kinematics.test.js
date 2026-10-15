import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import { DH, dhMatrix, matrixToRPYDegrees, solveForwardKinematics } from '../src/kinematics.js';

const near = (actual, expected, tolerance = 1e-9) => {
  assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} is not within ${tolerance} of ${expected}`);
};

test('DH identity parameters produce an identity transform', () => {
  const matrix = dhMatrix(0, { a: 0, d: 0, alpha: 0 });
  const identity = new THREE.Matrix4();
  matrix.elements.forEach((value, index) => near(value, identity.elements[index]));
});

test('zero-angle arm origin agrees with the sum of its DH translations', () => {
  const { frames, endTransform } = solveForwardKinematics([0, 0, 0, 0, 0, 0]);
  assert.equal(frames.length, DH.length + 1);
  near(frames[1].elements[14], 0.28);
  near(frames[2].elements[12], 0.38);
  near(frames[3].elements[12], 0.70);
  near(endTransform.elements[12], 0.92);
  near(endTransform.elements[13], -0.14);
  near(endTransform.elements[14], 0.28);
});

test('shoulder pitch rotates the upper arm into the vertical plane', () => {
  const { frames } = solveForwardKinematics([0, 90, 0, 0, 0, 0]);
  near(frames[2].elements[12], 0);
  near(frames[2].elements[13], 0);
  near(frames[2].elements[14], 0.66);
});

test('base yaw rotates the arm reach about the base z axis', () => {
  const { endTransform } = solveForwardKinematics([90, 0, 0, 0, 0, 0]);
  near(endTransform.elements[12], 0.14);
  near(endTransform.elements[13], 0.92);
  near(endTransform.elements[14], 0.28);
});

test('ZYX RPY output matches the orientation of the zero-angle pose', () => {
  const { endTransform } = solveForwardKinematics([0, 0, 0, 0, 0, 0]);
  const rpy = matrixToRPYDegrees(endTransform);
  near(rpy.roll, 90);
  near(rpy.pitch, 0);
  near(rpy.yaw, 0);
});

test('invalid joint-angle vectors are rejected', () => {
  assert.throws(() => solveForwardKinematics([0, 0, 0]), /Expected 6 joint angles/);
  assert.throws(() => solveForwardKinematics([0, 0, 0, 0, 0, Number.NaN]), /finite numbers/);
});
