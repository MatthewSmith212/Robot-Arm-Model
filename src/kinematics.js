import * as THREE from 'three';

/** Standard Denavit–Hartenberg dimensions for the 6R teaching model (meters, degrees). */
export const DH = Object.freeze([
  Object.freeze({ a: 0.00, d: 0.28, alpha: 90 }),
  Object.freeze({ a: 0.38, d: 0.00, alpha: 0 }),
  Object.freeze({ a: 0.32, d: 0.00, alpha: 0 }),
  Object.freeze({ a: 0.12, d: 0.00, alpha: 90 }),
  Object.freeze({ a: 0.10, d: 0.00, alpha: -90 }),
  Object.freeze({ a: 0.00, d: 0.14, alpha: 0 }),
]);

/**
 * Build Aᵢ = Rz(θᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rx(αᵢ), using the standard DH convention.
 * Three.js Matrix4.set takes values in row-major order.
 */
export function dhMatrix(thetaDeg, { a, d, alpha }) {
  const theta = THREE.MathUtils.degToRad(thetaDeg);
  const alphaRad = THREE.MathUtils.degToRad(alpha);
  const cTheta = Math.cos(theta);
  const sTheta = Math.sin(theta);
  const cAlpha = Math.cos(alphaRad);
  const sAlpha = Math.sin(alphaRad);

  return new THREE.Matrix4().set(
    cTheta, -sTheta * cAlpha, sTheta * sAlpha, a * cTheta,
    sTheta, cTheta * cAlpha, -cTheta * sAlpha, a * sTheta,
    0, sAlpha, cAlpha, d,
    0, 0, 0, 1,
  );
}

/**
 * Return all base-relative DH frames for a six-element vector of servo angles (degrees).
 * frames[0] is T₀₀ = I, frames[i] is T₀ᵢ; each joint axis is zᵢ₋₁.
 */
export function solveForwardKinematics(anglesDeg) {
  if (!Array.isArray(anglesDeg) || anglesDeg.length !== DH.length) {
    throw new TypeError(`Expected ${DH.length} joint angles in degrees.`);
  }
  if (anglesDeg.some((angle) => !Number.isFinite(angle))) {
    throw new TypeError('Joint angles must all be finite numbers.');
  }

  let transform = new THREE.Matrix4();
  const frames = [transform.clone()];
  const jointOrigins = [];
  const jointAxes = [];

  for (let i = 0; i < DH.length; i += 1) {
    const elements = transform.elements;
    jointOrigins.push(new THREE.Vector3(elements[12], elements[13], elements[14]));
    jointAxes.push(new THREE.Vector3(elements[8], elements[9], elements[10]).normalize());
    transform = transform.clone().multiply(dhMatrix(anglesDeg[i], DH[i]));
    frames.push(transform.clone());
  }
  return { frames, jointOrigins, jointAxes, endTransform: transform };
}

/** Convert the rotation of a homogeneous transform to ZYX roll/pitch/yaw in degrees. */
export function matrixToRPYDegrees(transform) {
  const e = transform.elements;
  const roll = Math.atan2(e[6], e[10]);
  const pitch = Math.atan2(-e[2], Math.hypot(e[6], e[10]));
  const yaw = Math.atan2(e[1], e[0]);
  return {
    roll: THREE.MathUtils.radToDeg(roll),
    pitch: THREE.MathUtils.radToDeg(pitch),
    yaw: THREE.MathUtils.radToDeg(yaw),
  };
}
