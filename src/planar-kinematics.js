/**
 * Planar 3R kinematics.
 *
 * Three revolute joints with parallel axes normal to a flat mounting plane (the
 * floor). Every link therefore stays in that plane, so the arm has exactly three
 * degrees of freedom: the tip's X, its Y, and its in-plane orientation φ. Its
 * height above the plane never changes.
 *
 * Angles are measured in degrees, counter-clockwise from the +X axis of the
 * floor frame; link lengths are in meters.
 */

export const PLANAR_LINKS = Object.freeze([
  Object.freeze({ label: 'Link 1', name: 'Shoulder link', length: 0.55 }),
  Object.freeze({ label: 'Link 2', name: 'Elbow link', length: 0.45 }),
  Object.freeze({ label: 'Link 3', name: 'Wrist link', length: 0.22 }),
]);

/** Joint travel used by the planar model (degrees). Joint 1 can face anywhere. */
export const PLANAR_LIMITS = Object.freeze([
  Object.freeze({ min: -180, max: 180, home: 22 }),
  Object.freeze({ min: -165, max: 165, home: 48 }),
  Object.freeze({ min: -180, max: 180, home: -32 }),
]);

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;
const EPSILON = 1e-9;

/** Fold an angle into the half-open interval [-180°, 180°). */
export function wrapDegrees(deg) {
  const wrapped = ((deg + 180) % 360 + 360) % 360 - 180;
  return Math.abs(wrapped) < EPSILON ? 0 : wrapped;
}

/**
 * Distance from the base a wrist center can sit at: the classic two-link
 * annulus. The tool tip adds one more link, so its reachable set is a disk.
 */
export const WRIST_WORKSPACE = Object.freeze({
  minReach: Math.abs(PLANAR_LINKS[0].length - PLANAR_LINKS[1].length),
  maxReach: PLANAR_LINKS[0].length + PLANAR_LINKS[1].length,
});

/** Reachable disk of the tool tip for any tool orientation. */
export const TIP_WORKSPACE = Object.freeze({
  innerRadius: Math.max(0, Math.abs(PLANAR_LINKS[0].length - PLANAR_LINKS[1].length) - PLANAR_LINKS[2].length),
  outerRadius: PLANAR_LINKS.reduce((total, link) => total + link.length, 0),
});

function assertAngles(anglesDeg) {
  if (!Array.isArray(anglesDeg) || anglesDeg.length !== PLANAR_LINKS.length) {
    throw new TypeError(`Expected ${PLANAR_LINKS.length} planar joint angles in degrees.`);
  }
  if (anglesDeg.some((angle) => !Number.isFinite(angle))) {
    throw new TypeError('Planar joint angles must all be finite numbers.');
  }
}

/**
 * Forward kinematics for the planar chain.
 *
 * x = Σ Lᵢ cos(θ₁ + … + θᵢ),  y = Σ Lᵢ sin(θ₁ + … + θᵢ),  φ = θ₁ + θ₂ + θ₃
 */
export function solvePlanarForward(anglesDeg) {
  assertAngles(anglesDeg);

  const points = [{ x: 0, y: 0 }];
  const absoluteDegrees = [];
  let x = 0;
  let y = 0;
  let absolute = 0;

  for (let i = 0; i < PLANAR_LINKS.length; i += 1) {
    absolute += anglesDeg[i];
    x += PLANAR_LINKS[i].length * Math.cos(absolute * DEG);
    y += PLANAR_LINKS[i].length * Math.sin(absolute * DEG);
    points.push({ x, y });
    absoluteDegrees.push(wrapDegrees(absolute));
  }

  return {
    points,
    absoluteDegrees,
    tip: { x, y },
    orientationDeg: wrapDegrees(absolute),
    reach: Math.hypot(x, y),
  };
}

/**
 * Task Jacobian for the planar arm. Rows are dX/dθ, dY/dθ, and dφ/dθ for the
 * three joints. |det J| is the classic planar dexterity measure: the tip moves
 * freely in every direction only when it is not zero.
 */
export function planarJacobian(anglesDeg) {
  assertAngles(anglesDeg);
  const [l1, l2, l3] = PLANAR_LINKS.map((link) => link.length);
  const [t1, t2, t3] = anglesDeg.map((angle) => angle * DEG);
  const s1 = Math.sin(t1);
  const c1 = Math.cos(t1);
  const s12 = Math.sin(t1 + t2);
  const c12 = Math.cos(t1 + t2);
  const s123 = Math.sin(t1 + t2 + t3);
  const c123 = Math.cos(t1 + t2 + t3);

  const matrix = [
    [-l1 * s1 - l2 * s12 - l3 * s123, -l2 * s12 - l3 * s123, -l3 * s123],
    [l1 * c1 + l2 * c12 + l3 * c123, l2 * c12 + l3 * c123, l3 * c123],
    [1, 1, 1],
  ];

  const determinant = -l1 * l2 * Math.sin(t2) - l1 * l3 * Math.sin(t2 + t3) - l2 * l3 * Math.sin(t3);
  return { matrix, determinant, dexterity: Math.abs(determinant) };
}

/** Tool tip position (x, y) and orientation (φ) expressed as a pose object. */
export function planarPose(anglesDeg) {
  const { tip, orientationDeg, reach } = solvePlanarForward(anglesDeg);
  return { x: tip.x, y: tip.y, phiDeg: orientationDeg, reach };
}

/**
 * Closed-form inverse kinematics.
 *
 * The wrist center w = tip − L₃·[cos φ, sin φ] must lie in the two-link annulus,
 * so the first two joints follow from the law of cosines and the third joint is
 * whatever is left of the requested orientation.
 *
 * options.elbow: 'up' | 'down' (which of the two mirror solutions to keep)
 * options.clamp: pull an out-of-reach target onto the nearest reachable pose
 */
export function solvePlanarInverse(target, options = {}) {
  const { elbow = 'up', clamp = false } = options;
  const phiDeg = target.phiDeg ?? 0;
  if (!Number.isFinite(target.x) || !Number.isFinite(target.y) || !Number.isFinite(phiDeg)) {
    throw new TypeError('Planar inverse kinematics requires finite x, y, and φ values.');
  }

  const [l1, l2, l3] = PLANAR_LINKS.map((link) => link.length);
  const phi = phiDeg * DEG;
  const requested = { x: target.x, y: target.y };

  let wristX = target.x - l3 * Math.cos(phi);
  let wristY = target.y - l3 * Math.sin(phi);
  let distance = Math.hypot(wristX, wristY);
  let clamped = false;

  const maxReach = l1 + l2;
  const minReach = Math.abs(l1 - l2);
  if (distance > maxReach + EPSILON || distance < minReach - EPSILON) {
    if (!clamp) {
      return {
        ok: false,
        reason: distance > maxReach ? 'Target is beyond the arm’s reach.' : 'Target is inside the arm’s inner limit.',
        wristCenter: { x: wristX, y: wristY },
        distance,
      };
    }
    clamped = true;
    const limited = Math.min(Math.max(distance, minReach), maxReach);
    if (distance < EPSILON) {
      wristX = limited;
      wristY = 0;
    } else {
      wristX *= limited / distance;
      wristY *= limited / distance;
    }
    distance = limited;
  }

  const cosElbow = Math.min(1, Math.max(-1, (distance ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)));
  const elbowMagnitude = Math.acos(cosElbow);
  const elbowAngle = elbow === 'down' ? -elbowMagnitude : elbowMagnitude;
  const shoulderAngle = Math.atan2(wristY, wristX)
    - Math.atan2(l2 * Math.sin(elbowAngle), l1 + l2 * Math.cos(elbowAngle));
  const wristAngle = phi - shoulderAngle - elbowAngle;

  const anglesDeg = [
    wrapDegrees(shoulderAngle * RAD),
    wrapDegrees(elbowAngle * RAD),
    wrapDegrees(wristAngle * RAD),
  ];
  const forward = solvePlanarForward(anglesDeg);

  return {
    ok: true,
    clamped,
    elbow,
    requested,
    anglesDeg,
    points: forward.points,
    tip: forward.tip,
    orientationDeg: forward.orientationDeg,
    wristCenter: { x: wristX, y: wristY },
    distance,
    dexterity: planarJacobian(anglesDeg).dexterity,
  };
}

/** Both mirror solutions (elbow up and elbow down) for a target pose. */
export function solvePlanarInverseBoth(target) {
  return ['up', 'down']
    .map((elbow) => solvePlanarInverse(target, { elbow }))
    .filter((solution) => solution.ok);
}

/** Shortest signed rotation (degrees) that takes `from` to `to` on the joint circle. */
export function shortestAngleDelta(from, to) {
  return wrapDegrees(to - from);
}
