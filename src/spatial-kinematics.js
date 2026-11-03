/**
 * Spatial shoulder–elbow arm.
 *
 * Three revolute joints, but unlike a planar arm the axes are not parallel:
 *
 *   θ₁  shoulder yaw    — turns the whole arm about the vertical Z axis, so it
 *                          can swing from one working plane to another
 *   θ₂  shoulder pitch  — raises the upper arm about a horizontal axis; this is
 *                          the second shoulder degree of freedom
 *   θ₃  elbow pitch     — folds the forearm about the same horizontal axis
 *
 * Because θ₂ and θ₃ act in the vertical plane that θ₁ points at, the arm is a
 * crane-like linkage: the shoulder pitch throws the elbow up into the air and the
 * elbow pitch drops the long forearm — and the tool on the end of it — down onto
 * a plane below. That is what gives the tool a full 3D position (x, y, z) in
 * three degrees of freedom.
 *
 * Tool orientation is not controlled: the gripper stays collinear with the
 * forearm, so the tip position alone is the task.
 */

/** Mounting plane (where the arm is bolted) and the lower plane it reaches down to. */
export const PLANES = Object.freeze({
  mount: Object.freeze({ key: 'mount', name: 'Plane 1 · mounting surface', height: 0.85 }),
  lower: Object.freeze({ key: 'lower', name: 'Plane 2 · lower surface', height: 0.45 }),
});

/** Dimensions in metres, angles in degrees. */
export const ARM = Object.freeze({
  /** Height of the shoulder pitch axis above the floor. */
  shoulderZ: 0.95,
  /** Pedestal height above the mounting plane (shoulderZ − mount plane). */
  pedestal: 0.10,
  /** Upper arm: shoulder → elbow. Short, so the elbow can be thrown high. */
  upperArm: 0.32,
  /** Forearm: elbow → wrist. Long, so it can hang down to the lower plane. */
  forearm: 0.62,
  /** Tool: wrist → tip. Kept collinear with the forearm. */
  tool: 0.16,
});

/** Distance from the elbow to the tool tip. */
export const ELBOW_TO_TIP = ARM.forearm + ARM.tool;

export const JOINTS = Object.freeze([
  Object.freeze({ name: 'Shoulder yaw', axis: 'REVOLUTE ABOUT VERTICAL Z', min: -180, max: 180, home: -56, hint: 'Swings the arm between the two working planes.' }),
  Object.freeze({ name: 'Shoulder pitch', axis: 'REVOLUTE ABOUT HORIZONTAL AXIS', min: -25, max: 135, home: 43, hint: 'Second shoulder axis: lifts the upper arm, elbow first.' }),
  Object.freeze({ name: 'Elbow pitch', axis: 'REVOLUTE ABOUT HORIZONTAL AXIS', min: 0, max: 190, home: 110, hint: 'Drops the forearm down from the raised elbow.' }),
]);

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;
const EPSILON = 1e-9;
const clamp = (value, min, max) => Math.min(Math.max(value, min), max);

/** Fold an angle into the half-open interval [-180°, 180°). */
export function wrapDegrees(deg) {
  const wrapped = ((deg + 180) % 360 + 360) % 360 - 180;
  return Math.abs(wrapped) < EPSILON ? 0 : wrapped;
}

function assertAngles(angles) {
  if (!Array.isArray(angles) || angles.length !== JOINTS.length) {
    throw new TypeError(`Expected ${JOINTS.length} joint angles in degrees.`);
  }
  if (angles.some((angle) => !Number.isFinite(angle))) {
    throw new TypeError('Joint angles must all be finite numbers.');
  }
}

/**
 * Tool tip height when the arm is folded as far as it goes, i.e. the lowest
 * surface the tool can touch. Used to check whether a plane is reachable.
 */
export const REACH = Object.freeze({
  /** Minimum and maximum shoulder-to-tip distance. */
  min: Math.abs(ARM.upperArm - ELBOW_TO_TIP),
  max: ARM.upperArm + ELBOW_TO_TIP,
});

/** Radial reach on a horizontal plane at height `z` (metres), as [inner, outer]. */
export function reachOnPlane(z) {
  const drop = Math.abs(z - ARM.shoulderZ);
  const outer = Math.sqrt(Math.max(0, REACH.max ** 2 - drop ** 2));
  const innerSquared = REACH.min ** 2 - drop ** 2;
  const inner = innerSquared > 0 ? Math.sqrt(innerSquared) : 0;
  return { inner, outer, reachable: outer > 0 };
}

/**
 * Forward kinematics.
 *
 *   A = L₁cos θ₂ + L_fcos ψ      (horizontal distance from the shoulder)
 *   B = L₁sin θ₂ + L_fsin ψ      (height above the shoulder)
 *   ψ = θ₂ − θ₃                  (forearm elevation, negative = pointing down)
 *
 * The base frame has the shoulder axis on Z at (0, 0, shoulderZ).
 */
export function solveForward(angles) {
  assertAngles(angles);
  const [yaw, pitch, elbow] = angles;
  const yawRad = yaw * DEG;
  const pitchRad = pitch * DEG;
  const forearmRad = (pitch - elbow) * DEG;

  const cosYaw = Math.cos(yawRad);
  const sinYaw = Math.sin(yawRad);
  const horizontal = ARM.upperArm * Math.cos(pitchRad) + ELBOW_TO_TIP * Math.cos(forearmRad);
  const rise = ARM.upperArm * Math.sin(pitchRad) + ELBOW_TO_TIP * Math.sin(forearmRad);

  const shoulder = { x: 0, y: 0, z: ARM.shoulderZ };
  const elbowPoint = {
    x: ARM.upperArm * Math.cos(pitchRad) * cosYaw,
    y: ARM.upperArm * Math.cos(pitchRad) * sinYaw,
    z: ARM.shoulderZ + ARM.upperArm * Math.sin(pitchRad),
  };
  const forearmDirection = {
    x: Math.cos(forearmRad) * cosYaw,
    y: Math.cos(forearmRad) * sinYaw,
    z: Math.sin(forearmRad),
  };
  const wrist = {
    x: elbowPoint.x + ARM.forearm * forearmDirection.x,
    y: elbowPoint.y + ARM.forearm * forearmDirection.y,
    z: elbowPoint.z + ARM.forearm * forearmDirection.z,
  };
  const tip = {
    x: shoulder.x + horizontal * cosYaw,
    y: shoulder.y + horizontal * sinYaw,
    z: shoulder.z + rise,
  };

  const radial = Math.hypot(tip.x, tip.y);
  return {
    angles: [...angles],
    shoulder,
    elbow: elbowPoint,
    wrist,
    tip,
    forearmDirection,
    /** Elbow elevation relative to the shoulder (negative = elbow below it). */
    elbowRise: elbowPoint.z - ARM.shoulderZ,
    /** Height of the elbow above the floor. */
    elbowZ: elbowPoint.z,
    /** Absolute forearm elevation in degrees (negative = pointing down). */
    forearmElevationDeg: wrapDegrees(pitch - elbow),
    radial,
    reach: Math.hypot(radial, tip.z - ARM.shoulderZ),
  };
}

/**
 * Jacobian of the tool tip with respect to the three joints, in base
 * coordinates. Columns are ∂(x, y, z)/∂θᵢ; |det J| drops to zero when the arm is
 * fully stretched or folded, or when the tool sits on the yaw axis.
 */
export function jacobian(angles) {
  assertAngles(angles);
  const [yaw, pitch, elbow] = angles;
  const yawRad = yaw * DEG;
  const pitchRad = pitch * DEG;
  const forearmRad = (pitch - elbow) * DEG;
  const cosYaw = Math.cos(yawRad);
  const sinYaw = Math.sin(yawRad);
  const horizontal = ARM.upperArm * Math.cos(pitchRad) + ELBOW_TO_TIP * Math.cos(forearmRad);

  const dHorizontal = {
    pitch: -ARM.upperArm * Math.sin(pitchRad) - ELBOW_TO_TIP * Math.sin(forearmRad),
    elbow: ELBOW_TO_TIP * Math.sin(forearmRad),
  };
  const dRise = {
    pitch: ARM.upperArm * Math.cos(pitchRad) + ELBOW_TO_TIP * Math.cos(forearmRad),
    elbow: -ELBOW_TO_TIP * Math.cos(forearmRad),
  };

  const matrix = [
    [-horizontal * sinYaw, dHorizontal.pitch * cosYaw, dHorizontal.elbow * cosYaw],
    [horizontal * cosYaw, dHorizontal.pitch * sinYaw, dHorizontal.elbow * sinYaw],
    [0, dRise.pitch, dRise.elbow],
  ];

  const determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
    - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
    + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

  return { matrix, determinant, dexterity: Math.abs(determinant) };
}

/**
 * Closest reachable position to (radial, drop) that also respects both pitch
 * limits, found by a coarse-to-fine search over (θ₂, θ₃).
 *
 * Some points are geometrically inside the arm's reach but cannot be adopted
 * within the shoulder's travel; rather than refusing the goal, the arm moves to
 * the nearest pose it can actually hold and the caller reports it as clamped.
 */
export function nearestFeasible(radial, drop) {
  const evaluate = (pitch, elbow) => {
    const forearmRad = (pitch - elbow) * DEG;
    const R = ARM.upperArm * Math.cos(pitch * DEG) + ELBOW_TO_TIP * Math.cos(forearmRad);
    const Z = ARM.upperArm * Math.sin(pitch * DEG) + ELBOW_TO_TIP * Math.sin(forearmRad);
    return { R, Z, error: Math.hypot(R - radial, Z - drop) };
  };

  let best = { pitch: JOINTS[1].home, elbow: JOINTS[2].home, error: Number.POSITIVE_INFINITY };
  let pitchSpan = [JOINTS[1].min, JOINTS[1].max];
  let elbowSpan = [JOINTS[2].min, JOINTS[2].max];
  for (let pass = 0; pass < 3; pass += 1) {
    const pitchStep = (pitchSpan[1] - pitchSpan[0]) / 60;
    const elbowStep = (elbowSpan[1] - elbowSpan[0]) / 60;
    for (let pitch = pitchSpan[0]; pitch <= pitchSpan[1] + 1e-9; pitch += pitchStep) {
      for (let elbow = elbowSpan[0]; elbow <= elbowSpan[1] + 1e-9; elbow += elbowStep) {
        const candidate = evaluate(pitch, elbow);
        if (candidate.error < best.error) best = { pitch, elbow, error: candidate.error };
      }
    }
    pitchSpan = [Math.max(JOINTS[1].min, best.pitch - pitchStep * 2), Math.min(JOINTS[1].max, best.pitch + pitchStep * 2)];
    elbowSpan = [Math.max(JOINTS[2].min, best.elbow - elbowStep * 2), Math.min(JOINTS[2].max, best.elbow + elbowStep * 2)];
  }
  return best;
}

/**
 * Inverse kinematics: place the tool tip at (x, y, z).
 *
 * The tip position in the vertical plane through the yaw direction is a
 * two-link problem (upper arm, then elbow-to-tip), so the law of cosines gives
 * both elbow configurations in closed form:
 *
 *   cos β = (d² + L₁² − L_f²) / (2 d L₁),   θ₂ = α ± β
 *   θ₃ = θ₂ − ψ,   ψ = elevation of the target seen from the elbow
 *
 * `elbow: 'up'` lifts the elbow above the shoulder-to-target line (the crane
 * pose); `elbow: 'down'` folds it underneath. With the tool well below the
 * shoulder only the raised solution stays inside the shoulder's travel.
 */
export function solveInverse(target, options = {}) {
  const { elbow = 'up', clamp: clampToWorkspace = false } = options;
  const { x, y, z } = target;
  if (![x, y, z].every(Number.isFinite)) {
    throw new TypeError('Inverse kinematics requires finite x, y, and z values.');
  }

  const requested = { x, y, z };
  const yawRad = Math.atan2(y, x);
  let radial = Math.hypot(x, y);
  let drop = z - ARM.shoulderZ;
  let distance = Math.hypot(radial, drop);
  let clamped = false;

  if (distance > REACH.max + EPSILON || distance < REACH.min - EPSILON) {
    if (!clampToWorkspace) {
      return {
        ok: false,
        reason: distance > REACH.max ? 'Target is beyond the arm’s reach.' : 'Target is closer than the arm can fold.',
        distance,
      };
    }
    const limited = clamp(distance, REACH.min, REACH.max);
    if (distance < EPSILON) {
      radial = limited;
      drop = 0;
    } else {
      radial *= limited / distance;
      drop *= limited / distance;
    }
    distance = limited;
    clamped = true;
  }

  const alpha = Math.atan2(drop, radial);
  const cosBeta = clamp((distance ** 2 + ARM.upperArm ** 2 - ELBOW_TO_TIP ** 2) / (2 * distance * ARM.upperArm), -1, 1);
  const beta = Math.acos(cosBeta);

  const solutions = ['up', 'down'].map((configuration) => {
    const pitchRad = alpha + (configuration === 'up' ? beta : -beta);
    const elbowRadial = ARM.upperArm * Math.cos(pitchRad);
    const elbowRise = ARM.upperArm * Math.sin(pitchRad);
    const forearmRad = Math.atan2(drop - elbowRise, radial - elbowRadial);
    const elbowDegrees = wrapDegrees((pitchRad - forearmRad) * RAD);
    return {
      elbow: configuration,
      pitchDeg: pitchRad * RAD,
      elbowDeg: elbowDegrees,
      elbowZ: ARM.shoulderZ + elbowRise,
      elbowRise,
    };
  });

  const withinLimits = (solution) => solution.pitchDeg >= JOINTS[1].min - EPSILON
    && solution.pitchDeg <= JOINTS[1].max + EPSILON
    && solution.elbowDeg >= JOINTS[2].min - EPSILON
    && solution.elbowDeg <= JOINTS[2].max + EPSILON;

  let chosen = solutions.find((solution) => solution.elbow === elbow);
  let usedFallback = false;
  if (!withinLimits(chosen)) {
    const alternative = solutions.find((solution) => solution.elbow !== elbow);
    if (withinLimits(alternative)) {
      chosen = alternative;
      usedFallback = true;
    } else {
      // Neither closed-form elbow fits inside the joint travel. Move to the
      // nearest pose the arm can actually hold instead of refusing the goal.
      const fallback = nearestFeasible(radial, drop);
      const fallbackAngles = [wrapDegrees(yawRad * RAD), fallback.pitch, fallback.elbow];
      const fallbackForward = solveForward(fallbackAngles);
      return {
        ok: true,
        clamped: true,
        usedFallback: true,
        requested,
        angles: fallbackAngles,
        elbow: fallback.pitch > alpha * RAD ? 'up' : 'down',
        elbowZ: fallbackForward.elbowZ,
        elbowRise: fallbackForward.elbowRise,
        tip: fallbackForward.tip,
        distance,
        reach: fallbackForward.reach,
        dexterity: jacobian(fallbackAngles).dexterity,
      };
    }
  }

  const angles = [wrapDegrees(yawRad * RAD), chosen.pitchDeg, chosen.elbowDeg];
  const forward = solveForward(angles);
  return {
    ok: true,
    clamped,
    usedFallback,
    requested,
    angles,
    elbow: chosen.elbow,
    elbowZ: chosen.elbowZ,
    elbowRise: chosen.elbowRise,
    tip: forward.tip,
    distance,
    reach: forward.reach,
    dexterity: jacobian(angles).dexterity,
  };
}

/** Solve for a point on one of the two working planes. */
export function solveOnPlane(planeKey, x, y, options = {}) {
  const plane = PLANES[planeKey];
  if (!plane) throw new TypeError(`Unknown plane "${planeKey}".`);
  return solveInverse({ x, y, z: plane.height }, options);
}

/** True when the tool tip is resting on (or within `tolerance` of) a plane. */
export function touchingPlane(z, tolerance = 0.004) {
  return Object.values(PLANES).find((plane) => Math.abs(z - plane.height) <= tolerance) ?? null;
}
