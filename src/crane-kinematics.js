/**
 * Bench-mounted three-axis crane arm.
 *
 * This is the arm from the hand sketch: a base plate bolted to a bench top, a
 * vertical yaw axis, then two hinges (shoulder, elbow) that fold the arm into an
 * A-frame. The elbow is thrown up into the air and the forearm comes back down
 * to land the tool on the same bench the arm is bolted to.
 *
 *   θ₁  yaw        — vertical axis standing on the bench; swings the whole
 *                    arm across the bench top
 *   θ₂  shoulder   — horizontal hinge at the top of the short mast; lifts the
 *                    upper arm (and with it the elbow) up or down
 *   θ₃  elbow      — horizontal hinge parallel to the shoulder hinge; folds the
 *                    forearm down onto the bench
 *
 * θ₂ and θ₃ share a plane, so the tip position is a two-link problem inside the
 * vertical plane θ₁ points at:
 *
 *   R = L₁ cos θ₂ + L_f cos(θ₂ − θ₃)          horizontal reach from the yaw axis
 *   z = z_S + L₁ sin θ₂ + L_f sin(θ₂ − θ₃)    height of the tool
 *
 * Three joints set three coordinates (x, y, z) of the tool point, so the tool
 * position is fully controllable. Its *orientation* is not: the tool is fixed to
 * the forearm, so it always points along it. On a bench that is usually what you
 * want — a pen, a probe or a suction cup that comes straight down onto the
 * surface — and it is the reason a real pick-and-place arm adds two wrist axes.
 *
 * Frame: right-handed, Z up, origin on the bench top at the centre of the yaw
 * axis. All lengths in metres, all angles in degrees.
 */

/** The workbench the arm is bolted to. */
export const BENCH = Object.freeze({
  /** Height of the bench top above the floor (a normal desk). */
  height: 0.74,
  width: 1.6,
  depth: 1.2,
  /** Bench centre on the floor; the yaw axis sits at the origin. */
  center: Object.freeze({ x: 0.25, y: 0 }),
});

/** Link lengths and the mast that lifts the shoulder hinge clear of the bench. */
export const ARM = Object.freeze({
  /** Yaw axis on the bench top up to the shoulder hinge. */
  mast: 0.17,
  /** Upper arm: shoulder hinge → elbow hinge. */
  upperArm: 0.36,
  /** Forearm: elbow hinge → tool point. */
  forearm: 0.52,
  /** Height of the shoulder hinge above the floor. */
  get shoulderZ() {
    return BENCH.height + this.mast;
  },
});

const DEG = Math.PI / 180;
const RAD = 180 / Math.PI;
const EPSILON = 1e-9;
const clamp = (value, min, max) => Math.min(Math.max(value, min), max);

/** Distance from the elbow hinge to the tool point. */
export const ELBOW_TO_TIP = ARM.forearm;

/** Height of the shoulder hinge above the floor, resolved once. */
export const SHOULDER_Z = BENCH.height + ARM.mast;

/** Shortest and longest shoulder-to-tool distances. */
export const REACH = Object.freeze({
  min: Math.abs(ARM.upperArm - ARM.forearm),
  max: ARM.upperArm + ARM.forearm,
});

/**
 * Practical working band on the bench, measured from the yaw axis.
 *
 * The kinematics can fold the tool closer than this, but the mast, the turret
 * and the gripper body all live inside ~0.15 m of the axis, so a real build
 * cannot put a tool there without hitting itself. Goals inside the band are
 * clamped to its edge rather than refused.
 */
export const WORK_BAND = Object.freeze({ inner: 0.17, outer: 0.84 });

export const JOINTS = Object.freeze([
  Object.freeze({
    name: 'Base yaw',
    axis: 'VERTICAL AXIS AT THE BENCH',
    min: -150,
    max: 150,
    home: 25,
    hint: 'Turns the whole arm about the vertical axis bolted to the bench.',
  }),
  Object.freeze({
    name: 'Shoulder pitch',
    axis: 'HORIZONTAL HINGE ON THE MAST',
    min: -30,
    max: 100,
    home: 51,
    hint: 'Lifts the upper arm. Positive tips the elbow up, away from the bench.',
  }),
  Object.freeze({
    name: 'Elbow pitch',
    axis: 'HORIZONTAL HINGE, PARALLEL TO THE SHOULDER',
    min: 0,
    max: 170,
    home: 110.9,
    hint: 'Folds the forearm back down. 0° is straight out, 180° folded onto itself.',
  }),
]);

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

/** Clamp joint angles to their travel; returns the angles plus which axes moved. */
export function clampAngles(angles) {
  assertAngles(angles);
  const limited = angles.map((angle, index) => clamp(angle, JOINTS[index].min, JOINTS[index].max));
  return {
    angles: limited,
    clamped: limited.some((angle, index) => Math.abs(angle - angles[index]) > 1e-9),
  };
}

/** Whether an angle vector sits inside every joint's travel. */
export function withinLimits(angles) {
  return angles.every((angle, index) => angle >= JOINTS[index].min - 1e-9 && angle <= JOINTS[index].max + 1e-9);
}

/**
 * Forward kinematics: joint angles (degrees) → points on the arm.
 *
 *   ψ = θ₂ − θ₃ is the forearm elevation: negative means the forearm (and the
 *   tool fixed to it) points down, which is how the tool reaches the bench.
 */
export function solveForward(angles) {
  assertAngles(angles);
  const [yaw, pitch, elbow] = angles;
  const yawRad = yaw * DEG;
  const pitchRad = pitch * DEG;
  const forearmRad = (pitch - elbow) * DEG;

  const ux = Math.cos(yawRad);
  const uy = Math.sin(yawRad);

  const shoulder = { x: 0, y: 0, z: SHOULDER_Z };
  const upperRise = ARM.upperArm * Math.sin(pitchRad);
  const upperReach = ARM.upperArm * Math.cos(pitchRad);
  const elbowPoint = {
    x: upperReach * ux,
    y: upperReach * uy,
    z: SHOULDER_Z + upperRise,
  };
  const forearmReach = ARM.forearm * Math.cos(forearmRad);
  const forearmRise = ARM.forearm * Math.sin(forearmRad);
  const tip = {
    x: (upperReach + forearmReach) * ux,
    y: (upperReach + forearmReach) * uy,
    z: SHOULDER_Z + upperRise + forearmRise,
  };
  const radial = Math.hypot(tip.x, tip.y);

  return {
    angles: [...angles],
    shoulder,
    elbow: elbowPoint,
    tip,
    /** Unit horizontal direction the arm is pointing in. */
    heading: { x: ux, y: uy },
    /** Elbow height above the bench top (negative = below the bench). */
    elbowAboveBench: elbowPoint.z - BENCH.height,
    elbowZ: elbowPoint.z,
    /** Forearm elevation in degrees; negative = pointing down at the bench. */
    forearmElevationDeg: wrapDegrees(pitch - elbow),
    /**
     * Angular position of the tool around the bench, and how high it is above
     * the bench top (0 = resting on it).
     */
    yawDeg: wrapDegrees(yaw),
    radial,
    heightAboveBench: tip.z - BENCH.height,
    reach: Math.hypot(radial, tip.z - SHOULDER_Z),
    distanceFromShoulder: Math.hypot(radial, tip.z - SHOULDER_Z),
  };
}

/**
 * Jacobian of the tool point with respect to (θ₁, θ₂, θ₃) in bench coordinates.
 * Columns are ∂(x, y, z)/∂θᵢ. |det J| falls to zero at a fully stretched or
 * fully folded arm, or when the tool stands on the yaw axis.
 */
export function jacobian(angles) {
  assertAngles(angles);
  const [yaw, pitch, elbow] = angles;
  const yawRad = yaw * DEG;
  const pitchRad = pitch * DEG;
  const forearmRad = (pitch - elbow) * DEG;
  const cosYaw = Math.cos(yawRad);
  const sinYaw = Math.sin(yawRad);

  const R = ARM.upperArm * Math.cos(pitchRad) + ARM.forearm * Math.cos(forearmRad);
  const dR_dPitch = -ARM.upperArm * Math.sin(pitchRad) - ARM.forearm * Math.sin(forearmRad);
  const dR_dElbow = ARM.forearm * Math.sin(forearmRad);
  const dZ_dPitch = ARM.upperArm * Math.cos(pitchRad) + ARM.forearm * Math.cos(forearmRad);
  const dZ_dElbow = -ARM.forearm * Math.cos(forearmRad);

  // J = [ ∂(x,y,z)/∂θ₁  ∂/∂θ₂  ∂/∂θ₃ ]
  const matrix = [
    [-R * sinYaw, dR_dPitch * cosYaw, dR_dElbow * cosYaw],
    [R * cosYaw, dR_dPitch * sinYaw, dR_dElbow * sinYaw],
    [0, dZ_dPitch, dZ_dElbow],
  ];

  const determinant = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
    - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
    + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

  return { matrix, determinant, dexterity: Math.abs(determinant) };
}

/**
 * Radial working band on the bench top: how close and how far the tool can get
 * while still touching the surface inside the joint travel.
 *
 * Returns null for a height the arm cannot touch at all.
 */
export function reachOnBench(z = BENCH.height) {
  const drop = Math.abs(z - SHOULDER_Z);
  const outerSquared = REACH.max ** 2 - drop ** 2;
  if (outerSquared <= 0) {
    return {
      inner: 0,
      outer: 0,
      reachable: false,
      reason: z > SHOULDER_Z
        ? 'That surface is higher than the arm can lift its tool.'
        : 'That surface is lower than the fully stretched arm can reach.',
    };
  }
  const innerSquared = REACH.min ** 2 - drop ** 2;
  return {
    inner: innerSquared > 0 ? Math.sqrt(innerSquared) : 0,
    outer: Math.sqrt(outerSquared),
    reachable: true,
    reason: null,
  };
}

/**
 * Nearest pose the hinges can actually hold, found by a coarse-to-fine search
 * over (θ₂, θ₃). Used when a goal is inside the arm's reach but outside its
 * joint travel: rather than refuse the point, the arm goes as close as it can.
 */
export function nearestFeasible(radial, height) {
  const evaluate = (pitch, elbow) => {
    const forearmRad = (pitch - elbow) * DEG;
    const R = ARM.upperArm * Math.cos(pitch * DEG) + ARM.forearm * Math.cos(forearmRad);
    const Z = SHOULDER_Z + ARM.upperArm * Math.sin(pitch * DEG) + ARM.forearm * Math.sin(forearmRad);
    return { R, Z, error: Math.hypot(R - radial, Z - height) };
  };

  let best = { pitch: JOINTS[1].home, elbow: JOINTS[2].home, error: Number.POSITIVE_INFINITY };
  let pitchSpan = [JOINTS[1].min, JOINTS[1].max];
  let elbowSpan = [JOINTS[2].min, JOINTS[2].max];
  for (let pass = 0; pass < 4; pass += 1) {
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
 * Inverse kinematics for a tool point (x, y, z), defaulting to the bench top.
 *
 * The yaw axis aims the arm at the point's bearing, so what is left is a
 * two-link problem in the vertical plane through that bearing. The elbow is
 * always the *upper* solution — a crane pose with the elbow folded up above the
 * line from the shoulder to the target — because the lower solution would put
 * the elbow underneath the bench.
 *
 *   d = |target − shoulder|
 *   cos β = (d² + L₁² − L_f²) / (2 d L₁)      β = elbow triangle
 *   θ₂ = α + β                                 α = elevation of the target
 *   θ₃ = θ₂ − ψ                                ψ = elevation of the forearm
 *
 * With `clamp: true` the call never fails: the bearing is brought inside the yaw
 * travel, the radius inside the working band, and a target outside the hinge
 * travel is answered with the nearest pose the arm can actually hold.
 */
export function solveInverse(target, options = {}) {
  const { clamp: clampToWorkspace = false, height = BENCH.height } = options;
  const { x, y } = target;
  if (!Number.isFinite(x) || !Number.isFinite(y)) {
    throw new TypeError('Inverse kinematics requires finite x and y values.');
  }
  const hasZ = target.z !== undefined && target.z !== null;
  const z = hasZ ? target.z : height;
  if (!Number.isFinite(z)) {
    throw new TypeError('Inverse kinematics requires a finite z value.');
  }

  const requested = { x, y, z };
  const refuse = (reason) => ({ ok: false, reason, distance: null, requested });
  let clamped = false;

  // 1. Bearing: the yaw axis has finite travel.
  let yawDeg = Math.atan2(y, x) * RAD;
  if (yawDeg < JOINTS[0].min - 1e-9 || yawDeg > JOINTS[0].max + 1e-9) {
    if (!clampToWorkspace) {
      return refuse(`That point sits outside the yaw travel (${JOINTS[0].min}° to ${JOINTS[0].max}°).`);
    }
    yawDeg = clamp(yawDeg, JOINTS[0].min, JOINTS[0].max);
    clamped = true;
  }

  // 2. Height: the tool must lie inside the shoulder-to-tool distance band.
  let drop = z - SHOULDER_Z;
  const band = reachOnBench(z);
  if (!band.reachable) {
    if (!clampToWorkspace) return refuse(band.reason);
    const magnitude = clamp(Math.abs(drop), REACH.min + 1e-6, REACH.max - 1e-6);
    drop = (drop < 0 ? -1 : 1) * magnitude;
    clamped = true;
  }

  // 3. Radius: clear of the mast, inside the arm's length, inside the travel.
  const geometryInner = Math.sqrt(Math.max(0, REACH.min ** 2 - drop ** 2));
  const geometryOuter = Math.sqrt(Math.max(0, REACH.max ** 2 - drop ** 2));
  const low = Math.max(WORK_BAND.inner, geometryInner);
  const high = Math.min(WORK_BAND.outer, geometryOuter);
  const allowed = { min: Math.min(low, high), max: high };

  const askedRadial = Math.hypot(x, y);
  let radial = askedRadial;
  if (radial < allowed.min - 1e-9 || radial > allowed.max + 1e-9) {
    if (!clampToWorkspace) {
      return refuse(radial > allowed.max
        ? 'That point is beyond the arm’s reach.'
        : 'That point is closer than the arm can fold, or inside the mast.');
    }
    radial = clamp(radial, allowed.min, allowed.max);
    clamped = true;
  }

  // 4. Two-link solution in the vertical plane through the bearing.
  const distance = Math.hypot(radial, drop);
  const alpha = Math.atan2(drop, radial);
  const cosBeta = clamp(
    (distance ** 2 + ARM.upperArm ** 2 - ARM.forearm ** 2) / (2 * distance * ARM.upperArm),
    -1,
    1,
  );
  const beta = Math.acos(cosBeta);
  const pitchRad = alpha + beta;
  const elbowRadial = ARM.upperArm * Math.cos(pitchRad);
  const elbowRise = ARM.upperArm * Math.sin(pitchRad);
  const forearmRad = Math.atan2(drop - elbowRise, radial - elbowRadial);
  const pitchDeg = pitchRad * RAD;
  const elbowDeg = wrapDegrees((pitchRad - forearmRad) * RAD);

  let angles = [yawDeg, pitchDeg, elbowDeg];
  const hingeFits = pitchDeg >= JOINTS[1].min - 1e-6 && pitchDeg <= JOINTS[1].max + 1e-6
    && elbowDeg >= JOINTS[2].min - 1e-6 && elbowDeg <= JOINTS[2].max + 1e-6;

  if (!hingeFits) {
    if (!clampToWorkspace) {
      return refuse('That point would need the hinges outside their travel.');
    }
    // Neither hinge can follow the closed form: hold the nearest pose it can.
    const fallback = nearestFeasible(radial, SHOULDER_Z + drop);
    angles = [yawDeg, fallback.pitch, fallback.elbow];
    clamped = true;
  }

  const forward = solveForward(angles);
  return {
    ok: true,
    clamped,
    usedFallback: !hingeFits,
    requested,
    angles,
    tip: forward.tip,
    elbowZ: forward.elbowZ,
    elbowAboveBench: forward.elbowAboveBench,
    forearmElevationDeg: forward.forearmElevationDeg,
    radial: forward.radial,
    heightAboveBench: forward.heightAboveBench,
    reach: forward.reach,
    distance,
    dexterity: jacobian(angles).dexterity,
  };
}

/** Solve for a point on the bench top. */
export function solveOnBench(x, y, options = {}) {
  return solveInverse({ x, y, z: BENCH.height }, options);
}

/** True when the tool point is resting on (or within `tolerance` of) the bench. */
export function onBench(z, tolerance = 0.004) {
  return Math.abs(z - BENCH.height) <= tolerance;
}

/** Whether a point lies over the bench top. */
export function overBench(x, y, margin = 0) {
  return Math.abs(x - BENCH.center.x) <= BENCH.width / 2 + margin
    && Math.abs(y - BENCH.center.y) <= BENCH.depth / 2 + margin;
}
