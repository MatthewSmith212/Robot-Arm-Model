# Robot Kinematics Lab

An interactive Three.js study of two robot arms that solve the same problem in two
different ways. Both live in the same page as tabs.

| Tab | Model | What it shows |
| --- | --- | --- |
| **Six-axis articulated arm** | 6 revolute joints (6R), standard Denavit–Hartenberg | Full 3D reach: base swivel, shoulder, elbow, three wrist axes, live transform matrix. |
| **Shoulder–elbow arm** | 3 revolute joints (3R): two-axis shoulder + elbow | A raised elbow and a long forearm reaching between two working planes, with joint-space and goal-point control. |

## Run locally

```bash
npm install
npm run dev
```

Open the local Vite URL printed in the terminal and pick a tab. A tab can be linked
directly with `#six` or `#spatial`. For a static production bundle, run `npm run build`;
`npm run preview` serves that bundle.

If the browser cannot create a WebGL context (hardware acceleration off, a blocklisted
GPU, or software rendering disabled), each 3D viewport shows a short "3D view unavailable"
notice instead of a blank panel, and the tab can still be switched safely.

### Tests

`npm test` runs:

- `kinematics.test.js` — six-axis DH transforms and ZYX roll–pitch–yaw.
- `spatial-kinematics.test.js` — shoulder–elbow forward kinematics, the elbow being
  thrown above the shoulder, Jacobian against finite differences, inverse kinematics
  round-trips on both planes, joint-limit fallbacks, workspace clamping.
- `markup.test.js` — tab wiring, unique element ids, and every DOM selector each view uses.
- `dom-smoke.test.js` — boots the real page in jsdom with a stubbed WebGL context,
  mounts both views, switches tabs, solves goals on both planes, clicks a bench, and
  fails on any runtime error.

## Tab 1 · Six-axis articulated arm

- 3D 6R arm with orbit / zoom controls, visible joint housings, tool gripper, world axes, and DH frames.
- Six angle controls with practical joint travel ranges, home / random poses, and a demo trajectory.
- Standard Denavit–Hartenberg parameter table and the live frame origins.
- Forward kinematics for `T₀⁶ = A₁A₂A₃A₄A₅A₆`; tool-center-point position and ZYX roll–pitch–yaw orientation.
- The live 4 × 4 homogeneous transform and an expandable DH matrix.

### Kinematic convention

The model uses a right-handed base frame with **Z up**. Each joint is revolute about the preceding frame's `z` axis. Its standard DH transform is

```text
Aᵢ = Rz(θᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rx(αᵢ)
T₀⁶ = A₁(θ₁) · A₂(θ₂) · … · A₆(θ₆)
pTCP = T₀⁶ · [0, 0, 0, 1]ᵀ
```

| Joint | dᵢ (m) | aᵢ (m) | αᵢ (deg) | Angle range (deg) |
| --- | ---: | ---: | ---: | ---: |
| 1 · Base swivel | 0.28 | 0.00 | 90 | −170 to 170 |
| 2 · Shoulder pitch | 0.00 | 0.38 | 0 | −95 to 105 |
| 3 · Elbow pitch | 0.00 | 0.32 | 0 | −135 to 135 |
| 4 · Wrist roll | 0.00 | 0.12 | 90 | −180 to 180 |
| 5 · Wrist pitch | 0.00 | 0.10 | −90 | −110 to 110 |
| 6 · Tool roll | 0.14 | 0.00 | 0 | −180 to 180 |

The position shown is relative to the center of joint 1's rotation axis; the base pedestal is only a visual offset. The end frame is the gripper's nominal tool-center point.

## Tab 2 · Shoulder–elbow arm between two planes

A three-axis arm bolted to an upper bench that throws its elbow into the air and
rests its tool on a lower bench — the two-tier setup in the reference sketch. The
shoulder carries **two** axes, which is what lets the tool move in three dimensions
instead of sliding around a single plane.

### Joints

| Joint | Axis | Role |
| --- | --- | --- |
| θ₁ · shoulder yaw | vertical (Z) | Swings the whole arm between the two working areas. |
| θ₂ · shoulder pitch | horizontal | **Second shoulder axis**: throws the short upper arm — and the elbow — up above the bench. |
| θ₃ · elbow pitch | horizontal | Drops the long forearm down onto the lower plane. |

### Geometry

| Link | Length (m) | Between |
| --- | ---: | --- |
| Upper arm L₁ | 0.32 | shoulder → elbow |
| Forearm L_f | 0.62 | elbow → wrist |
| Tool | 0.16 | wrist → tool center point (collinear with the forearm) |

| Working plane | Height | Notes |
| --- | ---: | --- |
| P1 · mounting surface | 0.85 m | The arm is bolted here; the shoulder pitch axis is 0.10 m above it. |
| P2 · lower surface | 0.45 m | The tool reaches down onto this bench. |

The short upper arm plus the long forearm is deliberate: the elbow can be thrown
~30 cm above the shoulder while the tool still reaches a surface 40 cm below it.

### Forward kinematics

```text
z_E = z_S + L₁ sin θ₂                          elbow height
R   = L₁ cos θ₂ + L_f cos(θ₂ − θ₃)             horizontal reach
z   = z_S + L₁ sin θ₂ + L_f sin(θ₂ − θ₃)       tool height
x   = R cos θ₁     y = R sin θ₁                yaw carries it into 3D
```

`θ₂ − θ₃` is the forearm elevation: negative means the forearm points down, which
is the pose that reaches the lower plane.

### Inverse kinematics (goal point)

Yaw points the arm at the goal, then the shoulder-to-tip line and the elbow form a
triangle:

```text
d = |TCP − S|
cos β = (d² + L₁² − L_f²) / (2 d L₁)
θ₂ = α ± β        α = elevation of the goal, β = elbow triangle
θ₃ = θ₂ − ψ       ψ = elevation of the goal seen from the elbow
```

**Elbow up** lifts the elbow above the shoulder-to-goal line (the crane pose);
**elbow down** folds it underneath. With the tool below the shoulder only the raised
solution stays inside the shoulder's travel. Goals that are inside the arm's reach
but outside the joint limits are solved to the nearest pose the arm can actually
hold and reported as clamped; goals beyond the arm's length are clamped to the
workspace boundary.

### What the tab includes

- Two-tier bench scene with both planes labelled, plus the reachable ring drawn flat
  on each surface.
- Three camera presets: **Bench view** (oblique), **Side view** (reads the elbow and
  the two pitches) and **Plan view** (looks straight down on both planes).
- Joint control with sliders and typed angles, live tool pose, elbow height and how
  far it rises above the shoulder, forearm elevation, and which plane the tool is
  touching.
- Goal control: pick plane 1 or 2, type X/Y, or click a bench in the scene — a click
  sets the goal while a drag still orbits.
- Live Jacobian `∂(x, y, z)/∂(θ₁, θ₂, θ₃)`, its determinant, and a DEXTEROUS /
  SINGULAR flag.
- Home and random poses, a demo tour that touches both planes, and toggles for the
  reach rings and joint frames.

## Model notes

Both arms are ideal geometric models: revolute joints are exact, links are perfectly
rigid, and no servo calibration, loads, backlash, compliance, gravity sag, collisions, or
trajectory planning is simulated. The six-axis model predicts forward kinematics only; the
shoulder–elbow tab also includes closed-form inverse kinematics, and its tool stays
collinear with the forearm (the gripper's roll is not modelled).
