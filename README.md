# Robot Kinematics Lab

An interactive Three.js study of two robot arms that solve the same problem in two
different ways. Both live in the same page as tabs.

| Tab | Model | What it shows |
| --- | --- | --- |
| **Six-axis articulated arm** | 6 revolute joints (6R), standard Denavit–Hartenberg | Full 3D reach: base swivel, shoulder, elbow, three wrist axes, live transform matrix. |
| **Planar arm on the floor** | 3 revolute joints (3R), parallel axes along the floor normal | Motion confined to the plane it is bolted to: X, Y and tool angle φ, with joint-space and goal-pose control. |

## Run locally

```bash
npm install
npm run dev
```

Open the local Vite URL printed in the terminal and pick a tab. A tab can be linked
directly with `#six` or `#planar`. For a static production bundle, run `npm run build`;
`npm run preview` serves that bundle. Tests: `npm test`.

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

## Tab 2 · Planar arm on the floor

A simplified teaching arm bolted flat to the floor. All three joints turn about the
**floor normal (Z)**, so every link stays in the horizontal mounting plane and the tool
only ever has three degrees of freedom: `x`, `y`, and its heading `φ` measured from the
floor's +X axis. Its height is fixed by the mount.

### Geometry

| Joint | Link | Length (m) | Range (deg) | Role |
| --- | --- | ---: | ---: | --- |
| 1 · Base swivel | L₁ | 0.55 | −180 to 180 | Aims the whole arm around the mount. |
| 2 · Shoulder | L₂ | 0.45 | −165 to 165 | Bends the upper arm within the plane. |
| 3 · Wrist | L₃ | 0.22 | −180 to 180 | Swings the gripper about the wrist point. |

### Forward kinematics

```text
x = Σ Lᵢ cos(χᵢ),   y = Σ Lᵢ sin(χᵢ),   φ = θ₁ + θ₂ + θ₃
χᵢ = θ₁ + … + θᵢ      (absolute link angle from the floor +X axis)
```

### Jacobian

```text
J = [ ∂x/∂θ₁  ∂x/∂θ₂  ∂x/∂θ₃ ]
    [ ∂y/∂θ₁  ∂y/∂θ₂  ∂y/∂θ₃ ]
    [    1        1        1    ]

det J = −L₁L₂ sin θ₂ − L₁L₃ sin(θ₂+θ₃) − L₂L₃ sin θ₃
```

`|det J|` is shown live as the plane's dexterity: it drops to zero when the arm is
fully stretched or fully folded, where some directions of tool motion become impossible.

### Inverse kinematics (goal-pose control)

Switch the control mode to **Goal pose** (or simply drag on the floor) to command `x`,
`y`, and `φ` instead of joint angles:

1. Take the wrist point `w = (x, y) − L₃·[cos φ, sin φ]`.
2. Solve the two-link triangle with the law of cosines
   `cos θ₂ = (|w|² − L₁² − L₂²) / (2L₁L₂)`; **Elbow up / down** picks the mirror answer.
3. Recover `θ₁` from the direction of `w` minus the interior angle, then `θ₃ = φ − θ₁ − θ₂`.

Goals outside the dashed workspace rings are clamped to the nearest reachable pose and
the marker turns amber.

### What the tab includes

- Top-down **Plan view** and oblique **Floor view** cameras, pointer drag on the floor plane,
  wheel zoom, shift-drag pan.
- Three joint sliders plus typed angles, with the tool pose, reach, absolute link angles,
  frame origins, and Jacobian updating live.
- Workspace ring overlays for the tool tip (disk) and the wrist point (annulus).
- Home / random poses, demo motion (sampling joints or tracing a goal path), and toggles
  for the rings and joint frames.

## Model notes

Both arms are ideal geometric models: revolute joints are exact, links are perfectly
rigid, and no servo calibration, loads, backlash, compliance, gravity sag, collisions, or
trajectory planning is simulated. The six-axis model predicts forward kinematics only;
the planar tab also includes closed-form inverse kinematics.
