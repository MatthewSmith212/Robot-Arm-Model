# Robot Kinematics Lab

An interactive Three.js study of two robot arms that solve the same problem in two
different ways. Both live in the same page as tabs.

| Tab | Model | What it shows |
| --- | --- | --- |
| **Six-axis articulated arm** | 6 revolute joints (6R), standard Denavit–Hartenberg | Full 3D reach: base swivel, shoulder, elbow, three wrist axes, live transform matrix. |
| **Bench crane arm** | 3 revolute joints (3R): vertical yaw + two parallel hinges | The hand sketch as a buildable mechanism: a mast on the upper platform, the elbow thrown up into an A-frame, and the tool folding down onto either platform — the one it stands on or the lower one beside it. |

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
- `crane-kinematics.test.js` — bench-crane forward kinematics, the A-frame home pose,
  Jacobian against finite differences, inverse kinematics round-trips over the whole
  working band, joint-limit and workspace clamping, unreachable-surface reporting, and
  solving points on **both** platforms.
- `markup.test.js` — tab wiring, unique element ids, and every DOM selector each view uses.
- `dom-smoke.test.js` — boots the real page in jsdom with a stubbed WebGL context,
  mounts both views, switches tabs, solves goals on the bench, clicks the bench, and
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

## Tab 2 · Bench crane arm (3R), two platforms

The sketch, taken literally, defines a perfectly ordinary and very buildable 3R
manipulator: a base bolted flat to a platform, a vertical axis, and two parallel
horizontal hinges. The point of the layout is that the **elbow goes up** while the
**tool comes down**: the upper arm rises away from the platform, the elbow turns
over, and the forearm returns to a surface below.

There are **two working surfaces**. The arm stands on the higher one (platform A)
and reaches across to the lower one (platform B). The task is point-to-point — touch
a spot on A, touch a spot on B — so nothing has to be controlled in mid-air:

```
                     elbow
                       ●
            upper arm ╱ ╲  forearm
                     ╱   ╲
              ┌─────╱     ╲
              │ J1         ╲
   ┌──────────┴───┐   ┌─────●─────┐
   │  Platform A  │   │ Platform B│
   │  0.74 m      │   │  0.45 m   │
   └──────────────┘   └───────────┘
```

### Joints

| Joint | Axis | Role |
| --- | --- | --- |
| θ₁ · base yaw | vertical, bolted to platform A | Swings the whole arm between the platforms. |
| θ₂ · shoulder pitch | horizontal, at the top of the mast | Throws the upper arm — and the elbow — up above the bench. |
| θ₃ · elbow pitch | horizontal, parallel to the shoulder | Folds the forearm back down so the tool lands on the surface. |

The short mast is what makes the sketch work: without it the shoulder hinge would
sit *on* the bench and the forearm could never get under the tool.

### Dimensions

| Item | Value | Between |
| --- | ---: | --- |
| Mast | 0.17 m | platform A top → shoulder hinge |
| Upper arm L₁ | 0.36 m | shoulder hinge → elbow hinge |
| Forearm L_f | 0.52 m | elbow hinge → tool point |
| **Platform A** (mount) | 0.50 × 0.40 m at 0.74 m | floor; the arm is bolted through its middle |
| **Platform B** (lower) | 0.68 × 0.52 m at 0.45 m | floor; centre 0.64 m from the yaw axis, clear of A |
| Shoulder hinge | 0.91 m | floor |
| Straight-arm reach | 0.88 m | shoulder hinge → fully stretched tool |

| Platform | Reachable band (radius from the yaw axis) | Share of the surface in reach |
| --- | --- | ---: |
| A · mount | 0.17 – 0.86 m | 55 % |
| B · lower | 0.17 – 0.75 m | 64 % |

A 0.36 m upper arm with a 0.52 m forearm is deliberate: the elbow stays around
shoulder height while the tool comes down onto A or reaches across and down to B.
B's band is smaller because the arm has further to drop to get there.

### Forward kinematics

```text
R = L₁ cos θ₂ + L_f cos(θ₂ − θ₃)            horizontal reach from the yaw axis
z = z_S + L₁ sin θ₂ + L_f sin(θ₂ − θ₃)      tool height (z_S = 0.91 m)
x = R cos θ₁     y = R sin θ₁               yaw carries it around the platforms
```

A goal is always a point on one platform, so `z` is fixed at 0.74 m (A) or 0.45 m
(B) and the search is over `x, y` alone.

`θ₂ − θ₃` is the forearm elevation. Negative means the forearm — and the tool fixed
to it — points down at the bench; that is the working pose.

### Inverse kinematics (goal point)

Yaw aims the arm at the goal's bearing, which leaves a two-link triangle in that
vertical plane:

```text
d = |TCP − S|
cos β = (d² + L₁² − L_f²) / (2 d L₁)
θ₂ = α + β        α = elevation of the goal, β = elbow triangle
θ₃ = θ₂ − ψ       ψ = elevation of the forearm
```

The upper solution is used: the elbow folds up above the shoulder-to-goal line like
a crane. The lower solution would put the elbow underneath the platforms. Goals
outside the joint travel — behind the base, inside the mast, off the end of the arm,
or on the far corner of B where the arm cannot reach — are answered with the nearest
pose the joints can actually hold and reported as such rather than refusing the move.

Because the task is point-to-point on a surface, `z` is not a free variable in
practice: each goal names a platform and the IK solves for the two remaining
coordinates. Moving between platforms is an ordinary joint-space move with the tool
in the air, which is exactly what the sketch shows.

### What the tab includes

- Two platform tables with legs and stretchers, each with its working band drawn on
  its own top surface, plus radial ticks every 15° across the yaw travel.
- Three camera presets: **Bench view** (oblique), **Side view** (perpendicular to the
  arm, i.e. the sketch's own view) and **Plan view** (straight down on the bench).
- Joint control with sliders and typed angles, live tool point, elbow height above
  the bench, forearm elevation, and whether the tool is on, above or below the surface.
- Goal control: pick **platform A or B**, then type X/Y or click that surface — a click
  sets the goal while a drag orbits.
- Live Jacobian `∂(x, y, z)/∂(θ₁, θ₂, θ₃)`, its determinant, and a DEXTEROUS /
  SINGULAR flag.
- Home and random poses, a demo tour that alternates between the two platforms, and
  toggles for the working bands and joint frames.

## Is the sketch workable?

Yes — it is the most sensible three-axis layout for a bench project, and it is what
desktop 3R arms (and the classic "SCARA-adjacent" teaching rigs) actually look like.
A few things worth knowing before you build:

- **Three axes place the tool, they do not aim it.** Yaw plus two hinges set X, Y and
  Z of the tool point. The tool stays fixed to the forearm, so its angle always
  follows the forearm. That is exactly right for a pen, a probe, a suction cup or a
  laser that has to come down onto a surface; add two wrist axes if you ever need to
  hold an object at a chosen angle.
- **Two levels costs nothing.** Reaching from the upper platform down to the lower one
  needs no extra mechanism: the same two hinges do it, because the arm simply angles
  further down. Platform B just ends up with a smaller usable band than A.
- **The mast matters.** Give the shoulder hinge 0.15–0.20 m of clearance above the
  bench, or the forearm has nowhere to fold down into.
- **Keep the tool clear of the axis.** The model reserves a 0.17 m inner radius.
  Inside that, the turret and the gripper body collide with the mast even though the
  maths can produce an answer.
- **The platform heights are fixed by the arm.** Folding the elbow further always
  pushes the tool lower, so the two surface heights are what the geometry has to be
  designed around, not something you can correct afterwards.
- **Two hinges in one plane is a weak spot.** When the arm is nearly straight, small
  changes in θ₂ and θ₃ cancel each other out; the Jacobian determinant on the tab
  drops towards zero there. That is why a real arm is usually driven through the
  middle of its travel and geared down near the elbow.

## Model notes

Both arms are ideal geometric models: revolute joints are exact, links are perfectly
rigid, and no servo calibration, loads, backlash, compliance, gravity sag, collisions
between the arm and the platforms, or trajectory planning is simulated. The tab places
the tool on a platform; it does not check the path taken to get there. The six-axis model predicts forward kinematics
only; the bench-crane tab also includes closed-form inverse kinematics, and its tool
stays collinear with the forearm (no wrist roll).
