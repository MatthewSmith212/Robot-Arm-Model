# Servo Arm · Kinematics Lab

An interactive Three.js model of a six-axis, servo-driven articulated arm. Move the six revolute-joint controls and the rendered links, joint frames, end-effector pose, DH table, and homogeneous transform update together.

## Run locally

```bash
npm install
npm run dev
```

Open the local Vite URL printed in the terminal. For a static production bundle, run `npm run build`; `npm run preview` serves that bundle.

## What is included

- 3D 6R arm with orbit / zoom controls, visible joint housings, tool gripper, world axes, and DH frames.
- Six angle controls with practical joint travel ranges, home / random poses, and a demo trajectory.
- Standard Denavit–Hartenberg parameter table and the live frame origins.
- Forward kinematics for `T₀⁶ = A₁A₂A₃A₄A₅A₆`; tool-center-point position and ZYX roll–pitch–yaw orientation.
- The live 4 × 4 homogeneous transform and an expandable DH matrix.
- Unit tests for the transform math: `npm test`.

## Kinematic convention

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

The position shown is relative to the center of joint 1's rotation axis; the base pedestal is only a visual offset. The end frame is the gripper's nominal tool-center point. This is an ideal geometric model: it does not simulate servo pulse calibration, loads, backlash, compliance, collisions, or inverse kinematics. Hardware servo commands need per-servo zero and travel calibration before use.
