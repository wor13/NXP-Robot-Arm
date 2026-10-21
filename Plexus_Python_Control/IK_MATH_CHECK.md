# Straight vertical ring motion and Wolfram verification

## Task definition

Let the ring-center position at grasp be `p0 = (x0, y0, z0)`, expressed in the calibrated robot base frame, in meters. With vertical travel `h`, duration `T`, and normalized time `u=t/T`:

```math
s(u)=10u^3-15u^4+6u^5,
\qquad p(u)=(x_0,\ y_0,\ z_0+h\,s(u)),\quad 0\le u\le1.
```

X and Y remain constant. Keep the ring's normal aligned with its initial upward normal. This assumes the post is vertical in the robot's base frame. A tilted post requires physical realignment or a generalized post-axis path; the current UI plans world-Z travel only.

A single joint moved upward normally follows an arc at the ring. Multiple joints must coordinate at each intermediate height. The planner uses forward kinematics from the bundled OMX-F URDF and bounded numerical inverse kinematics at each sample.

The arm has five independent arm joints plus a gripper. The solver uses **three position constraints and two independent normal-direction constraints**. Rotation about the ring normal is free, which is appropriate for an axisymmetric ring. It does not claim arbitrary six-dimensional pose control. The gripper's raw goal is fixed during vertical motion.

## Wolfram calculation performed

Wolfram Language was evaluated through the user's selected Wolfram app:

```wolfram
s[u_] := 10 u^3 - 15 u^4 + 6 u^5;
p[u_] := {x0, y0, z0 + h s[u]};
<|
 "HorizontalDerivative" -> D[p[u][[1 ;; 2]], u],
 "VelocityEndpoints" -> {s'[0], s'[1]},
 "AccelerationEndpoints" -> {s''[0], s''[1]},
 "PeakNormalizedVelocity" -> MaxValue[{s'[u], 0 <= u <= 1}, u],
 "PeakNormalizedAcceleration" -> MaxValue[{Abs[s''[u]], 0 <= u <= 1}, u],
 "RadialClearanceMM" -> N[(425/1000 - 3825/10000) 254/10/2]
|>
```

Returned results:

| Check | Result |
|---|---|
| Horizontal derivative | `{0, 0}` |
| Start/end normalized velocity | `{0, 0}` |
| Start/end normalized acceleration | `{0, 0}` |
| Maximum normalized speed | `15/8 = 1.875` |
| Maximum normalized acceleration magnitude | `10/Sqrt[3] ≈ 5.77350` |
| Nominal radial clearance | `0.53975 mm` |

Consequently the duration is at least `max(1.875 |h|/vmax, sqrt((10/sqrt(3)) |h|/amax))`, with consistent units. The code initially uses 2 mm/s and 4 mm/s². This establishes a straight, smoothly timed **desired** path. It does not validate the actual robot's geometry or measured tracking.

## Clearance and backlash

From the user-provided dimensions:

- Post diameter: 0.3825 in = 9.7155 mm.
- Hole diameter: 0.425 in = 10.795 mm.
- Diametral gap: 1.0795 mm.
- Centered radial clearance: half that gap = **0.53975 mm**.

That allowance must cover centering error, ring tilt through its approximately 12.7 mm thickness, tool-offset error, encoder quantization, backlash, compliance and post misalignment. Tilt consumes part of the radial allowance, so 0.53975 mm is not an available software-error allowance in every grasp.

A 2.5° angular disturbance corresponds to about 28.44 counts at 4096 counts/revolution. The previous estimate of 0.05 mm/count is pose- and joint-dependent; the actual lateral displacement follows the robot's Jacobian and lever arms. If the estimated 1.42 mm lateral disturbance occurs physically, it exceeds the nominal radial clearance. IK alone cannot correct motion that the motor encoders cannot observe. Measure actual lateral play and reduce it mechanically or with a more suitable measured control system.

## Implementation and checks

`plexus/ik.py`:

1. Starts from actual signed encoder positions with a verified raw-to-angle mapping.
2. Uses the measured ring-center offset in link5 local coordinates. Tool normal assumes the ring plane matches link5 XY.
3. Requires an initially upward, level model normal, and holds that initial normal.
4. Solves each height using the previous solution as the seed, with measured raw bounds converted to joint-angle bounds.
5. Rejects unreachable poses, singular or discontinuous solutions.
6. Searches neighboring integer encoder goals around the continuous solution and rejects excessive nominal XY/Z error, tilt or command increments.
7. Samples nominal joint interpolation between successive rounded commands for lateral drift.
8. Completes planning before executing. A route compiles all its vertical segments before any route movement.
9. Sends synchronized six-axis goals at the planned intervals, retaining the gripper command.
10. Checks health, following error and encoder-predicted lateral drift/tilt, including final settling. Faults abort the sequence and attempt a fresh-position hold.

The interpolation checks sample a model. They are not continuous collision proofs, and independently profiled servo response can differ from nominal joint interpolation. The PC is not a hard real-time controller.

## Initial tolerances

| Quantity | Initial setting |
|---|---:|
| Initial normal departure from world +Z | 2° maximum in the model |
| Rounded goal lateral error | 0.2 mm maximum |
| Rounded goal Z error | 0.35 mm maximum |
| Rounded goal normal change | 0.35° maximum |
| Encoder-predicted lateral drift during execution | 0.3 mm maximum |
| Encoder-predicted normal change during execution | 0.75° maximum |

These settings are starting points for commissioning. Physically level the ring closely; the 2° check is not a recommendation to begin tilted. Tight clearance, compliance and measurement uncertainty can demand tighter tolerances or a better mechanism. Do not increase these bounds merely to make a failed lift run.

## Included model example

`examples/vertical_lift_model.json` contains a freshly calculated +5 mm demonstration from the centered all-180° reference, with stock jaws and nominal URDF zeros. `examples/vertical_lift_model.png` visualizes that same calculation. Both are **simulation-only**. They must never be used as motor commands or accuracy evidence for the real arm. See `VALIDATION.md` for the calculated result and test coverage.

The real acceptance test is a measured small lift with no sideways loading or puzzle movement, followed by progressively larger tests up to measured full post clearance. Horizontal transfer begins only after the bottom of the ring clears the post top plus margin.
