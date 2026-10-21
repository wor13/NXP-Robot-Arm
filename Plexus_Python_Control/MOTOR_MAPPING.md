# Updated motors and centered 180-degree reference

Based on Thomas's October 21, 2026 direction observations and centered-arm photograph.

| Joint | Current ID | Previous ID | Increasing motor degrees | Decreasing motor degrees |
|---|---:|---:|---|---|
| Base rotation | 11 | 11 | Left | Right |
| Shoulder | 12 | 12 | Down | Up |
| Elbow | 13 | 13 | Down | Up |
| Wrist pitch | 1 | 14 | Claw down | Claw up |
| Wrist roll | 2 | 15 | Clockwise, looking from behind toward the claw | Counterclockwise from that viewpoint |
| Gripper | 3 | 16 | Open | Close/grip |

The program's motor order is **[11,12,13,1,2,3]**. It uses that order for diagnostics, synchronous feedback/goals, the UI, logs and generated C/C++ arrays. It does not renumber devices or write EEPROM. Expected motor models remain attached to their joint roles, so renaming 14 to 1 does not make that motor an XL430.

For base left/right, the nominal model interprets the view as standing behind the base looking toward the claw, consistent with the wrist-roll viewpoint. In base coordinates +X points toward the claw at the centered pose, +Y is left from that viewpoint, and +Z is up. All six configured raw-to-model signs are +1. A positive URDF pitch about +Y lowers the claw; a positive wrist roll about +X looks clockwise from behind. Positive gripper rotation increases modeled jaw separation. Confirm the interpretation with tiny supported jogs before verifying the mapping.

## What 180 degrees means

- All six reference motor readings: **180°**.
- Encoder reference: **2048 counts** at 4096 counts/revolution.
- Main sliders and telemetry display **absolute motor degrees**, including negative/multi-turn values without modulo wrapping in extended-position mode.
- Internal IK joint angles are URDF-relative radians. The preset maps the centered 180° pose to **nominal model angle 0** on all six axes.
- The stock gripper's nominal model zero has closed jaw tips, consistent with the user's photograph. This preset is a practical geometric starting point; a single photograph cannot establish submillimeter link/offset accuracy.
- Use the separate model-reference angle fields if measured geometry requires an offset. They are distinct from motor degree readings.

The conversion is:

```text
motor_degrees = raw * 360 / 4096
model_angle = model_reference + sign * (motor_degrees - reference_motor_degrees)
```

The second line uses one consistent angular unit; the Python implementation converts degrees to radians. In the supplied preset, `reference_motor_degrees=180`, `model_reference=0`, and `sign=+1`. Changing a model-reference offset never changes the motor's actual encoder reading or EEPROM.

The centered reference is a **preview/calibration preset**, not an automatic motion. Startup reads the actual encoders and does not command all motors to 180°. Do not jump a physical arm to the reference from an unknown pose; use a checked clear path and physical support.

## Gripper neutral is not a gripping target

**ID 3 at 180° is closed without gripping.** It is not proof that a ring is retained and is not automatically used as a successful grip.

The hardware open/grip target fields are deliberately unset. Teach actual `gripper_open` and `gripper_closed` poses for the installed pads/ring. Opening moves toward higher degrees; gripping moves toward lower degrees as observed. Do not infer how far below 180° to command from the photograph or continue driving against a hard stop. Position control does not regulate gripping force.

The revised mock demo uses an opening above 180° and a small simulated gripping command below 180°. These are only simulation examples, not hardware targets.

## Installing this revision

1. Extract the updated package to a **new folder**, keeping your old calibration, poses and logs as a backup. Reuse a compatible existing Python environment if desired, or run `setup_windows.bat` in the new folder.
2. Run `run_demo.bat`. All six sliders start at 180°, and the default stock-jaw pose corresponds nominally to the photo. **Preview centered 180° reference** only changes the orange model.
3. Read the actual motors:

   ```powershell
   .venv\Scripts\python.exe -m plexus diagnose --port COM7
   ```

   Replace COM7 with the real follower port. Expect IDs **11,12,13,1,2,3**, not 14–16.
4. Start a new hardware session:

   ```powershell
   .venv\Scripts\python.exe -m plexus app --hardware --port COM7 --gripper stock --session centered_v2
   ```

5. Enter measured raw limits and verify the reference geometry/directions using `RUN_STEPS.md`. The user-supplied 180° reference is populated, but physical verification is still false and limits remain blank. This prevents treating a nominal photo alignment as finished IK calibration.
6. Teach actual grip and opening targets, measure the ring-center tool offset, then repeat a small vertical lift test. Re-teach and approve all hardware route poses under this mapping. The fingerprint changes reject old taught hardware data automatically.

The default session is now `centered_180`; old `first_ring` demo data is not selected automatically. If explicitly opening a stale mock session, the app asks for a new session name instead of running its old reversed gripping actions.

For custom printed jaws use `--gripper v` or `--gripper slot` with a new session. Selecting a different jaw variant clears tool-offset verification and supplies an unverified nominal offset; remeasure the actual ring center. The photo uses the stock jaws, so those are now the default.
