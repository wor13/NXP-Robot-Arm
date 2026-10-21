# Running and commissioning the arm

For direct individual-joint movement before IK calibration, use `run_arm_COM7.bat` and `MOVE_ARM_COM7.md`. The steps below commission full calibrated motion and pick-and-place.

Follow this order. Keep each hardware test supervised and the arm physically supported during setup. This software has no obstacle detector, grip-success sensor or contact-force controller. A screen STOP requests a hold; the available physical power disconnect remains essential.

## 1. Install and prove the software in simulation

Use Python 3.12 64-bit. Extract the full folder. Open a terminal there:

```powershell
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe -m plexus app --mock
```

Open http://127.0.0.1:8080. Select a joint, drag its rotation handle or move its slider. Orange geometry should change without moving teal feedback. The UI is local to your PC; no ChatGPT plugin is required to operate it. No PyBullet, ROS or C++ compiler is required for normal Python operation.

Check the support/work-area box, click **Arm full motion**, then execute a small preview change. Use **STOP / hold**, then re-arm explicitly. Try the vertical demo from the unchanged initial simulated pose. Close the terminal with Ctrl+C when finished.

If port 8080 is occupied, add `--web-port 8085` and open that port. Keep simulation and hardware sessions in separate folders. Mock operation never opens the serial port.

## 2. Put the correct bridge on the OpenRB-150

This program speaks native DYNAMIXEL Protocol 2.0 using the official SDK. It will not communicate through a custom ASCII bridge expecting `HELLO`, `MOVE` or `POS` strings. Keep a backup of any firmware you want to preserve.

Use the [official OpenRB-150 manual](https://emanual.robotis.com/docs/en/parts/controller/openrb-150/) for the current Arduino board installation and USB-to-DYNAMIXEL example. In Arduino IDE:

1. Install the Arduino SAMD support and ROBOTIS OpenRB board package as directed by that manual. Select OpenRB-150 and its actual USB port.
2. Open the official `usb_to_dynamixel` example for OpenRB-150, compile and upload it. The board may already have this factory bridge; do not replace a working bridge unnecessarily.
3. Confirm the firmware enables the board's DYNAMIXEL power path as the official example requires. USB logic power alone is not the motor supply.
4. Preserve the assembled OMX-F power wiring and voltage conversion. The larger and smaller motors have different supply requirements. Do not bypass the arm's converter or connect all motors directly to an assumed supply voltage.
5. Close Serial Monitor, DYNAMIXEL Wizard and other controllers before starting Python. Only one program should own the serial port.

This package does not modify the manufacturer's bridge source. Its compilation/upload and your physical power path need on-bench verification.

## 3. Read all six motors before enabling motion

```powershell
.venv\Scripts\python.exe -m plexus ports
.venv\Scripts\python.exe -m plexus diagnose --port COM7
```

Confirm you selected the **follower** arm, not a leader device. Diagnostics should report each ID, model, operating mode, drive mode, torque state, signed raw position and hardware error. Expected layout:

| Motor | Model | Function |
|---|---|---|
| 11–13 | XL430-W250, model 1060 | First three arm axes |
| 1–2 | XL330-M288, model 1200 | Remaining arm axes |
| 3 | XL330-M288, model 1200 | Gripper |

The configuration expects 1 Mbps and operating mode 4. Read back and confirm. Hardware differences must be intentionally resolved in configuration or commissioning; do not blindly change EEPROM or copy assumed motor positions. The arm also requires velocity-based profiles. The program rejects a time-based-profile configuration.

A missing motor is a blocker. Resolve bridge firmware, power, cable continuity, baud rate and IDs before continuing. Do not scan by changing IDs on a connected chain.

## 4. Calibrate the raw-to-model mapping and limits

```powershell
.venv\Scripts\python.exe -m plexus app --hardware --port COM7 --gripper stock --session red_A_B
```

Initial startup reads feedback but sends no register writes. The teal model stays hidden until mapping verification. Orange geometry is initially an uncalibrated preview.

The 180° reference is preloaded as raw 2048, with a nominal URDF reference of 0° for each joint. The raw limits in `config/hardware.json` are still blank on purpose. Establish conservative usable bounds for **every** motor from measured, supported positions with adequate clearance from mechanical stops and cable limits. Use the measured safe range, not the URDF's broad ±360° limits. Do not use mechanical impacts to find limits.

At a physically supported reference pose:

1. Put the arm in the photographed, supported centered reference, with all six motor readings at 180° (approximately raw 2048). The main sliders now display motor degrees, not URDF angles. Folder 2 has separate **model reference angle** fields, initially 0°. Use **Preview model reference geometry**, compare against the real pose, and adjust those offsets if needed. A photo gives a nominal preset, not precision IK calibration; confirm offsets from measured geometry and several known poses.
2. In folder 2 enter each motor's conservative raw minimum/maximum. Set the initial sign for each axis. Leave the verification checkbox unchecked. **Capture reference and save calibration** stores current encoder positions and the separate model-reference fields together. The main motor-degree sliders do not define a new zero by themselves.
3. Check the support/work-area box. **Arm raw-jog calibration only** allows small count movements after the same motor/mode/error checks. The default jog is 4 counts, with an 8-count maximum. Check one motor at a time: increasing raw must move the real joint in the direction predicted by its configured sign. Use **Copy feedback into preview** and compare. If a direction is wrong, stop, correct its sign and recapture a matching reference.
4. Verify all six mappings, limits and the actual pose. Stop/disarm, check the verification box and capture the final supported reference with matching model-reference fields again. Restart the app so the slider ranges reflect the final calibration.
5. Click **Arm full motion** only after this is complete. Try a tiny unloaded movement in a clear area, then compare measured poses with model predictions at several positions.

Arming preloads current goals, applies nonzero RAM profiles and a watchdog, and may enable torque in mode 4. It does not change operating modes. Mode 3 is supported only with torque already enabled and a calibration valid for those coordinates; enabling mode-3 torque can reset encoder coordinates. The program refuses to do that automatically.

After a power cycle, reboot, operating-mode change, homing-offset change, motor replacement or detected coordinate reset, recheck references and limits before replaying. Extended-position counts are not an absolute multi-turn reference across every reset. Any calibration/settings change invalidates taught hardware poses and route approval.

## 5. Calibrate the ring-center tool point

IK controls the ring center, not a jaw tip or a convenient wrist origin. In the model the tool offset is measured from **link5**, in link5's local XYZ frame, in meters in JSON and millimeters in the UI. The colored frame shows its axes.

The stock jaws start with nominal `[91.93, -1.6, 0]` mm. The V/slot jaw variants start with nominal `[91.93, -1.6, -18]` mm. These are geometry-based guesses, not measurements of your ring center. Changing the jaw variant clears tool verification. With a ring securely centered between the pads, measure the center offset from the wrist frame. A useful check is to compare several arm orientations while keeping the same physical ring center over a reference target; an incorrect offset makes the predicted center shift with wrist rotation. Adjust using measurements, not a ring pressing against a post.

The tool normal currently assumes the ring plane is parallel to link5's XY plane, with link5 +Z pointing up at a level grasp. The code holds this normal; an angularly misaligned jaw mount requires correction of the mount or an extension to tool orientation calibration. The UI's XYZ offset does not correct an angular mount error.

Save the measured XYZ in folder 4 and check its physical verification box only after confirming the ring is level. Repeat or verify this calibration for different jaw variants, ring diameters and contact positions. A ring seating deeper into a V can change its center relative to the wrist. Use separate backed-up configurations and sessions as needed.

## 6. Prove a small straight lift before clearing the whole post

Measure post verticality and clamp the arm base in its final position. Center the ring on the post without side preload. Use an unloaded or sacrificial test arrangement first, then the actual ring at low speed. Do not secure the puzzle in a way that hides a lateral pushing problem.

1. Grip the ring gently and verify it cannot slip. Joint position alone does not prove a grip. Position control will not regulate contact force; a tightly commanded closed position can overload pads, prints or motors.
2. Start with **+1 or +2 mm**, not the full transfer. Preview the vertical IK plan from current feedback. It must solve completely without changing the motors' goals. Check the line and endpoint.
3. Arm full motion if necessary, then execute the plan. The fixed gripper raw command stays unchanged throughout the lift.
4. Measure lateral motion and ring tilt externally, for example with a fixed scale/camera in two views or a dial indicator that does not constrain the motion. Observe whether the puzzle moves. Encoder-derived XYZ alone cannot reveal gearbox backlash or a ring slipping in the pads.
5. Repeat upward and downward tests. If it binds, shifts the puzzle or faults, stop and inspect centering, backlash, geometry, grip and post alignment. Do not force the lift or simply increase allowed error.
6. Increase travel gradually only after small tests pass. Measure the required full lift: at the end, the **bottom of the ring** must be above the post top plus a measured clearance margin before any horizontal transfer. The bundled 5 mm sequence values are only demonstration values and may leave the ring on the post.

The software starts at 2 mm/s and 4 mm/s² Cartesian limits, 0.2 mm planned XY error and a 0.3 mm feedback-predicted XY drift limit. These are initial engineering choices, not proven mechanical tolerances. The model initially permits up to 2° normal tilt; physically establish a substantially level ring rather than treating that allowance as a target. The entire physical error budget must fit inside approximately 0.54 mm radial clearance, with margin.

If a posture is unreachable, near a singularity or impossible at encoder resolution, choose a different arm configuration or improve the mechanism. A different higher joint angle alone does not solve the straight-path problem.

## 7. Teach one complete pick-and-place route

Use a distinct session such as `red_A_B` for each required ring/contact geometry and transfer route. The new session creates `data/hardware/red_A_B/poses.json` and `sequence.json`. Stop/disarm before editing configuration or route files.

For Cartesian lift/lower, replace that session's `sequence.json` with a copy of `config/sequence.ik.template.json`. Change `travel_mm` to **measured, tested distances**. The source lift must fully clear its post; the target lowering must stop at the measured seating height. Reload it using **Reload edited sequence.json**.

Record these actual feedback poses using the exact names:

| Name | Meaning |
|---|---|
| `gripper_open` | Actual taught opening on ID 3; increasing degrees opens, with sufficient clearance from neighbors |
| `gripper_closed` | Actual taught grip on ID 3; decreasing degrees closes/grips. The 180° neutral position is closed without gripping and is not this target |
| `source_approach` | Clear entry position with arm axes already matching before playback |
| `source_grasp` | Centered, level grasp at the source |
| `transit_clear` | Clear transfer route above relevant posts/obstacles |
| `target_clear` | Centered and level above the target; start of measured vertical lowering |
| `target_retracted` | Tested clear retreat after opening and release |
| `return_clear` | Clear return route |

For teaching use **Record current feedback as pose**. Source and target must be centered by measurement. There is no vision-based post finding. Add more clear waypoints if one transit segment is insufficient. Joint-space moves between clear waypoints can curve and are not collision checked.

For the simpler `sequence.template.json`, also teach `source_lift_01`, `source_lift_02`, `source_clear`, `target_lower_01` and `target_seated`. That route is a manual joint-pose approach and does not guarantee vertical motion; use the IK template for constrained travel on a post.

Review each action and its complete path at low speed. Hardware poses must come from the current real calibration; simulated values are rejected. After reviewing, check the physical-path review box and approve the exact route. Editing poses/settings/steps invalidates approval.

Before playback manually bring the **arm axes** to the first arm entry pose using a separately checked clear path. Playback rejects a different entry position instead of inventing an approach. The program then preplans all vertical segments before movement, preserves the gripper during arm segments, and preserves the arm during grip segments. It aborts on communication, hardware-error, watchdog, tracking, limit or IK-drift faults.

No grip sensor verifies successful pickup or placement. Watch every early cycle and record outcomes. Qualify each required ring size and peg route, not just one convenient move.

## 8. Logs, recovery and export

Telemetry and commands are logged to `logs/*.jsonl`. Poses and route approval are under `data/hardware/<session>/`. Back up the matching `config/hardware.json` with each validated set. Do not mix ring variants or re-zeroed encoders with old taught data.

**STOP / hold** aborts and attempts a fresh-position hold, retaining the current gripper command. Re-arm before further motion. It is not a hard real-time or physical emergency stop. A disconnected PC/bus cannot guarantee a commanded hold; a watchdog may stop a motor profile without holding up a gravity load. The program does not automatically disable torque because an unsupported arm may fall. Inspect and support the mechanism during any recovery.

After a hardware route has passed physical qualification:

```powershell
.venv\Scripts\python.exe -m plexus export --session red_A_B --output porting\generated_poses.h
```

Export rejects mock/stale/unapproved poses and expands IK into timed raw-position frames. See `PORTING_TO_MCXN947.md`. The file is data for the firmware team, not finished MCXN947 firmware.

## Common problems

| Symptom | Next action |
|---|---|
| No motors respond | Confirm official bridge, correct COM port, motor supply, 1 Mbps and IDs; close other serial applications |
| Calibration/limits error | Enter measured bounds and references; do not substitute demo values |
| Model moves opposite the real axis | Recheck raw-to-model sign and reference at a supported known pose |
| Hardware mode/model mismatch | Read actual settings and resolve intentionally; program never changes EEPROM |
| IK says not level | Establish a level ring and correct model/reference/tool calibration before engaging the post |
| IK unreachable or rounded goal rejected | Change posture or improve calibration/resolution; do not disable the checks |
| Feedback drift or arrival fault | Inspect tracking, asynchronous motor behavior, binding and limits before retrying |
| Ring slips but encoders look normal | Improve jaw support/friction/grip retention; software cannot infer actual ring motion |
| Windows package install failure | Confirm Python 3.12 and architecture, then retain the error log; Windows ARM64 was not tested |

## Updated motor mapping

Use `MOTOR_MAPPING.md` for the 11,12,13,1,2,3 order and centered 180° preset. The program selects configured IDs; it does not change any device ID. Old taught poses and calibration fingerprints must not be reused after this update.
