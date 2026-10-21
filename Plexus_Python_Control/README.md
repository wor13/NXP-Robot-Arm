# Plexus arm control: start here

Created for Thomas Tran, October 21, 2026. Updated for motor IDs 11, 12, 13, 1, 2, 3 and the centered 180° reference. Goal: a supervised ring pick-and-place prototype with straight vertical removal from a post, then a C/C++ handoff for the MCXN947.

**Python runs on the PC.** The OpenRB-150 runs the official ROBOTIS USB-to-DYNAMIXEL bridge. The PC plans motion and sends synchronized motor goals through that bridge. This package does not put Python on the OpenRB-150.

## Move individual real joints on COM7

For direct encoder-based manual movement before full IK calibration, double-click **run_arm_COM7.bat** after setup. Or run:

```powershell
.\.venv\Scripts\python.exe -m plexus app --hardware --port COM7 --manual --web-port 8090 --gripper stock --session manual_COM7
```

Open http://127.0.0.1:8090, confirm the support/work area, click **Arm manual joint motion**, copy feedback, change one slider by 0.5 degrees and click **Execute preview**. Read **MOVE_ARM_COM7.md** for the small-command/session bounds and error messages. This mode does not save temporary limits as calibration or run IK/pick-and-place.

## Try the interactive model first

1. Extract the entire ZIP. Open the extracted `Plexus_Python_Control` folder in VS Code or File Explorer. Keep `plexus`, `config` and `models` together.
2. Install **Python 3.12, 64-bit**. On Windows run `setup_windows.bat`, or use the commands below from the extracted folder.
3. Run `run_demo.bat` and open **http://127.0.0.1:8080** in a browser on the same PC. Keep the terminal open.

```powershell
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe -m plexus app --mock
```

On Linux/macOS, create the environment with `python3.12 -m venv .venv`, then use `.venv/bin/python` instead of the Windows executable path.

The viewer starts with the stock jaws shown in your reference photo. Both custom jaw variants remain available. It has joint sliders displaying absolute motor degrees, a selected-joint rotation handle, taught poses and vertical inverse kinematics. Teal is encoder feedback, orange is a preview. Simulation feedback follows commands perfectly; real motors do not.

For the vertical demo, leave the initial pose unchanged, open folder **4. Cartesian vertical IK lift**, choose +5 mm and press **Preview vertical IK from feedback**. Check the planned line and endpoint. Check the support/work-area box, press **Arm full motion**, then **Execute the planned vertical IK lift**. All of this is simulated when `--mock` is used.

For a different mesh or independent data session:

```powershell
.venv\Scripts\python.exe -m plexus app --mock --gripper slot --session slot_demo
```

Sliders and dragging only change the preview. Motor commands require an explicit execute action. The mock pick-and-place poses are invented demonstrations, not a route for your physical puzzle. The default session is now `centered_180`, which avoids reusing the older demo session. Read `MOTOR_MAPPING.md` before migrating an existing installation.

## Move to your real arm

Read **RUN_STEPS.md** before hardware operation. This is a software-tested starting implementation, not a physically commissioned robot. Physical calibration, raw limits, ring-center tool geometry, post clearance and grip retention must be measured on your arm.

```powershell
.venv\Scripts\python.exe -m plexus ports
.venv\Scripts\python.exe -m plexus diagnose --port COM7
.venv\Scripts\python.exe -m plexus app --hardware --port COM7 --gripper stock --session red_A_B
```

Replace COM7 with the follower arm's actual port. Startup and diagnostics are read-only. The 180°/raw-2048 reference and reported directions are preloaded. Measured limits and physical mapping verification are still required before full motion. Do not copy `config/demo.json` into `config/hardware.json`.

The configured motor order is **11, 12, 13, 1, 2, 3**, using Protocol 2.0 at 1 Mbps. Expected models remain XL430-W250 on IDs 11–13 and XL330-M288 on IDs 1–3; ID changes do not identify a different motor model. Diagnose and confirm these on your own arm. Config defaults expect extended-position mode 4. The program never changes IDs, modes or EEPROM. If your hardware differs, resolve that deliberately before calibration.

## What solves the straight-up problem

A ring on a post needs its **center** to rise along the post while its plane stays level. A higher joint pose by itself does not produce a straight tip path.

`plexus/ik.py` plans a smooth Z path with fixed X/Y, solves every sample with bounded IK, keeps the gripper command fixed, checks encoder rounding, then sends synchronized goals. For the five arm axes it constrains XYZ and the ring normal; rotation about that normal is free. Full six-dimensional pose control is not assumed.

A complete lift must solve before its execution. Automatic sequences preplan every vertical segment before the first movement. During execution, encoder-based forward kinematics checks lateral drift and tilt. These checks detect model-predicted error, not actual contact force, backlash or puzzle displacement.

**Your nominal radial clearance is only 0.53975 mm.** IK can reduce commanded sideways motion; it cannot remove physical play, a tilted post, a bad center offset or slipping jaws. First test 1–2 mm vertical travel with an unrestrained puzzle and an external measurement. Do not compensate by loosening error tolerances. See **IK_MATH_CHECK.md**.

## Files to use

| File | Purpose |
|---|---|
| `MOVE_ARM_COM7.md` | Direct single-joint manual control on COM7 |
| `MOTOR_MAPPING.md` | Updated IDs, direction table, centered reference and migration steps |
| `RUN_STEPS.md` | Windows setup, firmware, calibration, first lift and taught routes |
| `MONTH_PLAN.md` | Four-week plan with measurable internal deliverables for your PM |
| `IK_MATH_CHECK.md` | Path equations, Wolfram checks, tolerances and physical limitations |
| `PORTING_TO_MCXN947.md` | C/C++ transfer strategy and teammate communication contract |
| `config/sequence.ik.template.json` | Route using vertical IK for lift and lowering; 5 mm test values only |
| `config/sequence.template.json` | Basic teach/repeat route without Cartesian guarantees |
| `porting/motion_core.h` | Portable C/C++ reference primitives |
| `VALIDATION.md` | Tests performed and what remains untested |
| `examples/vertical_lift_model.png` | Calculated model preview, not a photograph or hardware measurement |
| `THIRD_PARTY.md` | Model sources and third-party license information |

The meshes in this code package are display models in meters. Use the separate print kits for printable millimeter STL/STEP files.
