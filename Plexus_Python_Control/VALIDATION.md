# Validation record

Date: October 21, 2026. Environment: Linux x86_64, CPython 3.12. No physical arm, OpenRB-150, MCXN947 or puzzle was connected.

## Results

**69 automated checks passed** in the Execute feedback revision (`69 passed in 49.70s`). They cover:

- Manual uncalibrated joint execution on all six axes, COM7 default, temporary-limit isolation, preservation of measured limits, single-joint/2-degree checks, fresh-feedback holds, closed-gripper bound, and rejection of IK/automatic playback in manual mode.
- Updated IDs 11,12,13,1,2,3 in both configurations, SDK read/write groups and compiled C/C++ exports.
- All-180°/raw-2048 neutral conversions, absolute motor-degree display, extended negative/multi-turn values, direction derivatives in the model, gripper opening direction, duplicate/reserved ID rejection and feedback ID ordering.
- Signed position conversion, two's-complement wire bytes and raw/angle round trips, including negative references.
- Configuration, finite goals, raw limits and missing-calibration rejection.
- Quintic joint trajectory endpoints/increments and arm/gripper masking.
- Successful mock movement and complete mock sequence execution.
- Stalls, stop requests, motor error, lost communication and stale-feedback hold prevention.
- Raw-jog-only restrictions and exact taught-route approval/provenance invalidation.
- Required entry-pose checks and preservation of grip commands during arm movement.
- Forward kinematics, mimic joints and all three mesh variants.
- An independent comparison against yourdfpy's URDF forward kinematics at mixed angles, for all seven moving links. yourdfpy is an optional developer check, not a runtime dependency.
- Vertical IK fixed XY/normal/gripper constraints, encoder rounding and sampled interpolation bounds.
- Tilted/unreachable start rejection, calibration-change invalidation, tool verification and lateral-drift abort.
- Successful preplanned vertical execution through the mock transport.
- Complete-route IK preplanning failure before a gripping or moving action; only a fresh-position fault hold is permitted.
- SDK arming preflight rejection of a wrong model, operating mode or time-based-profile configuration before register writes.
- Generated motion tables compiling and running as **C11 and C++17** on the host; shared quintic interpolation matching Python count results.

The local Viser server was launched with the revised controls and stock meshes. A direct application smoke check verified six initial 180° sliders, motor/model slider conversions, the new motor order, stock-jaw selection, and zero motor writes during startup or preview changes. The test closed the server normally. This checked Python UI state, not browser rendering. A software HTTP smoke check returned **HTTP 200** and served 818,656 bytes of viewer content. This verifies startup/content serving. It does not verify rendering or mouse interaction in a real Windows browser. A browser screenshot was not available; the supplied PNG is calculated geometry and data, not a browser screenshot.

## COM7 Execute preview callback check

A separate application smoke check used the **hardware-mode UI with a mocked motor transport**, blank measured limits and unverified calibration. It invoked the actual registered **Arm manual joint motion** and **Execute preview** callbacks. A +0.5-degree ID11 preview produced only ID11 raw 2054, with the other five motors held at raw 2048. It checked refreshed preview/anchor state, no startup goal writes, unchanged on-disk calibration, and HTTP 200. This verifies the software button/controller path, not actual Windows/COM7 motor operation.

## Execute feedback revision

The revised Execute callback reads the current motor-degree widget values directly, independently of a possibly delayed model-render callback. A regression test deliberately leaves the model-angle cache stale and verifies that the visible +1-degree ID11 request still reaches the mock encoder target. Additional checks cover visible support/unchanged-preview errors, terminal request/result output, the direct `jog` command, invalid command rejection, and healthy-versus-faulted watchdog cleanup. Browser render behavior and real COM7 motor operation still require the user's test output.

The launcher uses port 8090; the UI and terminal identify **COM7 feedback 3** and the actual serial port. The printed browser port is taken from the running server so an automatically selected alternate port is reported accurately.

## +5 mm model example

Using the revised `config/demo.json`, raw `[2048,2048,2048,2048,2048,2048]`, nominal zero model angles and the stock-jaw tool offset:

| Quantity | Computed result |
|---|---:|
| Requested rise | 5 mm |
| Duration | 4.6875 s |
| Command samples | 47 |
| Largest rounded-goal XY displacement | 0.076234 mm |
| Largest rounded-goal Z error | 0.118024 mm |
| Largest normal change | approximately 0.000001° |

These values measure the mathematical model, **not hardware accuracy**. Perfect mock feedback omits delay, backlash, slip and flex. The model result must not be compared directly to clearance as proof that a real lift is safe or reliable.

Wolfram independently returned zero horizontal desired velocity, zero endpoint speed/acceleration, the quintic peak factors and 0.53975 mm nominal radial clearance. See `IK_MATH_CHECK.md` for the exact evaluated expression.

## Run the checks yourself

```powershell
.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
.venv\Scripts\python.exe -m pytest -q
```

The optional yourdfpy comparison skips if that extra package is absent. The C/C++ compilation check skips if host compilers are absent. These are developer checks; neither a compiler nor yourdfpy is required for normal control. Install yourdfpy separately only if you want that independent cross-check and its extra dependencies.

## Not yet verified

- Windows x64 or ARM64 installation and live browser interaction.
- Compiling/uploading the official OpenRB bridge in Arduino IDE.
- The user's reported motor IDs/directions are incorporated; their live bus responses, modes, power path, timing and physical behavior have not been independently tested.
- Mechanical mounting, pad retention, jaw strength, ring mass and grip holding force.
- Measured raw bounds, joint reference accuracy, link geometry and actual tool offset/normal.
- Post verticality, full lift distance, lateral accuracy, backlash, puzzle displacement and real pickup repeatability.
- Physical emergency stop/hold behavior under communication loss or power failure.
- Compiling for the actual MCXN947 target, its communication driver, scheduler, memory capacity and loaded motion.

These are commissioning tasks with measurable gates in `MONTH_PLAN.md`. This release should be described as a software-tested implementation ready for supervised hardware setup, not a physically validated finished arm controller.
