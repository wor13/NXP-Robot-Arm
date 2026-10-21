# Python to MCXN947 C/C++ handoff

## Keep the validated motion behavior

Python now runs on the PC, through the OpenRB-150 factory USB-to-DYNAMIXEL bridge. The local viewer, SciPy solver and Python SDK are PC components. The FRDM-MCXN947 firmware needs its own board support, transport, timing and state machine.

The fastest first port is to **solve IK offline in Python and export the qualified timed joint goals**. Replay those same count values on the 947 with feedback and stop checks. This preserves the tested motion without first porting SciPy or implementing a new embedded IK solver. New peg coordinates or geometry require new verified plans and exports.

Do not treat the 947 as a drop-in replacement until the team's actual communication path is established. It may use a designed TTL half-duplex motor interface or another agreed bridge. USB host operation and compatibility with an OpenRB USB device require explicit implementation if that architecture is chosen. This package does not assume the 947 already has a working USB/TTL path.

## Interface to agree with the communication teammate

| Operation | Required behavior |
|---|---|
| `read_state()` | Return six complete, current records: ID, signed 32-bit raw position, torque, hardware error and watchdog state; include timing/communication validity |
| `read_configuration()` | Confirm models, IDs, baud, operating modes and drive/profile configuration match the qualified Python setup |
| `sync_goal(int32_t raw[6])` | Send one synchronized six-motor goal group using the agreed reliable transport; preserve signed counts and order |
| `prepare_hold()` | Read current positions, verify configuration/limits/errors, preload current goals and apply the same supported RAM profiles/watchdog before enabling permitted torque |
| `monotonic_time_us()` | Monotonic clock for motion deadlines and feedback timing |
| `stop_requested()` | Prompt local stop input; integrate the team's physical disconnect independently |
| `report_fault()` | Latch/record a fault, abort the remaining sequence, attempt a fresh-position hold only with valid feedback, require deliberate recovery |

IDs are ordered **11,12,13,1,2,3**. The export obtains these IDs from the configuration, never from the old 11–16 assumption. Model neutral is 0 rad at motor 180°/raw 2048; raw motor goals remain absolute signed counts. Do not convert extended signed counts into unsigned 0–4095 values. Protocol 2.0 wire encoding for a negative 32-bit goal uses two's complement. `-152` encodes as bytes `68 FF FF FF` in hexadecimal, little-endian.

Useful common control-table addresses for the configured motor models are goal position 116 (4 bytes), present position 132 (4), torque enable 64 (1), hardware error 70 (1), watchdog 98 (1), profile acceleration 108 (4), and profile velocity 112 (4). Verify against the [XL430](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/) and [XL330](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/) manuals and actual installed firmware. The models differ in effort/current information; do not interpret a generic raw effort field as calibrated grip force.

## Export a physically qualified session

From the extracted project folder:

```powershell
.venv\Scripts\python.exe -m plexus export --session red_A_B --output porting\generated_poses.h
```

Export requires verified hardware calibration, hardware-taught poses with the current calibration fingerprint, and approval matching the current sequence. A route containing IK also requires a verified tool offset and must solve again during export. Mock demonstration data is rejected.

The generated header contains:

- `plexus_ids`: motor order.
- `plexus_entry`: six-count starting pose used for export.
- `plexus_frames[]`: `{pose, dt_us}` for every command sample, including precomputed Cartesian lifts and dwell intervals.
- `plexus_min` and `plexus_max`: qualified raw software bounds.
- `plexus_frame_count`: number of timed samples.

Copy the generated header together with `motion_core.h`. Archive the matching `hardware.json`, `poses.json`, `sequence.json`, model version and physical test results. Inspect memory usage on the 947 before allocating a large route. The code exports an array, not an unbounded streaming protocol.

## Embedded execution requirements

1. Establish current encoder coordinates, supported torque state and the same motor configuration as the qualified Python session. Never automatically change EEPROM/modes on startup. Reconfirm calibration after resets.
2. Verify the arm is already at the entry pose. Do not invent a motion from an arbitrary boot position to `plexus_entry`.
3. Check every exported frame against raw limits, maximum increments and feedback tracking before transmission. Keep independent communication/error/torque/watchdog checks throughout execution.
4. Send each frame's six goals together, then maintain its `dt_us` interval while continuing feedback/watchdog servicing. Use monotonic deadlines; a scheduler overrun must fault or be handled deliberately, not silently skip through a backlog.
5. Retain the velocity-based RAM profiles and watchdog settings used during qualification. Their device units are not Python counts/second. The exported `dt_us` and these device profiles both affect the physical motion.
6. Require final arrival/settling and grip/placement supervision. A command write alone is not arrival.
7. On fault, abort remaining frames. Do not advance to an opening/release action after a failed lift. Hold only from fresh valid feedback. Do not automatically torque off a gravity-loaded arm.

The exported array contains command positions, not the Python Cartesian feedback guard. To retain that guard on the 947, port the URDF-equivalent forward kinematics and carry the lift anchor/normal plus phase metadata into the embedded state machine. Until that is implemented, the port has reduced monitoring and must be treated accordingly and physically requalified. The current header alone is **not a feature-complete equivalent** of Python control.

`motion_core.h` supplies C/C++ structs, raw-limit checking, arm/grip masking, quintic interpolation and duration helpers. It contains no NXP HAL, motor driver, clock, feedback loop, fault latch or autonomous ring sequence. Those functions depend on the team's hardware interface. Offline-exported frames can be replayed directly; do not apply a second quintic curve over the full baked IK lift.

## Verification before the final demonstration

- Compile the shared header and generated data as C or C++ for the 947 project.
- Compare all transmitted raw targets and intervals against the Python export using a transport mock or bus logs.
- Prove communication loss, stale feedback, unexpected torque-off, limit violation and stop handling before a loaded transfer.
- Test supported unloaded motions, then a small measured vertical lift, then the qualified full route.
- Repeat the same ring/route acceptance matrix used for Python. Compilation and numerical parity do not establish mechanical pickup reliability.

If motor communication is delayed, keep the demonstrated Python/OpenRB system working and deliver the compiling data/control handoff plus a transport mock. Report the missing transport functions explicitly rather than claiming the 947 can operate the arm.
