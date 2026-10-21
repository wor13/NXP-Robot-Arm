# Move the real arm on COM7

This revision adds a separate manual encoder mode. It can execute an individual joint change without pretending the nominal IK model is physically calibrated.

## Run it

1. Extract this revision into a new folder. Run `setup_windows.bat` there if its `.venv` does not already exist.
2. Connect the usual motor supply and OpenRB USB cable. Close Wizard, Serial Monitor and any previous Python controller.
3. Double-click **run_arm_COM7.bat**, or run this in PowerShell from the project folder:

   ```powershell
   .\.venv\Scripts\python.exe -m plexus app --hardware --port COM7 --manual --web-port 8090 --gripper stock --session manual_COM7
   ```

4. Open **http://127.0.0.1:8090**. The banner says **MANUAL JOINTS**. The sliders start from actual encoder readings, not an automatic command to 180 degrees.
5. Physically support the arm and confirm the intended small motion has clearance. Check **Arm is supported; work area and power disconnect are ready**.
6. Click **Arm manual joint motion**. This checks motor models, modes, errors and encoder positions, preloads a current-position hold, and prepares the supported RAM profiles/watchdog. It does not move to the model's center pose.
7. Click **Copy feedback into preview**. Change **one** slider by **0.5 degrees** in a clear direction, then click **Execute preview**. For an initial base test, select ID 11; if its current reading is 180 degrees, change only that slider to 180.5 degrees. Use the current reading if it differs.
8. After completion, the preview automatically resynchronizes to feedback. Continue in small single-joint changes. The selected-joint raw jog buttons also work, at 1–8 counts per jog.

## Scope of manual mode

- No physical IK calibration or fabricated mechanical limit is claimed.
- Exactly one joint may change per Execute preview, at most **2 degrees per command**.
- Temporary limits are anchored within approximately **5 degrees of the launch readings**. Any existing measured limits make that window narrower, never wider. The window is a command bound, not proof of collision clearance.
- Other motors receive holds from fresh feedback, rather than stale preview values.
- ID 3 opens as degrees increase. With the reported closed-without-grip position at 180 degrees, this mode does not command below that closed position. If it starts below 180, it permits opening but not further closing below the launch reading. An actual ring grip is taught by closing from an open angle; it may be above 180.
- Temporary limits are kept in memory and are **not saved into hardware calibration**.
- Pick-and-place and vertical IK remain in calibrated hardware mode, launched without `--manual` after physical setup is complete.
- Arming does not change motor IDs or operating modes. This package still expects the configured motor models and mode 4 by default.

## If execution is rejected

Read the message at the top of the browser. It now distinguishes an unarmed controller, multiple changed sliders, a stale/large preview, temporary-window limits and device faults. After a rejected command, re-arm if the banner says DISARMED, copy feedback again and use a smaller change on one joint.

For a connection check:

```powershell
.\.venv\Scripts\python.exe -m plexus diagnose --port COM7
```

Expect IDs **11,12,13,1,2,3**. If a model, operating-mode, hardware-error or communication fault is reported, resolve that specific fault before moving. Do not replace measured configuration with invented values.

## If Execute preview appears to do nothing

This build is labeled **COM7 feedback 3** and shows the last action immediately below Execute preview and in the terminal. Use the manual launcher's new browser port **8090** so an older tab/server on 8080 is not mistaken for this build. Open the exact URL printed by the program if it selects another port.

Arm first, then change the slider. Arming synchronizes the preview to the real readings, so an edit made before arming is reset. An unchanged preview now reports that explicitly. The Execute callback reads motor-degree widget values directly rather than depending on a previously rendered model update.

To test the motor path without any browser, stop the GUI with Ctrl+C so COM7 is free, keep the intended 1-degree base motion clear, and run:

```powershell
.\.venv\Scripts\python.exe -m plexus jog --port COM7 --id 11 --delta-deg 1
```

This is a real movement request, not read-only diagnostics. It verifies the configured motors and commands ID11 approximately +1 degree while holding the others from fresh feedback. It prints the before/requested/present encoder counts, or the concrete error that stopped execution. It retains the same motor-error, mode, limit and tracking checks. The direct test accepts relative changes of 0.1–1.9 degrees and never marks IK calibration verified. Do not run it with Wizard or the GUI still owning COM7.

For example, a mocked test starting at raw 2048 returns requested/present 2059 (an 11-count change). That is sample output, not a prediction of the actual arm's reading. Share the actual RESULT or STOPPED line when troubleshooting. Add `--mock` only for a software-only test; leave it out to command the real arm.

Normal shutdown now reads a fresh healthy state, sends a current-position hold, and retires watchdog timers enabled by this session before releasing the port. It leaves torque on. A latched motor/watchdog fault or a failed read is reported and is not silently reset. An abrupt kill or lost connection can still leave a fault requiring inspection and deliberate recovery. The watchdog state is included in diagnostic snapshot output.
