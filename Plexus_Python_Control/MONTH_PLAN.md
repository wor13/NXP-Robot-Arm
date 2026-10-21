# One-month subsystem plan for the project manager

**Owner:** Thomas Tran  
**Window:** October 21–November 17, 2026  
**Semester objective:** Demonstrate a reliable gripper and supervised Python-controlled ring pick-and-place using the OpenRB-150, with measured vertical lift/lowering that does not move the puzzle. Deliver the tested motion logic, calibration, logs and C/C++ export for MCXN947 integration.

Python will run on the PC through the OpenRB-150's USB-to-DYNAMIXEL bridge. The 947 integration depends on the team providing a working motor communication interface. I will develop and document the motion interface in parallel so communication does not delay the Python prototype.

| Week | Work | Internal deliverables | Evidence that I am on target |
|---|---|---|---|
| **1: Oct 21–27** | Print/fit the reinforced jaws; apply and test pads; choose the jaw variant from physical pickup tests. Verify communication with all six motors. Measure software limits, joint mapping, ring-center tool offset, post height and required clearance. Bring up interactive 3D control and test a small vertical IK lift. | Installed gripper; jaw test comparison; read-only diagnostics for all six IDs; backed-up calibration; first 1–2 mm lift video and lateral-motion measurement; draft 947 read/write interface. | All six motors respond without hardware errors. Jaws fit without interference. Each ring size is retained through 10 supervised short lifts without a drop. At least one centered ring lifts 1–2 mm without moving the puzzle. Any unresolved slip, drift or binding is reported before increasing travel. |
| **2: Oct 28–Nov 3** | Complete straight lift to measured post clearance, clear transfer, vertical lowering and release for one source/target profile. Add taught entry/exit poses, abort handling and logging. Start coverage of the other ring sizes. | Running Python prototype; one approved hardware-taught route; measured lift distance; log and video of a complete cycle; repeatability results; confirmed communication-team status. | **First complete Python pick-and-place by Nov 3.** Baseline route passes 20 consecutive supervised cycles with no drops, binding, manual correction or observable puzzle movement. Record cycle time and lateral measurement. If the gate fails, report the cause and revised recovery date. |
| **3: Nov 4–10** | Qualify every ring size and agreed source/target route. Refine jaw retention and calibration from measured failures. Freeze a demonstrated Python version. Export motion frames and prepare C/C++ handoff. | Ring/route test matrix; numbered software/configuration backup; failure log; exported C header; portable control primitives; teammate interface contract and replay checks. | Each ring size passes 20 consecutive cycles on the baseline route; each additional required route passes 10 consecutive cycles. Fault tests stop the software without automatically releasing a held ring. Team can compile the export and compare its counts/timing against Python. |
| **4: Nov 11–17** | Integrate with MCXN947 if communication is ready. Replay the qualified route with matching IDs, coordinates, profiles, feedback checks and timing. Requalify physically and prepare final evidence. | C/C++ integration or a clearly identified communication blocker; Python/C parity results; final demo; concise setup/recovery instructions; remaining-work list. | On the 947, the baseline route passes 20 consecutive supervised cycles under the same mechanical criteria. If communication is blocked, deliver the working Python demonstration plus a compiling C/C++ data/control handoff and a transport mock, with the remaining dependency stated explicitly. |

These trial counts are internal milestone gates, not a claim of production-level reliability. The measured lateral-error budget must leave margin inside the nominal **0.53975 mm radial clearance**, including centering, tilt, backlash, flex and tool calibration. No sideways contact or puzzle motion is acceptable in the demonstration. We will agree on a practical measurement resolution and the complete required route list during Week 1.

## Internal reporting cadence

- **Two brief demonstrations per week:** show the current physical behavior, not only code or CAD progress.
- **One weekly status update:** completed deliverables, pass/total trials, measured lateral displacement, the next deliverable, and any dependency with an owner/date.
- **Green:** the weekly gate passed. **Yellow:** a gate is at risk with a specific recovery task/date. **Red:** a gate failed or a dependency prevents progress.
- Escalate a blocker lasting more than two working days. A failed pickup or bound lift resets that route's consecutive-success count and must be logged with its cause and corrective action.

## Communication handoff dependency

By **Oct 27**, agree on the MCXN947 transport architecture and functions to read all six signed positions/status values and issue synchronized goals. Target a bus-level read/write demonstration by **Nov 3**, and a usable feedback/goal interface by **Nov 10** for Week 4 integration. The teammate responsible for communication owns those dates; I own the motion planner, tested route, calibration format and comparison data.

If the interface arrives later, I will preserve the working Python/OpenRB demonstration, continue physical reliability work, and report exactly which embedded functions are still blocked. I will not claim a completed 947 transfer based on compiling code alone.

## First work block

1. Run the mock 3D viewer and vertical-IK preview.
2. Verify the real follower bridge and all six IDs in read-only diagnostics.
3. Check jaw fit/pad retention, then measure conservative raw limits and tool geometry.
4. Demonstrate a measured 1–2 mm lift before attempting full post clearance.
