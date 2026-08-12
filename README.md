# Apex Pathing
Check out our site at https://www.apexpathing.com/ for info about the project!

## Run the OpModes in FTCodeSim

The shared simulator setup is in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/sim`.
It registers the four drivetrain motors plus the custom Apex Pinpoint and telemetry adapters used
by all three current OpModes. Install and open
[AdvantageScope 26.0.2](https://github.com/Mechanical-Advantage/AdvantageScope/releases/tag/v26.0.2)
at least once before launching FTCodeSim. In PowerShell, run:

```powershell
.\gradlew.bat :TeamCode:testDebugUnitTest `
  --tests org.firstinspires.ftc.teamcode.sim.SimulateApexPathing
```

The simulated Driver Station presents `Apex Auto Test`, `Apex TeleOp Test`, and `Follower Tuner`.
Select one, then press Init and Start; FTCodeSim initializes and runs only that selected OpMode.
Stop it before selecting another. This is an interactive JUnit test and remains active until the
simulator windows are closed. Tuner output from a simulation is stored under
`build/ftcodesim-data` instead of the Android device's external storage.

FTCodeSim's original keyboard controls are preserved: arrows are the D-pad, WASD is the left
stick, `;` is gamepad A, `[` is B, `P` is X, and `-` is Y. Follower Tuner always opens its phase
options menu and requires `[` to accept the highlighted phase. Pressing Start first keeps the menu
open until a phase is explicitly selected. Locked phases cannot
be selected until the preceding phase has been tuned on the robot; the simulator unlocks all
phases so any phase can be selected as the starting point. After a phase's results are accepted,
Follower Tuner saves them and advances through every remaining phase in order. The simulation asks
you to use the red Stop button only after the final Velocity Feedback phase is complete.

Automatic PDS tuning uses a bounded relay-feedback test, rejects inconsistent oscillations, and
then validates the resulting gains with a capped point-to-point move. PDS and feedforward runs save
graph-ready CSV files beside `constants.json` (`FIRST/ApexPathing` on the Robot Controller and
`build/ftcodesim-data` in desktop simulation). Feedforward CSV rows include measured velocity and
acceleration plus fitted power and residuals; PDS CSV rows include target, position, error,
velocity, and commanded power over time.
