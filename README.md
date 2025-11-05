Short summary: single-file VEX V5 competition robot. Primary source: src/main.cpp. Focus changes there.

What this repo is (big picture)
- VEX V5 competition framework: main() → vexcodeInit() → preAutonomous() → Competition.autonomous(autonomous) / Competition.drivercontrol(userControl).
- One primary implementation file: src/main.cpp (drivetrain motor groups, intake motors, optical color sensor, touchscreen pre-auton UI, driver loop).
- autonomous() is a placeholder — autonomousSelection controls which routine to run.

Key places to read/edit
- src/main.cpp — everything. Look near the top for constants and motor/sensor declarations:
  - leftMotor1..3 (PORT1..PORT3), rightMotor1..3 (PORT4..PORT6), ColorSensor (PORT7), intakeUp (PORT8), intakeDown (PORT9).
- preAutonomous() — touchscreen UI, confirm sets autonomousSelection:
  - Snippet: autonomousSelection = (teamColor == blue ? 2 : 0) + (autonSide == 1 ? 1 : 0);
  - Meaning: 0=RedLeft, 1=RedRight, 2=BlueLeft, 3=BlueRight.
- userControl() — main driver loop. Key helpers: applyControlMap(), computeArcade(), signedQuadratic(), cubicMap(), quickHardwareOK().

Project-specific patterns & conventions
- Non-blocking main loop: use small waits (wait(5, msec)) and timers (timer objects) for low-rate tasks — avoid long/blocking waits.
- Low-rate checks use timers: tempScaleTimer, colorTimer, tempTimer, colorMessageCooldown.
- Controller mapping is edge-detected (ButtonRight toggles currentMap). currentMap enum: Linear, Quadratic, Cubic.
- Motor outputs are clamped and scaled; computeArcade() ensures outputs remain in [-100,100].
- playVexcodeSound(...) prints "VEXPlaySound:<name>" via printf for observable messages in serial/console.

How to build/run
- This repo has no CI/build scripts. Use VEXcode V5 (official IDE) or VEX toolchain to compile and upload the binary to the brain.
- Do not remove vexcodeInit() or the Competition.* hooks in main().
- For quick debug messages, use Brain.Screen, Controller1.Screen, or playVexcodeSound().

Common edits you will be asked to make
- Add/modify autonomous routines in autonomous() and select by autonomousSelection.
- Change motor/sensor ports near the top of src/main.cpp.
- Tune behavior via constants at the top: kDeadband, kSlewDeltaPerLoop, kHueBlueMin/kHueBlueMax, temperature thresholds, default currentMap.

Debugging tips
- Use Brain.Screen/Controller1.Screen for short messages; playVexcodeSound prints to serial as "VEXPlaySound:...".
- Preserve responsiveness: keep loops non-blocking and use timers for low-frequency work.

