#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>

#include "vex.h"
#include <algorithm>

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                      \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.


// generating and setting random seed
void initializeRandomSeed(){
  int systemTime = Brain.Timer.systemHighResolution();
  double batteryCurrent = Brain.Battery.current();
  double batteryVoltage = Brain.Battery.voltage(voltageUnits::mV);

  // Combine these values into a single integer
  int seed = int(batteryVoltage + batteryCurrent * 100) + systemTime;

  // Set the seed
  srand(seed);
}



void vexcodeInit() {

  //Initializing random seed.
  initializeRandomSeed(); 
}


// Helper to make playing sounds from the V5 in VEXcode easier and
// keeps the code cleaner by making it clear what is happening.
void playVexcodeSound(const char *soundName) {
  printf("VEXPlaySound:%s\n", soundName);
  wait(5, msec);
}

#pragma endregion VEXcode Generated Robot Configuration

//Begin project code
competition Competition;

// Global Variables
controller Controller1 = controller(primary); // The main controller.
int autonomousSelection = 0; // Stores which autonomous routine to run.
vex::color teamColor = red; // Stores our team's color (red or blue).
int autonSide = 0; // 0 = Left, 1 = Right

// Configuration constants
constexpr int kAutonCount = 4;
constexpr int kDeadband = 1; // joystick deadband in percent (~0.5-1%)
constexpr int kHueBlueMin = 200;
constexpr int kHueBlueMax = 260;
// piston/pneumatic constants removed (not used)
// Slew rate limit (percent change per control loop iteration ~20ms). Increase to allow faster response.
constexpr int kSlewDeltaPerLoop = 10;

// UI color helpers (use explicit colors so names exist)
static const vex::color kLightGray = vex::color(200, 200, 200);
static const vex::color kGreen = vex::color(0, 200, 0);
static const vex::color kCyan = vex::color(0, 200, 200);
static const vex::color kBlack = vex::color(0, 0, 0);
static const vex::color kWhite = vex::color(255, 255, 255);

// little helper stuff
static inline int clampInt(int v, int lo, int hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline int applyDeadband(int v, int deadband = kDeadband) {
  return (std::abs(v) < deadband) ? 0 : v;
}

// Mix forward and turn into left/right. If either value would be outside [-100,100]
// we scale both down so we don't send invalid values to the motors.
static inline void computeArcade(int forward, int turn, int &leftOut, int &rightOut) {
  int left = forward + turn;
  int right = forward - turn;
  int maxVal = std::max(std::abs(left), std::abs(right));
  if (maxVal > 100) {
    left = left * 100 / maxVal;
    right = right * 100 / maxVal;
  }
  leftOut = clampInt(left, -100, 100);
  rightOut = clampInt(right, -100, 100);
}

// Apply a signed quadratic mapping so y = (1/100) * x^2 with preserved sign.
// This gives finer control around zero while still allowing full range at max input.
static inline int signedQuadratic(int v) {
  if (v == 0) return 0;
  int sign = (v > 0) ? 1 : -1;
  int mag = (v * v) / 100; // (1/100) * x^2
  return sign * clampInt(mag, 0, 100);
}

// Other mapping options. keep it simple and switchable.
static inline int linearMap(int v) { return clampInt(v, -100, 100); }
static inline int cubicMap(int v) {
  if (v == 0) return 0;
  int sign = (v > 0) ? 1 : -1;
  // (1/10000) * x^3, scaled back to -100..100
  int mag = (v * v * std::abs(v)) / 10000;
  return sign * clampInt(mag, 0, 100);
}
// Controller mapping modes (keep it compact: Linear, Quadratic, Cubic)
enum class ControlMap : int { Linear = 0, Quadratic = 1, Cubic = 2 };
ControlMap currentMap = ControlMap::Linear; // start snappy for quickest response

// Helper to apply the selected mapping
static inline int applyControlMap(int v) {
  switch (currentMap) {
    case ControlMap::Linear: return linearMap(v);
    case ControlMap::Quadratic: return signedQuadratic(v);
    case ControlMap::Cubic: return cubicMap(v);
    default: return signedQuadratic(v);
  }
}

// Simple hardware check UI shown in pre-auton
// verbose runHardwareCheck removed - use quickHardwareOK() for fast checks

// Lightweight quick-check moved below motor declarations

// --- Drivetrain Motor Configuration ---
// Left Motors: Ports ?
// The 'true' reverses the motor direction.
motor leftMotor1 = motor(PORT1, ratio6_1, true); // Bottom Motor
motor leftMotor2 = motor(PORT2, ratio6_1, false); // Top Motor
motor leftMotor3 = motor(PORT3, ratio6_1, true); // Front Motor
// Groups the left motors so they can be controlled as one.
motor_group LeftMotors = motor_group(leftMotor1, leftMotor2, leftMotor3);

// Right Motors: Ports ?
motor rightMotor1 = motor(PORT4, ratio6_1, false); // Bottom Motor
motor rightMotor2 = motor(PORT5, ratio6_1, true); // Top Motor
motor rightMotor3 = motor(PORT6, ratio6_1, false); // Front Motor
motor_group RightMotors = motor_group(rightMotor1, rightMotor2, rightMotor3);

// Sensor definitions
optical ColorSensor = optical(PORT7);

// Intake motors
// Ports assumed: PORT8 and PORT9. Change these if your hardware uses different ports.
motor intakeUp = motor(PORT8, ratio6_1, false);
motor intakeDown = motor(PORT9, ratio6_1, true);

// Lightweight, non-blocking hardware quick-check used in preAutonomous.
// Returns true if basic health checks pass, false otherwise.
static inline bool quickHardwareOK() {
  // Fast checks only: battery voltage and motor temperatures.
  double volts = Brain.Battery.voltage(vex::voltageUnits::volt);
  double ltemp = LeftMotors.temperature(fahrenheit);
  double rtemp = RightMotors.temperature(fahrenheit);

  // thresholds chosen to be conservative and fast
  const double kMinVoltage = 11.0;   // below this show warning
  const double kMaxMotorTemp = 150;  // Fahrenheit

  if (volts < kMinVoltage) return false;
  if (ltemp >= kMaxMotorTemp) return false;
  if (rtemp >= kMaxMotorTemp) return false;
  return true;
}

void preAutonomous(void) {
  Brain.Screen.clearScreen();
  // no pneumatics here anymore — pulling piston code removed

  // Show selection and allow confirm via controller ButtonA or timeout.
  int lastSelection = -1;
  // require explicit confirmation from the CONFIRM button or Controller1.ButtonA

  // track last team color so we only redraw when something changes
  vex::color lastTeamColor = teamColor;

  while (true) {
    // redraw UI when selection or color changes. draw explicit buttons so it's clear.
    if (autonomousSelection != lastSelection || teamColor != lastTeamColor) {
      // clear background (use actual screen size and safe color)
      Brain.Screen.setFillColor(kWhite);
      Brain.Screen.drawRectangle(0, 0, 480, 272);

      // quick hardware status indicator (top-right corner)
      bool hwOK = quickHardwareOK();
      const int statusSize = 18; // smaller indicator
      const int statusX = 480 - statusSize - 6;
      const int statusY = 6;
      // draw a small circular indicator to keep the UI compact
      if (hwOK) {
        Brain.Screen.setFillColor(kGreen);
        Brain.Screen.drawCircle(statusX + statusSize/2, statusY + statusSize/2, statusSize/2);
      } else {
        Brain.Screen.setFillColor(red);
        Brain.Screen.drawCircle(statusX + statusSize/2, statusY + statusSize/2, statusSize/2);
  Brain.Screen.setPenColor(kWhite);
  // tiny exclamation mark inside the circle
  Brain.Screen.printAt(statusX + 6, statusY + 12, "!");
      }

      const int btnW = 140;
      const int btnH = 60;
      const int leftX = 12;
      const int rightX = 328; // right column placement
      const int topY = 18;   // side buttons
      const int midY = 100;  // color buttons
      const int confirmW = 160;
      const int confirmX = (480 - confirmW) / 2;
      const int confirmY = 190;
      const int textOffsetX = 30;
      const int textOffsetY = 38;

    // Left button
    if (autonSide == 0) Brain.Screen.setFillColor(kGreen); else Brain.Screen.setFillColor(kLightGray);
    Brain.Screen.drawRectangle(leftX, topY, btnW, btnH);
    Brain.Screen.setPenColor(kBlack);
    Brain.Screen.printAt(leftX + textOffsetX, topY + textOffsetY, "LEFT");

    // Right button
    if (autonSide == 1) Brain.Screen.setFillColor(kGreen); else Brain.Screen.setFillColor(kLightGray);
    Brain.Screen.drawRectangle(rightX, topY, btnW, btnH);
    Brain.Screen.setPenColor(kBlack);
    Brain.Screen.printAt(rightX + textOffsetX, topY + textOffsetY, "RIGHT");

    // Red button
    if (teamColor == red) Brain.Screen.setFillColor(red); else Brain.Screen.setFillColor(kLightGray);
    Brain.Screen.drawRectangle(leftX, midY, btnW, btnH);
    Brain.Screen.setPenColor(kBlack);
    Brain.Screen.printAt(leftX + textOffsetX, midY + textOffsetY, "RED");

    // Blue button
    if (teamColor == blue) Brain.Screen.setFillColor(blue); else Brain.Screen.setFillColor(kLightGray);
    Brain.Screen.drawRectangle(rightX, midY, btnW, btnH);
    Brain.Screen.setPenColor(kBlack);
    Brain.Screen.printAt(rightX + textOffsetX, midY + textOffsetY, "BLUE");

    // Confirm button
    Brain.Screen.setFillColor(kCyan);
    Brain.Screen.drawRectangle(confirmX, confirmY, confirmW, 36);
    Brain.Screen.setPenColor(kBlack);
    Brain.Screen.printAt(confirmX + 32, confirmY + 24, "CONFIRM");

      lastSelection = autonomousSelection;
      lastTeamColor = teamColor;
    }

    // Touchscreen button handling: check which button rect was pressed
    if (Brain.Screen.pressing()) {
      int x = Brain.Screen.xPosition();
      int y = Brain.Screen.yPosition();
  const int btnW = 140;
  const int btnH = 60;
  const int leftX = 12;
  const int rightX = 328;
  const int topY = 18;
  const int midY = 100;
  const int confirmW = 160;
  const int confirmX = (480 - confirmW) / 2;
  const int confirmY = 190;
  const int hitPad = 8; // pad touch area so taps register easier

      // Left button (toggle side for easier use)
      if (x >= leftX - hitPad && x <= leftX + btnW + hitPad && y >= topY - hitPad && y <= topY + btnH + hitPad) {
        autonSide = 1 - autonSide;
        // visual feedback: flash button darker briefly
        Brain.Screen.setFillColor(kBlack);
        Brain.Screen.drawRectangle(leftX, topY, btnW, btnH);
        wait(100, msec);
        // force UI redraw so selection highlight updates
        lastSelection = -1;
        lastTeamColor = (vex::color)0;
      }
      // Right button
      if (x >= rightX - hitPad && x <= rightX + btnW + hitPad && y >= topY - hitPad && y <= topY + btnH + hitPad) {
        autonSide = 1 - autonSide;
        Brain.Screen.setFillColor(kBlack);
        Brain.Screen.drawRectangle(rightX, topY, btnW, btnH);
        wait(100, msec);
        lastSelection = -1;
        lastTeamColor = (vex::color)0;
      }
      // Red button
      if (x >= leftX - hitPad && x <= leftX + btnW + hitPad && y >= midY - hitPad && y <= midY + btnH + hitPad) {
        // toggle color for simpler interaction
        teamColor = (teamColor == red) ? blue : red;
        Brain.Screen.setFillColor(kBlack);
        Brain.Screen.drawRectangle(leftX, midY, btnW, btnH);
        wait(100, msec);
        lastSelection = -1;
        lastTeamColor = (vex::color)0;
      }
      // Blue button
      if (x >= rightX - hitPad && x <= rightX + btnW + hitPad && y >= midY - hitPad && y <= midY + btnH + hitPad) {
        teamColor = (teamColor == blue) ? red : blue;
        Brain.Screen.setFillColor(kBlack);
        Brain.Screen.drawRectangle(rightX, midY, btnW, btnH);
        wait(100, msec);
        lastSelection = -1;
        lastTeamColor = (vex::color)0;
      }
      // Confirm button pressed: show a modal "Are you sure?" with YES/NO
      if (x >= confirmX - hitPad && x <= confirmX + confirmW + hitPad && y >= confirmY - hitPad && y <= confirmY + 36 + hitPad) {
          // wait for the initial press to be released so the release doesn't cancel the modal
          while (Brain.Screen.pressing()) wait(15, msec);

          // draw modal centered
          const int modalW = 360;
          const int modalH = 140;
          const int modalX = (480 - modalW) / 2;
          const int modalY = (272 - modalH) / 2;
          Brain.Screen.setFillColor(kWhite);
          Brain.Screen.drawRectangle(modalX, modalY, modalW, modalH);
          Brain.Screen.setPenColor(kBlack);
          Brain.Screen.printAt(modalX + 40, modalY + 40, "Are you sure?");

        // YES/NO buttons inside modal
        const int btnW2 = 120;
        const int btnH2 = 50;
        const int yesX = modalX + 40;
        const int noX = modalX + modalW - 40 - btnW2;
        const int btnY = modalY + modalH - btnH2 - 20;

        Brain.Screen.setFillColor(kGreen);
        Brain.Screen.drawRectangle(yesX, btnY, btnW2, btnH2);
        Brain.Screen.setPenColor(kBlack);
        Brain.Screen.printAt(yesX + 30, btnY + 32, "YES");

        Brain.Screen.setFillColor(kLightGray);
        Brain.Screen.drawRectangle(noX, btnY, btnW2, btnH2);
        Brain.Screen.setPenColor(kBlack);
        Brain.Screen.printAt(noX + 35, btnY + 32, "NO");

        // wait for an explicit YES or NO tap. controller A = YES.
        bool decided = false;
        bool confirmed = false;
        while (!decided) {
          // controller A can confirm
          if (Controller1.ButtonA.pressing()) {
            confirmed = true; decided = true; // treat as YES
            // small debounce
            wait(150, msec);
            break;
          }
          if (Brain.Screen.pressing()) {
            int mx = Brain.Screen.xPosition();
            int my = Brain.Screen.yPosition();
            // wait for release to get a stable tap
            while (Brain.Screen.pressing()) wait(15, msec);
            // check if tap was on YES or NO
            if (mx >= yesX && mx <= yesX + btnW2 && my >= btnY && my <= btnY + btnH2) {
              confirmed = true; decided = true;
            } else if (mx >= noX && mx <= noX + btnW2 && my >= btnY && my <= btnY + btnH2) {
              confirmed = false; decided = true;
            } else {
              // ignore taps outside YES/NO — keep waiting
            }
          }
        }

        if (confirmed) {
          // show a tiny confirmation so driver knows it accepted
          Brain.Screen.clearScreen();
          Brain.Screen.setPenColor(kBlack);
          Brain.Screen.printAt(20, 40, "Confirmed");
          wait(250, msec);
          Brain.Screen.clearScreen();
          autonomousSelection = (teamColor == blue ? 2 : 0) + (autonSide == 1 ? 1 : 0);
          break; // exit preAutonomous
        } else {
          // cancelled: force redraw of the main UI
          lastSelection = -1;
          lastTeamColor = (vex::color)0;
        }
      }

      // wait for release so user can see the change
      while (Brain.Screen.pressing()) {
        wait(15, msec);
      }
    }

    // Confirm with controller ButtonA (brief press) — no auto-confirm
    if (Controller1.ButtonA.pressing()) {
      // small debounce for button press
      wait(150, msec);
      // set combined autonomousSelection so older code expecting 0..3 still works
      // mapping: 0 = Red Left, 1 = Red Right, 2 = Blue Left, 3 = Blue Right
      autonomousSelection = (teamColor == blue ? 2 : 0) + (autonSide == 1 ? 1 : 0);
      break; // exit preAutonomous and let competition framework continue
    }

    wait(15, msec);
  }
}

void autonomous(void) {
  // Code for the autonomous period goes here.R
}

void userControl(void) {
  // previous outputs for slew-rate limiting
  static int prevLeftOut = 0;
  static int prevRightOut = 0;

  // Timers and cached values to avoid expensive calls every loop
  static timer tempScaleTimer;
  static bool tempScaleInit = false;
  static int cachedTempScale = 100; // percent

  static timer colorTimer;
  static bool colorTimerInit = false;
  static timer colorMessageCooldown;
  static bool colorMsgInit = false;

  // Edge-detection state for mapping button
  static bool rightWasPressed = false;
  static bool mapPrintedOnce = false;

  // The main loop for driver control.
  while (true) {
    // Read controller inputs once per loop (reduce driver API overhead)
    int rawAxis3 = Controller1.Axis3.position(percent);
    int rawAxis1 = Controller1.Axis1.position(percent);
    bool btnR2 = Controller1.ButtonR2.pressing();
    bool btnL2 = Controller1.ButtonL2.pressing();
    bool btnRight = Controller1.ButtonRight.pressing();
    bool btnA = Controller1.ButtonA.pressing();

    // Apply deadband once
    int rawForward = applyDeadband(rawAxis3);
    int rawTurn = applyDeadband(rawAxis1);

    // allow driver to cycle mapping with right arrow (edge-detect)
    if (!mapPrintedOnce) {
      // print initial map on controller row 2
      const char *mapNameInit = "?";
      switch (currentMap) {
        case ControlMap::Linear: mapNameInit = "Linear"; break;
        case ControlMap::Quadratic: mapNameInit = "Quadratic"; break;
        case ControlMap::Cubic: mapNameInit = "Cubic"; break;
      }
      Controller1.Screen.setCursor(2, 1);
      Controller1.Screen.clearLine();
      Controller1.Screen.print("Map: %s", mapNameInit);
      mapPrintedOnce = true;
    }

    if (btnRight) {
      if (!rightWasPressed) {
        // cycle mapping
        int next = (static_cast<int>(currentMap) + 1) % 3; // only 3 maps
        currentMap = static_cast<ControlMap>(next);

        // print new map immediately to controller so driver knows it changed
        const char *mapName = "?";
        switch (currentMap) {
          case ControlMap::Linear: mapName = "Linear"; break;
          case ControlMap::Quadratic: mapName = "Quadratic"; break;
          case ControlMap::Cubic: mapName = "Cubic"; break;
        }
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.clearLine();
        Controller1.Screen.print("Map: %s", mapName);
      }
      rightWasPressed = true;
    } else {
      rightWasPressed = false;
    }

    // apply the currently selected mapping
    int forwardSpeed = applyControlMap(rawForward);
    int turnSpeed = applyControlMap(rawTurn);

    int leftMotorSpeed = 0;
    int rightMotorSpeed = 0;
    computeArcade(forwardSpeed, turnSpeed, leftMotorSpeed, rightMotorSpeed);

    // Update cached temperature-based scaling at a lower rate (every 200ms)
    if (!tempScaleInit) { tempScaleTimer.reset(); tempScaleInit = true; }
    if (tempScaleTimer.time(msec) > 200) {
      double ltemp = LeftMotors.temperature(fahrenheit);
      double rtemp = RightMotors.temperature(fahrenheit);
      double avg = (ltemp + rtemp) / 2.0;
      if (avg < 100.0) cachedTempScale = 100;
      else if (avg < 115.0) cachedTempScale = 98;
      else if (avg < 130.0) cachedTempScale = 95;
      else cachedTempScale = 90;
      tempScaleTimer.reset();
    }

    int tempScale = cachedTempScale;
    int leftScaledTarget = leftMotorSpeed * tempScale / 100;
    int rightScaledTarget = rightMotorSpeed * tempScale / 100;

    // For fastest response, bypass slew limiting (can be enabled if desired)
    int leftScaled = leftScaledTarget;
    int rightScaled = rightScaledTarget;

    prevLeftOut = leftScaled;
    prevRightOut = rightScaled;

    // Run the motors with clamped/scaled values.
    LeftMotors.spin(forward, leftScaled, percent);
    RightMotors.spin(forward, rightScaled, percent);

    // --- Color Sorter Logic (checked at lower rate to reduce latency) ---
    if (!colorTimerInit) { colorTimer.reset(); colorTimerInit = true; }
    if (!colorMsgInit) { colorMessageCooldown.reset(); colorMsgInit = true; }
    if (colorTimer.time(msec) > 50) { // check color ~20Hz
      if (ColorSensor.isNearObject()) {
        double objectHue = ColorSensor.hue();
        bool isBlueBall = (objectHue > 200 && objectHue < 260);
        bool isRedBall = (objectHue < 30 || objectHue > 330);
        if ((teamColor == red && isBlueBall) || (teamColor == blue && isRedBall)) {
          // Avoid spamming the controller screen; only print once per 500ms
          if (colorMessageCooldown.time(msec) > 500) {
            Controller1.Screen.setCursor(2,1);
            Controller1.Screen.print("Other team's ball detected");
            colorMessageCooldown.reset();
          }
        }
      }
      colorTimer.reset();
    }

    // --- Motor Temperature on Controller (kept at 5s as before) ---
    static timer tempTimer;
    static bool tempTimerInit = false;
    if (!tempTimerInit) { tempTimer.reset(); tempTimerInit = true; }
    if (tempTimer.time(msec) > 5000) {
      double averageTemperature = (LeftMotors.temperature(fahrenheit) + RightMotors.temperature(fahrenheit)) / 2.0;
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.clearLine();
      Controller1.Screen.print("Avg Temp: %.1fF", averageTemperature);
      tempTimer.reset();
    }

    // --- Intake Controls ---
    if (btnR2) {
      intakeUp.spin(forward, 100, percent);
      intakeDown.spin(forward, 100, percent);
    } else if (btnL2) {
      intakeUp.spin(reverse, 100, percent);
      intakeDown.spin(reverse, 100, percent);
    } else {
      intakeUp.stop(coast);
      intakeDown.stop(coast);
    }

    // loop delay: reduce to 5ms for lower input latency while keeping CPU reasonable
    wait(5, msec);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Set up the competition functions.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(userControl);

  // Run the pre-autonomous routine.
  preAutonomous();

  // Prevents the program from ending.
  while (true) {
    wait(100, msec);
  }
}
