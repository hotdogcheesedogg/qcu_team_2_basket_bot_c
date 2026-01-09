#include "vex.h"
using namespace vex;

// ------------------- GLOBAL OBJECTS -------------------
brain       Brain;
controller  Controller1;
competition Competition;

// ------------------- MOTORS -------------------
motor right_motor_f = motor(PORT1, ratio18_1, true);
motor right_motor_b = motor(PORT5, ratio18_1, true);
motor_group right_drive(right_motor_f, right_motor_b);

motor left_motor_f  = motor(PORT3, ratio18_1, false);
motor left_motor_b  = motor(PORT4, ratio18_1, false);
motor_group left_drive(left_motor_f, left_motor_b);

// ------------------- AUTO MODES -------------------
enum AutoMode { STRAIGHT, ROTATE };

// ------------------- STEP STRUCTURE -------------------
struct Step {
    AutoMode mode;
    int value;      // distance in cm or rotation in deg
    int speed;      // percentage 0-100
    bool negative;  // reversed
};

// ------------------- PRESETS -------------------
const int MAX_PRESETS = 3;
Step presets[MAX_PRESETS][50];  // 50 steps per preset
int stepCount[MAX_PRESETS] = {0,0,0};
int currentPreset = 0;
int currentStep   = 0;

// -------- TUNING CONSTANTS --------
const double TICKS_PER_CM  = 15;
const double TICKS_PER_DEG = 5;

// ------------------- DISPLAY -------------------
void displayInfo() {
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);
    Brain.Screen.setFont(mono20);

    Brain.Screen.printAt(10, 20, "Preset: %d / %d", currentPreset+1, MAX_PRESETS);
    Brain.Screen.printAt(10, 50, "Step: %d / %d", currentStep+1, stepCount[currentPreset]);

    if(stepCount[currentPreset] > 0) {
        Step &s = presets[currentPreset][currentStep];

        // Mode
        Brain.Screen.printAt(10, 80, s.mode==STRAIGHT ? "Mode: STRAIGHT" : "Mode: ROTATE");

        // Value
        if(s.mode == STRAIGHT)
            Brain.Screen.printAt(10, 110, "Value: %d cm", s.value);
        else
            Brain.Screen.printAt(10, 110, "Value: %d deg", s.value);

        // Speed
        Brain.Screen.printAt(10, 140, "Speed: %d%%", s.speed);

        // Negative flag
      Brain.Screen.printAt(10, 170, "Neg: "); 
      Brain.Screen.print(s.negative ? "Yes" : "No");

    }

    // Motor temps & battery
    Brain.Screen.printAt(10, 200, "L Temp: %.1fC", left_motor_f.temperature(celsius));
    Brain.Screen.printAt(10, 230, "R Temp: %.1fC", right_motor_f.temperature(celsius));
    Brain.Screen.printAt(10, 260, "Battery: %.1fV", Brain.Battery.capacity());
}

// ------------------- RUN SINGLE STEP -------------------
void runStep(Step &s) {
    left_drive.setVelocity(s.speed, pct);
    right_drive.setVelocity(s.speed, pct);

    double ticks = (s.mode==STRAIGHT ? s.value*TICKS_PER_CM : s.value*TICKS_PER_DEG);
    if(s.negative) ticks *= -1;

    if(s.mode==STRAIGHT) {
        left_drive.spinFor(fwd, ticks, deg, false);
        right_drive.spinFor(fwd, ticks, deg, true);
    } else {
        left_drive.spinFor(fwd, ticks, deg, false);
        right_drive.spinFor(reverse, ticks, deg, true);
    }

    left_drive.stop(brake);
    right_drive.stop(brake);
}

// ------------------- RUN PRESET -------------------
void runPreset(int presetIdx) {
    for(int i=0; i<stepCount[presetIdx]; i++) {
        Step &s = presets[presetIdx][i];
        runStep(s);
          wait(1000, msec);
    }
   
}

// ------------------- AUTONOMOUS TUNER -------------------
void autonomous() {
    while(true) {
        displayInfo();

        // -------- Preset Selection --------
        if(Controller1.ButtonR2.pressing()) { currentPreset = (currentPreset+1)%MAX_PRESETS; wait(300,msec); }
        if(Controller1.ButtonL2.pressing()) { currentPreset = (currentPreset+MAX_PRESETS-1)%MAX_PRESETS; wait(300,msec); }

        // -------- Step Navigation --------
        if(Controller1.ButtonR1.pressing() && stepCount[currentPreset]>0) { currentStep=(currentStep+1)%stepCount[currentPreset]; wait(200,msec); }
        if(Controller1.ButtonL1.pressing() && stepCount[currentPreset]>0) { currentStep=(currentStep-1+stepCount[currentPreset])%stepCount[currentPreset]; wait(200,msec); }

        // -------- Add/Delete Step --------
        if(Controller1.ButtonA.pressing()) { // Add step
            Step &s = presets[currentPreset][stepCount[currentPreset]++];
            s.mode = STRAIGHT;
            s.value = 100;
            s.speed = 50;
            s.negative = false;
            currentStep = stepCount[currentPreset]-1;
            wait(200,msec);
        }
        if(Controller1.ButtonB.pressing() && stepCount[currentPreset]>0) { // Delete step
            for(int i=currentStep; i<stepCount[currentPreset]-1; i++)
                presets[currentPreset][i] = presets[currentPreset][i+1];
            stepCount[currentPreset]--;
            if(currentStep >= stepCount[currentPreset]) currentStep = stepCount[currentPreset]-1;
            wait(200,msec);
        }

        // -------- Modify Current Step --------
        if(stepCount[currentPreset]>0) {
            Step &s = presets[currentPreset][currentStep];

            // Mode & Negative
            if(Controller1.ButtonX.pressing()) { s.mode = (s.mode==STRAIGHT)?ROTATE:STRAIGHT; wait(200,msec); }
            if(Controller1.ButtonY.pressing()) { s.negative = !s.negative; wait(200,msec); }

            // Value adjustments
            if(Controller1.ButtonUp.pressing())    { s.value += 50; wait(150,msec); }
            if(Controller1.ButtonDown.pressing())  { s.value -= 50; wait(150,msec); }
            if(Controller1.ButtonRight.pressing()) { s.value += 10; wait(150,msec); }
            if(Controller1.ButtonLeft.pressing())  { s.value -= 10; wait(150,msec); }
            int axis = Controller1.Axis1.position(pct);
            if(axis > 80) s.value += 1;
            if(axis < -80) s.value -= 1;

            // Speed adjustments
            int axis2 = Controller1.Axis2.position(pct);
            if(axis2 > 80) s.speed += 5;
            if(axis2 < -80) s.speed -= 5;
            if(s.speed>100) s.speed=100;
            if(s.speed<0)   s.speed=0;
        }

        // -------- Run Preset --------
        if(Controller1.Axis3.position(pct) > 90 && stepCount[currentPreset]>0) {
            runPreset(currentPreset);
            wait(300,msec);
        }

        wait(20, msec);
    }
}

// ------------------- DRIVER CONTROL -------------------
void usercontrol() {
    while(true) { wait(100, msec); }
}

// ------------------- MAIN -------------------
int main() {
    displayInfo();
    autonomous();
    while(true) wait(100, msec);
}
