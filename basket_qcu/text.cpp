#include "vex.h"
using namespace vex;

// Define a minimal vexcodeInit if the generated function is not present.
void vexcodeInit() {}

// ------------------- BRAIN & CONTROLLER -------------------
brain Brain;

// ------------------- MOTORS & DRIVE -------------------
motor right_motor_f  = motor(PORT1,  ratio18_1, true);
motor right_motor_b  = motor(PORT5,  ratio18_1, true);
motor_group right_drive_smart(right_motor_f, right_motor_b);

motor left_motor_f = motor(PORT3, ratio18_1, false);
motor left_motor_b = motor(PORT4, ratio18_1, false);
motor_group left_drive_smart(left_motor_f, left_motor_b);

drivetrain driveTrain(left_drive_smart, right_drive_smart,
    319.19, 317.5, 300, mm, 0.2857142857142857);

    // Top long goal
motor motor6 = motor(PORT6, ratio18_1, true);
// Inside middle
motor motor7 = motor(PORT7, ratio18_1, true);
// Middle goal
motor motor8 = motor(PORT8, ratio18_1, true);
// Basket
motor motor20 = motor(PORT20, ratio18_1, true);
// Arm
motor motor19 = motor(PORT19, ratio18_1, true);

bool arm = false;

// ------------------- CONFIGURATION -------------------
const int LEFT_SPEED  = 100;
const int RIGHT_SPEED = 100;   // adjust to correct drift
const double SETTLE_TIME = 0.15; // seconds

const double TICKS_PER_CM = 15;     
const double TICKS_PER_DEG_TURN = 5; 

// ------------------- MOVEMENT FUNCTIONS -------------------
void drive_cm(double distance_cm, int left_speed=LEFT_SPEED, int right_speed=RIGHT_SPEED) {
    double ticks = distance_cm * TICKS_PER_CM;

    left_motor_f.setVelocity(left_speed, pct);
    left_motor_b.setVelocity(left_speed, pct);
    right_motor_f.setVelocity(right_speed, pct);
    right_motor_b.setVelocity(right_speed, pct);

    left_drive_smart.spinFor(fwd, ticks, deg, false);
    right_drive_smart.spinFor(fwd, ticks, deg, true);

    wait(SETTLE_TIME, sec);
}

void turn_deg(double degrees, int left_speed=LEFT_SPEED/2, int right_speed=RIGHT_SPEED/2) {
    double ticks = fabs(degrees) * TICKS_PER_DEG_TURN;

    left_motor_f.setVelocity(left_speed, pct);
    left_motor_b.setVelocity(left_speed, pct);
    right_motor_f.setVelocity(right_speed, pct);
    right_motor_b.setVelocity(right_speed, pct);

    if(degrees > 0) { // turn right
        left_drive_smart.spinFor(fwd, ticks, deg, false);
        right_drive_smart.spinFor(reverse, ticks, deg, true);
    } else { // turn left
        left_drive_smart.spinFor(reverse, ticks, deg, false);
        right_drive_smart.spinFor(fwd, ticks, deg, true);
    }

    wait(SETTLE_TIME, sec);
}

void warmup_motors(double duration_sec = 2.0) {
    left_drive_smart.spinFor(fwd, 10, deg, false);
    right_drive_smart.spinFor(fwd, 10, deg, true);
    wait(duration_sec, sec);
}

// ------------------- MOTOR TEMP COLOR FUNCTION -------------------
void updateMotorTemperatureColors() {
    double temps[] = {
        left_motor_f.temperature(celsius),
        left_motor_b.temperature(celsius),
        right_motor_f.temperature(celsius),
        right_motor_b.temperature(celsius)
    };

    double maxTemp = temps[0];
    for(int i = 1; i < 4; i++) {
        if(temps[i] > maxTemp) maxTemp = temps[i];
    }

    if(maxTemp < 45) Brain.Screen.setFillColor(green);
    else if(maxTemp < 55) Brain.Screen.setFillColor(orange);
    else Brain.Screen.setFillColor(red);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Max Motor Temp: %.1f C", maxTemp);
}
void waitUntilStall(motor &m) {
    wait(200, msec);
    while(true) {
        if(fabs(m.velocity(pct)) < 1) {
            m.stop(hold);
            break;
        }
        wait(10, msec);
    }
}

void toggleArm() {
    if(arm) {
        motor19.spin(fwd, 100, pct);
        waitUntilStall(motor19);
        arm = false;
    } else {
        motor19.spin(reverse, 100, pct);
        waitUntilStall(motor19);
        arm = true;
    }
}

void matchload(){
    toggleArm();
    
}

void long_goal(){
           motor6.spin(fwd, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(fwd, 100, pct);
    }

    void upper_middle(){
           motor6.spin(reverse, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(fwd, 100, pct);
    }
  void outtake(){
           motor8.spin(reverse, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor20.spin(fwd, 100, pct);
    }
void intake(){
           motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);
    }

// ------------------- AUTONOMOUS ROUTINE -------------------
void autonomous() {
    // Warmup motors
    warmup_motors();

    // Example routine
    drive_cm(97);    // move forward 60 cm
    //150 onwards left side get ahead 
    turn_deg(64.85);// in the current tick the 90 angle is over by 50%
    // Monitor motor temperature during routine
    for(int i=0; i<20; i++) { // arbitrary loop to simulate movement monitoring
        updateMotorTemperatureColors();
        wait(100, msec);
    }

    // Additional autonomous commands can go here...
}

// ------------------- MAIN -------------------
int main() {
    vexcodeInit();

    // Run autonomous routine
    autonomous();

    // Continue monitoring motor temperature indefinitely
    while(true) {
        updateMotorTemperatureColors();
        wait(100, msec);
    }
}
