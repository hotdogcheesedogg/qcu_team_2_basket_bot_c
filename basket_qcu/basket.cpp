#include "vex.h"
using namespace vex;

brain Brain;
controller controller1(primary);

// ------------------- MOTORS -------------------
motor left_motor_f  = motor(PORT1,  ratio18_1, true);
motor left_motor_b  = motor(PORT5,  ratio18_1, true);
motor_group left_drive_smart(left_motor_f, left_motor_b);

motor right_motor_f = motor(PORT3, ratio18_1, false);
motor right_motor_b = motor(PORT4, ratio18_1, false);
motor_group right_drive_smart(right_motor_f, right_motor_b);

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

// -------------------- ARM --------------------
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
// -------------------- DRIVETRAIN --------------------
void drivetrain_movement() {
    int forward = controller1.Axis3.position();
    int turn = controller1.Axis1.position();

    int left_speed = (forward - turn) * 2;
    int right_speed = (forward + turn) * 2;

    left_drive_smart.spin(fwd, left_speed, pct);
    right_drive_smart.spin(fwd, right_speed, pct);
}

// -------------------- MOTOR TEMP MONITOR --------------------
void updateMotorTemperatureColors() {
    // Get highest temperature among drivetrain motors
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

    // Change background color based on temperature
    if(maxTemp < 45) Brain.Screen.setFillColor(green);
    else if(maxTemp < 55) Brain.Screen.setFillColor(orange);
    else Brain.Screen.setFillColor(red);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Max Motor Temp: %.1f C", maxTemp);
}

// -------------------- DRIVER CONTROL --------------------
void drivers_control() {

    controller1.ButtonDown.pressed(toggleArm);
    //arm 
    while(true) {
        drivetrain_movement();

        if(controller1.ButtonB.pressing()) {
            motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);
            //  intake 

        }
    
        else if(controller1.ButtonL2.pressing()) {
            motor8.spin(reverse, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor20.spin(fwd, 100, pct);
            //outtake or low middle
        }
  
        else if(controller1.ButtonR1.pressing()) {
            motor6.spin(reverse, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(fwd, 100, pct);
            //upper middle
        }

        else if(controller1.ButtonR2.pressing()) {
            motor6.spin(fwd, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(fwd, 100, pct);
            //long goal
        }
 
        else {
            motor6.stop();
            motor7.stop();
            motor8.stop();
            motor20.stop();
        }

        // Update Brain screen color
        updateMotorTemperatureColors();

        wait(100, msec);
    }
}
    
// -------------------- MAIN --------------------
competition Competition;

int main() {
    motor19.setMaxTorque(45, pct);

    Competition.drivercontrol(drivers_control);
    Competition.autonomous([](){});

    while(true) wait(100, msec);
}
