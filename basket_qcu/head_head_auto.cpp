#include "vex.h"
using namespace vex;

brain Brain;
controller controller1(primary);

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

motor motor9 = motor(PORT9, ratio18_1, true);
// Basket
motor motor20 = motor(PORT20, ratio18_1, true);
// Arm
motor motor19 = motor(PORT19, ratio18_1, true);

bool arm = false;

// ------------------- CONFIGURATION -------------------
const int LEFT_SPEED  = 90;
const int RIGHT_SPEED = 90;   
const double SETTLE_TIME = 0.15; 

const double TICKS_PER_CM = 15;     
const double TICKS_PER_DEG_TURN = 5; 

// ------------------- MOVEMENT FUNCTIONS -------------------
void drive_cm(double distance_cm, double left_speed, double right_speed, bool intake_bool, double intake_wait) {
    double ticks = distance_cm * TICKS_PER_CM;

    left_motor_f.setVelocity(left_speed, pct);
    left_motor_b.setVelocity(left_speed, pct);
    right_motor_f.setVelocity(right_speed, pct);
    right_motor_b.setVelocity(right_speed, pct);

    left_drive_smart.spinFor(fwd, ticks, deg, false);
    right_drive_smart.spinFor(fwd, ticks, deg, true);

    if (intake_bool){
       motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);
            
            wait(intake_wait,sec);
            motor8.stop();
            motor7.stop();
            motor20.stop();
    }
    wait(SETTLE_TIME, sec);
}

void turn_deg(double degrees, double left_speed, double right_speed) {
    double ticks = fabs(degrees) * TICKS_PER_DEG_TURN;
    left_speed /= 2;
    right_speed /= 2;

    left_motor_f.setVelocity(left_speed, pct);
    left_motor_b.setVelocity(left_speed, pct);
    right_motor_f.setVelocity(right_speed, pct);
    right_motor_b.setVelocity(right_speed, pct);

    if(degrees > 0) { 
        left_drive_smart.spinFor(fwd, ticks, deg, false);
        right_drive_smart.spinFor(reverse, ticks, deg, true);
    } else { 
        left_drive_smart.spinFor(reverse, ticks, deg, false);
        right_drive_smart.spinFor(fwd, ticks, deg, true);
    }

    wait(SETTLE_TIME, sec);
}

void warmup_motors(double duration_sec = 0.1) {
    left_drive_smart.spinFor(fwd, 10, deg, false);
    right_drive_smart.spinFor(fwd, 10, deg, true);
    wait(duration_sec, sec);
}

void basket_release(double duration = 0.1){
    motor6.spin(reverse, 100, pct);
    wait(duration, sec);
    motor6.stop();
}

// curvatureFactor: 0.0 = straight, >0 = curve right, <0 = curve left
// distance_cm: target straight-line distance of robot's path
// speed_pct: speed for both sides
// curvature: -1.0 (max left curve) to 1.0 (max right curve), 0 = straight
void driveCurve(double distance_cm, int speed_pct, double curvature) {
    // Base distance in ticks
    double baseTicks = distance_cm * TICKS_PER_CM;

    // Adjust ticks for left/right motors
    double leftTicks  = baseTicks * (1.0 + curvature);  // left longer = curve right
    double rightTicks = baseTicks * (1.0 - curvature);  // right shorter = curve right

    // Apply negative if needed (curvature < 0 curves left)
    if(leftTicks < 0) leftTicks = -leftTicks;
    if(rightTicks < 0) rightTicks = -rightTicks;

    // Set velocities
    left_drive_smart.setVelocity(speed_pct, pct);
    right_drive_smart.setVelocity(speed_pct, pct);

    // Spin both sides simultaneously
    left_drive_smart.spinFor(fwd, leftTicks, deg, false);
    right_drive_smart.spinFor(fwd, rightTicks, deg, true);

    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);
}

void driveSmooth(double distance_cm, int maxSpeed, double accelDist, double decelDist) {
    // Direction handling
    int dir = (distance_cm >= 0) ? 1 : -1;
    distance_cm = fabs(distance_cm);

    double startLeft  = left_motor_f.position(deg);
    double startRight = right_motor_f.position(deg);
    double targetTicks = distance_cm * TICKS_PER_CM;

    const double minSpeed = 5;   // % minimum
    const int loopInterval = 20; // ms

    // Prevent invalid accel/decel
    if(accelDist < 1) accelDist = 1;
    if(decelDist < 1) decelDist = 1;

    while (true) {
        double leftTicks  = fabs(left_motor_f.position(deg) - startLeft);
        double rightTicks = fabs(right_motor_f.position(deg) - startRight);
        double avgTicks   = (leftTicks + rightTicks) / 2.0;

        if (avgTicks >= targetTicks) break;

        double remainingTicks = targetTicks - avgTicks;
        double speed = maxSpeed;

        // ---- ACCELERATION ----
        double accelTicks = accelDist * TICKS_PER_CM;
        if (avgTicks < accelTicks) {
            speed = maxSpeed * (avgTicks / accelTicks);
        }

        // ---- DECELERATION ----
        double decelTicks = decelDist * TICKS_PER_CM;
        if (remainingTicks < decelTicks) {
            double decelSpeed = maxSpeed * (remainingTicks / decelTicks);
            if (decelSpeed < speed) speed = decelSpeed;
        }

        // ---- Clamp speed ----
        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < minSpeed) speed = minSpeed;

        left_drive_smart.setVelocity(speed, pct);
        right_drive_smart.setVelocity(speed, pct);

        left_drive_smart.spin(dir == 1 ? fwd : reverse);
        right_drive_smart.spin(dir == 1 ? fwd : reverse);

        wait(loopInterval, msec);
    }

    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);
}

void turnSmooth(double angle_deg, int maxSpeed, double accelAngle, double decelAngle) {
    int dir = (angle_deg >= 0) ? 1 : -1;
    angle_deg = fabs(angle_deg);

    double startLeft  = left_motor_f.position(deg);
    double startRight = right_motor_f.position(deg);

    double targetTicks = angle_deg * TICKS_PER_DEG_TURN;

    const double minSpeed = 5;
    const int loopInterval = 20;

    if(accelAngle < 1) accelAngle = 1;
    if(decelAngle < 1) decelAngle = 1;

    while (true) {
        double leftTicks  = fabs(left_motor_f.position(deg) - startLeft);
        double rightTicks = fabs(right_motor_f.position(deg) - startRight);
        double avgTicks   = (leftTicks + rightTicks) / 2.0;

        if (avgTicks >= targetTicks) break;

        double remainingTicks = targetTicks - avgTicks;
        double speed = maxSpeed;

        // ---- ACCEL ----
        double accelTicks = accelAngle * TICKS_PER_DEG_TURN;
        if (avgTicks < accelTicks) {
            speed = maxSpeed * (avgTicks / accelTicks);
        }

        // ---- DECEL ----
        double decelTicks = decelAngle * TICKS_PER_DEG_TURN; 
        if (remainingTicks < decelTicks) {
            double decelSpeed = maxSpeed * (remainingTicks / decelTicks);
            if (decelSpeed < speed) speed = decelSpeed;
        }

        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < minSpeed) speed = minSpeed;

        left_drive_smart.setVelocity(speed, pct);
        right_drive_smart.setVelocity(speed, pct);

        // In-place turn
        left_drive_smart.spin(dir == 1 ? fwd : reverse);
        right_drive_smart.spin(dir == 1 ? reverse : fwd);

        wait(loopInterval, msec);
    }

    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);
}


void drive_for(double seconds, int speed, bool intake_bool,double intake_wait) {
    int direction = (seconds >= 0) ? 1 : -1;

    left_drive_smart.setVelocity(speed, pct);
    right_drive_smart.setVelocity(speed, pct);

    left_drive_smart.spin(direction == 1 ? fwd : reverse);
    right_drive_smart.spin(direction == 1 ? fwd : reverse);
    
    intake_wait /= 3;
    double intake_stop= 0.5;
    if (intake_bool){
       motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);
            
            wait(intake_wait,sec);
            motor8.stop();
            motor7.stop();
            motor20.stop();

            motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);

            wait(intake_stop,sec);
            motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);

            motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);

            wait(intake_wait,sec);
             motor8.stop();
            motor7.stop();
            motor20.stop();

            motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);

        wait(intake_wait,sec);
             motor8.stop();
            motor7.stop();
            motor20.stop();

    } 
    
    wait(fabs(seconds), sec);
    
    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);
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

void arm_up(double wait_seconds){
    motor19.spin(reverse, 100, pct);
    waitUntilStall(motor19);       
    wait(wait_seconds, sec);        
}

void arm_down(double wait_seconds){
    motor19.spin(fwd, 100, pct);
    waitUntilStall(motor19);        
    wait(wait_seconds, sec);     
}

void basket_unclog(double wait_seconds){
    motor20.spin(reverse, 100, pct);
    wait(wait_seconds, sec);        
    motor20.stop();
}

void long_goal(double wait_time){
           motor6.spin(fwd, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(fwd, 100, pct);
            wait(wait_time,sec);
            motor6.stop();
            motor7.stop();
            motor8.stop();  
            motor20.stop();
    }

    void upper_middle(double wait_time){
           motor6.spin(reverse, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(fwd, 100, pct);
            wait(wait_time,sec);
            motor6.stop();
            motor7.stop();
            motor8.stop();
            motor20.stop();
    }
  void outtake(double wait_time){
           motor8.spin(reverse, 100, pct);
            motor7.spin(reverse, 100, pct);
            motor20.spin(fwd, 100, pct);
            wait(wait_time,sec);
            motor8.stop();
            motor7.stop();
            motor20.stop();
            
    }
void intake(double wait_time){
           motor7.spin(fwd, 100, pct);
            motor8.spin(fwd, 100, pct);
            motor20.spin(reverse, 100, pct);
            wait(wait_time,sec);
            motor8.stop();
            motor7.stop();
            motor20.stop();
    }
int intakeThreadFunc() {
    motor7.spin(fwd, 100, pct);
    motor8.spin(fwd, 100, pct);
    motor20.spin(reverse, 100, pct);

    wait(0.3, sec);

    motor7.stop();
    motor8.stop();
    motor20.stop();
    return 0;
}


    void reverse_middle(double wait_time){
        motor8.spin(reverse, 100, pct);
        motor7.spin(fwd, 100, pct);
        wait(wait_time,sec);
            motor8.stop();
            motor7.stop();
    }
    
    int reverseMiddleThread() {
    motor8.spin(reverse, 100, pct);
    motor7.spin(fwd, 100, pct);

    wait(1, sec);

    motor8.stop();
    motor7.stop();
    return 0;
}

// ------------------- AUTONOMOUS ROUTINE -------------------
void autonomous() {
 /*
   //ver 1 routine 30 secs head to head 
   // 0 . pre_autonomous or start up 
    basket_release(0.1);
    // basket_release(duration);
    arm_down(0.1);
    // arm_down(duration);
    
    // 1. parking to matchload 
    drive_cm(97.8, 70, 70, false, 0); // move forward 60 cm
     //  drive_cm(distance ,left_side_speed,right_side_speed,intake,intake_duration);
    turn_deg(62,60,60);
    // turn_deg(rotation, left_side_speed, right_side_speed);

    // 2. moving forward to matchload to intake 
    drive_cm(31,75,75,true,3);// adjust time in intake 
    //1.5 for skills time

    // 3. adjusting position to long goal 
    drive_cm(-15,35,35, false ,.2); //. 
    turn_deg(-128,50,50); //.
    arm_up(0.1);//. 

    // 4. moving forward to long goal to shoot 
    drive_cm(48,60,60,false,0);
    basket_unclog(.4);
    long_goal(.4);

    long_goal(3.6);
    basket_unclog(.4);
    long_goal(.4);
    long_goal(3.6);

    drive_cm(-15, 35, 35, false, 0);
    turn_deg(62, 60, 60);
    
 */

  /*
    //routine 2 30 secs head to head 
    //0 . pre_autonomous or start up 
   basket_release(0.1);
    arm_down(0.1);
   //1. parking to lower middle goal 
   drive_cm(114, 70, 70, false, 0); // move forward 60 cm
    // drive_cm(99,90,90,false,0);

    // 2. adjusting positiom to shoot in lower middle goal
   turn_deg(34,40,40);
   drive_cm(5.2,50,50,false,0);// adjust time in intake
   long_goal(2);
  // outtake(1);
    drive_cm(-96.5, 70, 70, false, 0);


    // 3. moving from lower middle goal to matchload (unfinished 
   //red
    turn_deg(89.4,50,50);
   
   drive_cm(47.5,70,70, true, 1); //binago
   drive_cm(-20,50,50,false,0);
   arm_up(0.1);
   turn_deg(125,50,50);
   drive_cm(46,70,70,true,.3);
   long_goal(6);

  */
 
  /*
 
  // tune by auto_pathing cpp blue side
    basket_release(0.1);
    arm_down(0.1);
   //1. parking to lower middle goal 
   drive_cm(110, 30, 30, false, 0); // move forward 60 cm
    // drive_cm(99,90,90,false,0);

    // 2. adjusting positiom to shoot in lower middle goal
   turn_deg(35,30,30);
   drive_cm(10,30,30,false,0);// adjust time in intake
   long_goal(2);
  // outtake(1);
    drive_cm(-110, 30, 30, false, 0);


    // 3. moving from lower middle goal to matchload (unfinished 
   //red
    turn_deg(95,30,30);
*/
//driveSmooth(105,82,100*.20, 100*.30);
//turnSmooth(35, 100, 20, 30);
//driveSmooth(-100,82,100*.0, 100*.30);
        
// routine 3 revised blue side

arm_down(0.1);
basket_release(0.1);
driveSmooth(98,80,100*0, 100*0.3);

turnSmooth(-61.5,60,40,40);
wait(0.2,sec);


driveSmooth(37.5,80,100*.10,100*0.3);
intake(.75);


driveSmooth(-30,80,100*0.1,100*0.3);
arm_up(0.1);
arm_down(0.1);
thread intakeThread1(intakeThreadFunc);
turnSmooth(-96.7,60,40,40);

wait(0.2,sec);

driveSmooth(106.1,80,100*.20,100*0.3);
upper_middle(1.75);


thread revMid(reverseMiddleThread);
driveSmooth(-36,80,100*0,100*0.3);
thread intakeThread2(intakeThreadFunc);
turnSmooth(27.5,70,30,30);
wait(0.2,sec);

/*
driveSmooth(100,80,100*.10,100*0.3);
arm_up(0.1);

turnSmooth(-87,60,30,30);
driveSmooth(20,80,100*.10,100*0.3);
outtake(1.2);

*/
/*
//routine 3 revised  red side 
arm_down(0.1);
basket_release(0.1);
driveSmooth(98,80,100*0, 100*0.3);

turnSmooth(-63,60,40,40);
wait(0.2,sec);


driveSmooth(36.5,80,100*.10,100*0.3);
intake(.80);

driveSmooth(-35,80,100*0.1,100*0.3);
arm_up(0.1);
arm_down(0.1);
thread intakeThread1(intakeThreadFunc);
turnSmooth(-95.25, 60, 40, 40);

wait(0.2,sec);

driveSmooth(105.5,80,100*.20,100*0.3);
upper_middle(1.7);

thread revMid(reverseMiddleThread);
driveSmooth(-36, 80, 0, 100 * 0.3);
thread intakeThread2(intakeThreadFunc);
turnSmooth(27.5, 60, 30, 30);
wait(0.2,sec);

driveSmooth(100,80,100*.10,100*0.3);
arm_up(0.1);

turnSmooth(-87,60,30,30);
driveSmooth(20,80,100*.10,100*0.3);
outtake(1.2);

*/

} 

// ------------------- MAIN -------------------
int main() {


    // Run autonomous routine
    autonomous();

    // Continue monitoring motor temperature indefinitely
    while(true) {
        updateMotorTemperatureColors();

        wait(100, msec);
    }
}
    
// -------------------- MAIN --------------------
competition Competition;

int main() {
    motor19.setMaxTorque(45, pct);

    
    Competition.autonomous([](){});

    while(true) wait(100, msec);
}
