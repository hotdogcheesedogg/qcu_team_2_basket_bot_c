#include "vex.h"
using namespace vex;

brain Brain;
controller controller1(primary);

// ------------------- MOTORS -------------------
motor left_motor_f  = motor(PORT1,  ratio18_1, false);
motor left_motor_b  = motor(PORT7,  ratio18_1, false);
motor left_motor_c = motor(PORT3,  ratio18_1, true);
motor_group left_drive_smart(left_motor_f, left_motor_b, left_motor_c);

motor right_motor_f = motor(PORT4, ratio18_1, true);
motor right_motor_b = motor(PORT5, ratio18_1, true);
motor right_motor_c = motor(PORT6, ratio18_1, false);
motor_group right_drive_smart(right_motor_f, right_motor_b, right_motor_c);

// ------------------- ARM & MECHS -------------------
motor motor19 = motor(PORT9, ratio18_1, true);
motor intake_middle_f = motor(PORT11, ratio18_1, true);
motor long_f          = motor(PORT12, ratio18_1, true);
motor store_basket    = motor(PORT13, ratio18_1, true);
motor long_middle_b   = motor(PORT14, ratio18_1, true);
motor unclog          = motor(PORT15, ratio18_1, true);
motor descore         = motor(PORT16, ratio18_1, true);

vision myVisionSensor = vision(PORT17);

vision::signature BLUE_BALL = vision::signature(1, -4393, -2133, -3263, -1, 4567, 2283, 1.1, 0);
vision::signature RED_BALL  = vision::signature(2, 6539, 10627, 8583, -2007, -1013, -1510, 2.5, 0);



bool arm = false;
bool drive_clutch = false;
bool mech_clutch  = false;
bool wing = false;   

const int LEFT_SPEED  = 100;
const int RIGHT_SPEED = 100;   
const double SETTLE_TIME = 0;//0.15; 

// Updated for 4" omni wheels
const double TICKS_PER_CM = 11.28;            // 4" omni wheel circumference
const double TICKS_PER_DEG_TURN = 3.12;       
// -------------------- ARM --------------------
void waitUntilStall(motor &m, int timeout_ms = 1200) {
    int elapsed = 0;
    wait(300, msec);

    while (elapsed < timeout_ms) {
        if (fabs(m.velocity(pct)) < 1) {
            m.stop(hold);
            return;
        }
        wait(10, msec);
        elapsed += 10;
    }


    m.stop(hold);
}

void toggleArm() {
    if(arm) {
        motor19.spin(forward, 100, pct);
        waitUntilStall(motor19);
        arm = false;
    } else {
  motor19.spin(reverse, 100, pct);
  wait(520, msec);
  motor19.stop(hold);
        arm = true;
    }
}
void arm_up(double wait_seconds){
    motor19.spin(forward, 100, pct);
    waitUntilStall(motor19);       
    wait(wait_seconds, sec);        
}

void arm_down(double wait_seconds){
   motor19.spin(reverse, 100, pct);
  wait(500, msec);
  motor19.stop(hold);
    wait(wait_seconds, sec);
}

// -------------------- DESCORE WING --------------------

void moveWing(motor &m, int timeout_ms = 10) {
    int elapsed = 0;
    wait(300, msec);

    while (elapsed < timeout_ms) {
        if (fabs(m.velocity(pct)) < 1) {
            m.stop(hold);
            return;
        }
        wait(10, msec);
        elapsed += 10;
    }

    // Safety exit
    m.stop(hold);
}
bool wingBusy = false;

void toggleWing() {
    if (wingBusy) return;
    wingBusy = true;

    if (wing) {
        descore.spin(reverse, 100, pct);
        wait(320, msec);
        descore.stop(hold);
        wing = false;
    } else {
        descore.spin(forward, 100, pct);
        wait(310, msec);
        descore.stop(hold);
        wing = true;
    }

    wingBusy = false;
}
void wingOut() {
  descore.spinToPosition(1000, degrees, 80, velocityUnits::pct);
}

void wingUp() {
  descore.spinToPosition(1000, degrees, 80, velocityUnits::pct);

}

void wingDown(){
  descore.spinToPosition(756, degrees, 80, velocityUnits::pct);
    
}

void drivetrainLock() {
    left_drive_smart.setStopping(hold);  
    right_drive_smart.setStopping(hold);

    left_drive_smart.setVelocity(0, pct);
    right_drive_smart.setVelocity(0, pct);

    left_drive_smart.stop();
    right_drive_smart.stop();
}

void deploy2v2Extensionsions() {
   unclog.spin(forward, 100 , pct);
   intake_middle_f.spin(forward, 100 , pct);
   descore.spinToPosition(1000, degrees, 80, velocityUnits::pct);
    wait(0.5, sec);  
    unclog.stop();
  
    intake_middle_f.stop();
      descore.stop(hold);
      
}

void deployExtensionsions() {
   unclog.spin(forward, 100 , pct);
intake_middle_f.spin(forward, 100 , pct);

    wait(0.5, sec);  

    intake_middle_f.stop();
}

// -------------------- DRIVETRAIN --------------------
void drivetrain_movement() {
    int forward = controller1.Axis3.position();
    int turn = controller1.Axis1.position();

    // If drive clutch is pressed, reduce speed
    double scale = controller1.ButtonDown.pressing() ? 0.4 : 1.0;

    int left_speed = (forward - turn) * scale;
    int right_speed = (forward + turn) * scale;

    left_drive_smart.spin(fwd, left_speed, pct);
    right_drive_smart.spin(fwd, right_speed, pct);
}

// -------------------- MOTOR TEMP MONITOR --------------------
void updateMotorTemperatureColors() {
    double temps[] = {
        left_motor_f.temperature(celsius),
        left_motor_b.temperature(celsius),
        left_motor_c.temperature(celsius),
        right_motor_f.temperature(celsius),
        right_motor_b.temperature(celsius),
        right_motor_c.temperature(celsius)
    };

    double maxTemp = temps[0];
    for(int i = 1; i < 6; i++)
        if(temps[i] > maxTemp) maxTemp = temps[i];

    if(maxTemp < 45) Brain.Screen.setFillColor(green);
    else if(maxTemp < 55) Brain.Screen.setFillColor(orange);
    else Brain.Screen.setFillColor(red);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("Max Motor Temp: %.1f C", maxTemp);
}

// -------------------- DRIVER CONTROL --------------------
void drivers_control() {

    controller1.ButtonRight.pressed(toggleArm);
    controller1.ButtonY.pressed(toggleWing);

    while(true) {

        // Update drivetrain
        drivetrain_movement();

        // Mechanism clutch (Y = slow mechanism)
        double mechScale = controller1.ButtonL1.pressing() ? 0.4 : 1.0;

        if(controller1.ButtonB.pressing()) {
            // intake
            intake_middle_f.spin(forward, 100 * mechScale, pct);
            store_basket.spin(reverse, 100 * mechScale, pct);
            unclog.spin(reverse, 100 * mechScale, pct);
        }
        else if(controller1.ButtonL2.pressing()) {
            // outtake
            intake_middle_f.spin(reverse, 100 * mechScale, pct);
            store_basket.spin(forward, 100 * mechScale, pct);
            unclog.spin(fwd, 100 * mechScale, pct);
        }
        else if(controller1.ButtonR2.pressing()) {
            // middle
            intake_middle_f.spin(fwd, 100 * mechScale, pct);
            store_basket.spin(fwd, 100 * mechScale, pct);
            long_f.spin(forward, 100 * mechScale, pct);
            long_middle_b.spin(fwd, 100 * mechScale, pct);
            unclog.spin(fwd, 100 * mechScale, pct);
        }
        else if(controller1.ButtonR1.pressing()) {
            // long goal
            intake_middle_f.spin(fwd, 100 * mechScale, pct);
            store_basket.spin(forward, 100 * mechScale, pct);
            long_f.spin(reverse, 100 * mechScale, pct);
            long_middle_b.spin(fwd, 100 * mechScale, pct);
            unclog.spin(fwd, 100 * mechScale, pct);
        }
        else {
            intake_middle_f.stop();
            store_basket.stop();
            long_f.stop();
            long_middle_b.stop();
            unclog.stop();
       
        }

        // Update brain screen
        updateMotorTemperatureColors();

        wait(20, msec);
    }
}

void drive_cm(double distance_cm, double left_speed, double right_speed, bool intake_bool, double intake_wait) {
    double ticks = distance_cm * TICKS_PER_CM;

    left_motor_f.setVelocity(left_speed, pct);
    left_motor_b.setVelocity(left_speed, pct);
    left_motor_c.setVelocity(left_speed,pct);
    right_motor_f.setVelocity(right_speed, pct);
    right_motor_b.setVelocity(right_speed, pct);
    right_motor_c.setVelocity(right_speed,pct);

    left_drive_smart.spinFor(fwd, ticks, deg, false);
    right_drive_smart.spinFor(fwd, ticks, deg, true);

    if (intake_bool){
        intake_middle_f.spin(forward, 100 , pct);
            store_basket.spin(reverse, 100 , pct);
            unclog.spin(reverse, 100 , pct);
            
            wait(intake_wait,sec);
            intake_middle_f.stop();
            store_basket.stop();
            unclog.stop();
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

void driveSmooth(double distance_cm,
                 int maxSpeed,
                 double accelDist,
                 double decelDist,
                 bool intake_bool,
                 double intake_wait)
{
    int dir = (distance_cm >= 0) ? 1 : -1;
    distance_cm = fabs(distance_cm);

    double startLeft  = left_motor_f.position(deg);
    double startRight = right_motor_f.position(deg);
    double targetTicks = distance_cm * TICKS_PER_CM;

    const double minSpeed = 5;
    const int loopInterval = 20; // ms

    if(accelDist < 1) accelDist = 1;
    if(decelDist < 1) decelDist = 1;

    /* ================== INTAKE START ================== */
    timer intakeTimer;
    bool intakeRunning = false;

    if (intake_bool && intake_wait > 0) {
        intake_middle_f.spin(forward, 100, pct);
        store_basket.spin(reverse, 100, pct);
        unclog.spin(reverse, 100, pct);
        intakeTimer.reset();
        intakeRunning = true;
    }
    
    /* ================================================== */

    while (true) {
        double leftTicks  = fabs(left_motor_f.position(deg) - startLeft);
        double rightTicks = fabs(right_motor_f.position(deg) - startRight);
        double avgTicks   = (leftTicks + rightTicks) / 2.0;

        if (avgTicks >= targetTicks) break;

        /* ---------- INTAKE TIME CONTROL ---------- */
        if (intakeRunning && intakeTimer.time(sec) >= intake_wait) {
            intake_middle_f.stop();
            store_basket.stop();
            unclog.stop();
            intakeRunning = false;
        }
        /* ----------------------------------------- */

        double remainingTicks = targetTicks - avgTicks;
        double speed = maxSpeed;

        // Acceleration
        double accelTicks = accelDist * TICKS_PER_CM;
        if (avgTicks < accelTicks) {
            speed = maxSpeed * (avgTicks / accelTicks);
        }

        // Deceleration
        double decelTicks = decelDist * TICKS_PER_CM;
        if (remainingTicks < decelTicks) {
            double decelSpeed = maxSpeed * (remainingTicks / decelTicks);
            if (decelSpeed < speed) speed = decelSpeed;
        }

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

    /* ---------- SAFETY STOP ---------- */
    if (intakeRunning) {
        intake_middle_f.stop();
        store_basket.stop();
    
    }
}

// Smooth turn
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

        // Acceleration
        double accelTicks = accelAngle * TICKS_PER_DEG_TURN;
        if (avgTicks < accelTicks) {
            speed = maxSpeed * (avgTicks / accelTicks);
        }

        // Deceleration
        double decelTicks = decelAngle * TICKS_PER_DEG_TURN;
        if (remainingTicks < decelTicks) {
            double decelSpeed = maxSpeed * (remainingTicks / decelTicks);
            if (decelSpeed < speed) speed = decelSpeed;
        }

        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < minSpeed) speed = minSpeed;

        left_drive_smart.setVelocity(speed, pct);
        right_drive_smart.setVelocity(speed, pct);

        left_drive_smart.spin(dir == 1 ? fwd : reverse);
        right_drive_smart.spin(dir == 1 ? reverse : fwd);

        wait(loopInterval, msec);
    }

    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);
}
void long_goal(double wait_time, double speed){
         intake_middle_f.spin(fwd, speed, pct);
            store_basket.spin(forward, speed , pct);
            long_f.spin(reverse, speed , pct);
            long_middle_b.spin(fwd, speed , pct);
            unclog.spin(reverse, speed , pct);
            wait(wait_time,sec);
            long_f.stop();
            long_middle_b.stop();
  
            intake_middle_f.stop();
            store_basket.stop();
    }

    void upper_middle(double wait_time, double speed){
          intake_middle_f.spin(fwd, speed, pct);
            store_basket.spin(fwd, speed, pct);
            long_f.spin(forward, speed, pct);
            long_middle_b.spin(fwd, speed, pct);
            unclog.spin(reverse, 100, pct);
            wait(wait_time,sec);
            long_f.stop();
            long_middle_b.stop();
         
            intake_middle_f.stop();
            store_basket.stop();
    }
  void outtake(double wait_time,double speed){
           intake_middle_f.spin(reverse, speed , pct);
            store_basket.spin(forward, 60 , pct);
            unclog.spin(reverse, 100 , pct); 
            wait(wait_time,sec);
            intake_middle_f.stop();
            store_basket.stop();
       
            
    }
void intake(double wait_time, double speed){
            intake_middle_f.spin(forward, speed , pct);
            store_basket.spin(reverse, speed , pct);
            unclog.spin(reverse, speed , pct);
            wait(wait_time,sec);
            intake_middle_f.stop();
            store_basket.stop();
   
    }

void autonomous() {
//Routine with matchload 




//basket setup
//skills routine 1 


/*
//routine 1 skills 
drivetrainLock();
deployExtensionsions();
driveSmooth(60, 90, 40, 42, false, 0); 
turnSmooth(80, 80, 50, 45); 
driveSmooth(72.9, 90, 40, 42, false, 0); 
turnSmooth(80, 80, 50, 45);
arm_down(0.1);
driveSmooth(53., 90, 40, 42, false, 0);
intake(3,100);
driveSmooth(-40, 90, 40, 42, false,0);
turnSmooth(170, 80, 50, 42);

//skills routine 2
*/
/*
drivetrainLock();
deployExtensionsions();
driveSmooth(60, 90, 40, 42, false, 0); 
turnSmooth(80, 80, 50, 45); 
driveSmooth(73, 90, 40, 42, false, 0); 
turnSmooth(80, 80, 50, 45);
arm_down(0.1);
driveSmooth(53, 90, 40, 42, false, 0);

driveSmooth(-40, 90, 40, 42, false,0);
turnSmooth(80, 80, 50, 39.5);
driveSmooth(234.7, 90, 40, 42, true, 2.25);
wait(0.5,sec);
turnSmooth(-79.5, 80, 50, 40); 
driveSmooth(44, 90, 40, 42, false, 0); 
driveSmooth(-30, 80, 40, 42, false, 0); 
turnSmooth(-121, 80, 50, 40);
arm_up(0.1);

driveSmooth(120, 90, 40, 42, true, 2);
outtake(8,60);



driveSmooth(-120, 90, 40, 42, false, 0);
arm_down(0.1);
turnSmooth(130, 80, 50, 40);
driveSmooth(30, 80,40, 42, false, 0);

/*
driveSmooth(-40, 90, 40, 42, false,0);
turnSmooth(-80, 80, 50, 40); 
driveSmooth(234.7, 90, 40, 42, true, 2.25);
turnSmooth(80, 80, 50, 40);



*/

/*
// red alliance revised 2v2
drivetrainLock();
deploy2v2Extensionsions();
driveSmooth(60, 90, 40, 42, false, 0); 
turnSmooth(80, 80, 50, 45); 
driveSmooth(73, 90, 40, 42, false, 0); 
turnSmooth(80, 80, 50, 45);
arm_down(0.1);
driveSmooth(53, 90, 40, 42, false, 0);

driveSmooth(-40, 90, 40, 42, true,1);
intake(.5,100);
turnSmooth(180, 80, 50, 45);

arm_up(0.1);
driveSmooth(28,80,40,42,true,2.0);
long_goal(4.0,100); 

driveSmooth(-30, 90, 40, 42, false, 0);
turnSmooth(-35, 80, 50, 45);
wingDown();
drive_cm(78, 100,100, false, 0);


/*
//routine 2v2 red alliance 



drivetrainLock();
deploy2v2Extensionsions();
drive_cm(60, 50,50, false, 0); //drive forward to clear wall
wait(.5,sec);

turnSmooth(80, 80, 50, 45); 

drive_cm(78, 50,50, false, 0); //drive to align with matchload
wait(.5,sec);
turnSmooth(83, 80, 50, 45);
arm_down(0.1);


drive_cm(46, 40,40,false, 0); //drive to matchload and intake
intake(1,100);
driveSmooth(-32, 80, 40, 42, true,3); //reverse from matchload


turnSmooth(173, 80, 50, 45);

arm_up(0.1);
drive_cm(31,80,80,true,2.0);
long_goal(2.0,100);

driveSmooth(-30,80,40,42,false,0);
turnSmooth(173, 80, 50, 45);


/*

arm_down(0.1);
drive_cm(60,80,80,true, 3);
driveSmooth(-30,90,40,42,true, 1);
turnSmooth(-40,80,50,45);
outtake(2.5,100);

/*
driveSmooth(-30,90,40,42,false, 0);
turnSmooth(40, 80, 50 ,45);
arm_up(0.1);
outtake(2.5,100);

turnSmooth(-40,80,50,45);
arm_down(0.1);
drive_cm(30,90,90,false,0);
intake(4,100);


driveSmooth(-30, 90, 40, 42, false,0); //reverse from matchload
turnSmooth(173, 80, 50, 45);
arm_up(0.1);
drive_cm(30,90,90,true,2.0);
long_goal(3.0,100);


driveSmooth(-30,90,40,42,false,0);


*/
//2v2 blue alliance 

drivetrainLock();
deploy2v2Extensionsions();
driveSmooth(60, 90, 0, 42, false, 0); 


turnSmooth(80, 80, 50, 45); 
driveSmooth(73, 90, 40, 42, false, 0); 

turnSmooth(80, 80, 50, 45);
arm_down(0.1);

driveSmooth(53, 90, 40, 42, false, 0);


driveSmooth(-40, 100, 40, 42, true,1);

turnSmooth(112.5, 80, 50, 45);


driveSmooth(115,80,0,50,true,2.0);

upper_middle(2.0,100);

driveSmooth(-40, 90, 40, 42, false, 0);
turnSmooth(-35, 80, 50, 45);
arm_up(0.1);
driveSmooth(110, 100, 0,50, false, 0);
turnSmooth(80, 80, 50, 45);
//*/
}
// -------------------- MAIN --------------------
competition Competition;

int main() {
//drivers_control();
autonomous();

    while(true) wait(100, msec);
   motor19.setMaxTorque(45, pct);

    Competition.drivercontrol(drivers_control);
    Competition.autonomous(autonomous);

    while(true) wait(100, msec);
}
