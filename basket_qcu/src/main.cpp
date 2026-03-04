#include "vex.h"
using namespace vex;

brain Brain;
controller controller1(primary);

// ------------------- MOTORS -------------------
motor left_motor_f  = motor(PORT1,  ratio18_1, false);
motor left_motor_b  = motor(PORT4,  ratio18_1, false);
motor left_motor_c = motor(PORT3,  ratio18_1, false);
motor_group left_drive_smart(left_motor_f, left_motor_b, left_motor_c);

motor right_motor_f = motor(PORT8, ratio18_1, true);
motor right_motor_b = motor(PORT9, ratio18_1, true);
motor right_motor_c = motor(PORT10, ratio18_1, true);
motor_group right_drive_smart(right_motor_f, right_motor_b, right_motor_c);

// ------------------- ARM & MECHS -------------------
motor motor19 = motor(PORT15, ratio18_1, true);
motor conveyor_left = motor(PORT11, ratio18_1, true);//left
motor intake_ruigan    = motor(PORT7, ratio18_1, true);//first
motor conveyor_right   = motor(PORT12, ratio18_1, false);//right
motor descore         = motor(PORT17, ratio18_1, true);
motor change_shooting = motor(PORT19, ratio18_1, true);



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
  wait(660, msec);
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
  wait(600, msec);
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
        wait(165, msec);
        descore.stop(hold);
        wing = false;
    } else {
        descore.spin(forward, 100, pct);// orignally the descore but wil be used to set up the angle of descore
        wait(400, msec);
        descore.stop(hold);
        wing = true;
    }

    wingBusy = false;
}
void wingOut() {
  descore.spinToPosition(900, degrees, 80, velocityUnits::pct);
}

void wingUp() {
  descore.spinToPosition(1000, degrees, 80, velocityUnits::pct);

}

void wingDown(){
  descore.spinToPosition(756, degrees, 80, velocityUnits::pct);
    
}

bool change = false;
bool shoot = false;

// -------------------- SHOOTING MECHANISM --------------------
//This is used to align a shot for the upper middle goal
void toggleShootingM() {
    if (change) return;
    change = true;

    if (shoot) {
        change_shooting.spin(reverse, 100, pct);
        wait(200, msec);
        change_shooting.stop(hold);
        shoot = false;
    } else {
        change_shooting.spin(forward, 100, pct);// orignally the descore but wil be used to set up the angle of descore
        wait(200, msec);
        change_shooting.stop(hold);
        shoot = true;
    }

    change = false;
}

bool change_l = false;
bool shoot_l = false;

//This is used to align a shot for the long goal
void toggleShootingL() {
    if (change_l) return;
    change_l = true;

    if (shoot_l) {
        change_shooting.spin(fwd, 100, pct);
        wait(200, msec);
        change_shooting.stop(hold);
        shoot_l = false;
    } else {
        change_shooting.spin(reverse, 100, pct);// orignally the descore but wil be used to set up the angle of descore
        wait(200, msec);
        change_shooting.stop(hold);
        shoot_l = true;
    }

    change_l = false;
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

   conveyor_left.spin(forward, 100 , pct);
   descore.spinToPosition(1000, degrees, 80, velocityUnits::pct);
    wait(0.5, sec);  

  
    conveyor_left.stop();
      descore.stop(hold);
      
}

void deployExtensionsions() {

conveyor_left.spin(forward, 100 , pct);

    wait(0.5, sec);  

    conveyor_left.stop();
}

void toggleDriveClutch() {
    drive_clutch = !drive_clutch;   
}// switch ON / OFF


// -------------------- DRIVETRAIN --------------------
void drivetrain_movement() {
    int forward = controller1.Axis3.position();
    int turn    = controller1.Axis1.position();

    // Deadband (prevents small joystick noise)
    if (abs(forward) < 5) forward = 0;
    if (abs(turn) < 5) turn = 0;

    // If joystick is centered → HOLD brake
    if (forward == 0 && turn == 0) {
        left_drive_smart.stop(hold);
        right_drive_smart.stop(hold);
        return;
    }

    // Scale based on toggle state
    double scale = drive_clutch ? 0.4 : 1.0;

    double left_percent  = (forward - turn) * scale;
    double right_percent = (forward + turn) * scale;

    // Convert % to RPM (max 127)
    double left_rpm  = left_percent * 1.27;
    double right_rpm = right_percent * 1.27;

    // Clamp to ±127
    if (left_rpm > 127) left_rpm = 127;
    if (left_rpm < -127) left_rpm = -127;

    if (right_rpm > 127) right_rpm = 127;
    if (right_rpm < -127) right_rpm = -127;

    left_drive_smart.spin(fwd, left_rpm, rpm);
    right_drive_smart.spin(fwd, right_rpm, rpm);
}

void toggleMechClutch() {
    mech_clutch = !mech_clutch;
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
void drivercontrol() {

    controller1.ButtonRight.pressed(toggleArm);
    controller1.ButtonY.pressed(toggleWing);
    controller1.ButtonDown.pressed(toggleDriveClutch); 
    controller1.ButtonL1.pressed(toggleMechClutch);
    controller1.ButtonR2.pressed(toggleShootingM);
    controller1.ButtonR2.released(toggleShootingM);
    controller1.ButtonR1.pressed(toggleShootingL);
    controller1.ButtonR1.released(toggleShootingL);
    while(true) {

        // Update drivetrain
        drivetrain_movement();

        // Mechanism clutch (Y = slow mechanism)
        double mechScale = mech_clutch ? 0.4 : 1.0;


        if(controller1.ButtonB.pressing()) {
            // intake
            conveyor_left.spin(reverse, 100 * mechScale, pct);
            intake_ruigan.spin(reverse, 100 * mechScale, pct);
           
            conveyor_right.spin(reverse, 100 * mechScale, pct);
        }
        else if(controller1.ButtonL2.pressing()) {
            // outtake
              conveyor_left.spin(forward, 100 * mechScale, pct);
            intake_ruigan.spin(forward, 100 * mechScale, pct);
       
            conveyor_right.spin(fwd, 100 * mechScale, pct);
        }
          else if(controller1.ButtonR2.pressing()) {
  
            conveyor_left.spin(reverse, 100 * mechScale, pct);
            intake_ruigan.spin(reverse, 100 * mechScale, pct);
           
            conveyor_right.spin(reverse, 100 * mechScale, pct);
        
            // middle goal
            }
       else if(controller1.ButtonR1.pressing()) {
           conveyor_left.spin(reverse, 100 * mechScale, pct);
            intake_ruigan.spin(reverse, 100 * mechScale, pct);
    
            conveyor_right.spin(reverse, 100 * mechScale, pct);
            // long goal
        }
      
        else {
            conveyor_left.stop();
            intake_ruigan.stop();
           
            conveyor_right.stop();
         
       
        }

      
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
        conveyor_left.spin(forward, 100 , pct);
            intake_ruigan.spin(reverse, 100 , pct);
      
            
            wait(intake_wait,sec);
            conveyor_left.stop();
            intake_ruigan.stop();

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
        conveyor_left.spin(forward, 100, pct);
        intake_ruigan.spin(reverse, 100, pct);
  
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
            conveyor_left.stop();
            intake_ruigan.stop();
 
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
        conveyor_left.stop();
        intake_ruigan.stop();
    
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

void turnSmooth1(double angle_deg,
                int maxSpeed,
                double accelAngle,
                double decelAngle,
                bool intake_bool,
                double intake_wait)
{
    int dir = (angle_deg >= 0) ? 1 : -1;
    angle_deg = fabs(angle_deg);

    double startLeft  = left_motor_f.position(deg);
    double startRight = right_motor_f.position(deg);
    double targetTicks = angle_deg * TICKS_PER_DEG_TURN;

    const double minSpeed = 5;
    const int loopInterval = 20; // ms

    if (accelAngle < 1) accelAngle = 1;
    if (decelAngle < 1) decelAngle = 1;

    /* ================== INTAKE START ================== */
    timer intakeTimer;
    bool intakeRunning = false;

    if (intake_bool && intake_wait > 0) {
        conveyor_left.spin(forward, 100, pct);
        conveyor_right.spin(forward, 100, pct);
        intake_ruigan.spin(reverse, 100, pct);


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
            conveyor_left.stop();
            conveyor_right.stop();
            intake_ruigan.stop();
   
            intakeRunning = false;
        }
        /* ----------------------------------------- */

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

    /* ---------- SAFETY STOP ---------- */
    if (intakeRunning) {
        conveyor_left.stop();
        conveyor_right.stop();
        intake_ruigan.stop();

    }
}

void long_goal(double wait_time, double speed){
         conveyor_left.spin(fwd, speed, pct);
            intake_ruigan.spin(forward, speed , pct);
          
              conveyor_right.spin(fwd, speed , pct);
        
            wait(wait_time,sec);
           
            conveyor_right.stop();
    conveyor_right.stop();
            conveyor_left.stop();
            intake_ruigan.stop();
    }

    void upper_middle(double wait_time, double speed){
          conveyor_left.spin(fwd, speed, pct);
            intake_ruigan.spin(fwd, speed, pct);
           
            conveyor_right.spin(fwd, speed, pct);

            wait(wait_time,sec);
           
            conveyor_right.stop();
         
            conveyor_left.stop();
            intake_ruigan.stop();
    }
  void outtake(double wait_time,double speed){
           conveyor_left.spin(reverse, speed , pct);
            intake_ruigan.spin(forward, 60 , pct);
         
            wait(wait_time,sec);
            conveyor_left.stop();
            intake_ruigan.stop();
       
            
    }
void intake(double wait_time, double speed){
            conveyor_left.spin(forward, speed , pct);
            conveyor_right.spin(fwd, speed , pct);
            intake_ruigan.spin(reverse, speed , pct);
          
            wait(wait_time,sec);
            conveyor_left.stop();
            intake_ruigan.stop();
            conveyor_right.stop();
   
    }

void autonomous() {


}
// -------------------- MAIN --------------------
competition Competition;

int main() {

//autonomous();
 motor19.setMaxTorque(45, pct);
  Competition.drivercontrol(drivercontrol);
Competition.autonomous(autonomous);
   
    while(true) {
        wait(100, msec);
    }
}
