#include "vex.h"
using namespace vex;

brain Brain;
controller controller1(primary);

// ------------------- MOTORS -------------------
motor left_motor_f = motor(PORT11, ratio18_1, false);
motor left_motor_b = motor(PORT12, ratio18_1, false);
motor left_motor_c = motor(PORT13, ratio18_1, false);
motor_group left_drive_smart(left_motor_f, left_motor_b, left_motor_c);

motor right_motor_f = motor(PORT18, ratio18_1, true);
motor right_motor_b = motor(PORT19, ratio18_1, true);
motor right_motor_c = motor(PORT20, ratio18_1, true);
motor_group right_drive_smart(right_motor_f, right_motor_b, right_motor_c);

// ------------------- ARM & MECHS -------------------
motor motor19 = motor(PORT14, ratio18_1, true);
motor intake_middle_f = motor(PORT7, ratio18_1, true); // left
motor long_f = motor(PORT2, ratio18_1, true);
motor store_basket = motor(PORT9, ratio18_1, true);   // first
motor long_middle_b = motor(PORT8, ratio18_1, false); // right
motor unclog = motor(PORT15, ratio18_1, true);
motor descore = motor(PORT1, ratio18_1, true);
motor change_shooting = motor(PORT10, ratio18_1, true);
vision myVisionSensor = vision(PORT17);
drivetrain Drivetrain = drivetrain(left_drive_smart, right_drive_smart, 319.19, 320, 320, mm, 1.0);

vision::signature BLUE_BALL = vision::signature(1, -4393, -2133, -3263, -1, 4567, 2283, 1.1, 0);
vision::signature RED_BALL = vision::signature(2, 6539, 10627, 8583, -2007, -1013, -1510, 2.5, 0);

bool arm = false;
bool drive_clutch = false;
bool mech_clutch = false;
bool wing = false;

const int LEFT_SPEED = 100;
const int RIGHT_SPEED = 100;
const double SETTLE_TIME = 0; // 0.15;

// Updated for 4" omni wheels
const double TICKS_PER_CM = 11.28; // 4" omni wheel circumference
const double TICKS_PER_DEG_TURN = 3.12;
// -------------------- ARM --------------------
void waitUntilStall(motor &m, int timeout_ms = 1200)
{
    int elapsed = 0;
    wait(300, msec);

    while (elapsed < timeout_ms)
    {
        if (fabs(m.velocity(pct)) < 1)
        {
            m.stop(hold);
            return;
        }
        wait(10, msec);
        elapsed += 10;
    }

    m.stop(hold);
}

void toggleArm()
{
    if (arm)
    {
        /* motor19.spin(forward, 100, pct);
         waitUntilStall(motor19);
         arm = false;
         */
        motor19.spinToPosition(-250, degrees, 100, velocityUnits::pct);
        arm = false;
    }
    else
    {
        /*
        motor19.spin(reverse, 100, pct);
        waitUntilStall(motor19);
        motor19.stop(hold);
        arm = true;
        */
        motor19.spinToPosition(0, degrees, 100, velocityUnits::pct);
        arm = true;
    }
}
void arm_up(double wait_seconds)
{
    motor19.spinToPosition(0, degrees, 100, velocityUnits::pct);
    wait(wait_seconds, sec);
}

void arm_down(double wait_seconds)
{
    motor19.spinToPosition(250, degrees, 100, velocityUnits::pct);

    wait(wait_seconds, sec);
}
void arm_down_skills(double wait_seconds)
{
    motor19.spinToPosition(200, degrees, 100, velocityUnits::pct);

    wait(wait_seconds, sec);
}
void arm_up_skills(double wait_seconds)
{
    motor19.spinToPosition(0, degrees, 100, velocityUnits::pct);

    wait(wait_seconds, sec);
}
// -------------------- DESCORE WING --------------------

void moveWing(motor &m, int timeout_ms = 10)
{
    int elapsed = 0;
    wait(300, msec);

    while (elapsed < timeout_ms)
    {
        if (fabs(m.velocity(pct)) < 1)
        {
            m.stop(hold);
            return;
        }
        wait(10, msec);
        elapsed += 10;
    }

    // Safety exit
    m.stop(hold);
}
/*
bool wingBusy = false;
bool firstDeployDone = false;
bool firstDeploy = false;

void toggleWing() {
    if (wingBusy) return;
    wingBusy = true;

    if (wing) {
         if (!firstDeploy) {

          descore.spin(reverse, 100, pct);
        wait(165, msec);
        descore.stop(hold);

            firstDeploy = true;
        wing = false;
        }
            else {
        descore.spin(reverse, 100, pct);
        wait(200, msec);
        descore.stop(hold);
        wing = false;
            }
    }
    else {
       if (!firstDeployDone) {

            descore.spin(forward, 100, pct);
            wait(425, msec);
            descore.stop(hold);

            firstDeployDone = true;
            wing = true;
        }
        else {

            descore.spin(forward, 100, pct);
            wait(200, msec);
            descore.stop(hold);
 wing = true;
        }

    }

    wingBusy = false;
}
    */

const int FIRST_DEPLOY_OUT = 10; // reverse
const int NORMAL_OUT = 10;
const int FIRST_DEPLOY_IN = 100;
const int NORMAL_IN = 100;

bool wingBusy = false;
bool firstDeployDone = false;
bool firstDeploy = false;
double descoreBaseAngle = 0;
// false = retracted, true = deployed
void toggleWing()
{
    if (wingBusy)
        return;
    wingBusy = true;

    double OUT_OFFSET = 0; // adjust as needed
    double IN_OFFSET = 55; // base position

    double outAngle = descoreBaseAngle + OUT_OFFSET;
    double inAngle = descoreBaseAngle + IN_OFFSET;

    if (wing)
    {
        // Deploy (OUT)
        descore.spinToPosition(outAngle, degrees, 60, velocityUnits::pct);
        wing = false;
    }
    else
    {
        // Retract (IN)
        descore.spinToPosition(inAngle, degrees, 60, velocityUnits::pct);
        wing = true;
    }

    wingBusy = false;
}
void wingOut()
{
    descore.setPosition(0, degrees);
    descore.spinToPosition(170, degrees, 100, velocityUnits::pct);
}

void wingUp()
{
    descore.spinToPosition(170, degrees, 100, velocityUnits::pct);
}

void wingDown()
{
    descore.spinToPosition(130, degrees, 100, velocityUnits::pct);
}

bool change = false;
bool shoot = false;
/*
void toggleShootingM() {
    if (change) return;
    change = true;

    if (shoot) {
        change_shooting.spin(reverse, 100, pct);  //reset
        wait(350, msec);
        change_shooting.stop(hold);
        shoot = false;
    } else {
        change_shooting.spin(forward, 100, pct);
        wait(300, msec);
        change_shooting.stop(hold);
        shoot = true;
    }

    change = false;
}
*/
bool change_l = false;
bool shoot_l = false;
/*void toggleShootingL() {
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
}*/

int lastPressTime = 0;
double baseAngle = 75;
// this is the pressing with angle fix will be removed
/*
void toggleShootingM()
{
    int now = Brain.timer(msec);

    // debounce: ignore clicks within 250ms
    if (now - lastPressTime < 250)
        return;
    lastPressTime = now;

    if (shoot)
    {
        change_shooting.spinToPosition(65, degrees, 100, velocityUnits::pct);
        shoot = false;
    }
    else
    {
        change_shooting.spinToPosition(130, degrees, 100, velocityUnits::pct);
        shoot = true;
    }

    Brain.Screen.print("shoot: %d\n", shoot);
}
*/
bool middleGoalRunning = false;
int lastPressTimeR2 = 0;
bool longGoalRunning = false; // current state
int lastPressTimeR1 = 0;

void toggleShootingM()
{
    int now = Brain.timer(msec);

    if (now - lastPressTimeR2 < 250)
        return;
    lastPressTimeR2 = now;
    longGoalRunning = false;
    middleGoalRunning = !middleGoalRunning;

    if (middleGoalRunning)
    {

        change_shooting.spinToPosition(baseAngle + 75, degrees, 100, velocityUnits::pct, false);
    }
    else
    {

        change_shooting.spinToPosition(baseAngle, degrees, 100, velocityUnits::pct, false);
    }
}

// current updated
/*
void toggleShootingL()
{

    if (change_l)
        return;
    change_l = true;

    if (shoot_l)
    {
        change_shooting.spinToPosition(65, degrees, 100, velocityUnits::pct);

        shoot_l = false;
    }
    else
    {
        change_shooting.spinToPosition(0, degrees, 100, velocityUnits::pct);
        shoot_l = true;
    }

    change_l = false;
}
*/

// ------------------- TOGGLE VARIABLES -------------------
// debounce timer

// ------------------- TOGGLE FUNCTION -------------------
/*

void toggleShootingL()
{
    int now = Brain.timer(msec);

    if (now - lastPressTimeR1 < 250)
        return;
    lastPressTimeR1 = now;

    longGoalRunning = !longGoalRunning;

    if (longGoalRunning)
    {
        change_shooting.spinToPosition(0, degrees, 100, velocityUnits::pct);
    }
    else
    {
        change_shooting.spinToPosition(65, degrees, 100, velocityUnits::pct);
    }
}
    */
void toggleShootingL()
{
    int now = Brain.timer(msec);

    if (now - lastPressTimeR1 < 250)
        return;
    lastPressTimeR1 = now;
    middleGoalRunning = false;
    longGoalRunning = !longGoalRunning;

    if (longGoalRunning)
    {
        change_shooting.spinToPosition(baseAngle - 75, degrees, 100, velocityUnits::pct, false);
    }
    else
    {
        change_shooting.spinToPosition(baseAngle, degrees, 100, velocityUnits::pct, false);
    }
}

void drivetrainLock()
{
    left_drive_smart.setStopping(hold);
    right_drive_smart.setStopping(hold);

    left_drive_smart.setVelocity(0, pct);
    right_drive_smart.setVelocity(0, pct);

    left_drive_smart.stop();
    right_drive_smart.stop();
}

void deploy2v2Extensionsions()
{
    unclog.spin(forward, 100, pct);
    intake_middle_f.spin(forward, 100, pct);
    descore.spinToPosition(1000, degrees, 80, velocityUnits::pct);
    wait(0.5, sec);
    unclog.stop();

    intake_middle_f.stop();
    descore.stop(hold);
}

void deployExtensionsions()
{
    change_shooting.spinToPosition(75, degrees, 100, velocityUnits::pct);

    intake_middle_f.spin(reverse, 100, velocityUnits::pct);
    store_basket.spin(reverse, 100, velocityUnits::pct);
    long_middle_b.spin(reverse, 100, velocityUnits::pct);

    wait(1.5, sec);

    intake_middle_f.stop();
    store_basket.stop();
    long_middle_b.stop();
}

void toggleDriveClutch()
{
    // switch ON / OFF
    drive_clutch = !drive_clutch;
}

// -------------------- DRIVETRAIN --------------------
void drivetrain_movement()
{
    int forward = controller1.Axis3.position();
    int turn = controller1.Axis1.position();

    if (abs(forward) < 5)
        forward = 0;
    if (abs(turn) < 5)
        turn = 0;

    if (forward == 0 && turn == 0)
    {
        /*
        left_drive_smart.stop(hold);
        right_drive_smart.stop(hold);
        return;
        */
    }

    // Speed scaling
    double scale = drive_clutch ? 0.6 : 1.0;

    double left_percent = -(forward + turn) * scale;
    double right_percent = -(forward - turn) * scale;

    double left_rpm = left_percent * 1.8;
    double right_rpm = right_percent * 1.8;

    // Clamp
    if (left_rpm > 180)
        left_rpm = 180;
    if (left_rpm < -180)
        left_rpm = -180;

    if (right_rpm > 180)
        right_rpm = 180;
    if (right_rpm < -180)
        right_rpm = -180;

    left_drive_smart.spin(fwd, left_rpm, rpm);
    right_drive_smart.spin(fwd, right_rpm, rpm);
}
void toggleMechClutch()
{
    mech_clutch = !mech_clutch;
}

// -------------------- MOTOR TEMP MONITOR --------------------
void updateMotorTemperatureColors()
{
    double temps[] = {
        left_motor_f.temperature(celsius),
        left_motor_b.temperature(celsius),
        left_motor_c.temperature(celsius),
        right_motor_f.temperature(celsius),
        right_motor_b.temperature(celsius),
        right_motor_c.temperature(celsius)};

    double maxTemp = temps[0];
    for (int i = 1; i < 6; i++)
        if (temps[i] > maxTemp)
            maxTemp = temps[i];

    if (maxTemp < 45)
        Brain.Screen.setFillColor(green);
    else if (maxTemp < 55)
        Brain.Screen.setFillColor(orange);
    else
        Brain.Screen.setFillColor(red);

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Max Motor Temp: %.1f C", maxTemp);
}

// -------------------- DRIVER CONTROL --------------------

void drivercontrol()
{

    int currentAngle = 0;
    int step = 4;
    int descoreAngleUpdated = 0;
    int descoreStep = 8;
    bool increasing = true;
    bool lastXState = false;

    change_shooting.setPosition(0, degrees);

    controller1.ButtonRight.pressed(toggleArm);
    controller1.ButtonY.pressed(toggleWing);
    controller1.ButtonDown.pressed(toggleDriveClutch);
    controller1.ButtonL1.pressed(toggleMechClutch);
    controller1.ButtonR2.pressed(toggleShootingM);
    //   controller1.ButtonR2.released(toggleShootingM);
    // controller1.ButtonR1.pressed(toggleShootingL);
    // controller1.ButtonR1.released(toggleShootingL);
    controller1.ButtonR1.pressed(toggleShootingL);
    controller1.ButtonA.pressed(drivetrainLock);
    while (true)
    {

        drivetrain_movement();

        double mechScale = mech_clutch ? 0.4 : 1.0;
        bool xPressed = controller1.ButtonX.pressing();
        if (controller1.ButtonB.pressing())
        {
            // intake
            intake_middle_f.spin(reverse, 100 * mechScale, pct);
            store_basket.spin(reverse, 100 * mechScale, pct);

            long_middle_b.spin(reverse, 100 * mechScale, pct);
            change_shooting.stop(hold);
        }
        else if (controller1.ButtonL2.pressing())
        {
            // outtake
            intake_middle_f.spin(forward, 100 * mechScale, pct);
            store_basket.spin(forward, 100 * mechScale, pct);

            long_middle_b.spin(fwd, 100 * mechScale, pct);
        }
        else if (middleGoalRunning)
        {
            // middle

            intake_middle_f.spin(reverse, 100 * mechScale, pct);
            store_basket.spin(reverse, 100 * mechScale, pct);

            long_middle_b.spin(reverse, 100 * mechScale, pct);

            // middle
        }
        else if (controller1.ButtonUp.pressing())
        {
            currentAngle += step;

            change_shooting.spinToPosition(
                currentAngle, degrees, 100, velocityUnits::pct);
            baseAngle = currentAngle;
        }

        else if (controller1.ButtonLeft.pressing())
        {
            currentAngle -= step;

            change_shooting.spinToPosition(
                currentAngle, degrees, 100, velocityUnits::pct);
            baseAngle = currentAngle;
        }

        /*  else if (controller1.ButtonR1.pressing())
          {
                  // long goal
              intake_middle_f.spin(reverse, 100 * mechScale, pct);
              store_basket.spin(reverse, 100 * mechScale, pct);

              long_middle_b.spin(reverse, 100 * mechScale, pct);

          }
              */

        else if (longGoalRunning)
        {

            intake_middle_f.spin(reverse, 100 * mechScale, pct);
            store_basket.spin(reverse, 100 * mechScale, pct);
            long_middle_b.spin(reverse, 100 * mechScale, pct);
        }

        else if (xPressed)
        {
            if (!lastXState)
            {
                increasing = !increasing;
            }

            if (increasing)
            {
                descoreAngleUpdated += descoreStep;
            }
            else
            {
                descoreAngleUpdated -= descoreStep;
            }

            descore.spinToPosition(
                descoreAngleUpdated, degrees, 100, velocityUnits::pct);
        }

        // Update last state AFTER logic

        else
        {
            intake_middle_f.stop();
            store_basket.stop();
            long_f.stop();
            long_middle_b.stop();
            unclog.stop();
        }

        updateMotorTemperatureColors();
        if (!xPressed && lastXState)
        {

            descoreBaseAngle = descoreAngleUpdated;
        }
        wait(20, msec);
        lastXState = xPressed;
    }
}

void drive_cm(double distance_cm, double left_speed, double right_speed, bool intake_bool, double intake_wait)
{
    double ticks = distance_cm * TICKS_PER_CM;

    left_motor_f.setVelocity(left_speed, pct);
    left_motor_b.setVelocity(left_speed, pct);
    left_motor_c.setVelocity(left_speed, pct);
    right_motor_f.setVelocity(right_speed, pct);
    right_motor_b.setVelocity(right_speed, pct);
    right_motor_c.setVelocity(right_speed, pct);

    left_drive_smart.spinFor(fwd, ticks, deg, false);
    right_drive_smart.spinFor(fwd, ticks, deg, true);

    if (intake_bool)
    {
        intake_middle_f.spin(forward, 100, pct);
        store_basket.spin(reverse, 100, pct);
        unclog.spin(reverse, 100, pct);

        wait(intake_wait, sec);
        intake_middle_f.stop();
        store_basket.stop();
        unclog.stop();
    }
    wait(SETTLE_TIME, sec);
}

void turn_deg(double degrees, double left_speed, double right_speed)
{
    double ticks = fabs(degrees) * TICKS_PER_DEG_TURN;
    left_speed /= 2;
    right_speed /= 2;

    left_motor_f.setVelocity(left_speed, pct);
    left_motor_b.setVelocity(left_speed, pct);
    right_motor_f.setVelocity(right_speed, pct);
    right_motor_b.setVelocity(right_speed, pct);

    if (degrees > 0)
    {
        left_drive_smart.spinFor(fwd, ticks, deg, false);
        right_drive_smart.spinFor(reverse, ticks, deg, true);
    }
    else
    {
        left_drive_smart.spinFor(reverse, ticks, deg, false);
        right_drive_smart.spinFor(fwd, ticks, deg, true);
    }

    wait(SETTLE_TIME, sec);
}

void driveSmooth(double distance_cm,
                 int maxSpeedLeft,
                 int maxSpeedRight,
                 double accelDist,
                 double decelDist,
                 bool intake_bool,
                 double intake_wait)
{
    int dir = (distance_cm >= 0) ? 1 : -1;
    distance_cm = fabs(distance_cm);

    double startLeft = left_motor_f.position(deg);
    double startRight = right_motor_f.position(deg);
    double targetTicks = distance_cm * TICKS_PER_CM;

    const double minSpeed = 5;
    const int loopInterval = 20; // ms

    if (accelDist < 1)
        accelDist = 1;
    if (decelDist < 1)
        decelDist = 1;

    /* ================== INTAKE START ================== */
    timer intakeTimer;
    bool intakeRunning = false;

    if (intake_bool && intake_wait > 0)
    {

        intake_middle_f.spin(reverse, 100, pct);
        store_basket.spin(reverse, 100, pct);
        long_middle_b.spin(reverse, 100, pct);
        intakeTimer.reset();
        intakeRunning = true;
    }

    /* ================================================== */

    while (true)
    {
        double leftTicks = fabs(left_motor_f.position(deg) - startLeft);
        double rightTicks = fabs(right_motor_f.position(deg) - startRight);
        double avgTicks = (leftTicks + rightTicks) / 2.0;

        if (avgTicks >= targetTicks)
            break;

        /* ---------- INTAKE TIME CONTROL ---------- */
        if (intakeRunning && intakeTimer.time(sec) >= intake_wait)
        {
            intake_middle_f.stop();
            store_basket.stop();
            long_middle_b.stop();
            intakeRunning = false;
        }
        /* ----------------------------------------- */

        double remainingTicks = targetTicks - avgTicks;

        double leftSpeed = maxSpeedLeft;
        double rightSpeed = maxSpeedRight;

        // Acceleration
        double accelTicks = accelDist * TICKS_PER_CM;
        if (avgTicks < accelTicks)
        {
            leftSpeed = maxSpeedLeft * (avgTicks / accelTicks);
            rightSpeed = maxSpeedRight * (avgTicks / accelTicks);
        }

        // Deceleration
        double decelTicks = decelDist * TICKS_PER_CM;
        if (remainingTicks < decelTicks)
        {
            double decelLeft = maxSpeedLeft * (remainingTicks / decelTicks);
            double decelRight = maxSpeedRight * (remainingTicks / decelTicks);

            if (decelLeft < leftSpeed)
                leftSpeed = decelLeft;
            if (decelRight < rightSpeed)
                rightSpeed = decelRight;
        }

        if (leftSpeed > maxSpeedLeft)
            leftSpeed = maxSpeedLeft;
        if (rightSpeed > maxSpeedRight)
            rightSpeed = maxSpeedRight;

        if (leftSpeed < minSpeed)
            leftSpeed = minSpeed;
        if (rightSpeed < minSpeed)
            rightSpeed = minSpeed;

        left_drive_smart.setVelocity(leftSpeed, pct);
        right_drive_smart.setVelocity(rightSpeed, pct);

        left_drive_smart.spin(dir == 1 ? fwd : reverse);
        right_drive_smart.spin(dir == 1 ? fwd : reverse);

        wait(loopInterval, msec);
    }

    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);

    /* ---------- SAFETY STOP ---------- */
    if (intakeRunning)
    {
        intake_middle_f.stop();
        store_basket.stop();
    }
}

// Smooth turn
void turnSmooth(double angle_deg, int maxSpeed, double accelAngle, double decelAngle)
{
    int dir = (angle_deg >= 0) ? 1 : -1;
    angle_deg = fabs(angle_deg);

    double start = left_motor_f.position(deg);
    double targetTicks = angle_deg * TICKS_PER_DEG_TURN;

    const double minSpeed = 3;
    const int loopInterval = 20;

    while (true)
    {
        double currentTicks = fabs(left_motor_f.position(deg) - start);

        if (currentTicks >= targetTicks)
            break;

        double remaining = targetTicks - currentTicks;
        double speed = maxSpeed;

        double accelTicks = accelAngle * TICKS_PER_DEG_TURN;
        double decelTicks = decelAngle * TICKS_PER_DEG_TURN;

        // accel
        if (currentTicks < accelTicks)
            speed = maxSpeed * (currentTicks / accelTicks);

        // decel
        if (remaining < decelTicks)
        {
            double decelSpeed = maxSpeed * (remaining / decelTicks);
            if (decelSpeed < speed)
                speed = decelSpeed;
        }

        if (speed < minSpeed)
            speed = minSpeed;
        if (speed > maxSpeed)
            speed = maxSpeed;

        left_drive_smart.setVelocity(speed, pct);
        right_drive_smart.setVelocity(speed, pct);

        left_drive_smart.spin(dir == 1 ? fwd : reverse);
        right_drive_smart.spin(dir == 1 ? reverse : fwd);

        wait(loopInterval, msec);
    }

    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);
}
/*
void turnSmooth(double angle_deg, int maxSpeed, double accelAngle, double decelAngle)
{
    int dir = (angle_deg >= 0) ? 1 : -1;
    angle_deg = fabs(angle_deg);

    double startLeft = left_motor_f.position(deg);
    double startRight = right_motor_f.position(deg);
    double targetTicks = angle_deg * TICKS_PER_DEG_TURN;

    const double minSpeed = 5;
    const int loopInterval = 20;

    if (accelAngle < 1)
        accelAngle = 1;
    if (decelAngle < 1)
        decelAngle = 1;

    while (true)
    {
        double leftTicks = fabs(left_motor_f.position(deg) - startLeft);
        double rightTicks = fabs(right_motor_f.position(deg) - startRight);
        double avgTicks = (leftTicks + rightTicks) / 2.0;

        if (avgTicks >= targetTicks)
            break;

        double remainingTicks = targetTicks - avgTicks;
        double speed = maxSpeed;

        // Acceleration
        double accelTicks = accelAngle * TICKS_PER_DEG_TURN;
        if (avgTicks < accelTicks)
        {
            speed = maxSpeed * (avgTicks / accelTicks);
        }

        // Deceleration
        double decelTicks = decelAngle * TICKS_PER_DEG_TURN;
        if (remainingTicks < decelTicks)
        {
            double decelSpeed = maxSpeed * (remainingTicks / decelTicks);
            if (decelSpeed < speed)
                speed = decelSpeed;
        }

        if (speed > maxSpeed)
            speed = maxSpeed;
        if (speed < minSpeed)
            speed = minSpeed;

        left_drive_smart.setVelocity(speed, pct);
        right_drive_smart.setVelocity(speed, pct);

        left_drive_smart.spin(dir == 1 ? fwd : reverse);
        right_drive_smart.spin(dir == 1 ? reverse : fwd);

        wait(loopInterval, msec);
    }

    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);
}
*/
void turnSmooth1(double angle_deg,
                 int maxSpeed,
                 double accelAngle,
                 double decelAngle,
                 bool intake_bool,
                 double intake_wait)
{
    int dir = (angle_deg >= 0) ? 1 : -1;
    angle_deg = fabs(angle_deg);

    double startLeft = left_motor_f.position(deg);
    double startRight = right_motor_f.position(deg);
    double targetTicks = angle_deg * TICKS_PER_DEG_TURN;

    const double minSpeed = 5;
    const int loopInterval = 20; // ms

    if (accelAngle < 1)
        accelAngle = 1;
    if (decelAngle < 1)
        decelAngle = 1;

    /* ================== INTAKE START ================== */
    timer intakeTimer;
    bool intakeRunning = false;

    if (intake_bool && intake_wait > 0)
    {
        intake_middle_f.spin(forward, 100, pct);
        long_middle_b.spin(forward, 100, pct);
        store_basket.spin(reverse, 100, pct);
        unclog.spin(reverse, 100, pct);

        intakeTimer.reset();
        intakeRunning = true;
    }
    /* ================================================== */

    while (true)
    {
        double leftTicks = fabs(left_motor_f.position(deg) - startLeft);
        double rightTicks = fabs(right_motor_f.position(deg) - startRight);
        double avgTicks = (leftTicks + rightTicks) / 2.0;

        if (avgTicks >= targetTicks)
            break;

        /* ---------- INTAKE TIME CONTROL ---------- */
        if (intakeRunning && intakeTimer.time(sec) >= intake_wait)
        {
            intake_middle_f.stop();
            long_middle_b.stop();
            store_basket.stop();
            unclog.stop();
            intakeRunning = false;
        }
        /* ----------------------------------------- */

        double remainingTicks = targetTicks - avgTicks;
        double speed = maxSpeed;

        // Acceleration
        double accelTicks = accelAngle * TICKS_PER_DEG_TURN;
        if (avgTicks < accelTicks)
        {
            speed = maxSpeed * (avgTicks / accelTicks);
        }

        // Deceleration
        double decelTicks = decelAngle * TICKS_PER_DEG_TURN;
        if (remainingTicks < decelTicks)
        {
            double decelSpeed = maxSpeed * (remainingTicks / decelTicks);
            if (decelSpeed < speed)
                speed = decelSpeed;
        }

        if (speed > maxSpeed)
            speed = maxSpeed;
        if (speed < minSpeed)
            speed = minSpeed;

        left_drive_smart.setVelocity(speed, pct);
        right_drive_smart.setVelocity(speed, pct);

        left_drive_smart.spin(dir == 1 ? fwd : reverse);
        right_drive_smart.spin(dir == 1 ? reverse : fwd);

        wait(loopInterval, msec);
    }

    left_drive_smart.stop(brake);
    right_drive_smart.stop(brake);

    /* ---------- SAFETY STOP ---------- */
    if (intakeRunning)
    {
        intake_middle_f.stop();
        long_middle_b.stop();
        store_basket.stop();
        unclog.stop();
    }
}

void long_goal(double wait_time, double speed)
{
    // Move shooter to shoot position
    intake_middle_f.spin(fwd, 90, pct);
    store_basket.spin(fwd, 90, pct);
    long_middle_b.spin(fwd, 90, pct);
    wait(0.2, sec);
    change_shooting.spinToPosition(0, degrees, 100, velocityUnits::pct);

    intake_middle_f.spin(reverse, speed, pct);
    store_basket.spin(reverse, speed, pct);
    long_middle_b.spin(reverse, speed, pct);

    wait(wait_time, sec);

    intake_middle_f.stop();
    store_basket.stop();
    long_middle_b.stop();

    // Reset shooter
    change_shooting.spinToPosition(75, degrees, 100, velocityUnits::pct);
}

void upper_middle(double wait_time, double speed)
{

    change_shooting.spinToPosition(130, degrees, 100, velocityUnits::pct);

    intake_middle_f.spin(reverse, speed, velocityUnits::pct);
    store_basket.spin(reverse, speed, velocityUnits::pct);
    long_middle_b.spin(reverse, speed, velocityUnits::pct);

    wait(wait_time, sec);

    intake_middle_f.stop();
    store_basket.stop();
    long_middle_b.stop();

    change_shooting.spinToPosition(75, degrees, 100, velocityUnits::pct);
}
void outtake(double wait_time, double speed)
{
    intake_middle_f.spin(forward, speed, pct);
    store_basket.spin(forward, speed, pct);

    long_middle_b.spin(fwd, speed, pct);

    change_shooting.stop(hold);

    wait(wait_time, sec);

    intake_middle_f.stop();
    store_basket.stop();
    long_middle_b.stop();
}
void intake(double wait_time, double speed)
{
    intake_middle_f.spin(reverse, speed, pct);
    store_basket.spin(reverse, speed, pct);
    long_middle_b.spin(reverse, speed, pct);

    change_shooting.stop(hold);

    wait(wait_time, sec);

    intake_middle_f.stop();
    store_basket.stop();
    long_middle_b.stop();
}

void autonomous()

{
  // skills ruigan v1
    drivetrainLock();
    descore.setPosition(0, degrees);
    motor19.setPosition(0, degrees);
    descore.spinToPosition(400, degrees, 100, velocityUnits::pct);
    deployExtensionsions();

    driveSmooth(-240, 60, 60, 0, 0, false, 0);
    wait(0.5,sec)
    driveSmooth(17, 70, 0, 0, 0, false, 0);
    wait(0.5,sec)
    driveSmooth(-85, 70, 70, 0, 0, false, 0);
    wait(0.5,sec)
    driveSmooth(-25, 70, 0, 0, 0, false, 0);
    wait(0.5,sec)
    driveSmooth(240, 60, 60, 0, 0, false, 0);
    wait(0.5,sec)
    turnSmooth(-60, 70, 0, 0);
  

  

    // skills ruigan v2

    descore.setPosition(0, degrees);
    motor19.setPosition(0, degrees);

    deployExtensionsions();

    driveSmooth(-95, 60, 60, 0, 0, true, .5);
    wait(0.5,sec)
    arm_down(0.1);
    wait(0.5, sec);
    arm_up(0.1);
    wait(1, sec);

    driveSmooth(-16, 0, 70, 0, 0, false, 0);
    wait(0.5,sec)
    driveSmooth(-5, 30, 30, 0, 0, false, 0);
    arm_down(0.1);
    driveSmooth(60, 30, 30, 0, 0, true, 1);

  

      // ruigan skills v3
    drivetrainLock();
    descore.setPosition(0, degrees);
    motor19.setPosition(0, degrees);
    change_shooting.setPosition(0, degrees);
    deployExtensionsions();

    arm_down_skills(0.1);
    driveSmooth(25, 30, 30, 0, 0, true, 2);
    intake(1, 100);
    Drivetrain.setTurnVelocity(60, percent); 
    Drivetrain.turnFor(right, 10, degrees);

    wait(2, sec);
    arm_up_skills(0.1);
    driveSmooth(-15, 30, 30, 0, 0, true, 2.5);
    wait(0.5, sec);
    Drivetrain.turnFor(right, 24, degrees);
    wait(0.5, sec);
    driveSmooth(-110.3, 80, 80, 0, 0, true, 2);
    intake(3, 100);
    driveSmooth(80, 80, 80, 0, 40, false, 0);
    wait(0.5, sec);
    Drivetrain.turnFor(right, 35, degrees);
    wait(0.5, sec);
    driveSmooth(-140, 80, 80, 0, 40, false, 0);
    wait(0.5, sec);

    driveSmooth(15, 0, 40, 0, 0, false, 0);
    driveSmooth(10, 60, 60, 0, 0, false, 0);

    upper_middle(5, 50);
}
// -------------------- MAIN --------------------
competition Competition;

int main()
{

    // autonomous();
    motor19.setMaxTorque(45, pct);
    Competition.drivercontrol(drivercontrol);
    Competition.autonomous(autonomous);

    while (true)
    {
        wait(100, msec);
    }
}
