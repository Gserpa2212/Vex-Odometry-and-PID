//VexV5 odometry and PID Implementation
#pragma region VEXcode Generated Robot Configuration
// Make sure all required headers are included.
#include <sudio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>


#include "vex.h"

using namespace vex;

// Brain should be defined by default
brain Brain;


// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
motor L1 = motor(PORT1, ratio18_1, false);

motor L2 = motor(PORT2, ratio18_1, false);

motor R1 = motor(PORT4, ratio18_1, false);

motor R2 = motor(PORT5, ratio18_1, false);

potV2 LeftDeg = potV2(Brain.ThreeWirePort.H);
potV2 RightDeg = potV2(Brain.ThreeWirePort.G);
controller Controller1 = controller(primary);


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



// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

#pragma endregion VEXcode Generated Robot Configuration

#include "vex.h"
using namespace vex;

motor_group leftSide = motor_group(L1, L2);
motor_group rightSide = motor_group(R1, R2);

// PID Controller Class
class PID {
public:
    double kP, kI, kD;
    double error, lastError, integral, derivative;

    PID(double p, double i, double d) : kP(p), kI(i), kD(d), error(0), lastError(0), integral(0), derivative(0) {}

    double calculate(double target, double currentPos) {
        error = target - currentPos;
        integral += error;
        derivative = error - lastError;
        lastError = error;
        return kP * error + kI * integral + kD * derivative;
    }
};

// Odometry variables
double botAngle = 0.0;
double Ydist = 0.0;
double prevL = 0.0;
double prevR = 0.0;
double motorPower = 0.0;
int lPotPos = 0;
int rPotPos = 0;

// Reset potentiometer readings at start
void resetPot() {
    lPotPos = LeftDeg.angle(degrees);
    rPotPos = RightDeg.angle(degrees);
}

// Odometry Function
void odometry() {
    double trackWidth = 2.18;  // Distance between wheels
    double wheelDiameter = 2.75;
    double wheelCircumference = M_PI * wheelDiameter;
    double scalingFactor = wheelCircumference / 360.0;  // Convert degrees to distance

    // Store the last potentiometer readings
    double prevLReading = LeftDeg.angle(degrees);
    double prevRReading = RightDeg.angle(degrees);

    while (true) {
        // Read new potentiometer values
        double newL = LeftDeg.angle(degrees);
        double newR = RightDeg.angle(degrees);

        // Calculate wheel movement
        double deltaL = newL - prevLReading;
        double deltaR = newR - prevRReading;

        // Correct for wraparound at 0°/360°
        if (deltaL > 180) deltaL -= 360;
        if (deltaL < -180) deltaL += 360;
        if (deltaR > 180) deltaR -= 360;
        if (deltaR < -180) deltaR += 360;

        // Convert degrees to actual movement distance
        deltaL *= scalingFactor;
        deltaR *= scalingFactor;

        // Apply sign correction (left should be negative)
        deltaL = -deltaL;

        // Update stored readings
        prevLReading = newL;
        prevRReading = newR;

        // Properly accumulate botAngle (in radians)
        double deltaTheta = (deltaR - deltaL) / trackWidth;
        if (fabs(deltaTheta) > 0.0001) {  // Avoid unnecessary drift
            botAngle += deltaTheta;
        }

        // Calculate Y distance using botAngle
        if (fabs(botAngle) < 0.01) {
            Ydist += (deltaL + deltaR) / 2;  // Approximate straight movement
        } else {
            double arcRadius = (deltaR + deltaL) / (2 * botAngle);
            Ydist += arcRadius * sin(botAngle);
        }

        // Debug output to Brain screen
        Brain.Screen.printAt(20, 40, "Bot Angle (deg): %.2f", botAngle * (180.0 / M_PI));
        Brain.Screen.printAt(20, 60, "LeftPot: %.4f", newL);
        Brain.Screen.printAt(20, 80, "RightPot: %.4f", newR);
        Brain.Screen.printAt(20, 100, "Ydist: %.4f", Ydist);

        this_thread::sleep_for(15);
    }
}

// Motor Controller for Movement
void moveStraight(double targetDistance, PID& controller) {
    double targetY = Ydist + targetDistance;

    while (true) {
        double motorPower = controller.calculate(targetY, Ydist);

        // Clamp motor power using if statements
        if (motorPower > 100) motorPower = 100;
        if (motorPower < -100) motorPower = -100;

        if (fabs(controller.error) < 2) {
            leftSide.stop();
            rightSide.stop();
            break;
        }

        leftSide.spin(forward, motorPower, percent);
        rightSide.spin(forward, motorPower, percent);
        wait(20, msec);
    }
}

// Motor Controller for Turning
void turnTo(double targetAngle, PID& controller) {
    while (true) {
        double motorPower = controller.calculate(targetAngle, botAngle);

        // Clamp motor power using if statements
        if (motorPower > 100) motorPower = 100;
        if (motorPower < -100) motorPower = -100;

        if (fabs(controller.error) < 2) {
            leftSide.stop();
            rightSide.stop();
            break;
        }

        leftSide.spin(forward, motorPower, percent);
        rightSide.spin(forward, -motorPower, percent);
        wait(20, msec);
    }
}

int main() {
    vexcodeInit();
    resetPot();  // Call BEFORE odometry starts

    thread mythread = thread(odometry);  // Start odometry thread

    PID pidDistanceController(0.5, 0.01, 0.1);
    PID pidAngleController(0.5, 0.01, 0.1);

    // Example usage: Move straight and then turn
    // moveStraight(1000, pidDistanceController);
    // turnTo(90, pidAngleController);

    wait(500, msec);  // Wait before ending

    return 0;
}