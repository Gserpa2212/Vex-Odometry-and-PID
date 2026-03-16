# Vex-Odometry-and-PID
Autonomous vex robot control using Odometry and PID

This project implements an autonomous navigation system for a differential drive VEX V5 robot using wheel odometry and PID feedback control. The robot estimates its position by measuring wheel rotation with potentiometer-based tracking sensors and integrates this data over time to determine its displacement.

Using this position estimate, the robot can execute controlled autonomous movements and track its Y-component position on the field in order to perform autonomous routines.

This robot was used in competition and won a Gold Medal at the SkillsUSA Massachusetts State Leadership and Skills Conference, earning qualification for the SkillsUSA National Leadership and Skills Conference.

System overview

The robot is built on the VEX V5 Robotics System platform and uses a differential drive drivetrain with external wheel tracking sensors.

Wheel rotation is measured using potentiometers connected to tracking wheels. These measurements are converted from angular rotation to linear displacement and used to estimate the robot's motion.

An odometry algorithm continuously calculates the robot's orientation and displacement while the robot moves. This process runs in a dedicated thread so that position updates occur independently of motion control commands.

Motion commands are executed using PID controllers that regulate the robot's movement toward a target distance or target heading.

Distance Controller  
Controls forward motion by regulating the robot’s displacement toward a target distance.

Turning Controller  
Controls robot rotation by regulating the robot’s heading toward a target angle.

The controller outputs motor power commands that are applied to the drivetrain. Motor power is clamped within valid limits to prevent commands outside the allowed range.

Robotics Concepts Demonstrated:

PID feedback control

differential drive kinematics

wheel odometry localization

multithreaded sensor processing

autonomous navigation


Key Features

Custom PID controller implementation used for both distance and heading control.

Wheel odometry algorithm that converts wheel rotation into displacement and heading estimates.

Multithreaded sensor processing so odometry updates run independently from motion control.

Competition-tested autonomous control system.

Notes and Limitations

The current implementation tracks forward displacement (Y direction) and robot heading rather than full two-dimensional position. Extending the system to track both X and Y position would enable coordinate-based navigation and more advanced autonomous path planning.

## Code Highlights

Below are several key parts of the control system used in the robot.

### PID Controller

A reusable PID controller class was implemented to control both forward movement and turning. The controller calculates motor output based on the difference between the target value and the current measured value.

```cpp
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
```

### Odometry Calculation

The robot estimates its heading using the difference between left and right wheel displacement.

```cpp
double deltaTheta = (deltaR - deltaL) / trackWidth;
botAngle += deltaTheta;
```

Forward displacement is estimated using the average motion of both wheels.

```cpp
Ydist += (deltaL + deltaR) / 2;
```

### Motion Control Loop

Motor power is calculated using the PID controller and applied to both sides of the drivetrain.

```cpp
double motorPower = controller.calculate(targetY, Ydist);

leftSide.spin(forward, motorPower, percent);
rightSide.spin(forward, motorPower, percent);
```

These control loops allow the robot to drive to target distances and perform controlled turns during autonomous routines.
