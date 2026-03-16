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
