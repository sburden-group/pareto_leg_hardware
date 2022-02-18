# Pareto Leg Hardware

contains all hardware (and control software) designs for the Pareto Leg


# Project TODOs:

* bring in label machine

## Hardware
* CAD
    * Add strain relief for robot wires. (3D printed part)
* PCBs
    * populate 8x encoder PCBs
    * test them
* make 2m wire harnesses for each robot
    * order wire harness stuff
    * longer ethernet cables (6 ft, very flexible)
    * make encoder power harnesses (Molex Microfit 3.0)
* label stuff
    * label all phases (1, 2, 3) to match
    * label motor wires (motor0 and motor1)
    * label encoder RJ45 wires (axis0, axis1)
    * label the robot's motors
* Test Rig
    * get a mount for the odrive


## Conventions
* get a picture that defines conventions:
    * motor 0
    * motor 1
    * positive theta rotation for motor 0
    * positive theta rotation for motor 1


## Software
* write controller
* calibrate both motors.
* make both move without errors.
