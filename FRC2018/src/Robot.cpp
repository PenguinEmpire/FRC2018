/*
 * Robot.cpp
 * Created 2018-01-07
 *
 * Last Update: 2018-01-07
 */

#include "PenguinEmpire.h"

const int pwm0 = 0;
const int pwm1 = 1;
const int pwm2 = 2;
const int pwm3 = 3;

const int usb0 = 0;
const int usb1 = 1;
const int usb2 = 2;


Robot::Robot() : // Robot constructor - Initialize all subsystem and component classes here
	left(usb0),
	right(usb1),
	handheld(usb2),
	l1(pwm0),
	l2(pwm1),
	r1(pwm2),
	r2(pwm3)
{

}

Robot::~Robot() { // Robot destructor - Delete pointer values here

}

void Robot::RobotInit() { // Runs only when robot code starts initially

}

void Robot::AutonomousInit() { // Runs at start of autonomous phase, only once

}

void Robot::AutonomousPeriodic() { // Looped through iteratively during autonomous phase - do not put loops here!

}

/*
 * Autonomous Functions:
 * None
 */

void Robot::TeleopInit() { // Runs at start of teleoperated phase, only once

}

void Robot::TeleopPeriodic() { // Looped through iteratively during teleoperated phase - do not put loops here! Only teleop function calls!

}

/*
 * Teleop Functions:
 * None
 */

void Robot::TestInit() { // Runs at start of test phase, only once

}

void Robot::TestPeriodic() { // Looped through iteratively during test phase - do not put loops here!

}

/*
 * Test Functions:
 * None
 */



START_ROBOT_CLASS(Robot)
