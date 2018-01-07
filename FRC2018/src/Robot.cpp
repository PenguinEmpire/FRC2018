/*
 * Robot.cpp
 * Created 2018-01-07
 *
 * Last Update: 2018-01-07
 */

#include "PenguinEmpire.h"

Robot::Robot() { // Robot constructor - Initialize all subsystem and component classes here

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

void Robot::TestPeriodic() { // Looped through iteratively during teleoperated phase - do not put loops here!

}

/*
 * Test Functions:
 * None
 */



START_ROBOT_CLASS(Robot)
