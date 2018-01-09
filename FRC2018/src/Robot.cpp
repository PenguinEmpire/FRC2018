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
	leftauto = false;

	m_left.init(&left);
	m_right.init(&right);
	m_handheld.init(&handheld);

}

Robot::~Robot() { // Robot destructor - Delete pointer values here
	delete m_left, m_right, m_handheld;
}

void Robot::RobotInit() { // Runs only when robot code starts initially

}

void Robot::AutonomousInit() { // Runs at start of autonomous phase, only once
	CheckSide();
}

void Robot::AutonomousPeriodic() { // Looped through iteratively during autonomous phase - do not put loops here!

}

/*
 * Autonomous Functions:
 * CheckSide
 * LeftAuto - wip
 * RightAuto - wip
 */

void Robot::CheckSide() {
	std::string gameData;
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if(gameData[0] == 'L')
	{
		leftauto = true;
	} else {
		leftauto = false;
	}
}

void Robot::LeftAuto() {

}

void Robot::RightAuto() {

}

void Robot::TeleopInit() { // Runs at start of teleoperated phase, only once

}

void Robot::TeleopPeriodic() { // Looped through iteratively during teleoperated phase - do not put loops here! Only teleop function calls!
	TankDrive();
}

/*
 * Teleop Functions:
 * TankDrive - temp
 */

void Robot::TankDrive() {
	double leftInput;
	double rightInput;

	leftInput = left.GetRawAxis(1);
	rightInput = right.GetRawAxis(1);

	if(fabs(leftInput) > 0.3) {
		l1.Set(leftInput * -0.65);
		l2.Set(leftInput * -0.65);
	}
	else
	{
		l1.Set(0);
		l2.Set(0);
	}

	if(fabs(rightInput) > 0.3) {
		r1.Set(rightInput * -0.65);
		r2.Set(rightInput * -0.65);
	}
	else
	{
		r1.Set(0);
		r2.Set(0);
	}
}

void Robot::TestInit() { // Runs at start of test phase, only once

}

void Robot::TestPeriodic() { // Looped through iteratively during test phase - do not put loops here!

}

/*
 * Test Functions:
 * None
 */


START_ROBOT_CLASS(Robot)
