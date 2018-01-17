/*
 * Robot.cpp
 * Created 2018-01-07
 *
 * Last Update: 2018-01-09
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
	leftswitch = false;
	leftscale = false;

	// Point MyJoysticks to Joysticks
	m_left.init(&left);
	m_right.init(&right);
	m_handheld.init(&handheld);

	ahrs = new AHRS(SerialPort::kMXP);

	fpos = Center;

	autosteps = {};
	curstep = 0;
	numsteps = 0;
}

Robot::~Robot() { // Robot destructor - Delete pointer values here

}

void Robot::RobotInit() { // Runs only when robot code starts initially
	r1.SetInverted(true);
	r2.SetInverted(true);
}

void Robot::AutonomousInit() { // Runs at start of autonomous phase, only once
	CheckSide();
	CheckPos();
	if (fpos == Left) {
		LeftAuto();
	}
	else if (fpos == Center) {
		CenterAuto();
	}
	else if (fpos == Right) {
		RightAuto();
	}
}

void Robot::AutonomousPeriodic() { // Looped through iteratively during autonomous phase - do not put loops here!

}

/*
 * Autonomous Functions:
 * CheckSide
 * CheckPos - empty
 * LeftAuto - empty
 * RightAuto - empty
 */

void Robot::CheckSide() {
	/*
	 * Get FMS, set left switch and scale bools
	 */

	std::string gamedata;
	gamedata = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if (gamedata[0] == 'L') {
		leftswitch = true;
	} else {
		leftswitch = false;
	}

	if (gamedata[1] == 'L') {
		leftscale = true;
	} else {
		leftscale = false;
	}
}

void Robot::CheckPos() {
	/*
	 * Check TBD physical switch
	 */
}

void Robot::LeftAuto() {
	/*
	 * Priority:
	 * Scale
	 * Switch
	 * Auto Line
	 */
}

void Robot::CenterAuto() {
	/*
	 * Use FMS and place cube
	 */
}

void Robot::RightAuto() {
	/*
	 * Priority:
	 * Scale
	 * Switch
	 * Auto Line
	 */
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
	/*
	 * Get left and right stick values, set left and right motors accordingly
	 *
	 * Temp: may add control overrides
	 */
	double leftInput;
	double rightInput;

	leftInput = left.GetRawAxis(1);
	rightInput = right.GetRawAxis(1);

	double inputMultiplier = 0.65;

	if(fabs(leftInput) > 0.3) {
		l1.Set(leftInput * -inputMultiplier);
		l2.Set(leftInput * -inputMultiplier);
	}
	else
	{
		l1.Set(0);
		l2.Set(0);
	}

	if(fabs(rightInput) > 0.3) {
		r1.Set(rightInput * -inputMultiplier);
		r2.Set(rightInput * -inputMultiplier);
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

Robot::Step::Step(Robot *r, StepType steptype, std::vector<double> parameters) : robot(r) {
	params = parameters;
	type = steptype;
	complete = false;
	setup = true;
}

Robot::Step::~Step() {
	delete robot;
}

void Robot::Step::Run() {
	/*
	 * Checks what the step is, then reads in the arguments and executes it
	 */
	if (type == reset) {
		/*
		 * Stop stuff
		 */
	}
	else if (type == encodermove) {
		if (setup) {
			/*
			 * Stop stuff and initialize
			 */
			setup = false;
		}
//		double speed, distance;
//		if (params.size() == 2) {
//			speed = params[0];
//			distance = params[1];
//		}
//		else {
//			speed = 0;
//			distance = 0;
//		}

		/*
		 * Move at speed until encoders reach target value
		 */
	}
	else if (type == gyroturn) {
		if (setup) {
			/*
			 * Stop stuff
			 */
			setup = false;
		}
//		double speed, angle;
//		if (params.size() == 2) {
//			speed = params[0];
//			angle = params[1];
//		}
//		else {
//			speed = 0;
//			angle = 0;
//		}

		/*
		 * Turn at speed until gyro reaches target value
		 */
	}
}


START_ROBOT_CLASS(Robot)
