/*
 * Robot.cpp
 * Created 2018-01-07
 *
 * Last Update: 2018-01-20
 */

#include "PenguinEmpire.h"

const int pch0 = 0;
const int pch1 = 1;
const int pch2 = 2;
const int pch3 = 3;

const int pcm0 = 0;

const int pwm0 = 0;
const int pwm1 = 1;
const int pwm2 = 2;
const int pwm3 = 3;

const int usb0 = 0;
const int usb1 = 1;
const int usb2 = 2;


Robot::Robot() : // Robot constructor - Initialize all subsystem and component classes here
	leftStick(usb0),
	rightStick(usb1),
	handheld(usb2),
	l1(pwm0),
	l2(pwm1),
	r1(pwm2),
	r2(pwm3),
	compressor(pcm0),
	leftGearbox(pcm0, pch0, pch1),
	rightGearbox(pcm0, pch2, pch3)
{
	leftSwitch = false;
	leftScale = false;
	controlOverride = false;
	gyroTurning = false;
	compressorEnabled = false;
	pressureStatus = false;
	current = 0.0;

	ahrs = new AHRS(SerialPort::kMXP);

	fpos = centerPos;

	autosteps = {};
	curstep = 0;
	numsteps = 0;

	l1.SetExpiration(0.1);
	l2.SetExpiration(0.1);
	r1.SetExpiration(0.1);
	r2.SetExpiration(0.1);
}

Robot::~Robot() { // Robot destructor - Delete pointer values here

}

void Robot::RobotInit() { // Runs only when robot code starts initially
	r1.SetInverted(true);
	r2.SetInverted(true);

	compressorEnabled = compressor.Enabled();
	pressureStatus = compressor.GetPressureSwitchValue();
	current = compressor.GetCompressorCurrent();
	compressor.SetClosedLoopControl(true);
}

void Robot::AutonomousInit() { // Runs at start of autonomous phase, only once
	CheckSide();
	CheckPos();
	if (fpos == leftPos) {
		LeftAuto();
	}
	else if (fpos == centerPos) {
		CenterAuto();
	}
	else if (fpos == rightPos) {
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
		leftSwitch = true;
	} else {
		leftSwitch = false;
	}

	if (gamedata[1] == 'L') {
		leftScale = true;
	} else {
		leftScale = false;
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
	// Point MyJoysticks to Joysticks
	m_left.Init(&leftStick);
	m_right.Init(&rightStick);
	m_handheld.Init(&handheld);
}

void Robot::TeleopPeriodic() { // Looped through iteratively during teleoperated phase - do not put loops here! Only teleop function calls!
	// Read in button value changes at start of teleop iteration
	m_left.ReadJoystick();
	m_right.ReadJoystick();
	m_handheld.ReadJoystick();

	if (!controlOverride) {
		TankDrive();
	}

	GyroTurn(m_left.ReadButton(11), -90, 1.0);
	GyroTurn(m_left.ReadButton(10), 180, 1.0);
	GyroTurn(m_left.ReadButton(12), 90, 1.0);
	ManualShiftGears(m_right.ReadButton(6), m_right.ReadButton(4));
}

/*
 * Teleop Functions:
 * SetLeftSpeed
 * SetRightSpeed
 * TankDrive
 * GyroTurn
 * ManualShiftGears - wip
 */
void Robot::SetLeftSpeed(float speed) {
	l1.Set(speed);
	l2.Set(speed);
}

void Robot::SetRightSpeed(float speed) {
	r1.Set(speed);
	r2.Set(speed);
}

void Robot::StopMotors() {
	SetLeftSpeed(0.0);
	SetRightSpeed(0.0);
}

void Robot::TankDrive() {
	/*
	 * Get left and right stick values, set left and right motors accordingly
	 */

	double leftInput;
	double rightInput;

	leftInput = leftStick.GetRawAxis(1);
	rightInput = rightStick.GetRawAxis(1);

	double inputMultiplier = 0.65;

	if(fabs(leftInput) > 0.3) {
		SetLeftSpeed(leftInput * -inputMultiplier);
	}
	else
	{
		SetLeftSpeed(0.0);
	}

	if(fabs(rightInput) > 0.3) {
		SetRightSpeed(rightInput * -inputMultiplier);
	}
	else
	{
		r1.Set(0);
		r2.Set(0);
	}
}

void Robot::GyroTurn(bool btn, float speed, double angle) { // Turn based on button value
	if (btn || gyroTurning) {
		if (angle < 0) {
			if (ahrs->GetYaw() > angle) { // Turn counterclockwise
				gyroTurning = true;
				controlOverride = true;
				SetLeftSpeed(-speed);
				SetRightSpeed(speed);
			}
			else {
				StopMotors();
				gyroTurning = false;
				controlOverride = false;
			}
		}
		else {
			if (ahrs->GetYaw() < angle) { // Turn clockwise
				gyroTurning = true;
				controlOverride = true;
				SetLeftSpeed(speed);
				SetRightSpeed(-speed);
			}
			else {
				StopMotors();
				gyroTurning = false;
				controlOverride = false;
			}
		}
	}
}

void Robot::GyroTurn(int pov, float speed) { // Turn based on POV
	/*
	 * Get the direction of the POV, then turn at speed to the angle relative to the robot
	 */
	if (pov == 0 || gyroTurning) { // Turn 90 degrees clockwise
		if (ahrs->GetYaw() > -90) {
			gyroTurning = true; // Allows us to continue turning to the angle if pov is released
			controlOverride = true; // Prevents tank drive control during turn
			SetLeftSpeed(-speed);
			SetRightSpeed(speed);
		}
		else {
			StopMotors();
			gyroTurning = false; // Stop turning if POV released
			controlOverride = false; // Return manual control of drive
			ahrs->Reset(); // Reset yaw to zero
		}
	}
	else if (pov == 180 || gyroTurning) { // Turn 180 degrees clockwise
		if (ahrs->GetYaw() < 180) {
			gyroTurning = true;
			controlOverride = true;
			SetLeftSpeed(-speed);
			SetRightSpeed(speed);
		}
		else {
			StopMotors();
			gyroTurning = false;
			controlOverride = false;
			ahrs->Reset();
		}
	}
	else if (pov == 270 || gyroTurning) { // Turn 90 degrees counterclockwise
		if (ahrs->GetYaw() < 90) {
			gyroTurning = true;
			controlOverride = true;
			SetLeftSpeed(-speed);
			SetRightSpeed(speed);
		}
		else {
			StopMotors();
			gyroTurning = false;
			controlOverride = false;
			ahrs->Reset();
		}
	}
}

void Robot::ManualShiftGears(bool upBtn, bool downBtn) {
	if (upBtn) {
		ShiftGears("up");
	}

	if (downBtn) {
		ShiftGears("down");
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

/*
 * Other Functions:
 * ShiftGears - wip
 */

void Robot::ShiftGears(std::string dir) {
	/*
	 * Initiate or uninitiate gearboxes to change gear ratio
	 */
	DoubleSolenoid::Value state;
	if (dir == "up") {
		state = DoubleSolenoid::kForward;
	}
	else {
		state = DoubleSolenoid::kReverse;
	}

	leftGearbox.Set(state);
	rightGearbox.Set(state);
}

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
	switch (type) {
	case reset:
		/*
		 * Stop stuff
		 */
		break;
	case encoderMove:
		if (setup) {
			/*
			 * Stop stuff and initialize
			 */
			setup = false;
		}
		else {
//			double speed, distance;
//			if (params.size() == 2) {
//				speed = params[0];
//				distance = params[1];
//			}
//			else {
//				speed = 0;
//				distance = 0;
//			}
		}

		/*
		 * Move at speed until encoders reach target value
		 */
		break;
	case gyroTurn:
		if (setup) {
			/*
			 * Stop stuff
			 */
			setup = false;
		}
		else {
//			double speed, angle;
//			if (params.size() == 2) {
//				speed = params[0];
//				angle = params[1];
//			}
//			else {
//				speed = 0;
//				angle = 0;
//			}
		}

		/*
		 * Turn at speed until gyro reaches target value
		 */
		break;
	}
}


START_ROBOT_CLASS(Robot)
