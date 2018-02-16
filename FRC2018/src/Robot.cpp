/*
 * Robot.cpp
 * Created 2018-01-07
 *
 * Last Update: 2018-01-20
 */

#include "PenguinEmpire.h"

const int dio0 = 0;

const int pch0 = 0;
const int pch1 = 1;
const int pch2 = 2;
const int pch3 = 3;
const int pch4 = 4;
const int pch5 = 5;
const int pch6 = 6;
const int pch7 = 7;

const int pcm0 = 0;

const int pwm0 = 0;
const int pwm1 = 1;
const int pwm2 = 2;
const int pwm3 = 3;
const int pwm4 = 4;
const int pwm5 = 5;
const int pwm6 = 6;
const int pwm7 = 7;

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
	leftIO(pwm4),
	rightIO(pwm5),
	lift1(pwm6),
	lift2(pwm7),
	compressor(pcm0),
	leftGearbox(pcm0, pch0, pch1),
	rightGearbox(pcm0, pch2, pch3),
	liftGearbox(pcm0, pch4, pch5),
	omniDropper(pcm0, pch6, pch7)
{
	leftSwitch = false;
	leftScale = false;
	controlOverride = false;
	gyroTurning = false;
	compressorEnabled = false;
	pressureStatus = false;
	current = 0.0;
	latestYaw = 0;
	turnSetup = true;

	gl90 = false;
	gr90 = false;
	gl180 = false;
	gr180 = false;

	ahrs = new AHRS(SerialPort::kMXP);

	fpos = centerPos;

	autosteps = {{0, -90, 0.65}, {0, 90, 0.65}};//,
//				 /*Step(this, gyroTurn, {1.0,  90})*/};
	curstep = 0;
	numsteps = 0;
	stepSetup = true;
	stepComplete = false;

	l1.SetExpiration(0.1);
	l2.SetExpiration(0.1);
	r1.SetExpiration(0.1);
	r2.SetExpiration(0.1);
	lift1.SetExpiration(0.1);
	lift2.SetExpiration(0.1);

	timer = new Timer();
	hallSensor = new DigitalInput(dio0);
	testStep = 0;


}

Robot::~Robot() { // Robot destructor - Delete pointer values here

}

void Robot::RobotInit() { // Runs only when robot code starts initially
	// Set Spark inversion
	l1.SetInverted(false);
	l2.SetInverted(false);
	r1.SetInverted(true);
	r2.SetInverted(true);
	leftIO.SetInverted(true);
	rightIO.SetInverted(false);
	lift1.SetInverted(false);
	lift2.SetInverted(false);

	compressorEnabled = compressor.Enabled();
	pressureStatus = compressor.GetPressureSwitchValue();
	current = compressor.GetCompressorCurrent();
	compressor.SetClosedLoopControl(true);
}

void Robot::AutonomousInit() { // Runs at start of autonomous phase, only once
//	CheckSide();
//	CheckPos();
//	if (fpos == leftPos) {
//		LeftAuto();
//	}
//	else if (fpos == centerPos) {
//		CenterAuto();
//	}
//	else if (fpos == rightPos) {
//		RightAuto();
//	}

	numsteps = autosteps.size();
}

void Robot::AutonomousPeriodic() { // Looped through iteratively during autonomous phase - do not put loops here!
//	RunCubeIO(backward);

	RunSteps();
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
	}
	else {
		leftSwitch = false;
	}

	if (gamedata[1] == 'L') {
		leftScale = true;
	}
	else {
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

void Robot::RunSteps() {
	if (curstep < numsteps) {
		std::vector<double> step = autosteps[curstep];
		if (!stepComplete) {
			if (step[0] == 0) { //Gyro Turn
				if (stepSetup) {
					ahrs->ZeroYaw();
					SetLeftSpeed(0.0);
					SetRightSpeed(0.0);
					stepSetup = false;
				}
				else {
					if (step[1] < 0 && ahrs->GetYaw() > step[1]) {
						SetLeftSpeed(-step[2]);
						SetRightSpeed(step[2]);
					}
					else if (step[1] > 0 && ahrs->GetYaw() < step[1]) {
						SetLeftSpeed(step[2]);
						SetRightSpeed(-step[2]);
					}
					else {
						SetLeftSpeed(0.0);
						SetRightSpeed(0.0);
						stepComplete = true;
					}
				}
			}
		}
		else {
			curstep++;
			stepSetup = true;
			stepComplete = false;
		}
	}
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

	Gyro90L(m_left.ReadButton(9));
	Gyro90R(m_left.ReadButton(10));
//	Gyro180L(m_left.ReadButton(11));
//	Gyro180R(m_left.ReadButton(12));
	ManualShiftGears(m_right.ReadButton(6), m_right.ReadButton(4));
	ManualShiftLift(m_left.ReadButton(6), m_left.ReadButton(4));
	ManualCubeIO(m_left.ReadButton(1), m_right.ReadButton(1));
	RunLifter(m_right.ReadButton(5), m_right.ReadButton(3));
//	DropOmnis(m_left.ReadButton(5), m_left.ReadButton(3));
	HoldOmnis(m_right.ReadButton(2));
	CheckHallSensor();


	//Send dashboard values
	SmartDashboard::PutNumber("Gyro Turning Yaw", latestYaw);
	SmartDashboard::PutNumber("Current Yaw", ahrs->GetYaw());
}

/*
 * Teleop Functions:
 * SetLeftSpeed
 * SetRightSpeed
 * TankDrive
 * GyroTurn
 * ManualShiftGears
 * ManualShiftLift
 * ManualCubeIO
 */

void Robot::SetLeftSpeed(double speed) {
	l1.Set(speed);
	l2.Set(speed);
}

void Robot::SetRightSpeed(double speed) {
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

	if(fabs(leftInput) > 0.3 && fabs(leftInput) < 0.7) {
		SetLeftSpeed(leftInput * -inputMultiplier);
	}
	else if (fabs(leftInput) >= 0.7) {
		SetLeftSpeed(-leftInput);
	}
	else {
		SetLeftSpeed(0.0);
	}

	if(fabs(rightInput) > 0.3 && fabs(rightInput) < 0.7) {
		SetRightSpeed(rightInput * -inputMultiplier);
	}
	else if (fabs(rightInput) >= 0.7) {
		SetRightSpeed(-rightInput);
	}
	else {
		SetRightSpeed(0.0);
	}
}

void Robot::Gyro90L(bool btn) {
	if (btn) {
		gl90 = true;
	}

	if (gl90) {
		if (turnSetup) {
			SmartDashboard::PutNumber("Latest Yaw Target", -90);
			ahrs->ZeroYaw();
			turnSetup = false;
		}
		else {
			if (ahrs->GetYaw() > -80) {
				SetLeftSpeed(-1.0);
				SetRightSpeed(1.0);
			}
			else {
				SetLeftSpeed(0.0);
				SetRightSpeed(0.0);
				turnSetup = true;
				gl90 = false;
			}
		}
	}
}

void Robot::Gyro90R(bool btn) {
	if (btn) {
		gr90 = true;
	}

	if (gr90) {
		if (turnSetup) {
			SmartDashboard::PutNumber("Latest Yaw Target", 90);
			ahrs->ZeroYaw();
			turnSetup = false;
		}
		else {
			if (ahrs->GetYaw() < 80) {
				SetLeftSpeed(1.0);
				SetRightSpeed(-1.0);
			}
			else {
				SetLeftSpeed(0.0);
				SetRightSpeed(0.0);
				turnSetup = true;
				gr90 = false;
			}
		}
	}
}

//void Robot::Gyro180L(bool btn) {
//	if (btn) {
//		gl180 = true;
//	}
//
//	if (gl180) {
//		if (turnSetup) {
//			SmartDashboard::PutNumber("Latest Yaw Target", -180);
//			ahrs->ZeroYaw();
//			turnSetup = false;
//		}
//		else {
//			if (ahrs->GetYaw() > -179) {
//				SetLeftSpeed(-1.0);
//				SetRightSpeed(1.0);
//			}
//			else if (ahrs->GetYaw() <= -160 || ahrs->GetYaw() > 130) {
//				SetLeftSpeed(0.0);
//				SetRightSpeed(0.0);
//				turnSetup = true;
//				gl180 = false;
//			}
//		}
//	}
//}
//
//void Robot::Gyro180R(bool btn) {
//	if (btn) {
//		gr180 = true;
//	}
//
//	if (gr180) {
//		if (turnSetup) {
//			SmartDashboard::PutNumber("Latest Yaw Target", 180);
//			ahrs->ZeroYaw();
//			turnSetup = false;
//		}
//		else {
//			if (ahrs->GetYaw() < 179) {
//				SetLeftSpeed(1.0);
//				SetRightSpeed(-1.0);
//			}
//			else if (ahrs->GetYaw() >= 160 || ahrs->GetYaw() < -130) {
//				SetLeftSpeed(0.0);
//				SetRightSpeed(0.0);
//				turnSetup = true;
//				gr180 = false;
//			}
//		}
//	}
//}

void Robot::ManualShiftGears(bool upBtn, bool downBtn) {
	if (upBtn) {
		ShiftGears(up);
	}

	if (downBtn) {
		ShiftGears(down);
	}
}

void Robot::ManualShiftLift(bool upBtn, bool downBtn) {
	if (upBtn) {
		ShiftLift(up);
	}

	if (downBtn) {
		ShiftLift(down);
	}
}

void Robot::ManualCubeIO(bool in, bool out) {
	float inSpeedL = -0.65;
	float inSpeedR = -0.65;
	float outSpeed = 0.75;

	if (in && !out) {
		leftIO.Set(inSpeedL);
		rightIO.Set(inSpeedR);
	}
	else if (!in && out) {
		leftIO.Set(outSpeed);
		rightIO.Set(outSpeed);
	}
	else if (!in && !out) {
		leftIO.Set(0.0);
		rightIO.Set(0.0);
	}
}

void Robot::DropOmnis(bool dropBtn, bool raiseBtn) {
	if (dropBtn && !raiseBtn) {
		omniDropper.Set(DoubleSolenoid::kReverse);
	}

	if (!dropBtn && raiseBtn) {
		omniDropper.Set(DoubleSolenoid::kForward);
	}
}

void Robot::HoldOmnis(bool btn) {
	if (btn) {
		omniDropper.Set(DoubleSolenoid::kReverse);
	}
	else {
		omniDropper.Set(DoubleSolenoid::kForward);
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
 * ShiftGears
 * ShiftLift
 * RunCubeIO
 * RunLifter
 */

void Robot::ShiftGears(Direction dir) {
	/*
	 * Initiate or uninitiate gearboxes to change gear ratio
	 */
	DoubleSolenoid::Value state;
	if (dir == up) {
		state = DoubleSolenoid::kForward;
	}
	else {
		state = DoubleSolenoid::kReverse;
	}

	leftGearbox.Set(state);
	rightGearbox.Set(state);
}

void Robot::ShiftLift(Direction dir) {
	/*
	 * Initiate or uninitiate lifter gearbox
	 */

	DoubleSolenoid::Value state;
	if (dir == up) {
		state = DoubleSolenoid::kForward;
	}
	else {
		state = DoubleSolenoid::kReverse;
	}

	liftGearbox.Set(state);
}

void Robot::RunCubeIO(Direction dir) {
	/*
	 * Set IO motors to run forward or backward
	 */
	if (dir == forward) {
		SmartDashboard::PutString("IO Direction", "Out");
		leftIO.Set(0.65);
		rightIO.Set(0.65);
	}
	else if (dir == backward) {
		SmartDashboard::PutString("IO Direction", "In");
		leftIO.Set(-0.65);
		rightIO.Set(-0.65);
	}
}

void Robot::RunLifter(bool up, bool down) {
	float upSpeed = 0.65;
	float downSpeed  = -0.65;

	if (up && !down) {
		lift1.Set(upSpeed);
		lift2.Set(upSpeed);
	}
	else if (!up && down) {
		lift1.Set(downSpeed);
		lift2.Set(downSpeed);
	}
	else if (!up && !down) {
		lift1.Set(0.0);
		lift2.Set(0.0);
	}
}

void Robot::CheckHallSensor() {
	SmartDashboard::PutBoolean("Sensor Detecting?", hallSensor->Get());
}

//Robot::Step::Step(Robot* r , StepType steptype, std::vector<double> parameters) : robot(r) {
//	params = parameters;
//	type = steptype;
//
//	complete = false;
//	setup = true;
//}
//
//Robot::Step::~Step() {
//	delete robot;
//}
//
//
//void Robot::Step::Run() {
//	/*
//	 * Checks what the step is, then reads in the arguments and executes it
//	 */
//	switch (type) {
//	case reset:
//		/*
//		 * Stop stuff
//		 */
//		break;
//	case encoderMove:
//		if (setup) {
//			/*
//			 * Stop stuff and initialize
//			 */
//			setup = false;
//		}
//		else {
////			double speed, distance;
////			if (params.size() == 2) {
////				speed = params[0];
////				distance = params[1];
////			}
////			else {
////				speed = 0.0;
////				distance = 0.0;
////			}
//		}
//
//		/*
//		 * Move at speed until encoders reach target value
//		 */
//		break;
//	case gyroTurn:
//		if (setup) {
////			robot->ahrs->ZeroYaw();
//			setup = false;
//		}
//		else {
//			double speed, angle;
//			if (params.size() == 2) {
//				speed = params[0];
//				angle = params[1];
//			}
//			else {
//				speed = 0.0;
//				angle = 0.0;
//			}
//
////			if (angle < 0 && robot->ahrs->GetYaw() > angle) {
////				robot->SetLeftSpeed(-speed);
////				robot->SetRightSpeed(speed);
////			}
////			else if (angle > 0 && robot->ahrs->GetYaw() < angle) {
////				robot->SetLeftSpeed(speed);
////				robot->SetRightSpeed(-speed);
////			}
////			else {
////				robot->SetLeftSpeed(0.0);
////				robot->SetRightSpeed(0.0);
////				complete = true;
////			}
//		}
//
//		/*
//		 * Turn at speed until gyro reaches target value
//		 */
//		break;
//	case cubeIO:
//		if (setup) {
//			/*
//			 * Stop stuff
//			 */
//			robot->timer->Reset();
//			setup = false;
//		}
//		else {
//			double leftSpeed, rightSpeed, duration;
//			if (params.size() == 3) {
//				leftSpeed = params[0];
//				rightSpeed = params[1];
//				duration = params[2];
//			}
//			else {
//				leftSpeed = 0.0;
//				rightSpeed = 0.0;
//				duration = 0.0;
//			}
//
//			if (robot->timer->Get() < duration) {
//				robot->leftIO.Set(leftSpeed);
//				robot->rightIO.Set(rightSpeed);
//			}
//			else {
//				robot->leftIO.Set(0.0);
//				robot->rightIO.Set(0.0);
//				complete = true;
//			}
//
//		}
//
//		/*
//		 * Run IO at speed until the duration is over
//		 */
//		break;
//	case lifter:
//		if (setup) {
//			/*
//			 * Stop stuff
//			 */
//			setup = false;
//		}
//		else {
////			double speed, height, direction;
////			if (params.size() == 3) {
////				speed = params[0];
////				height = params[1];
////				direction = params[2];
////			}
////			else {
////				speed = 0.0;
////				height = 0.0;
////				direction = 0;
////			}
//		}
//
//		/*
//		 * Run lifter at speed until it reaches target height
//		 */
//		break;
//	case trackCube:
//		if (setup) {
//			/*
//			 * Stop stuff
//			 */
//			setup = false;
//		}
//		else {
////			double targetX, targetY, targetArea;
////			if (params.size() == 3) {
////				targetX = params[0];
////				targetY = params[1];
////				targetArea = params[2];
////			}
////			else {
////				targetX = 0.0;
////				targetY = 0.0;
////				targetArea = 0.0;
////			}
////
////			double moveSpeed = 0.5;
////			double turnSpeed = 0.5;
//		}
//
//		/*
//		 * Approach cube until it is in the proper range to be picked up
//		 */
//		break;
//	}
//}

START_ROBOT_CLASS(Robot)
