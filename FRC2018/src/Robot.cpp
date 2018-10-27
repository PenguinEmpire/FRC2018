/*
 * Robot.cpp
 * Created 2018-01-07
 *
 * Last Update: 2018-10-24
 */

#include "PenguinEmpire.h"

const int dio0 = 0;
const int dio1 = 1;
const int dio2 = 2;
const int dio3 = 3;
const int dio4 = 4;
const int dio5 = 5;
const int dio6 = 6;
const int dio7 = 7;
const int dio8 = 8;
const int dio9 = 9;

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

const float pulseIn = 0.16;

Lidar::Lidar()
{
	I2CBus = new I2C(I2C::kOnboard, Lidar::ADDRESS_DEFAULT);
	// Wait(1.);
}

unsigned int Lidar::AquireDistance(/*Timer* m_timer*/)
{
	unsigned char distance[Lidar::READ_2_REGISTERS];
	unsigned char distanceRegister_1st[Lidar::WRITE_1_REGISTER];
	distanceRegister_1st[Lidar::WRITE_1_REGISTER - 1] = Lidar::DISTANCE_1_2;

//	printf("Time =  %f starting Lidar::AquireDistance\n", m_timer->Get());

//	do{Wait(.0001);} while (Busy());

//	printf("Time =  %f acquiring distance\n", m_timer->Get());

	/***********acquire distance**********/		//	WriteBulk() also works
	if ( I2CBus->Write(Lidar::COMMAND, Lidar::ACQUIRE_DC_CORRECT) )printf ( "Write operation failed! line %d\n", __LINE__ ); // initiate distance acquisition with DC stabilization

//	do{Wait(.0001);} while (Busy());

//	printf("Time =  %f reading distance\n", m_timer->Get());

	/**********read distance**********/     // Read() does not work
	if ( I2CBus->WriteBulk(distanceRegister_1st, Lidar::WRITE_1_REGISTER)) printf ( "WriteBulk distance failed! line %d\n", __LINE__ );
	else
	if ( I2CBus->ReadOnly(Lidar::READ_2_REGISTERS, distance)) printf ( "ReadOnly distance failed! line %d\n", __LINE__ );

	unsigned int dist = (unsigned int)(distance[0]<<8) + (unsigned int)(distance[1]);

//	printf("Time =  %f, Distance= %d (0x%0x)\n", m_timer->Get(), dist, dist);
	return dist;
}

Robot::Robot() : // Robot constructor - Initialize all subsystem and component classes here
	leftStick(usb0),
	rightStick(usb1),
	handheld(usb2),
	l1(pwm0), //PDP0
	l2(pwm1), //PDP1
	r1(pwm2), //PDP14
	r2(pwm3), //PDP15
	leftIO(pwm4), //PDP13
	rightIO(pwm5), //PDP12
	lift1(pwm6), //PDP2
	lift2(pwm7), //PDP3
	compressor(pcm0),
	driveGearboxes(pcm0, pch0, pch1),
	liftGearbox(pcm0, pch6, pch7),
	omniDropper(pcm0, pch4, pch5),
	leftEnc(dio3, dio2),
	rightEnc(dio1, dio0)

/*
 * things to change
 * Right 5 does left in right out
 * Right 3 does left out right in
 * Omni is left 6 and 4
 * forward right is winch down
 * back right is winch up
 * right 4/6 is shift drive (they are the correct direction)
 * omni leak
 * reverse omni direction
 * right trigger is right back
 * spark 7 and 6, one is flipped polarity
 * left stick drives right
 * left trigger opposite of right trigger
 */
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

	// USE SPI AND NOT SERIALPORT
	ahrs = new AHRS(SPI::kMXP);

//	fpos = centerPos;
	fpos = 1;

	autosteps = {};//,
//				 /*Step(this, gyroTurn, {1.0,  90})*/};
	/*
	 * For testing:
	 * L1 = do a left turn
	 * L2 = do a left turn
	 * L3 = move forward 2 feet
	 * C1 = move forward 2 feet
	 * R1 = do a right turn
	 * R2 = do a right turn
	 * R3 = move back 2 feet
	 */
//	lll = {{1, -90, 0.65}, {1, -90, 0.65}, {2, 24, 0.65}};
//	llr = {{1, -90, 0.65}, {1, -90, 0.65}, {2, -24, 0.65}};
//	lrl = {{1, -90, 0.65}, {1, 90, 0.65}, {2, 24, 0.65}};
//	lrr = {{1, -90, 0.65}, {1, 90, 0.65}, {2, -24, 0.65}};
//	cll = {{2, 24, 0.65}, {1, -90, 0.65}, {2, 24, 0.65}};
//	clr = {{2, 24, 0.65}, {1, -90, 0.65}, {2, -24, 0.65}};
//	crl = {{2, 24, 0.65}, {1, 90, 0.65}, {2, 24, 0.65}};
//	crr = {{2, 24, 0.65}, {1, 90, 0.65}, {2, -24, 0.65}};
//	rll = {{1, 90, 0.65}, {1, -90, 0.65}, {2, 24, 0.65}};
//	rlr = {{1, 90, 0.65}, {1, -90, 0.65}, {2, -24, 0.65}};
//	rrl = {{1, 90, 0.65}, {1, 90, 0.65}, {2, 24, 0.65}};
//	rrr = {{1, 90, 0.65}, {1, 90, 0.65}, {2, -24, 0.65}};

	/*
		 * 1: Gyro Turn
		 * 2: Encoder Move
		 * 3: Lifter
		 * 4: LIDAR Approach
		 * 5: IO Timed
		 * 6: IO Set
		 * 7: Auto Aim
		 * 8: Shift drive gears down
		 * 9: Combination lift & move		{9, stage, dist, speed}
		 * 10: Set omni wheels				{10, set}
		 * 11: Shift lift
		 *
		 * 97: IO Hold						{97, max dist, speed (eject from IO speed)}		UNUSED
		 * 98: Loop all steps																UNUSED
		 * 99: Go to specified step (changes curstep) {99, wanted step number as index}		UNUSED
		 */

	rrr = {{8},
		   {9, 1, 260, 1.0},
		   {10, 1},
		   {1, -40, 0.5},
		   {10, 0},
		   {3, 2},
		   {2, 24, 0.5},
		   {5, 0.5, 1.0},
		   {2, -10, 0.5},
		   {11, 1},
		   {9, 0, -14, 0.5},
		   {11, 0},
		   {10, 1},
		   {1, -75, 0.75},
		   {10, 0},
		   {2, 24, 0.65},
		   // {7, 256, 384, 0.5},
		   {7, 150, 225, .5},
		   {6, -1.0},
		   {4, 20, 0.65},
		   {6, 0.0},
		   {3, 1},
		   {2, 2, 0.5},
		   {5, 0.5, 1.0},
		   {2, -5, 0.5},
		   {3, 0}
	};

	rrl = {{8},
		  {9, 1, 125, 1.0},
//		  {3, 1},
//		  {2, 75, 1.0},
		  {10, 1},
		  {1, -60, 0.75},
		  {10, 0},
		  {2, 26, 0.75},
		  {5, 0.5, 1.0},
		  {9, 0, -26, 0.75},
//		  {2, -5, 0.75},
//		  {3, 0},
		  {10, 1},
		  {1, 60, 0.65},
		  {10, 0},
		  {2, 85, 0.90},
		  {10, 1},
		  {1, -60, 0.65},
		  {10, 0},
		  {2, 54, 0.65},
		  {10, 1},
		  {1, -75, 0.65},
		  {10, 0},
		  //{7, 256, 384, 0.5},
		  {7, 150, 225, 0.5},
		  {6, -1.0},
		  {4, 20, 0.65},
		  {6, 0.0},
		  {3, 1},
		  {2, 2, 0.5},
		  {5, 0.5, 1.0},
		  {2, -5, 0.5},
		  {3, 0}
	};

	rlr = {{8},
		   {9, 1, 260, 1.0},
		   {10, 1},
		   {1, -30, 0.5},
		   {10, 0},
		   {3, 2},
		   {2, 24, 0.5},
		   {5, 0.5, 1.0},
		   {2, -10, 0.5},
		   {11, 1},
		   {9, 0, -14, 0.5},
		   {11, 0},
		   {10, 1},
		   {1, -150, 0.75},
		   {10, 0},
		   {7, 150, 225, 0.5} // changed!
	};

	rll = {{8},
		  {9, 1, 204, 1.0},
//		  {3, 1},
//		  {2, 205, 1.0},
		  {10, 1},
		  {1, -60, 0.65},
		  {10, 0}
	};

	cl = {{8},
		  {2, 27, 1.0},
//		  {3, 1},
//		  {2, 48, 0.65},
		  {10, 1},
		  {1, -30, 0.5},
		  {10, 0},
		  {9, 1, 63, 0.65},
		  {10, 1},
		  {1, 30, 0.5},
		  {10, 0},
//		  {2, 3, 0.65},
		  {2, 36, 0.65},
		  {5, 0.5, 1.0},
		  {2, -5, 0.5},
		  {10, 1},
		  {1, -60, 0.75},
		  {10, 0},
		  {3, 0}
	};

	cr = {{8},
		  {2, 30, 0.65},
//		  {3, 1},
//		  {2, 48, 0.65},
		  {10, 1},
		  {1, 30, 0.5},
		  {10, 0},
		  {9, 1, 60, 0.65},
		  {10, 1},
		  {1, -30, 0.5},
		  {10, 0},
//		  {2, 3, 0.65},
		  {2, 36, 0.65},
		  {5, 0.5, 1.0},
		  {2, -5, 0.5},
		  {10, 1},
		  {1, 60, 0.75},
		  {10, 0},
		  {3, 0}
	};
	lll = {{8},
		   {9, 1, 260, 1.0},
		   {10, 1},
		   {1, 40, 0.5},
		   {10, 0},
		   {2, 24, 0.5},
		   {5, 0.5, 1.0},
		   {2, -10, 0.5},
		   {11, 1},
		   {9, 0, -14, 0.5},
		   {11, 0},
		   {10, 1},
		   {1, 75, 0.75},
		   {10, 0},
		   {7, 150, 225, 0.5}, //changed!
		   {6, -1.0},
		   {4, 20, 0.65},
		   {6, 0.0},
		   {3, 1},
		   {2, 2, 0.5},
		   {5, 0.5, 1.0},
		   {2, -5, 0.5},
		   {3, 0}

//	  {8},
//	  {9, 1, 125, 1.0},
////		  {3, 1},
////		  {2, 75, 1.0},
//	  {10, 1},
//	  {1, 60, 0.75},
//	  {10, 0},
//	  {2, 26, 0.75},
//	  {5, 0.5, 1.0},
//	  {9, 0, -26, 0.75},
////		  {2, -5, 0.75},
////		  {3, 0},
//	  {10, 1},
//	  {1, -60, 0.65},
//	  {10, 0},
//	  {2, 85, 0.90},
//	  {10, 1},
//	  {1, 60, 0.65},
//	  {10, 0},
//	  {2, 54, 0.65},
//	  {10, 1},
//	  {1, 75, 0.65},
//	  {10, 0},
//	  {7, 150, 225, 0.5}, //changed!
//	  {6, -1.0},
//	  {4, 20, 0.65},
//	  {6, 0.0},
//	  {3, 1},
//	  {2, 2, 0.5},
//	  {5, 0.5, 1.0},
//	  {2, -5, 0.5},
//	  {3, 0}
	};

	llr = {{8},
		  {9, 1, 125, 1.0},
//		  {3, 1},
//		  {2, 75, 1.0},
		  {10, 1},
		  {1, 60, 0.75},
		  {10, 0},
		  {2, 26, 0.75},
		  {5, 0.5, 1.0},
		  {9, 0, -26, 0.75},
//		  {2, -5, 0.75},
//		  {3, 0},
		  {10, 1},
		  {1, -60, 0.65},
		  {10, 0},
		  {2, 85, 0.90},
		  {10, 1},
		  {1, 60, 0.65},
		  {10, 0},
		  {2, 54, 0.65},
		  {10, 1},
		  {1, 75, 0.65},
		  {10, 0},
		  {7, 150, 225, 0.5}, // changed!
		  {6, -1.0},
		  {4, 20, 0.65},
		  {6, 0.0},
		  {3, 1},
		  {2, 2, 0.5},
		  {5, 0.5, 1.0},
		  {2, -5, 0.5},
		  {3, 0}
	};

	lrl = {{8},
		   {9, 1, 260, 1.0},
		   {10, 1},
		   {1, 30, 0.5},
		   {10, 0},
		   {3, 2},
		   {2, 24, 0.5},
		   {5, 0.5, 1.0},
		   {2, -10, 0.5},
		   {11, 1},
		   {9, 0, -14, 0.5},
		   {11, 0},
		   {10, 1},
		   {1, 150, 0.75},
		   {10, 0},
		   {7, 150, 225, 0.5} //chagned!
	};

	lrr = {{8},
		  {9, 1, 204, 1.0},
//		  {3, 1},
//		  {2, 205, 1.0},
		  {10, 1},
		  {1, 60, 0.65},
		  {10, 0}
	};


	mode = "lll";

	curstep = 0;
	numsteps = 0;
	stepSetup = true;
	stepComplete = false;
	comboLift = false;
	comboDrive = false;
	autoDrop = false;

	l1.SetExpiration(0.1);
	l2.SetExpiration(0.1);
	r1.SetExpiration(0.1);
	r2.SetExpiration(0.1);
	lift1.SetExpiration(0.1);
	lift2.SetExpiration(0.1);

	mainTimer = new Timer();
	bottomSensor = new DigitalInput(dio7);
	switchSensor = new DigitalInput(dio8);
	topSensor = new DigitalInput(dio9);
	centerPosSwitch = new DigitalInput(dio4);
	rightPosSwitch = new DigitalInput(dio5);
	testStep = 0;

	leftEnc.SetDistancePerPulse(pulseIn);
	rightEnc.SetDistancePerPulse(pulseIn);

	lastLiftState = 0;
	haltLifter = false;
	goingPastSwitch = false;
	checkSwitch = true;
	ioForward = false;
	ioBackward = false;
	released = true;
	autoRaise = false;
	raiseCounter = 0;
	sightCounter = 0;

	visionAligned = false;

	lidarTimer = new Timer();
	lidar = new Lidar;
	dist = 0;

	contour = NetworkTable::GetTable("GRIP/myContoursReport");
	centerX = {0};
	centerY = {0};
	area = {0};
	width = {0};

	contour->PutNumber("Quit", 0);
}

Robot::~Robot() { // Robot destructor - Delete pointer values here

}

void Robot::RobotInit() { // Runs only when robot code starts initially
	// Initialize camera server
//	CameraServer::GetInstance()->StartAutomaticCapture("cam0", 0);

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

	mainTimer->Start();
	lidarTimer->Start();
	mainTimer->Reset();
	lidarTimer->Reset();
}

void Robot::AutonomousInit() { // Runs at start of autonomous phase, only once
	ahrs->ZeroYaw();
	leftEnc.Reset();
	rightEnc.Reset();
	mainTimer->Reset();
	curstep = 0;

	CheckSide();
	fpos = CheckPos();
	if (fpos == 0 && leftSwitch && leftScale) {
		autosteps = lll;
		mode = "lll";
	}
	else if (fpos == 0 && !leftSwitch && leftScale) {
		autosteps = lrl;
		mode = "lrl";
	}
	else if (fpos == 0 && leftSwitch && !leftScale) {
		autosteps = llr;
		mode = "llr";
	}
	else if (fpos == 0 && !leftSwitch && !leftScale) {
		autosteps = lrr;
		mode = "lrr";
	}
	else if (fpos == 1 && leftSwitch) {
		autosteps = cl;
		mode = "cl";
	}
	else if (fpos == 1 && !leftSwitch) {
		autosteps = cr;
		mode = "cr";
	}
	else if (fpos == 2 && leftSwitch && leftScale) {
		autosteps = rll;
		mode = "rll";
	}
	else if (fpos == 2 && !leftSwitch && leftScale) {
		autosteps = rrl;
		mode = "rrl";
	}
	else if (fpos == 2 && leftSwitch && !leftScale) {
		autosteps = rlr;
		mode = "rlr";
	}
	else if (fpos == 2 && !leftSwitch && !leftScale) {
		autosteps = rrr;
		mode = "rrr";
	}

	//Auto Aim Params: minX = 256, maxX = 384
	/*
	 * 1: Gyro Turn
	 * 2: Encoder Move
	 * 3: Lifter
	 * 4: LIDAR Approach
	 * 5: IO Timed
	 * 6: IO Set
	 * 7: Auto Aim
	 * 99: Loop all previous steps
	 */
//	autosteps = {{7, 256, 384, 0.4}, {4, 200, 0.65}, {7, 256, 384, 0.4}, {4, 100, 0.65}, {6, -1.0}, {4, 30, 0.65}, {6, -0.5}, {4, 13, 0.65}, {6, 0}, {97, 18, -0.5}};
//	autosteps = {{3, 1},
//				 {2, 24, 0.65},
//				 {5, 1, 1.0},
//				 {3, 0},
//				 {1, -90, 0.55},
//				 {7, 256, 384, 0.4},
//				 {4, 300, 0.65},
//				 {7, 256, 384, 0.4},
//				 {4, 200, 0.65},
//				 {7, 256, 384, 0.4},
//				 {4, 100, 0.65},
//				 {6, -1.0},
//				 {4, 30, 0.65},
//				 {6, -0.5},
//				 {4, 13, 0.65},
//				 {6, 0},
//				 {3, 1},
//				 {2, 48, 0.65},
//				 {3, 2},
//				 {5, 1, 1.0},
//				 {3, 0}};

	numsteps = autosteps.size();
	ShiftLift(down);
	checkSwitch = true;
}

void Robot::AutonomousPeriodic() { // Looped through iteratively during autonomous phase - do not put loops here!
//	RunCubeIO(backward);
	SmartDashboard::PutString("Auto Mode", mode);
	SmartDashboard::PutNumber("Timer Value", mainTimer->Get());
	SmartDashboard::PutNumberArray("Center X", centerX);
	SmartDashboard::PutNumberArray("Center Y", centerY);
	SmartDashboard::PutNumberArray("Area", area);
	SmartDashboard::PutNumberArray("Width", width);
	SmartDashboard::PutNumber("Left Encoder", leftEnc.GetDistance());
	SmartDashboard::PutNumber("Right Encoder", rightEnc.GetDistance());
	SmartDashboard::PutNumber("Average Distance", (rightEnc.GetDistance() - leftEnc.GetDistance()) / 2);
	SmartDashboard::PutNumber("Yaw", ahrs->GetYaw());
	centerX = contour->GetNumberArray("centerX", llvm::ArrayRef<double>());
	centerY = contour->GetNumberArray("centerY", llvm::ArrayRef<double>());
	area = contour->GetNumberArray("area", llvm::ArrayRef<double>());
	width = contour->GetNumberArray("width", llvm::ArrayRef<double>());

	dist = lidar->AquireDistance();
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

	if (gamedata [1] == 'L') {
		leftScale = true;
	}
	else {
		leftScale = false;
	}
}

int Robot::CheckPos() {
	/*
	 * Check physical switch
	 */

	if (centerPosSwitch->Get()) {
		return 1;
	}
	else {
		if (rightPosSwitch->Get()) {
			return 2;
		}
		else {
			return 0;
		}
	}

	// Comment out above conditionals and uncomment/change below value to spoof position on practice bot
//	return 1;
}

void Robot::RunSteps() {
	/*
	 * Runs through the selected autonomous mode step by step
	 */

	if (curstep < numsteps) {
		std::vector<double> step = autosteps[curstep];
		SmartDashboard::PutNumber("Current Step", curstep);
		SmartDashboard::PutNumber("Current Step Type", step[0]);
		if (!stepComplete) {
			if (step[0] == 0) { // Test Initial Lifter {0}
				if (stepSetup) {
					mainTimer->Reset();
					stepSetup = false;
				}
				else {
					if (mainTimer->Get() < 1) {
						lift1.Set(1.0);
						lift2.Set(1.0);
					}
					else if (mainTimer->Get() >= 1 && mainTimer->Get() < 1.5) {
						lift1.Set(-1.0);
						lift2.Set(-1.0);
					}
					else {
						lift1.Set(0.0);
						lift2.Set(0.0);
						stepComplete = true;
					}
				}
			}
			else if (step[0] == 1) { //Gyro Turn {1, angle, speed}
				if (stepSetup) {
					ResetAll();
					StopMotors();
					if (ahrs->GetYaw() > -5 && ahrs->GetYaw() < 5) {
						stepSetup = false;
					}
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
						ResetAll();
						StopMotors();
						stepComplete = true;
					}
				}
			}
			else if (step[0] == 2) { // Encoder Move {2, distance, speed}
				if (stepSetup) {
					ResetAll();
					StopMotors();
					stepSetup = false;
				}
				else {
					double avgEnc = (rightEnc.GetDistance() - leftEnc.GetDistance()) / 2;
					if (step[1] > 0 && avgEnc < step[1]) {
						SetLeftSpeed(step[2]);
						SetRightSpeed(step[2]);
					}
					else if (step[1] < 0 && avgEnc > step[1]) {
						SetLeftSpeed(-step[2]);
						SetRightSpeed(-step[2]);
					}
					else {
						ResetAll();
						StopMotors();
						stepComplete = true;
					}
				}
			}
			else if (step[0] == 3) { // Run Lifter {3, stage (0-2)}
 				if (stepSetup) {
					StopMotors();
					stepSetup = false;
				}
				else {
					int stage = step[1];
					switch (stage) {
					case 0:
						AutoRunLifter(false, true);
						if (!bottomSensor->Get()) {
							StopMotors();
							stepComplete = true;
						}
						break;
					case 1:
						if (!bottomSensor->Get()) {
							AutoRunLifter(true, false);
						}
						else if (!topSensor->Get()) {
							AutoRunLifter(false, true);
						}

						if (!switchSensor->Get()) {
							StopMotors();
							stepComplete = true;
						}
						break;
					case 2:
						AutoRunLifter(true, false);
						if (!topSensor->Get()) {
							StopMotors();
							stepComplete = true;
						}
					}
				}
			}
			else if (step[0] == 4) { //LIDAR Approach Object {4, end dist, speed}
				if (stepSetup) {
					ResetAll();
					StopMotors();
					stepSetup = false;
				}
				else {
					if (dist > step[1]) {
						SetLeftSpeed(step[2]);
						SetRightSpeed(step[2]);
					}
					else {
						ResetAll();
						StopMotors();
						stepComplete = true;
					}
				}
			}
			else if (step[0] == 5) { //Timed IO {5, time, speed}
				if (stepSetup) {
					ResetAll();
					StopMotors();
					mainTimer->Reset();
					mainTimer->Start();
					stepSetup = false;
				}
				else {
					if (mainTimer->Get() < step[1]) {
						leftIO.Set(step[2]);
						rightIO.Set(step[2]);
					}
					else {
						ResetAll();
						StopMotors();
						leftIO.Set(0.0);
						rightIO.Set(0.0);
						stepComplete = true;
					}
				}
			}
			else if (step[0] == 6) { // IO Set {6, speed}
				if (stepSetup) {
					ResetAll();
					StopMotors();
					stepSetup = false;
				}
				else {
					leftIO.Set(step[1]);
					rightIO.Set(step[1]);
					stepComplete = true;
				}
			}
			else if (step[0] == 7) { // Auto Aim {7, minX, maxX, speed}
				if (stepSetup) {
					ResetAll();
					StopMotors();
					stepSetup = false;
				}
				else {
					if (centerX.size() > 0) {
						if (centerX[0] < step[1]) {
							SetLeftSpeed(-step[3]);
							SetRightSpeed(step[3]);
						}
						else if (centerX[0] > step[2]) {
							SetLeftSpeed(step[3]);
							SetRightSpeed(-step[3]);
						}
						else {
							ResetAll();
							StopMotors();
							stepComplete = true;
						}
					}
				}
			}
			else if (step[0] == 8) { // Shift drive gears down {8}
				if (stepSetup) {
					ResetAll();
					StopMotors();
					stepSetup = false;
				}
				else {
					ShiftGears(down);
					stepComplete = true;
				}
			}
			else if (step[0] == 9) { // Combination lift and move {9, stage, dist, speed}
				if (stepSetup) {
					ResetAll();
					StopMotors();
					lift1.Set(0.0);
					lift2.Set(0.0);
					stepSetup = false;
				}
				else {
					if (!comboLift) {
						int stage = step[1];
						switch (stage) {
						case 0:
							AutoRunLifter(false, true);
							if (!bottomSensor->Get()) {
								lift1.Set(0.0);
								lift2.Set(0.0);
								comboLift = true;
							}
							break;
						case 1:
							if (!bottomSensor->Get()) {
								AutoRunLifter(true, false);
							}
							else if (!topSensor->Get()) {
								AutoRunLifter(false, true);
							}

							if (!switchSensor->Get()) {
								lift1.Set(0.0);
								lift2.Set(0.0);
								comboLift = true;
							}
							break;
						case 2:
							AutoRunLifter(true, false);
							if (!topSensor->Get()) {
								lift1.Set(0.0);
								lift2.Set(0.0);
								comboLift = true;
							}
						}
					}

					if (!comboDrive) {
						double avgEnc = (rightEnc.GetDistance() - leftEnc.GetDistance()) / 2;
						if (step[2] > 0 && avgEnc < step[2]) {
							SetLeftSpeed(step[3]);
							SetRightSpeed(step[3]);
						}
						else if (step[2] < 0 && avgEnc > step[2]) {
							SetLeftSpeed(-step[3]);
							SetRightSpeed(-step[3]);
						}
						else {
							SetLeftSpeed(0.0);
							SetRightSpeed(0.0);
							ResetAll();
							comboDrive = true;
						}
					}

					if (comboLift && comboDrive) {
						StopMotors();
						ResetAll();
						stepComplete = true;
					}
				}
			}
			else if (step[0] == 10) { // Set omni wheels {10, set}
				if (step[1] == 1) {
					omniDropper.Set(DoubleSolenoid::kReverse);
				}
				else {
					omniDropper.Set(DoubleSolenoid::kForward);
				}
				stepComplete = true;
			}
			else if (step[0] == 11) {
				if (step[1] == 1) {
					ShiftLift(up);
				}
				else {
					ShiftLift(down);
				}
				stepComplete = true;
			}
			else if (step[0] == 97) { // IO Hold {97, max dist, speed}
				if (stepSetup) {
					ResetAll();
					StopMotors();
					leftIO.Set(0.0);
					rightIO.Set(0.0);
					stepSetup = false;
				}
				else {
					if (dist > step[1]) {
						leftIO.Set(step[2]);
						rightIO.Set(step[2]);
					}
					else {
						leftIO.Set(0.0);
						rightIO.Set(0.0);
					}
				}
			}
			else if (step[0] == 98) { // Loop all steps {98}
				if (stepSetup) {
					ResetAll();
					StopMotors();
				}
				else {
					curstep = -1;
					stepComplete = true;
				}
			}
			else if (step[0] == 99) { // Go to step {99, step number as index}
				if (stepSetup) {
					ResetAll();
					StopMotors();
				}
				else {
					curstep = step[1]; //step[1] - 1; | CHANGED - Noah lied
					stepComplete = true;
				}
			}
		}
		else {
			curstep++;
			comboLift = false;
			comboDrive = false;
			stepSetup = true;
			stepComplete = false;
		}
	}
}


void Robot::ResetAll() {
	ahrs->ZeroYaw();
	leftEnc.Reset();
	rightEnc.Reset();
	stepSetup = false;
}

void Robot::StopMotors() {
	SetLeftSpeed(0.0);
	SetRightSpeed(0.0);
	lift1.Set(0.0);
	lift2.Set(0.0);
}

void Robot::AutoRunLifter(bool up, bool down) {
	if (up && !down) {
		lift1.Set(1.0);
		lift2.Set(1.0);
	}
	else if (!up && down) {
		lift1.Set(-1.0);
		lift2.Set(-1.0);
	}
	else if (!up && !down) {
		lift1.Set(0.0);
		lift2.Set(0.0);
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
	dist = lidar->AquireDistance(/*lidarTimer*/);
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
	ManualShiftLift(m_left.ReadButton(6) || m_handheld.ReadButton(4), m_left.ReadButton(4) || m_handheld.ReadButton(2));
	ManualCubeIO(m_handheld.ReadButton(8), m_handheld.ReadButton(6));
	RunLifter(m_handheld.ReadButton(5) || m_right.ReadButton(1), m_handheld.ReadButton(7) || m_left.ReadButton(1));
//	DropOmnis(m_left.ReadButton(5), m_left.ReadButton(3));
	HoldOmnis(m_right.ReadButton(2));
	ToggleSwitchSensor(m_handheld.ReadButton(1), m_handheld.ReadButton(3));
	CheckHallSensor();
//	ToggleIO(m_handheld.ReadButton(9), m_handheld.ReadButton(10));
	ManualVision(m_left.ReadButton(2) || m_handheld.ReadButton(9));
	QuitVision(m_handheld.ReadButton(10));

	centerX = contour->GetNumberArray("centerX", llvm::ArrayRef<double>());


	//Send dashboard values
	SmartDashboard::PutNumber("Gyro Turning Yaw", latestYaw);
	SmartDashboard::PutNumber("Current Yaw", ahrs->GetYaw());
	SmartDashboard::PutNumber("Encoder L", leftEnc.GetDistance());
	SmartDashboard::PutNumber("Encoder R", rightEnc.GetDistance());
	SmartDashboard::PutNumber("Average Distance", (rightEnc.GetDistance() - leftEnc.GetDistance()) / 2);
	SmartDashboard::PutBoolean("Top Sensor", topSensor->Get());
	SmartDashboard::PutBoolean("Switch Sensor", switchSensor->Get());
	SmartDashboard::PutBoolean("Bottom Sensor", bottomSensor->Get());
	SmartDashboard::PutNumber("LIDAR", dist);
	SmartDashboard::PutBoolean("centerPos", centerPosSwitch->Get());
	SmartDashboard::PutBoolean("rightPos", rightPosSwitch->Get());
	SmartDashboard::PutNumberArray("Center X", centerX);
	SmartDashboard::PutBoolean("Auto Raise?", autoRaise);
	SmartDashboard::PutNumber("Raise Counter", raiseCounter);
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

void Robot::StopDriveMotors() {
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
		SetLeftSpeed(-leftInput * 0.9);
	}
	else {
		SetLeftSpeed(0.0);
	}

	if(fabs(rightInput) > 0.3 && fabs(rightInput) < 0.7) {
		SetRightSpeed(rightInput * -inputMultiplier);
	}
	else if (fabs(rightInput) >= 0.7) {
		SetRightSpeed(-rightInput * 0.9);
	}
	else {
		SetRightSpeed(0.0);
	}

//	if (fabs(rightInput - leftInput) > 0.5 && fabs(rightInput) > 0.5 && fabs(leftInput) > 0.5) {
//		omniDropper.Set(DoubleSolenoid::kReverse);
//		autoDrop = true;
//	}
//	else {
//		autoDrop = false;
//	}
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
	float inSpeedL = -1.0;
	float inSpeedR = -1.0;
	float outSpeed = 1.0;

	if (in && !out) {
		leftIO.Set(inSpeedL);
		rightIO.Set(inSpeedR);
	}
	else if (!in && out) {
		leftIO.Set(outSpeed);
		rightIO.Set(outSpeed);
	}
	else if (!in && !out && !ioForward && !ioBackward) {
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
	else if (!autoDrop){
		omniDropper.Set(DoubleSolenoid::kForward);
	}
}

void Robot::ToggleSwitchSensor(bool on, bool off) {
	if (on) {
		checkSwitch = true;
	}

	if (off) {
		checkSwitch = false;
	}
}

void Robot::ManualVision(bool btn) {
	//Resolution of Camera is (672, 376) in VGA mode
	//Genius Noah split into fifths. 2/5 left 1/5 aligned 2/5 right
	if (btn) {
		if (centerX.size() > 0) {
			if (centerX[0] < 150) {
				SetLeftSpeed(-0.5);
				SetRightSpeed(0.5);
				visionAligned = false;
			}
			else if (centerX[0] > 225) {
				SetLeftSpeed(0.5);
				SetRightSpeed(-0.5);
				visionAligned = false;
			}
			else {
				SetLeftSpeed(0.0);
				SetRightSpeed(0.0);
				visionAligned = true;
			}
		}

		if (visionAligned) {
			if (lidar->AquireDistance() > 25) {
				leftIO.Set(-1.0);
				rightIO.Set(-1.0);
				SetLeftSpeed(0.65);
				SetRightSpeed(0.65);
			}
			else {
				leftIO.Set(0.0);
				rightIO.Set(0.0);
				SetLeftSpeed(0.0);
				SetRightSpeed(0.0);
			}
		}
		contour->PutNumber("Quit", 0);
	}
}

void Robot::QuitVision(bool btn) {
	if (btn) {
		contour->PutNumber("Quit", 1);
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

	driveGearboxes.Set(state);
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

	if (up) {
		lastLiftState = 1;
	}
	else if (down) {
		lastLiftState = 2;
	}
	else {
		lastLiftState = 0;
	}

	if (!bottomSensor->Get() || !autoRaise) {
		raiseCounter = 0;
	}

	if (lidar->AquireDistance() < 15 && !bottomSensor->Get()) {
		sightCounter++;
		if (sightCounter >= 3) {
			autoRaise = true;
		}
	}
	else {
		sightCounter = 0;
	}

	if (autoRaise) {
		lift1.Set(0.75);
		lift2.Set(0.75);
	}
	else {
		if (!bottomSensor->Get()) {
			if (down) {
				lift1.Set(0.0);
				lift2.Set(0.0);
			}
			else if (up) {
				lift1.Set(upSpeed);
				lift2.Set(upSpeed);
			}
		}
		else if (!switchSensor->Get() && checkSwitch) {
			if ((up || down) && !goingPastSwitch) {
				haltLifter = true;
				lift1.Set(0.0);
				lift2.Set(0.0);
			}

			if (haltLifter && !up && !down) {
				haltLifter = false;
				goingPastSwitch = true;
			}

			if (!haltLifter) {
				if (up) {
					lift1.Set(upSpeed);
					lift2.Set(upSpeed);
				}
				else if (down) {
					lift1.Set(downSpeed);
					lift2.Set(downSpeed);
				}
			}
		}
		else if (!topSensor->Get()) {
			if (up) {
				lift1.Set(0.0);
				lift2.Set(0.0);
			}
			else if (down) {
				lift1.Set(downSpeed);
				lift2.Set(downSpeed);
			}
		}
		else {
			if (up) {
				lift1.Set(upSpeed);
				lift2.Set(upSpeed);
			}

			if (down) {
				lift1.Set(downSpeed);
				lift2.Set(downSpeed);
			}
		}
	}

	if ((!switchSensor->Get() || !topSensor->Get() || raiseCounter >= 15 || up || down) && autoRaise) {
		autoRaise = false;
		raiseCounter = 0;
	}

	if (switchSensor->Get()) {
		goingPastSwitch = false;
	}

	if (!up && !down && !autoRaise) {
		lift1.Set(0.0);
		lift2.Set(0.0);
	}

//	if (topSensor->Get()) {
//		if (up && !down) {
//			lift1.Set(upSpeed);
//			lift2.Set(upSpeed);
//		}
//		else if (!up && down) {
//			lift1.Set(downSpeed);
//			lift2.Set(downSpeed);
//		}
//		else if (!up && !down) {
//			lift1.Set(0.0);
//			lift2.Set(0.0);
//		}
//	}
//	else {
//		lift1.Set(0.0);
//		lift2.Set(0.0);
//	}

	raiseCounter++;
}

void Robot::CheckHallSensor() {
//	SmartDashboard::PutBoolean("Sensor Detecting?", hallSensor->Get());
}

void Robot::ToggleIO(bool forward, bool backward) {
	if ((forward || backward) && released) {
		if (forward && ioForward) {
			ioForward = false;
		}
		else if (forward && !ioForward){
			ioForward = true;
		}

		if (backward && ioBackward) {
			ioBackward = false;
		}
		else if (backward && !ioBackward) {
			ioBackward = true;
		}
		released = false;
	}
	else if (!forward && !backward){
		released = true;
	}

	if (ioForward) {
		leftIO.Set(1.0);
		rightIO.Set(1.0);
	}
	else if (ioBackward) {
		leftIO.Set(1.0);
		rightIO.Set(1.0);
	}
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
