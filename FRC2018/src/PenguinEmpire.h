/*
 * PenguinEmpire.h
 * Created 2018-01-07
 *
 * Last update: 2018-10-24
 */

#ifndef SRC_PENGUINEMPIRE_H_
#define SRC_PENGUINEMPIRE_H_

#include <string>
#include "WPILib.h"
#include "MyJoystick.h"
#include "AHRS.h"

class Lidar {
public:
	Lidar();
	unsigned int AquireDistance(/*Timer**/);
private:
	enum Address {ADDRESS_DEFAULT=0x62}; // default I2C bus address for the LIDAR Lite v2
	enum Register {COMMAND=0x00, STATUS=0x01, DISTANCE_1_2=0x8f};
	enum Command {ACQUIRE_DC_CORRECT=0x04};
	enum NumberOfRegistersToRead {READ_1_REGISTER=0x01, READ_2_REGISTERS=0x02};
	enum NumberOfRegistersToWrite {WRITE_1_REGISTER=0x01};
	I2C* I2CBus;

	bool Busy()
	{
		unsigned char Status[Lidar::READ_1_REGISTER];
		unsigned char statusRegister[Lidar::WRITE_1_REGISTER];
		statusRegister[Lidar::WRITE_1_REGISTER-1] = Lidar::STATUS;

		/**********read status**********/
		if ( I2CBus->WriteBulk(statusRegister, Lidar::WRITE_1_REGISTER)) {printf ( "WriteBulk status failed! line %d\n", __LINE__ ); return true;}
		if ( I2CBus->ReadOnly(Lidar::READ_1_REGISTER, Status) ) {printf ( "ReadOnly status failed! line %d\n", __LINE__ ); return true;}
		//printf("Status at line %d %0x, bit0=%0x\n", __LINE__, Status[0], Status[0] & (unsigned char)0x01);
		return (Status[0] & (unsigned char)0x01); // bit 0 is LIDAR Lite v2 busy bit
	};
};

class Robot : public IterativeRobot {
public:
// Components and Systems
	Joystick leftStick, rightStick, handheld; // Joysticks
	MyJoystick m_left, m_right, m_handheld; // Button reading for Joysticks
	Spark l1, l2, r1, r2; // Drive motor controllers
	Spark leftIO, rightIO; // IO motor controllers
	Spark lift1, lift2; // Lifter motor controllers
	AHRS *ahrs; // Purple sensor board
	Compressor compressor;
	DoubleSolenoid driveGearboxes, liftGearbox, omniDropper; // Gearbox shifters and omni actuator
	Timer* mainTimer;
	Timer* lidarTimer;
	Lidar* lidar;
	DigitalInput* bottomSensor;
	DigitalInput* switchSensor;
	DigitalInput* topSensor;
	DigitalInput* centerPosSwitch;
	DigitalInput* rightPosSwitch;
	Encoder leftEnc, rightEnc;

// Values and Structures
	bool leftSwitch; // Is our color on the left side of the switch?
	bool leftScale; // Is our color on the left side of the scale?
	bool controlOverride; // Prevents manual control of drive
	bool gyroTurning; // Checks if performing manual turn-to-angle
	bool compressorEnabled; // Is compressor enabled?
	bool pressureStatus;
	float current;
	int latestYaw;
	bool turnSetup;
	bool comboLift;
	bool comboDrive;
	bool autoDrop;

	bool gl90, gr90, gl180, gr180;

	int testStep;

	enum StepType { // Used for autonomous
		reset,
		encoderMove,
		gyroTurn,
		cubeIO,
		lifter,
		trackCube
	};

	enum Direction {
		up,
		down,
		left,
		right,
		forward,
		backward
	};

//	struct Step {
////		Robot *robot;
//		StepType type;
//		bool complete, setup;
//		std::vector<double> params;
//		Step(Robot *r, StepType steptype, std::vector<double> parameters);
//		~Step();
//		void Run();
//	};

	std::vector<std::vector<double>> autosteps;
	std::vector<std::vector<double>> lll, llr, lrl, lrr, cl, cr, rrr, rrl, rlr, rll;
	int numsteps, curstep;
	bool stepSetup, stepComplete;
	int fpos;
	std::string mode;

	int lastLiftState;
	bool haltLifter;
	bool goingPastSwitch;
	bool checkSwitch;
	bool ioForward, ioBackward;
	bool released;
	bool autoRaise;
	long raiseCounter;
	int sightCounter;

	bool visionAligned;

	std::shared_ptr<NetworkTable> contour;
	std::vector<double> centerX, centerY, area, width;

	int dist;

// Stages

	// Setup
	Robot();
	~Robot();
	void RobotInit();

	// Autonomous
	void AutonomousInit();
	void AutonomousPeriodic();
	void CheckSide();
	int CheckPos();
	void RunSteps();
	void ResetAll();
	void StopMotors();
	void AutoRunLifter(bool up, bool down);

	// Teleoperated
	void TeleopInit();
	void TeleopPeriodic();
	void SetLeftSpeed(double speed);
	void SetRightSpeed(double speed);
	void StopDriveMotors();
	void TankDrive();
	void Gyro90L(bool btn);
	void Gyro90R(bool btn);
//	void Gyro180L(bool btn);
//	void Gyro180R(bool btn);
	void ManualShiftGears(bool upBtn, bool downBtn);
	void ManualShiftLift(bool upBtn, bool downBtn);
	void ManualCubeIO(bool inBtn, bool outBtn);
	void DropOmnis(bool dropBtn, bool raiseBtn);
	void HoldOmnis(bool btn);
	void ToggleSwitchSensor(bool on, bool off);
	void ManualVision(bool btn);
	void QuitVision(bool btn);

	// Test
	void TestInit();
	void TestPeriodic();

	//Other - functions that run in multiple states
	void ShiftGears(Direction dir);
	void ShiftLift(Direction dir);
	void RunCubeIO(Direction dir);
	void RunLifter(bool up, bool down);
	void CheckHallSensor();
	void ToggleIO(bool forward, bool backward);

};

#endif /* SRC_PENGUINEMPIRE_H_ */
