/*
 * PenguinEmpire.h
 * Created 2018-01-07
 *
 * Last update: 2018-01-20
 */

#ifndef SRC_PENGUINEMPIRE_H_
#define SRC_PENGUINEMPIRE_H_

#include <string>
#include "WPILib.h"
#include "MyJoystick.h"
#include "AHRS.h"

class Robot : public IterativeRobot {
public:
// Components and Systems
	Joystick leftStick, rightStick, handheld; // Joysticks
	MyJoystick m_left, m_right, m_handheld; // Button reading for Joysticks
	Spark l1, l2, r1, r2; // Drive motor controllers
	Spark leftIO, rightIO; // IO motor controllers
	AHRS *ahrs; // Purple sensor board
	Compressor compressor;
	DoubleSolenoid leftGearbox, rightGearbox;

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

	enum FieldPosition { // Used for autonomous
		leftPos,
		centerPos,
		rightPos
	} fpos;

	enum StepType { // Used for autonomous
		reset,
		encoderMove,
		gyroTurn
	};

	enum Direction {
		up,
		down,
		left,
		right,
		forward,
		backward
	};

	struct Step {
		Robot *robot;
		StepType type;
		bool complete, setup;
		std::vector<double> params;
		Step(Robot *r, StepType steptype, std::vector<double> parameters);
		~Step();
		void Run();
	};

	std::vector<Step> autosteps;
	int numsteps, curstep;

// Stages

	// Setup
	Robot();
	~Robot();
	void RobotInit();

	// Autonomous
	void AutonomousInit();
	void AutonomousPeriodic();
	void CheckSide();
	void CheckPos();
	void LeftAuto();
	void CenterAuto();
	void RightAuto();

	// Teleoperated
	void TeleopInit();
	void TeleopPeriodic();
	void SetLeftSpeed(float speed);
	void SetRightSpeed(float speed);
	void StopMotors();
	void TankDrive();
	void GyroTurn(bool btn, float speed, float angle); // Deprecated
	void GyroTurn(int pov, float speed); // Deprecated
	void GyroLeft(float speed, float angle);
	void ManualShiftGears(bool upBtn, bool downBtn);

	// Test
	void TestInit();
	void TestPeriodic();

	//Other - functions that run in multiple states
	void ShiftGears(Direction dir);
	void RunCubeIO(bool run, Direction dir);

};

#endif /* SRC_PENGUINEMPIRE_H_ */
