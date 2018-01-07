/*
 * PenguinEmpire.h
 * Created 2018-01-07
 *
 * Last update: 2018-01-07
 */

#ifndef SRC_PENGUINEMPIRE_H_
#define SRC_PENGUINEMPIRE_H_

#include "WPILib.h"

class Robot : public IterativeRobot {
public:
// Components and Systems
	Joystick left, right, handheld;
	Spark l1, l2, r1, r2;

// Stages

	// Setup
	Robot();
	~Robot();
	void RobotInit();

	// Autonomous
	void AutonomousInit();
	void AutonomousPeriodic();

	// Teleoperated
	void TeleopInit();
	void TeleopPeriodic();

	// Test
	void TestInit();
	void TestPeriodic();

};

#endif /* SRC_PENGUINEMPIRE_H_ */
