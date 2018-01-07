/*
 * PenguinEmpire.h
 * Created 2018-01-07
 *
 * Last update: 2018-01-07
 * 			Set up basic Robot class with default functions
 * 			- Noah
 */

#ifndef SRC_PENGUINEMPIRE_H_
#define SRC_PENGUINEMPIRE_H_

#include "WPILib.h"

class Robot : public IterativeRobot {
public:
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
