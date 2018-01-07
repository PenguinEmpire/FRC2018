/*
 * MyJoystick.h
 * Created 2018-01-07
 *
 * Last update: 2018-01-07
 */

#include "WPILib.h"

#ifndef SRC_MYJOYSTICK_H_
#define SRC_MYJOYSTICK_H_

#define MAXBUTTONS 10

class MyJoystick {
private:
	Joystick* m_Joystick = NULL;			// Pointer to existing Joystick object to read from
	JoystickButton* buttonArray[MAXBUTTONS]; // List of buttons on referenced Joystick
	bool buttonValueArray[MAXBUTTONS];		 // List of values corresponding to buttons on the same indices of above array
public:

	// Setup
	MyJoystick();
	virtual ~MyJoystick();
	void init(Joystick* theJoystick);

	// Read values
	void readJoystick();
	bool readButton(int buttonNumber);
	float checkLeftStickX();
	float checkLeftStickY();
	float checkRightStickX();
	float checkRightStickY();
	int getPOV();
};

#endif /* SRC_MYJOYSTICK_H_ */
