/*
 * MyJoystick.h
 * Created 2018-01-07
 *
 * Last update: 2018-01-07
 */

#include "WPILib.h"

#ifndef SRC_MYJOYSTICK_H_
#define SRC_MYJOYSTICK_H_

#define maxButtons 12

class MyJoystick {
private:
	Joystick* m_Joystick = NULL;			// Pointer to existing Joystick object to read from
	JoystickButton* buttonArray[maxButtons]; // List of buttons on referenced Joystick
	bool buttonValueArray[maxButtons];		 // List of values corresponding to buttons on the same indices of above array
public:

	// Setup
	MyJoystick();
	virtual ~MyJoystick();
	void Init(Joystick* theJoystick);

	// Read values
	void ReadJoystick();
	bool ReadButton(int buttonNumber);
	float CheckLeftStickX();
	float CheckLeftStickY();
	float CheckRightStickX();
	float CheckRightStickY();
	int GetPOV();
};

#endif /* SRC_MYJOYSTICK_H_ */
