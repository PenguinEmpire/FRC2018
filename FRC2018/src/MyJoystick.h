/*
 * MyJoystick.h
 * Created 2018-01-07
 *
 * Last update: 2018-01-07
 * 			Imported existing header to project
 * 			- Noah
 */

#include "WPILib.h"

#ifndef SRC_MYJOYSTICK_H_
#define SRC_MYJOYSTICK_H_

#define MAXBUTTONS 10

class MyJoystick {
private:
	Joystick* m_Joystick = NULL;
	JoystickButton* buttonArray[MAXBUTTONS];
	bool buttonValueArray[MAXBUTTONS];
public:
	MyJoystick();
	virtual ~MyJoystick();

	void init(Joystick* theJoystick);

	void readJoystick();

	bool readButton(int buttonNumber);

	float checkLeftStickX();

	float checkLeftStickY();

	float checkRightStickX();

	float checkRightStickY();

	int getPOV();
};

#endif /* SRC_MYJOYSTICK_H_ */
