/*
 * MyJoystick.cpp
 * Created 2018-01-07
 *
 * Last update: 2018-01-07
 */
#include "WPILib.h"
#include <MyJoystick.h>

#define MAXBUTTONS 10

MyJoystick::MyJoystick() {

}

MyJoystick::~MyJoystick() { // Delete all pointers in button array
	if( m_Joystick != NULL) {
		for (int i = 0; i < MAXBUTTONS; i++) {
			if( buttonArray[i] != NULL ) {
				delete buttonArray[i];
				buttonArray[i] = NULL;
			}
		}
		m_Joystick = NULL;
	}
}

void MyJoystick::Init(Joystick* theJoystick) { // Create list of buttons from referenced Joystick
	m_Joystick = theJoystick;

	for (int i = 0; i < MAXBUTTONS; i++) {
		buttonArray[i] = new JoystickButton(m_Joystick, i + 1);
	}
}

void MyJoystick::ReadJoystick() { // Check all buttons to update values
	for (int i = 0; i < MAXBUTTONS; i++) {
		buttonValueArray[i] = buttonArray[i]->Get();
	}
}

bool MyJoystick::ReadButton(int buttonNumber) { // Retrieve specific button value from value array
	return buttonValueArray[buttonNumber - 1];
}

float MyJoystick::CheckLeftStickX() { // Get Joystick X-axis (left X on gamepad)
	return m_Joystick->GetX();
}

float MyJoystick::CheckLeftStickY() { // Get Joystick Y-axis (left Y on gamepad)
	return m_Joystick->GetY()*-1;
}

float MyJoystick::CheckRightStickX() { // Get Joystick Z-axis (right X on gamepad)
	return m_Joystick->GetZ();
}

float MyJoystick::CheckRightStickY() { // Get Joystick throttle (right Y on gamepad)
	return m_Joystick->GetThrottle();
}

int MyJoystick::GetPOV() { // Get d-pad direction
	return m_Joystick->GetPOV();
}




