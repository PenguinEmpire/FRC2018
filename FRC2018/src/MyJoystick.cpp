/*
 * MyJoystick.cpp
 * Created 2018-01-07
 *
 * Last update: 2018-01-07
 * 			Imported existing source to project
 * 			- Noah
 */
#include "WPILib.h"
#include <MyJoystick.h>

#define MAXBUTTONS 10

MyJoystick::MyJoystick() {

}

MyJoystick::~MyJoystick()
{
	if( m_Joystick != NULL)
	{
		for (int i = 0; i < MAXBUTTONS; i++)
		{
			if( buttonArray[i] != NULL )
			{
				delete buttonArray[i];
				buttonArray[i] = NULL;
			}
		}
//		delete m_joystick;
		m_Joystick = NULL;

	}
}

void MyJoystick::init(Joystick* theJoystick)
{
	m_Joystick = theJoystick;

	for (int i = 0; i < MAXBUTTONS; i++)
	{
		buttonArray[i] = new JoystickButton(m_Joystick, i + 1);
	}
}

void MyJoystick::readJoystick()
{
	// get all the button values
	for (int i = 0; i < MAXBUTTONS; i++)
	{
		buttonValueArray[i] = buttonArray[i]->Get();
	}
}

bool MyJoystick::readButton(int buttonNumber){
	// get the correct button value

	return buttonValueArray[buttonNumber - 1];
}

float MyJoystick::checkLeftStickX()
{
	return m_Joystick->GetX();
}

float MyJoystick::checkLeftStickY()
{
	return m_Joystick->GetY()*-1;
}

float MyJoystick::checkRightStickX()
{
	return m_Joystick->GetZ();
}

float MyJoystick::checkRightStickY()
{
	return m_Joystick->GetThrottle();
}

int MyJoystick::getPOV()
{
	return m_Joystick->GetPOV();
}




