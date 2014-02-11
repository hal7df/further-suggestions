#include "JoystickWrapper.h"
#include "WPILib.h"

JoystickWrapper::JoystickWrapper (Joystick* gamepad) {
	m_gamepad = gamepad;
}

JoystickWrapper::JoystickWrapper(int gamepad) {
	m_gamepad = new Joystick (gamepad);
}

float JoystickWrapper::adjust (float input) {
	if (input < -0.2) {
		return input;
	} else if (input < 0.2) {
		return 0.0;
	} else {
		return input;
	}
}

bool JoystickWrapper::GetRawButton (int channel) {
	return m_gamepad->GetRawButton (channel);
}

float JoystickWrapper::GetRawAxis (int channel) {
	return adjust(m_gamepad->GetRawAxis (channel));
}
