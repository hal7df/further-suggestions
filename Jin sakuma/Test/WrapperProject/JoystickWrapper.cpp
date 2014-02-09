#include "JoystickWrapper.h"

JoystickWrapper::JoystickWrapper (Joystick* gamepad) {
	m_gamepad = gamepad;
}

JoystickWrapper::JoystickWrapper (int channel) {
	m_gamepad = new Joystick  (channel);
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
