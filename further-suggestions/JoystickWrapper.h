#include "WPILib.h"

class JoystickWrapper {
private:
	Joystick* m_gamepad;
	float adjust (float);
public:
	JoystickWrapper (Joystick*);
	JoystickWrapper (int);
	bool GetRawButton (int);
	float GetRawAxis (int);
};
