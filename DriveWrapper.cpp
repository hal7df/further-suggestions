#include "DriveWrapper.h"

DriveWrapper::DriveWrapper (SpeedController* m_motor1, SpeedController* m_motor2)
{
	m_drive1 = m_motor1;
	m_drive2 = m_motor2;
}

void DriveWrapper::Set (float speed, uint8_t syncGroup=0)
{
	m_drive1 -> Set(speed,syncGroup);
	m_drive2 -> Set(speed,syncGroup);
}

float DriveWrapper::Get ()
{
	return m_drive1->Get();
}

void DriveWrapper::Disable ()
{
	m_drive1->Disable();
	m_drive2->Disable();
}

void DriveWrapper::PIDWrite (float output)
{
	Set(output);
}
