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

void DriveWrapper::Set (float speed1, float speed2, uint8_t syncGroup=0)
{
	m_drive1 -> Set(speed1,syncGroup);
	m_drive2 -> Set(speed2,syncGroup);
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


// ---------- Drive Rotate ----------
/*
 * Left is Negative.
 * Angle is given in degree.
 * Initial Angle is angle of the robot when the function runs.
 * Note:
 * 	When you use the functions with initial angle, you cannot run it again and agin in loop because they initialize initial angle again and again.
 */
DriveRotate::DriveRotate (RobotDrive* robotDrive, Encoder* lEncoder, Encoder* rEncoder)
{
	// ----- Get Components -----
	m_robotDrive = robotDrive;
	m_lEncoder = lEncoder;
	m_rEncoder = rEncoder;
	
	// ----- Initialize PID -----
	PID = new PIDController (0.05, 0.0, 0.0, this, this);
}

/*
 * Angle should be given in degree -180 to +180
 * the Angle is difference from initial actual angle.
 */
void DriveRotate::SetAngle (double angle)
{
	// Save Initilized Encoder Values
	if (!f_angleInitialized) {
		iniLEncoder = m_lEncoder->GetDistance();
		iniREncoder = m_rEncoder->GetDistance();
		f_angleInitialized = true;
	}

	// Start PID
	PID->SetSetpoint(angle);
}
/*
 * Enable Set Angle PID
 */
void DriveRotate::PIDEnable()
{
	PID->Enable();
	PIDFlag = true;
	
	// ----- Debug Display -----
	SmartDashboard::PutNumber("Value to PID: ", PIDGet());
	SmartDashboard::PutNumber("Left Encoder Ini: ", iniLEncoder);
	SmartDashboard::PutNumber("Right Encoder Ini: ", iniREncoder);
	SmartDashboard::PutNumber("Rotate Set Point: ", PID->GetSetpoint());
}

/*
 * Disable Set Angle PID
 * You need to call this function before you set next set point otherwise the angle is not initialized.
 */
void DriveRotate::PIDDisable()
{
	if (PID->IsEnabled()) {
		PID->Disable();
		f_angleInitialized = false;
	}
}

/*
 * Determine if the robot got to set angle
 * return bool.
 */
bool DriveRotate::IsRotating (double gap)
{
	double Dleft = iniLEncoder - m_lEncoder->GetDistance();
	double Dright = iniREncoder - m_rEncoder->GetDistance();
	
	return fabs(Dleft - Dright) / DEGREE_FACTOR > gap;
}
bool DriveRotate::IsRotating ()
{
	return IsRotating (ROTATE_ANGLE_GAP);
}

// ----- For PID Use -----
double DriveRotate::PIDGet ()
{
	double Dleft = iniLEncoder - m_lEncoder->GetDistance();
	double Dright = iniREncoder - m_rEncoder->GetDistance();
	return (Dleft - Dright) / DEGREE_FACTOR;
}

void DriveRotate::PIDWrite(float input)
{
	SmartDashboard::PutNumber("Value from PID: ", input);
	m_robotDrive->TankDrive(-input, input);
}
