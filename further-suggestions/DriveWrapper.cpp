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

// Drive Auton ***************************************

DriveAuton::DriveAuton (RobotDrive* robotDrive, Encoder* lEncoder, Encoder* rEncoder)
{
	// ----- Initialize drive source objects -----
	m_dStraight = new DriveStraightSource (this);
	m_dRotate = new DriveRotateSource (this);
	
	// ----- Set Components ------
	m_robotDrive = robotDrive;
	m_lEncoder = lEncoder;
	m_rEncoder = rEncoder;
	
	// ----- Reset Encoder -----
	m_lEncoder->Reset();
	m_rEncoder->Reset();
	
	// ----- Initialize -----
	m_straightVal = 0.0;
	m_rotateVal = 0.0;
	f_enabled = false;
	f_setted = false;
}

void DriveAuton::Set(double dist, double angle)
{
	if (!f_setted) {
		m_dStraight->Set(dist);
		m_dRotate->Set(angle);
		f_setted = true;
	}
}

void DriveAuton::Enable()
{
	m_dStraight->Enable();
	m_dRotate->Enable();
	f_enabled = true;
}

void DriveAuton::Disable()
{
	if (f_enabled) {
		m_dStraight->Enable();
		m_dRotate->Enable();
		f_enabled = false;
		f_setted = false;
	}
}

bool DriveAuton::IsEnabled()
{
	return f_enabled;
}

double DriveAuton::GetDist()
{
	return m_dStraight->PIDGet();
}

double DriveAuton::GetSetDist()
{
	return m_dStraight->GetSetPoint();
}

double DriveAuton::GetAngle()
{
	return m_dRotate->PIDGet();
}

double DriveAuton::GetSetAngle()
{
	return m_dRotate->GetSetPoint();
}

bool DriveAuton::IsFinished()
{
	return IsDriveFinished() && IsRotateFinished();
}

bool DriveAuton::IsRotateFinished()
{
	return m_dRotate->IsFinished();
}

bool DriveAuton::IsDriveFinished()
{
	return m_dStraight->IsFinished();
}
		

// Dummy Straight ***********************************

DriveStraightSource::DriveStraightSource(DriveAuton *p)
{
	parent = p;
	PID = new PIDController(DRV_P, DRV_I, DRV_D, this, this);
}

void DriveStraightSource::Set(double dist)
{
	PID->SetSetpoint(dist);
}

void DriveStraightSource::Enable()
{
	PID->Enable();
}

void DriveStraightSource::Disable()
{
	PID->Disable();
}

double DriveStraightSource::GetSetPoint()
{
	return PID->GetSetpoint();
}

bool DriveStraightSource::IsFinished()
{
	return fabs(PID->GetSetpoint() - PIDGet()) < 4;
}

double DriveStraightSource::PIDGet()
{
	return (parent->m_lEncoder->GetDistance() - parent->m_rEncoder->GetDistance()) / (2 * REV_IN);
}

void DriveStraightSource::PIDWrite(double output)
{
	parent->m_straightVal = output;
	parent->m_robotDrive->ArcadeDrive(parent->m_straightVal, parent->m_rotateVal);
}

// Dummy Rotate *****************************************************

DriveRotateSource::DriveRotateSource (DriveAuton *p)
{
	parent = p;
	PID = new PIDController(DRV_P, DRV_I, DRV_D, this, this);
}

void DriveRotateSource::Set(double dist)
{
	PID->SetSetpoint(dist);
}

void DriveRotateSource::Enable()
{
	PID->Enable();
}

void DriveRotateSource::Disable()
{
	PID->Disable();
}

double DriveRotateSource::GetSetPoint()
{
	return PID->GetSetpoint();
}

bool DriveRotateSource::IsFinished()
{
	return fabs(PID->GetSetpoint() - PIDGet()) < 3;
}

double DriveRotateSource::PIDGet()
{
	return (parent->m_lEncoder->GetDistance() + parent->m_rEncoder->GetDistance()) / (2 * DEGREE_FACTOR);
	return 0.0;
}

void DriveRotateSource::PIDWrite(double output)
{
	parent->m_rotateVal = output;
	parent->m_robotDrive->ArcadeDrive(parent->m_straightVal, parent->m_rotateVal);
}

//Drive Straight PID***********************************

DriveStraightPID::DriveStraightPID (RobotDrive* roboDrv, Encoder* leftEncode, Encoder* rightEncode)
{
	m_drive = roboDrv;
	m_lEncode = leftEncode;
	m_rEncode = rightEncode;
}

double DriveStraightPID::PIDGet()
{
	return (((m_lEncode->GetDistance()+m_rEncode->GetDistance())/2)/REV_IN);
}

void DriveStraightPID::PIDWrite(float output)
{
	m_pidOut = output;
	
	if (output > 0.8)
		output = 0.8;
	else if (output < -0.8)
		output = -0.8;
	
	if (m_lEncode->GetDistance() + 5 > m_rEncode->GetDistance())
		m_drive->TankDrive(output - 0.1, output + 0.1);
	else if (m_rEncode->GetDistance() + 5 > m_lEncode->GetDistance())
		m_drive->TankDrive(output + 0.1, output - 0.1);
	else
		m_drive->TankDrive(output,output);
	
	SmartDashboard::PutNumber("Drive PID Output: ", output);
}

// ---------- Drive Rotate ----------
/*
 * Clockwise is Negative.
 * Angle is measured in degrees.
 * Note:
 * 	When you use the functions with initial angle, you cannot run it again and again in loop because they initialize initial angle again and again.
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
void DriveRotate::SetAngle (double forwardspeed, double forwarddistance, double angle)
{
	// Save Initilized Encoder Values
	if (!f_angleInitialized) {
		m_lEncoder -> Reset();
		m_rEncoder -> Reset();
		f_angleInitialized = true;
	}

	// Start PID
	PID->SetSetpoint(angle);
	m_forwardspeed = forwardspeed;
	m_forwarddistance = forwarddistance * REV_IN;
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
	double Dleft = m_lEncoder->GetDistance();
	double Dright = m_rEncoder->GetDistance();
	
	return fabs(Dleft - Dright) / DEGREE_FACTOR > gap;
}
bool DriveRotate::IsRotating ()
{
	return IsRotating (ROTATE_ANGLE_GAP);
}
bool DriveRotate::Finished ()
{
	return (fabs(PID->GetSetpoint() - PIDGet()) < 5);
}
bool DriveRotate::PIDIsEnabled ()
{
	return PID->IsEnabled();
}

// ----- For PID Use -----
double DriveRotate::PIDGet ()
{
	double Dleft = m_lEncoder->GetDistance();
	double Dright = m_rEncoder->GetDistance();
	return (Dleft - Dright) / (2 * DEGREE_FACTOR);
}

void DriveRotate::PIDWrite(float output)
{
	SmartDashboard::PutNumber("Value from PID: ", output);
	
	if (output > 0.8)
	    output = 0.8;
	else if (output < -0.8)
	    output = -0.8;
	
	double adveragedistance = (m_lEncoder ->GetDistance() - m_rEncoder->GetDistance())/2;
    
	if (m_forwarddistance <= adveragedistance - (5* REV_IN))
    	m_robotDrive->ArcadeDrive(m_forwardspeed , -output);
    else
    	m_robotDrive->ArcadeDrive(0 , -output);
}
