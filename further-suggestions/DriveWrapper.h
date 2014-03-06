#include "WPILib.h"
#include "Defines.h"
#include <cmath>

#ifndef DRIVEWRAPPER_H
#define DRIVEWRAPPER_H

class DriveWrapper : public SpeedController {
public:
	//Constructor
	DriveWrapper(SpeedController*, SpeedController*);
	
	void Set(float,uint8_t);
	void Set(float,float,uint8_t);
	float Get();
	void Disable();
	void PIDWrite(float);
	
	void Rotate(double speed, double angle);
	void RatateEnable();
	bool IsRotating();
	
private:
	SpeedController* m_drive1;
	SpeedController* m_drive2;
	
	
};

class DriveRotate: public PIDSource, public PIDOutput
{
public:
	DriveRotate (RobotDrive* robotDrive, Encoder* lEncoder, Encoder* rEncoder);
	
	void SetAngle (double angle);
	void PIDEnable ();
	void PIDDisable ();
	bool IsRotating (double gap);
	bool IsRotating ();

	void PIDWrite(float input);
	double PIDGet();
private:
	// ----- Components -----
	RobotDrive* m_robotDrive;
	Encoder* m_lEncoder;
	Encoder* m_rEncoder;
	
	// ----- PID -----
	PIDController* PID;
	
	// ----- Values -----
	double iniLEncoder, iniREncoder;
	
	// ----- Flags -----
	// TO Avoid initializing ini_angle in loop
	bool f_angleInitialized;
	// Check the direction of rotation
	enum {kLeft, kRight} f_turningDirection;
};

#endif //DRIVEWRAPPER_H
