#include "WPILib.h"

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
private:
	SpeedController* m_drive1;
	SpeedController* m_drive2;
};

#endif //DRIVEWRAPPER_H
