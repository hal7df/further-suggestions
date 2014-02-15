	#include "WPILib.h"
	#include <cmath>
	#include "nivision.h"

	enum state_t {
		kNone,
		kLeft,
		kRight,
		kError
	};

	class CameraHandler	
	{
	public:
		CameraHandler(AxisCamera *camera, DriverStationLCD *m_dsLCD, Relay *relay);
		double getCenter();
		state_t getHotGoal();
		bool getLeftHot();
		bool getRightHot();
	private:
		AxisCamera *camera;
		DriverStationLCD *m_dsLCD;
		ColorImage *img;
		ColorImage *img2;
		Relay *light;
	};
