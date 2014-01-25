#include "WPILib.h"
#include "Math.h"
#include "nivision.h"

class cameraHandler     
{
public:
    cameraHandler(AxisCamera *camera, DriverStationLCD *m_dsLCD, SmartDashboard *dash);
    double getCenter();
private:
    AxisCamera *camera;
    DriverStationLCD *m_dsLCD;
    ColorImage *img;
    SmartDashboard *dash;
};
