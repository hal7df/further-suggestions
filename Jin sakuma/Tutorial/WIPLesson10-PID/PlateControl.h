#include "WPILib.h"

class PlateControl: public PIDOutput
{
public:
	PlateControl::PlateControl (Victor* m_moter1, Victor* m_moter2);
	void PIDWrite (float output);
private:
	Victor* m_plate1;
	Victor* m_plate2;
};
