#include "WPILib.h"

class PlateWrap: public PIDOutput
{
public:
	PlateWrap::PlateWrap (Victor* m_moter1, Victor* m_moter2);
	void PIDWrite (float output);
private:
	Victor* m_plate1;
	Victor* m_plate2;
};
