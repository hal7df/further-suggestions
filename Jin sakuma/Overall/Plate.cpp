#include "Plate.h"
#include "Defines.h"

PlateWrap::PlateWrap (Victor* m_moter1, Victor* m_moter2)
{
	m_plate1 = m_moter1;
	m_plate2 = m_moter2;
}

void PlateWrap::PIDWrite(float output)
{
	m_plate1->Set(output);
	m_plate2->Set(output);
}
