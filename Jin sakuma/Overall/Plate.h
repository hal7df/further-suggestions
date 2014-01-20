#include "WPILib.h"
#include "Defines.h"

class PlateControl
{
public:
    PlateControl (int moterChannel1 = PLATE1, int moterChannel2 = PLATE2, int potenChannel = PLATE_POTEN);
    void SetSpeed (float speed);                // Set Target Speed
    void SetPosition (float pos);               // Set Target Position
    float GetSpeed ();                          // Return current speed
    float GetPosition ();                       // Return current position
    
    // Set PID Parameter
    void SetPID (float p, float i, float d);    // Set parameter of PID Control
    
    // Display Status
    // - Target Position (0x1)
    // - Target Speed (0x2)
    // - Current Position (0x4)
    // - Current Speed (0x8)
    // - Moter Channel (0x10)
    // - Poten Channel (0x20)
    void Printstat (int option = 0x7f);
    
private:
    PIDController* m_PID;
    Victor* m_plate1;
    Victor* m_plate2;
    AnalogChannel* m_poten;
    
    float tSpeed, tPosition;    // Target Speed and Position
    float p, i, d;              // PID Control Parameter
    
}