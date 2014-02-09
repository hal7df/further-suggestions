#include "WPILib.h"
#include "Defines.h"

class PlateControl
{
public:
    PlateControl (
        int moterMap1 = PLATE1, int moterMap2 = PLATE2, int potenMap = PLATE_POTEN,     // Mapping
        int p = PLATE_P, int i = PLATE_I, int d = PLATE_D                               // PID Setting
    );
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
    // - Moter Mapping (0x10)
    // - Poten Mapping (0x20)
    void PrintStat (int option = 0x3f);
    
    const int tPos,   // 0x1
              tSpeed, // 0x2
              cPos,   // 0x4
              cSpeed, // 0x8
              mMap,   // 0x10
              pMap;   // 0x20
    
private:
    PIDController* m_PID;
    Victor* m_plate1;
    Victor* m_plate2;
    AnalogChannel* m_poten;
    
    float tSpeed, tPosition;    // Target Speed and Position
    float p, i, d;              // PID Control Parameter
    
}