#include "WIPLib.h"
#include "Defines.s"

class LauncherControl
{
public:
    LauncherControl (
        int motorMap1 = LAUNCHER1, int motorMap2 = LAUNCHER2, int motorMap3 = LAUNCHER3,
        float saftyTime = LAUNCHER_SAFTY_TIME
    );
    void Set (float speed);
    float Get ();
    
    void Enable ();
    
    float SetSaftyTime(float time);
    
    // - Target Speed (0x1)
    // - Current Speed (0x2)
    // - Mapping (0x4)
    void PrintStat (int option = 0x7);
    
    const int target,
              current,
              map;
Private:
    Victor* m_launcher1;
    Victor* m_launcher2;
    Victor* m_launcher3;
    Timer* m_timer;
    float saftyT;
    
    float tSpeed;
    bool accel;
}