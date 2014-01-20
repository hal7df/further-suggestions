#include "Launcher.h"
#include "WPILib.h"
#include "Defines.h"

LauncherControl::LauncherControl (
        int motorMap1 = LAUNCHER1, int motorMap2 = LAUNCHER2, int motorMap3 = LAUNCHER3,
        float saftyTime = LAUNCHER_SAFTY_TIME
    )
{
    // ----- Get Objects -----
    m_launcher1 = new Victor (motorMap1);
    m_launcher2 = new Victor (motorMap2);
    m_launcher3 = new Victor (motorMap3);
    saftyT = saftyT;
}

LauncherControl::Set (float speed)
{
    tSpeed = speed;
    
    if(tSpeed > m_launcher1->Get()) {
        // Timer Start
        
    }
}

LauncherControl::Enable ()
{
    if(tSpeed > )
}