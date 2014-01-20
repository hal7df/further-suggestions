#include "WPILib.h"
#include "Defines.h"

class BuiltinDefaultCode : public IterativeRobot
{
    // ----- Driver and Operator -----
    Joystick* m_driver;
    Joystick* m_operator;
    DriverStationLCD* m_ds;
    
    // ----- Robot Components -----
    // Victors
    Victor* m_lDrive;       // 1
    Victor* m_rDrive;       // 2
    Victor* m_launcher1;    // 4
    Victor* m_launcher2;    // 5
    Victor* m_launcher3;    // 6
    Victor* m_feeder;       // 7
    Victor* m_plate1;       // 8
    Victor* m_plate2;       // 10
    
    // Relay
    Relay* m_climeRachet;   // 1
    
    // Encoder
    Encoder* m_lDriveEnc;   // 1, 2
    Encoder* m_rDriveEnc;   // 3, 4
    
    // Potentiometer
    AnnalogChannel* m_platePoten;   // 1
    AnnalogChannel* m_climberPoten; // ?
    
    // Limit Switch
    DigitalInput* m_climberLim;     // ?
    DigitalInput* m_feederLim;      // ?
    
    
    // ----- Robot Controller -----
    RobotDrive* m_robotDrive;       // Control Drive
    DriveControl* m_drive;          // Control Drive (Better)
    PlateControl* m_plate;          // Control Plate
    LauncherControl* m_launcher;    // Control Launcher
    

public:
	BuiltinDefaultCode()	{
        
	}
	
	
	// ----- Initial Routines -----
	void RobotInit() {
	  
	}
	
	void DisabledInit() {
	  
	}

	void AutonomousInit() {
	  
	}

	void TeleopInit() {
	  
	}

	// ----- Periodic Routines -----
	void DisabledPeriodic()  {
	  
	}

	void AutonomousPeriodic() {
	  
	}

	
	void TeleopPeriodic() {

	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);
