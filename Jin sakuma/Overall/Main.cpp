#include "WPILib.h"
#include "Defines.h"

float joystickAdjust (float input)
{
    if(input < 0.2) {
        output = 0.0;
    } else {
        output = 1/0.8*(input-0.2)^2;
    }
    
    return output;
}

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
    Relay* m_climbeRachet;   // 1
    
    // Encoder
    Encoder* m_lDriveEnc;   // 1, 2
    Encoder* m_rDriveEnc;   // 3, 4
    
    // Potentiometer
    AnnalogChannel* m_platePoten;   // 1
    AnnalogChannel* m_climberPoten; // ?
    
    // Limit Switch
    DigitalInput* m_climberLimSwitch;     // ?
    DigitalInput* m_feederLimSwitch;      // ?
    
    
    // ----- Robot Controller -----
    RobotDrive* m_drive;       // Control Drive
    

public:
	BuiltinDefaultCode()	{
        m_driver = new Joystick(DRIVER);
        m_driver = new Joystick(OPERATOR);
        m_ds = DriverStation::GetInstatnce();
        
        m_lDrive = new Victor(lDRIVE);
        m_rDrive = new Victor(rDRIVE);
        m_launcher1 = new Victor(LAUNCHER1);
        m_launcher2 = new Victor(LAUNCHER2);
        m_launcher3 = new Victor(LAUNCHER3);
        m_feeder = new Victor(FEEDER);
        m_plate1 = new Victor(PLATE1);
        m_plate2 = new Victor(PLATE2);
        
        m_climbrachet = new Relay(CLIMBRACHET);
        
        m_ldriveEnc = new Encoder (lDRIVE_ENCODER1, lDRIVE_ENCODER2);
        m_rDriveEnc = new Encoder (rDRIVE_ENCODER1, rDRIVE_ENCODER2);
        
        m_platePoten = new AnnalogChannel (PLATE_POTEN);
        m_climberPoten = new AnnalogChannel (CLIMBER_POTEN);
        
        m_climberLimSwitch = new DigitalInput (BLIMBER_LIM);
        m_feederLimSwitch = new DigitalInput (FEEDER_LIM);
        
        // ----- Timers -----
        m_launcherTimer = new Timer;
        
        // ----- Robot Controller -----
        m_drive = new RobotDrive (m_lDrive, m_rDrive);
	}
	
	
	// ----- Initial Routines -----
	void RobotInit() {
	  
	}
	
	void DisabledInit() {
	  
	}

	void AutonomousInit() {
	  
	}

	void TeleopInit() {
        m_launcherTimer->Start();
		m_launcherTimer->Stop();
		m_launcherTimer->Reset();
	}

	// ----- Periodic Routines -----
	void DisabledPeriodic()  {
	  
	}

	void AutonomousPeriodic() {
	  
	}

	
	void TeleopPeriodic() {
        // ----- Drive -----
        RobotDrive->Arcade (joystickAdjust(m_driver->GetRawAxis(LEFT_Y)), joystickAdjust(m_driver->GetRawAxis(RIGHT_X)));
        
        // ----- Launcher -----
        if(m_driver->GetRawButton(BUTTON_A) && m_launcherTimer->Get() == 0) {
            // Start Launcher
            m_launcherTimer->Start();
        }
        
        if(m_launcherTimer->Get() < LAUNCHER_SAFTY_TIME) {
            m_launcher1-<Set((float)m_launcherTimer->Get()/LAUNCHER_SAFTY_TIME);
            m_launcher2-<Set((float)m_launcherTimer->Get()/LAUNCHER_SAFTY_TIME);
            m_launcher3-<Set((float)m_launcherTimer->Get()/LAUNCHER_SAFTY_TIME);
        } else {
            m_launcher1->Set(1);
            m_launcher2->Set(1);
            m_launcher3->Set(1);
        }
        
        if(m_driver->GerRawButton(BUTTON_B)) {
            // Stop Launcher
            m_launcherTimer->Stop();
            m_launcherTimer->Reset();
        }
        
        // ----- Plate -----
        
	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);
