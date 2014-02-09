#include "WPILib.h"
#include "Defines.h"
#include "Plate.h"
#include <cmath>


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
    Victor* m_climber;		// 3
    Victor* m_launcher1;    // 4
    Victor* m_launcher2;    // 5
    Victor* m_launcher3;    // 6
    Victor* m_feeder;       // 7
    Victor* m_plate1;       // 8
    Victor* m_plate2;       // 10
    
    // Relay
    Relay* m_climberRachet;   // 1
    
    // Encoder
    Encoder* m_lDriveEnc;   // 1, 2
    Encoder* m_rDriveEnc;   // 3, 4
    
    // Potentiometer
    AnalogChannel* m_platePoten;   // 1
    AnalogChannel* m_climberPoten; // ?
    
    // Limit Switch
    DigitalInput* m_climberLimSwitch;     // ?
    DigitalInput* m_feederLimSwitch;      // ?
    
    // Timer
    Timer* m_launcherTimer;
    Timer* m_climberTimer;
    
    // ----- Robot Controller -----
    RobotDrive* m_drive;       // Control Drive
    PIDController* m_plate;
    PlateWrap* m_plateWrap;
    
    // ----- Status Parameter -----
    PlateStatus plateStat;
    FeederStatus feederStat;
    LauncherStatus launcherStat;
    ClimberStatus climberStat;
    
    // ----- Private Functions -----
	float joystickAdjust (float input)
	{
		if (input < -0.2) {
			return - pow((input + 0.2) / 0.8, 2.0);
		} else if (input < 0.2) {
			return 0.0;
		} else {
			return pow((input - 0.2) / 0.8, 0.2);
		}
	}

public:
	BuiltinDefaultCode()	{
        m_driver = new Joystick(DRIVER);
        m_operator = new Joystick(OPERATOR);
        m_ds = DriverStationLCD::GetInstance();

        m_lDrive = new Victor(lDRIVE);
        m_rDrive = new Victor(rDRIVE);
        m_climber = new Victor(CLIMBER);
        m_launcher1 = new Victor(LAUNCHER1);
        m_launcher2 = new Victor(LAUNCHER2);
        m_launcher3 = new Victor(LAUNCHER3);
        m_feeder = new Victor(FEEDER);
        m_plate1 = new Victor(PLATE1);
        m_plate2 = new Victor(PLATE2);
        
        m_climberRachet = new Relay(CLIMBRACHET);
        
        m_lDriveEnc = new Encoder (lDRIVE_ENCODER1, lDRIVE_ENCODER2);
        m_rDriveEnc = new Encoder (rDRIVE_ENCODER1, rDRIVE_ENCODER2);
        
        m_platePoten = new AnalogChannel (PLATE_POTEN);
        m_climberPoten = new AnalogChannel (CLIMBER_POTEN);
        
        m_climberLimSwitch = new DigitalInput (CLIMBER_LIM_SWITCH);
        m_feederLimSwitch = new DigitalInput (FEEDER_LIM_SWITCH);
        
        // ----- Timers -----
        m_launcherTimer = new Timer;
        m_climberTimer = new Timer;
        
        // ----- Robot Controller -----
        m_drive = new RobotDrive (m_lDrive, m_rDrive);
        m_drive->SetSafetyEnabled(false);
        m_plateWrap = new PlateWrap(m_plate1, m_plate2);
        m_plate = new PIDController(PLATE_P, PLATE_I, PLATE_D, m_platePoten, m_plateWrap);

        // ----- Status -----
        climberStat = cSTOPPED;
        
	}
	
	// ----- Initial Routines -----
	void RobotInit() {
	  
	}
	
	void DisabledInit() {
	  
	}

	void AutonomousInit() {
	  
	}

	void TeleopInit() {
		// Reset Timers
        m_launcherTimer->Start();
		m_launcherTimer->Stop();
		m_launcherTimer->Reset();
		
		m_climberTimer->Start();
		m_climberTimer->Stop();
		m_climberTimer->Reset();
		
	}

	// ----- Periodic Routines -----
	void DisabledPeriodic()  {
	  
	}

	void AutonomousPeriodic() {
	  
	}

	
	void TeleopPeriodic() {
        // ----- Drive -----
		m_drive->ArcadeDrive (-m_driver->GetRawAxis(LEFT_Y), -m_driver->GetRawAxis(RIGHT_X));
        

        // ----- Launcher -----
        if(m_operator->GetRawButton(BUTTON_X) && m_launcherTimer->Get() == 0) {
            // Start Launcher
            m_launcherTimer->Start();
        }
        
        if(m_launcherTimer->Get() < LAUNCHER_SAFTY_TIME) {
            m_launcher1->Set((float)m_launcherTimer->Get()/LAUNCHER_SAFTY_TIME);
            m_launcher2->Set((float)m_launcherTimer->Get()/LAUNCHER_SAFTY_TIME);
            m_launcher3->Set((float)m_launcherTimer->Get()/LAUNCHER_SAFTY_TIME);
            
            launcherStat = lSTARTING;
        } 
        else {
            m_launcher1->Set(1);
            m_launcher2->Set(1);
            m_launcher3->Set(1);
            
            launcherStat = lRUNNING;
        }
        
        if(m_operator->GetRawButton(BUTTON_Y)) {
            // Stop Launcher
            m_launcherTimer->Stop();
            m_launcherTimer->Reset();
            
            launcherStat = lSTOPPED;
        }
        
        // ----- Feeder -----
        if(m_operator->GetRawButton(BUTTON_A)) {
        	m_feeder->Set(1.0);
        	feederStat = fRUNNING;
        } 
        else {
        	m_feeder->Set(0.0);
        	feederStat = fSTOPPED;
        }
        
        // ----- Plate -----
        if(m_driver->GetRawButton(BUTTON_A)) {
        	// ----- Pyramid Three Point -----
        	m_plate->SetSetpoint(PLATE_PYRAMID_THREE_POINT);
        	m_plate->Enable();
        	
        	plateStat = pPYRAMID_THREE;
        } else if(m_driver->GetRawButton(BUTTON_B)) {
        	// ----- Feeder Three Point -----
        	m_plate->SetSetpoint(PLATE_FEEDER_THREE_POINT);
        	m_plate->Enable();
        	
        	plateStat = pFEEDER_THREE;
        } else if(m_driver->GetRawButton(BUTTON_X)) {
        	// ----- Feeder Two Point -----
        	m_plate->SetSetpoint(PLATE_TEN_POINT_CLIMB);
        	m_plate->Enable();
        	
        	plateStat = pFEEDER_TWO;
        } else {
        	// ----- Manual Control -----
        	m_plate->Disable();
        	
        	m_plate1->Set(-m_driver->GetRawAxis(TRIGGERS));
        	m_plate2->Set(-m_driver->GetRawAxis(TRIGGERS));
        	
        	plateStat = pMANUAL;
        }
        
        // ----- Climber -----
        // it is perfect up now. maybe
        if (- m_operator->GetRawAxis(LEFT_Y) > 0.2) {
        	// EXTEND
			if (climberStat != cEXTENDING) {
				climberStat = cEXTENDING;

				m_climberTimer->Stop();
				m_climberTimer->Reset();
				m_climberTimer->Start();
			}
		}
		else if (- m_operator->GetRawAxis(LEFT_Y) < -0.2) {
			// CLOSE
			if (climberStat != cCLOSING) {
				climberStat = cCLOSING;
			}
		}
		else {
			// STOP
			
			if (climberStat != cSTOPPED) {
				climberStat = cSTOPPED;

				m_climberTimer->Stop();
				m_climberTimer->Reset();
				m_climberTimer->Start();
			}
		}
 
		switch (climberStat)  {
		case cEXTENDING:
			if (m_climberTimer->Get() < 0.125) {
				m_climberRachet->Set(Relay::kForward);
				m_climber->Set(-0.3);
			}
			else {
				m_climberRachet->Set(Relay::kOff);
				m_climber->Set(-m_operator->GetRawAxis(LEFT_Y));
			}
			break;
		case cSTOPPED:
			if (m_climberTimer->Get() < 0.125) {
				m_climber->Set(0.0);
				m_climberRachet->Set(Relay::kReverse);
			} else {
				m_climberRachet->Set(Relay::kOff);
			}
			break;
		case cCLOSING:
			if (m_climberTimer->Get() < 0.125) {
				m_climberRachet->Set(Relay::kReverse);
			}
			else {
				m_climberRachet->Set(Relay::kOff);
				m_climber->Set(-m_operator->GetRawAxis(LEFT_Y));
			}
			break;
		}
		
        // ----- Display -----
        SmartDashboard::PutNumber("Climber Status: ", climberStat);
        SmartDashboard::PutNumber("Plate Status: ", plateStat);
        SmartDashboard::PutNumber("Feeder Status: ", feederStat);
        SmartDashboard::PutNumber("Launcher Status: ", launcherStat);
                
        SmartDashboard::PutNumber("Plate Position: ", m_platePoten->GetVoltage());
        SmartDashboard::PutNumber("Climber Position: ", m_climberPoten->GetVoltage());
        
        SmartDashboard::PutBoolean("Climber Limit Switch: ", m_climberLimSwitch->Get());
        SmartDashboard::PutBoolean("Feeder Limit Switch: ", m_feederLimSwitch->Get());
	}
};


START_ROBOT_CLASS(BuiltinDefaultCode);


