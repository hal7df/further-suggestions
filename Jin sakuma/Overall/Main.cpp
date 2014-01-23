#include "WPILib.h"
#include "Defines.h"
#include "Plate.h"

float joystickAdjust (float input)
{
    if(input < 0.2) {
        output = 0.0;
    } else {
        output = 1 / 0.8^2 * (input - 0.2)^2;
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
    VIctor* m_climber;		// 3
    Victor* m_launcher1;    // 4
    Victor* m_launcher2;    // 5
    Victor* m_launcher3;    // 6
    Victor* m_feeder;       // 7
    Victor* m_plate1;       // 8
    Victor* m_plate2;       // 10
    
    // Relay
    Relay* m_climbRachet;   // 1
    
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
    string launcherStatus;
    string feederStatus;
    string plateStatus;
    string climberStatus;
    
public:
	BuiltinDefaultCode()	{
        m_driver = new Joystick(DRIVER);
        m_driver = new Joystick(OPERATOR);
        m_ds = DriverStation::GetInstatnce();
        
        m_lDrive = new Victor(lDRIVE);
        m_rDrive = new Victor(rDRIVE);
        m_climber = new Victor(CLIMBER);
        m_launcher1 = new Victor(LAUNCHER1);
        m_launcher2 = new Victor(LAUNCHER2);
        m_launcher3 = new Victor(LAUNCHER3);
        m_feeder = new Victor(FEEDER);
        m_plate1 = new Victor(PLATE1);
        m_plate2 = new Victor(PLATE2);
        
        m_climbRachet = new Relay(CLIMBRACHET);
        
        m_lDriveEnc = new Encoder (lDRIVE_ENCODER1, lDRIVE_ENCODER2);
        m_rDriveEnc = new Encoder (rDRIVE_ENCODER1, rDRIVE_ENCODER2);
        
        m_platePoten = new AnnalogChannel (PLATE_POTEN);
        m_climberPoten = new AnnalogChannel (CLIMBER_POTEN);
        
        m_climberLimSwitch = new DigitalInput (BLIMBER_LIM);
        m_feederLimSwitch = new DigitalInput (FEEDER_LIM);
        
        // ----- Timers -----
        m_launcherTimer = new Timer;
        m_climberTimer = new Timer;
        
        // ----- Robot Controller -----
        m_drive = new RobotDrive (m_lDrive, m_rDrive);
        m_plateWrap = new PlateControl(m_plate1, m_plate2);
        m_plate = new PIDController(PLATE_P, PLATE_I, PLATE_D, m_platePoten, m_plateWrap);
        
        // ----- Status -----
        climberStatus = "Locked";
        launcherStatus = "Stopped";
        feederStatus = "Stopped";
        plateStatus = "Manual";
		
		enum ClimberStat {
			EXTEND, STOP, CLOSE
		} climberStat;
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
        RobotDrive->Arcade (joystickAdjust(m_driver->GetRawAxis(LEFT_Y)), joystickAdjust(m_driver->GetRawAxis(RIGHT_X)));
        
        // ----- Launcher -----
        if(m_operator->GetRawButton(BUTTON_X) && m_launcherTimer->Get() == 0) {
            // Start Launcher
            m_launcherTimer->Start();
        }
        
        if(m_launcherTimer->Get() < LAUNCHER_SAFTY_TIME) {
            m_launcher1->Set((float)m_launcherTimer->Get()/LAUNCHER_SAFTY_TIME);
            m_launcher2->Set((float)m_launcherTimer->Get()/LAUNCHER_SAFTY_TIME);
            m_launcher3->Set((float)m_launcherTimer->Get()/LAUNCHER_SAFTY_TIME);
            
            launcherStatus = "Starting";
        } else {
            m_launcher1->Set(1);
            m_launcher2->Set(1);
            m_launcher3->Set(1);
            
            launcherStatus = "Running";
        }
        
        if(m_operator->GerRawButton(BUTTON_Y)) {
            // Stop Launcher
            m_launcherTimer->Stop();
            m_launcherTimer->Reset();
            
            launcherStatus = "Stopped";
        }
        
        // ----- Feeder -----
        if(m_operator->GetRawButton(BUTTON_A)) {
        	m_feeder->Set(-1.0);
        	feederStatus = "Running";
        } else {
        	m_feeder->Set(0.0);
        	feederStatus = "Stopped";
        }
        
        // ----- Plate -----
        if(m_driver->GetRawButton(BUTTON_A)) {
        	// ----- Pyramid Three Point -----
        	m_plate->SetSetpoint(PLATE_PYRAMID_THREE_POINT);
        	m_plate->Enable();
        	
        	plateStatus = "Pyramid Three Point";
        } else if(m_driver->GetRawButton(BUTTON_B)) {
        	// ----- Feeder Three Point -----
        	m_plate->SetSetpoint(PLATE_FEEDER_THREE_POINT);
        	m_plate->Enable();
        	
        	plateStatus = "Feeder Three Point";
        } else if(m_driver->GetRawBUtton(BUTTON_Y)) {
        	// ----- Feeder Two Point -----
        	m_plate->SetSetpoint(PLATE_TEN_POINT_CLIMB);
        	m_plate->Enable();
        	
        	plateStatus = "Feeder Two Point";
        } else {
        	// ----- Manual Control -----
        	m_plate->Disable();
        	
        	m_plate1->Set(joystickAdjust(m_driver->GetRawAxis(TRIGGERS)));
        	m_plate2->Set(joystickAdjust(m_driver->GetRawAxis(TRIGGERS)));
        	
        	plateStatus = "Manual";
        }
        
        // ----- Climber -----
        // it is messed up now.
		if (joysticAdjust(m_operator->GetRawAxis(LEFT_Y)) > 0.0) {
			if (climberStat != EXTEND) {
				cliberStat = EXTEND;

				climberTimer->Stop();
				climberTimer->Reset();
				climberTimer->Start();
			}
		}
		else if (joysticAdjust(m_operator->GetRawAxis(LEFT_Y)) == 0.0) {
			// STOP
			if (climberStat != STOP) {
				cliberStat = STOP;

				climberTimer->Stop();
				climberTimer->Reset();
				climberTimer->Start();
			}
		}
		else if (joysticAdjust(m_operator->GetRawAxis(LEFT_Y)) < 0.0) {
			// CLOSE
			if (climberStat != CLOSE) {
				cliberStat = CLOSE;
			}
		}
 
		switch (climberStat)  {
		case EXTEND:
			if (climberTimer->Get() < 0.125) {
				climberRachet->Set(Relay::kForward);
				climber->Set(-0.3);
			}
			else {
				climber->Set(joystickAdjust(m_operator->GetRawAxis(LEFT_Y)));
			}
			break;
		case STOP:
			if (climberTimer->Get() > 0.125) {
				climer->Set(0.0);
				climerRachet->Set(Relay::kReverse);
			}
			break;
		case CLOSE:
			if (climberTimer->Get() > 0.125) {
				climerRachet->Set(Relay::kReverse);
			}
			else {
				climber->Set(joystickAdjust(m_operator->GetRawAxis(LEFT_Y)));
			}
			break;
		}
        
        
        // ----- Display -----
        SmartDashboard::PutString("Climber Status: ", climberStatus);
        SmartDashboard::PutString("Plate Status: ", plateStatus);
        SmartDashboard::PutString("Feeder Status: ", feederStatus);
        SmartDashboard::PutString("Launcher Status: ", launcherStatus);
                
        SmartDashboard::PutNumber("Plate Position: ", m_platePoten->GetVoltage());
        SmartDashboard::PutNumber("Climber Position: ", m_climberPoten->GetVoltage());
	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);
