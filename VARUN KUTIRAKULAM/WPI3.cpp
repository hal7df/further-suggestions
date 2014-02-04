#include "WPILib.h"


class BuiltinDefaultCode : public IterativeRobot
{
	
		
public:
		RobotDrive *m_robotDrive;
		Joystick *m_driver;
		Joystick *m_operator;
		Victor *m_lDrive;
		Victor *m_rDrive;
		//l is for left and r is for right
		Victor *m_climber;
		Victor *m_plate1;
		Victor *m_plate2;
		Victor *m_launcher1;
		Victor *m_launcher2;
		Victor *m_launcher3;
		Relay *m_ratchet;
		Encoder *m_left;
		Encoder *m_right;
		AnalogChannel *m_plateH;
		AnalogChannel *m_climberP;
		DriverStationLCD *m_dsLCD;
		
	BuiltinDefaultCode()	{
		m_robotDrive = new RobotDrive (m_lDrive, m_rDrive);
		m_lDrive = new Victor (1);
		m_rDrive = new Victor (2);
		m_driver = new Joystick (1);
		m_operator = new Joystick (2);
		m_climber = new Victor (3);
		m_plate1 = new Victor (8);
		m_plate2 = new Victor (10);
		m_launcher1 = new Victor (4);
		m_launcher2 = new Victor (5);
		m_launcher3 = new Victor (6);
		m_ratchet = new Relay (1);
		m_left = new Encoder (1,2,true);
		m_left->SetDistancePerPulse(1);
		m_left->SetMaxPeriod(1.0);
		m_left->Start();
		m_right = new Encoder (3,4,false);
		m_right->SetDistancePerPulse(1);
		m_right->SetMaxPeriod(1.0);
		m_right->Start();
		m_plateH = new AnalogChannel (1);
		m_climberP = new AnalogChannel (2);
		m_dsLCD = DriverStationLCD::GetInstance();
	}
	
	
	/********************************** Init Routines *************************************/


	void RobotInit() {
	  
	}

	void DisabledInit() {
	  
	}

	void AutonomousInit() {
	  
	}

	void TeleopInit() {
	  
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic()  {
	  
	}

	void AutonomousPeriodic() {
	  
	}

	
	void TeleopPeriodic() {
	  
	} // TeleopPeriodic()

/********************************** Miscellaneous Routines *************************************/
	
};

START_ROBOT_CLASS(BuiltinDefaultCode);
