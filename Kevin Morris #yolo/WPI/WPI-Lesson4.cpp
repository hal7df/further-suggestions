#include "WPILib.h"
#include "Defines.h"

class BuiltinDefaultCode : public IterativeRobot
{
	
		
public:
	Joystick *m_BUTTON_A;
	Joystick *m_BUTTON_B;
	DriverStationLCD* m_dsLCD;
	Timer *m_timer;
	Float x;
	
	BuiltinDefaultCode()	{
	
	m_timer = new Timer();
	m_BUTTON_A = new Joystick (1);
	m_BUTTON_B = new Joystick (2);
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
	    if (m_timer->HasPeriodPassed(1.5)) {
	    	if (m_BUTTON_A->GetRawButton(1)) {
	    		void SmartDashboard::PutBoolean("A Button Pressed!");
	    	    SmartDashboard::PutNumber("Counter: ",x++);
	    	    m_timer->Stop();
	    	    m_timer->Start();
	    	    m_timer->Reset();
	    	}
	    }
	   if (m_timer->HasPeriodPassed(1.5)) {
	    	if (m_BUTTON_B->GetRawButton(1)) {
	    		void SmartDashboard::PutBoolean("B Button Pressed!");
	    	    SmartDashboard::PutNumber("Counter: ",x--);
	    	    m_timer->Stop();
	    	    m_timer->Start();
	    	    m_timer->Reset();
	    	}
	    }
	    m_dsLCD->UpdateLCD();
	  
	} // TeleopPeriodic()

/********************************** Miscellaneous Routines *************************************/
	
};

START_ROBOT_CLASS(BuiltinDefaultCode);
