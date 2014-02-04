#include "WPILib.h"
#include "Defines.h"
#include "cmath"

class BuiltinDefaultCode : public IterativeRobot
{

	Joystick *m_gamepad1;
	DriverStationLCD* m_dsLCD;
	Timer *m_timer;
	double x;
	double y;	
		
public:

	BuiltinDefaultCode()	{
	
	m_timer = new Timer();
	m_gamepad1 = new Joystick (1);
    m_dsLCD = DriverStationLCD::GetInstance();
    x = 0.00;
	}
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
	    if (m_timer->HasPeriodPassed(.1)) {
	    	if (m_gamepad1->GetRawButton(1)) {
	    		m_timer->Start();
	    		x+=1;
	    	    m_timer->Stop();
	    	    m_timer->Reset();
	    	}
	    }
	   if (m_timer->HasPeriodPassed(.1)) {
	    	if (m_gamepad1->GetRawButton(1)) {
	    	    m_timer->Start();
	    	    x-=1;
	    	    m_timer->Stop();
	    	    m_timer->Reset();
	    	}
	   }
	   SmartDashboard::PutNumber("Button: ", x);
	   
	   if (fabs(m_gamepad1 -> GetRawAxis(LEFT_Y)) > 0.2) {
		   y += -m_gamepad1 -> GetRawAxis(LEFT_Y);
	   }
	   SmartDashboard::PutNumber("Stick: ", y);
	} // TeleopPeriodic()

/********************************** Miscellaneous Routines *************************************/
	
};

START_ROBOT_CLASS(BuiltinDefaultCode);
