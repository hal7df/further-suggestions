#include "WPILib.h"
#include "Defines.h"
// #include "camera.h"
#include <cmath>


class BuiltinDefaultCode : public IterativeRobot
{
	// Gamepad
	Joystick *m_driver;
	Joystick *m_operater;
	
	// Declare variable for the robot drive system
	RobotDrive *m_robotDrive;               // robot will use PWM 1-4 for drive motors
	
	// Declare Drive Motors
	Victor *m_lDrive;
	Victor *m_rDrive;
	
	// Declare Shooter Motors
	Jaguar *m_Cannon1;
	Jaguar *m_Cannon2;
	Relay *m_Index;
	Victor *m_hopperRoller;
	int m_spinning;
	Timer *m_Cannontimer;
	
	// Arm
	Jaguar *m_arm1;
	Jaguar *m_arm2;
	Jaguar *m_Roller1;
	Jaguar *m_Roller2;
	
	// Driver Station
	DriverStationLCD *m_dsLCD;
	SmartDashboard *m_dash;
	
	// Camera
	AxisCamera *m_camera;
	
	// Light
	Relay *m_light;
	
	// cameraHandler *m_camHandle;
	
	float joystickAdjust(float input) {
		if (input < -0.2) {
			// Negative
			return - 0.8 * pow((input + 0.2) / 0.8, 2.0) - 0.2;
		} else if (input < 0.2) {
			// Deadbound
			return 0.0;
		} else {
			// Positive
			return 0.8 * pow((input - 0.2) / 0.8, 2.0) + 0.2;
		}
	}

public:
	BuiltinDefaultCode()	{
		// Gamepad
		m_driver = new Joystick(1);
		m_operater = new Joystick(2);
		
		// Drive
		m_lDrive = new Victor(1);
		m_rDrive = new Victor(2);
		m_robotDrive = new RobotDrive(m_lDrive, m_rDrive);
		
		// Cannon
		m_Cannon1 = new Jaguar(5);
		m_Cannon2 = new Jaguar(6);
		m_Index = new Relay(1);
		m_hopperRoller = new Victor(3);
		m_Cannontimer = new Timer;
		m_spinning = 0;
		
		// Arm
		m_arm1 = new Jaguar(7);
		m_arm2 = new Jaguar(8);
		m_Roller1 = new Jaguar(9);
		m_Roller2 = new Jaguar(10);
		
		// Driver Station
		// m_dash = SmartDashboard::GetInstance();
		// m_dsLCD = DriverStationLCD::GetInstance();
		
		// AxisCamera *camera = &AxisCamera::GetInstance("10.0.67.11");
		// m_camHandle = new cameraHandler(camera, m_dsLCD, m_dash);
		
		// Camera
		m_camera = &AxisCamera::GetInstance("");
		
		// Light
		m_light = new Relay(3);
		
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
		// ----- Light -----
		if(m_operater->GetRawButton(BUTTON_A)) {
			m_light->Set(Relay::kForward);
		}
		
        // ----- Drive -----
		m_robotDrive->ArcadeDrive(-joystickAdjust(m_driver->GetRawAxis(LEFT_Y)), -joystickAdjust(m_driver->GetRawAxis(RIGHT_X)));
		
		// ----- Cannon ----
		if (m_driver -> GetRawButton(BUTTON_A)){
			// FULL
			m_Cannontimer -> Start();
			m_spinning = 1;
		}
		else if (m_driver -> GetRawButton(BUTTON_X)){
			// HALF
			m_Cannontimer -> Start();
			m_spinning = 2;
		}
		if (m_driver->GetRawButton(BUTTON_B)) {
			// STOP
			m_spinning = 0;
		}
		
		
		switch (m_spinning){
		case 0:		// STOP
			m_Cannon1->Set(0.0);
			m_Cannon2->Set(0.0);
			
			m_Cannontimer->Stop();
			m_Cannontimer->Reset();
			break;
		case 1:		// FULL SPEED
			if (m_Cannon1 -> Get() < -.8){
				m_Cannon1 -> Set(-1.0);
				m_Cannon2 -> Set(-1.0);
				m_Cannontimer -> Stop();
				m_Cannontimer -> Reset();
			}
			else if (!m_Cannontimer -> HasPeriodPassed(2.0)){
				m_Cannon1 -> Set(-m_Cannontimer -> Get() / 2);
				m_Cannon2 -> Set(-m_Cannontimer -> Get() / 2);
			}
			break;
		case 2:		// HALF SPEED
			if (m_Cannon1 -> Get() < -.4){
				m_Cannon1 -> Set(-.5);
				m_Cannon2 -> Set(-.5);
				m_Cannontimer -> Stop();
				m_Cannontimer -> Reset();
			}
			else if (!m_Cannontimer -> HasPeriodPassed(1.0)){
				m_Cannon1 -> Set(-m_Cannontimer -> Get() / 2);
				m_Cannon2 -> Set(-m_Cannontimer -> Get() / 2);
			}
			break;
		}
		
		
		
		// Hopper Roller
		if (fabs(m_driver->GetRawAxis(TRIGGERS)) > 0.8) {
			m_Index->Set(Relay::kForward);
			m_hopperRoller->Set(-1.0);
		} else {
			m_Index->Set(Relay::kOff);
			m_hopperRoller->Set(0.0);
		}
		
		// ----- Arm -----
		m_arm1->Set(-joystickAdjust(m_operater -> GetRawAxis(RIGHT_Y)));
		m_arm2->Set(joystickAdjust(m_operater -> GetRawAxis(RIGHT_Y)));
		m_Roller1->Set(-joystickAdjust(m_operater->GetRawAxis(TRIGGERS)));
		m_Roller2->Set(joystickAdjust(m_operater->GetRawAxis(TRIGGERS)));
	
	}
};


START_ROBOT_CLASS(BuiltinDefaultCode);


