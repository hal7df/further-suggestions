#include "WPILib.h"
#include "Defines.h"

/**
 * This "BuiltinDefaultCode" provides the "default code" functionality as used in the "Benchtop Test."
 * 
 * The BuiltinDefaultCode extends the IterativeRobot base class to provide the "default code"
 * functionality to confirm the operation and usage of the core control system components, as 
 * used in the "Benchtop Test" described in Chapter 2 of the 2009 FRC Control System Manual.
 * 
 * This program provides features in the Disabled, Autonomous, and Teleop modes as described
 * in the benchtop test directions, including "once-a-second" debugging printouts when disabled, 
 * a "KITT light show" on the solenoid lights when in autonomous, and elementary driving
 * capabilities and "button mapping" of joysticks when teleoperated.  This demonstration
 * program also shows the use of the MotorSafety timer.
 * 
 * This demonstration is not intended to serve as a "starting template" for development of
 * robot code for a team, as there are better templates and examples created specifically
 * for that purpose.  However, teams may find the techniques used in this program to be
 * interesting possibilities for use in their own robot code.
 * 
 * The details of the behavior provided by this demonstration are summarized below:
 *  
 * Disabled Mode:
 * - Once per second, print (on the console) the number of seconds the robot has been disabled.
 * 
 * Autonomous Mode:
 * - Flash the solenoid lights like KITT in Knight Rider
 * - Example code (commented out by default) to drive forward at half-speed for 2 seconds
 * 
 * Teleop Mode:
 * - Select between two different drive options depending upon Z-location of Joystick1
 * - When "Z-Up" (on Joystick1) provide "arcade drive" on Joystick1
 * - When "Z-Down" (on Joystick1) provide "tank drive" on Joystick1 and Joystick2
 * - Use Joystick buttons (on Joystick1 or Joystick2) to display the button number in binary on
 *   the solenoid LEDs (Note that this feature can be used to easily "map out" the buttons on a
 *   Joystick.  Note also that if multiple buttons are pressed simultaneously, a "15" is displayed
 *   on the solenoid LEDs to indicate that multiple buttons are pressed.)
 *
 * This code assumes the following connections:
 * - Driver Station:
 *   - USB 1 - The "right" joystick.  Used for either "arcade drive" or "right" stick for tank drive
 *   - USB 2 - The "left" joystick.  Used as the "left" stick for tank drive
 * 
 * - Robot:
 *   - Digital Sidecar 1:
 *     - PWM 1/3 - Connected to "left" drive motor(s)
 *     - PWM 2/4 - Connected to "right" drive motor(s)
 */
class BuiltinDefaultCode : public IterativeRobot
{
	// ----- Declare Components -----
	Victor* m_plate1;
	Victor* m_plate2;
	Victor* m_launcher1;
	Victor* m_launcher2;
	Victor* m_launcher3;
	Relay* m_climbLight;
	
	Timer* m_timer;
	Timer* m_launcherTimer;
	
	Joystick* m_driver;
	
	DriverStationLCD* m_dsLCD;
	
public:
	BuiltinDefaultCode()	{
			m_plate1 = new Victor(8);
			m_plate2 = new Victor(10);
			m_launcher1 = new Victor(4);
			m_launcher2 = new Victor(5);
			m_launcher3 = new Victor(6);
			m_climbLight = new Relay(2);
			
			m_timer = new Timer;
			m_launcherTimer = new Timer;
			
			m_driver = new Joystick(1);
			
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
		m_launcherTimer->Start();
		m_launcherTimer->Stop();
		m_launcherTimer->Reset();
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic()  {

	}

	void AutonomousPeriodic() {

	}

	
	void TeleopPeriodic() {
		// ----- Plate -----
		m_plate1->Set(m_driver->GetRawAxis(LEFT_Y));
		m_plate2->Set(m_driver->GetRawAxis(LEFT_Y));
		
		// ----- Launcher -----
		if(m_driver->GetRawButton(BUTTON_A) && m_launcherTimer->Get() == 0){
			// Start Launcher
			// Start Timer
			m_launcherTimer->Stop();
			m_launcherTimer->Reset();
			m_launcherTimer->Start();
		}
		if(m_launcherTimer->Get() > 2){
			// After 2 sec
			m_launcher1->Set(1);
			m_launcher2->Set(1);
			m_launcher3->Set(1);
		} else {
			// During 2 sec after starting
			
			m_launcher1->Set((float)m_launcherTimer->Get()/2);
			m_launcher2->Set((float)m_launcherTimer->Get()/2);
			m_launcher3->Set((float)m_launcherTimer->Get()/2);
		}
		if(m_driver->GetRawButton(BUTTON_B)) {
			// End Launcher
			m_launcher1->Set(0.0);
			m_launcher2->Set(0.0);
			m_launcher3->Set(0.0);
			// End Timer
			m_launcherTimer->Start();
			m_launcherTimer->Stop();
			m_launcherTimer->Reset();
		}
		
		// ----- Climb Indicator -----
		if(m_driver->GetRawButton(BUTTON_X))
			m_climbLight->Set(Relay::kForward);
		else
			m_climbLight->Set(Relay::kOff);
	}	
};

START_ROBOT_CLASS(BuiltinDefaultCode);
