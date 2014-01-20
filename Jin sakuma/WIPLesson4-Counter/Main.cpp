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
	// Declare Components of the robot
	Joystick* m_driver;			// Driver
	Timer* m_timer;	// Timer to sleep btw push
	DriverStationLCD* m_dsLCD;	// Driver Station
	float m_buttonCounter;		// Button Counter
	float m_stickCounter;		// Stick Counter
	float sleeptime;			// Sleep time bwt push
	
		
public:

	BuiltinDefaultCode()	{
		// Constructor.
		// Used for mapping and initializing each components
		
		m_driver = new Joystick(1);
		m_timer = new Timer;
		m_dsLCD = DriverStationLCD::GetInstance();
		m_buttonCounter = 0;
		m_stickCounter = 0;
		sleeptime = 0.1;
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
		float left_x, right_x;
		
		if(m_timer->HasPeriodPassed(sleeptime) || m_timer->Get() == 0) {
			// BUTTON
			if(m_driver->GetRawButton(BUTTON_A)) {
				m_buttonCounter++;
				
				// Timer set
				m_timer->Stop();
				m_timer->Start();
				m_timer->Reset();
			}
			if(m_driver->GetRawButton(BUTTON_B)) {
				m_buttonCounter--;
				
				// Timer set
				m_timer->Stop();
				m_timer->Start();
				m_timer->Reset();
			}
			
			// AXIS
			left_x = m_driver->GetRawAxis(LEFT_X);
			right_x = m_driver->GetRawAxis(RIGHT_X);
			if(left_x)
			{
				m_stickCounter -= (int) (left_x*5);
				
				// Timer set
				m_timer->Stop();
				m_timer->Start();
				m_timer->Reset();
			}
			
			// Change Sleeping Time
			if(m_driver->GetRawButton(BUTTON_Y)){
				sleeptime += 0.1;
				
				// Timer set
				m_timer->Stop();
				m_timer->Start();
				m_timer->Reset();
			}
			if(m_driver->GetRawButton(BUTTON_X)){
				sleeptime -= 0.1;
				
				// Timer set
				m_timer->Stop();
				m_timer->Start();
				m_timer->Reset();
			}
		}
		
		// Display
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"BUTTON COUNTER: %f",m_buttonCounter);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"STICK COUNTER: %f",m_stickCounter);
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3,1,"SLEEP TIME: %f",sleeptime);
		m_dsLCD->UpdateLCD();
	}
	
	/********************************** Continuous Routines *************************************/
	
	void DisabledContinuous() {
		
	}
	
	void AutonomousContinuous() {
		
	}
	
	void TeleopContinuous() {

	}

/********************************** Miscellaneous Routines *************************************/
				
};

START_ROBOT_CLASS(BuiltinDefaultCode);
