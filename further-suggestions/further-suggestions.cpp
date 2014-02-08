#include "WPILib.h"
#include "DriveWrapper.h"
#include "Defines.h"
#include <cmath>

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
	
		
public:
	
	//Declare drive motors
	Talon* m_lDrive1; //Two motors
	Talon* m_rDrive1; //One motor
	Talon* m_lDrive2; //Two motors
	Talon* m_rDrive2; //One motor
	
	//Declare drive objects
	DriveWrapper* m_rDrive;
	DriveWrapper* m_lDrive;
	RobotDrive* m_robotDrive;
	
	//Declare joysticks
	Joystick* m_driver;
	Joystick* m_operator;
	
	//Declare driver station
	DriverStationLCD* m_dsLCD;
	
	//Declare bGrabber solenoids
	Solenoid* m_catch;
	Solenoid* m_bArm;
	
	//Declare bGrabber PWM
	Talon* m_roller;
	
	
	
	
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	
	BuiltinDefaultCode()	{
		//Initialze drive controllers
		m_rDrive1 = new Talon (1);
		m_rDrive2 = new Talon (2);
		m_lDrive1 = new Talon (3);
		m_lDrive2 = new Talon (4);
		
		//Initialize drive wrappers
		m_rDrive = new DriveWrapper (m_rDrive1, m_rDrive2);
		m_lDrive = new DriveWrapper (m_lDrive1, m_lDrive2);
		
		//Initialize robot drive
		m_robotDrive = new RobotDrive (m_lDrive, m_rDrive);
		
		//Initialize joysticks
		m_driver = new Joystick (1);
		m_operator = new Joystick (2);
		
		//Grab driver station object
		m_dsLCD = DriverStationLCD::GetInstance();
		
		//Initialize bGrabber Solenoids
		m_catch = new Solenoid (3);
		m_bArm = new Solenoid (2);
		
		//initialize bGrabber PWM
		m_roller = new Talon (8);
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
	  TeleopDrive();
	} // TeleopPeriodic()

/********************************** Miscellaneous Routines *************************************/
	
	void TeleopDrive()
	{
		if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.2 || fabs(m_driver->GetRawAxis(RIGHT_X)) > 0.2)
			m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),-m_driver->GetRawAxis(RIGHT_X));
	}
	
	void TestbGrabber()
	{
		if (m_operator->GetRawAxis(TRIGGERS) > 0.4) {
			m_roller->Set(0.75);
		}
		else if (m_operator->GetRawAxis(TRIGGERS) > 0.4) {
			m_roller->Set(-0.75);
		}
		else; {
			m_roller->Set(0.0);
		}	
		if (m_operator->GetRawButton(BUTTON_A)) {
			m_catch->Set(true); 
		}
		else if (m_operator->GetRawButton(BUTTON_B)) {
			m_catch->Set(false);
		}
		
	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);
