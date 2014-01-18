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
	// ---- Components -----
	Victor* m_lDrive;
	Victor* m_rDrive;
	
	Victor* m_plate1;
	Victor* m_plate2;
	
	Victor* m_launcher1;
	Victor* m_launcher2;
	Victor* m_launcher3;
	
	Timer* m_launcherTimer;
	
	RobotDrive* m_robotDrive;
	
	Joystick* m_driver;
	
	// ----- MODE -----
	bool mode;
		
public:
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	BuiltinDefaultCode()	{
		m_lDrive = new Victor(1);
		m_rDrive = new Victor(2);
		/*
		m_plate1 = new Victor(8);
		m_plate2 = new Victor(10);
		
		m_launcher1 = new Victor(4);
		m_launcher2 = new Victor(5);
		m_launcher3 = new Victor(6);
		
		m_launcherTimer = new Timer;
		*/
		m_robotDrive = new RobotDrive(m_lDrive,m_rDrive);
		m_robotDrive->SetSafetyEnabled(false);
		
		m_driver = new Joystick(1);
	}
	
	
	/********************************** Init Routines *************************************/


	void RobotInit() {
	  
	}
	
	void DisabledInit() {
	  
	}

	void AutonomousInit() {
	  
	}

	void TeleopInit() {
		/*
		m_launcherTimer->Start();
		m_launcherTimer->Stop();
		m_launcherTimer->Reset();
		*/
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic()  {
	  
	}

	void AutonomousPeriodic() {
	  
	}

	
	void TeleopPeriodic() {
		// ----- Control -----
		// Drive
		m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),-m_driver->GetRawAxis(RIGHT_X));
		/*
		// Launcher
		if(m_launcher->Get() < 2) {
			m_launcher1->Set(m_launcherTimer->Get()/2);
			m_launcher2->Set(m_launcherTimer->Get()/2);
			m_launcher3->Set(m_launcherTimer->Get()/2);
		} else {
			m_launcher1->Set(1.0);
			m_launcher2->Set(1.0);
			m_launcher3->Set(1.0);
		}
		
		// Start and Stop
		if(m_driver->GetRawButton(BUTTON_A) && m_launcherTimer->Get() != 0) {
			// Start Timer
			m_launcherTimer->Start();
		}
		if(m_driver->GetRawButton(BUTTON_B)) {
			m_launcherTimer->Stop();
			m_launcherTimer->Reset();
		}
		
		// Plate
		m_plate1->Set(-m_driver->GetRawAxis(RIGHT_Y));
		m_plate2->Set(-m_driver->GetRawAxis(RIGHT_Y));
		*/
	} // TeleopPeriodic()

/********************************** Miscellaneous Routines *************************************/
	
};

START_ROBOT_CLASS(BuiltinDefaultCode);
