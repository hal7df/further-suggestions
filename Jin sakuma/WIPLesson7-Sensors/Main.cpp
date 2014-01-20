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
	// ----- Cmoponents -----
	// Moters
	Victor* m_lDrive;
	Victor* m_rDrive;
	Victor* m_climber;
	Relay* m_climbRachet;
	Victor* m_feeder;
	Victor* m_plate1;
	Victor* m_plate2;
	
	RobotDrive* m_robotDrive;
	
	// Sensors
	Encoder* m_lDriveEncoder;
	Encoder* m_rDriveEncoder;
	AnalogChannel* m_plateSensor;
	
	// Controler
	Joystick* m_driver;
	Joystick* m_operator;
	
	
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
		m_climber = new Victor(3);
		m_climbRachet = new Relay(1);
		m_feeder = new Victor(7);
		m_plate1 = new Victor(8);
		m_plate2 = new Victor(10);
		
		m_robotDrive = new RobotDrive(m_lDrive, m_rDrive);
		
		m_lDriveEncoder = new Encoder(1,2,true);
		m_rDriveEncoder = new Encoder(3,4,false);
		m_plateSensor = new AnalogChannel(1);
		
		m_driver = new Joystick(1);
		m_operator = new Joystick(2);
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
		// ----- Display Values -----
		// Driver
		SmartDashboard::PutNumber("Left Drive: ",(double)m_lDrive->Get());
		SmartDashboard::PutNumber("Right Drive: ", (double)m_rDrive->Get());
		
		// Climber
		SmartDashboard::PutNumber("Climber: ", (double)m_climber->Get());
		if(m_climbRachet->Get() == m_climbRachet->kForward || m_climbRachet->Get() == m_climbRachet->kReverse) {
			SmartDashboard::PutBoolean("Climb Rachet: ", true);
		} else {
			SmartDashboard::PutBoolean("Climb Rachet: ", false);
		}
		
		
		// Feeder
		SmartDashboard::PutNumber("Feeder: ", (double)m_feeder->Get());
		
		// Plate
		SmartDashboard::PutNumber("Plate1: ", (double)m_plate1->Get());
		SmartDashboard::PutNumber("Plate2: ", (double)m_plate2->Get());
		SmartDashboard::PutNumber("Plate Sensor: ", (double)m_plateSensor->GetVoltage());
		
		// Encoder
		SmartDashboard::PutNumber("Left Encoder: ", m_lDriveEncoder->GetDistance());
		SmartDashboard::PutNumber("Right Encoder: ", m_rDriveEncoder->GetDistance());
		
		// ----- Control Robot -----
		// Drive
		m_robotDrive->ArcadeDrive(m_driver->GetRawAxis(LEFT_Y), m_driver->GetRawAxis(RIGHT_X));
		
		// Plate
		m_plate1->Set(m_driver->GetRawAxis(TRIGGERS));
		m_plate2->Set(m_driver->GetRawAxis(TRIGGERS));
		
		
	} // TeleopPeriodic()

/********************************** Miscellaneous Routines *************************************/
	
};

START_ROBOT_CLASS(BuiltinDefaultCode);
