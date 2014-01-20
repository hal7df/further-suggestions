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

#include "WPILib.h"
#include "Defines.h"
#include "PlateControl.h"

float deadband (float input)
{
	if(input < 0.2 && input > -0.2) {
		return 0.0;
	} else {
		return input;
	}
}

class BuiltinDefaultCode : public IterativeRobot
{
	// Plate
	Victor* m_plate1;
	Victor* m_plate2;
	AnalogChannel* m_plateSensor;
	
	// Controler
	PIDController* m_PIDController;
	PlateControl* m_platePID;
	Joystick* m_driver;
	
	// PID MODE
	bool pid;
	float plateSetPoint;
	float plateSetPointCoeff;
	Timer* plateTimer;
	
public:
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	BuiltinDefaultCode()	{
		// Plate
		m_plate1 = new Victor(8);
		m_plate2 = new Victor(10);
		m_plateSensor = new AnalogChannel (1);
		
		// Controler
		m_platePID = new PlateControl (m_plate1, m_plate2);
		m_PIDController = new PIDController (-0.025,0,0, m_plateSensor, m_platePID);
		m_driver = new Joystick(1);
		
		// PID MODE
		
		pid = false;
		plateSetPointCoeff = 0.1;
		plateTimer = new Timer;
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
		SmartDashboard::PutNumber("TRIGGER VALUE:", m_driver->GetRawAxis(TRIGGERS));
		
	}

	void AutonomousPeriodic() {
	  
	}

	
	void TeleopPeriodic() {
		if(m_driver->GetRawButton(BUTTON_LB)) {
			// PYRAMID
			m_PIDController->SetSetpoint(PLATE_PYRAMID_THREE_POINT);
			m_PIDController->Enable();
		} else if(m_driver->GetRawButton(BUTTON_RB)) {
			// FEEDER
			m_PIDController->SetSetpoint(PLATE_FEEDER_THREE_POINT);
			m_PIDController->Enable();
		} else if(m_driver->GetRawAxis(TRIGGERS) > 0.5) {
			m_PIDController->SetSetpoint(PLATE_TEN_POINT_CLIMB);
			m_PIDController->Enable();
		} else {
			// MANUAL CONTROL
			m_PIDController->Disable();
			
			m_plate1->Set(-deadband(m_driver->GetRawAxis(LEFT_Y)));
			m_plate2->Set(-deadband(m_driver->GetRawAxis(LEFT_Y)));
		}
		
		// ----- PRINT -----
		SmartDashboard::PutNumber("Plate Position: ", m_plateSensor->GetVoltage());
		SmartDashboard::PutNumber("PID GET: ", m_plateSensor->PIDGet());
		
	} // TeleopPeriodic()

/********************************** Miscellaneous Routines *************************************/
	
};

START_ROBOT_CLASS(BuiltinDefaultCode);
