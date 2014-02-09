#include "WPILib.h"
#include "JoystickWrapper.h"
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
private:
	// ----- Drive -----
	// Drive motors
	Talon* m_lDrive1; //Two motors
	Talon* m_rDrive1; //One motor
	Talon* m_lDrive2; //Two motors
	Talon* m_rDrive2; //One motor
	
	// Drive objects
	DriveWrapper* m_rDrive;
	DriveWrapper* m_lDrive;
	RobotDrive* m_robotDrive;
	
	// Drive encoders
	Encoder* m_lEncode;
	Encoder* m_rEncode;
	
	// Shifters
	Solenoid *m_shifters;
	
	// ----- Arm -----
	// Arm motors
	Talon* m_rArm;			// PWM 6
	Talon* m_lArm;			// PWM 7
	
	// Arm encoder
	Encoder* m_armAngle;	// Digital Input 5, 6
	
	// ----- Ramrod -----
	// Ramrod motor
	Talon *m_ramMotor;
	
	// Ramrod servo
	Servo *m_ramServo;
	
	// Ramrod encoder
	Encoder *m_ramEncoder;
	
	// ----- bGrabber -----
	// bGrabber motor
	Talon* m_roller;

	// bGrabber solenoids
	Solenoid* m_catch;
	Solenoid* m_bArm;

	// ----- Compressor -----
	Compressor* m_compressor;

	// ----- General -----
	// Joysticks
	JoystickWrapper* m_driver;
	JoystickWrapper* m_operator;

	// Driver station
	DriverStationLCD* m_dsLCD;

	// Counter
	int teleopCounter, autonCounter, disabledCounter, testCounter;
	
public:

/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */

	BuiltinDefaultCode()	{
		// ----- Drive -----
		// Drive controllers
		m_rDrive1 = new Talon (1);
		m_rDrive2 = new Talon (2);
		m_lDrive1 = new Talon (3);
		m_lDrive2 = new Talon (4);
		
		// Drive wrappers
		m_rDrive = new DriveWrapper (m_rDrive1, m_rDrive2);
		m_lDrive = new DriveWrapper (m_lDrive1, m_lDrive2);
		
		// Robot drive
		m_robotDrive = new RobotDrive (m_lDrive, m_rDrive);
		
		// Shifters
		m_shifters = new Solenoid(1);
		
		// Drive encoders
		m_rEncode = new Encoder (1,2,true);
		m_rEncode->SetDistancePerPulse(1);
		m_rEncode->SetMaxPeriod(1.0);
		m_rEncode->Start();
		m_lEncode = new Encoder (3,4,false);
		m_lEncode->SetDistancePerPulse(1);
		m_lEncode->SetMaxPeriod(1.0);
		m_lEncode->Start();
		
		// ----- Arm -----
		// Arm Moter Controller
		m_rArm = new Talon (6);
		m_lArm = new Talon (7);
		
		// Arm encoder
		m_armAngle = new Encoder (5, 6, true);
		m_armAngle->SetDistancePerPulse(1);
		m_armAngle->SetMaxPeriod(1.0);
		m_armAngle->Start();
		
		// ----- Ramrod -----
		// Ramrod motor
		m_ramMotor = new Talon (5);
		
		// Ramrod servo
		m_ramServo = new Servo (9);
		
		// Ramrod encoder
		m_ramEncoder = new Encoder (7,8,false);
		m_ramEncoder->SetDistancePerPulse(1);
		m_ramEncoder->SetMaxPeriod(1.0);
		m_ramEncoder->Start();
		
		// ----- bGrabber -----
		// bGrabber motor
		m_roller = new Talon (8);
		
		// bGrabber Solenoids
		m_bArm = new Solenoid (2);
		m_catch = new Solenoid (3);
		
		
		// ----- Compressor -----
		m_compressor = new Compressor(9, 1);

		
		// ----- General -----
		// Joysticks
		m_driver = new JoystickWrapper (1);
		m_operator = new JoystickWrapper (2);

		// Driver station object
		m_dsLCD = DriverStationLCD::GetInstance();		

		// Counter
		teleopCounter = 0;
		autonCounter = 0;
		disabledCounter = 0;
		testCounter = 0;
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

	void TestInit () {

	}

	/********************************** Periodic Routines *************************************/
	void DisabledPeriodic()  {

	}

	void AutonomousPeriodic() {

	}


	void TeleopPeriodic() {
		
	} // TeleopPeriodic()

	void TestPeriodic () {
		ManageCompressor();
		TestArm();
		TestDrive();
		TestBGrabber();
		TestRamMotion();
		TestRamLock();
	}

/********************************** Miscellaneous Routines *************************************/

	/*********************** TELEOP FUNCTIONS **************************/

	void TeleopDrive()
	{
		// Drive
		m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),-m_driver->GetRawAxis(RIGHT_X));
		
		// Shifter
		if (m_driver -> GetRawButton(BUTTON_LB)){
			m_shifters -> Set(true);
		}
		else {
			m_shifters -> Set(false);
		}
	}

	/*************************** TEST FUNCTIONS *****************************/

	void TestDrive()
	{
		// Drive
		m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),-m_driver->GetRawAxis(RIGHT_X));

		// Shifter
		if (m_driver -> GetRawButton(BUTTON_A)){
			m_shifters -> Set(true);
		}
		else {
			m_shifters -> Set(false);
		}
	}

	void TestArm ()
	{
		// Control Arm
		m_lArm->Set(-m_operator->GetRawAxis(LEFT_Y) * 0.5);
		m_rArm->Set(m_operator->GetRawAxis(LEFT_Y) * 0.5);

		// Reset Arm Encoder
		if (m_operator->GetRawButton(BUTTON_L3)) {
			m_armAngle->Reset();
		}
	}

	void TestBGrabber()
	{
		// ROLLERS	
		if (m_operator->GetRawAxis(TRIGGERS) > 0.4) {
			m_roller->Set(1.0);
		}
		else if (m_operator->GetRawAxis(TRIGGERS) < -0.4)
			m_roller->Set(-1.0);
		else {
			m_roller->Set(0.0);
		}	

		// BALL CATCH (#Sweg)
		if (m_operator->GetRawButton(BUTTON_A)) {
			m_catch->Set(true); 
		}
		else if (m_operator->GetRawButton(BUTTON_B)) {
			m_catch->Set(false);
		}

		// bArm OPEN / CLOSE
		if (m_operator->GetRawButton(BUTTON_X)) {
			m_bArm->Set(true);
		}
		else if (m_operator->GetRawButton(BUTTON_Y)) {
			m_bArm->Set(false);
		}
	}

	void TestRamrod ()
	{
		if (m_driver->GetRawButton(BUTTON_LB))
			// IN
			m_ramMotor->Set(.8);
		else if (m_driver->GetRawButton(BUTTON_RB))
			// OUT
			m_ramMotor->Set(-.2);
		else
			m_ramMotor->Set(0);
		
		if (m_driver->GetRawAxis(TRIGGERS) < -.2) {
			m_ramServo->SetAngle(120);
		} else {
			m_ramServo->SetAngle(0);
		}
	}


	/************** UNIVERSAL FUNCTIONS ***************/

	void ManageCompressor () {
		if (m_compressor->GetPressureSwitchValue()) {
			m_compressor->Stop();
		} else {
			m_compressor->Start();
		}
	}
	void AutonStraighDrive(){
		if (m_lEncode -> GetDistance() > 200 && m_rEncode -> GetDistance() > 200){
			m_robotDrive->TankDrive(.8 + ((m_rEncode -> GetRate()) - (m_lEncode -> GetRate())), .8 + ((m_lEncode -> GetRate()) - (m_rEncode -> GetRate())));
		}
	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);
