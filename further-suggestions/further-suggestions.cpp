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
	//Declare drive motors
	Talon* m_lDrive1; //Two motors
	Talon* m_rDrive1; //One motor
	Talon* m_lDrive2; //Two motors
	Talon* m_rDrive2; //One motor

	//Drivetrain Encodes
	Encoder *m_rEncode;
	Encoder *m_lEncode;

	//Declare arm
	ArmWrapper* m_arm;
	
	//Declare ramrod motor
	Talon *m_ramMotor;
	
	//Declare bGrabber motor
	Talon* m_roller;
	
	//Declare ramrod servo
	Servo *m_ramServo;
	
	//Declare drive objects
	DriveWrapper* m_rDrive;
	DriveWrapper* m_lDrive;
	RobotDrive* m_robotDrive;
	
	//Declare Compressor
	Compressor* m_compressor;
	
	// shifters
	Solenoid *m_shifters;
	
	//Declare bGrabber solenoids
	Solenoid* m_catch;
	Solenoid* m_bArm;

	
	//Declare ramrod encoder
	Encoder *m_ramEncoder;
	
	//Declare joysticks
	JoystickWrapper* m_driver;
	JoystickWrapper* m_operator;
	
	//Declare driver station
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
		//Initialze drive controllers
		m_rDrive1 = new Talon (1);
		m_rDrive2 = new Talon (2);
		m_lDrive1 = new Talon (3);
		m_lDrive2 = new Talon (4);
		
		//Drive encoders
		m_rEncode = new Encoder (1,2,true);
			m_rEncode->SetDistancePerPulse(1);
			m_rEncode->SetMaxPeriod(1.0);
			m_rEncode->Start();
		m_lEncode = new Encoder (3,4,false);
			m_lEncode->SetDistancePerPulse(1);
			m_lEncode->SetMaxPeriod(1.0);
			m_lEncode->Start();
	
		
		//Initialize ramrod motor
		m_ramMotor = new Talon (5);
		
		//Initialize Arm
		m_arm = new ArmWrapper (7, 6, 5, 6, 10);
		m_arm->StartPID(0.0, 0.0, 0.0);
		
		//initialize bGrabber motor
		m_roller = new Talon (8);
		
		//Initialize ramrod servo
		m_ramServo = new Servo (9);
		
		//Initialize drive wrappers
		m_rDrive = new DriveWrapper (m_rDrive1, m_rDrive2);
		m_lDrive = new DriveWrapper (m_lDrive1, m_lDrive2);
		
		//Initialize robot drive
		m_robotDrive = new RobotDrive (m_lDrive, m_rDrive);
		
		//Initialize ramrod encoder
		m_ramEncoder = new Encoder (7,8,false);
		m_ramEncoder->SetDistancePerPulse(1);
		m_ramEncoder->SetMaxPeriod(1.0);
		m_ramEncoder->Start();
		
		//Initialize Compressor
		m_compressor = new Compressor(9, 1);
		
		//shifters
		m_shifters = new Solenoid(1);
		
		//Initialize bGrabber Solenoids
		m_bArm = new Solenoid (2);
		m_catch = new Solenoid (3);

		//Initialize joysticks
		m_driver = new JoystickWrapper (1);
		m_operator = new JoystickWrapper (2);
		
		//Grab driver station object
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
		ManageCompressor();
		TeleopDrive();
		TeleopArm();
		TeleopBGrabber();
		TeleopRanrod();
		
		teleopCounter++;
	}
	
	void TestPeriodic () {
		ManageCompressor();
		TestArm();
		TestDrive();
		TestBGrabber();
		TestRamMotion();
		TestRamLock();
		
		testCounter++;
	}

/********************************** Miscellaneous Routines *************************************/
	
	/*********************** TELEOP FUNCTIONS **************************/
	
	void TeleopDrive()
	{
		if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.2 || fabs(m_driver->GetRawAxis(RIGHT_X)) > 0.2)
			m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),-m_driver->GetRawAxis(RIGHT_X));
		else
			m_robotDrive->ArcadeDrive(0.0,0.0);
		if (m_driver -> GetRawButton(BUTTON_LB)){
			m_shifters -> Set(true);
		}
		else {
			m_shifters -> Set(false);
		}
	}
	
	void TeleopBGrabber()
	{
		//ROLLERS	
		if (m_operator->GetRawAxis(TRIGGERS) > 0.4) {
			m_roller->Set(1);
		}
		else if (m_operator->GetRawAxis(TRIGGERS) < -0.4)
			m_roller->Set(1);
		else {
			m_roller->Set(0.0);
		}	
		
		//BALL CATCH (#Sweg)
		if (m_operator->GetRawButton(BUTTON_LB)) {
			m_catch->Set(true); 
		}
		else {
			m_catch->Set(false);
		}
		
		//bArm OPEN / CLOSE
		if (m_operator->GetRawButton(BUTTON_X)) {
			m_bArm->Set(true);
		}
		else if (m_operator->GetRawButton(BUTTON_Y)) {
			m_bArm->Set(false);
		}
	}
	
	void TeleopArm ()
	{
		// ----- PID -----
		if (m_operator->GetRawButton(BUTTON_A)) {
			// Floor Picking
			m_arm->SetAngle (FLOOR_PICKING_POS);
			m_arm->PIDEnable();
			
		} else if (m_operator->GetRawButton(BUTTON_B)) {
			// Medium (12ft) Shoot Position
			m_arm->SetAngle (MED_SHOOT_POS);
			m_arm->PIDEnable();
			
		} else if (m_operator->GetRawButton(BUTTON_X)) {
			// Long (18ft) Shoot Position
			m_arm->SetAngle(LONG_SHOOT_POS);
			m_arm->PIDEnable();
			
		} else if (m_operator->GetRawButton(BUTTON_Y)) {
			// Catch Position
			m_arm->SetAngle(CATCH_POS);
			m_arm->PIDEnable();
			
		} else {
			m_arm->PIDDisable();
			m_arm->PIDEnable();
			
			// Control With Joystick
			m_arm->Set(m_operator->GetRawAxis(LEFT_Y));
		}
		
		// Reset Arm
		if (m_arm->GetLimSwitch()) {
			m_arm->Reset();
		}
	}
		  
	
	/*************************** TEST FUNCTIONS *****************************/
	
	void TestDrive(){
		
		if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.2 || fabs(m_driver->GetRawAxis(RIGHT_X)) > 0.2)
			m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),-m_driver->GetRawAxis(RIGHT_X));
		else
			m_robotDrive->ArcadeDrive(0.0,0.0);
		if (m_driver -> GetRawButton(BUTTON_A)){
			m_shifters -> Set(true);
		}
		else {
			m_shifters -> Set(false);
		}
		
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"Right encoder count: %d",m_rEncode->Get());
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3,1,"Left encoder count: %d",m_lEncode->Get());
	}
	
	void TestArm ()
	{
		// Control Arm
		m_arm->Set(m_operator->GetRawAxis(LEFT_Y));

		// Reset Arm Encoder
		if (m_operator->GetRawButton(BUTTON_L3)) {
			m_arm->Reset();
		}
	}
	
	void TestBGrabber()
	{
		// Doesn't make sense
		//ROLLERS	
		if (m_operator->GetRawAxis(TRIGGERS) > 0.4) {
			m_roller->Set(1.0);
		}
		else if (m_operator->GetRawAxis(TRIGGERS) < -0.4)
			m_roller->Set(-1.0);
		else {
			m_roller->Set(0.0);
		}	
		
		//BALL CATCH (#Sweg)
		if (m_operator->GetRawButton(BUTTON_A)) {
			m_catch->Set(true); 
		}
		else if (m_operator->GetRawButton(BUTTON_B)) {
			m_catch->Set(false);
		}
		
		//bArm OPEN / CLOSE
		if (m_operator->GetRawButton(BUTTON_X)) {
			m_bArm->Set(true);
		}
		else if (m_operator->GetRawButton(BUTTON_Y)) {
			m_bArm->Set(false);
		}
	}
	
	void TestRamMotion()
	{
		if (m_driver->GetRawButton(BUTTON_LB))
			// IN
			m_ramMotor->Set(.8);
		else if (m_driver->GetRawButton(BUTTON_RB))
			// OUT
			m_ramMotor->Set(-.2);
		else
			m_ramMotor->Set(0);
	}
	
	void TestRamLock()
	{
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
	void Auton360s(){
		m_robotDrive->TankDrive( -1.0, 1.0);
	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);
