#include "WPILib.h"
#include "JoystickWrapper.h"
#include "DriveWrapper.h"
#include "ArmWrapper.h"
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
	bool Drive_Status;

	//Declare arm
	Talon* m_armMotor;
	Encoder* m_armEncoder;
	PIDController* m_armPID;
	bool m_armPIDFlag;
	// igitalInput* m_armLimSwitch;
	// ArmWrapper* m_arm;
	// Encoder* m_armEncoder;
	
	//Declare ramrod motor
	Talon *m_ramMotor;
	
	//Declare bGrabber motor
	Talon* m_roller;
	
	//Declare ramrod servo
	Servo *m_ramServo;
	
	//Declare catch arm stop servos
	Servo *m_catchServo1;
	Servo *m_catchServo2;
	
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
	
	//Declare arm encoder
	//Encoder* m_armAngle;	// Digital Input 5, 6
	
	//Declare ramrod encoder
	Encoder *m_ramEncode;
	
	//Declare joysticks
	JoystickWrapper* m_driver;
	JoystickWrapper* m_operator;
	
	//Declare driver station
	DriverStationLCD* m_dsLCD;
	
	//Timers
	Timer *m_ramTime;
	Timer *m_bGrabberTime;
	Timer *m_autonTime;
	
	// Counter
	int countLoop;
	int armCount;
	int autonCount;
	int ramFire;
	int m_medRamCase;
	int m_ramCase;
	bool m_ramInit;
	
	//Auton
	enum AutonChoice {
		AutonDBrebound, AutonDFshoot, AutonCheckHotleft, AutonCheckHotright, AutonDoNothing, AutonDf
	} autonChoice;
	
	// Auton Steps
	int AutonDBSteps;
	/*enum AutonDBSteps {
		DF1=1, ShootAngle1=2, DriveBack=3, DriveBack2=4,ShootAngle2=5, DF2=6
	} AutonDBSteps;
		*/	
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
		Drive_Status = false;
		
		//Initialize ramrod motor
		m_ramMotor = new Talon (5);
		
		//Initialize Arm
		m_armMotor = new Talon (6);
		m_armEncoder = new Encoder (5, 6, false);
		m_armEncoder->SetDistancePerPulse(1.0);
		m_armEncoder->SetMaxPeriod(1.0);
		m_armEncoder->Start();
		m_armPID = new PIDController(ARM_P, ARM_I, ARM_D, m_armEncoder, m_armMotor);
		
		// m_arm = new ArmWrapper (6, 8, 5, 6, 10);
		//m_arm->StartPID(0.0, 0.0, 0.0);
		/*
		m_armEncoder = new Encoder (5,6,true);
		m_armEncoder->SetDistancePerPulse(1.0);
		m_armEncoder->SetMaxPeriod(1.0);
		m_armEncoder->Start();
		*/
		
		//initialize bGrabber motor
		m_roller = new Talon (7);
		
		//Initialize ramrod servo
		m_ramServo = new Servo (10);
		
		#ifdef CompetitionBot
		//Initialize catch servos
		m_catchServo1 = new Servo (8);
		m_catchServo2 = new Servo (9);
		#endif
		
		//Initialize drive wrappers
		m_rDrive = new DriveWrapper (m_rDrive1, m_rDrive2);
		m_lDrive = new DriveWrapper (m_lDrive1, m_lDrive2);
		
		//Initialize robot drive
		m_robotDrive = new RobotDrive (m_lDrive, m_rDrive);
		m_robotDrive->SetSafetyEnabled(false);
		
		//Initialize ramrod encoder
		m_ramEncode = new Encoder (7,8,true);
		m_ramEncode->SetDistancePerPulse(1);
		m_ramEncode->SetMaxPeriod(1.0);
		m_ramEncode->Start();

		//Initialize arm encoder
		
		
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
		
		//Timers
		m_ramTime = new Timer;
		m_bGrabberTime = new Timer;
		m_autonTime = new Timer;
		m_ramCase = -1;
		m_medRamCase = -1;
		countLoop = 0;
		armCount = 0;
		
		// Auton Steps
		AutonDBSteps = 1;
	}
	
	void AutonDBRebound(){
		RamFire();
		switch(AutonDBSteps) {
		case 1:
			
			m_armPID->SetSetpoint(LONG_SHOOT_POS);
			m_armPID->Enable();
			AutonStraightDrive(-0.7,-32 * REV_IN);
			if (Drive_Status){
				AutonDBSteps = 2;
			}
			break;
			
		case 2:
			SmartDashboard::PutNumber("Arm Difference", fabs(m_armEncoder->GetDistance() - LONG_SHOOT_POS));
			if (fabs(m_armEncoder->GetDistance() - LONG_SHOOT_POS) < AUTON_ANGLE_GAP) {
				if (m_ramCase == -1)
				{
					m_ramCase = 0;
					//m_ramCase = 5;
				}
			}
			
			if (m_ramCase == 5){
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				Drive_Status = false;
				AutonDBSteps = 3;
			}
			break;

		case 3:	
			m_armPID->SetSetpoint(FLOOR_PICKING_POS);
			m_armPID->Enable();
			AutonStraightDrive(0.7,30 * REV_IN);
			m_roller->Set(-1.0);
			SmartDashboard::PutNumber("Arm Difference", fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS));
			SmartDashboard::PutBoolean("Arm Difference Bool", fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS) < AUTON_ANGLE_GAP);
						
			if ((fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS) < AUTON_ANGLE_GAP) || (m_autonTime->HasPeriodPassed(2.0))){
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_autonTime->Stop();
				Drive_Status = false;
				AutonDBSteps = 4;
			}
			break;
		case 4:	
			AutonStraightDrive(0.7,24 * REV_IN);
			SmartDashboard::PutNumber("Arm Difference", fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS));
			SmartDashboard::PutBoolean("Arm Difference Bool", fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS) < AUTON_ANGLE_GAP);
						
			if(Drive_Status){
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				AutonDBSteps = 5;
				Drive_Status = false;
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
			}
			break;	
		
		case 5:
			m_armPID->SetSetpoint(LONG_SHOOT_POS);
			m_armPID->Enable();
			SmartDashboard::PutBoolean("Auton Time: ", m_autonTime->Get());
			if(m_autonTime->Get() > 1)
			{
				AutonStraightDrive(-0.7, -55 * REV_IN);
				m_autonTime->Stop();
			}
			if (Drive_Status){
				AutonDBSteps = 6;
				m_robotDrive->TankDrive(0.,0.);
				m_rEncode -> Reset();
				m_lEncode -> Reset();
			}
			break;
			
		case 6:
			SmartDashboard::PutNumber("Arm Difference", fabs(m_armEncoder->GetDistance() - LONG_SHOOT_POS));
			if (fabs(m_armEncoder->GetDistance() - LONG_SHOOT_POS) < AUTON_ANGLE_GAP) {
				if (m_ramCase == -1)
				{
					m_roller->Set(0.0);
					m_ramCase = 0;
					AutonDBSteps = 7;
					//m_ramCase = 5;
				}
				
			}
			break;
		case 7:
			break;
		}
	}
	/*
	void AutonDFShoot(){
		RamFire();
		AutonStraightDrive(1,200);
		if (Drive_Status){
			m_arm->SetAngle(LONG_SHOOT_POS);
			if (m_arm->GetAngle() > LONG_SHOOT_POS - AUTON_ANGLE_GAP && m_arm->GetAngle() < LONG_SHOOT_POS + AUTON_ANGLE_GAP) {
				m_ramCase = 0;
			}
		}
	}
/*
	void AutonCheckHotLeft(){
		AutonStraightDrive(200);
		RamFire();
		if (Drive_Status){
			if (m_cameraHandler->getHotGoal() == state_t::kLeft){
				m_ramCase = 0;		
			}
			Drive_Status = false;
		}
	}
	void AutonCheckHotRight(){
		AutonStraightDrive(200);
		RamFire();
		if (Drive_Status){
			if (m_cameraHandler->getHotGoal() == state_t::kRight){
				m_ramCase = 0;		
			}
			Drive_Status = false;
		}
	}

	void AutonDF(){
		AutonStraightDrive(1,200);
	}
	/********************************** Init Routines *************************************/


	void RobotInit() {
	  
	}
	
	void DisabledInit() {
		
	}

	void AutonomousInit() {
		// Auton Steps
		AutonDBSteps = 1;
		m_armEncoder->Reset();
		m_ramEncode->Reset();
		m_rEncode->Reset();
		m_lEncode->Reset();
		m_ramCase = -1;
		m_medRamCase = -1;
		m_shifters -> Set(true);
		Drive_Status = false;
		m_bArm -> Set(false);
	}

	void TeleopInit() {
		m_shifters -> Set(false);
		m_ramCase = -1;
		m_medRamCase = -1;
		m_ramInit = false;
		m_ramTime->Stop();
		m_ramTime->Start();
		m_ramTime->Reset();
		m_rEncode->Reset();
		m_lEncode->Reset();
		m_armEncoder->Reset();
		// m_arm->PIDDisable();
	}
	
	void TestInit () {
		
	}

	/********************************** Periodic Routines *************************************/
	void DisabledPeriodic()  {
		autonChoice = AutonDBrebound;
	}

	void AutonomousPeriodic() {
		ManageCompressor();
		PrintData();
	    AutonDBRebound();
	    RamFire();
	    //m_bArm -> Set(false);
	}

	
	void TeleopPeriodic() {
		ManageCompressor();
		TeleopDrive();
		RamrodInit();
		RamFire();
		MedRamFire();
		RamrodOverride();
		TeleopArm();
		TeleopBGrabber();
		//TeleopRanrod();
		PrintData();
		// TestArm();
		
	}
	

	
	void TestPeriodic () {
		ManageCompressor();
		// TestArm();
		TestDrive();
		TestBGrabber();
		TestRamMotion();
		TestRamLock();
		PrintData();
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
		if(m_driver->GetRawButton(BUTTON_BACK))
		{
			m_rEncode->Reset();
			m_lEncode->Reset();
		}
		
		if (m_driver->GetRawButton(BUTTON_START)) {
			AutonStraightDrive(1.0,32 * REV_IN);
		}
	}
	
	 void TeleopBGrabber()
	{
		//ROLLERS	
		if (m_operator->GetRawAxis(TRIGGERS) > 0.4) {
			m_roller->Set(1);
		}
		else if (m_operator->GetRawAxis(TRIGGERS) < -0.4)
			m_roller->Set(-1);
		else {
			m_roller->Set(0.0);
		}
		
		//BALL CATCH
		if (m_operator->GetRawButton(BUTTON_BACK) && m_bGrabberTime->Get() > 0.1) {
			// Timer
			m_bGrabberTime->Stop();
			m_bGrabberTime->Reset();
			
			m_catch->Set(!m_catch->Get());
		}
		else if (m_operator->GetRawButton(BUTTON_BACK)) {
			m_bGrabberTime->Reset();
			m_bGrabberTime->Start();
		}
/*
		//BALL CATCH (#Sweg)
		if (m_operator->GetRawButton(BUTTON_BACK)) {
			
			m_catch->Set(true); 
		}
		else {
			m_catch->Set(false);
		}
	*/	
		//BALL CATCH SERVOS
		/*
		if (m_operator->GetRawButton(BUTTON_START))
		{
			m_catchServo1->SetAngle(180);
			m_catchServo2->SetAngle(180);
		}
		else
		{
			m_catchServo1->SetAngle(0);
			m_catchServo2->SetAngle(0);
		}
		*/
		//bArm OPEN / CLOSE
		if (m_operator->GetRawButton(BUTTON_START) && m_bGrabberTime->Get() > 0.1) {
			// Timer
			m_bGrabberTime->Stop();
			m_bGrabberTime->Reset();
			
			m_bArm->Set(!m_bArm->Get());
		}
		else if (m_operator->GetRawButton(BUTTON_START)) {
			m_bGrabberTime->Reset();
			m_bGrabberTime->Start();
		}
		
	}
		  

	
	void TeleopArm ()
	{
		// ----- PID -----
		if (m_operator->GetRawButton(BUTTON_A)) {
			// Floor Picking
			m_armPIDFlag = true;
			m_armPID->SetSetpoint (FLOOR_PICKING_POS);
			m_armPID->Enable();
			
		} else if (m_operator->GetRawButton(BUTTON_B)) {
			// Medium (12ft) Shoot Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint (MED_SHOOT_POS);
			m_armPID->Enable();
			
		} else if (m_operator->GetRawButton(BUTTON_X)) {
			// Long (18ft) Shoot Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(LONG_SHOOT_POS);
			m_armPID->Enable();
			
		} else if (m_operator->GetRawButton(BUTTON_Y)) {
			// Catch Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(CATCH_POS);
			m_armPID->Enable();
			
		} 
		else {
			if (m_armPIDFlag) {
				m_armPID->Disable();
				m_armPIDFlag = false;
			}
			
			// Control With Joystick
			m_armMotor->Set(m_operator->GetRawAxis(LEFT_Y));
			SmartDashboard::PutNumber("Arm Motor Input:",m_operator->GetRawAxis(LEFT_Y));
		}
		SmartDashboard::PutNumber("Arm PID Output:",m_armPID->Get());
		// Reset Arm
		// if (m_arm->GetLimSwitch()) {
			//m_arm->Reset();
		// }
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
	/*
	void TestArm ()
	{
		// Control Arm
		m_arm->Set(m_operator->GetRawAxis(LEFT_Y));

		// Reset Arm Encoder
		if (m_operator->GetRawButton(BUTTON_L3)) {
			m_arm->Reset();
		}
		
		
	}
	*/
	void TestBGrabber()
		{
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
			m_ramMotor->Set(1);
		else if (m_driver->GetRawButton(BUTTON_RB))
			// OUT
			m_ramMotor->Set(-1);
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
	void AutonStraightDrive(double output, double distance)
		{
		    SmartDashboard::PutNumber("Straight Distance: ",distance);
			float comp_spd = 0.05;
		    if (fabs(m_rEncode -> GetDistance()) < fabs(distance)){
		    	if (m_rEncode->GetDistance() + 10 > m_lEncode->GetDistance())
		    	{
		            m_robotDrive->TankDrive(output + comp_spd, output - comp_spd);
				}
				else if (m_lEncode->GetDistance() + 10 > m_rEncode->GetDistance())
				{
						m_robotDrive->TankDrive(output - comp_spd, output + comp_spd);
				}
				else
				{
						m_robotDrive->TankDrive(output, output);
				}
		    }
		    else{
		    	Drive_Status = true;
		    	m_robotDrive->TankDrive(0.0,0.0);
		    }
		}
	void RamrodInit(){
		if (!m_ramInit)
		{
			m_ramServo->SetAngle(0);
			m_ramMotor->Set(-0.2);
			if (m_ramTime->HasPeriodPassed(0.2))
			{
				if (abs(m_ramEncode->GetRate()) < 100)
				{
					m_ramEncode->Reset();
					m_ramMotor->Set(0.0);
					m_ramTime->Stop();
					m_ramTime->Reset();
					m_ramInit = true;
					m_ramCase = -1;
				}
					
			}
		}
	}
	
	void RamFire()
	{
		//Do the thing with bGrabber
		if (m_driver->GetRawAxis(TRIGGERS) < -0.4 && m_ramCase == -1)
		{
			m_ramCase = 0;
		}
		/*else if (m_driver->GetRawAxis(TRIGGERS) > 0.4 && m_ramCase == -1)
		{
			m_ramCase = 3;
		}*/
		
		switch(m_ramCase)
		{
		case 0:
			m_ramTime->Stop();
			m_ramTime->Start();
			m_ramTime->Reset();
			m_ramCase++;
			break;
		case 1:
			m_ramServo->SetAngle(120);
			if(m_ramTime->HasPeriodPassed(0.5))
				m_ramCase++;
			break;
		case 2:
			m_ramServo->SetAngle(0);
			m_ramCase++;
			m_ramTime->Stop();
			break;
		case 3:
			if (abs(m_ramEncode->GetDistance()) < RAM_LOCK_POSITION)
				m_ramMotor->Set(1);
			else
				m_ramCase++;
			break;
		case 4:
			if (abs(m_ramEncode->GetDistance()) < 20)
			{
				m_ramMotor->Set(-0.2);
				m_ramCase++;
			}
			else
				m_ramMotor->Set(-1.0);
			break;
		case 5:
			if(m_ramEncode->GetDistance() < 50)
			{	
				m_ramMotor->Set(0.0);
				m_ramCase = -1;
			}
			break;
		}
		
	}
	
	void MedRamFire() {
		if (m_driver->GetRawAxis(TRIGGERS) > 0.4)
			m_medRamCase = 0;
		switch (m_medRamCase)
		{
		case 0:
			if (abs(m_ramEncode->GetDistance()) > (RAM_MID_POSITION - 100))
				m_ramMotor->Set(1);
			else
				m_medRamCase++;
			break;
		case 1:
			if (abs(m_ramEncode->GetDistance()) > (RAM_MID_POSITION - 20))
			{
				m_ramMotor->Set(0.2);
				m_medRamCase++;
			}
			else
				m_ramMotor->Set(1.0);
			break;
		case 2:
			if (m_ramEncode->GetDistance() > (RAM_MID_POSITION - 50))
			{	
				m_ramMotor->Set(0.0);
				m_ramTime->Stop();
				m_ramTime->Start();
				m_ramTime->Reset();
				m_medRamCase++;
			}
			break;
		case 3:
			if (m_ramServo->GetAngle() != 120)
				m_ramServo->SetAngle(120);
			if (m_ramTime->HasPeriodPassed(0.3))
			{
				m_ramTime->Stop();
				m_ramTime->Reset();
				m_medRamCase++;
			}
			break;
		case 4:
			if (m_ramEncode->GetDistance() > 20)
			{
				m_ramMotor->Set(-0.3);
			}
			else
			{
				m_ramCase = 2;
				m_medRamCase = -1;
			}
			break;
		}
	}
	
	void RamrodOverride()
	{
		if (m_operator->GetRawButton(BUTTON_LB) && m_operator->GetRawButton(BUTTON_RB))
		{
			SmartDashboard::PutNumber("Joystick Value: ",m_operator->GetRawAxis(RIGHT_Y));
			
			if (fabs(m_operator->GetRawAxis(RIGHT_Y)) > 0.2)
					m_ramMotor->Set(m_operator->GetRawAxis(RIGHT_Y));
			else
					m_ramMotor->Set(0.0);					
		}
	}
	
	void PrintData()
	{
		SmartDashboard::PutNumber("Ramrod Encoder: ", m_ramEncode->GetDistance());
		SmartDashboard::PutNumber("Ramrod Rate: ",m_ramEncode->GetRate());
		SmartDashboard::PutNumber("Ramrod Raw: ",m_ramEncode->GetRaw());
		SmartDashboard::PutNumber("Ramrod Case: ",m_ramCase);
		SmartDashboard::PutNumber("Medium Ramrod Fire Case: ",m_medRamCase);
		
		SmartDashboard::PutNumber("Arm Actual Position: ", m_armEncoder->GetDistance());
		SmartDashboard::PutNumber("Arm PID Output: ", m_armEncoder->PIDGet());
		
		SmartDashboard::PutNumber("lEncoder: ",m_rEncode->GetDistance());
		SmartDashboard::PutNumber("rEncoder: ",m_lEncode->GetDistance());
		
		// Auton
		SmartDashboard::PutNumber("Auton Step: ", AutonDBSteps);
		SmartDashboard::PutNumber("Ram Time: ",m_ramTime->Get());
		SmartDashboard::PutBoolean("Drive Status: ", Drive_Status);
	}
	
};

START_ROBOT_CLASS(BuiltinDefaultCode);
