#include "WPILib.h"
#include "JoystickWrapper.h"
#include "DriveWrapper.h"
#include "ArmWrapper.h"
#include "Defines.h"
#include <cmath>

/**
 * HOTBOT 2014 v1.0 - Build Date: 3/3/14
 * 
 * See the wiki on the GitHub repository for more information
 * 
 * Controller Map:
 * 
 * Teleop
 * ======
 * 
 * Driver
 * ------
 * 
 * Left Analog Up/Down: Drive Forward/Backward
 * Right Analog Left/Right: Move Left/Right
 * 
 * Right Bumper (HOLD): Shift Down
 * 
 * Left Trigger: Medium-power shot
 * Right Trigger: Full-power shot
 * 
 * Operator
 * --------
 * 
 * Left Analog Up/Down: Manual arm manipulation
 * 
 * A: Arm at floor pickup position
 * B: Arm at medium shot position
 * X: Arm at long shot position
 * Y: Arm at catch position
 * 
 * Back: Toggle catch arms
 * Start: Toggle bottom arm
 * 
 * Left Trigger: Roller backwards
 * Right Trigger: Roller forwards
 * 
 * Test
 * ====
 * 
 * Driver
 * ------
 * 
 * Left Analog Up/Down: Drive Forward/Backward
 * Right Analog Left/Right: Move Left/Right
 * 
 * Left Analog Push: Reset left encoder
 * Right Analog Push: Reset right encoder
 * 
 * A: Shifter
 * 
 * Left Bumper: Ramrod In
 * Right Bumper: Ramrod Out
 * 
 * Right Trigger: Release ramrod
 * Left Trigger: Lock ramrod
 * 
 * Start: Reset ramrod encoder
 * 
 * Operator
 * --------
 * 
 * Left Analog Up/Down: Arm control
 * 
 * Left Analog Push: Reset arm encoder
 * 
 * A: Extend catch arms
 * B: Retract catch arms
 * X: Grab with bottom arm
 * Y: Release with bottom arm
 * 
 * Start: Camera light
 * 
 * Left trigger: Roll out
 * Right trigger: Roll in
 * 
 * Other useful information
 * ========================
 * Left trigger: TRIGGERS > 0
 * Right trigger: TRIGGERS < 0
 */

class BuiltinDefaultCode : public IterativeRobot
{
private:
	// MOTOR CONTROLLERS *********************************
      
	//Declare drive motors
	Talon* m_lDrive1; //Two motors
	Talon* m_rDrive1; //One motor
	Talon* m_lDrive2; //Two motors
	Talon* m_rDrive2; //One motor

	//Declare arm
	Talon* m_armMotor;
	
	//Declare ramrod motor
	Talon *m_ramMotor;
	
	//Declare bGrabber motor
	Talon* m_roller;
	
	// SERVOS ********************************************
	
	//Declare ramrod servo
	Servo *m_ramServo;
	
	//Declare catch arm stop servos
	Servo *m_catchServo1;
	Servo *m_catchServo2;
	
	// PNEUMATICS ****************************************
	
	//Declare Compressor
	Compressor* m_compressor;
	
	// shifters
	Solenoid *m_shifters;
	
	//Declare bGrabber solenoids
	Solenoid* m_catch;
	Solenoid* m_bArm;
	
	// DRIVE ABSTRACTION OBJECTS *************************
	
	//Declare drive objects
	DriveWrapper* m_rDrive;
	DriveWrapper* m_lDrive;
	RobotDrive* m_robotDrive;
	
	// SENSORS *******************************************
	
	//Drivetrain Encodes
	Encoder *m_rEncode;
	Encoder *m_lEncode;
	
	//Arm encoder
	Encoder* m_armEncoder;
	
	//Declare ramrod encoder
	Encoder *m_ramEncode;
	
	//Arm light sensor
	DigitalInput* m_armReset;
	
	// PID CONTROLLERS ***********************************
	
	//Arm PID controller
	PIDController* m_armPID;
	
	// OTHER ABSTRACTION OBJECTS *************************
	
	// ArmWrapper* m_arm;
	
	// DRIVER INTERFACE OBJECTS **************************
	
	//Declare joysticks
	JoystickWrapper* m_driver;
	JoystickWrapper* m_operator;
	
	//Declare driver station
	DriverStationLCD* m_dsLCD;
	
	// MISCELLANEOUS *************************************
	
	//Timers
	Timer *m_ramTime;
	Timer *m_autonTime;
	
	// Counter
	int countLoop;
	int armCount;
	int autonCount;
	
	int ramFire;
	int m_medRamCase;
	int m_ramCase;
	bool m_ramInit;
	bool Drive_Status;
	bool m_armPIDFlag;
	
	double m_armResetPos;
	bool m_armResetCheck;
	bool m_armResetFlag [2];
	
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
		// MOTOR CONTROLLERS *********************************
	  
		//Initialze drive controllers
		m_rDrive1 = new Talon (1);
		m_rDrive2 = new Talon (2);
		m_lDrive1 = new Talon (3);
		m_lDrive2 = new Talon (4);
		
		//Initialize ramrod motor
		m_ramMotor = new Talon (5);
		
		//Initialize Arm
		m_armMotor = new Talon (6);
		
		//initialize bGrabber motor
		m_roller = new Talon (7);
		
		// SERVOS ********************************************
		
		//Initialize ramrod servo
		m_ramServo = new Servo (10);
		
		#ifdef CompetitionBot
		//Initialize catch servos
		m_catchServo1 = new Servo (8);
		m_catchServo2 = new Servo (9);
		#endif
		
		// DRIVE ABSTRACTION OBJECTS *************************
		
		//Initialize drive wrappers
		m_rDrive = new DriveWrapper (m_rDrive1, m_rDrive2);
		m_lDrive = new DriveWrapper (m_lDrive1, m_lDrive2);
		
		//Initialize robot drive
		m_robotDrive = new RobotDrive (m_lDrive, m_rDrive);
		m_robotDrive->SetSafetyEnabled(false);
		
		// SENSORS *******************************************
		
		//Drive encoders
		m_rEncode = new Encoder (1,2,true);
		m_rEncode->SetDistancePerPulse(1);
		m_rEncode->SetMaxPeriod(1.0);
		m_rEncode->Start();
		
		m_lEncode = new Encoder (3,4,false);
		m_lEncode->SetDistancePerPulse(1);
		m_lEncode->SetMaxPeriod(1.0);
		m_lEncode->Start();
		
		//Initialize the arm encoder
		m_armEncoder = new Encoder (5, 6, false);
		//m_armEncoder = new Encoder (5, 6, true);
		m_armEncoder->SetDistancePerPulse(1.0);
		m_armEncoder->SetMaxPeriod(1.0);
		m_armEncoder->Start();
		
		//Initialize ramrod encoder
		m_ramEncode = new Encoder (7,8,true);
		m_ramEncode->SetDistancePerPulse(1);
		m_ramEncode->SetMaxPeriod(1.0);
		m_ramEncode->Start();
		
		//Arm reset light sensor
		m_armReset = new DigitalInput (11);
		
		// PNEUMATICS ****************************************
		
		//Initialize Compressor
		m_compressor = new Compressor(9, 1);
		
		//shifters
		m_shifters = new Solenoid(1);
		
		//Initialize bGrabber Solenoids
		m_bArm = new Solenoid (2);
		m_catch = new Solenoid (3);
		
		// PID CONTROLLERS ***********************************
		
		m_armPID = new PIDController(ARM_P, ARM_I, ARM_D, m_armEncoder, m_armMotor);
		
		// OTHER ABSTRACTION OBJECTS *************************
		
		// m_arm = new ArmWrapper (6, 8, 5, 6, 10);
		//m_arm->StartPID(0.0, 0.0, 0.0);
		
		// DRIVER INTERFACE OBJECTS **************************

		//Initialize joysticks
		m_driver = new JoystickWrapper (1);
		m_operator = new JoystickWrapper (2);
		
		//Grab driver station object
		m_dsLCD = DriverStationLCD::GetInstance();
		
		// MISCELLANEOUS *************************************
		
		//Timers
		m_ramTime = new Timer;
		m_autonTime = new Timer;
		
		m_ramCase = -1;
		m_medRamCase = -1;
		countLoop = 0;
		armCount = 0;
		Drive_Status = false;
		m_armResetPos = 0;
		m_armResetCheck = false;
		m_armResetFlag[0] = false;
		m_armResetFlag[1] = false;
		
		// Auton Steps
		AutonDBSteps = 1;
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

/********************************** External Routines *************************************/
	
	/*********************** AUTONOMOUS FUNCTIONS ****************************/
	
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
	}*/
	
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
		if (m_operator->GetRawButton(BUTTON_RB)) {			
			m_catch->Set(true);
		}
		else
			m_catch->Set(false);
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
		if (m_operator->GetButtonPress(BUTTON_START)) {
			m_bArm->Set(!m_bArm->Get());
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
			m_armPID->SetSetpoint (MED_SHOT_BACK);
			m_armPID->Enable();
			
		} else if (m_operator->GetRawButton(BUTTON_X)) {
			// Long (18ft) Shoot Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(MED_SHOOT_POS);
			m_armPID->Enable();
			
		} else if (m_operator->GetRawButton(BUTTON_Y) && m_operator->GetRawButton(BUTTON_BACK)) {
				//Reset arm encoder
				double dirToZero;
				
				dirToZero = m_armEncoder->GetDistance()-m_armResetPos;
				if (fabs(dirToZero) < 5)
					m_armMotor->Set(0.0);
				else if (dirToZero < 0)
					m_armMotor->Set(0.2);
				else if (dirToZero > 0)
					m_armMotor->Set(-0.2);
				
				SmartDashboard::PutNumber("Direction to Arm Zero: ",dirToZero);
	
				ArmReset();
		
		} else if (m_operator->GetRawButton(BUTTON_Y)) {
			// Catch Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(0.0);
			m_armPID->Enable();
			
		} else if (m_operator->GetRawButton(BUTTON_LB))
		{
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
		}
		
		if (!m_operator->GetRawButton(BUTTON_BACK))
		{
			if (!m_armReset->Get()&& m_armEncoder->GetRate() < 0 && !m_armResetFlag[0])
			{
				if (fabs(m_armEncoder->GetDistance()) > 10)
				{
					m_armResetPos = m_armEncoder->GetDistance();
					m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"ARM RESET NEEDED     ");
					m_armResetFlag [0] = true;
				}
			}
			else if (!m_armReset->Get() && !m_armResetCheck && m_armEncoder->GetRate() > 0)
			{
				if (fabs(m_armEncoder->GetDistance()) > 5)
				{
						m_armResetFlag[0] = true;
						m_armResetCheck = true;
				}
			}
			else if (m_armResetCheck && m_armReset->Get() && m_armEncoder->GetRate() > 0)
			{
				m_armResetPos = m_armEncoder->GetDistance();
				m_armResetCheck = false;
				
				m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"ARM RESET NEEDED     ");
			}
			else if (m_armResetCheck && m_armReset->Get())
			{
				m_armResetCheck = false;
			}
		}
		
		m_dsLCD->UpdateLCD();
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
	
	void ArmReset()
	{
		if (!m_armReset->Get() && m_armEncoder->GetRate() < 0 && !m_armResetFlag[1])
		{
			m_armEncoder->Reset();
			m_armResetPos = 0;
			m_armResetFlag[1] = true;
			m_armMotor->Set(0.0);
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"                     ");
		}
		else if (!m_armReset->Get() && !m_armResetCheck && m_armEncoder->GetRate() > 0)
		{
			m_armResetFlag[1] = true;
			m_armResetCheck = true;
		}
		else if (m_armReset->Get() && m_armResetCheck)
		{
			m_armEncoder->Reset();
			m_armResetPos = 0;
			m_armMotor->Set(0.0);
			m_armResetCheck = false;
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"                     ");
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
		if (m_operator->GetRawButton(BUTTON_BACK))
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
		SmartDashboard::PutNumber("Ram Time: ",m_ramTime->Get());
		SmartDashboard::PutNumber("Medium Ramrod Fire Case: ",m_medRamCase);
		
		SmartDashboard::PutNumber("Arm Actual Position: ", m_armEncoder->GetDistance());
		SmartDashboard::PutNumber("Arm Rate: ", m_armEncoder->GetRate());
		SmartDashboard::PutNumber("Arm PID Input: ", m_armEncoder->PIDGet());
		SmartDashboard::PutNumber("Arm PID Output:",m_armPID->Get());
		SmartDashboard::PutNumber("Arm Motor Input:",m_operator->GetRawAxis(LEFT_Y));
		SmartDashboard::PutBoolean("Arm Difference Bool", fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS) < AUTON_ANGLE_GAP);
		
		SmartDashboard::PutNumber("lEncoder: ",m_rEncode->GetDistance());
		SmartDashboard::PutNumber("rEncoder: ",m_lEncode->GetDistance());
		SmartDashboard::PutBoolean("Drive Status: ", Drive_Status);

		SmartDashboard::PutNumber("Arm Reset Position: ",m_armResetPos);
		SmartDashboard::PutBoolean("Arm Reset Sensor",m_armReset->Get());
		SmartDashboard::PutBoolean("Arm Reset Check Flag",m_armResetCheck);
		
		// Auton
		SmartDashboard::PutNumber("Auton Step: ", AutonDBSteps);
		SmartDashboard::PutBoolean("Auton Time: ", m_autonTime->Get());
	}
	
};

START_ROBOT_CLASS(BuiltinDefaultCode);
