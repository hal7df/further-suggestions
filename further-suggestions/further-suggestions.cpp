#include "WPILib.h"
#include "JoystickWrapper.h"
#include "DriveWrapper.h"
#include "ArmWrapper.h"
#include "Defines.h"
#include <cmath>
#include "CameraHandler.h"

/**
 * HOTBOT 2014 v1.5 - Build Date: 3/6/14
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

enum AutonChoice {
		AutonDBrebound,
		AutonDFshoot,
		AutonCheckHotleft,
		AutonCheckHotright,
		AutonDoNothing,
		AutonDf,
		AutonBalltrack
};

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
	
	//Declare camera light
	Relay* m_camLight;
	
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
	DriveRotate* m_driveRotate;
	DriveStraightPID* m_drvSource;
	
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
	
	//Drive PID controller
	PIDController* m_drvStraightPID;

		
	// OTHER ABSTRACTION OBJECTS *************************
	
	ArmWrapper* m_arm;
	
	//Declare camera handler object
	//CameraHandler* m_cameraHandler;
	
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
	
	//Auton Selector Variables
	AutonChoice autonChoice;
	int m_selectorCountdown;
	int m_selectorPage;
	
	//Arm reset variables
	double m_armResetPos;
	bool m_armResetFlag;
	bool m_canResetArm;
	int m_armResetCase;
	
	
	// Auton Steps
	int AutonDBSteps;
	int AutonSteps;
	int autondance;

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
		
		//Initialize relays
		m_camLight = new Relay (2);
		
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
			
		
		// DRIVE ABSTRACTION OBJECTS *************************
				
		//Initialize drive wrappers
		m_rDrive = new DriveWrapper (m_rDrive1, m_rDrive2);
		m_lDrive = new DriveWrapper (m_lDrive1, m_lDrive2);
		
		//Initialize robot drive
		m_robotDrive = new RobotDrive (m_lDrive, m_rDrive);
		m_robotDrive->SetSafetyEnabled(false);
		
		//Initialize Drive Rotate
		m_driveRotate = new DriveRotate (m_robotDrive, m_lEncode, m_rEncode);
		m_drvSource = new DriveStraightPID (m_robotDrive, m_lEncode, m_rEncode);
		
		// OTHER ABSTRACTION OBJECTS *************************
		
		m_arm = new ArmWrapper (6, 8, 5, 6, 10);
		
		// PID CONTROLLERS ***********************************
				
		m_armPID = new PIDController(ARM_P, ARM_I, ARM_D, m_armEncoder, m_armMotor);
		m_drvStraightPID = new PIDController(DRV_P, DRV_I, DRV_D, m_drvSource, m_drvSource);
		
		// DRIVER INTERFACE OBJECTS **************************

		//Initialize joysticks
		m_driver = new JoystickWrapper (1);
		m_operator = new JoystickWrapper (2);
		
		//Initialize camera handler object
		/*
		AxisCamera *camera = &AxisCamera::GetInstance("10.0.67.11");
		m_cameraHandler = new CameraHandler (camera, m_dsLCD, m_camLight);
		*/
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
		
		//Arm reset variables
		m_armResetPos = 0;
		m_armResetFlag = false;
		m_canResetArm = false;
		m_armResetCase = 0;
		
		//Auton Selector Variables
		m_selectorCountdown = 100;
		m_selectorPage = 0;
		
		// Auton Steps
		AutonDBSteps = 1;
		AutonSteps = 0;
		autondance = 0;
	}

	/********************************** Init Routines *************************************/


	void RobotInit() {
	  
	}
	
	void DisabledInit() {
		autonChoice = AutonDFshoot;
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
		//m_shifters -> Set(true);
		
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"      AUTONOMOUS     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"         Mode:       ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3,1,"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5,1,"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5,1,"                     ");
		m_dsLCD->UpdateLCD();
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
		//m_armEncoder->Reset();
		m_armPID->Disable();
		
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"       TELEOP        ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3,1,"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5,1,"                     ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5,1,"                     ");
		m_dsLCD->UpdateLCD();
	}
	
	void TestInit () {
		
	}

	/********************************** Periodic Routines *************************************/
	void DisabledPeriodic()  {
		
		m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"HOTBOT b.3-06-14 v1.5");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"  ||   ||  __  ----- ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line3,1,"  ||--|| /    \\   |  ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"  ||   || \\__/   |   ");
		m_dsLCD->Printf(DriverStationLCD::kUser_Line5,1,"        Auton:       ");
		
		if (m_operator->GetButtonPress(BUTTON_A))
		{
			autonChoice = AutonDFshoot;
			m_selectorPage = 0;
		}
		else if (m_operator->GetButtonPress(BUTTON_B))
		{
			autonChoice = AutonCheckHotright;
			m_selectorPage = 0;
		}
		else if (m_operator->GetButtonPress(BUTTON_X))
		{
			autonChoice = AutonCheckHotleft;
			m_selectorPage = 0;
		}
		else if (m_operator->GetButtonPress(BUTTON_Y))
		{
			autonChoice = AutonDBrebound;
			m_selectorPage = 0;
		}
		else if (m_operator->GetButtonPress(BUTTON_RB))
		{
			autonChoice = AutonDf;
			m_selectorPage = 0;
		}
		else if (m_operator->GetButtonPress(BUTTON_LB))
		{
			autonChoice = AutonBalltrack;
			m_selectorPage = 0;
		}
		else if (m_operator->GetButtonPress(BUTTON_START))
		{
			if (m_selectorPage == 2)
				m_selectorPage = 0;
			else
				m_selectorPage++;
		}
		
		switch (autonChoice)
		{
		case AutonDBrebound:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"      DB 2 Ball      ");
			break;
		case AutonDFshoot:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"      DF Shoot       ");
			break;
		case AutonCheckHotleft:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"   Check Left Hot    ");
			break;
		case AutonCheckHotright:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"   Check Right Hot   ");
			break;
		case AutonDoNothing:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"      DISABLED       ");
			break;
		case AutonDf:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"    Drive Forward    ");
			break;
		case AutonBalltrack:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"     Ball Tracker    ");
			break;
		}
		
		if (m_operator->GetRawButton(BUTTON_BACK) && autonChoice != AutonDoNothing)
		{
			if (m_selectorCountdown > 0)
			{
				m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"                     ");
				m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"DISABLING...%d"+m_selectorCountdown);
				m_selectorCountdown--;
			}
			else
			{
				m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"DISABLING...RELEASE");
			}
			m_selectorPage = 0;
		}
		else if (m_selectorCountdown == 0)
		{
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"      DISABLED       ");
			autonChoice = AutonDoNothing;
			m_selectorCountdown = 100;
		}
		
		switch (m_selectorPage)
		{
		case 1:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"A: DF Shoot          ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"B: Check Left Hot    ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3,1,"X: Check Right Hot   ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"Y: Drive Back 2 Ball ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line5,1,"RB: Drive Forward    ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"LB: Shoot & Ball Trk ");
			break;
		case 2:
			m_dsLCD->Printf(DriverStationLCD::kUser_Line1,1,"Back (HOLD): Disable ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"                     ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line3,1,"                     ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line4,1,"                     ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line5,1,"                     ");
			m_dsLCD->Printf(DriverStationLCD::kUser_Line6,1,"                     ");
			break;
		}
		
		m_dsLCD->UpdateLCD();
	}

	void AutonomousPeriodic() {
		ManageCompressor();
		PrintData();
		RamFire();
		
		switch (autonChoice)
		{
		case AutonDf:
			AutonDF();
			break;
		case AutonDFshoot:
			AutonDFShoot();
			break;
		case AutonDBrebound:
			AutonDBRebound();
			break;
	/*	case AutonCheckHotleft:
			AutonCheckHotLeft();
			break;
		case AutonCheckHotright:
			AutonCheckHotRight();
			break;
		case AutonBalltrack:
			//Ball tracker auton
			break;
	*/
		}
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
		switch(AutonDBSteps) {
		case 1:
			
			if (!m_armPID->IsEnabled())
			{
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				m_armPID->Enable();
			}
			
			if (!m_drvStraightPID->IsEnabled())
			{
				m_drvStraightPID->SetSetpoint(-64.0);
				m_drvStraightPID->Enable();
			}
			
			if (fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint()) < 5){
				AutonDBSteps++;
				m_drvStraightPID->Disable();
			}
			if (fabs(m_armEncoder->GetDistance() - MED_SHOT_BACK) < AUTON_ANGLE_GAP && m_ramCase == -1) {
				m_ramCase = 0;
			}
			break;
			
		case 2:
			SmartDashboard::PutNumber("Arm Difference", fabs(m_armEncoder->GetDistance() - LONG_SHOOT_POS));
			if (fabs(m_armEncoder->GetDistance() - MED_SHOT_BACK) < AUTON_ANGLE_GAP && m_ramCase == -1) {
				m_ramCase = 0;
			}
			
			if (m_ramCase > 2){
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				AutonDBSteps++;
			}
			break;

		case 3:
			m_armPID->SetSetpoint(FLOOR_PICKING_POS);
			m_roller->Set(-1.0);
			if(m_autonTime->HasPeriodPassed(0.5))
			{
				m_drvStraightPID->SetSetpoint(55.0);
				m_drvStraightPID->Enable();
			}
			SmartDashboard::PutNumber("Arm Difference", fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS));
						
			if ((fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS) < AUTON_ANGLE_GAP) && fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint()) < 5)
			{
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				m_drvStraightPID->Disable();	
				AutonDBSteps++;
			}
			break;
		case 4:
			if (fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint()) < 5)
			{
				m_autonTime->Stop();
				m_autonTime->Start();
				m_autonTime->Reset();
				m_rEncode -> Reset();
				m_lEncode -> Reset();
				m_drvStraightPID->Disable();
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				AutonDBSteps++;
			}
			
			if (m_autonTime->HasPeriodPassed(.5))
			{
				m_autonTime->Stop();
				m_autonTime->Reset();
				m_drvStraightPID->SetSetpoint(40.0);
				m_drvStraightPID->Enable();
			}
		break;
		case 5:	
			m_drvStraightPID->SetSetpoint(-80.0);
			m_drvStraightPID->Enable();
		
			SmartDashboard::PutNumber("Arm Difference", fabs(m_armEncoder->GetDistance() - FLOOR_PICKING_POS));
						
			if((fabs(m_armEncoder->GetDistance() - MED_SHOT_BACK) < AUTON_ANGLE_GAP) && fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint()) < 5){
				m_rEncode -> Reset();
				m_lEncode -> Reset();		
				m_drvStraightPID->Disable();
				m_autonTime->Stop();
				m_autonTime->Reset();
				m_roller->Set(0.0);
				AutonDBSteps++;
			}
			break;	
		case 6:
			if (m_ramCase == -1)
				m_ramCase = 0;
			else if (m_ramCase == 3)
				AutonDBSteps++;
			break;
		case 7:
			m_armPID->SetSetpoint(0.0);
			m_armPID->Enable();
			
			break;
		}
	}
	
	void AutonDFShoot(){
		switch(AutonSteps){
		case 0:
			
			if (!m_drvStraightPID->IsEnabled())
			{
				m_drvStraightPID->SetSetpoint(-64.0);
				m_drvStraightPID->Enable();
			}
			
			if (!m_armPID->IsEnabled())
			{
				m_armPID->SetSetpoint(MED_SHOT_BACK);
				m_armPID->Enable();
			}
			
			if (fabs(MED_SHOT_BACK - m_armEncoder->GetDistance()) < AUTON_ANGLE_GAP && fabs(m_drvSource->PIDGet() - m_drvStraightPID->GetSetpoint()) < 5) {
				m_ramCase = 0;
				AutonSteps++;
			}	
		break;
		case 1:
			if (m_drvStraightPID->IsEnabled())
				m_drvStraightPID->Disable();
			if (m_armPID->IsEnabled())
				m_armPID->Disable();
		break;
		}
	}
/*
	void AutonCheckHotLeft(){
		AutonStraightDrive(-1, 32 * REV_IN);
		RamFire();
		switch(AutonSteps){
		case 0:
			m_arm->SetAngle(MED_SHOOT_POS);
			m_arm->PIDEnable();
			if (Drive_Status){
				if (m_cameraHandler->getHotGoal() == kLeft){
					m_ramCase = 0;		
					AutonSteps = 1;
				}			
			}
		break;
		case 1:
		break;
		}
	}
	void AutonCheckHotRight(){
		AutonStraightDrive(-1, 32 * REV_IN);
		RamFire();
		switch(AutonSteps){
		case 0:
			m_arm->SetAngle(MED_SHOOT_POS);
			m_arm->PIDEnable();
			if (Drive_Status){
				if (m_cameraHandler->getHotGoal() == kRight){
					m_ramCase = 0;	
					AutonSteps = 1;	
				}
			}
		case 1:
		break;
		}
	}
*/
	void AutonDF(){
		if (!m_drvStraightPID->IsEnabled())
		{
			m_drvStraightPID->SetSetpoint(-64.0);
			m_drvStraightPID->Enable();
		}
	}
/*	
	void AutonTracker () {
		double ballX;
		
		switch (AutonSteps) {
		case 0:		// Drive Foward + Arm Set Point
			// Drive Foward
			AutonStraightDrive(1,32 * REV_IN);
			
			// Arm Set Point
			m_arm->SetAngle(MED_SHOOT_POS);
			m_arm->PIDEnable();
			
			// Check if arm and drive are ready
			if (Drive_Status) {
				m_arm->PIDDisable();
				AutonSteps++;
			}
			break;
		case 1:		// Move to Center of the Hot Goal + Shoot
			m_robotDrive->ArcadeDrive(0.0, m_cameraHandler->getCenter());
			
			if (fabs(m_cameraHandler->getCenter()) < ROTATE_ANGLE_GAP) {
				m_ramCase = 0;
				AutonSteps++;
			}
			break;
		case 2:		// Dance and find Ball
			// Dance
			Dance();
			
			// Move The Arm to the Pick up position
			m_arm->SetAngle(FLOOR_PICKING_POS);
			m_arm->PIDEnable();
			
			// Check Ball
			ballX = m_cameraHandler->getBallX();
			if (-1.0 < ballX && ballX < 1.0) {
				// Find The Ball
				AutonSteps++;
			}
			break;
		case 3:		// Track Ball
			m_roller->Set(-1.0);
			ballX = m_cameraHandler->getBallX();
			if (-1.0 < ballX && ballX < 1.0) {
			m_robotDrive->ArcadeDrive(0.8, ballX);
			}
			else if (-1.0 > ballX && ballX > 1.0){
				m_autonTime->Start();
				m_autonTime->Reset();
				if (m_autonTime-> HasPeriodPassed(1.0)){
					AutonSteps++;
				}
				else{
					m_robotDrive->ArcadeDrive(1.0,0.0);
				}
				
			}
			
			break;
		case 4:
			m_robotDrive->TankDrive(-.25,.25);
			if (-1.0 < ballX && ballX < 1.0) {
				AutonSteps = 3;
			}
			if (m_autonTime-> HasPeriodPassed(2.0)){
				AutonSteps++;
			}
			break;
		case 5:
			m_autonTime->Stop();
			m_arm->SetAngle(MED_SHOOT_POS);
			m_arm->PIDEnable();
			if (fabs(m_cameraHandler->getCenter()) < ROTATE_ANGLE_GAP) {
				m_ramCase = 0;
				AutonSteps++;
			break;
		case 6:
			break;
		}
		}
	}
	
	void Dance(){
			switch(autondance){
			case 0:     //
				if (m_lEncode > m_rEncode){
					m_driveRotate->SetAngle(61.87 - (.5 * CAMERA_VIEW));
					m_driveRotate->PIDEnable();
					if(!m_driveRotate->IsRotating()){
						autondance = 1;
						m_driveRotate->PIDDisable();
					}
				}
				else if (m_rEncode > m_lEncode){
					m_driveRotate->SetAngle(-61.87 + (.5 * CAMERA_VIEW));
					m_driveRotate->PIDEnable();
					if(!m_driveRotate->IsRotating()){
						autondance = 2;
						m_driveRotate->PIDDisable();
					}
				}
				break;

			case 1:
				m_driveRotate->SetAngle(-180 + CAMERA_VIEW);
				m_driveRotate->PIDEnable();
				if(!m_driveRotate->IsRotating()){
					autondance = 2;
					m_driveRotate->PIDDisable();
	            }
				break;

			case 2:
				m_driveRotate->SetAngle(180 - CAMERA_VIEW);
				m_driveRotate->PIDEnable();
				if(!m_driveRotate->IsRotating()){
					autondance = 1;
					m_driveRotate->PIDDisable();
	            }
				break;
			}
		}
*/

	
	
	/*********************** TELEOP FUNCTIONS **************************/
	
	void TeleopDrive()
	{
		if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.2 || fabs(m_driver->GetRawAxis(RIGHT_X)) > 0.2)
			m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),-m_driver->GetRawAxis(RIGHT_X));
		else
		{
			if (m_driver->GetRawButton(BUTTON_START)) {
				if (!m_drvStraightPID->IsEnabled())
				{
					m_drvStraightPID->SetSetpoint(32.0);
					m_drvStraightPID->Enable();
				}
			}
			else
			{
				if (m_drvStraightPID->IsEnabled())
				{
					m_drvStraightPID->Disable();
				}
				m_robotDrive->ArcadeDrive(0.0,0.0);
			}
		}
		
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
		
		if (m_driver->GetButtonPress(BUTTON_A))
			m_drvStraightPID->SetPID((m_drvStraightPID->GetP()+0.01),m_drvStraightPID->GetI(),m_drvStraightPID->GetD());
		else if (m_driver->GetButtonPress(BUTTON_B))
			m_drvStraightPID->SetPID((m_drvStraightPID->GetP()-0.01),m_drvStraightPID->GetI(),m_drvStraightPID->GetD());
		else if (m_driver->GetButtonPress(BUTTON_X))
			m_drvStraightPID->SetPID(m_drvStraightPID->GetP(),m_drvStraightPID->GetI(),(m_drvStraightPID->GetD()+0.01));
		else if (m_driver->GetButtonPress(BUTTON_Y))
			m_drvStraightPID->SetPID(m_drvStraightPID->GetP(),m_drvStraightPID->GetI(),(m_drvStraightPID->GetD()-0.01));
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
			m_catchServo1->SetAngle(30);
			m_catchServo2->SetAngle(30);
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
		if(m_operator->GetRawButton(BUTTON_BACK) && m_operator->GetRawButton(BUTTON_START))
			m_armEncoder->Reset();
		if (m_operator->GetRawButton(BUTTON_A)) {
			// Floor Picking
			m_armPIDFlag = true;
			m_armPID->SetSetpoint (FLOOR_PICKING_POS);
			m_armPID->Enable();
			
		} else if (m_operator->GetRawButton(BUTTON_LB) && m_operator->GetRawButton(BUTTON_B))
		{
			m_armPIDFlag = true;
			m_armPID->SetSetpoint (GUARDED_SHOT_BACK);
			m_armPID->Enable();
		} else if (m_operator->GetRawButton(BUTTON_B)) {
			// Medium (12ft) Shoot Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint (MED_SHOT_BACK);
			m_armPID->Enable();
			
		} else if (m_operator->GetRawButton(BUTTON_LB) && m_operator->GetRawButton(BUTTON_X))
		{
			m_armPIDFlag = true;
			m_armPID->SetSetpoint (GUARDED_SHOT_FRONT);
			m_armPID->Enable();
		} else if (m_operator->GetRawButton(BUTTON_X)) {
			// Long (18ft) Shoot Position
			m_armPIDFlag = true;
			m_armPID->SetSetpoint(MED_SHOOT_POS);
			m_armPID->Enable();
			
		} else if (m_operator->GetRawButton(BUTTON_Y) && !m_operator->GetRawButton(BUTTON_BACK)) {
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
			if (!m_operator->GetRawButton(BUTTON_BACK))
			  m_armMotor->Set(m_operator->GetRawAxis(LEFT_Y));
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
			if(m_driver->GetRawButton(BUTTON_BACK))
				m_ramServo->SetAngle(180);
			else
				m_ramServo->SetAngle(120);
		} else {
			m_ramServo->SetAngle(30);
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
	
	void ArmReset()
	{
	  double dirToZero;
	  
	  // MOVING IN NEGATIVE DIRECITON (TOWARDS BACK) *******************************
	  
	  if (m_armEncoder->GetRate() < 0)
	  {
	    if (!m_armReset->Get() && !m_armResetFlag)
	    {
	      m_armResetPos = m_armEncoder->GetDistance();
	      m_canResetArm = true;
	      m_armResetFlag = true;
	    }
	    
	    m_armResetCase = 0;
	  }
	  
	  // MOVING IN POSITIVE DIRECTION (TOWARDS FRONT) ******************************
	  
	  if (m_armEncoder->GetRate() > 0)
	  {
	    switch (m_armResetCase)
	    {
	      case 0:
		if (!m_armReset->Get())
		  m_armResetCase++;
		break;
	      case 1:
		if (m_armReset->Get())
		{
		  m_armResetPos = m_armEncoder->GetDistance();
		  m_canResetArm = true;
		  m_armResetCase = 0;
		}
		break;
	    }
	  }
	  
	  dirToZero = m_armEncoder->GetDistance() - m_armResetPos;
	  
	  // RESET VARIABLES WHEN OUTSIDE RANGE *****************************************
	  
	  if (m_armReset->Get())
	  {
	    m_armResetFlag = false;
	    m_armResetCase = 0;
	  }
	  
	  SmartDashboard::PutNumber("Direction to Arm Zero: ",dirToZero);
	  
	  // CHECK TO SEE IF STILL IN RANGE *********************************************
	  
	  if (fabs(dirToZero) > 5)
	    m_canResetArm = false;
	  
	  // RESET ARM ******************************************************************
	  
	  if (m_operator->GetRawButton(BUTTON_BACK) && m_operator->GetRawButton(BUTTON_Y)
	  {
	    if (m_canResetArm)
	    {
	      m_armEncoder->Reset();
	      m_armMotor->Set(0.0);
	      m_armResetPos = m_armEncoder->GetDistance();
	      m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"                     ");
	    }
	    else
	    {
	      if (dirToZero < 0)
		m_armMotor->Set(0.2);
	      else if (dirToZero > 0)
		m_armMotor->Set(0.2);
	      else
		m_armMotor->Set(0.0);
	    }
	  }
	  
	  // PRINT MESSAGE **************************************************************
	  
	  else if (m_canResetArm && fabs(m_armEncoder->GetDistance()) > 10)
	    m_dsLCD->Printf(DriverStationLCD::kUser_Line2,1,"RESET ARM NOW PLEASE ");
	  
	  m_dsLCD->UpdateLCD();
	}
	
	void RamrodInit(){
		if (!m_ramInit)
		{
			m_ramServo->SetAngle(30);
			m_ramMotor->Set(-0.15);
			if (m_ramTime->HasPeriodPassed(0.3))
			{
				if (abs(m_ramEncode->GetRate()) < 5)
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
	
	void AdvancedDrive(){
	    if (m_driver->GetRawButton(BUTTON_LB)){
	        m_shifters->Set(true);
	        m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),-m_driver->GetRawAxis(RIGHT_X));
	    }
	    else if (m_shifters->Get()){
	        if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.2){
	            m_robotDrive->ArcadeDrive(-m_driver->GetRawAxis(LEFT_Y),-m_driver->GetRawAxis(RIGHT_X));
	        }
	        else if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.75){
	            m_shifters->Set(false);
	        }
	    }
	    else if (!m_shifters->Get()){
	        if (fabs(m_driver->GetRawAxis(LEFT_Y)) > 0.65){
	            m_robotDrive->ArcadeDrive((2.4 * (-m_driver->GetRawAxis(LEFT_Y) - 75)) + .40,-m_driver->GetRawAxis(RIGHT_X));
	        }
	        else{
	            m_shifters->Set(true);
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
			m_ramServo->SetAngle(180);
			if(m_ramTime->HasPeriodPassed(1.0))
				m_ramCase++;
			break;
		case 2:
			m_ramServo->SetAngle(30);
			m_ramCase++;
			m_ramTime->Stop();
			m_ramTime->Start();
			m_ramTime->Reset();
			m_ramMotor->Set(0.8);
			break;
		case 3:
			if (m_ramTime->HasPeriodPassed(0.1))
			{
				if (abs(m_ramEncode->GetRate()) < 20)
				{
					m_ramMotor->Set(0.0);
					m_ramTime->Stop();
					m_ramTime->Start();
					m_ramTime->Reset();
					m_ramCase++;
				}
			}
			break;
		case 4:
			if (m_ramTime->HasPeriodPassed(0.1))
			{
				if (abs(m_ramEncode->GetDistance()) < 200)
				{
					m_ramMotor->Set(-0.15);
					m_ramCase++;
				}
				else
					m_ramMotor->Set(-0.4);
			}
			break;
		case 5:
			if (abs(m_ramEncode->GetRate()) < 5)
			{
				m_ramEncode->Reset();
				m_ramMotor->Set(0.0);
				m_ramTime->Stop();
				m_ramTime->Start();
				m_ramTime->Reset();
				m_ramCase = -1;
			}
			break;
		}
	}
		/*
		switch(m_ramCase)
		{
		case 0:
			m_ramTime->Stop();
			m_ramTime->Start();
			m_ramTime->Reset();
			m_ramCase++;
			break;
		case 1:
			m_ramServo->SetAngle(180);
			if(m_ramTime->HasPeriodPassed(0.7))
				m_ramCase++;
			break;
		case 2:
			m_ramServo->SetAngle(30);
			m_ramCase++;
			m_ramTime->Stop();
			m_ramTime->Start();
			m_ramTime->Reset();			
			break;
		case 3:
			if(m_ramTime->HasPeriodPassed(0.3))
			{
				m_ramTime->Stop();
				m_ramTime->Start();
				m_ramTime->Reset();	
				m_ramCase++;
			}
			else
				m_ramMotor->Set(1);
			break;
		case 4:
			if (m_ramTime->HasPeriodPassed(0.5))
			{
				m_ramTime->Stop();
				m_ramTime->Start();
				m_ramTime->Reset();
				m_ramCase++;
			}
			else
				m_ramMotor->Set(0.7);
			break;
		case 5:
			if (m_ramTime->HasPeriodPassed(0.15))
			{
				m_ramTime->Stop();
				m_ramTime->Start();
				m_ramTime->Reset();
				m_ramCase++;
			}
			else
				m_ramMotor->Set(-1);
			break;
		case 6:
			if (m_ramTime->HasPeriodPassed(0.3))
			{
				m_ramTime->Stop();
				m_ramTime->Start();
				m_ramTime->Reset();
				m_ramCase++;
			}
			else
				m_ramMotor->Set(-0.2);
			break;
		case 7:
			m_ramMotor->Set(0.0);
			m_ramTime->Stop();
			m_ramCase = -1;
			break;
		}
	}
	*/
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
		SmartDashboard::PutBoolean("Arm Reset Flag 1", m_armResetFlag[0]);
		SmartDashboard::PutBoolean("Arm Reset Flag 2", m_armResetFlag[1]);
		
		// Auton
		SmartDashboard::PutNumber("Auton Step: ", AutonDBSteps);
		SmartDashboard::PutBoolean("Auton Time: ", m_autonTime->Get());
		
		SmartDashboard::PutNumber("Drive PID Output: ",m_drvStraightPID->Get());
		SmartDashboard::PutNumber("Drive PID Input: ",(m_lEncode->GetDistance()+m_rEncode->GetDistance())/(2.0*REV_IN));
		SmartDashboard::PutNumber("Left Drive Set: ",m_lDrive->Get());
		SmartDashboard::PutNumber("Right Drive Set: ",m_rDrive->Get());
		SmartDashboard::PutNumber("Drive P: ",m_drvStraightPID->GetP());
		SmartDashboard::PutNumber("Drive D: ",m_drvStraightPID->GetD());
	}
};

START_ROBOT_CLASS(BuiltinDefaultCode);
