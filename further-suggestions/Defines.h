// Buttons

#define BUTTON_A 1
#define BUTTON_B 2
#define BUTTON_X 3
#define BUTTON_Y 4
#define BUTTON_LB 5
#define BUTTON_RB 6
#define BUTTON_BACK 7
#define BUTTON_START 8
#define BUTTON_L3 9
#define BUTTON_R3 10

// Axes

#define LEFT_X 1
#define LEFT_Y 2
#define TRIGGERS 3
#define RIGHT_X 4
#define RIGHT_Y 5

#define PracticeBot
// #define CompetitionBot


#ifdef PracticeBot
	// Place practice bot values here
	#define FLOOR_PICKING_POS 1
	#define MED_SHOOT_POS 140
	#define LONG_SHOOT_POS -141
	#define CATCH_POS 1
	#define BGRABBER_SAFE 3
	
	// PID Config for PID
	#define ARM_P 0.01
	#define ARM_I 0.0
	#define ARM_D 0.0

	#define RAM_LOCK_POSITION 900
#endif

#ifdef CompetitionBot
	//Place competition bot values here
	#define FLOOR_PICKING_POS 1
	#define MED_SHOOT_POS 1
	#define LONG_SHOOT_POS 1
	#define CATCH_POS 1
	#define BGRABBER_SAFE 3
	
	// PID Config for PID
	#define ARM_P 0.1
	#define ARM_I 0.0
	#define ARM_D 0.0

#define RAM_LOCK_POSITION 900
#endif
