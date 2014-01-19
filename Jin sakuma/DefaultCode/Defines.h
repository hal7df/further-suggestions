// ----- Joystick -----
// Button
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

// Axis
#define LEFT_X 1
#define LEFT_Y 2
#define TRIGGERS 3
#define RIGHT_X 4
#define RIGHT_Y 5

// ----- Drive -----
// Victor
#define lDRIVE 1
#define rDRIVE 2
#define LAUNCHER1 4
#define LAUNCHER2 5
#define LAUNCHER3 6
#define FEEDER 7
#define PLATE1 8
#define PLATE2 10

// Relay
#define CLIMBRACHET 1

// ----- Sensors -----
// Encoder
#define lDRIVE_ENCODER1 1
#define lDRIVE_ENCODER2 2
#define rDRIVE_ENCODER1 3
#define rDRIVE_ENCODER2 4

// Potentiometer
#define PLATE_POTEN 1
#define CLIMBER_POTEN

// Limit Switch
#define CLIMBER_LIM_SWITCH
#define FEEDER_LIM_SWITCH


// We are using Competition Robot.
//#define PracticeBot
#define CompetitionBot

// Def for Practice Robot
#ifdef PracticeBot
        #define PLATE_P -0.05
        #define PLATE_PYRAMID_THREE_POINT 626.0
        #define PLATE_PYRAMID_AUTON 626.0
        #define PLATE_FEEDER_THREE_POINT 592
        #define PLATE_FEEDER_TWO_POINT 581
        #define PLATE_TEN_POINT_CLIMB 282
        #define PLATE_CLOSE_SHOT 648
        #define PLATE_LOW_LIMIT 162
        #define PLATE_HIGH_LIMIT 686
#endif

// Def for Competition Robot
#ifdef CompetitionBot
        #define PLATE_P -0.03
        #define PLATE_PYRAMID_THREE_POINT 623.0 //589.0
        #define PLATE_PYRAMID_AUTON 631.0
        #define PLATE_FEEDER_THREE_POINT 587.0
        #define PLATE_FEEDER_TWO_POINT 558.0
        #define PLATE_TEN_POINT_CLIMB 298.0
        #define PLATE_LOW_LIMIT 1.18
        #define PLATE_HIGH_LIMIT 4.0
        #define PLATE_CLOSE_SHOT 3.8
#endif
