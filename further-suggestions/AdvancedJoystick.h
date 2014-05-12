#include "WPILib.h"

#define JOYSTICK_TIMEOUT 0.1
#define JOYSTICK_DEADBAND 0.2

class AdvancedJoystick {
public:

    // BUTTON AND AXIS ENUMS ----------
    typedef enum {
        kButtonA = 1,
        kButtonB = 2,
        kButtonX = 3,
        kButtonY = 4,
        kButtonLB = 5,
        kButtonRB = 6,
        kButtonBack = 7,
        kButtonStart = 8,
        kButtonL3 = 9,
        kButtonR3 = 10,
        kTriggerL = 11,
        kTriggerR = 12
    }button_t;

    typedef enum {
        kNone,
        kFlat,
        kQuadratic,
        kCubic
    }deadband_t;

    typedef enum {
        kLeftX = 1,
        kLeftY = 2,
        kRawTrigger = 3,
        kRightX = 4,
        kRightY = 5,
        kTriggerL = 6,
        kTriggerR = 7
    }axis_t;

    //CONSTRUCTORS --------------
    AdvancedJoystick (Joystick* gamepad);
    AdvancedJoystick (Joystick* gamepad, deadband_t deadbandType, float deadband, float timeout);
    AdvancedJoystick (Joystick* gamepad, deadband_t deadbandType);
    AdvancedJoystick (Joystick* gamepad, float deadband, float timeout);

    AdvancedJoystick (int channel, deadband_t deadbandType, float deadband, fl);
    AdvancedJoystick (int channel, deadband_t deadbandType, float deadband, float timeout);
    AdvancedJoystick (int channel, deadband_t deadbandType);
    AdvancedJoystick (int channel, float deadband, float timeout);

    //JOYSTICK ACCESS FUNCTIONS ---
    /* You can pass the enum values
     * to these functions, they're
     * typecast to ints.
     */
    bool GetRawButton (int);
    bool GetButtonPress (int);
    float GetRawAxis (int);

    //CONFIGURATION FUNCTIONS --------
    void SetPressTimeout (float);
    void SetDeadband (float);
    void SetDeadbandType (deadband_t);

    //UPDATE ---------------------------------------
    /* Note: This function is only public
     * because it needs to be accessible to
     * the object updating the joystick.
     *
     * This function updates the internal variables
     * of the AdvancedJoystick, and does not grab
     * any new data from the driver station for
     * uses other than this.
     *
     * Use only once per CPU loop.
     */
    void update ();

private:
    // INTERNAL FUNCTIONS ------
    //Deadband
    float applyDeadband (float);
    float applyDeadbandFlat (float);
    float applyDeadbandQuad (float);
    float applyDeadbandCube (float);

    void trackTimer();

    // MEMBER OBJECTS --------
    Joystick* m_gamepad;
    Timer* m_timer;
    float m_buttonTimeout;
    float m_deadband;
    deadband_t m_deadbandType;
};
