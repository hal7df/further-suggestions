#include "WPILib.h"

#define JOYSTICK_TIMEOUT 0.1

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
        kLinear,
        kQuadratic
    }deadband_t;

    typedef enum {
        kLeftX = 1,
        kLeftY = 2,
        kRawTrigger = 3,
        kRightX = 4,
        kRightY = 5,
        kLeftTrigger = 6,
        kRightTrigger = 7
    }axis_t;

    //CONSTRUCTORS --------------
    AdvancedJoystick (Joystick*);
    AdvancedJoystick (int);

    //JOYSTICK ACCESS FUNCTIONS ---
    bool GetRawButton (button_t);
    bool GetButtonPress (button_t);
    float GetRawAxis (axis_t);

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
    float adjust (float);
    void trackTimer();

    // MEMBER OBJECTS --------
    Joystick* m_gamepad;
    Timer* m_timer;
    float m_buttonTimeout;
    deadband_t m_deadbandType;
};
