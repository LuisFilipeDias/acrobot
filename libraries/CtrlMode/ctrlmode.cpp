/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       ctrlmode.cpp
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       10-11-2017
 *
 *  @version    2.0
 *  
 *  @brief      Implementation of control modes - autonomous algorythm and manual remote control.
 *
 *  @section DESCRIPTION
 *  
 * ...
 *
****************************************************************************/

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
/* None */

/* Foreign headerfiles */
/* None */

/* Own headerfiles */
#include <ctrlmode.h>

/* Collection of motor objects. */
Motors        oMotorLeft  (E_MOTOR_LEFT, MOTOR_UNIT_A, MOTOR_TIMER_A, MOTOR_PWMFWD_A, MOTOR_PWMBWD_A, E_PIN_MOTORFWD_A, E_PIN_MOTORBWD_A);
Motors        oMotorRight (E_MOTOR_RIGHT, MOTOR_UNIT_B, MOTOR_TIMER_B, MOTOR_PWMFWD_B, MOTOR_PWMBWD_B, E_PIN_MOTORFWD_B, E_PIN_MOTORBWD_B);
Motors        oMotors[E_MOTOR_COUNT] = {oMotorLeft, oMotorRight};

bool boIsWall = false;
bool boIsWallToggle = true;

double dblOriSetpoint = 1.5;
double dblSetpoint = dblOriSetpoint;
double dblInputAngle, dblOutputSpeed;

//adjust these values to fit your own design
double Kp = 10;
double Kd = 0;
// double Kd = 1.4;
double Ki = 0;
// double Ki = 60;

PID oPID(&dblInputAngle, &dblOutputSpeed, &dblSetpoint, Kp, Ki, Kd, DIRECT);

/***************************************************************************
 * C IMPLEMENTATION OF PRIVATE FUNCTIONS
 ****************************************************************************/
tenError CtrlMode::enMotorsWrapper(float flSpeed[E_MOTOR_COUNT])
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Adjust left motor speed. */
    // flSpeed[E_MOTOR_LEFT] = flSpeed[E_MOTOR_LEFT] * flAdj;

    /* Stop all motors. */
    for (int i = 0; i < E_MOTOR_COUNT; i++)
    {
        char chDir[E_MOTOR_COUNT];

        /* Adjust direction according to power. */
        chDir[i] = (flSpeed[i] > 0) ? MOTOR_FWD : MOTOR_BWD;

        /* Set power to absolute value. */
        flSpeed[i] = (flSpeed[i] > 0) ? flSpeed[i] : (-1) * flSpeed[i];

        /* Act on respective motor. */
        enError = oMotors[i].enConfigMotor(flSpeed[i], chDir[i]);

        /* Failed to config a motor, abort. */
        if (ERR_NONE != enError)
        {
            break;
        }
    }
    return enError;
}

/***************************************************************************
 * C IMPLEMENTATION OF PUBLIC FUNCTIONS
 ****************************************************************************/
CtrlMode::CtrlMode(void)
{
    /* Must rethink this. */
    #if 0
    /* Left-right motor adjustment. */
    flAdj = 1 + (float)(MOTOR_ADJ_PC)/100;
    #endif

    /* Setup oPID. */
    oPID.SetMode(AUTOMATIC);
    oPID.SetSampleTime(10);
    oPID.SetOutputLimits(-255, 255); 
}

tenError CtrlMode::enSetCtrl(float flDirection)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    float aflSpeed[E_MOTOR_COUNT];

    /* flDirection is a [-1, 1] variable that defines the direction. */
    /* Left is 0% to 200%. */
    aflSpeed[E_MOTOR_LEFT]  = dblOutputSpeed * (1 + flDirection);

    /* Right is 200% to 0%. */
    aflSpeed[E_MOTOR_RIGHT] = dblOutputSpeed * (1 - flDirection);

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Get Pid output but also get direction. */

    enError = CtrlMode::enMotorsWrapper(aflSpeed);

    return enError;
}

tenError CtrlMode::enSetPID(float flAngle, float flSpeed)
{
    /* Raw mechanism to convert speed to angle setpoint - speed [0,100] directly converted to angle. */
    dblSetpoint = flSpeed;

    dblInputAngle = (double) flAngle;
    printf("\nIN -> dblInputAngle: %4.4f", dblInputAngle);

    /* Set oPID and save oPID configuration here. */
    oPID.Compute();

    printf("\tOUT -> dblOutputSpeed: %4.4f", dblOutputSpeed);
}