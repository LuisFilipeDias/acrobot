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

/*
dblMeasuredSpeed -> [  PID  ] -> dblSetpointAngle
dblSetpointSpeed -> [       ]

dblMeasuredAngle -> [  PID  ] -> dblOutputMotorPower
dblSetpointAngle -> [       ]
*/

double dblMeasuredSpeed;
double dblOriSetpointSpeed = 0;
double dblSetpointSpeed = dblOriSetpointSpeed;
double dblSetpointAngle;
double dblMeasuredAngle, dblOutputMotorPower;

/* Must gradually adjust these values: start by KpAngle, then KiAngle finally KdAngle - search for this info somewhere. */
double KpAngle = 10;
double KdAngle = 0;
// double KdAngle = 1.4;
double KiAngle = 0;
// double KiAngle = 60;

double KpSpeed = 10;
double KdSpeed = 0;
double KiSpeed = 0;

PID oPIDSpeed(&dblMeasuredSpeed, &dblSetpointAngle, &dblSetpointSpeed, KpSpeed, KiSpeed, KdSpeed, DIRECT);
PID oPIDAngle(&dblMeasuredAngle, &dblOutputMotorPower, &dblSetpointAngle, KpAngle, KiAngle, KdAngle, DIRECT);

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

    /* Setup oPIDAngle. */
    oPIDAngle.SetMode(AUTOMATIC);
    oPIDAngle.SetSampleTime(10);
    oPIDAngle.SetOutputLimits(-255, 255); 
}

tenError CtrlMode::enSetCtrl(float flDirection)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    float aflSpeed[E_MOTOR_COUNT];

    /* flDirection is a [-1, 1] variable that defines the direction. */
    /* Left is 0% to 200%. */
    aflSpeed[E_MOTOR_LEFT]  = dblOutputMotorPower * (1 + flDirection);

    /* Right is 200% to 0%. */
    aflSpeed[E_MOTOR_RIGHT] = dblOutputMotorPower * (1 - flDirection);

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Get Pid output but also get direction. */

    enError = CtrlMode::enMotorsWrapper(aflSpeed);

    return enError;
}

tenError CtrlMode::enSetPID(float flAngle, float flActualSpeed, float flDesiredSpeed)
{
    /* Save the actual speed. */
    dblMeasuredSpeed = (double) flActualSpeed;

    /* Save the setpoint speed. */
    dblSetpointSpeed = (double) flDesiredSpeed;

    /* Calculate the Angle set point, based on the speed. */
    oPIDSpeed.Compute();

    /* Save the setpoint angle. */
    dblMeasuredAngle = (double) flAngle;

    /* Calculate the motor power based on the angle difference. */
    oPIDAngle.Compute();

    // printf("\tOUT -> dblOutputMotorPower: %4.4f", dblOutputMotorPower);
}