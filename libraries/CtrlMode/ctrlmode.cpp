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

/* Must rethink this. */
#if 0
Motors        oMotorLeft  (E_MOTOR_LEFT, MOTOR_UNIT_A, MOTOR_TIMER_A, MOTOR_PWMFWD_A, MOTOR_PWMBWD_A, E_PIN_MOTORFWD_A, E_PIN_MOTORBWD_A);
Motors        oMotorRight (E_MOTOR_RIGHT, MOTOR_UNIT_B, MOTOR_TIMER_B, MOTOR_PWMFWD_B, MOTOR_PWMBWD_B, E_PIN_MOTORFWD_B, E_PIN_MOTORBWD_B);
Motors        oMotorBottom(E_MOTOR_BOTTOM, MOTOR_UNIT_C, MOTOR_TIMER_C, MOTOR_PWMFWD_C, MOTOR_PWMBWD_C, E_PIN_MOTORFWD_C, E_PIN_MOTORBWD_C);
Motors        oMotors[E_MOTOR_COUNT] = {oMotorLeft, oMotorRight, oMotorBottom};

bool boIsWall = false;
bool boIsWallToggle = true;
#endif

double dblOriSetpoint = 173;
double dblSetpoint = dblOriSetpoint;
double dblInput, dblOutput;

//adjust these values to fit your own design
double Kp = 50;   
double Kd = 1.4;
double Ki = 60;

PID oPID(&dblInput, &dblInput, &dblSetpoint, Kp, Ki, Kd, DIRECT);

/***************************************************************************
 * C IMPLEMENTATION OF PRIVATE FUNCTIONS
 ****************************************************************************/
tenError CtrlMode::enMotorsWrapper(float flSpeed[E_MOTOR_COUNT])
{
    /* Must rethink this. */
    #if 0
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Adjust left motor speed. */
    flSpeed[E_MOTOR_LEFT] = flSpeed[E_MOTOR_LEFT] * flAdj;

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
    #endif
}

tenError CtrlMode::enManualCtrl(float flSpeed[E_MOTOR_COUNT])
{
    /* Must rethink this. */
    #if 0
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    return CtrlMode::enMotorsWrapper(flSpeed);
    #endif
}

tenError CtrlMode::enAutoCtrl(float flSenseFront, float flSenseBottom)
{
    /* Must rethink this. */
    #if 0
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    /* Initial state is init. */
    static tenCtrlStates enState = E_M_START;

    static int iFwdTime = 0, iRandomTime = 0;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);
   
    /* If ground is getting close...go up, else stay put. */
    if (flSenseBottom < SENSE_BOTTOM)
    {
        enState = E_M_OBS_GROUND;
    }
    /* If there is an obstacle in defined horizontal distance. */
    else if (flSenseFront < SENSE_FRONT)
    {
        enState = E_M_OBS_WALL;
    }

    /* Main state machine. */
    switch(enState)
    {
        case E_M_START:
            enState = E_M_FWD;
            break;
        case E_M_FWD:
            enError = CtrlMode::enModeFwd();
            /* Check if 5 seconds have passed. */
            if (xTaskGetTickCount() - iFwdTime > TIME_10_SECONDS_MS)
            {
                iRandomTime = xTaskGetTickCount();
                enState = E_M_RANDOM;
            }
            break;
        case E_M_RANDOM:
            enError = CtrlMode::enModeRandom();
            /* Check if 2 seconds have passed. */
            if (xTaskGetTickCount() - iRandomTime > TIME_2_SECONDS_MS)
            {
                iFwdTime = xTaskGetTickCount();
                enState = E_M_FWD;
            }
            break;
        case E_M_OBS_GROUND:
            enError = CtrlMode::enModeGround();
            iFwdTime = xTaskGetTickCount();
            enState = E_M_FWD;
            break;
        case E_M_OBS_WALL:
            enError = CtrlMode::enModeWall();
            iFwdTime = xTaskGetTickCount();
            enState = E_M_FWD;
            break;
        default:
            enState = E_M_START;
            break;
    }

    return enError;
    #endif
}

tenError CtrlMode::enModeFwd(void)
{
    /* Must rethink this. */
    #if 0
    float flSpeedZeros[E_MOTOR_COUNT] = {0, 0, 0};
    float flSpeedFwd[E_MOTOR_COUNT] = {MOTOR_AUTO_FWD, MOTOR_AUTO_FWD2, 0};
    float flSpeedWall[E_MOTOR_COUNT] = {-MOTOR_AUTO_FWD, MOTOR_AUTO_FWD2, 0};
    float flSpeedWallInv[E_MOTOR_COUNT] = {MOTOR_AUTO_FWD, -MOTOR_AUTO_FWD2, 0};

    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    static char chCnt = 0;

    chCnt++;

    if(boIsWall)
    {
        if(chCnt == 3)
        {
            chCnt = 0;
            boIsWall = false;
            boIsWallToggle = ! boIsWallToggle;
            enError = CtrlMode::enMotorsWrapper(flSpeedZeros);
        }
        else
        {
            if(boIsWallToggle)
            {
                enError = CtrlMode::enMotorsWrapper(flSpeedWallInv);
            }
            else
            {
                enError = CtrlMode::enMotorsWrapper(flSpeedWall);
            }
        }
    }
    else
    {
        enError = CtrlMode::enMotorsWrapper(flSpeedFwd);
    }

    return enError;
    #endif
}

tenError CtrlMode::enModeRandom(void)
{
    /* Must rethink this. */
    #if 0
    /* TODO: got to randomize this. */
    float flSpeed[E_MOTOR_COUNT] = {0, 0, 0};
    
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Go Random. */
    return CtrlMode::enMotorsWrapper(flSpeed);
    #endif
}

tenError CtrlMode::enModeGround(void)
{
    /* Must rethink this. */
    #if 0
    float flSpeed[E_MOTOR_COUNT] = {0, 0, MOTOR_AUTO_GROUND};

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    return CtrlMode::enMotorsWrapper(flSpeed);
    #endif
}

tenError CtrlMode::enModeWall(void)
{
    /* Must rethink this. */
    #if 0
    float flSpeedZeros[E_MOTOR_COUNT] = {0, 0, 0};
    float flSpeedFwd[E_MOTOR_COUNT] = {MOTOR_AUTO_FWD, MOTOR_AUTO_FWD2, 0};
    float flSpeedWall[E_MOTOR_COUNT] = {-MOTOR_AUTO_FWD, MOTOR_AUTO_FWD2, 0};
    float flSpeedWallInv[E_MOTOR_COUNT] = {MOTOR_AUTO_FWD, -MOTOR_AUTO_FWD2, 0};

    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    boIsWall = true;

    if(boIsWallToggle)
    {
        enError = CtrlMode::enMotorsWrapper(flSpeedWall);
    }
    else
    {
        enError = CtrlMode::enMotorsWrapper(flSpeedWallInv);
    }

    return enError;
    #endif
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

tenError CtrlMode::enSetCtrl(bool boIsAuto, float flSpeed[E_MOTOR_COUNT], float flSenseFront, float flSenseBottom)
{
    /* Must rethink this. */
    #if 0
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    if (boIsAuto)
    {
        enError = CtrlMode::enAutoCtrl(flSenseFront, flSenseBottom);
    }    
    else
    {
        enError = CtrlMode::enManualCtrl(flSpeed);
    }

    return enError;
    #endif
}

tenError CtrlMode::enSetPID(float flAngle)
{
    dblInput = (double) flAngle;

    /* Set oPID and save oPID configuration here. */
    oPID.Compute();
}