/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       ctrlmode.h
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       10-11-2017
 *
 *  @version    2.0
 *  
 *  @brief      Header file for control modes - autonomous algorythm and manual remote control.
 *
 *  @section DESCRIPTION
 *  
 *  ...
 *
****************************************************************************/

#ifndef CTRLMODE_H
#define CTRLMODE_H

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
#include <stdio.h>
#include <stdlib.h>

/* Foreign headerfiles */
#include <acrobot.h>
#include <motors.h>
#include <PID_v1.h>

/* Own headerfiles */
/* None */

/**
* @brief List of manual control mode states.
*/
typedef enum
{
    E_M_START                             = 0,
    E_M_FWD                               = 1,
    E_M_RANDOM                            = 2,
    E_M_OBS_GROUND                        = 3,
    E_M_OBS_WALL                          = 4,
    E_M_GO_MANUAL                         = 5,

    E_M_COUNT                             = 6
} tenCtrlStates;

class CtrlMode{
    private:

        /**
        * @brief Wrapper of motors: single point of configuration for all 3 motors.
        *
        * @param[flSpeed[E_M_COUNT]]  motor power array;
        *
        * @returns Error code.
        */
        tenError enMotorsWrapper(float flSpeed[E_M_COUNT]);

    public:
        /***************************************************************************
         * DECLARATION OF FUNCTIONS
         ****************************************************************************/
        /**
        * @brief Class constructor.
        */
        CtrlMode(void);

        /**
        * @brief Configure manual control mode.
        *
        * @param[flDirection]          direction of wheels (right/left);
        *
        * @returns Error code.
        */
        tenError enSetCtrl(float flDirection);

        /**
        * @brief Setup the PID with angle info.
        *
        * @param[flAngle]           desired agle to maintain;
        * @param[flActualSpeed]     current speed;
        * @param[flDesiredSpeed]    actual speed;
        *
        * @returns Error code.
        */
        tenError enSetPID(float flAngle, float flActualSpeed, float flDesiredSpeed);
};

#endif /* CTRLMODE_H */
