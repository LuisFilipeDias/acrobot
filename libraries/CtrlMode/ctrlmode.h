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

/* Foreign headerfiles */
#include <acrobot.h>
#include <motors.h>

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
        * @brief Right/Left motor adjustment.
        */
        float flAdj;

        /**
        * @brief Motors object references.
        */
        // Motors *oMotors[E_M_COUNT];


        /**
        * @brief Wrapper of motors: single point of configuration for all 3 motors.
        *
        * @param[flSpeed[E_M_COUNT]]  motor power array;
        *
        * @returns Error code.
        */
        tenError enMotorsWrapper(float flSpeed[E_M_COUNT]);

        /**
        * @brief Configure manual control mode.
        *
        * @param[flSpeed[E_M_COUNT]]  motor power array;
        *
        * @returns Error code.
        */
        tenError enManualCtrl(float flSpeed[E_M_COUNT]);

        /**
        * @brief Configure manual control mode.
        *
        * @param[flSenseFront]       front sensor reading;
        * @param[flSenseBottom]      bottom sensor reading;
        *
        * @returns Error code.
        */
        tenError enAutoCtrl(float flSenseFront, float flSenseBottom);

        /**
        * @brief Set motors for mode FWD.
        *
        * @returns Error code.
        */
        tenError enModeFwd(void);

        /**
        * @brief Set motors for mode RANDOM.
        *
        * @returns Error code.
        */
        tenError enModeRandom(void);

        /**
        * @brief Set motors for mode GROUND IN SIGHT.
        *
        * @returns Error code.
        */
        tenError enModeGround(void);

        /**
        * @brief Set motors for mode WALL IN SIGHT.
        *
        * @returns Error code.
        */
        tenError enModeWall(void);

        /**
        * @brief Go manual from auto.
        *
        * @returns Error code.
        */
        tenError enGoManual(void);

    public:
        /***************************************************************************
         * DECLARATION OF FUNCTIONS
         ****************************************************************************/
        /**
        * @brief Class constructor.
        *
        * @param[oLocMotors]        motors object array;
        */
        CtrlMode(void);//Motors oLocMotors[E_M_COUNT]);

        /**
        * @brief Configure manual control mode.
        *
        * @param[boIsAuto]               change of mode toggler;
        * @param[flSpeed[E_M_COUNT]]     motor power array;
        * @param[flSenseFront]           front sensor distance;
        * @param[flSenseBottom]          bottom sensor distance;
        *
        * @returns Error code.
        */
        tenError enSetCtrl(bool boisAuto, float flSpeed[E_M_COUNT], float flSenseFront, float flSenseBottom);

};

#endif /* CTRLMODE_H */
