/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       bluectrl.h
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Header file for bluetooth controller client.
 *
 *  @section DESCRIPTION
 *  
 *  This module handles communication with the bluetooth controller.
 *
****************************************************************************/

#ifndef BLUECTRL_H
#define BLUECTRL_H

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
#include <stdio.h>

/* Foreign headerfiles */
#include <acrobot.h>

/* Own headerfiles */
/* None */

class BlueCtrl{
    public:
        /**
        * @brief Class constructor.
        */
        BlueCtrl(void);

        /***************************************************************************
         * DECLARATION OF FUNCTIONS
         ****************************************************************************/
        /**
        * @brief Get bluetooth commands from Android controller.
        *
        * @returns Error code.
        */
        tenError enGetCtrlCmd(void);
};

#endif /* BLUECTRL_H */