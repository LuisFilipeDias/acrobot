/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       mylcd.h
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Header file for lcd wrapper.
 *
 *  @section DESCRIPTION
 *  
 *  <Add description here>
 *
 ****************************************************************************/
#ifndef MYLCD_H
#define MYLCD_H

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
#include <stdio.h>

/* Foreign headerfiles */
#include <acrobot.h>
#include <rgb_lcd.h>

/* Own headerfiles */
/* None */

class MyLCD{
    public:
        /**
        * @brief Class constructor.
        */
        MyLCD(void);

        /***************************************************************************
         * DECLARATION OF FUNCTIONS
         ****************************************************************************/
        /**
        * @brief Refresh information on LCD.
        *
        * @param[char] msg     message to be displayed in the LCD;
        *
        * @returns Error code.
        */
        tenError enRefresh(String sMsg);
};

#endif /* MYLCD_H */