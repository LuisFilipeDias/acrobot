/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       myserial.h
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Header file for serial wrapper library.
 *
 *  @section DESCRIPTION
 *  
 *  <Add description here>
 *
 ****************************************************************************/
#ifndef MYSERIAL_H
#define MYSERIAL_H

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
#include <Arduino.h>

/* Foreign headerfiles */
#include <acrobot.h>

/* Own headerfiles */
/* None */

/***************************************************************************
 * PUBLIC DEFINITIONS 
 ****************************************************************************/
/**
* @brief Serial communications baudrate.
*/
#define SERIAL_BAUDRATE 115200

class MySerial
{
    private:
        /*************************************************************************** 
         * DECLARATION OF FUNCTIONS
         ****************************************************************************/
        /**
        * @brief Serial-related configurations.
        */
        void serialSetup(void);

    public:
        /**
        * @brief Class constructor.
        */
        MySerial(void);
};

#endif /* SERIAL_H */