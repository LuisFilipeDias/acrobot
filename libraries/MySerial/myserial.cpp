/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       myserial.cpp
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Implementation of serial wrapper library.
 *
 *  @section DESCRIPTION
 *  
 *  See header file for more information.
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
#include <myserial.h>

/*************************************************************************** 
 * IMPLEMENTATION OF PRIVATE FUNCTIONS
 ****************************************************************************/
void MySerial::serialSetup(void)
{
    /* Begin serial communications. */
    Serial.begin(SERIAL_BAUDRATE);

    /* Wait for serial port to connect. */
    while (!Serial);
}

/*************************************************************************** 
 * IMPLEMENTATION OF PUBLIC FUNCTIONS
 ****************************************************************************/
MySerial::MySerial(void)
{
    /* Initial setup. */
    serialSetup();
}