/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       mylcd.cpp
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Implementation of lcd wrapper.
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
#include <mylcd.h>

/***************************************************************************
 * IMPLEMENTATION OF FUNCTIONS
 ****************************************************************************/
/**
* @brief Lower level LCD object.
*/
rgb_lcd oRgbLCD;

MyLCD::MyLCD(void)
{
/* Only print to LCD if no INFO or DEBUG trace level - else we check on serial information. */
#if VERBOSE_LEVEL <= VERBOSE_WARNING
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Set up the LCD's number of columns and rows. */
    oRgbLCD.begin(16, 2);

    /* Set the cursor location and RGB color. */
    oRgbLCD.setCursor(0, 0);
    oRgbLCD.setRGB(255, 0, 255);

    /* Print a message to the LCD on 2 lines. */
    oRgbLCD.print("ACROBOT v1.0");
    oRgbLCD.setCursor(0, 1);
    oRgbLCD.print("Booting...");

    /* Turn on the display: */
    oRgbLCD.display();
#else
    /* Turn off the display: */
    oRgbLCD.noDisplay();
#endif
}

tenError MyLCD::enRefresh(String sMsg)
{
/* Only print to LCD if no INFO or DEBUG trace level - else we check on serial information. */
#if VERBOSE_LEVEL <= VERBOSE_WARNING

    /* Flag to indicate line to be written. */
    static bool boFull = true;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Set a new color. */
    oRgbLCD.setRGB(255, 255, 255);

    /* If LCD is boFull, clear and reset cursor position to first line... */
    if(boFull)
    {
        oRgbLCD.clear();
        oRgbLCD.setCursor(0, 0);
        boFull = false;
    }
    /* ...else set cursor position to second line. */
    else
    {
        oRgbLCD.setCursor(0, 1);
        boFull = true;
    }
 
    /* Print the message: */
    oRgbLCD.print(sMsg);
#else
    /* Turn off the display: */
    oRgbLCD.noDisplay();
#endif

    return ERR_NONE;
}