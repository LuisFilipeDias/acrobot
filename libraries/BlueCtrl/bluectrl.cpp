/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       bluectrl.cpp
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Implementation of bluetooth controller client.
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
#include <bluectrl.h>

/***************************************************************************
 * IMPLEMENTATION OF FUNCTIONS
 ****************************************************************************/
BlueCtrl::BlueCtrl(void)
{
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);
}

tenError BlueCtrl::enGetCtrlCmd(void)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);
    /* Will wrap around bluetooth commands.
    * A standard will have to be created for communication.
    * Not sure yet what kind of commands will be received: position? velocity?
    * May also upload the data back to the device (similarly to the upload to server functionality)
    * So many open points...must define them... */

    return enError;
}