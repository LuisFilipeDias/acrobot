/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       mywifi.cpp
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Implementation of WiFi wrapper library.
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
#include <mywifi.h>

/*************************************************************************** 
 * IMPLEMENTATION OF PRIVATE FUNCTIONS
 ****************************************************************************/
void MyWifi::printWifiStatus(void)
{
    long lRssi;
    
    /* SSID of the network attached to. */
    printInfo("\nSSID: ");
    Serial.print(WiFi.SSID());

    /* WiFi shield's IP address. */
    IPAddress xIp = WiFi.localIP();
    printInfo("\nIP Address: ");
    Serial.print(xIp);

    /* Received signal strength. */
    lRssi = WiFi.RSSI();
    printInfo("\nsignal strength (RSSI):");
    printInfo("%ld", lRssi);
    printInfo(" dBm\n\n");
}

/*************************************************************************** 
 * IMPLEMENTATION OF PUBLIC FUNCTIONS
 ****************************************************************************/
MyWifi::MyWifi()
{
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    //pchSsid = "Vodafone-DE0235";
    //pchWifiPassword = "FD91B1958E";
    // pchSsid = "MyAquaris";
    // pchWifiPassword = "luisdias";
    // pchWifiPassword = "CASA LT";
    pchWifiPassword = WIFI_PASSWORD;
    // pchSsid = "CSW-Guest";
    // pchWifiPassword = "Critical98";
    // pchSsid = "Sidonio";
    // pchWifiPassword = "margaride12";

    sInsertionUrl = "http://acrobotsensors.000webhostapp.com/insert.php";
}

tenError MyWifi::enWifiSetup(void)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

#if WIFI_SUPPORT == YES
    char chTries = 0;

    /* Select WiFi mode. */
    if (!WiFi.mode(WIFI_STA))
    {
        enError = ERR_WIFI_SETUP;
        printError("\nERROR: Selecting Wifi mode.");
    }
    /* Connect to WiFi with SSID and password. */
    else if (!WiFi.begin(pchSsid, pchWifiPassword))
    {
        enError = ERR_WIFI_CONNECTION;
        printError("\nERROR: Attempting Wifi connection.");
    }
    else
    {
        /* While waiting for connection to be successful...*/
        while (WiFi.waitForConnectResult() != WL_CONNECTED)
        {

            /* Attempt connection for a limited number of times. */
            if (chTries < WIFI_CONNECTION_RETRIES)
            {
                /* Increment the trying counter. */
                chTries ++;

                /* Warn the user. */
                printWarning("\nWARNING: Connection failed[%d]. Trying again...", 6-chTries);

                delay(DELAY_WIFI_CONNECTION_RETRY_MS);
            }
            else
            {
                enError = ERR_WIFI_CONNECTION_TIMEOUT;
                
                printError("\nERROR: Connection failed %d times. Aborting!\n", WIFI_CONNECTION_RETRIES);
                
                delay(DELAY_WIFI_CONNECTION_RETRY_MS);
                
                break;
            }
        }
    }

    if (ERR_NONE == enError)
    {
        /* Finalize by printing WiFi status. */
        printWifiStatus();
    }
#endif
    // return enError;
    return ERR_NONE;
}

tenError MyWifi::enUploadData(String sUrlExt)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    String sNewUrl = sInsertionUrl + sUrlExt;

    /* Check connection. */
    if(WiFi.status()== WL_CONNECTED)
    {
          HTTPClient xHttp;

          /* Configure server and URL. */
          xHttp.begin(sNewUrl);

          /* Start connection and send HTTP header. */
          int iHttpCode = xHttp.GET();

          /* HttpCode will be negative on error. */
          if(iHttpCode > 0)
          {
               /* HTTP header has been send and Server response header has been handled. */
               printError("[HTTP] GET... code: %d\n", iHttpCode);

               /* File found at server. */
               if(iHttpCode == HTTP_CODE_OK)
               {
                    String sPayload = xHttp.getString();
                    Serial.print(sPayload);
               }
          } 
          else
          {
               printError("\n[HTTP] GET... failed, error: %s", xHttp.errorToString(iHttpCode).c_str());
          }
          xHttp.end();
     }
    return enError;
}