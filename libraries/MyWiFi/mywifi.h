
/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       mywifi.h
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Header file for WiFi wrapper library.
 *
 *  @section DESCRIPTION
 *  
 *  <Add description here>
 *
 ****************************************************************************/
#ifndef MYWIFI_H
#define MYWIFI_H

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
#include <stdio.h>
#include <WiFiUdp.h>

/* Foreign headerfiles */
#include <acrobot.h>

/* Own headerfiles */
#if (PLATFORM == WEMOS)
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <ESP8266mDNS.h>
#elif (PLATFORM == ESP32)
    #include <WiFi.h>
    #include <HTTPClient.h>
#endif

/***************************************************************************
 * PUBLIC DEFINITIONS 
 ****************************************************************************/
/**
* @brief Iterations before sending data to server.
*/
#define UPLOAD_COUNT 100

/**
* @brief Upload data delay.
*/
#define DELAY_UPLOAD_DATA_MS 5000

class MyWifi{
    private:        
        /*************************************************************************** 
        * DECLARATION OF VARIABLES
        ****************************************************************************/
        /* None */
        /**
        * @brief Wifi SSID & password.
        */
        const char* pchSsid;
        const char* pchWifiPassword;

        /**
        * @brief URL of the PHP page that performs the insertion -> TODO: this script must be changed in the future to accept arguments.
        */
        const char* sInsertionUrl;

        /*************************************************************************** 
        * DECLARATION OF FUNCTIONS
        ****************************************************************************/       
        /**
        * @brief Print current WiFi status.
        */
        void printWifiStatus(void);

    public:
        /**
        * @brief Class constructor.
        */
        MyWifi(void);

        /*************************************************************************** 
        * DECLARATION OF FUNCTIONS
        ****************************************************************************/
        /**
        * @brief WiFi-related configurations.
        *
        * @returns Error code.
        */
        tenError enWifiSetup(void);

        /**
        * @brief Upload the data to the server.
        *
        * @returns Error code.
        */
        tenError enUploadData(String sUrlExt);
};

#endif /* MYWIFI_H */