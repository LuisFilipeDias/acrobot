/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       mysensors.h
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Header file for sensors library.
 *
 *  @section DESCRIPTION
 *  
 *  This module handles the interaction with the sensors. The list of sensors is the following:
 *    - SHARP GP2D12
 *    - Sonar LZ MAXSONAR EZ
 *    - Grove Sound Sensor v1.6
 *    - Grove Gas Sensor v1.4 (MQ2)
 *    - Grove Air Quality Sensor v1.2
 *    - Grove Temperature Sensor v1.2
 *    - Grove PIR Motion Sensor v1.2
 *
 *  A state machine was created to read each sensor at a time.
 *  This implementation was made to run as a standalone task in a multithreading environment.
 *
 ****************************************************************************/
#ifndef MYSENSORS_H
#define MYSENSORS_H

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
#include <stdio.h>
#include <Arduino.h>

/* Foreign headerfiles */
#include <acrobot.h>
#ifdef HAS_AIRQ
    #include <AirQuality.h>
#endif

#ifdef HAS_MPU
    #include "MPU6050_6Axis_MotionApps20.h"
#endif

/* Own headerfiles */
/* None */

/***************************************************************************
 * PUBLIC DEFINITIONS 
 ****************************************************************************/
/**
* @brief Thermistor beta value.
*/
#define NTC_BETA     4275

/**
* @brief Thermistor resistance at zero degrees.
*/
#define NTC_R0       100000

/**
* @brief Thermistor fTemperature calculation constants.
*/
#define NTC_K1       298.15
#define NTC_K2       273.15

/**
* @brief ADC Maximum value.
*/
#define ADC_MAX       4095.0

/**
* @brief Structure with all the sensors information.
*/
typedef struct{
    byte   bNoise;
    float  flGas;
    bool   boMovement;
    int    iAirQual;
    String sAirQual;
    int    iDistSharp;
    float  aflDistSonar[E_SONAR_COUNT];
    float  flTemperature;
    float  flAnglePitch;
} tstSensors;

class MySensors{
    private:
        /*************************************************************************** 
         * DECLARATION OF FUNCTIONS
         ****************************************************************************/
        /**
        * @brief Read Sonar sensor.
        *
        * @returns Error code.
        */
        tenError enReadSonar(char chSonarID);

        /**
        * @brief Read Sharp sensor.
        *
        * @returns Error code.
        */
        tenError enReadSharp(void);

        /**
        * @brief Read PIR sensor.
        *
        * @returns Error code.
        */
        tenError enReadPIR(void);

        /**
        * @brief Read Temperature sensor.
        *
        * @returns Error code.
        */
        tenError enReadTemp(void);

        /**
        * @brief Read Air Quality sensor.
        *
        * @returns Error code.
        */
        tenError enReadAirQuality(void);

        /**
        * @brief Read Gas sensor.
        *
        * @returns Error code.
        */
        tenError enReadGasSensor(void);

        /**
        * @brief Read Noise sensor.
        *
        * @returns Error code.
        */
        tenError enReadNoise(void);

        /**
        * @brief Read MPU sensor.
        *
        * @returns Error code.
        */
        tenError enReadMPU(void);

        /**
        * @brief Setup MPU sensor.
        *
        * @returns Error code.
        */
        tenError enSetupMPU(void);

    public:
        /**
        * @brief Class constructor.
        */
        MySensors(void);

        /*************************************************************************** 
         * DECLARATION OF VARIABLES
         ****************************************************************************/
        /**
        * @brief Collection of sensors data.
        */
        tstSensors stSensors;

        /*************************************************************************** 
         * DECLARATION OF FUNCTIONS
         ****************************************************************************/
        /**
        * @brief Update Air Quality sensor measurement -> triggered by a 2s timer.
        *
        * @returns Error code.
        */
        tenError enReadAirQualityUpdate(void);

        /**
        * @brief Get sonar A readings (for action over the motors).
        *
        * @returns Sonar A distance value.
        */
        float flGetSonarA(void);

        /**
        * @brief Get sonar B readings (for action over the motors).
        *
        * @returns Sonar B distance value.
        */
        float flGetSonarB(void);

        /**
        * @brief Get sonar C readings (for action over the motors).
        *
        * @returns Sonar C distance value.
        */
        float flGetSonarC(void);

        /**
        * @brief Get MPU pitch angle.
        *
        * @returns Angle in degrees.
        */
        float flGetAnglePitch(void);

        /**
        * @brief The sensors state machine, that iterates over every sensor read function.
        *
        * @returns Error code.
        */
        tenError enProcessSensors(void);

        /**
        * @brief Build a display message to be used with an LCD.
        *
        * @returns String built from the sensors information selected.
        */
        String sBuildMsg(void);

        /**
        * @brief Build a message to be sent to a database, via the url of a GET message.
        *
        * @returns String built from all the sensors information.
        */
        String sBuildUrl(void);

};
#endif /* MYSENSORS_H */