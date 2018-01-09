/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       mysensors.cpp
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Implementation of sensors library.
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
#include "mysensors.h"

#include "Maxbotix.h"

Maxbotix rangeSensorPWA(E_PIN_SONAR_A, Maxbotix::PW, Maxbotix::LV, Maxbotix::SIMPLE);
Maxbotix rangeSensorPWB(E_PIN_SONAR_C, Maxbotix::PW, Maxbotix::LV, Maxbotix::SIMPLE);
Maxbotix rangeSensors[E_SONAR_COUNT-1] = {rangeSensorPWA, rangeSensorPWB};

/***************************************************************************
 * LOCAL VARIABLES 
 ****************************************************************************/
/**
* @brief Flag for boMovement detection.
*/
bool boMovement = false;

#ifdef HAS_AIRQ
    /**
    * @brief Quality sensor object.
    */
    AirQuality oAirQualitySensor;
#endif

/**
* @brief Holds the quality measured and calculated.
*/
int iCurrentQuality = -1;

/***************************************************************************
 * DECLARATION OF ISRs (MUST BE LOCAL) 
 ****************************************************************************/
/**
* @brief Internal ISR for PIR pin transitions.
*/
void __int_readPIR__(void);

/***************************************************************************
 * IMPLEMENTATION OF PRIVATE FUNCTIONS
 ****************************************************************************/
void __int_readPIR__(void)
{
    /* If interrupt was triggered, means movement was detected. */
    boMovement = true;
}

/***************************************************************************
 * IMPLEMENTATION OF PRIVATE FUNCTIONS
 ****************************************************************************/
tenError MySensors::enReadSharp(void)
{
#ifdef HAS_SHARP
    int enReadSharp = 0;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Sample value from 32 readings. */
    for(int i=0; i<32; i++)
    {
        enReadSharp += analogRead(E_PIN_SHARP);
    }
 
    /* Divide by 32. */
    enReadSharp >>= 5;

    /* If invalid reading, signal it... */
    if (enReadSharp < 3)
    {
        /* TODO: Add mutex here. */
        stSensors.iDistSharp = -1;
    }
    /* ...else, use the formula to convert and save the actual value. */
    else
    {
        /* TODO: Add mutex here. */
        stSensors.iDistSharp = (6787.0 /((float)enReadSharp - 3.0)) - 4.0;
    }
    
    /* If outside iDistSharp, set to invalid. */
    if(stSensors.iDistSharp < 10 || stSensors.iDistSharp > 80)
    {
        /* TODO: Add mutex here. */
        stSensors.iDistSharp = -1;
    }

    printInfo("\nDistance Sharp: %dcm", stSensors.iDistSharp);

    return ERR_NONE;
#else
    printWarning("\nenReadSharp NOT_IMPLEMENTED");
    return ERR_NONE;
#endif
}

tenError MySensors::enReadSonar(char chSonarID)
{
#ifdef HAS_SONAR
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    stSensors.aflDistSonar[chSonarID] = rangeSensors[chSonarID].getRange();

    /* TODO: Add mutex here. */
    /* Limit maximum distance to 150 cm. */
    stSensors.aflDistSonar[chSonarID] = (stSensors.aflDistSonar[chSonarID] > 150) ? 150 : stSensors.aflDistSonar[chSonarID];

    printInfo("\nDistance Sonar %d: %2.2fcm", chSonarID, stSensors.aflDistSonar[chSonarID]);

    return ERR_NONE;
#else
    printWarning("\nenReadSonar NOT_IMPLEMENTED");
    return ERR_NONE;
#endif
}

tenError MySensors::enReadPIR(void)
{
#ifdef HAS_PIR
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Set the local structure flag according to detection. */
    if (boMovement)
    {
    /* TODO: Add mutex here. */
        stSensors.boMovement = false;
    }
    else
    {
    /* TODO: Add mutex here. */
        stSensors.boMovement = true;
    }

    printInfo("\nMovement: %s", (stSensors.boMovement ? "False" : "True"));

    return ERR_NONE;
#else
    printWarning("\nenReadPIR NOT_IMPLEMENTED");
    return ERR_NONE;
#endif
}

tenError MySensors::enReadTemp(void)
{
#ifdef HAS_TEMP
    float fR;
    int iReadTemp = 0;

    /* B value of the thermistor. */
    const int B = NTC_BETA;

    /* R0 = 100k.`*/
    const int fR0 = NTC_R0;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Sample value from 32 readings. */
    for(int i=0; i<32; i++)
    {
        iReadTemp += analogRead(E_PIN_TEMP);
    }
 
    /* Divide by 32. */
    iReadTemp >>= 5;

    /* Calculate the resistance value. */
    fR = ADC_MAX/iReadTemp-1.0;
    fR = fR0*fR;

    /* TODO: Add mutex here. */
    /* Convert to flTemperature via datasheet. */
    stSensors.flTemperature = 1.0/(log(fR/fR0)/B+1/NTC_K1)-NTC_K2;

    printInfo("\nTemperature: %.1fC", stSensors.flTemperature);

    return ERR_NONE;
#else
    printWarning("\nenReadTemp NOT_IMPLEMENTED");
    return ERR_NONE;
#endif
}

tenError MySensors::enReadAirQuality(void)
{
#ifdef HAS_AIRQ
    /* Get quality from slope, to get air quality. */
    int iCurrentQuality = oAirQualitySensor.slope();
    
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);
    
    /* Set the air quality, according to thresholds. */
    if (iCurrentQuality >= 0)
    {
        if (iCurrentQuality==0)
        {
            printInfo("\nHigh pollution! Danger!");
    /* TODO: Add mutex here. */
            stSensors.iAirQual = 4;
            stSensors.sAirQual = "High pollution! Danger!";
        }
        else if (iCurrentQuality==1)
        {
            printInfo("\nHigh pollution! Danger!");
    /* TODO: Add mutex here. */
            stSensors.iAirQual = 3;
            stSensors.sAirQual = "High pollution! Warning!";
        }
        else if (iCurrentQuality==2)
        {
            printInfo("\nLow pollution!");
    /* TODO: Add mutex here. */
            stSensors.iAirQual = 2;
            stSensors.sAirQual = "Low pollution!";
        }
        else if (iCurrentQuality ==3)
        {
            printInfo("\nFresh air!");
    /* TODO: Add mutex here. */
            stSensors.iAirQual = 1;
            stSensors.sAirQual = "Fresh air!";
        }
    }
    /* ...else save quality as invalid. */
    else
    {
    /* TODO: Add mutex here. */
        stSensors.iAirQual = 0;
        stSensors.sAirQual = "N/A";
    }

    return ERR_NONE;
#else
    printWarning("\nenReadAirQuality NOT_IMPLEMENTED");
    return ERR_NONE;
#endif
}

tenError MySensors::enReadGasSensor(void)
{
#ifdef HAS_GAS
    /* R0 determined empirically by using clean air.
    See link below or more information:
    http://wiki.seeed.cc/Grove-Gas_Sensor-MQ2/ */
    float fR0 = 4.4; 
    float fSensorVolt;

    /* Get value of RS in a GAS. */
    float fRSGas;
    int iReadGas = 0;

    /* Sample value from 32 readings. */
    for(int ii=0; ii<32; ii++)
    {
        iReadGas += analogRead(E_PIN_GAS);
    }

    /* Divide by 32. */
    iReadGas >>= 5;

    /* Calculate the Gas sensor reading, from the measured voltage. */
    fSensorVolt=(float)iReadGas/4095*5.0;
    fRSGas = (5.0-fSensorVolt)/fSensorVolt; 

    /* TODO: Add mutex here. */
    /* Save the measurement locally. */
    stSensors.flGas = fRSGas/fR0;

    printInfo("\nGas: %.1f", stSensors.flGas);

    return ERR_NONE;
#else
    printWarning("\nenReadGasSensor NOT_IMPLEMENTED");
    return ERR_NONE;
}
#endif

tenError MySensors::enReadNoise(void)
{
#ifdef HAS_NOISE
    long lNoise = 0;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Sample value from 32 readings. */
    for(int ii=0; ii<32; ii++)
    {
        lNoise += analogRead(E_PIN_NOISE);
    }

    /* Divide by 32. */
    lNoise >>= 5;

    /* TODO: Add mutex here. */
    /* Calculate the noise percentage (assuming a full range). */
    stSensors.bNoise = (char) (100 * lNoise) / 4095;

    printInfo("\nNoise: %.1f", stSensors.bNoise);

    return ERR_NONE;
#else
    printWarning("\nenReadNoise NOT_IMPLEMENTED");
    return ERR_NONE;
#endif
}

/***************************************************************************
 * IMPLEMENTATION OF PUBLIC FUNCTIONS
 ****************************************************************************/
MySensors::MySensors(void)
{
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Set input pins. */
    #ifdef HAS_PIR
        pinMode(E_PIN_PIR,      INPUT);
    #endif
    #ifdef HAS_GAS
        pinMode(E_PIN_GAS,      INPUT);
    #endif
    #ifdef HAS_SHARP
        pinMode(E_PIN_SHARP,    INPUT);
    #endif
    #ifdef HAS_SONAR
        pinMode(E_PIN_SONAR_A,  INPUT);
    #endif
    #ifdef HAS_SONAR
        pinMode(E_PIN_SONAR_B,  INPUT);
    #endif
    #ifdef HAS_PIR
        /* Attach interrupt to detect when PIR pin is high. */
        attachInterrupt(digitalPinToInterrupt(E_PIN_PIR), __int_readPIR__, HIGH);
    #endif
}

tenError MySensors::enProcessSensors(void)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    static tenStateSensors enState         = E_READ_SONAR;
    static tenStateSensors enInternalState = E_READ_SHARP;

    int i = 0;

    /* Cycle through all states until an invalid state is found. */
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);
    
    while(ERR_NONE == enError)
    {
        /* Sensors state machine. */
        switch(enState)
        {
            case E_READ_SONAR:
                /* Read sonar sensor data. */
                // 1 is off
                if( i != 1)
                    enError = enReadSonar(i);
                i++;
                
                if(i == E_SONAR_COUNT)
                    i = 0;
                enState = E_READ_OTHERS;
                break;
            /* Separate to increase priority of sonar readings. */
            case E_READ_OTHERS:
            /* Disabling the rest of the functionality for a quicker/better reading of the sonars. */
            #if 0
                switch(enInternalState)
                {
                    case E_READ_SHARP:
                        /* Read sharp sensor data. */
                        enError = enReadSharp();
                        enInternalState = E_READ_PIR;
                        break;
                    case E_READ_PIR:
                        /* Read PIR sensor data. */
                        enError = enReadPIR();
                        enInternalState = E_READ_TEMP;
                        break;
                    case E_READ_TEMP:
                        /* Read Temperature. */
                        enError = enReadTemp();
                        enInternalState = E_READ_AIR_Q;
                        break;
                    case E_READ_AIR_Q:
                        /* Read qualitative air quality. */
                        enError = enReadAirQuality();
                        enInternalState = E_READ_GAS;
                        break;
                    case E_READ_GAS:
                        /* Act the motors. */
                        enError = enReadGasSensor();
                        enInternalState = E_READ_NOISE;
                        break;
                    case E_READ_NOISE:
                        /* Read the noise. */
                        enError = enReadNoise();
                        enInternalState = E_READ_SHARP;
                        break;
                    default:
                        enError = ERR_UNK_STATE;
                        break;
                }
            #endif
                enState = E_READ_SONAR;
                break;
            default:
                enError = ERR_UNK_STATE;
                break;
        }

        /* Wait before changing status - also lets other tasks take the processor. */
        delay(DELAY_TASK_SENSORS_MS);
    }

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);
    return ERR_NONE;
}

tenError MySensors::enReadAirQualityUpdate(void)
{
#ifdef HAS_AIRQ
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Collect the air reading as per the timed reading. */
    oAirQualitySensor.last_vol = oAirQualitySensor.first_vol;
    oAirQualitySensor.first_vol = analogRead(E_PIN_AIRQ); // change this value if you use another A port
    oAirQualitySensor.counter = 0;
    oAirQualitySensor.timer_index = 1;
#endif
    return ERR_NONE;
}

float MySensors::flGetSonarA(void)
{
    return stSensors.aflDistSonar[E_SONAR_A];
}

float MySensors::flGetSonarB(void)
{
    return stSensors.aflDistSonar[E_SONAR_B];
}

float MySensors::flGetSonarC(void)
{
    return stSensors.aflDistSonar[E_SONAR_C];
}

String MySensors::sBuildMsg(void)
{
    /* Message to return. */
    String sMsg, sMovement;

    /* Start with sonar message. */
    static char chMsgIndex = E_MSG_SONAR_A;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* TODO: use a queue here of strings here, and get one at a time, this way it's simpler to print if we remove sensors or add them. */
    switch(chMsgIndex)
    {
        case E_MSG_SONAR_A:
            sMsg = "Sonar A: " + String(stSensors.aflDistSonar[E_SONAR_A]) + "cm";
            break;
        case E_MSG_SONAR_B:
            sMsg = "Sonar B: " + String(stSensors.aflDistSonar[E_SONAR_B]) + "cm";
            break;
        case E_MSG_SHARP:
            sMsg = "Sharp: " + String(stSensors.iDistSharp) + "cm";
            break;
        case E_MSG_PIR:
            sMovement = (stSensors.boMovement) ? "False" : "True";
            sMsg = "Movement: " + sMovement;
            break;
        case E_MSG_TEMP:
            sMsg = "Temp: " + String(stSensors.flTemperature) + "C";
            break;
        case E_MSG_AIRQ:
            sMsg = stSensors.sAirQual;
            break;
        case E_MSG_GAS:
            sMsg = "Gas: " + String(stSensors.flGas);
            break;
        case E_MSG_NOISE:
            /* Reset index. */
            sMsg = "Noise: " + String(stSensors.bNoise) + "ppc";
            chMsgIndex = E_MSG_LOOP;
            break;
        default:
            printError("Invalid sMsg index.\n");
            break;
    }
    
    /* Increment index, so that a new message is set in the next loop. */
    chMsgIndex++;

    return sMsg;
}

String MySensors::sBuildUrl(void)
{
    /* TODO: Use a mutex on the reading of these values. Add mutex to the setting of these values. */
    int iMovement = (stSensors.boMovement) ? 0 : 1;
    // int timestamp = ?

    return       "?sharp="      + String((int)stSensors.iDistSharp*100)               + "&" +
                 "sonar="       + String((int)stSensors.aflDistSonar[E_SONAR_A]*100)    + "&" +
                 "sonar="       + String((int)stSensors.aflDistSonar[E_SONAR_B]*100)    + "&" +
                 "gas="         + String((int)stSensors.flGas*100)                     + "&" +
                 "air="         + String(stSensors.iAirQual)                          + "&" +
                 "temperature=" + String((int)stSensors.flTemperature*100)             + "&" +
                 "movement="    + String(iMovement)                                          + "&" +
                 "noise="       + String(stSensors.bNoise);//                 + "&" +
                 // "noise="       + String(timestamp);
}