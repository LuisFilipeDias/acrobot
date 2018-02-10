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
 *  ToDo:
 *          - Calculate speedpitch and place in flSpeedPitch var;
 *
 ****************************************************************************/

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
/* None */

/* Foreign headerfiles */

/* Own headerfiles */
#include "mysensors.h"

#ifdef HAS_SONAR
    Maxbotix oRangeSensorPWA(E_PIN_SONAR_A, Maxbotix::PW, Maxbotix::LV, Maxbotix::SIMPLE);
    Maxbotix oRangeSensorPWB(E_PIN_SONAR_C, Maxbotix::PW, Maxbotix::LV, Maxbotix::SIMPLE);
    Maxbotix oRangeSensors[E_SONAR_COUNT-1] = {oRangeSensorPWA, oRangeSensorPWB};
#endif

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

/**************************************************************************************/
/****************************** MPU-related variables. ********************************/
/**************************************************************************************/
#ifdef HAS_MPU
    /**
    * @brief MPU object.
    */
    MPU6050 oMPU(0x69);

    /**
    * @brief Return status after each device operation (0 = success, !0 = error).
    */
    char chDevStatus;

    /**
    * @brief Holds actual interrupt status byte from MPU.
    */
    char chMPUIntStatus;

    /**
    * @brief Set true if DMP init was successful.
    */
    bool boDmpReady = false;

    /**
    * @brief Expected DMP packet size (default is 42 bytes).
    */
    int iPacketSize;

    /**
    * @brief Count of all bytes currently in FIFO.
    */
    int iFifoCount;

    /**
    * @brief FIFO storage buffer.
    */
    uint8_t chFifoBuffer[64];

    /**
    * @brief xGravity vector [x, y, z].
    */
    VectorFloat xGravity;

    /**
    * @brief Yaw/pitch/roll container [yaw, pitch, roll].
    */
    float flYPR[3];

    /**
    * @brief Quaternion container [w, x, y, z].
    */
    Quaternion xQuaternion;

    /**
    * @brief Indicates whether MPU interrupt pin has gone high.
    */
    volatile bool oMPUInterrupt = false; 
#endif

/***************************************************************************
 * DECLARATION OF ISRs (MUST BE LOCAL) 
 ****************************************************************************/
#ifdef HAS_PIR
    /**
    * @brief Internal ISR for PIR pin transitions.
    */
    void __int_readPIR__(void);
#endif

#ifdef HAS_MPU
    /**
    * @brief Internal ISR to check for DMP data availability.
    */
    void __int_dmpDataReadyPIR__(void);
#endif

/***************************************************************************
 * IMPLEMENTATION OF PRIVATE FUNCTIONS
 ****************************************************************************/
#ifdef HAS_PIR
    void __int_readPIR__(void)
    {
        /* If interrupt was triggered, means movement was detected. */
        boMovement = true;
    }
#endif

#ifdef HAS_MPU
    void __int_dmpDataReadyPIR__()
    {
        /* DMP data is ready. */
        oMPUInterrupt = true;
    }
#endif

/***************************************************************************
 * IMPLEMENTATION OF PRIVATE FUNCTIONS
 ****************************************************************************/





int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    oMPU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    oMPU.setXAccelOffset(ax_offset);
    oMPU.setYAccelOffset(ay_offset);
    oMPU.setZAccelOffset(az_offset);

    oMPU.setXGyroOffset(gx_offset);
    oMPU.setYGyroOffset(gy_offset);
    oMPU.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}









tenError MySensors::enSetupMPU(void)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

#ifdef HAS_MPU
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        // Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
        Wire.setClock(100000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    /* Initialize device. */
    printf("\nInitializing I2C devices...");
    oMPU.initialize();

    /* Verify connection. */
    printf("\nTesting device connections...");
    // oMPU.testConnection() ? printDebug("\nMPU6050 connection successful") : printDebug("\nMPU6050 connection failed");
    if(oMPU.testConnection() == false)
    {
        printf("\n\nMPU6050 connection failed");
        return ERR_NONE;
    }    

    printf("\nInitializing DMP...");
    chDevStatus = oMPU.dmpInitialize();


    /* Calibration block. */
    while(1)
    {

        oMPU.setXGyroOffset(0);
        oMPU.setYGyroOffset(0);
        oMPU.setZGyroOffset(0);
        oMPU.setXAccelOffset(0);
        oMPU.setYAccelOffset(0);
        oMPU.setZAccelOffset(0);


        /* Make sure it worked (returns 0 if so). */
         if (ERR_NONE == chDevStatus)
         {
            /* Turn on the DMP, now that it's ready. */
            oMPU.setDMPEnabled(true);

            chMPUIntStatus = oMPU.getIntStatus();

            /* Set our DMP Ready flag so the main loop() function knows it's okay to use it. */
            boDmpReady = true;

            /* Get expected DMP packet size for later comparison. */
            iPacketSize = oMPU.dmpGetFIFOPacketSize();
         }
         else
         {
             /* ERROR!
             1 = initial memory load failed
             2 = DMP configuration updates failed
             (if it's going to break, usually the code will be 1). */
             printError("\nDMP Initialization failed.");
         }

        while(1)
        {
            if (state==0){
                Serial.println("\nReading sensors for first time...");
                meansensors();
                state++;
                delay(1000);
              }

              if (state==1) {
                Serial.println("\nCalculating offsets...");
                calibration();
                state++;
                delay(1000);
              }

              if (state==2) {
                meansensors();
                Serial.println("\nFINISHED!");
                Serial.print("\nSensor readings with offsets:\t");
                Serial.print(mean_ax); 
                Serial.print("\t");
                Serial.print(mean_ay); 
                Serial.print("\t");
                Serial.print(mean_az); 
                Serial.print("\t");
                Serial.print(mean_gx); 
                Serial.print("\t");
                Serial.print(mean_gy); 
                Serial.print("\t");
                Serial.println(mean_gz);
                Serial.print("Your offsets:\t");
                Serial.print(ax_offset); 
                Serial.print("\t");
                Serial.print(ay_offset); 
                Serial.print("\t");
                Serial.print(az_offset); 
                Serial.print("\t");
                Serial.print(gx_offset); 
                Serial.print("\t");
                Serial.print(gy_offset); 
                Serial.print("\t");
                Serial.println(gz_offset); 
                Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
                Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
                Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
                while (1);
              }
        }

    }

    /* Supply your own gyro offsets here, scaled for min sensitivity. */
    oMPU.setXGyroOffset(-6);
    oMPU.setYGyroOffset(39);
    oMPU.setZGyroOffset(-45);
    oMPU.setXAccelOffset(481); // 1688 factory default for my test chip. */
    oMPU.setYAccelOffset(-739); // 1688 factory default for my test chip. */
    oMPU.setZAccelOffset(742); // 1688 factory default for my test chip. */

     /* Make sure it worked (returns 0 if so). */
     if (ERR_NONE == chDevStatus)
     {
        /* Turn on the DMP, now that it's ready. */
        oMPU.setDMPEnabled(true);

        chMPUIntStatus = oMPU.getIntStatus();

        /* Set our DMP Ready flag so the main loop() function knows it's okay to use it. */
        boDmpReady = true;

        /* Get expected DMP packet size for later comparison. */
        iPacketSize = oMPU.dmpGetFIFOPacketSize();
     }
     else
     {
         /* ERROR!
         1 = initial memory load failed
         2 = DMP configuration updates failed
         (if it's going to break, usually the code will be 1). */
         printError("\nDMP Initialization failed.");
     }
#endif
     return enError;
}

tenError MySensors::enReadMPU(void)
{
#ifdef HAS_MPU
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* If programming failed, don't try to do anything. */
    if (!boDmpReady)
    {
        printError("\nDMP programming failed!");
        /* Let's not kill the program here, and let it try again. */
        return ERR_NONE;
    }
    /* Wait for MPU interrupt or extra packet(s) available. */
    else
    {
        if (!oMPUInterrupt && iFifoCount < iPacketSize)
        {  
            /* Go back, ther is no information from previous interrupt,
            nor is there information still to be processed. */
            printDebug("\nNot yet an interrupt generated.");
            return ERR_NONE;
        }
        else
        {
            /* Reset interrupt flag and get INT_STATUS byte. */
            oMPUInterrupt = false;

            chMPUIntStatus = oMPU.getIntStatus();

            /* Get current FIFO count. */
            iFifoCount = oMPU.getFIFOCount();

            /* Check for overflow (this should never happen unless our code is too inefficient). */
            if ((chMPUIntStatus & 0x10) || iFifoCount == 1024)
            {
                /* Reset so we can continue cleanly. */
                oMPU.resetFIFO();
                printDebug("\nFIFO overflow!");
            }
            /* Otherwise, check for DMP data ready interrupt (this should happen frequently) */
            else if (chMPUIntStatus & 0x02)
            {
                /* Wait for correct available data length, should be a VERY short wait. */
                while (iFifoCount < iPacketSize)
                {
                    iFifoCount = oMPU.getFIFOCount();
                }

                /* Read a packet from FIFO. */
                oMPU.getFIFOBytes(chFifoBuffer, iPacketSize);
                
                /* Track FIFO count here in case there is > 1 packet available
                (this lets us immediately read more without waiting for an interrupt). */
                iFifoCount -= iPacketSize;

                /* Display Euler angles in degrees. */
                oMPU.dmpGetQuaternion(&xQuaternion, chFifoBuffer);
                oMPU.dmpGetGravity(&xGravity, &xQuaternion);
                oMPU.dmpGetYawPitchRoll(flYPR, &xQuaternion, &xGravity);

                stSensors.flAnglePitch = flYPR[1] * 180/M_PI;
                // printf("\nflYPR[1] - Pitch: %2.2f\t", stSensors.flAnglePitch);
            }
        }
    }
#else
    // printWarning("\nenReadMPU NOT_IMPLEMENTED");
#endif
    return ERR_NONE;
}

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
#else
    printWarning("\nenReadSharp NOT_IMPLEMENTED");
#endif
    return ERR_NONE;
}

tenError MySensors::enReadSonar(char chSonarID)
{
#ifdef HAS_SONAR
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    stSensors.aflDistSonar[chSonarID] = oRangeSensors[chSonarID].getRange();

    /* TODO: Add mutex here. */
    /* Limit maximum distance to 150 cm. */
    stSensors.aflDistSonar[chSonarID] = (stSensors.aflDistSonar[chSonarID] > 150) ? 150 : stSensors.aflDistSonar[chSonarID];

    printInfo("\nDistance Sonar %d: %2.2fcm", chSonarID, stSensors.aflDistSonar[chSonarID]);
#else
    printWarning("\nenReadSonar NOT_IMPLEMENTED");
#endif
    return ERR_NONE;
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
#else
    printWarning("\nenReadPIR NOT_IMPLEMENTED");
#endif
    return ERR_NONE;
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
#else
    printWarning("\nenReadTemp NOT_IMPLEMENTED");
#endif
    return ERR_NONE;
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
#else
    printWarning("\nenReadAirQuality NOT_IMPLEMENTED");
#endif
    return ERR_NONE;
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
#else
    printWarning("\nenReadGasSensor NOT_IMPLEMENTED");
#endif
    return ERR_NONE;
}

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
#else
    printWarning("\nenReadNoise NOT_IMPLEMENTED");
#endif
    return ERR_NONE;
}

/***************************************************************************
 * IMPLEMENTATION OF PUBLIC FUNCTIONS
 ****************************************************************************/

MySensors::MySensors(void)
{
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Set input pins. */
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
        pinMode(E_PIN_PIR, INPUT);
        /* Attach interrupt to detect when PIR pin is high. */
        attachInterrupt(digitalPinToInterrupt(E_PIN_PIR), __int_readPIR__, HIGH);
    #endif
    #ifdef HAS_MPU
        pinMode(E_PIN_MPU_INTERRUPT, INPUT);
        /* Attach interrupt to detect when PIR pin is high. */
        attachInterrupt(digitalPinToInterrupt(E_PIN_MPU_INTERRUPT), __int_dmpDataReadyPIR__, RISING);
    #endif

}

tenError MySensors::enProcessSensors(void)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    /* To handle multiple sonars. */
    int iSonarID = 0;

    static tenStateSensors enState         = E_READ_MPU;
    static tenStateSensors enInternalState = E_READ_SHARP;

    /* Cycle through all states until an invalid state is found. */
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Consider moving this to constructor, after getting this to work. */
    #ifdef HAS_MPU
        if( ERR_NONE != enSetupMPU())
        {
            printError("\nError setting up the MTU.");
        }
    #endif

    while(ERR_NONE == enError)
    {
        /* Sensors state machine. */
        switch(enState)
        {
            case E_READ_MPU:
                /* Read sonar sensor data. */
                enError = enReadMPU();

                enState = E_READ_OTHERS;
                break;
            case E_READ_SONAR:
                /* Read sonar sensor data. */
                iSonarID++;

                if(iSonarID == E_SONAR_COUNT)
                {
                    iSonarID = 0;
                }
                enError = enReadSonar(iSonarID);

                enState = E_READ_MPU;
                break;
            /* Separate to increase priority of sonar readings. */
            case E_READ_OTHERS:
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
                /* Let's not read the sonar for now... 
                enState = E_READ_SONAR;*/

                enState = E_READ_MPU;
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
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

#ifdef HAS_AIRQ
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

float MySensors::flGetAnglePitch(void)
{
    return stSensors.flAnglePitch;
}

float MySensors::flGetSpeedPitch(void)
{
    return stSensors.flSpeedPitch;
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