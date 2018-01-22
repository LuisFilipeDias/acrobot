/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       acrobot.cpp
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       07-01-2018
 *
 *  @version    2.0 
 *  
 *  @brief      Main ACROBOT ino project file.
 *
 *  @section DESCRIPTION
 *  
 *  This is the main project and source file. It triggers the tasks that handle
 *  each of the operations: database communication, sensor reading, motor setting,
 *  among others. It also contains the code for the timers configuration 
 *  and execution.
 *
 ****************************************************************************/

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
/* None */

/* Foreign headerfiles */
#include <motors.h>
#include <bluectrl.h>
#include <mylcd.h>
#include <mywifi.h>
#include <myserial.h>
#include <mysensors.h>
#include <ctrlmode.h>
#include <SimpleBLE.h>

/* Own headerfiles */
#include <acrobot.h>

/***************************************************************************
 * PRIVATE VARIABLES 
 ****************************************************************************/
/**
* @brief Library objects for each component.
*/
MyLCD         oLCD;
MyWifi        oWifi;
MySerial      oSerial;
MySensors     oSensors;
CtrlMode      oCtrlMode;
BlueCtrl      oBlueCtrl;

/**
* @brief For comms between tasks.
*/
QueueHandle_t xQueue;

/**
* @brief HW Timer object.
*/
hw_timer_t *pxTimer = NULL;

/**
* @brief Semaphore for syncing interrupt with main thread.
*/
volatile SemaphoreHandle_t xTimerSemaphore;

/**
* @brief Semaphore for exchanging sensitive information.
*/
portMUX_TYPE xTimerMultx = portMUX_INITIALIZER_UNLOCKED;

/***************************************************************************
 * C IMPLEMENTATION OF TIMER-RELATED FUNCTIONS 
 * (APPEARS TO BE A LIMITATION - TIMER RELATED FUNCTIONS MUST BE IN THE .INO)
 ****************************************************************************/
void IRAM_ATTR onTimer()
{
    /* Do sensitive operations to shared variables here. */
    portENTER_CRITICAL_ISR(&xTimerMultx);
    portEXIT_CRITICAL_ISR(&xTimerMultx);

    /* Give a semaphore that we can check in the loop. */
    xSemaphoreGiveFromISR(xTimerSemaphore, NULL);
}

tenError enTimerSetup(void)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Create semaphore to inform us when the timer has fired. */
    xTimerSemaphore = xSemaphoreCreateBinary();

    /* If failed to create semaphore... */
    if (NULL == xTimerSemaphore)
    {
        enError = ERR_SYSCALL;
        printError("\nERROR: Creating semaphore.");
    }
    /* ...else semaphore created successfully. */
    else
    {
        /* Use 1st timer of 4 (counted from zero).
        Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more
        info). */
        pxTimer = timerBegin(0, 80, true);

        /* If failed to begin timer... */
        if (NULL == pxTimer)
        {
            enError = ERR_SYSCALL;
            printError("\nERROR: Starting Timer.");
        }
        /* ...else timer began with success. */
        else
        {
            /* Attach onTimer function to our timer. */
            timerAttachInterrupt(pxTimer, &onTimer, true);

            /* Set alarm to call onTimer function every 2 seconds (value in microseconds). */
            /* Repeat the alarm (third parameter). */
            timerAlarmWrite(pxTimer, 1000000, true);

            /* Start an alarm */
            timerAlarmEnable(pxTimer);
        }
    }

    return enError;
}

tenError enTimerExec(void)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);
    
    /* If failed to take semaphore... */
    if (pdTRUE == xSemaphoreTake(xTimerSemaphore, SEMAPHORE_TIMEOUT_TICKS))
    {
        /* Do sensitive operations to shared variables here. */
        portENTER_CRITICAL(&xTimerMultx);
        portEXIT_CRITICAL(&xTimerMultx);

        /* Do something inside the timer... */
    }
    else
    {
        enError = ERR_SYSCALL;
        printError("\nERROR: Semaphore timed'out.");
    }

    return enError;
}

/***************************************************************************
 * C IMPLEMENTATION OF ARDUINO FUNCTIONS
 ****************************************************************************/
void setup() 
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    /* Wait before setting up the remaining configurations. */
    delay(DELAY_SETUP_MS/2);

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Setup Wifi -> cannot be initiated via constructor, else it crashes. */
    enError = oWifi.enWifiSetup();

    if (ERR_NONE == enError)
    {
        /* No Queue is needed for now. */
        #if 0 
        /* Initialize the message queue for comms. */
        xQueue = xQueueCreate(5, sizeof(int));

        if(xQueue == NULL)
        {
            enError = ERR_SYSCALL;
            printError("\nERROR creating the Queue.");
        }
        #endif
        if (pdPASS != xTaskCreate(vTaskSensors,               /* Task function. */
                                       "vTaskSensors",             /* String with name of task. */
                                       10000,                      /* Stack size in words. */
                                       NULL,                       /* Parameter passed as input of the task */
                                       4,                          /* Priority of the task. */
                                       NULL))                      /* Task handle. */
        {
            enError = ERR_SYSCALL;
            printError("\nERROR: Creating vTaskSensors.");
        }
        else if (pdPASS != xTaskCreate(vTaskMotors,                  /* Task function. */
                                       "vTaskMotors",                /* String with name of task. */
                                       10000,                      /* Stack size in words. */
                                       NULL,                       /* Parameter passed as input of the task */
                                       7,                          /* Priority of the task. */
                                       NULL))                       /* Task handle. */
        {
            enError = ERR_SYSCALL;
            printError("\nERROR: Creating vTaskMotors.");
        }
        /* At the moment there is no data to upload. Leave it here though for the future. */
        #if 0
        else if (pdPASS != xTaskCreate(vTaskUploadData,            /* Task function. */
                                       "vTaskUploadData",          /* String with name of task. */
                                       10000,                      /* Stack size in words. */
                                       NULL,                       /* Parameter passed as input of the task */
                                       1,                          /* Priority of the task. */
                                       NULL))                      /* Task handle. */
        {
            enError = ERR_SYSCALL;
            printError("\nERROR: Creating vTaskUploadData.");
        }
        #endif
        else
        {
            /* This block was causing trouble, not yet investigated. */
            #if 0
                Serial.begin(115200);
                Serial.println("Connected!");
            #endif
        }
    }

    /* Something went wrong. Terminate now. */
    if (ERR_NONE != enError)
    {
        /* Task finished, get rid of it. */
        vTaskDelete(NULL);
    }
}

void vTaskSensors(void *parameter)
{
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    if (ERR_NONE != oSensors.enProcessSensors())
    {
        /* Task finished, get rid of it. */
        vTaskDelete(NULL);
    }
}

void vTaskMotors(void *parameter)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    /* Adjusted speed, with lever info. */
    float flSpeedAdj[E_MOTOR_COUNT];

    /* Initial state is init. */
    static tenState enState = E_INIT;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    while(ERR_NONE == enError)
    {
        /* Main state machine. */
        switch(enState)
        {
            case E_INIT:
                enState = E_GET_CTRL_CMD;
                break;
            case E_GET_CTRL_CMD:
            #if BLUETOOTH_SUPPORT == YES
                /* Get bluetooth control commands. */
                enError = oBlueCtrl.enGetCtrlCmd();
            #endif
                enState = E_SET_MOTORS;
                break;
            case E_SET_MOTORS:
                
                // /* Set the motors control, note that mode is set internally, so pass all the info at this level. */
                // enError = oCtrlMode.enSetCtrl(boIsAuto, flSpeedAdj, oSensors.flGetSonarA(), oSensors.flGetSonarC());

                enState = E_REFRESH_DISPLAY;
                break;
            case E_REFRESH_DISPLAY:
            #if LCD_SUPPORT == YES
                /* Build a message and display in the LCD. */
                enError = oLCD.enRefresh(oSensors.sBuildMsg());
            #endif
                enState = E_GET_CTRL_CMD;
                break;
            default:
                enError = ERR_UNK_STATE;
                break;
        }
        delay(DELAY_TASK_LOOP_MS);
    }

    /* Task finished, get rid of it. */
    vTaskDelete(NULL);
}

#if 0
void vTaskUploadData(void *parameter)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    while(ERR_NONE == enError)
    {

        delay(DELAY_TASK_UPLOAD_MS);

        /* This is unecessary for current configuration. */
        #if 0
        /* Upload data compiled from sensors to database. */
        enError = oWifi.enUploadData(oSensors.sBuildUrl());
        #endif

        /* Do not overwritte previous error, if it exists. */
        if (ERR_NONE == enError)
        {
            /* Let's do a non-time critical task here, in the other tasks this has a negative impact. */
            enError = enTimerExec();
        }
    }

    /* Task finished, get rid of it. */
    vTaskDelete(NULL);
}
#endif

void loop(void)
{
    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Task finished, get rid of it, other tasks take over. */
    vTaskDelete(NULL);
}