/**************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       acrobot.h
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Header file for ACROBOT.
 *
 *  @section DESCRIPTION
 *  
 *  Contains a set of configurations, from VERBOSE options, to WiFi settings
 *  tasks delays, pwm settings, motor configurations, and a set of enum types.
 *  It is basically a the project configuration file.
 *
 ****************************************************************************/
#ifndef ACROBOT_H
#define ACROBOT_H

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */

/* Foreign headerfiles */
/* None */

/* Own headerfiles */
/* None */

/***************************************************************************
 * PUBLIC CONFIGURATIONS
 ****************************************************************************/
/**
* @brief Verbose levels.
*/
#define VERBOSE_DEBUG                       4
#define VERBOSE_INFO                        3
#define VERBOSE_WARNING                     2
#define VERBOSE_ERROR                       1
#define VERBOSE_NONE                        0

/**
* @brief Actual Verbose level.
*/
#define VERBOSE_LEVEL                       VERBOSE_INFO

/**
* @brief Verbose traces activation. (To avoid compilation errors, disable Compiler Warnings on the Arduino definitions)
*/
#define printError
#define printWarning
#define printInfo
#define printDebug
#if (VERBOSE_LEVEL >= VERBOSE_ERROR)
    #undef  error
    #define printError                      printf
#if (VERBOSE_LEVEL >= VERBOSE_WARNING)
    #undef  warning
    #define printWarning                    printf
#if (VERBOSE_LEVEL >= VERBOSE_INFO)
    #undef  info
    #define printInfo                       printf
#if (VERBOSE_LEVEL >= VERBOSE_DEBUG)
    #undef  debug
    #define printDebug                      printf
#endif
#endif
#endif
#endif

/**
* @brief WiFi Data.
*/
    #define WIFI_USER                           "AndroidAP3445"
    #define WIFI_PASSWORD                       "65c32f94cba4"
#if 0
    #define WIFI_USER                           "CSW-Guest"
    #define WIFI_PASSWORD                       "Critical98"
#define WIFI_USER                           "ThomsonD223E7"
#define WIFI_PASSWORD                       "60ED2539CA"
#define WIFI_USER                           "CASA LT"
#define WIFI_PASSWORD                       "felgueiras"
#define WIFI_USER                           "Vodafone-DE0235"
#define WIFI_PASSWORD                       "FD91B1958E"
#define WIFI_USER                           "MyAquaris"
#define WIFI_PASSWORD                       "luisdias"
#define WIFI_USER                           "Sidonio"
#define WIFI_PASSWORD                       "margaride12"
#endif

/**
* @brief YES/NO macros.
*/
#define YES                                 1
#define NO                                  0

/**
* @brief Define WiFi support.
*/
#define WIFI_SUPPORT                        NO

/**
* @brief Define LCD support.
*/
#define LCD_SUPPORT                         NO

/**
* @brief Define Bluetooth support.
*/
#define BLUETOOTH_SUPPORT                   NO

#if WIFI_SUPPORT == NO
    /**
    * @brief Uploading activation.
    */
    #define UPLOAD_DATA                     YES
#endif

/**
* @brief Define platform being used.
*/
/* #define PLATFORM                         WEMOS */
#define PLATFORM                            ESP32

/**
* @brief Initial setup delay.
*/
#define DELAY_SETUP_MS                      100

/**
* @brief Loop task delay between iterations.
*/
#define DELAY_TASK_LOOP_MS                  10

/**
* @brief Sensors task delay between iterations.
*/
#define DELAY_TASK_SENSORS_MS               50

/**
* @brief Upload task delay between iterations.
*/
#define DELAY_TASK_UPLOAD_MS                10000

/**
* @brief Upload task delay between iterations.
*/
#define DELAY_WIFI_CONNECTION_RETRY_MS      1000

/**
* @brief Number of attempts to connect WiFi.
*/
#define WIFI_CONNECTION_RETRIES             5

/**
* @brief Ticks to semaphore timeout.
*/
#define SEMAPHORE_TIMEOUT_TICKS             65535

/**
* @brief Frequency for the motors PWM - actual value is divided by 2 (e.g. 1000 is 500 Hz).
*/
#define MOTORS_PWM_FREQUENCY                100000

/**
* @brief Motor direction macro.
*/
#define MOTOR_BWD                           0
#define MOTOR_FWD                           1
#define MOTOR_STOP                          2

/**
* @brief Set time and unit and pwm for motor A.
*/
#define MOTOR_UNIT_A                        MCPWM_UNIT_0
#define MOTOR_TIMER_A                       MCPWM_TIMER_0
#define MOTOR_PWMFWD_A                      MCPWM0A
#define MOTOR_PWMBWD_A                      MCPWM0B

/**
* @brief Set time and unit and pwm for motor B.
*/
#define MOTOR_UNIT_B                        MCPWM_UNIT_1
#define MOTOR_TIMER_B                       MCPWM_TIMER_1
#define MOTOR_PWMFWD_B                      MCPWM1A
#define MOTOR_PWMBWD_B                      MCPWM1B

/**
* @brief Set time and unit and pwm for motor C.
*/
#define MOTOR_UNIT_C                        MCPWM_UNIT_1
#define MOTOR_TIMER_C                       MCPWM_TIMER_2
#define MOTOR_PWMFWD_C                      MCPWM2A
#define MOTOR_PWMBWD_C                      MCPWM2B

/**
* @brief Value of lever in the middle (stop point) from Blynk.
*/
#define BLINK_LEVER_MID                     512

/**
* @brief Value of lever for left right adjustment.
*/
#define HORIZONTAL_MID_LEVER                50

/**
* @brief Value of lever maximum for left right adjustment.
*/
#define HORIZONTAL_MAX_LEVER                100

/**
* @brief Step for manual control of horizontal control.
*/
#define MOTOR_HORIZONTAL_STEP                1

/**
* @brief Step for manual control of vertical control.
*/
#define MOTOR_VERTICAL_STEP                  30

/**
* @brief Maximum value of motors.
*/
#define MOTOR_MAX_VALUE                      100

/**
* @brief To adjust difference of motors - hardcoded here.
*/
#define MOTOR_ADJ_PC                          9

/**
* @brief 10 seconds in ms.
*/
#define TIME_10_SECONDS_MS                    8000

/**
* @brief 2 seconds in ms.
*/
#define TIME_2_SECONDS_MS                     4000

/**
* @brief Auto mode FWD power.
*/
#define MOTOR_AUTO_FWD                        20

#define MOTOR_AUTO_FWD2                       25


/**
* @brief Auto mode WALL left power.
*/
#define MOTOR_AUTO_WALL_L                     45

/**
* @brief Auto mode WALL right power.
*/
#define MOTOR_AUTO_WALL_R                     0

/**
* @brief Auto mode GROUND power.
*/
#define MOTOR_AUTO_GROUND                     55

/**
* @brief Threshold for bottom sensor on auto mode.
*/
#define SENSE_BOTTOM                          40

/**
* @brief Threshold for front sensor on auto mode.
*/
#define SENSE_FRONT                           55

/***************************************************************************
 * PUBLIC VARIABLES
 ****************************************************************************/
/**
* @brief List of errors.
*/
typedef enum
{
    ERR_NONE                                = 0,
    ERR_SYSCALL,
    ERR_MOTORCALL,
    ERR_UNK_STATE,
    ERR_WIFI_SETUP,
    ERR_UNK_PLATFORM,
    ERR_INVALID_LCD_MSG,
    ERR_WIFI_CONNECTION,
    ERR_WIFI_CONNECTION_TIMEOUT
} tenError;

/**
* @brief List of motors.
*/
typedef enum
{
    E_SONAR_A                               = 0,
    E_SONAR_B                               = 1,
    E_SONAR_C                               = 2,

    E_SONAR_COUNT                           = 3
} tenSonar;

/**
* @brief List of motors.
*/
typedef enum
{
    E_MOTOR_LEFT                            = 0,
    E_MOTOR_RIGHT                           = 1,
    E_MOTOR_BOTTOM                          = 2,

    E_MOTOR_COUNT                           = 3
} tenMotor;

/**
* @brief List of states from general state machine.
*/
typedef enum
{
    E_INIT                                  = 0,
    E_SET_MOTORS,
    E_GET_CTRL_CMD,
    E_REFRESH_DISPLAY,
} tenState;

/**
* @brief List of states from sensors state machine.
*/
typedef enum
{
    E_READ_PIR                            = 0,
    E_READ_GAS,
    E_READ_TEMP,
    E_READ_SONAR,
    E_READ_SHARP,
    E_READ_AIR_Q,
    E_READ_NOISE,
    E_READ_OTHERS,
} tenStateSensors;

/**
* @brief List of mapped pins.
*/
typedef enum
{
    E_PIN_GAS                           = 34,
    E_PIN_PIR                           = 25,
    E_PIN_TEMP                          = 35,
    E_PIN_AIRQ                          = 33,
    E_PIN_NOISE                         = 36,
    E_PIN_SHARP                         = 32,
    E_PIN_SONAR_A                       = 26,
    E_PIN_SONAR_B                       = 27,
    E_PIN_SONAR_C                       = 14,
    E_PIN_MOTORFWD_A                    = 4,
    E_PIN_MOTORBWD_A                    = 5,
    E_PIN_MOTORFWD_B                    = 2,
    E_PIN_MOTORBWD_B                    = 15,
    E_PIN_MOTORFWD_C                    = 18,
    E_PIN_MOTORBWD_C                    = 19,
    E_PIN_HEADER_IN                     = 16,
    E_PIN_HEADER_OUT                    = 17
} tenPins;

/**
* @brief List of LCD messages.
*/
typedef enum
{
    E_MSG_GAS                           = 0,
    E_MSG_PIR,
    E_MSG_TEMP,
    E_MSG_AIRQ,
    E_MSG_SHARP,
    E_MSG_NOISE,
    E_MSG_SONAR_A,
    E_MSG_SONAR_B,
    E_MSG_LOOP                          =  -1
} tenLCDMsg;

#endif /* ACROBOT_H */