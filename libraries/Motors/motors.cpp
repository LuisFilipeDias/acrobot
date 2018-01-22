/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       motors.cpp
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Implementation of motors library.
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
#include <motors.h>

/***************************************************************************
 * C IMPLEMENTATION OF PRIVATE FUNCTIONS
 ****************************************************************************/
tenError Motors::enBrushedMotorFW(float flDuty)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    if (ESP_OK != mcpwm_set_signal_low(this->xUnit, this->xTimer, MCPWM_OPR_B))
    {
        enError = ERR_MOTORCALL;
        printError("\nERROR: Setting MCPWM Signal Low.");
    }
    else if (ESP_OK != mcpwm_set_duty(this->xUnit, this->xTimer, MCPWM_OPR_A, flDuty))
    {
        enError = ERR_MOTORCALL;
        printError("\nERROR: Setting MCPWM Duty.");
    }
    /* Call this each time, if operator was previously in low/high state. */
    else if (ESP_OK != mcpwm_set_duty_type(this->xUnit, this->xTimer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0))
    {
        enError = ERR_MOTORCALL;
        printError("\nERROR: Setting MCPWM Signal Type.");
    }

    return enError;
}

tenError Motors::enBrushedMotorBW(float flDuty)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    if (ESP_OK != mcpwm_set_signal_low(this->xUnit, this->xTimer, MCPWM_OPR_A))
    {
        enError = ERR_MOTORCALL;
        printError("\nERROR: Setting MCPWM Signal Low.");
    }
    else if (ESP_OK != mcpwm_set_duty(this->xUnit, this->xTimer, MCPWM_OPR_B, flDuty))
    {
        enError = ERR_MOTORCALL;
        printError("\nERROR: Setting MCPWM Duty.");
    }
    /* Call this each time, if operator was previously in low/high state. */
    else if (ESP_OK != mcpwm_set_duty_type(this->xUnit, this->xTimer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0))
    {
        enError = ERR_MOTORCALL;
        printError("\nERROR: Setting MCPWM Signal Type.");
    }

    return enError;
}

tenError Motors::enBrushedMotorStop(void)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    if (ESP_OK != mcpwm_set_signal_low(this->xUnit, this->xTimer, MCPWM_OPR_B))
    {
        enError = ERR_MOTORCALL;
        printError("\nERROR: Setting MCPWM Signal Low.");
    }
    else if (ESP_OK != mcpwm_set_signal_low(this->xUnit, this->xTimer, MCPWM_OPR_A))
    {
        enError = ERR_MOTORCALL;
        printError("\nERROR: Setting MCPWM Signal Low.");
    }

    return enError;
}

/***************************************************************************
 * C IMPLEMENTATION OF PUBLIC FUNCTIONS
 ****************************************************************************/
Motors::Motors(tenMotor enMotor, mcpwm_unit_t xUnit, mcpwm_timer_t xTimer, mcpwm_io_signals_t xPWMFwd, mcpwm_io_signals_t xPWMBwd, int iPinFwd, int iPinBwd)
{
    /* Init function with no error. */
    tenError enError = ERR_NONE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    /* Save unit and timer to the object. */
    this->xUnit  = xUnit;
    this->xTimer = xTimer;

    /* Initializie mcpwm gpio. */
    if ((ESP_OK != mcpwm_gpio_init(this->xUnit, xPWMFwd, iPinFwd)) || (ESP_OK != mcpwm_gpio_init(this->xUnit, xPWMBwd, iPinBwd)))
    {
        enError = ERR_MOTORCALL;
        printError("\nERROR: Init MCPWM GPIOs for motor A.");
    }
    else
    {
        /* Configure Initial Parameters of mcpwm. */
        mcpwm_config_t xPWMConfig;

        /* Frequency = 500Hz. */
        xPWMConfig.frequency = MOTORS_PWM_FREQUENCY;

        /* Duty cycle of PWMxA = 0. */
        xPWMConfig.cmpr_a = 0;

        /* Duty cycle of PWMxB = 0. */
        xPWMConfig.cmpr_b = 0;

        /* Counter mode ???? */
        xPWMConfig.counter_mode = MCPWM_UP_COUNTER;

        /* Duty mode ???? */
        xPWMConfig.duty_mode = MCPWM_DUTY_MODE_0;

        /* Configure PWM0s with above settings. */
        if (ESP_OK != mcpwm_init(this->xUnit, this->xTimer, &xPWMConfig))
        {
            enError = ERR_MOTORCALL;
            printError("\nERROR: Init MCPWM Unit 0.");
        }
    }

    /* Something went wrong. Terminate now. */
    if (ERR_NONE != enError)
    {
        /* Task finished, get rid of it. */
        vTaskDelete(NULL);
    }
}

tenError Motors::enConfigMotor(float flSpeed, char chDir)
{
    /* Init function with no error. */
    tenError enError = ERR_UNK_STATE;

    printDebug("\n%s [%d]",__FUNCTION__, __LINE__);

    // printf("\nConfig Motor. Direction: %s. Speed %2.2f", (chDir == 0)?"FWD":"BWD", flSpeed);

    /* If power is zero, make it stop. */
    if (flSpeed == 0)
    {
        chDir = MOTOR_STOP;
    }

    /* Get direction and call corresponding function. */
    switch(chDir)
    {
        case MOTOR_FWD:
            enError = enBrushedMotorFW(flSpeed);
        break;
        case MOTOR_BWD:
            enError = enBrushedMotorBW(flSpeed);
        break;
        case MOTOR_STOP:
            enError = enBrushedMotorStop();
        break;
        default:
            printError("\nERROR: Unknown direction.");
        break;
    }

    return enError;
}