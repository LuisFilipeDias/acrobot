/***************************************************************************
 *============= © 2018 LUIS FILIPE DIAS  - ALL RIGHTS RESERVED  =============
 ****************************************************************************
 *
 *  @file       motors.h
 *
 *  @author     Luis Filipe Dias
 *
 *  @date       30-08-2017
 *
 *  @version    2.0
 *  
 *  @brief      Header file for motors library.
 *
 *  @section DESCRIPTION
 *  
 *  This module handles the motors control.
 *
 ****************************************************************************/
#ifndef MOTORS_H
#define MOTORS_H

/***************************************************************************
 * HEADER-FILES (Only those that are needed in this file)
 ****************************************************************************/
/* System headerfiles */
#include <stdio.h>

/* Foreign headerfiles */
#include <acrobot.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_attr.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_reg.h>
#include <soc/mcpwm_struct.h>


/* Own headerfiles */
/* None */

/***************************************************************************
 * LOCAL DEFINITIONS 
 ****************************************************************************/

class Motors{

    private:
        mcpwm_unit_t  xUnit;
        mcpwm_timer_t xTimer;

        /***************************************************************************
         * DECLARATION OF FUNCTIONS
         ****************************************************************************/
        /**
        * @brief Motor moves in forward direction, with duty cycle = duty %.
        *
        * @param[float] fDutyCycle      duty cycle to be set;
        *
        * @returns Error code.
        */
        tenError enBrushedMotorFW(float fDutyCycle);

        /**
        * @brief Motor moves in backward direction, with duty cycle = duty %.
        *
        * @param[float] fDutyCycle      duty cycle to be set;
        *
        * @returns Error code.
        */
        tenError enBrushedMotorBW(float fDutyCycle);

        /**
        * @brief Motor stop.
        *
        * @returns Error code.
        */
        tenError enBrushedMotorStop(void);

    public:
        /**
        * @brief Class constructor.
        *
        * @param[tenMotor] enMotor              motor identification;
        * @param[mcpwm_unit_t] xUnit            pwm unit to control;
        * @param[mcpwm_timer_t] xTimer          timer to control the pwm;
        * @param[mcpwm_io_signals_t] xPWMFwd    pwm for forward operation;
        * @param[mcpwm_io_signals_t] xPWMBwd    pwm for backward operation;
        * @param[int] iPinFwd                   pwm pin for forward operation;
        * @param[int] iPinBwd                   pwm pin for backward operation;
        *
        */
        Motors(tenMotor enMotor, mcpwm_unit_t xUnit, mcpwm_timer_t xTimer, mcpwm_io_signals_t xPWMFwd, mcpwm_io_signals_t xPWMBwd, int iPinFwd, int iPinBwd);

        /***************************************************************************
         * DECLARATION OF FUNCTIONS
         ****************************************************************************/
        /**
        * @brief Configure the motor.
        *
        * @param[flSpeed]         power of motor (0 - 100);
        * @param[chDir]         direction of motor;
        *
        * @returns Error code.
        */
        tenError enConfigMotor(float flSpeed, char chDir);
};

#endif /* MOTORS_H */