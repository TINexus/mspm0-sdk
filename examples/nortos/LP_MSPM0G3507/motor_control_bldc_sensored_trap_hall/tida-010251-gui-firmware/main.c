/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************
// the includes
//*****************************************************************************
#include "tida-010251.h"

//*****************************************************************************
//local variables
//*****************************************************************************
drv8328_Instance    drv8328;
Halltrap_Instance   hallTrap;
TIDAInstance        tida010251;

int main(void)
{
    /* initialize the modules   */
    SYSCFG_DL_init();
    HAL_init();

    drv8328.drvoff      = HAL_GPIO_OUT_01;
    drv8328.nsleep      = HAL_GPIO_OUT_02;
    drv8328.Isen        = HAL_ADC_CHAN_2;
    drv8328.faultIn     = HAL_FAULT_INPUT_1;

    tida010251.led      = HAL_GPIO_OUT_03;
    tida010251.dir      = HAL_GPIO_IN_04;
    tida010251.trigger  = HAL_GPIO_IN_05;
    tida010251.Vsen     = HAL_ADC_CHAN_0;
    tida010251.speed    = HAL_ADC_CHAN_1;

    hallTrap.hallA      = HAL_GPIO_IN_01;
    hallTrap.hallB      = HAL_GPIO_IN_02;
    hallTrap.hallC      = HAL_GPIO_IN_03;
    hallTrap.phaseA     = HAL_PWM_02;
    hallTrap.phaseB     = HAL_PWM_03;
    hallTrap.phaseC     = HAL_PWM_01;
    hallTrap.captureInput = HAL_CAPTURE_TIMER_01;

    Halltrap_Init(&hallTrap);
    HAL_enableADCInterrupt(tida010251.Vsen);
    HAL_enableADCInterrupt(drv8328.Isen);
    HAL_enableADCInterrupt(tida010251.speed);

    HAL_writeGPIOPin(tida010251.led, HAL_GPIO_PIN_HIGH);

    /* set GUI parameters   */
    GUI_setupGuiValues(&drv8328);

    /* enable interrupt to increment or decrement the dutycycle */
    HAL_enablePWMInterrupt(HAL_PWM_03);

    HAL_clearTimerFault(drv8328.faultIn);
    Halltrap_disablePWM(&hallTrap);

    while(1)
    {
        GUI_changeCheck(&tida010251, &drv8328, &hallTrap);
    }
}

void PWM_0_INST_IRQHandler(void)
{
    GUI_accMotor(&hallTrap);
    guiSpeedTimeout++;

    if(guiSpeedTimeout >= firmVar.pwmFreq)
    {
        guiMotorSpeed = 0;
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t gpioV = HAL_getGPIOEnabledInterruptStatus(tida010251.trigger);

    if ((gpioV & gpioInputPin[tida010251.trigger].pin) ==
            gpioInputPin[tida010251.trigger].pin)
    {
        motorState = HAL_readGPIOPin(tida010251.trigger);
    }
    else
    {
        if(motorState == GUI_MOTOR_RUN)
        {
            Halltrap_PWMUpdate(&hallTrap, firmVar.motorDirection, firmVar.pulseWidth);
        }
        else
        {
            Halltrap_stopMotor(&hallTrap, firmVar.motorBraketype);
        }
    }
}

#ifdef GEN_ADC_CHAN_0_INST
void GEN_ADC_CHAN_0_INST_IRQHandler(void)
{
    HAL_ADC_IIDX pend_irq = HAL_processADCIRQ(GEN_ADC_CHAN_0_INST);
    guiADCStatus |= GUI_ADC0_DATA_READY;
}
#endif

#ifdef GEN_ADC_CHAN_1_INST
void GEN_ADC_CHAN_1_INST_IRQHandler(void)
{
    HAL_ADC_IIDX pend_irq = HAL_processADCIRQ(GEN_ADC_CHAN_1_INST);
    guiADCStatus |= GUI_ADC1_DATA_READY;
}
#endif

void CAPTURE_0_INST_IRQHandler(void)
{
    guiMotorSpeed = Halltrap_calculateMotorSpeed(&hallTrap, guiMotorPoles);
    guiSpeedTimeout = 0;
}
