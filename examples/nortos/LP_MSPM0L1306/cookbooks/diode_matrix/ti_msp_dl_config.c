/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the LP_MSPM0L1306
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_LED_STATE_init();
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_TimerG_reset(LED_STATE_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_TimerG_enablePower(LED_STATE_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initDigitalOutput(COLUMN_COLUMN_1_IOMUX);

    DL_GPIO_initDigitalOutput(COLUMN_COLUMN_2_IOMUX);

    DL_GPIO_initDigitalOutput(COLUMN_COLUMN_3_IOMUX);

    DL_GPIO_initDigitalOutput(ROW_ROW_1_IOMUX);

    DL_GPIO_initDigitalOutput(ROW_ROW_2_IOMUX);

    DL_GPIO_initDigitalOutput(ROW_ROW_3_IOMUX);

    DL_GPIO_setPins(GPIOA, COLUMN_COLUMN_1_PIN |
		COLUMN_COLUMN_2_PIN |
		COLUMN_COLUMN_3_PIN |
		ROW_ROW_1_PIN |
		ROW_ROW_2_PIN |
		ROW_ROW_3_PIN);
    DL_GPIO_enableOutput(GPIOA, COLUMN_COLUMN_1_PIN |
		COLUMN_COLUMN_2_PIN |
		COLUMN_COLUMN_3_PIN |
		ROW_ROW_1_PIN |
		ROW_ROW_2_PIN |
		ROW_ROW_3_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 4000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gLED_STATEClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * LED_STATE_INST_LOAD_VALUE = (5 ms * 4000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gLED_STATETimerConfig = {
    .period     = LED_STATE_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_LED_STATE_init(void) {

    DL_TimerG_setClockConfig(LED_STATE_INST,
        (DL_TimerG_ClockConfig *) &gLED_STATEClockConfig);

    DL_TimerG_initTimerMode(LED_STATE_INST,
        (DL_TimerG_TimerConfig *) &gLED_STATETimerConfig);
    DL_TimerG_enableInterrupt(LED_STATE_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
    DL_TimerG_enableClock(LED_STATE_INST);




}


