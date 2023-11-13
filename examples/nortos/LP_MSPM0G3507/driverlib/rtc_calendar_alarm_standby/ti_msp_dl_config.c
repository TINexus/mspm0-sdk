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

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the LP_MSPM0G3507
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
    SYSCFG_DL_RTC_init();
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_RTC_reset(RTC);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_RTC_enablePower(RTC);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initDigitalOutput(GPIO_LEDS_USER_LED_1_IOMUX);

    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    DL_GPIO_enableOutput(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();

	//Low Power Mode is configured to be STANDBY0
    DL_SYSCTL_setPowerPolicySTANDBY0();
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

}


static const DL_RTC_Calendar gRTCCalendarConfig = {
    .seconds    = 58,   /* Seconds = 58 */
    .minutes    = 32,   /* Minute = 32 */
    .hours      = 10,   /* Hour = 10 */
    .dayOfWeek  = 1,    /* Day of week = 1 (Monday) */
    .dayOfMonth = 5,    /* Day of month = 5 */
    .month      = 4,    /* Month = 4 (April) */
    .year       = 2021, /* Year = 2021 */
};

static const DL_RTC_CalendarAlarm gRTCAlarm1Config = {
    .minutes    = 33, /* Minute = 33 */
    .hours      = 10, /* Hour = 10 */
    .dayOfWeek  = 1,  /* Day of week = 1 (Monday) */
    .dayOfMonth = 5,  /* Day of month = 5 */
};

SYSCONFIG_WEAK void SYSCFG_DL_RTC_init(void)
{
    /* Initialize RTC Calendar */
    DL_RTC_initCalendar(RTC, gRTCCalendarConfig, DL_RTC_FORMAT_BINARY);

    /* Clear RTC Calendar Alarm 1 */
    DL_RTC_disableAlarm1MinutesBinary(RTC);
    DL_RTC_disableAlarm1HoursBinary(RTC);
    DL_RTC_disableAlarm1DayOfWeekBinary(RTC);
    DL_RTC_disableAlarm1DayOfMonthBinary(RTC);
    DL_RTC_disableInterrupt(RTC, DL_RTC_INTERRUPT_CALENDAR_ALARM1);

    /* Configure RTC Calendar Alarm 1 */
    DL_RTC_setCalendarAlarm1(RTC, gRTCAlarm1Config);
    DL_RTC_enableAlarm1MinutesBinary(RTC);
    DL_RTC_enableAlarm1HoursBinary(RTC);
    DL_RTC_enableAlarm1DayOfWeekBinary(RTC);
    DL_RTC_enableAlarm1DayOfMonthBinary(RTC);

    /* Configure Interrupts */
    DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_CALENDAR_ALARM1);





}

