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
    SYSCFG_DL_I2C_init();
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_I2C_reset(I2C_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_I2C_enablePower(I2C_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    const uint8_t unusedPinIndexes[] =
    {
        IOMUX_PINCM30, IOMUX_PINCM31, IOMUX_PINCM32, IOMUX_PINCM33,
        IOMUX_PINCM34, IOMUX_PINCM35, IOMUX_PINCM36, IOMUX_PINCM37,
        IOMUX_PINCM38, IOMUX_PINCM39, IOMUX_PINCM40, IOMUX_PINCM43,
        IOMUX_PINCM44, IOMUX_PINCM45, IOMUX_PINCM46, IOMUX_PINCM47,
        IOMUX_PINCM48, IOMUX_PINCM49, IOMUX_PINCM50, IOMUX_PINCM51,
        IOMUX_PINCM52, IOMUX_PINCM53, IOMUX_PINCM54, IOMUX_PINCM55,
        IOMUX_PINCM56, IOMUX_PINCM58, IOMUX_PINCM59, IOMUX_PINCM60,
        IOMUX_PINCM1, IOMUX_PINCM2, IOMUX_PINCM3, IOMUX_PINCM4,
        IOMUX_PINCM5, IOMUX_PINCM6, IOMUX_PINCM7, IOMUX_PINCM8,
        IOMUX_PINCM9, IOMUX_PINCM10, IOMUX_PINCM11, IOMUX_PINCM12,
        IOMUX_PINCM13, IOMUX_PINCM14, IOMUX_PINCM17, IOMUX_PINCM18,
        IOMUX_PINCM19, IOMUX_PINCM20, IOMUX_PINCM21, IOMUX_PINCM22,
        IOMUX_PINCM23, IOMUX_PINCM24, IOMUX_PINCM25, IOMUX_PINCM26,
        IOMUX_PINCM27, IOMUX_PINCM28, IOMUX_PINCM29
    };

    for(int i = 0; i < sizeof(unusedPinIndexes)/sizeof(unusedPinIndexes[0]); i++)
    {
        DL_GPIO_initDigitalOutput(unusedPinIndexes[i]);
    }

    DL_GPIO_clearPins(GPIOA,
        (DL_GPIO_PIN_12 | DL_GPIO_PIN_13 | DL_GPIO_PIN_14 | DL_GPIO_PIN_15 |
        DL_GPIO_PIN_16 | DL_GPIO_PIN_17 | DL_GPIO_PIN_18 | DL_GPIO_PIN_21 |
        DL_GPIO_PIN_22 | DL_GPIO_PIN_23 | DL_GPIO_PIN_24 | DL_GPIO_PIN_25 |
        DL_GPIO_PIN_26 | DL_GPIO_PIN_27 | DL_GPIO_PIN_0 | DL_GPIO_PIN_1 |
        DL_GPIO_PIN_28 | DL_GPIO_PIN_29 | DL_GPIO_PIN_30 | DL_GPIO_PIN_31 |
        DL_GPIO_PIN_2 | DL_GPIO_PIN_3 | DL_GPIO_PIN_4 | DL_GPIO_PIN_5 |
        DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_8 | DL_GPIO_PIN_9 |
        DL_GPIO_PIN_10 | DL_GPIO_PIN_11));
    DL_GPIO_enableOutput(GPIOA,
        (DL_GPIO_PIN_12 | DL_GPIO_PIN_13 | DL_GPIO_PIN_14 | DL_GPIO_PIN_15 |
        DL_GPIO_PIN_16 | DL_GPIO_PIN_17 | DL_GPIO_PIN_18 | DL_GPIO_PIN_21 |
        DL_GPIO_PIN_22 | DL_GPIO_PIN_23 | DL_GPIO_PIN_24 | DL_GPIO_PIN_25 |
        DL_GPIO_PIN_26 | DL_GPIO_PIN_27 | DL_GPIO_PIN_0 | DL_GPIO_PIN_1 |
        DL_GPIO_PIN_28 | DL_GPIO_PIN_29 | DL_GPIO_PIN_30 | DL_GPIO_PIN_31 |
        DL_GPIO_PIN_2 | DL_GPIO_PIN_3 | DL_GPIO_PIN_4 | DL_GPIO_PIN_5 |
        DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_8 | DL_GPIO_PIN_9 |
        DL_GPIO_PIN_10 | DL_GPIO_PIN_11));
    DL_GPIO_clearPins(GPIOB,
        (DL_GPIO_PIN_13 | DL_GPIO_PIN_14 | DL_GPIO_PIN_15 | DL_GPIO_PIN_16 |
        DL_GPIO_PIN_17 | DL_GPIO_PIN_18 | DL_GPIO_PIN_19 | DL_GPIO_PIN_20 |
        DL_GPIO_PIN_21 | DL_GPIO_PIN_22 | DL_GPIO_PIN_23 | DL_GPIO_PIN_24 |
        DL_GPIO_PIN_25 | DL_GPIO_PIN_27 | DL_GPIO_PIN_0 | DL_GPIO_PIN_1 |
        DL_GPIO_PIN_4 | DL_GPIO_PIN_5 | DL_GPIO_PIN_6 | DL_GPIO_PIN_7 |
        DL_GPIO_PIN_8 | DL_GPIO_PIN_9 | DL_GPIO_PIN_10 | DL_GPIO_PIN_11 |
        DL_GPIO_PIN_12));
    DL_GPIO_enableOutput(GPIOB,
        (DL_GPIO_PIN_13 | DL_GPIO_PIN_14 | DL_GPIO_PIN_15 | DL_GPIO_PIN_16 |
        DL_GPIO_PIN_17 | DL_GPIO_PIN_18 | DL_GPIO_PIN_19 | DL_GPIO_PIN_20 |
        DL_GPIO_PIN_21 | DL_GPIO_PIN_22 | DL_GPIO_PIN_23 | DL_GPIO_PIN_24 |
        DL_GPIO_PIN_25 | DL_GPIO_PIN_27 | DL_GPIO_PIN_0 | DL_GPIO_PIN_1 |
        DL_GPIO_PIN_4 | DL_GPIO_PIN_5 | DL_GPIO_PIN_6 | DL_GPIO_PIN_7 |
        DL_GPIO_PIN_8 | DL_GPIO_PIN_9 | DL_GPIO_PIN_10 | DL_GPIO_PIN_11 |
        DL_GPIO_PIN_12));

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_IOMUX_SDA,
        GPIO_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_IOMUX_SCL,
        GPIO_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_IOMUX_SCL);

    DL_GPIO_initDigitalOutput(GPIO_LEDS_USER_LED_1_IOMUX);

    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    DL_GPIO_enableOutput(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

}


static const DL_I2C_ClockConfig gI2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_init(void) {

    DL_I2C_setClockConfig(I2C_INST,
        (DL_I2C_ClockConfig *) &gI2CClockConfig);
    DL_I2C_disableAnalogGlitchFilter(I2C_INST);

    /* Configure Target Mode */
    DL_I2C_setTargetOwnAddress(I2C_INST, I2C_TARGET_OWN_ADDR);
    DL_I2C_setTargetTXFIFOThreshold(I2C_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setTargetRXFIFOThreshold(I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);

    DL_I2C_enableTargetClockStretching(I2C_INST);
    /* Enable module */
    DL_I2C_enableTarget(I2C_INST);


}

