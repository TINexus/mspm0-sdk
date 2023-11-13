/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the LP_MSPM0L1306
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_LP_MSPM0L1306

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for LIN_0 */
#define LIN_0_INST                                                         UART0
#define LIN_0_INST_IRQHandler                                   UART0_IRQHandler
#define LIN_0_INST_INT_IRQN                                       UART0_INT_IRQn
#define GPIO_LIN_0_RX_PORT                                                 GPIOA
#define GPIO_LIN_0_TX_PORT                                                 GPIOA
#define GPIO_LIN_0_RX_PIN                                          DL_GPIO_PIN_9
#define GPIO_LIN_0_TX_PIN                                          DL_GPIO_PIN_8
#define GPIO_LIN_0_IOMUX_RX                                      (IOMUX_PINCM10)
#define GPIO_LIN_0_IOMUX_TX                                       (IOMUX_PINCM9)
#define GPIO_LIN_0_IOMUX_RX_FUNC                       IOMUX_PINCM10_PF_UART0_RX
#define GPIO_LIN_0_IOMUX_TX_FUNC                        IOMUX_PINCM9_PF_UART0_TX
#define LIN_0_BAUD_RATE                                                  (19200)
#define LIN_0_IBRD_32_MHZ_19200_BAUD                                       (104)
#define LIN_0_FBRD_32_MHZ_19200_BAUD                                        (11)
#define LIN_0_TBIT_WIDTH                                                  (1666)





/* Port definition for Pin Group GPIO_LIN_ENABLE */
#define GPIO_LIN_ENABLE_PORT                                             (GPIOA)

/* Defines for USER_LIN_ENABLE: GPIOA.22 with pinCMx 23 on package pin 26 */
#define GPIO_LIN_ENABLE_USER_LIN_ENABLE_PIN                     (DL_GPIO_PIN_22)
#define GPIO_LIN_ENABLE_USER_LIN_ENABLE_IOMUX                    (IOMUX_PINCM23)
/* Port definition for Pin Group GPIO_LEDS */
#define GPIO_LEDS_PORT                                                   (GPIOA)

/* Defines for USER_LED_1: GPIOA.26 with pinCMx 27 on package pin 30 */
#define GPIO_LEDS_USER_LED_1_PIN                                (DL_GPIO_PIN_26)
#define GPIO_LEDS_USER_LED_1_IOMUX                               (IOMUX_PINCM27)
/* Defines for USER_LED_2: GPIOA.27 with pinCMx 28 on package pin 31 */
#define GPIO_LEDS_USER_LED_2_PIN                                (DL_GPIO_PIN_27)
#define GPIO_LEDS_USER_LED_2_IOMUX                               (IOMUX_PINCM28)
/* Port definition for Pin Group GPIO_SWITCHES1 */
#define GPIO_SWITCHES1_PORT                                              (GPIOA)

/* Defines for USER_SWITCH_1: GPIOA.18 with pinCMx 19 on package pin 22 */
// pins affected by this interrupt request:["USER_SWITCH_1","USER_SWITCH_2"]
#define GPIO_SWITCHES1_INT_IRQN                                 (GPIOA_INT_IRQn)
#define GPIO_SWITCHES1_INT_IIDX                 (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_SWITCHES1_USER_SWITCH_1_IIDX                   (DL_GPIO_IIDX_DIO18)
#define GPIO_SWITCHES1_USER_SWITCH_1_PIN                        (DL_GPIO_PIN_18)
#define GPIO_SWITCHES1_USER_SWITCH_1_IOMUX                       (IOMUX_PINCM19)
/* Defines for USER_SWITCH_2: GPIOA.14 with pinCMx 15 on package pin 18 */
#define GPIO_SWITCHES1_USER_SWITCH_2_IIDX                   (DL_GPIO_IIDX_DIO14)
#define GPIO_SWITCHES1_USER_SWITCH_2_PIN                        (DL_GPIO_PIN_14)
#define GPIO_SWITCHES1_USER_SWITCH_2_IOMUX                       (IOMUX_PINCM15)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_LIN_0_init(void);



#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
