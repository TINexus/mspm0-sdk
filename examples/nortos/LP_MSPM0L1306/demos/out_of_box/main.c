/* --COPYRIGHT--,BSD
 * Copyright (c) 2022, Texas Instruments Incorporated
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
 * --/COPYRIGHT--*/
//*****************************************************************************
//
//  main.c
//
//  Out of box demo for the MSPM0L1306 LaunchPad
//
//  This demo uses the MSPM0L1306's analog peripherals to showcase several
//  basic analog use cases.
//    - Blinking LED with a configurable blinking rate (via GUI)
//    - The photodiode, ADC, and OPA are used to implement a light sensor with
//      LED feedback
//    - The thermistor and ADC are used to implement a temperature sensor with
//      LED feedback
// ****************************************************************************

#include "ti_msp_dl_config.h"

#include "blink_led.h"
#include "lightsensor.h"
#include "thermistor.h"

#include <stdbool.h>
#include <stdint.h>
#include <ti/gui_composer/HAL.h>
#include <ti/gui_composer/IQMathLib/QmathLib.h>
#include "MSP_GUI/GUIComm_mpack.h"
#include "MSP_GUI/GUI_mpack.h"
#include "callbacks_mpack.h"

/**
 * \brief RX Command structure.
 *        The corresponding callback will be called when the command is
 *        received from GUI.
 * Note: shorter command names take less time to process
 */
const tGUI_RxCmd gGUI_RXCommands[] = {
    {"blinkEnable", callback_blinkEnable},
    {"blinkTick", callback_setBlinkDelay},
    {"lightEnable", callback_lightEnable},
    {"thermEnable", callback_thermistorEnable},
};

int main(void)
{
    /*
     * Initialize peripherals used in the overall out of box i.e. UART and
     * switch to change between examples
     */
    HAL_System_Init();

    /* Initialize GUI layer */
    GUI_Init();
    GUI_InitRxCmd(&gGUI_RXCommands[0],
        (sizeof(gGUI_RXCommands) / sizeof(gGUI_RXCommands[0])));

    while (1) {
        /* The particular demo mode can be changed by GUI or switch */
        switch (gDemoMode) {
            case BLINK_LED_MODE:
                blink_led_runExample();

                break;
            case LIGHTSENSOR_MODE:
                lightsensor_runExample();

                break;
            case THERMISTOR_MODE:
                thermistor_runExample();

                break;
            case IDLE:
                /* LED1 toggles every 500ms in IDLE mode */
                DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

                delay_cycles(16000000);

                break;
            default:
                break;
        }
    }
}
