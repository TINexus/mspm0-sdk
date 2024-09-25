/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 * This project provides a simple blinky style project.
 * The simply blinky demo is implemented and described in main_blinky.c.
 *
 * The blinky demo uses FreeRTOS's tickless idle mode to reduce power
 * consumption.
 *
 * This file implements the code that is not demo specific.
 */

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* TI includes */
#include "ti_msp_dl_config.h"

/*-----------------------------------------------------------*/

/*
 * Set up the hardware ready to run this demo.
 */
static void prvSetupHardware(void);

extern void main_blinky(void);

/*-----------------------------------------------------------*/

int main(void)
{
    /* Prepare the hardware to run this demo. */
    prvSetupHardware();

    main_blinky();

    return 0;
}
/*-----------------------------------------------------------*/

static void prvSetupHardware(void)
{
    SYSCFG_DL_init();
}
/*-----------------------------------------------------------*/

// #if (configSUPPORT_STATIC_ALLOCATION == 1)
// /*
//  *  ======== vApplicationGetIdleTaskMemory ========
//  *  When static allocation is enabled, the app must provide this callback
//  *  function for use by the Idle task.
//  */
// void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
//     StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
// {
//     static StaticTask_t xIdleTaskTCB;
//     static StackType_t uxIdleTaskStack[configIDLE_TASK_STACK_DEPTH];

//     *ppxIdleTaskTCBBuffer   = &xIdleTaskTCB;
//     *ppxIdleTaskStackBuffer = uxIdleTaskStack;
//     *pulIdleTaskStackSize   = configIDLE_TASK_STACK_DEPTH;
// }

// #if (configUSE_TIMERS == 1)
// /*
//  *  ======== vApplicationGetTimerTaskMemory ========
//  *  When static allocation is enabled, and timers are used, the app must provide
//  *  this callback function for use by the Timer Service task.
//  */
// void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
//     StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
// {
//     static StaticTask_t xTimerTaskTCB;
//     static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

//     *ppxTimerTaskTCBBuffer   = &xTimerTaskTCB;
//     *ppxTimerTaskStackBuffer = uxTimerTaskStack;
//     *pulTimerTaskStackSize   = configTIMER_TASK_STACK_DEPTH;
// }
// #endif

// #endif

#if (configCHECK_FOR_STACK_OVERFLOW)
/*
     *  ======== vApplicationStackOverflowHook ========
     *  When stack overflow checking is enabled the application must provide a
     *  stack overflow hook function. This default hook function is declared as
     *  weak, and will be used by default, unless the application specifically
     *  provides its own hook function.
     */
#if defined(__IAR_SYSTEMS_ICC__)
__weak void vApplicationStackOverflowHook(
    TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__TI_COMPILER_VERSION__))
#pragma WEAK(vApplicationStackOverflowHook)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#elif (defined(__GNUC__) || defined(__ti_version__))
void __attribute__((weak))
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#endif
{
    /* default to spin upon stack overflow */
    while (1) {
    }
}
#endif

/*-----------------------------------------------------------*/
