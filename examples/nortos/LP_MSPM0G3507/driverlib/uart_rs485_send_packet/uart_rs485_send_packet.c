/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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

#include "ti_msp_dl_config.h"

/* Number of bytes for UART packet size, the maximum FIFO size is 4 */
#define UART_PACKET_SIZE (4)

/* Delay for 5ms to ensure UART TX is idle before starting transmission */
#define UART_TX_DELAY (160000)

/* Data for UART to transmit */
uint8_t gTxPacket[UART_PACKET_SIZE] = {'M', 'S', 'P', '!'};

int main(void)
{
    SYSCFG_DL_init();

    /* Optional delay to ensure UART TX is idle before starting transmission */
    delay_cycles(UART_TX_DELAY);

    /* Transmit data, set DE high before */
    DL_GPIO_setPins(GPIO_DE_TX_PORT, GPIO_DE_TX_TRANSMITTER_PIN);
    DL_UART_Main_fillTXFIFO(UART_0_INST, &gTxPacket[0], UART_PACKET_SIZE);

    /*
     * Wait until all bytes have been transmitted and the TX FIFO
     * is empty
     */
    while (DL_UART_isBusy(UART_0_INST))
        ;

    /* Set DE low after transmission */
    DL_GPIO_clearPins(GPIO_DE_TX_PORT, GPIO_DE_TX_TRANSMITTER_PIN);

    /* If write successful, toggle LED */
    while (1) {
        DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
        delay_cycles(5000000);
    }
}
