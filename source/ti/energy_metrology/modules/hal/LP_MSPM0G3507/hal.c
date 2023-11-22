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
#include "hal.h"

/*!
 * @brief Array for storing the output GPIO pin instances
 */
HAL_GPIOInstance    gpioOutputPin[HAL_GPIO_OUT_MAX];

/*!
 * @brief Array for storing the input GPIO pin instances
 */
HAL_GPIOInstance    gpioInputPin[HAL_GPIO_IN_MAX];

/*!
 * @brief Array for storing the SPI instances
 */
HAL_SPIInstance     SPIChannel[HAL_SPI_CHAN_MAX];

/*!
 * @brief Array for storing the chip selects for SPI peripheral
 */
DL_SPI_CHIP_SELECT  SPICS[HAL_SPI_CS_MAX];

/*!
 * @brief  Initializes the hal module
 */
void HAL_init()
{
    SPIChannel[HAL_SPI_CHAN_0].inst        = GEN_SPI_0_INST;
    SPIChannel[HAL_SPI_CHAN_0].dmaChanIdTx = DMA_CH0_CHAN_ID;
    SPIChannel[HAL_SPI_CHAN_0].dmaChanIdRx = DMA_CH1_CHAN_ID;

    SPICS[HAL_SPI_CS_0] = DL_SPI_CHIP_SELECT_0;
    SPICS[HAL_SPI_CS_1] = DL_SPI_CHIP_SELECT_1;
    SPICS[HAL_SPI_CS_2] = DL_SPI_CHIP_SELECT_2;
    SPICS[HAL_SPI_CS_3] = DL_SPI_CHIP_SELECT_3;

    /* Define GPIO output array */
    gpioOutputPin[HAL_GPIO_OUT_00].iomux  = GEN_GPIO_OUT_PINO_01_IOMUX;
    gpioOutputPin[HAL_GPIO_OUT_00].port   = GEN_GPIO_OUT_PINO_01_PORT;
    gpioOutputPin[HAL_GPIO_OUT_00].pin    = GEN_GPIO_OUT_PINO_01_PIN;

    gpioOutputPin[HAL_GPIO_OUT_01].iomux  = GEN_GPIO_OUT_PINO_02_IOMUX;
    gpioOutputPin[HAL_GPIO_OUT_01].port   = GEN_GPIO_OUT_PINO_02_PORT;
    gpioOutputPin[HAL_GPIO_OUT_01].pin    = GEN_GPIO_OUT_PINO_02_PIN;

    gpioOutputPin[HAL_GPIO_OUT_02].iomux  = GEN_GPIO_OUT_PINO_03_IOMUX;
    gpioOutputPin[HAL_GPIO_OUT_02].port   = GEN_GPIO_OUT_PINO_03_PORT;
    gpioOutputPin[HAL_GPIO_OUT_02].pin    = GEN_GPIO_OUT_PINO_03_PIN;

#ifdef GEN_GPIO_OUT_PINO_04_PIN
    gpioOutputPin[HAL_GPIO_OUT_03].iomux  = GEN_GPIO_OUT_PINO_04_IOMUX;
    gpioOutputPin[HAL_GPIO_OUT_03].port   = GEN_GPIO_OUT_PINO_04_PORT;
    gpioOutputPin[HAL_GPIO_OUT_03].pin    = GEN_GPIO_OUT_PINO_04_PIN;
#endif

    /* Define GPIO input array */
    gpioInputPin[HAL_GPIO_IN_00].iomux    = GEN_GPIO_IN_PINI_01_IOMUX;
    gpioInputPin[HAL_GPIO_IN_00].port     = GEN_GPIO_IN_PINI_01_PORT;
    gpioInputPin[HAL_GPIO_IN_00].pin      = GEN_GPIO_IN_PINI_01_PIN;
    gpioInputPin[HAL_GPIO_IN_00].IRQn     = GEN_GPIO_IN_INT_IRQN;

#ifdef GEN_GPIO_IN_PINI_02_PIN
    gpioInputPin[HAL_GPIO_IN_01].iomux    = GEN_GPIO_IN_PINI_02_IOMUX;
    gpioInputPin[HAL_GPIO_IN_01].port     = GEN_GPIO_IN_PINI_02_PORT;
    gpioInputPin[HAL_GPIO_IN_01].pin      = GEN_GPIO_IN_PINI_02_PIN;
    gpioInputPin[HAL_GPIO_IN_01].IRQn     = GEN_GPIO_IN_INT_IRQN;
#endif
}

/*!
 * @brief     reads a gpio
 * @param[in] pin   gpiopin
 * @return    status of pin
 */
bool HAL_readGPIOPin(HAL_GPIO_IN pin)
{
    return(DL_GPIO_readPins(gpioInputPin[pin].port, gpioInputPin[pin].pin));
}

/*!
 * @brief     write to a gpio
 * @param[in] pin       gpiopin
 * @param[in] pinState  pin state
 */
void HAL_writeGPIOPin(HAL_GPIO_OUT pin, HAL_GPIO_STATE pinState)
{
    if (pinState == HAL_GPIO_PIN_HIGH)
    {
        DL_GPIO_setPins(gpioOutputPin[pin].port, gpioOutputPin[pin].pin);
    }
    else if (pinState == HAL_GPIO_PIN_LOW)
    {
        DL_GPIO_clearPins(gpioOutputPin[pin].port, gpioOutputPin[pin].pin);
    }
    else
    {
        /* Code shouldn't get here  */
    }
}

/*!
 * @brief     enables GPIO Interrupt
 * @param[in] pin       gpiopin
 */
void HAL_enableGPIOInterrupt(HAL_GPIO_IN pin)
{
    DL_GPIO_enableInterrupt(gpioInputPin[pin].port, gpioInputPin[pin].pin);
    NVIC_EnableIRQ(gpioInputPin[pin].IRQn);
}

/*!
 * @brief Start SPI data transfer through DMA
 * @param[in]  dataTx[]  The transfer data
 * @param[in]  dataRx[]  The receive data
 * @param[in]  byteLength Number of bytes transfered
 * @param[in]  chan      The SPI channel
 * @param[in]  cs        The SPI Chip select pin
 */
void HAL_startSPIDataTransfer(uint8_t dataTx[], uint8_t dataRx[], uint8_t byteLength, HAL_SPI_CHAN chan, HAL_SPI_CS cs)
{
    SPI_Regs *spi = SPIChannel[chan].inst;
    DL_SPI_setChipSelect(spi, (DL_SPI_CHIP_SELECT)SPICS[cs]);
    uint8_t dmaTxId = SPIChannel[chan].dmaChanIdTx;
    uint8_t dmaRxId = SPIChannel[chan].dmaChanIdRx;

    DL_DMA_setSrcAddr(DMA, dmaTxId, (uint32_t) &dataTx[0]);
    DL_DMA_setDestAddr(DMA, dmaTxId, (uint32_t)(&spi->TXDATA));
    DL_DMA_setTransferSize( DMA, dmaTxId, byteLength);

    DL_DMA_setSrcAddr(DMA, dmaRxId, (uint32_t)(&spi->RXDATA));
    DL_DMA_setDestAddr(DMA, dmaRxId, (uint32_t) &dataRx[0]);
    DL_DMA_setTransferSize(DMA, dmaRxId, byteLength);
    DL_DMA_enableChannel(DMA, dmaRxId);

    DL_SYSCTL_disableSleepOnExit();

    DL_DMA_enableChannel(DMA, dmaTxId);
}

/*!
 * @brief clear Memory block
 * @param[in] startAddr  The start address of Memory block
 */
void HAL_clearMemoryBlock(HAL_MEMORY_BLOCK_STARTADDR startAddr)
{
    DL_FlashCTL_unprotectSector(FLASHCTL, startAddr, DL_FLASHCTL_REGION_SELECT_MAIN);
    DL_FlashCTL_eraseMemory(FLASHCTL, startAddr, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    DL_FlashCTL_waitForCmdDone(FLASHCTL);
}

/*!
 * @brief secure Memory block
 * @param[in] startAddr  The start address of Memory block
 */
void HAL_secureMemoryBlock(HAL_MEMORY_BLOCK_STARTADDR startAddr)
{
    DL_FlashCTL_protectSector(FLASHCTL, startAddr, DL_FLASHCTL_REGION_SELECT_MAIN);
}

/*!
 * @brief copy block data to flash
 * @param[in] dstAddr   The pointer to destination address
 * @param[in] srcAddr   The pointer to source address
 * @param[in] len The length of data
 * @param[in] startAddr  The start address of Memory block
*  @return status
 */
int HAL_copyMemoryBlock(void *dstAddr, void *srcAddr, int len, HAL_MEMORY_BLOCK_STARTADDR startAddr)
{
    uint8_t pad[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint32_t remainder = len % 8;
    uint32_t prog64bytes = len/8;
    uint32_t prog64length = prog64bytes * 2; /* FLASHCTL expects addresses in 32-bit chunks */

    if(remainder)
    {
        int i = remainder;
        uint8_t * srcPtr = ( ((uint8_t *) srcAddr)  + prog64bytes * 8); /* at the start of the remainder    */
        for(i = 0; i < remainder; i++)
        {
            *(pad + i) = *(srcPtr + i);
        }
    }

    if(prog64bytes > 0)
    {
       if(((uint32_t) dstAddr) % 8 != 0)
       {
           return -1;
       }

        DL_FlashCTL_programMemoryBlocking64WithECCGenerated(FLASHCTL, (uint32_t) dstAddr, (uint32_t *) srcAddr,
           prog64length, DL_FLASHCTL_REGION_SELECT_MAIN);
        DL_FlashCTL_waitForCmdDone(FLASHCTL);
    }

    if(remainder)
    {
        DL_FlashCTL_unprotectSector(FLASHCTL, startAddr, DL_FLASHCTL_REGION_SELECT_MAIN);
        DL_FlashCTL_programMemory64WithECCGenerated(FLASHCTL, (uint32_t) (((uint32_t *) dstAddr) + (prog64bytes * 2)), (uint32_t *) pad);
    }
    DL_FlashCTL_waitForCmdDone(FLASHCTL);

    return 0;
}

/*!
 * @brief Get real time clock
 * @param[in] buf[] buffer to store clock data
 */
void HAL_getRTC(uint8_t buf[6])
{
    DL_RTC_Calendar RTC_values;
    RTC_values = DL_RTC_getCalendarTime(RTC);
    buf[0]     = RTC_values.seconds;
    buf[1]     = RTC_values.minutes;
    buf[2]     = RTC_values.hours;
    buf[3]     = RTC_values.dayOfWeek;
    buf[4]     = RTC_values.dayOfMonth;
    buf[5]     = RTC_values.month;
    buf[6]     = RTC_values.year;
}

/*!
 * @brief Set real time clock
 * @param[in] buf[] buffer to store clock data
 */
void HAL_setRTC(const uint8_t buf[6])
{
    DL_RTC_Calendar new_time;
    new_time.year       = buf[6];
    new_time.month      = buf[5];
    new_time.dayOfMonth = buf[4];
    new_time.dayOfWeek  = buf[3];
    new_time.hours      = buf[2];
    new_time.minutes    = buf[1];
    new_time.seconds    = buf[0];

    DL_RTC_initCalendar(RTC, new_time, DL_RTC_FORMAT_BINARY);
    DL_RTC_setOffsetCalibrationFrequency(RTC, RTC_CAL_RTCCALFX_F1HZ );
    DL_RTC_setOffsetCalibrationAdjValue(RTC, DL_RTC_OFFSET_CALIBRATION_SIGN_DOWN, 0);
    DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_READY);

    DL_RTC_enableInterrupt(RTC, DL_RTC_INTERRUPT_PRESCALER1);

    DL_RTC_enableClockControl(RTC);
}

