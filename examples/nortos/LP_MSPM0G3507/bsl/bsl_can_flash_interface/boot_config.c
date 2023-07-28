/*
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
 */

/*
 *  ============ boot_config.c =============
 *  Configured MSPM0 NONMAIN options
 *
 */

#include "boot_config.h"


/* Bootcode configuration */

PLACE_IN_MEMORY(".BCRConfig")
const BCR_Config BCRConfig =
{
    .bcrConfigID          = 0x1,
    .debugAccess          = BCR_CFG_DEBUG_ACCESS_EN,
    .swdpMode             = BCR_CFG_SWDP_EN,
    .tifaMode             = BCR_CFG_TIFA_EN,
    .bslPinInvokeEnable   = BCR_CFG_BSL_PIN_INVOKE_EN,
    .passwordDebugLock    = {CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE,
        CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE},
    .fastBootMode         = BCR_CFG_FAST_BOOT_DIS,
    .bootloaderMode       = BCR_CFG_BOOTLOADER_MODE_EN,
    .massEraseMode        = BCR_CFG_MASS_ERASE_EN,
    .factoryResetMode     = BCR_CFG_FACTORY_RESET_EN,
    .passwordMassErase    = {CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE,
        CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE},
    .passwordFactoryReset = {CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE,
        CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE},
    .staticWriteProtectionMainLow  = 0xfff0000f,
    .staticWriteProtectionMainHigh = CFG_DEFAULT_VALUE,
    .staticWriteProtectionNonMain  = BCR_CFG_NON_MAIN_STATIC_PROT_EN,
    .secureBootMode                = BCR_CFG_SECURE_BOOT_DIS,
    .userSecureAppStartAddr        = CFG_DEFAULT_VALUE,
    .userSecureAppLength           = CFG_DEFAULT_VALUE,
    .userSecureAppCrc              = CFG_DEFAULT_VALUE,
    .userCfgCRC                    = 0xfb9aa786,
};

/* Bootloader configuration */

PLACE_IN_MEMORY(".BSLConfig")
const BSL_Config BSLConfig =
{
    .configID                          = 0x1,
    .interfacePins.UART_RXD_pad_num    = DEF_UART_RXD_PAD,
    .interfacePins.UART_RXD_PF_mux_sel = DEF_UART_RXD_MUX,
    .interfacePins.UART_TXD_pad_num    = DEF_UART_TXD_PAD,
    .interfacePins.UART_TXD_PF_mux_sel = DEF_UART_TXD_MUX,
    .interfacePins.I2C_SDA_pad_num     = DEF_I2C_SDA_PAD,
    .interfacePins.I2C_SDA_PF_mux_sel  = DEF_I2C_SDA_MUX,
    .interfacePins.I2C_SCL_pad_num     = DEF_I2C_SCL_PAD,
    .interfacePins.I2C_SCL_PF_mux_sel  = DEF_I2C_SCL_MUX,
    .pin.pinData0                      = DEFAULT_BSL_PIN_INVOCATION_DATA0,
    .pin.pinData1                      = DEFAULT_BSL_PIN_INVOCATION_DATA1,
    .memoryRead         = BSL_CFG_MEMORY_READOUT_DISABLE,
    .password           = {CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE,
        CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE,
        CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE, CFG_DEFAULT_VALUE},
    .pluginType         = BSL_CFG_PLUGIN_TYPE_ANY,
    .flashPluginEnable  = BSL_CFG_PLUGIN_FLASH_EXIST,
    .pluginSramSize     = 0xff,
    .pluginHook[0]      = (uint32_t) BSL_PI_MCAN_init,
    .pluginHook[1]      = (uint32_t) BSL_PI_MCAN_receive,
    .pluginHook[2]      = (uint32_t) BSL_PI_MCAN_send,
    .pluginHook[3]      = (uint32_t) BSL_PI_MCAN_deinit,
    .BSLAlternateConfig = BSL_CFG_FLASH_BSL_NOT_EXIST,
    .reserved           = 0xFFFF,
    .BSLAlternateAddr   = CFG_DEFAULT_VALUE,
    .appRev             = (uint32_t *) CFG_DEFAULT_VALUE,
    .securityAlert      = BSL_CFG_SECURITY_IGNORE,
    .i2cSlaveAddress    = 0x48,
    .userCfgCRC         = 0x79d4b7e0,
};

/* Added for secondary_bsl to build */
/* 'main' function will never get called */
#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
__attribute__((weak)) int main(void)
{
    return (0);
}
#elif defined(__IAR_SYSTEMS_ICC__)
__weak int main(void);
#elif (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) || defined(__GNUC__)
__attribute__((weak)) int main(void);
#else
#error "Non-Main table currently not supported for this compiler"
#endif
