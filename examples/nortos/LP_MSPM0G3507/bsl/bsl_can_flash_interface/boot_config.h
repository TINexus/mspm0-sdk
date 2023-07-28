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
 *  ============ boot_config.h =============
 *  Configured MSPM0 NONMAIN options
 *
 */

#ifndef boot_config_h
#define boot_config_h

#include <ti/driverlib/dl_common.h>

/* clang-format off */

/* General defines */
#define PASSWORD_WORD_LEN                                                  (4U)

/*! The default value used in the BCR and BSL Config structures */
#define CFG_DEFAULT_VALUE                                         (0xFFFFFFFFU)

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define PLACE_IN_MEMORY(x) __attribute__((section(x))) __attribute__((retain))
#elif defined(__GNUC__) || defined(__clang__)
#define PLACE_IN_MEMORY(x) __attribute__((section(x))) __attribute__((used))
#elif defined(__IAR_SYSTEMS_ICC__)
#define STRINGIFY(a) #a
#define PLACE_IN_MEMORY(x) _Pragma(STRINGIFY(location = x)) __root
#else
#error "Non-Main table currently not supported for this compiler"
#endif

/* Defines for BSL UART and I2C interface pins */
#define DEF_UART_RXD_PAD                                                 (0x16)
#define DEF_UART_RXD_MUX                                                  (0x2)
#define DEF_UART_TXD_PAD                                                 (0x15)
#define DEF_UART_TXD_MUX                                                  (0x2)
#define DEF_I2C_SDA_PAD                                                   (0x1)
#define DEF_I2C_SDA_MUX                                                   (0x3)
#define DEF_I2C_SCL_PAD                                                   (0x2)
#define DEF_I2C_SCL_MUX                                                   (0x3)

#define DEFAULT_BSL_PIN_INVOCATION_DATA0                       ((uint8_t) 0xa8)
#define DEFAULT_BSL_PIN_INVOCATION_DATA1                       ((uint8_t) 0x12)

/**
 * @brief Possible values used for BCR configuration struct
 */

/*! @enum BCR_CFG_DEBUG_ACCESS */
typedef enum {
    /*! AHB-AP, ET-AP, PWR-AP are enabled */
    BCR_CFG_DEBUG_ACCESS_EN = 0xAABBU,
    /*! AHB-AP, ET-AP, PWR-AP are enabled with password through DSSM */
    BCR_CFG_DEBUG_ACCESS_EN_PW = 0xCCDDU,
    /*! AHB-AP, ET-AP, PWR-AP are disabled */
    BCR_CFG_DEBUG_ACCESS_DIS = 0xFFFFU,
} BCR_CFG_DEBUG_ACCESS;

/*! @enum BCR_CFG_SWDP_MODE */
typedef enum {
    /*! SWD port access is enabled */
    BCR_CFG_SWDP_EN = 0xAABBU,
    /*! SWD port access is disabled */
    BCR_CFG_SWDP_DIS = 0xFFFFU,
} BCR_CFG_SWDP_MODE;

/*! @enum BCR_CFG_TIFA_MODE */
typedef enum {
    /*! If enabled, re-test request through DSSM is serviced */
    BCR_CFG_TIFA_EN = 0xAABBU,
    /*! Re-test request through DSSM is not serviced */
    BCR_CFG_TIFA_DIS = 0xFFFFU,
} BCR_CFG_TIFA_MODE;

/*! @enum BCR_CFG_BSL_PIN_INVOKE */
typedef enum {
    /*! GPIO based BSL invocation check will be executed */
    BCR_CFG_BSL_PIN_INVOKE_EN = 0xAABBU,
    /*! GPIO based BSL invocation check will not be executed */
    BCR_CFG_BSL_PIN_INVOKE_DIS = 0xFFFFU,
} BCR_CFG_BSL_PIN_INVOKE;

/*! @enum BCR_CFG_FAST_BOOT */
typedef enum {
    /*! If enabled, checks only the register invoke condition and skips other
     *  BSL invoke conditions */
    BCR_CFG_FAST_BOOT_EN = 0xAABBU,
    /*! If disabled, all enabled BSL invoke conditions are evaluated */
    BCR_CFG_FAST_BOOT_DIS = 0xFFFFU,
} BCR_CFG_FAST_BOOT;

/*! @enum BCR_CFG_BOOTLOADER_MODE */
typedef enum {
    /*! Enable Bootloader mode */
    BCR_CFG_BOOTLOADER_MODE_EN = 0xAABBU,
    /*! Disable Bootloader mode */
    BCR_CFG_BOOTLOADER_MODE_DIS = 0xFFFFU,
} BCR_CFG_BOOTLOADER_MODE;

/*! @enum BCR_CFG_MASS_ERASE */
typedef enum {
    /*! Enable Mass Erase without password */
    BCR_CFG_MASS_ERASE_EN = 0xAABBU,
    /*! Enable Mass Erase with password */
    BCR_CFG_MASS_ERASE_EN_PW = 0xCCDDU,
    /*! Disable Mass Erase */
    BCR_CFG_MASS_ERASE_DIS = 0xFFFFU,
} BCR_CFG_MASS_ERASE;

/*! @enum BCR_CFG_FACTORY_RESET */
typedef enum {
    /*! Enable Factory Reset without password */
    BCR_CFG_FACTORY_RESET_EN = 0xAABBU,
    /*! Enable Factory Reset with password */
    BCR_CFG_FACTORY_RESET_EN_PW = 0xCCDDU,
    /*! Disable Factory Reset */
    BCR_CFG_FACTORY_RESET_DIS = 0xFFFFU,
} BCR_CFG_FACTORY_RESET;

/*! @enum BCR_CFG_NON_MAIN_STATIC_PROT */
typedef enum {
    /*! Enable static write protection for non-main memory */
    BCR_CFG_NON_MAIN_STATIC_PROT_EN = 0xFFFEU,
    /*! Disable static write protection for non-main memory */
    BCR_CFG_NON_MAIN_STATIC_PROT_DIS = 0xFFFFU,
} BCR_CFG_NON_MAIN_STATIC_PROT;

/*! @enum BCR_CFG_SECURE_BOOT */
typedef enum {
    /*! Enable CRC check part of application */
    BCR_CFG_SECURE_BOOT_EN = 0xAABBU,
    /*! Disable CRC check part of application */
    BCR_CFG_SECURE_BOOT_DIS = 0xFFFFU,
} BCR_CFG_SECURE_BOOT;


/* Bootcode user configuration structure */
typedef struct
{
    /*! Configuration signature */
    uint32_t bcrConfigID;
    /*! Enable/disable AHB-AP, ET-AP, PWR-AP.
     * One of @ref BCR_CFG_DEBUG_ACCESS */
    BCR_CFG_DEBUG_ACCESS debugAccess;
    /*! Enable/disable SWD port access. One of @ref BCR_CFG_SWDP_MODE */
    BCR_CFG_SWDP_MODE swdpMode;
    /*! Enable/disable servicing re-test request through DSSM.
     * One of @ref BCR_CFG_TIFA_MODE */
    BCR_CFG_TIFA_MODE tifaMode;
    /*! Enable/disable BSL pin invocation.
    * One of @ref BCR_CFG_BSL_PIN_INVOKE */
    BCR_CFG_BSL_PIN_INVOKE bslPinInvokeEnable;
    /*! Used with Command DSSM_BC_PW_AUTH_REQUEST */
    uint32_t passwordDebugLock [PASSWORD_WORD_LEN];
    /*! The fast boot mode. One of @ref BCR_CFG_FAST_BOOT*/
    BCR_CFG_FAST_BOOT fastBootMode;
    /*! The bootloader mode. One of @ref BCR_CFG_BOOTLOADER_MODE */
    BCR_CFG_BOOTLOADER_MODE bootloaderMode;
    /*! The mass erase mode. One of @ref BCR_CFG_MASS_ERASE */
    BCR_CFG_MASS_ERASE massEraseMode;
    /*! The factory reset mode. One of @ref BCR_CFG_FACTORY_RESET */
    BCR_CFG_FACTORY_RESET factoryResetMode;
    /*! DSSM_BC_MASS_ERASE_REQUEST, DSSM_DATA_EXCHANGE */
    uint32_t passwordMassErase [PASSWORD_WORD_LEN];
    /*! DSSM_BC_FACTORY_RESET_REQUEST, DSSM_DATA_EXCHANGE */
    uint32_t passwordFactoryReset [PASSWORD_WORD_LEN];
    /*! Programs static write protection of first 32K bytes.
     * One bit corresponds to one sector, LSB is Sector 0. Setting a bit
     * to 0 disables write, setting a bit to 1 enables write Possible values:
     *    - 0x0 to 0xFFFFFFFF */
    uint32_t staticWriteProtectionMainLow;
    /*! Programs static write protection of first 32K bytes.
     * One bit corresponds to eight sectors. Setting a bit
     * to 0 disables write, setting a bit to 1 enables write Possible values:
     *    - 0x0 to 0xFFFFFFF0 */
    uint32_t staticWriteProtectionMainHigh;
    /*! Non Main Flash Static Write Protection.
    * One of @ref BCR_CFG_NON_MAIN_STATIC_PROT */
    BCR_CFG_NON_MAIN_STATIC_PROT staticWriteProtectionNonMain;
    /*! CRC check part of the application. One of @ref BCR_CFG_SECURE_BOOT */
    BCR_CFG_SECURE_BOOT secureBootMode;
    /*! The Starting address from where CRC has to be calculated for the application */
    uint32_t userSecureAppStartAddr;
    /*! The Length of the application for which CRC is calculated */
    uint32_t userSecureAppLength;
    /*! CRC-32 of application range defined by above two parameters */
    uint32_t userSecureAppCrc;
    /*! CRC-32 of this structure*/
    uint32_t userCfgCRC;
} BCR_Config;

/* Structure to store the Bootloader UART and I2C Pin details. */
typedef struct
{
    uint8_t UART_RXD_pad_num;
    uint8_t UART_RXD_PF_mux_sel;
    uint8_t UART_TXD_pad_num;
    uint8_t UART_TXD_PF_mux_sel;
    uint8_t I2C_SDA_pad_num;
    uint8_t I2C_SDA_PF_mux_sel;
    uint8_t I2C_SCL_pad_num;
    uint8_t I2C_SCL_PF_mux_sel;
} BSLInterfacePins;

/* Structure to store the Bootloader invocation Pin details.
 * This structure is followed in TRIM memory where default BSL invocation
 * pin details are stored and also in Bootloader configuration memory.
 *
 * Pin_data_0
 * BIT [0-5]  :pincmIndex   -> IOMUX_SECCFG_PINCM index for the pin (0 to 63)
 * BIT [7]    :pinLevel     -> Pin state High(1)/ Low(0) which should be
 *                             considered as BSL invocation condition
 * Pin_data_1
 * BIT [0-4]  :gpioNum      -> GPIO Pin number (0 to 31)
 * BIT [5]    :gpioBase     -> GPIO Instance number or GPIO Base Index (0 or 1)
 */
typedef struct
{
    uint8_t pinData0;
    uint8_t pinData1;
} BSLInvokePin;

/**
 * @brief Possible values used for BSL configuration struct
 */

/*! Mask for BSLInvokePin.pinData0 to get IOMUX_PINCM */
#define BSL_CFG_INVOKE_PIN_PINCM_INDEX_MASK                             (0x3FU)
#define BSL_CFG_INVOKE_PIN_PINCM_INDEX_POS                                 (0U)

/*! Mask for BSLInvokePin.pinData0 to get the pin state, which determines the
 * Bootloader invocation condition) */
#define BSL_CFG_INVOKE_PIN_LEVEL_MASK                                   (0x80U)
#define BSL_CFG_INVOKE_PIN_LEVEL_POS                                       (7U)

/*! Mask for BSLInvokePin.pinData1 to get GPIO pin number */
#define BSL_CFG_INVOKE_PIN_GPIO_PIN_NUM_MASK                            (0x1FU)
#define BSL_CFG_INVOKE_PIN_GPIO_PIN_NUM_POS                                (0U)

/*! Mask for BSLInvokePin.pinData1 to get GPIO Base Index */
#define BSL_CFG_INVOKE_PIN_GPIO_PORT_NUM_MASK                           (0x20U)
#define BSL_CFG_INVOKE_PIN_GPIO_PORT_NUM_POS                               (5U)

/*! @enum BSL_CFG_MEMORY_READOUT */
typedef enum {
    /*! Memory readout is enabled */
    BSL_CFG_MEMORY_READOUT_ENABLE = 0xAABBU,
    /*! Memory readout is disabled */
    BSL_CFG_MEMORY_READOUT_DISABLE = 0xFFFFU,
} BSL_CFG_MEMORY_READOUT;

/*! @enum BSL_CFG_PLUGIN_TYPE */
typedef enum {
    /*! Plugin type in Flash is UART */
    BSL_CFG_PLUGIN_TYPE_UART = 0x1000U,
    /*! Plugin type in Flash is I2C */
    BSL_CFG_PLUGIN_TYPE_I2C = 0x2000U,
    /*! Any other interface with valid hooks will be added to the plugin
     * list */
    BSL_CFG_PLUGIN_TYPE_ANY = 0xFFFFU,
} BSL_CFG_PLUGIN_TYPE;

/*! @enum BSL_CFG_PLUGIN_FLASH */
typedef enum {
    /*! Flash plugin will be used */
    BSL_CFG_PLUGIN_FLASH_EXIST = 0xBBU,
    /*! Only ROM plugins will be used */
    BSL_CFG_PLUGIN_FLASH_NOT_EXIST = 0xFFU,
} BSL_CFG_PLUGIN_FLASH;

/*! @enum BSL_CFG_FLASH_BSL */
typedef enum {
    /*! Secondary BSL does exist */
    BSL_CFG_FLASH_BSL_EXIST = 0xAABBU,
    /*! Secondary BSL does not exist */
    BSL_CFG_FLASH_BSL_NOT_EXIST = 0xFFFFU,
} BSL_CFG_FLASH_BSL;

/*! @enum BSL_CFG_SECURITY */
typedef enum {
    /*! Factory reset performed on security alert */
    BSL_CFG_SECURITY_FACTORY_RESET = 0xAABBU,
    /*! BSL disabled on security alert */
    BSL_CFG_SECURITY_DISABLE_BSL = 0xCCDDU,
    /*! Ignore security alert */
    BSL_CFG_SECURITY_IGNORE = 0xFFFFU,
} BSL_CFG_SECURITY;

/*
 * Flash plugin prototypes should be as expected. Refer to the BSL User's Guide
 * for more detail.
 */
PLACE_IN_MEMORY(".flashPluginInit")
extern uint16_t BSL_PI_MCAN_init(uint8_t* buffer, uint16_t bufferSize);

PLACE_IN_MEMORY(".flashPluginReceive")
extern uint32_t BSL_PI_MCAN_receive(void);

PLACE_IN_MEMORY(".flashPluginSend")
extern uint8_t BSL_PI_MCAN_send(uint8_t* data, uint16_t len);

PLACE_IN_MEMORY(".flashPluginDeinit")
extern bool BSL_PI_MCAN_deinit(void);

/**
 * @brief Bootstrap loader (BSL) configuration structure values.
 * See BSL_CFG defines for possible values.
 */
typedef struct
{
    /*! Predetermined config signature magic ID */
    uint32_t configID;
    /*! BSL interface pins. Static Default BSL invocation Pin details are
     * as per TRIM open AREA default. */
    BSLInterfacePins interfacePins;
    /*! BSL pin invocation. Static Default BSL invocation Pin details are
     * as per TRIM open Area default. */
    BSLInvokePin pin;
    /*! Enable/disable memory readout. One of @ref BSL_CFG_MEMORY_READOUT */
    BSL_CFG_MEMORY_READOUT memoryRead;
    /*! Bootloader 64-byte password */
    uint32_t password[8];
    /*! Plugin type. One of @ref BSL_CFG_PLUGIN_TYPE */
    BSL_CFG_PLUGIN_TYPE pluginType;
    /*! Enable use of Flash plugin or ROM plugin.
     * One of @ref BSL_CFG_PLUGIN_FLASH */
    BSL_CFG_PLUGIN_FLASH flashPluginEnable;
    /*! SRAM memory consumed by the Flash Plugin. Range from 0x00 to 0xFF*/
    uint8_t pluginSramSize;
    /*! Function pointers for Flash Plugins.
     *   - Byte [3-0] : Init
     *   - Byte [7-4] : Receive
     *   - Byte [11-8] : Send
     *   - Byte [15-12] : Deinit */
    uint32_t pluginHook[4];
    /*! ID field to enable invocation of an alternate BSL.
     * One of @ref BSL_CFG_FLASH_BSL */
    BSL_CFG_FLASH_BSL  BSLAlternateConfig;
    /*! Reserved */
    uint16_t reserved;
    /*! Address of the alternate BSL */
    uint32_t  BSLAlternateAddr;
    /*! Pointer to the version information in MAIN flash. Possible values:
     *    - Valid Flash Address
     *    - All other values: Address will not be accessed */
    uint32_t* appRev;
    /*! Security alert configuration. One of @ref BSL_CFG_SECURITY */
    BSL_CFG_SECURITY securityAlert;
    /* I2C slave address to be used for BSL I2C communication. Any 7-bit value */
    uint16_t i2cSlaveAddress;
    /*! CRC of this BSL_Config structure */
    uint32_t userCfgCRC;
} BSL_Config;

/* Added for secondary_bsl to build */
extern const BSL_Config BSLConfig;
extern const BCR_Config BCRConfig;

/* clang-format on */

#endif /* boot_config_h */
