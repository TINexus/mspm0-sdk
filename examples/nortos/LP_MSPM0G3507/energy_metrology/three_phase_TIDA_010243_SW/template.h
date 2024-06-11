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

/*!****************************************************************************
 *  @file       template.h
 *  @brief      TIDA-010243 application template
 *
 *
 *  @anchor TIDA-010243_template_h
 *  # Overview
 *    Defines the inputs to metrology, ADS modules as per hardware design, application
 *    requirements in float.
 *
 *  <hr>
 ******************************************************************************/
/** @addtogroup TIDA-010243
 * @{
 */

#ifndef TEMPLATE_H
#define TEMPLATE_H

#ifdef __cplusplus
extern "C" {
#endif

/*! @brief Defines Single phase measurement support */
#undef SINGLE_PHASE_SUPPORT
/*! @brief Defines Two phase measurement support */
#undef TWO_PHASE_SUPPORT
/*! @brief Defines Three phase measurement support */
#define THREE_PHASE_SUPPORT
/*! @brief Defines Split phase measurement support */
#undef SPLIT_PHASE_SUPPORT
/*! @brief Defines split phase single voltage configuration */
#undef SPLIT_PHASE_SINGLE_VOLTAGE_CONFIGURATION
/*! @brief Defines split phase two voltage configuration */
#undef SPLIT_PHASE_TWO_VOLTAGE_CONFIGURATION
/*! @brief Defines split phase star configuration */
#undef SPLIT_PHASE_STAR_CONFIGURATION
/*! @brief Defines split phase delta configuration */
#undef SPLIT_PHASE_DELTA_CONFIGURATION
/*! @brief Defines neutral monitor support */
#define NEUTRAL_SUPPORT
/*! @brief Defines Rogoswki coil sensing support */
#undef ROGOSWKI_SUPPORT
/*! @brief Defines ADS channel count    */
#define ADS_CHANNELCOUNT        (8)
/*! @brief Defines ADS sample rate  */
#define SAMPLE_RATE             (8000)

/*! @brief Defines Mains Nominal Frequency in Hz */
#define MAINS_NOMINAL_FREQUENCY                 50
/*! @brief Defines Mains Minimum Frequency in Hz */
#define MAINS_MIN_FREQUENCY                     40
/*! @brief Defines Mains Maximum Frequency in Hz */
#define MAINS_MAX_FREQUENCY                     70
/*! @brief Defines Mains Nominal Voltage in volts*/
#define MAINS_NOMINAL_VOLTAGE                  230

/*!
 * @brief Defines the voltage scale factor for phase A
 *        It depends on hardware design and ADS configuration
 */
#define PHASE_A_VOLTAGE_SCALE_FACTOR                1585.2
/*!
 * @brief Defines the current scale factor for phase A
 *        It depends on hardware design and ADS configuration
 */
#define PHASE_A_CURRENT_SCALE_FACTOR                125
/*!
 * @brief Defines the power scale factor for phase A
 *        It is product of voltage and current scale factor
 */
#define PHASE_A_POWER_SCALE_FACTOR                  (PHASE_A_VOLTAGE_SCALE_FACTOR * PHASE_A_CURRENT_SCALE_FACTOR)
/*!
 * @brief Defines estimate for the DC level of the voltage channel of phase A.
 *  Using this estimate helps reduce the initial settling time of the DC
 *  voltage filter for phase A
 */
#define PHASE_A_VOLTAGE_DC_ESTIMATE                 100
/*!
 * @brief Defines estimate for the DC level of the current channel of phase A.
 *  Using this estimate helps reduce the initial settling time of the DC
 *  current filter for phase A
 */
#define PHASE_A_CURRENT_DC_ESTIMATE                 0
/*!
 * @brief Defines the offset in AC voltage, this value is substracted from the
 * computed voltage in phase A.
 */
#define PHASE_A_VOLTAGE_AC_OFFSET                   0
/*!
 * @brief Defines the offset in AC fundamental voltage, this value is
 * substracted from the computed voltage in phase A.
 */
#define PHASE_A_VOLTAGE_FUNDAMENATAL_OFFSET         0
/*!
 * @brief Defines the offset in AC current, this value is substracted from the
 * computed current in phase A.
 */
#define PHASE_A_CURRENT_AC_OFFSET                   0
/*!
 * @brief Defines the offset in AC fundamental current, this value is
 * substracted from the computed current in phase A.
 */
#define PHASE_A_CURRENT_FUNDAMENATAL_OFFSET         0
/*!
 * @brief Defines the offset in active power, this value is substracted from
 * the computed active power in phase A.
 */
#define PHASE_A_ACTIVE_POWER_OFFSET                 0
/*!
 * @brief Defines the offset in fundamental active power, this value is
 * substracted from the computed active power in phase A.
 */
#define PHASE_A_ACTIVE_POWER_FUNDAMENATAL_OFFSET    0
/*!
 * @brief Defines the offset in reactive power, this value is substracted from
 * the computed active power in phase A.
 */
#define PHASE_A_REACTIVE_POWER_OFFSET               0
/*!
 * @brief Defines the offset in fundamental reactive power, this value is
 * substracted from the computed active power in phase A.
 */
#define PHASE_A_REACTIVE_POWER_FUNDAMENATAL_OFFSET  0

/*!
 * @brief Defines the voltage scale factor for phase B
 *        It depends on hardware design and ADS configuration
 */
#define PHASE_B_VOLTAGE_SCALE_FACTOR                1585.2
/*!
 * @brief Defines the current scale factor for phase B
 *        It depends on hardware design and ADS configuration
 */
#define PHASE_B_CURRENT_SCALE_FACTOR                125
/*!
 * @brief Defines the power scale factor for phase B
 */
#define PHASE_B_POWER_SCALE_FACTOR                  (PHASE_B_VOLTAGE_SCALE_FACTOR * PHASE_B_CURRENT_SCALE_FACTOR)
/*!
 * @brief Defines estimate for the DC level of the voltage channel of phase B.
 *  Using this estimate helps reduce the initial settling time of the DC
 *  voltage filter for phase B
 */
#define PHASE_B_VOLTAGE_DC_ESTIMATE                 100
/*!
 * @brief Defines estimate for the DC level of the current channel of phase B.
 *  Using this estimate helps reduce the initial settling time of the DC
 *  current filter for phase B
 */
#define PHASE_B_CURRENT_DC_ESTIMATE                 0
/*!
 * @brief Defines the offset in AC voltage, this value is substracted from the
 * computed voltage in phase B.
 */
#define PHASE_B_VOLTAGE_AC_OFFSET                   0
/*!
 * @brief Defines the offset in AC fundamental voltage, this value is
 * substracted from the computed voltage in phase B.
 */
#define PHASE_B_VOLTAGE_FUNDAMENATAL_OFFSET         0
/*!
 * @brief Defines the offset in AC current, this value is substracted from the
 * computed current in phase B.
 */
#define PHASE_B_CURRENT_AC_OFFSET                   0
/*!
 * @brief Defines the offset in AC fundamental current, this value is
 * substracted from the computed current in phase B.
 */
#define PHASE_B_CURRENT_FUNDAMENATAL_OFFSET         0
/*!
 * @brief Defines the offset in active power, this value is substracted from
 * the computed active power in phase B.
 */
#define PHASE_B_ACTIVE_POWER_OFFSET                 0
/*!
 * @brief Defines the offset in fundamental active power, this value is
 * substracted from the computed active power in phase B.
 */
#define PHASE_B_ACTIVE_POWER_FUNDAMENATAL_OFFSET    0
/*!
 * @brief Defines the offset in reactive power, this value is substracted from
 * the computed active power in phase B.
 */
#define PHASE_B_REACTIVE_POWER_OFFSET               0
/*!
 * @brief Defines the offset in fundamental reactive power, this value is
 * substracted from the computed active power in phase A.
 */
#define PHASE_B_REACTIVE_POWER_FUNDAMENATAL_OFFSET  0

/*!
 * @brief Defines the voltage scale factor for phase C
 *        It depends on hardware design and ADS configuration
 */
#define PHASE_C_VOLTAGE_SCALE_FACTOR                1585.2
/*!
 * @brief Defines the current scale factor for phase B
 *        It depends on hardware design and ADS configuration
 */
#define PHASE_C_CURRENT_SCALE_FACTOR                125
/*!
 * @brief Defines the power scale factor for phase
 */
#define PHASE_C_POWER_SCALE_FACTOR                  (PHASE_C_VOLTAGE_SCALE_FACTOR * PHASE_C_CURRENT_SCALE_FACTOR)
/*!
 * @brief Defines estimate for the DC level of the voltage channel of phase C.
 *  Using this estimate helps reduce the initial settling time of the DC
 *  voltage filter for phase C
 */
#define PHASE_C_VOLTAGE_DC_ESTIMATE                 100
/*!
 * @brief Defines estimate for the DC level of the current channel of phase C.
 *  Using this estimate helps reduce the initial settling time of the DC
 *  current filter for phase C
 */
#define PHASE_C_CURRENT_DC_ESTIMATE                 0
/*!
 * @brief Defines the offset in AC voltage, this value is substracted from the
 * computed voltage in phase C.
 */
#define PHASE_C_VOLTAGE_AC_OFFSET                   0
/*!
 * @brief Defines the offset in AC fundamental voltage, this value is
 * substracted from the computed voltage in phase C.
 */
#define PHASE_C_VOLTAGE_FUNDAMENATAL_OFFSET         0
/*!
 * @brief Defines the offset in AC current, this value is substracted from the
 * computed current in phase C.
 */
#define PHASE_C_CURRENT_AC_OFFSET                   0
/*!
 * @brief Defines the offset in AC fundamental current, this value is
 * substracted from the computed current in phase C.
 */
#define PHASE_C_CURRENT_FUNDAMENATAL_OFFSET         0
/*!
 * @brief Defines the offset in active power, this value is substracted from
 * the computed active power in phase C.
 */
#define PHASE_C_ACTIVE_POWER_OFFSET                 0
/*!
 * @brief Defines the offset in fundamental active power, this value is
 * substracted from the computed active power in phase C.
 */
#define PHASE_C_ACTIVE_POWER_FUNDAMENATAL_OFFSET    0
/*!
 * @brief Defines the offset in reactive power, this value is substracted from
 * the computed active power in phase C.
 */
#define PHASE_C_REACTIVE_POWER_OFFSET               0
/*!
 * @brief Defines the offset in fundamental reactive power, this value is
 * substracted from the computed active power in phase C.
 */
#define PHASE_C_REACTIVE_POWER_FUNDAMENATAL_OFFSET  0
/*!
 * @brief Defines the current scale factor for neutral
 *        It depends on hardware design and ADS configuration
 */
#define NEUTRAL_CURRENT_SCALE_FACTOR                125
/*!
 * @brief Defines estimate for the DC level of the current channel of neutral.
 *  Using this estimate helps reduce the initial settling time of the DC
 *  current filter for neutral
 */
#define NEUTRAL_CURRENT_DC_ESTIMATE                 0
/*!
 * @brief Defines the offset in AC current, this value is substracted from the
 * computed current in neutral.
 */
#define NEUTRAl_CURRENT_AC_OFFSET                   0
/*!
 * @brief Defines the offset in AC fundamental current, this value is
 * substracted from the computed current in neutral.
 */
#define NEUTRAL_CURRENT_FUNDAMENATAL_OFFSET         0
/*!
 * @brief Defines the power scale factor for neutral
 */
#define NEUTRAL_POWER_SCALE_FACTOR                  125
/*!
 * @brief Defines the offset in active power, this value is subtracted from
 * the computed active power in neutral.
 */
#define NEUTRAL_ACTIVE_POWER_OFFSET                 0
/*!
 * @brief Defines the offset in reactive power, this value is subtracted from
 * the computed active power in neutral.
 */
#define NEUTRAL_REACTIVE_POWER_OFFSET               0
/*!
 * @brief Defines the offset in fundamental active power, this value is
 * substracted from the computed active power in neutral.
 */
#define NEUTRAL_ACTIVE_POWER_FUNDAMENTAL_OFFSET     0
/*!
 * @brief Defines the offset in fundamental reactive power, this value is
 * substracted from the computed active power in neutral.
 */
#define NEUTRAL_REACTIVE_POWER_FUNDAMENTAL_OFFSET   0

#ifdef __cplusplus
}
#endif
#endif /* TEMPLATE_H */
/** @}*/
