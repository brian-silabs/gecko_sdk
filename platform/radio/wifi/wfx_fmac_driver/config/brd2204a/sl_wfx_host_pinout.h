/***************************************************************************//**
 * @file
 * @brief WFX host configuration and pinout
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef SL_WFX_HOST_PINOUT_H
#define SL_WFX_HOST_PINOUT_H

#ifdef SL_CATALOG_WFX_BUS_SDIO_PRESENT
#ifdef SL_CATALOG_POWER_MANAGER_PRESENT
#error "SDIO and Power Manager is an invalid combination for the GG11 STK"
#endif
#endif

#ifdef SL_CATALOG_WFX_BUS_SPI_PRESENT
#include "sl_wfx_host_bus_pinout.h"
#endif

// <<< sl:start pin_tool >>>
// <gpio> SL_WFX_HOST_PINOUT_RESET
// $[GPIO_SL_WFX_HOST_PINOUT_RESET]
#define SL_WFX_HOST_PINOUT_RESET_PORT             gpioPortC
#define SL_WFX_HOST_PINOUT_RESET_PIN              4
// [GPIO_SL_WFX_HOST_PINOUT_RESET]$

// <gpio> SL_WFX_HOST_PINOUT_WUP
// $[GPIO_SL_WFX_HOST_PINOUT_WUP]
#define SL_WFX_HOST_PINOUT_WUP_PORT               gpioPortA
#define SL_WFX_HOST_PINOUT_WUP_PIN                12
// [GPIO_SL_WFX_HOST_PINOUT_WUP]$

// <gpio optional=true> SL_WFX_HOST_PINOUT_GPIO_WIRQ
// $[GPIO_SL_WFX_HOST_PINOUT_GPIO_WIRQ]
//#define SL_WFX_HOST_PINOUT_GPIO_WIRQ_PORT
//#define SL_WFX_HOST_PINOUT_GPIO_WIRQ_PIN
// [GPIO_SL_WFX_HOST_PINOUT_GPIO_WIRQ]$

// <gpio optional=true> SL_WFX_HOST_PINOUT_LP_CLK
// $[GPIO_SL_WFX_HOST_PINOUT_LP_CLK]
//#define SL_WFX_HOST_PINOUT_LP_CLK_PORT
//#define SL_WFX_HOST_PINOUT_LP_CLK_PIN
// [GPIO_SL_WFX_HOST_PINOUT_LP_CLK]$
// <<< sl:end pin_tool >>>

#endif
