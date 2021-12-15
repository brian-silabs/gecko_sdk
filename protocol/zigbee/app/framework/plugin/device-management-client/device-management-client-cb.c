/***************************************************************************//**
 * @file
 * @brief
 *******************************************************************************
 * # License
 * <b>Copyright 2021 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "af.h"

/** @brief Enact Change Of Tenancy
 *
 * This callback will be implemented by a meter to enact a change of tenancy at
 * the requested time of implementation.
 *
 * @param endpoint   Ver.: always
 * @param tenancy   Ver.: always
 */
WEAK(void emberAfPluginDeviceManagementClientEnactChangeOfTenancyCallback(uint8_t endpoint,
                                                                          EmberAfDeviceManagementTenancy *tenancy))
{
}

/** @brief Enact Change Of Supplier
 *
 * This callback will be implemented by a meter to enact a change of supplier
 * at the requested time of implementation.
 *
 * @param endpoint   Ver.: always
 * @param supplier   Ver.: always
 */
WEAK(void emberAfPluginDeviceManagementClientEnactChangeOfSupplierCallback(uint8_t endpoint,
                                                                           EmberAfDeviceManagementSupplier *supplier))
{
}

/** @brief Enact Change Supply
 *
 * This callback will be implemented by a meter to enact a supply change at the
 * requested time of implementation.
 *
 * @param endpoint   Ver.: always
 * @param supply   Ver.: always
 */
WEAK(EmberStatus emberAfPluginDeviceManagementClientEnactChangeSupplyCallback(uint8_t endpoint,
                                                                              EmberAfDeviceManagementSupply *supply))
{
  return EMBER_ZCL_STATUS_SUCCESS;
}

/** @brief Set Supply Status
 *
 * This callback will be implemented by a meter to appropriately set the supply
 * status.
 *
 * @param endpoint   Ver.: always
 * @param supplyStatus   Ver.: always
 */
WEAK(void emberAfPluginDeviceManagementClientSetSupplyStatusCallback(uint8_t endpoint,
                                                                     EmberAfDeviceManagementSupplyStatusFlags *supplyStatus))
{
}

/** @brief Enact Update Uncontrolled Flow Threshold
 *
 * This callback will be implemented by a meter to enact an update to the
 * uncontrolled flow threshold as specified.
 *
 * @param endpoint   Ver.: always
 * @param supplier   Ver.: always
 */
WEAK(void emberAfPluginDeviceManagementClientEnactUpdateUncontrolledFlowThresholdCallback(uint8_t endpoint,
                                                                                          EmberAfDeviceManagementUncontrolledFlowThreshold *supplier))
{
}
