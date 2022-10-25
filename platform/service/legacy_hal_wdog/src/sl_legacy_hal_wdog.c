/***************************************************************************//**
 * @file sl_legacy_hal_wdog.c
 * @brief Legacy HAL Watchdog
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories, Inc, www.silabs.com</b>
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

#include PLATFORM_HEADER
#include "sl_legacy_hal_wdog.h"

#include "sl_power_manager.h"

// sl_legacy_hal_wdog.h needs to be able to define all SL_LEGACY_HAL_WDOG_*
// before these function will compile.
#if defined(SL_LEGACY_HAL_WDOG)             \
  && defined(SL_LEGACY_HAL_WDOG_IRQn)       \
  && defined(SL_LEGACY_HAL_WDOG_IRQHandler) \
  && defined(SL_LEGACY_HAL_WDOG_CMUCLOCK)

// EM Events
#define SLEEP_EM_EVENT_MASK      (SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM0   \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM0  \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM1 \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM1  \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM2 \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM2  \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_ENTERING_EM3 \
                                  | SL_POWER_MANAGER_EVENT_TRANSITION_LEAVING_EM3)

static void events_handler(sl_power_manager_em_t from,
                           sl_power_manager_em_t to);

static sl_power_manager_em_transition_event_handle_t events_handle;
static sl_power_manager_em_transition_event_info_t events_info =
{
  .event_mask = SLEEP_EM_EVENT_MASK,
  .on_event = events_handler,
};

static bool watchdogDisableInSleep;


void halInternalEnableWatchDog(void)
{
  //Ensure the Power Manager is active, as halInit can occur before that 
  sl_power_manager_init();
  sl_power_manager_subscribe_em_transition_event(&events_handle,
                                                 &events_info);
  // Enable LE interface
#if !defined(_SILICON_LABS_32B_SERIES_2)
  CMU_ClockEnable(cmuClock_HFLE, true);
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
#endif

#if defined(_SILICON_LABS_32B_SERIES_2) && !defined(_SILICON_LABS_32B_SERIES_2_CONFIG_1)
  CMU_ClockEnable(SL_LEGACY_HAL_WDOG_CMUCLOCK, true);
#endif

  // Make sure FULL reset is used on WDOG timeout
#if defined(_RMU_CTRL_WDOGRMODE_MASK)
  RMU_ResetControl(rmuResetWdog, rmuResetModeFull);
#endif

  /* Note: WDOG_INIT_DEFAULT comes from platform/emlib/inc/em_wdog.h */
  WDOG_Init_TypeDef init = WDOG_INIT_DEFAULT;

  /* Trigger watchdog reset after 2 seconds (64k / 32k) and
   * warning interrupt is triggered after 1.5 seconds (75% of timeout). */
  init.perSel = wdogPeriod_64k;
  init.warnSel = wdogWarnTime75pct;

#if defined(_WDOG_CTRL_CLKSEL_MASK)
  init.clkSel = wdogClkSelLFRCO;
#else
  // Series 2 devices select watchdog oscillator with the CMU.
  CMU_ClockSelectSet(SL_LEGACY_HAL_WDOG_CMUCLOCK, cmuSelect_LFRCO);
#endif

  WDOGn_Init(SL_LEGACY_HAL_WDOG, &init);

  /* Enable WARN interrupt. */
#if defined(WDOG_IF_WARN) && !defined(BOOTLOADER)
  NVIC_ClearPendingIRQ(SL_LEGACY_HAL_WDOG_IRQn);
  WDOGn_IntClear(SL_LEGACY_HAL_WDOG, WDOG_IF_WARN);
  NVIC_EnableIRQ(SL_LEGACY_HAL_WDOG_IRQn);
  WDOGn_IntEnable(SL_LEGACY_HAL_WDOG, WDOG_IEN_WARN);
#endif
}

void halResetWatchdog(void)
{
  WDOGn_Feed(SL_LEGACY_HAL_WDOG);
}

void halInternalDisableWatchDog(uint8_t magicKey)
{
  if ( magicKey == MICRO_DISABLE_WATCH_DOG_KEY ) {
    WDOGn_SyncWait(DEFAULT_WDOG);
    WDOGn_Enable(DEFAULT_WDOG, false);
  }
}

bool halInternalWatchDogEnabled(void)
{
  return WDOGn_IsEnabled(SL_LEGACY_HAL_WDOG);
}

/**
 * @brief On All Events callback
 *
 * @param from EM level from where we start from
 *
 * @param to   EM level where we are going
 */
static void events_handler(sl_power_manager_em_t from,
                           sl_power_manager_em_t to)
{
  if (from == SL_POWER_MANAGER_EM0) {
    switch (to) {
      case SL_POWER_MANAGER_EM1:
        watchdogDisableInSleep = halInternalWatchDogEnabled();
        if (watchdogDisableInSleep) {
          halInternalDisableWatchDog(MICRO_DISABLE_WATCH_DOG_KEY);
        }
        break;

      case SL_POWER_MANAGER_EM0:
        break;

      case SL_POWER_MANAGER_EM2:
      case SL_POWER_MANAGER_EM3:
        if (halInternalWatchDogEnabled()) {
          WDOGn_SyncWait(SL_LEGACY_HAL_WDOG);
        }
        break;

      default:
        break;
    }
  } else { //from low power modes other than EM0
    // restart watchdog if it was running when we entered sleep
    // do this before dispatching interrupts while we still have tight
    // control of code execution
    if (watchdogDisableInSleep) {
      WDOGn_Enable(SL_LEGACY_HAL_WDOG, true);
    }

  }
}

void SL_LEGACY_HAL_WDOG_IRQHandler(void)
{
  uint32_t intFlags = WDOGn_IntGet(SL_LEGACY_HAL_WDOG);
  WDOGn_IntClear(SL_LEGACY_HAL_WDOG, intFlags);

  NMI_Handler();
}

#else

void halInternalEnableWatchDog(void)
{
}

void halResetWatchdog(void)
{
}

void halInternalDisableWatchDog(uint8_t magicKey)
{
  (void) magicKey;
}

bool halInternalWatchDogEnabled(void)
{
  return false;
}

#endif
