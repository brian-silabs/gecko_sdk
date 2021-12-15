#ifndef HAL_CONFIG_BOARD_H
#define HAL_CONFIG_BOARD_H

#include "em_device.h"
#include "hal-config-types.h"

// This file is auto-generated by Hardware Configurator in Simplicity Studio.
// Any content between $[ and ]$ will be replaced whenever the file is regenerated.
// Content outside these regions will be preserved.

// $[ANTDIV]
// [ANTDIV]$

// $[BTL_BUTTON]

#define BSP_BTL_BUTTON_PIN                   (5U)
#define BSP_BTL_BUTTON_PORT                  (gpioPortA)

// [BTL_BUTTON]$

// $[BUTTON]
#define BSP_BUTTON_PRESENT                   (1)

#define BSP_BUTTON0_PIN                      (5U)
#define BSP_BUTTON0_PORT                     (gpioPortA)

#define BSP_BUTTON1_PIN                      (4U)
#define BSP_BUTTON1_PORT                     (gpioPortB)

#define BSP_BUTTON_COUNT                     (2U)
#define BSP_BUTTON_INIT                      { { BSP_BUTTON0_PORT, BSP_BUTTON0_PIN }, { BSP_BUTTON1_PORT, BSP_BUTTON1_PIN } }
#define BSP_BUTTON_GPIO_DOUT                 (HAL_GPIO_DOUT_LOW)
#define BSP_BUTTON_GPIO_MODE                 (HAL_GPIO_MODE_INPUT)
// [BUTTON]$

// $[CMU]
#define BSP_CLK_HFXO_PRESENT                 (1)
#define BSP_CLK_HFXO_FREQ                    (38400000UL)
#define BSP_CLK_HFXO_INIT                     CMU_HFXOINIT_DEFAULT
#define BSP_CLK_HFXO_CTUNE                   (140)
#define BSP_CLK_LFXO_PRESENT                 (1)
#define BSP_CLK_LFXO_INIT                     CMU_LFXOINIT_DEFAULT
#define BSP_CLK_LFXO_FREQ                    (32768U)
#define BSP_CLK_LFXO_CTUNE                   (63U)
// [CMU]$

// $[COEX]
// [COEX]$

// $[DCDC]
#define BSP_DCDC_PRESENT                     (1)

#define BSP_DCDC_INIT                         EMU_DCDCINIT_DEFAULT
// [DCDC]$

// $[EMU]
// [EMU]$

// $[EUART0]
#define PORTIO_EUART0_RX_PIN                 (6U)
#define PORTIO_EUART0_RX_PORT                (gpioPortB)

#define PORTIO_EUART0_TX_PIN                 (5U)
#define PORTIO_EUART0_TX_PORT                (gpioPortB)

#define BSP_EUART0_TX_PIN                    (5U)
#define BSP_EUART0_TX_PORT                   (gpioPortB)

#define BSP_EUART0_RX_PIN                    (6U)
#define BSP_EUART0_RX_PORT                   (gpioPortB)

// [EUART0]$

// $[EXTFLASH]
// [EXTFLASH]$

// $[EZRADIOPRO]
// [EZRADIOPRO]$

// $[FEM]
// [FEM]$

// $[GPIO]
#define PORTIO_GPIO_SWV_PIN                  (4U)
#define PORTIO_GPIO_SWV_PORT                 (gpioPortA)

#define BSP_TRACE_SWO_PIN                    (4U)
#define BSP_TRACE_SWO_PORT                   (gpioPortA)

// [GPIO]$

// $[I2C0]
#define PORTIO_I2C0_SCL_PIN                  (8U)
#define PORTIO_I2C0_SCL_PORT                 (gpioPortA)

#define PORTIO_I2C0_SDA_PIN                  (7U)
#define PORTIO_I2C0_SDA_PORT                 (gpioPortA)

#define BSP_I2C0_SCL_PIN                     (8U)
#define BSP_I2C0_SCL_PORT                    (gpioPortA)

#define BSP_I2C0_SDA_PIN                     (7U)
#define BSP_I2C0_SDA_PORT                    (gpioPortA)

// [I2C0]$

// $[I2C1]
// [I2C1]$

// $[I2CSENSOR]
// [I2CSENSOR]$

// $[IADC0]
// [IADC0]$

// $[IOEXP]
// [IOEXP]$

// $[LED]
#define BSP_LED_PRESENT                      (1)

#define BSP_LED0_PIN                         (8U)
#define BSP_LED0_PORT                        (gpioPortC)

#define BSP_LED1_PIN                         (9U)
#define BSP_LED1_PORT                        (gpioPortC)

#define BSP_LED_COUNT                        (2U)
#define BSP_LED_INIT                         { { BSP_LED0_PORT, BSP_LED0_PIN }, { BSP_LED1_PORT, BSP_LED1_PIN } }
#define BSP_LED_POLARITY                     (1)
// [LED]$

// $[LETIMER0]
// [LETIMER0]$

// $[LFXO]
// [LFXO]$

// $[MODEM]
// [MODEM]$

// $[PA]

#define BSP_PA_VOLTAGE                       (1800U)
// [PA]$

// $[PDM]
// [PDM]$

// $[PORTIO]
// [PORTIO]$

// $[PRS]
// [PRS]$

// $[PTI]
// [PTI]$

// $[SERIAL]
#define BSP_SERIAL_APP_PORT                  (HAL_SERIAL_PORT_USART0)
#define BSP_SERIAL_APP_TX_PIN                (5U)
#define BSP_SERIAL_APP_TX_PORT               (gpioPortB)

#define BSP_SERIAL_APP_RX_PIN                (6U)
#define BSP_SERIAL_APP_RX_PORT               (gpioPortB)
// [SERIAL]$

// $[SPIDISPLAY]
// [SPIDISPLAY]$

// $[SPINCP]
// [SPINCP]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[TIMER2]
// [TIMER2]$

// $[TIMER3]
// [TIMER3]$

// $[TIMER4]
// [TIMER4]$

// $[UARTNCP]
// [UARTNCP]$

// $[USART0]
#define PORTIO_USART0_RX_PIN                 (5U)
#define PORTIO_USART0_RX_PORT                (gpioPortB)

#define PORTIO_USART0_TX_PIN                 (6U)
#define PORTIO_USART0_TX_PORT                (gpioPortB)
// [USART0]$

// $[USART1]
// [USART1]$

// $[VCOM]
// [VCOM]$

// $[VUART]
// [VUART]$

// $[WDOG]
// [WDOG]$

// $[Custom pin names]
// [Custom pin names]$

#if defined(_SILICON_LABS_MODULE)
#include "sl_module.h"
#endif

#endif /* HAL_CONFIG_BOARD_H */
