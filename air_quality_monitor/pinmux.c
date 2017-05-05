//*****************************************************************************
// pin_mux_config.c
//
// configure the device pins for different signals
//
// Copyright (c) 2016, Texas Instruments Incorporated - http://www.ti.com/
// All rights reserved.
//
//*****************************************************************************

// This file was automatically generated on 3/7/2017 at 5:00:59 AM
// by TI PinMux version 4.0.1482
//
//*****************************************************************************

#include "pinmux.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "gpio.h"
#include "prcm.h"
//*****************************************************************************
void PinMuxConfig(void)
{


    //
    // Set unused pins to PIN_MODE_0 with the exception of JTAG pins 16,17,19,20
    //
    PinModeSet(PIN_05, PIN_MODE_0);
    PinModeSet(PIN_06, PIN_MODE_0);
    PinModeSet(PIN_07, PIN_MODE_0);
    PinModeSet(PIN_08, PIN_MODE_0);
    PinModeSet(PIN_15, PIN_MODE_0);
    PinModeSet(PIN_21, PIN_MODE_0);
    PinModeSet(PIN_45, PIN_MODE_0);
    PinModeSet(PIN_52, PIN_MODE_0);
    PinModeSet(PIN_53, PIN_MODE_0);
    PinModeSet(PIN_55, PIN_MODE_0);
    PinModeSet(PIN_57, PIN_MODE_0);
    PinModeSet(PIN_59, PIN_MODE_0);
    PinModeSet(PIN_60, PIN_MODE_0);
    PinModeSet(PIN_62, PIN_MODE_0);
    PinModeSet(PIN_63, PIN_MODE_0);

    //
    // Enable Peripheral Clocks
    //
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);

    PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);

    //

    // Configure PIN_61 for UART0 UART0_rts
        //
        PinTypeUART(PIN_61, PIN_MODE_5);

    // Configure PIN_03 for UART0 UART0_TX
    //
    PinTypeUART(PIN_03, PIN_MODE_7);
    //



    // Configure PIN_04 for UART0 UART0_RX
    //
    PinTypeUART(PIN_04, PIN_MODE_7);
    //
    // Configure PIN_01 for I2C0 I2C_SCL
    //
    PinTypeI2C(PIN_01, PIN_MODE_1);
    //
    // Configure PIN_02 for I2C0 I2C_SDA
    //
    PinTypeI2C(PIN_02, PIN_MODE_1);

    //
    // Configure PIN_58 for GPIOInput
    //
   PinTypeGPIO(PIN_58, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA0_BASE, 0x8, GPIO_DIR_MODE_IN);

    //
    // Configure PIN_64 for GPIOOutput
    //
    PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);

    // Configure PIN_18 for GPIO Output
     //
     PinTypeGPIO(PIN_18, PIN_MODE_0, false);
     GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_OUT);

}
