// Standard includes
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"

// Common interface includes
#include "uart_if.h"
#include "i2c_if.h"

#include "pin_mux_config.h"


//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define APPLICATION_VERSION     "1.4.0"
#define APP_NAME                "Sliding Ball"
#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define SPI_IF_BIT_RATE         100000
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}

// Color definitions
#define BLACK           0x0000
#define WHITE           0xFFFF

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//****************************************************************************

//*****************************************************************************
//
//! Display the buffer contents over I2C
//!
//! \param  pucDataBuf is the pointer to the data store to be displayed
//! \param  ucLen is the length of the data to be displayed
//!
//! \return none
//!
//*****************************************************************************
//
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t      CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
}

//****************************************************************************
//
//! Reads registers 0x2 - 0x5 corresponding to X/Y acceleration data
//!
//! This function
//!    1. Invokes the corresponding I2C APIs
//!
//! \return array containing X and Y byte data
//
//****************************************************************************
int8_t*
ReadAccData()
{
    unsigned char ucRegOffset = 2;
    unsigned char aucRdDataBuf[256];

    //
    // Write the register address to be read from.
    // Stop bit implicitly assumed to be 0.
    //
    RET_IF_ERR(I2C_IF_Write(24,&ucRegOffset,1,0));

    //
    // Read 4 bytes of successive data
    //
    RET_IF_ERR(I2C_IF_Read(24, &aucRdDataBuf[0], 4));

    //
    // Return the X and Y acceleration data
    //
    static int8_t data[2];
    data[0] = (int8_t)aucRdDataBuf[3];
    data[1] = (int8_t)aucRdDataBuf[1];

    return data;
}

//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function handling the I2C example
//!
//! \param  None
//!
//! \return None
//!
//*****************************************************************************
void main(){
    int iRetVal;
    char acCmdStore[512];

    //
    // Initialize board configurations
    //
    BoardInit();

    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Configuring UART
    //
    InitTerm();

    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //
    // Display the banner followed by the usage description
    //
    DisplayBanner(APP_NAME);

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    // Initialize ball
    Adafruit_Init();
    fillScreen(BLACK);
    fillCircle(64, 64, 4, WHITE);

    int ballPosition[2] = { 64, 64 };
    int8_t ballVelocity[2] = { 0, 0 };
    int BALL_RADIUS = 4;
    int SCREEN = 128;

    while (FOREVER)
    {
        // Erase ball from the old position
        fillCircle(ballPosition[0], ballPosition[1], 4, BLACK);

        // Get acceleration data and scale with max acceleration
        int8_t* accData = ReadAccData();
        int8_t xAcc = (int8_t)(((double)accData[0] / 64) * 6);
        int8_t yAcc = (int8_t)(((double)accData[1] / 64) * 6);
        Report("X Acc: %d, Y Acc: %d\n\r", accData[0], accData[1]);

        // Update velocity based on acceleration and apply friction
        ballVelocity[0] = (ballVelocity[0] + xAcc) * 0.99;
        ballVelocity[1] = (ballVelocity[1] + yAcc) * 0.99;

        // Update position based on velocity
        ballPosition[0] += ballVelocity[0];
        ballPosition[1] += ballVelocity[1];

        // Keep ball position within screen bounds
        if (ballPosition[0] <= BALL_RADIUS) {
            ballPosition[0] = BALL_RADIUS;
            ballVelocity[0] *= -0.95;
        } else if (ballPosition[0] > SCREEN - BALL_RADIUS - 1) {
            ballPosition[0] = SCREEN - BALL_RADIUS - 1;
            ballVelocity[0] *= -0.95;
        }

        if (ballPosition[1] <= BALL_RADIUS) {
            ballPosition[1] = BALL_RADIUS;
            ballVelocity[1] *= -0.95;
        } else if (ballPosition[1] > SCREEN - BALL_RADIUS - 1) {
            ballPosition[1] = SCREEN - BALL_RADIUS - 1;
            ballVelocity[1] *= -0.95;
        }

        // Draw ball at the new position
        fillCircle(ballPosition[0], ballPosition[1], BALL_RADIUS, WHITE);
    }

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @
//
//*****************************************************************************
