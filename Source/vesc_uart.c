/**
 * @file    vesc_uart.c
 * @brief   Wrapper for TI HalUART driver (DMA implementation)
 *          Configured for UART1 Alt 2 via Manual Register Overrides
 */

#include "hal_mcu.h"
#include "hal_types.h"
#include "hal_uart.h"
#include "OSAL.h"
#include "vesc_uart.h"

/*********************************************************************
 * CONSTANTS
 */

// We manually configure pins, so we don't rely on these defines for logic,
// but we keep them for reference if needed.
#if !defined( VESC_UART_PORT )
  #define VESC_UART_PORT  HAL_UART_PORT_1
#endif

// Helper macros
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

/*********************************************************************
 * LOCAL VARIABLES
 */

static vescUartCB_t appCallback = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void halUartCback(uint8 port, uint8 event)
{
    // If the driver reports RX data (or any event), we notify the app.
    // Optimally, we could verify event == HAL_UART_RX_FULL or HAL_UART_RX_ABOUT_FULL or HAL_UART_RX_TIMEOUT
    if (appCallback)
    {
        appCallback();
    }
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void VescUART_Init(vescUartCB_t callback)
{
    halUARTCfg_t uartConfig;
    
    appCallback = callback;
    
    // Ensure HAL UART is initialized
    HalUARTInit();
    
    // Determine HAL Port ID based on define
    uint8 halPort = (VESC_UART_PORT == 0) ? HAL_UART_PORT_0 : HAL_UART_PORT_1;
    
    // Configure UART parameters
    uartConfig.configured           = TRUE;
    uartConfig.baudRate             = HAL_UART_BR_115200;
    uartConfig.flowControl          = FALSE;
    uartConfig.flowControlThreshold = 48; 
    uartConfig.rx.maxBufSize        = 256; 
    uartConfig.tx.maxBufSize        = 128; 
    uartConfig.idleTimeout          = 6;   
    uartConfig.intEnable            = TRUE;
    uartConfig.callBackFunc         = halUartCback;
    
    // Open the port
    HalUARTOpen(halPort, &uartConfig);
    
    // ----------------------------------------------------------------
    // NUCLEAR PIN CONFIGURATION (Dynamic)
    // ----------------------------------------------------------------
    
    if (VESC_UART_PORT == 0)
    {
        // UART0
        // Ensure UART0 has priority on Port 0 (P2DIR.PRIP0 = 00) -> Default is usually OK (00)
        // If needed: P2DIR &= ~0xC0;
        
        if (VESC_UART_ALT == 1)
        {
            // UART0 Alt 1: P0.2 (RX), P0.3 (TX) 
            PERCFG &= ~0x01; // U0CFG = 0
            P0SEL |= 0x0C;   // Peripheral on P0.2/P0.3
            P0DIR &= ~0x04;  // P0.2 In
            P0DIR |= 0x08;   // P0.3 Out
        }
        else
        {
            // UART0 Alt 2: P1.4 (RX), P1.5 (TX)
            PERCFG |= 0x01;  // U0CFG = 1
            P1SEL |= 0x30;   // Peripheral on P1.4/P1.5
            P1DIR &= ~0x10;  // P1.4 In
            P1DIR |= 0x20;   // P1.5 Out
        }
    }
    else
    {
        // UART1
        // Ensure UART1 has priority on Port 1 (P2DIR[4:3] = 01)
        P2DIR &= ~0x18; 
        P2DIR |= 0x08;
        
        if (VESC_UART_ALT == 1)
        {
            // UART1 Alt 1: P0.5 (RX), P0.4 (TX)
            PERCFG &= ~0x02; // U1CFG = 0
            P0SEL |= 0x30;   // Peripheral on P0.4/P0.5
            P0DIR |= 0x10;   // P0.4 Out
            P0DIR &= ~0x20;  // P0.5 In
        }
        else
        {
            // UART1 Alt 2: P1.7 (RX), P1.6 (TX)
            PERCFG |= 0x02;  // U1CFG = 1
            P1SEL |= 0xC0;   // Peripheral on P1.6/P1.7
            P1DIR |= 0x40;   // P1.6 Out
            P1DIR &= ~0x80;  // P1.7 In
        }
    }
}

void VescUART_Write(uint8 *buf, uint8 len)
{
    HalUARTWrite(VESC_UART_PORT, buf, len);
}

uint16 VescUART_RxBufLen(void)
{
    // HalUARTRead(..., 0) doesn't return len. 
    // Hal_UART_RxBufLen() returns the count.
    return Hal_UART_RxBufLen(VESC_UART_PORT);
}

uint16 VescUART_Read(uint8 *buf, uint16 maxLen)
{
    return HalUARTRead(VESC_UART_PORT, buf, maxLen);
}

void VescUART_Poll(void)
{
    HalUARTPoll();
}