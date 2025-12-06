/**
 * @file    vesc_uart.c
 * @brief   Hardware UART implementation for CC2540
 *          Uses USART1 Alt 2:
 *          TX = P1.6
 *          RX = P1.7
 */

#include "hal_mcu.h"
#include "hal_types.h"
#include "OSAL.h"
#include "vesc_uart.h"

/*********************************************************************
 * CONSTANTS
 */

/*
 * UART PIN DEFINITIONS
 * 
 * UART0 Alt 1: P0.2 (RX), P0.3 (TX)
 * UART0 Alt 2: P1.4 (RX), P1.5 (TX) 
 * 
 * UART1 Alt 1: P0.5 (RX), P0.4 (TX)
 * UART1 Alt 2: P1.7 (RX), P1.6 (TX)
 */

#if (VESC_UART_PORT == 0)
    // UART 0
    #define UXCSR           U0CSR
    #define UXUCR           U0UCR
    #define UXDBUF          U0DBUF
    #define UXBAUD          U0BAUD
    #define UXGCR           U0GCR
    #define URXIF           URX0IF
    #define URXIE           URX0IE
    #define URX_VECTOR      URX0_VECTOR
    #define PERCFG_UART_BIT 0x01
    
    #if (VESC_UART_ALT == 1)
        #define PIN_RX          2
        #define PIN_TX          3
        #define PIN_SEL         P0SEL
        #define PIN_DIR         P0DIR
        #define PERCFG_ALT_BIT  0x00 // Alt 1 = 0
    #else
        #define PIN_RX          4
        #define PIN_TX          5
        #define PIN_SEL         P1SEL
        #define PIN_DIR         P1DIR
        #define PERCFG_ALT_BIT  0x02 // Alt 2 = 1 (Bit 1 of PERCFG for UART0 ?? No check datasheet!)
        // CC254x Datasheet:
        // UART0 Alt 1: PERCFG.U0CFG = 0
        // UART0 Alt 2: PERCFG.U0CFG = 1 (Bit 0)
        #undef PERCFG_ALT_BIT
        #define PERCFG_ALT_BIT  0x01
    #endif

#else
    // UART 1
    #define UXCSR           U1CSR
    #define UXUCR           U1UCR
    #define UXDBUF          U1DBUF
    #define UXBAUD          U1BAUD
    #define UXGCR           U1GCR
    #define URXIF           URX1IF
    #define URXIE           URX1IE
    #define URX_VECTOR      URX1_VECTOR
    
    #if (VESC_UART_ALT == 1)
        #define PIN_RX          5
        #define PIN_TX          4
        #define PIN_SEL         P0SEL
        #define PIN_DIR         P0DIR
        #define PERCFG_ALT_BIT  0x00 // Alt 1 = 0
    #else
        #define PIN_RX          7
        #define PIN_TX          6
        #define PIN_SEL         P1SEL
        #define PIN_DIR         P1DIR
        #define PERCFG_ALT_BIT  0x02 // Alt 2 = 1 (Bit 1 of PERCFG for UART1)
    #endif

#endif

// Baud Rate Generation for 32 MHz Clock
// 115200: M=216, E=11
#define BAUD_M_115200   216
#define BAUD_E_115200   11

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 rxBuffer[VESC_UART_RX_BUF_SIZE];
static volatile uint16 rxHead = 0;
static volatile uint16 rxTail = 0;

static vescUartCB_t appCallback = NULL;

static void VescUART_TxByte(uint8 byte);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @fn      VescUART_Init
 * @brief   Initialize Hardware UART
 */
void VescUART_Init(vescUartCB_t callback)
{
    // Store callback
    appCallback = callback;
    
    // Clear buffer
    rxHead = 0;
    rxTail = 0;

    // 1. Configure Pins for Peripheral function
    PIN_SEL |= (BV(PIN_RX) | BV(PIN_TX));
    
    // 2. Configure Pin Direction (optional, handled by peripheral usually)
    
    // 3. Configure UART Priority/Alt Location
    if (PERCFG_ALT_BIT)
        PERCFG |= PERCFG_ALT_BIT;
    else
        PERCFG &= ~PERCFG_ALT_BIT;
        
    // 4. Configure USART Control
    // CSR.MODE = 1 (UART)
    // CSR.RE = 1 (Receiver Enable)
    UXCSR = 0x80 | 0x40; 
    
    // 6. Configure UART Control
    // UCR.FLUSH = 1
    // UCR.STOP = 1 (High)
    // UCR.START = 0 (Low)
    UXUCR = BV(7); // Flush
    UXUCR = BV(1); // 8N1, Stop High
    
    // 7. Baud Rate
    UXBAUD = BAUD_M_115200;
    UXGCR = (UXGCR & 0xE0) | BAUD_E_115200;
    
    // 8. Interrupts
    URXIF = 0;
    URXIE = 1;
}

/**
 * @fn      VescUART_TxByte
 * @brief   Transmit one byte using Hardware UART
 */
static void VescUART_TxByte(uint8 byte)
{
    UXDBUF = byte;
    while (!(UXCSR & 0x02)); // Wait for TX_BYTE (Bit 1)
    UXCSR &= ~0x02;          // Clear TX_BYTE
}

/**
 * @fn      VescUART_Write
 * @brief   Write multiple bytes
 */
void VescUART_Write(uint8 *buf, uint8 len)
{
    uint8 i;
    for (i = 0; i < len; i++)
    {
        VescUART_TxByte(buf[i]);
    }
}


/**
 * @fn      VescUART_RxBufLen
 * @brief   Get number of bytes in RX buffer
 */
uint16 VescUART_RxBufLen(void)
{
    uint16 head = rxHead;  // Read volatile once
    uint16 tail = rxTail;  // Read volatile once
    
    if (head >= tail)
        return (head - tail);
    else
        return (VESC_UART_RX_BUF_SIZE - tail + head);
}

/**
 * @fn      VescUART_Read
 * @brief   Read from RX buffer
 */

uint16 VescUART_Read(uint8 *buf, uint16 maxLen)
{
    uint16 count = 0;
    uint16 head = rxHead;  // Read volatile once
    
    while (rxTail != head && count < maxLen)
    {
        buf[count++] = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % VESC_UART_RX_BUF_SIZE;
    }
    
    return count;
}

/**
 * @fn      VescUART_Poll
 * @brief   Polls the buffer and triggers callback if data exists.
 *          Call this periodically (e.g. 10ms) from main loop.
 */
void VescUART_Poll(void)
{
    if (rxHead != rxTail)
    {
        if (appCallback)
        {
            appCallback();
        }
    }
}

/**
 * @brief   UART RX Interrupt Service Routine
 *          OPTIMIZED: No Callback Overhead!
 */
#pragma vector = URX_VECTOR
__interrupt void uartRxIsr(void)
{
    URXIF = 0; // Clear interrupt flag
    
    uint8 byte = UXDBUF;
    
    uint16 nextHead = (rxHead + 1) % VESC_UART_RX_BUF_SIZE;
    if (nextHead != rxTail)
    {
        rxBuffer[rxHead] = byte;
        rxHead = nextHead;
        
        // Removed appCallback() to reduce ISR latency and prevent blocking
    }
}