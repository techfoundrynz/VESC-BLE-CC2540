/**
 * @file    soft_uart.c
 * @brief   Software UART for CC2540 USB Dongle (P1.6=TX, P1.7=RX)
 * 
 * Bit-banged UART for pins not supported by hardware UART.
 * 
 * @note    115200 baud is difficult with software UART.
 *          Recommend 57600 or 38400 for reliability.
 */

#include "hal_mcu.h"
#include "hal_types.h"
#include "OSAL.h"
#include "soft_uart.h"

/*********************************************************************
 * CONSTANTS
 */

// Pin definitions
#define SOFT_UART_TX_PIN    P1_6
#define SOFT_UART_RX_PIN    P1_7

#if (SOFT_UART_BAUD == 115200)
  #define BIT_DELAY()   { asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); \
                          asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); \
                          asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); }
  #define HALF_BIT_DELAY() { asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); \
                             asm("NOP"); asm("NOP"); }
#elif (SOFT_UART_BAUD == 57600)
  #define BIT_DELAY()   { uint8 i; for(i=0; i<18; i++) { asm("NOP"); } }
  #define HALF_BIT_DELAY() { uint8 i; for(i=0; i<9; i++) { asm("NOP"); } }
#elif (SOFT_UART_BAUD == 38400)
  #define BIT_DELAY()   { uint8 i; for(i=0; i<27; i++) { asm("NOP"); } }
  #define HALF_BIT_DELAY() { uint8 i; for(i=0; i<13; i++) { asm("NOP"); } }
#else // Default 9600
  #define BIT_DELAY()   { uint8 i; for(i=0; i<110; i++) { asm("NOP"); } }
  #define HALF_BIT_DELAY() { uint8 i; for(i=0; i<55; i++) { asm("NOP"); } }
#endif

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 rxBuffer[SOFT_UART_RX_BUF_SIZE];
static uint8 rxHead = 0;
static uint8 rxTail = 0;

static softUartCB_t appCallback = NULL;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @fn      SoftUART_Init
 * @brief   Initialize software UART pins
 */
void SoftUART_Init(softUartCB_t callback)
{
    // Configure TX pin (P1.6) as output, high (idle)
    P1SEL &= ~BV(6);    // GPIO mode
    P1DIR |= BV(6);     // Output
    SOFT_UART_TX_PIN = 1; // Idle high
    
    // Configure RX pin (P1.7) as input
    P1SEL &= ~BV(7);    // GPIO mode
    P1DIR &= ~BV(7);    // Input
    P1INP &= ~BV(7);    // Pull-up/down enabled
    P2INP &= ~BV(6);    // Pull-up on port 1
    
    // Store callback
    appCallback = callback;
    
    // Clear buffer
    rxHead = 0;
    rxTail = 0;
}

/**
 * @fn      SoftUART_TxByte
 * @brief   Transmit one byte
 */
void SoftUART_TxByte(uint8 byte)
{
    uint8 i;
    halIntState_t intState;
    
    // Disable interrupts for timing
    HAL_ENTER_CRITICAL_SECTION(intState);
    
    // Start bit (low)
    SOFT_UART_TX_PIN = 0;
    BIT_DELAY();
    
    // Data bits (LSB first)
    for (i = 0; i < 8; i++)
    {
        if (byte & 0x01)
            SOFT_UART_TX_PIN = 1;
        else
            SOFT_UART_TX_PIN = 0;
        byte >>= 1;
        BIT_DELAY();
    }
    
    // Stop bit (high)
    SOFT_UART_TX_PIN = 1;
    BIT_DELAY();
    
    HAL_EXIT_CRITICAL_SECTION(intState);
}

/**
 * @fn      SoftUART_Write
 * @brief   Write multiple bytes
 */
void SoftUART_Write(uint8 *buf, uint8 len)
{
    uint8 i;
    for (i = 0; i < len; i++)
    {
        SoftUART_TxByte(buf[i]);
    }
}

/**
 * @fn      SoftUART_RxByte
 * @brief   Receive one byte (blocking, with timeout)
 * @return  Received byte or 0 if timeout
 */
uint8 SoftUART_RxByte(void)
{
    uint8 byte = 0;
    uint8 i;
    uint16 timeout;
    halIntState_t intState;
    
    // Wait for start bit (low)
    timeout = 65000;
    while (SOFT_UART_RX_PIN == 1 && timeout > 0)
    {
        timeout--;
    }
    
    if (timeout == 0)
        return 0;
    
    HAL_ENTER_CRITICAL_SECTION(intState);
    
    // Wait half bit to sample in middle
    HALF_BIT_DELAY();
    
    // Sample 8 data bits
    for (i = 0; i < 8; i++)
    {
        BIT_DELAY();
        byte >>= 1;
        if (SOFT_UART_RX_PIN)
            byte |= 0x80;
    }
    
    // Wait for stop bit
    BIT_DELAY();
    
    HAL_EXIT_CRITICAL_SECTION(intState);
    
    return byte;
}

/**
 * @fn      SoftUART_Poll
 * @brief   Poll for incoming data (call from main loop)
 */
void SoftUART_Poll(void)
{
    // Check for start bit
    if (SOFT_UART_RX_PIN == 0)
    {
        uint8 byte = SoftUART_RxByte();
        
        // Store in buffer
        uint8 nextHead = (rxHead + 1) % SOFT_UART_RX_BUF_SIZE;
        if (nextHead != rxTail)
        {
            rxBuffer[rxHead] = byte;
            rxHead = nextHead;
        }
        
        // Notify app
        if (appCallback)
        {
            appCallback();
        }
    }
}

/**
 * @fn      SoftUART_RxBufLen
 * @brief   Get number of bytes in RX buffer
 */
uint8 SoftUART_RxBufLen(void)
{
    if (rxHead >= rxTail)
        return (rxHead - rxTail);
    else
        return (SOFT_UART_RX_BUF_SIZE - rxTail + rxHead);
}

/**
 * @fn      SoftUART_Read
 * @brief   Read from RX buffer
 */
uint8 SoftUART_Read(uint8 *buf, uint8 maxLen)
{
    uint8 count = 0;
    
    while (rxTail != rxHead && count < maxLen)
    {
        buf[count++] = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % SOFT_UART_RX_BUF_SIZE;
    }
    
    return count;
}