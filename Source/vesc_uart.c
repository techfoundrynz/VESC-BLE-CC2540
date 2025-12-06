#include "hal_mcu.h"
#include "hal_types.h"
#include "OSAL.h"
#include "hal_uart.h"
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

// Define which UART port to use based on VESC_UART_PORT
#if (VESC_UART_PORT == 0)
  #define HAL_UART_PORT_USED HAL_UART_PORT_0
#else
  #define HAL_UART_PORT_USED HAL_UART_PORT_1
#endif

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 rxBuffer[VESC_UART_RX_BUF_SIZE];
static volatile uint16 rxHead = 0;
static volatile uint16 rxTail = 0;

static vescUartCB_t appCallback = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void uartCallback(uint8 port, uint8 event);

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @fn      VescUART_Init
 * @brief   Initialize UART using TI HAL Driver
 */
void VescUART_Init(vescUartCB_t callback)
{
    halUARTCfg_t uartConfig;

    // Store callback
    appCallback = callback;
    
    // Clear buffer
    rxHead = 0;
    rxTail = 0;

    // Configure HAL UART
    uartConfig.configured           = TRUE;
    uartConfig.baudRate             = HAL_UART_BR_115200;
    uartConfig.flowControl          = FALSE;
    uartConfig.flowControlThreshold = 48; // Recommended: 48
    uartConfig.rx.maxBufSize        = 128; // Driver internal buffer
    uartConfig.tx.maxBufSize        = 128; // Driver internal buffer
    uartConfig.idleTimeout          = 6;   // ~6ms timeout
    uartConfig.intEnable            = TRUE;
    uartConfig.callBackFunc         = uartCallback;
    
    HalUARTOpen(HAL_UART_PORT_USED, &uartConfig);
}

/**
 * @fn      VescUART_TxByte
 * @brief   Transmit one byte (Helper)
 */
static void VescUART_TxByte(uint8 byte)
{
    HalUARTWrite(HAL_UART_PORT_USED, &byte, 1);
}

/**
 * @fn      VescUART_Write
 * @brief   Write multiple bytes
 */
void VescUART_Write(uint8 *buf, uint8 len)
{
    HalUARTWrite(HAL_UART_PORT_USED, buf, len);
}

/**
 * @fn      VescUART_RxBufLen
 * @brief   Get number of bytes in RX buffer
 */
uint16 VescUART_RxBufLen(void)
{
    uint16 head = rxHead;  // Volatile read
    uint16 tail = rxTail;  // Volatile read
    
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
    uint16 head = rxHead; // Volatile read
    
    while (rxTail != head && count < maxLen)
    {
        buf[count++] = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % VESC_UART_RX_BUF_SIZE;
    }
    
    return count;
}

/**
 * @fn      VescUART_Poll
 * @brief   Polls - In HAL mode, this just triggers callback if data available
 */
void VescUART_Poll(void)
{
    // Check if we need to manually pull from HAL driver (usually callback handles it)
    // But we can trigger the app callback if we have data.
    if (rxHead != rxTail)
    {
        if (appCallback)
        {
            appCallback();
        }
    }
}

/**
 * @fn      uartCallback
 * @brief   TI HAL UART Callback
 */
static void uartCallback(uint8 port, uint8 event)
{
    // Read available bytes from driver into our circular buffer
    uint16 len = Hal_UART_RxBufLen(port);
    uint8 tempBuf[32]; // Temporary chunk buffer
    
    while (len > 0)
    {
        // Read chunk
        uint16 toRead = (len > sizeof(tempBuf)) ? sizeof(tempBuf) : len;
        uint16 read = HalUARTRead(port, tempBuf, toRead);
        
        // Push to circular buffer
        for (uint16 i = 0; i < read; i++)
        {
            uint16 nextHead = (rxHead + 1) % VESC_UART_RX_BUF_SIZE;
            if (nextHead != rxTail)
            {
                rxBuffer[rxHead] = tempBuf[i];
                rxHead = nextHead;
            }
        }
        
        len -= read;
        
        // Refresh len if more arrived
        if (len == 0) len = Hal_UART_RxBufLen(port);
    }
}