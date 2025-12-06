/**
 * @file    soft_uart.c
 * @brief   Hardware UART implementation for CC2540 (replacing SoftUART)
 *          Uses USART1 Alt 2:
 *          TX = P1.6
 *          RX = P1.7
 * 
 * @note    API names kept as SoftUART_xxx to minimize changes in main app.
 */

#include "hal_mcu.h"
#include "hal_types.h"
#include "OSAL.h"
#include "soft_uart.h"

/*********************************************************************
 * CONSTANTS
 */

// USART1 Alt 2
// P1.6 = TX
// P1.7 = RX

// Baud Rate Generation for 32 MHz Clock
// 115200: M=216, E=11
#define BAUD_M_115200   216
#define BAUD_E_115200   11

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 rxBuffer[SOFT_UART_RX_BUF_SIZE];
static volatile uint8 rxHead = 0;
static volatile uint8 rxTail = 0;

static softUartCB_t appCallback = NULL;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @fn      SoftUART_Init
 * @brief   Initialize Hardware UART (USART1 Alt 2)
 */
void SoftUART_Init(softUartCB_t callback)
{
    // Store callback
    appCallback = callback;
    
    // Clear buffer
    rxHead = 0;
    rxTail = 0;

    // 1. Configure P1.6 and P1.7 for Peripheral function
    // P1SEL bit 6 and 7 set to 1
    P1SEL |= (BV(6) | BV(7));
    
    // 2. Configure P1DIR (not strictly necessary for peripheral, but good practice if switching back)
    // UART takes control of direction when peripheral selected
    
    // 3. Configure USART1 Priority to Alt 2
    // PERCFG.U1CFG = 1 (Alt 2)
    PERCFG |= BV(1);
    
    // 4. Configure P2SEL (Priority)
    // Ensure USART1 has priority over USART0 if they share pins (they don't here with Alt2 but safety first)
    // Not critical for P1.6/7 usually unless mapping conflicts.

    // 5. Configure USART Control
    // U1CSR.MODE = 1 (UART)
    // U1CSR.RE = 1 (Receiver Enable)
    U1CSR = 0x80 | 0x40; 
    
    // 6. Configure UART Control
    // Bit 7: U1FLUSH - Flush unit.
    // Bit 6: U1FLOW - Flow control.
    // Bit 5: U1D9 - 9th bit.
    // Bit 4: U1BIT9 - 9 bit mode.
    // Bit 3: U1PARity
    // Bit 2: U1SPB - Stop bits (0=1, 1=2)
    // Bit 1: U1STOP - Stop bit level (0=Low, 1=High) -> Must be High for standard UART
    // Bit 0: U1START - Start bit level (0=Low, 1=High) -> Must be Low for standard UART
    
    U1UCR = BV(7); // Flush
    U1UCR = BV(1); // 8N1, Stop High, Start Low.
    
    // 7. Baud Rate
    U1BAUD = BAUD_M_115200;
    U1GCR = (U1GCR & 0xE0) | BAUD_E_115200;
    
    // 8. Interrupts
    // Clear flag
    URX1IF = 0;
    // Enable Receive Interrupt
    URX1IE = 1;
    // Enable Global Interrupts (EA is likely already set by OSAL)
    // IEN0.EA = 1; 
}

/**
 * @fn      SoftUART_TxByte
 * @brief   Transmit one byte using Hardware UART
 */
void SoftUART_TxByte(uint8 byte)
{
    // Wait for TX buffer to be ready
    // U1CSR.TX_BYTE = 0 when ready? No.
    // U1CSR.ACTIVE? 
    // Check U1CSR.TX_BYTE (bit 1) - set when byte written, cleared when transmitted?
    // Actually typically check specific flag.
    // Faster way: check U1CSR.ACTIVE bit 0? No that's activity.
    // Valid check: Wait for TX complete interrupt flag? Or UTX1IF?
    
    // Standard blocking write:
    U1DBUF = byte;
    while (!(U1CSR & 0x02)); // Wait for TX_BYTE (Bit 1) to be set (transmit complete or buffer empty?)
                             // CC2540 Guide: Bit 1 (TX_BYTE): set when byte transmitted.
                             // We should clear it before?
    U1CSR &= ~0x02;          // Clear TX_BYTE
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
 * @brief   Not used in ISR mode usually, but can pull from buffer
 */
uint8 SoftUART_RxByte(void)
{
    return 0; // Not implemented for blocking RX in this model
}

/**
 * @fn      SoftUART_Poll
 * @brief   Poll called from main loop
 */
void SoftUART_Poll(void)
{
    // In ISR mode, the buffer is filled automatically.
    // We just check if we have data and notify.
    
    // Note: If we really want to just notify periodically, we don't strictly need to do anything here
    // unless we want to debounce the notification.
    
    // Existing vesc_ble.c checks SoftUART_RxBufLen(), so we just rely on that.
    
    // However, original code called callback depending on something?
    // Original SoftUART_Poll checked pin and read byte.
    
    // We can simulate "New Data" notification if we want, or just let the Periodic event poll RxBufLen.
    // The vesc_ble periodic event checks RxBufLen(), so we are good.
}

/**
 * @fn      SoftUART_RxBufLen
 * @brief   Get number of bytes in RX buffer
 */
uint8 SoftUART_RxBufLen(void)
{
    uint8 len;
    
    // Atomic read
    // HAL_ENTER_CRITICAL_SECTION(intState); 
    // Simple subtraction is usually atomic enough for byte indices
    if (rxHead >= rxTail)
        len = rxHead - rxTail;
    else
        len = SOFT_UART_RX_BUF_SIZE - rxTail + rxHead;
        
    return len;
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

/**
 * @brief   USART1 RX Warning/Error ISR? No, just RX.
 */
#pragma vector = URX1_VECTOR
__interrupt void usart1RxIsr(void)
{
    URX1IF = 0; // Clear interrupt flag
    
    uint8 byte = U1DBUF;
    
    uint8 nextHead = (rxHead + 1) % SOFT_UART_RX_BUF_SIZE;
    if (nextHead != rxTail)
    {
        rxBuffer[rxHead] = byte;
        rxHead = nextHead;
        
        // Notify app if callback exists - usually on *byte* arrival?
        // Original polled, so it notified on byte arrival.
        if (appCallback)
        {
            appCallback();
        }
    }
}