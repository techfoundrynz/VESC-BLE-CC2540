/**
 * @file    soft_uart.h
 */

#ifndef SOFT_UART_H
#define SOFT_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * CONSTANTS
 */

// Baud rate 
#ifndef SOFT_UART_BAUD
#define SOFT_UART_BAUD      9600
#endif

// RX buffer size
#ifndef SOFT_UART_RX_BUF_SIZE
#define SOFT_UART_RX_BUF_SIZE   128
#endif

/*********************************************************************
 * TYPEDEFS
 */

typedef void (*softUartCB_t)(void);

/*********************************************************************
 * FUNCTIONS
 */

void SoftUART_Init(softUartCB_t callback);
void SoftUART_TxByte(uint8 byte);
void SoftUART_Write(uint8 *buf, uint8 len);
uint8 SoftUART_RxByte(void);
void SoftUART_Poll(void);
uint8 SoftUART_RxBufLen(void);
uint8 SoftUART_Read(uint8 *buf, uint8 maxLen);

#ifdef __cplusplus
}
#endif

#endif /* SOFT_UART_H */