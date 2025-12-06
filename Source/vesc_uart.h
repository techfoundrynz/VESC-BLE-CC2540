/**
 * @file    vesc_uart.h
 */

#ifndef VESC_UART_H
#define VESC_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * CONSTANTS
 */

// Baud rate
#ifndef VESC_UART_BAUD
#define VESC_UART_BAUD      115200
#endif

// RX buffer size
#ifndef VESC_UART_RX_BUF_SIZE
#define VESC_UART_RX_BUF_SIZE   256
#endif

// UART Selection (0 or 1)
#ifndef VESC_UART_PORT
#define VESC_UART_PORT  1
#endif

// UART Alternate Location (1 or 2)
#ifndef VESC_UART_ALT
#define VESC_UART_ALT   2
#endif

/*********************************************************************
 * TYPEDEFS
 */

typedef void (*vescUartCB_t)(void);

/*********************************************************************
 * FUNCTIONS
 */

// API moved to Hardware UART logic
void VescUART_Init(vescUartCB_t callback);
void VescUART_Write(uint8 *buf, uint8 len);
uint16 VescUART_RxBufLen(void);
uint16 VescUART_Read(uint8 *buf, uint16 maxLen);
void VescUART_Poll(void);

#ifdef __cplusplus
}
#endif

#endif /* VESC_UART_H */