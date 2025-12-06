/**
 * @file    vesc_ble.h
 * @brief   VESC BLE UART Bridge Application Header
 * 
 * @author  Port for VESC compatibility
 * @date    2024
 */

#ifndef VESC_BLE_H
#define VESC_BLE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// VESC BLE Task Events
#define VESC_BLE_START_DEVICE_EVT     0x0001

#define VESC_BLE_UART_RX_EVT          0x0004

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/**
 * @fn      VESC_BLE_Init
 * @brief   Initialization function for the VESC BLE UART Bridge Task.
 * @param   task_id - the ID assigned by OSAL.
 */
extern void VESC_BLE_Init(uint8 task_id);

/**
 * @fn      VESC_BLE_ProcessEvent
 * @brief   Application Task event processor.
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.
 * @return  events not processed
 */
extern uint16 VESC_BLE_ProcessEvent(uint8 task_id, uint16 events);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* VESC_BLE_H */
