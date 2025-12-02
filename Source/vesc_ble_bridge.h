/**
 * vesc_ble_bridge.h
 * 
 * VESC BLE Bridge main application header
 */

#ifndef VESC_BLE_BRIDGE_H
#define VESC_BLE_BRIDGE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "vesc_service.h"

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/**
 * @fn      vescBLE_Init
 *
 * @brief   Initialization function for the VESC BLE Task.
 *
 * @param   task_id - the ID assigned by OSAL.
 *
 * @return  none
 */
extern void vescBLE_Init(uint8 task_id);

/**
 * @fn      vescBLE_ProcessEvent
 *
 * @brief   VESC BLE Task event processor.
 *
 * @param   task_id - the ID assigned by OSAL.
 * @param   events - events to process.
 *
 * @return  events not processed
 */
extern uint16 vescBLE_ProcessEvent(uint8 task_id, uint16 events);

/**
 * @fn      vescBLE_SetNotificationsEnabled
 *
 * @brief   Enable/disable notifications (called by VESC Service)
 *
 * @param   enabled - TRUE to enable, FALSE to disable
 *
 * @return  none
 */
extern void vescBLE_SetNotificationsEnabled(uint8 enabled);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* VESC_BLE_BRIDGE_H */