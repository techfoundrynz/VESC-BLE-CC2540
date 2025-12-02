/**
 * VESC BLE Service header file
 */

#ifndef VESC_SERVICE_H
#define VESC_SERVICE_H

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

// VESC Service UUID
#define VESC_SERVICE_UUID               0xFFE0

// VESC Service Characteristics
#define VESC_TX_CHAR_UUID               0xFFE1  // Notify (device to app)
#define VESC_RX_CHAR_UUID               0xFFE2  // Write (app to device)

/*********************************************************************
 * TYPEDEFS
 */

// Callback function type for RX data
typedef void (*vescServiceCB_t)(uint8 *data, uint16 len);

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

/*********************************************************************
 * API FUNCTIONS 
 */

/**
 * @fn      vescService_AddService
 *
 * @brief   Initializes the VESC Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
extern bStatus_t vescService_AddService(void);

/**
 * @fn      vescService_RegisterRxCallback
 *
 * @brief   Register a callback function to handle RX data
 *
 * @param   callback - callback function
 *
 * @return  none
 */
extern void vescService_RegisterRxCallback(vescServiceCB_t callback);

/**
 * @fn      vescService_SendNotification
 *
 * @brief   Send a notification with data
 *
 * @param   connHandle - connection handle
 * @param   pData - pointer to data
 * @param   len - length of data
 *
 * @return  Success or Failure
 */
extern bStatus_t vescService_SendNotification(uint16 connHandle, uint8 *pData, uint16 len);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* VESC_SERVICE_H */