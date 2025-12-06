/**
 * @file    nus_service.h
 * @brief   Nordic UART Service (NUS) Header for CC2540
 * 
 * This defines the Nordic UART Service compatible with VESC Tool.
 * 
 * @author  Port for VESC compatibility
 * @date    2024
 */

#ifndef NUS_SERVICE_H
#define NUS_SERVICE_H

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

// Profile Parameters
#define NUS_RX_DATA               0   // RX data (written by central)
#define NUS_TX_DATA               1   // TX data (notified to central)

// NUS Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
// Stored in little-endian (reversed) format for BLE stack
#define NUS_SERVICE_UUID_BYTES    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                                  0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E

// Maximum data length per characteristic (BLE 4.0 default MTU - 3)
#define NUS_MAX_DATA_LEN          20

// Service bit field
#define NUS_SERVICE               0x00000001

/*********************************************************************
 * TYPEDEFS
 */

// NUS Service callback function
typedef void (*nusServiceCB_t)(uint8 paramID);

// Callback structure
typedef struct
{
    nusServiceCB_t pfnNusServiceCB;  // Called when characteristic changes
} nusServiceCBs_t;

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
 * @fn      NUS_AddService
 * @brief   Initializes the Nordic UART Service by registering
 *          GATT attributes with the GATT server.
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 * @return  Success or Failure
 */
extern bStatus_t NUS_AddService(uint32 services);

/**
 * @fn      NUS_RegisterAppCBs
 * @brief   Registers the application callback function.
 * @param   appCallbacks - pointer to application callbacks.
 * @return  Success or Failure
 */
extern bStatus_t NUS_RegisterAppCBs(nusServiceCBs_t *appCallbacks);

/**
 * @fn      NUS_SetParameter
 * @brief   Set a NUS parameter.
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write
 * @return  bStatus_t
 */
extern bStatus_t NUS_SetParameter(uint8 param, uint8 len, void *value);

/**
 * @fn      NUS_GetParameter
 * @brief   Get a NUS parameter.
 * @param   param - Profile parameter ID
 * @param   len - pointer to length of data
 * @param   value - pointer to data
 * @return  bStatus_t
 */
extern bStatus_t NUS_GetParameter(uint8 param, uint8 *len, void *value);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* NUS_SERVICE_H */
