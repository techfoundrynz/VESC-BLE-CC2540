/**
 * VESC BLE Service implementation for CC2540
 * Provides TX/RX characteristics for UART bridge functionality
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"
#include "vesc_service.h"

/*********************************************************************
 * CONSTANTS
 */

// VESC Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (Nordic UART Service compatible)
#define VESC_SERVICE_UUID               0xFFE0

// TX Characteristic UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
#define VESC_TX_CHAR_UUID               0xFFE1

// RX Characteristic UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
#define VESC_RX_CHAR_UUID               0xFFE2

// Max data length
#define VESC_MAX_DATA_LEN               20

// Max number of connections (CC2540 supports 1)
#ifndef GATT_MAX_NUM_CONN
#define GATT_MAX_NUM_CONN               4
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// VESC Service UUID
CONST uint8 vescServiceUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(VESC_SERVICE_UUID), HI_UINT16(VESC_SERVICE_UUID)
};

// TX Characteristic UUID (notify from device to app)
CONST uint8 vescTxCharUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(VESC_TX_CHAR_UUID), HI_UINT16(VESC_TX_CHAR_UUID)
};

// RX Characteristic UUID (write from app to device)
CONST uint8 vescRxCharUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(VESC_RX_CHAR_UUID), HI_UINT16(VESC_RX_CHAR_UUID)
};

/*********************************************************************
 * LOCAL VARIABLES
 */

// Callback function pointer
static vescServiceCB_t vescServiceCB = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// VESC Service attribute
static CONST gattAttrType_t vescService = { ATT_BT_UUID_SIZE, vescServiceUUID };

// TX Characteristic Properties
static uint8 vescTxCharProps = GATT_PROP_NOTIFY;

// TX Characteristic Value
static uint8 vescTxCharValue[VESC_MAX_DATA_LEN] = {0};

// TX Characteristic Configs
static gattCharCfg_t vescTxCharConfig[GATT_MAX_NUM_CONN];

// TX Characteristic User Description
static uint8 vescTxCharUserDesc[] = "VESC TX";

// RX Characteristic Properties
static uint8 vescRxCharProps = GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

// RX Characteristic Value
static uint8 vescRxCharValue[VESC_MAX_DATA_LEN] = {0};

// RX Characteristic User Description
static uint8 vescRxCharUserDesc[] = "VESC RX";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t vescAttrTbl[] = 
{
  // VESC Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&vescService                     /* pValue */
  },

    // TX Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &vescTxCharProps 
    },

      // TX Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, vescTxCharUUID },
        0, 
        0, 
        (uint8 *)vescTxCharValue 
      },

      // TX Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)vescTxCharConfig 
      },
      
      // TX Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        vescTxCharUserDesc 
      },      

    // RX Characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &vescRxCharProps 
    },

      // RX Characteristic Value
      { 
        { ATT_BT_UUID_SIZE, vescRxCharUUID },
        GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)vescRxCharValue 
      },

      // RX Characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        vescRxCharUserDesc 
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t vescService_ReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr, 
                                     uint8 *pValue, uint8 *pLen, uint16 offset, 
                                     uint8 maxLen, uint8 method);
static bStatus_t vescService_WriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                          uint8 *pValue, uint8 len, uint16 offset,
                                          uint8 method);
static void vescService_HandleConnStatusCB(uint16 connHandle, uint8 changeType);

/*********************************************************************
 * PROFILE CALLBACKS
 */
// VESC Service Callbacks
CONST gattServiceCBs_t vescServiceCBs =
{
  vescService_ReadAttrCB,  // Read callback function pointer
  vescService_WriteAttrCB, // Write callback function pointer
  NULL                     // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @fn      vescService_AddService
 *
 * @brief   Initializes the VESC Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t vescService_AddService(void)
{
  uint8 status = SUCCESS;

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, vescTxCharConfig);

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register(vescService_HandleConnStatusCB);
  
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(vescAttrTbl, 
                                        GATT_NUM_ATTRS(vescAttrTbl),
                                        GATT_MAX_ENCRYPT_KEY_SIZE,
                                        &vescServiceCBs);

  return (status);
}

/**
 * @fn      vescService_RegisterRxCallback
 *
 * @brief   Register a callback function to handle RX data
 *
 * @param   callback - callback function
 *
 * @return  none
 */
void vescService_RegisterRxCallback(vescServiceCB_t callback)
{
  vescServiceCB = callback;
}

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
bStatus_t vescService_SendNotification(uint16 connHandle, uint8 *pData, uint16 len)
{
  attHandleValueNoti_t noti;
  
  // Check if notifications are enabled
  if (GATTServApp_ReadCharCfg(connHandle, vescTxCharConfig) != GATT_CLIENT_CFG_NOTIFY)
  {
    return FAILURE;
  }
  
  // Limit length
  if (len > VESC_MAX_DATA_LEN)
  {
    len = VESC_MAX_DATA_LEN;
  }
  
  // Copy data to characteristic value
  VOID osal_memcpy(vescTxCharValue, pData, len);
  
  // Setup notification
  noti.len = len;
  noti.pValue = (uint8 *)GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, len, NULL);
  
  if (noti.pValue != NULL)
  {
    // Find the handle
    uint8 i;
    for (i = 0; i < GATT_NUM_ATTRS(vescAttrTbl); i++)
    {
      if (vescAttrTbl[i].pValue == (uint8 *)vescTxCharValue)
      {
        noti.handle = i + 1; // Handle is 1-based
        break;
      }
    }
    
    VOID osal_memcpy(noti.pValue, pData, len);
    
    // Send notification
    if (GATT_Notification(connHandle, &noti, FALSE) != SUCCESS)
    {
      GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
      return FAILURE;
    }
    
    return SUCCESS;
  }
  
  return FAILURE;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/**
 * @fn      vescService_ReadAttrCB
 *
 * @brief   Read an attribute.
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be read
 * @param   pLen - length of data to be read
 * @param   offset - offset of the first octet to be read
 * @param   maxLen - maximum length of data to be read
 * @param   method - type of read message
 *
 * @return  Success or Failure
 */
static bStatus_t vescService_ReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr, 
                                     uint8 *pValue, uint8 *pLen, uint16 offset, 
                                     uint8 maxLen, uint8 method)
{
  bStatus_t status = SUCCESS;

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }
 
  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    
    switch (uuid)
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads
      
      // TX and RX characteristics don't have read permissions, but return no error
      case VESC_TX_CHAR_UUID:
      case VESC_RX_CHAR_UUID:
        *pLen = 0;
        status = SUCCESS;
        break;
        
      default:
        // Should never get here!
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return (status);
}

/**
 * @fn      vescService_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  Success or Failure
 */
static bStatus_t vescService_WriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                          uint8 *pValue, uint8 len, uint16 offset,
                                          uint8 method)
{
  bStatus_t status = SUCCESS;
  
  // Make sure it's not a blob operation
  if (offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }
  
  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    
    switch (uuid)
    {
      case VESC_RX_CHAR_UUID:
        // Validate the value length
        if (len > VESC_MAX_DATA_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
        else
        {
          // Copy the data
          VOID osal_memcpy(pAttr->pValue, pValue, len);
          
          // Call the callback function if registered
          if (vescServiceCB != NULL)
          {
            vescServiceCB(pValue, len);
          }
        }
        break;
        
      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY);
        
        // Notify application of configuration change
        if (status == SUCCESS)
        {
          uint16 charCfg = BUILD_UINT16(pValue[0], pValue[1]);
          
          // Call external function to update notification status
          extern void vescBLE_SetNotificationsEnabled(uint8 enabled);
          vescBLE_SetNotificationsEnabled(charCfg == GATT_CLIENT_CFG_NOTIFY);
        }
        break;       
        
      default:
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  return (status);
}

/**
 * @fn      vescService_HandleConnStatusCB
 *
 * @brief   Link DB Status change callback
 *
 * @param   connHandle - connection handle
 * @param   changeType - type of change
 *
 * @return  none
 */
static void vescService_HandleConnStatusCB(uint16 connHandle, uint8 changeType)
{ 
  // Make sure this is not loopback connection
  if (connHandle != LOOPBACK_CONNHANDLE)
  {
    // Reset Client Char Config if connection has dropped
    if ((changeType == LINKDB_STATUS_UPDATE_REMOVED) ||
        ((changeType == LINKDB_STATUS_UPDATE_STATEFLAGS) && 
         (!linkDB_Up(connHandle))))
    { 
      GATTServApp_InitCharCfg(connHandle, vescTxCharConfig);
    }
  }
}

/*********************************************************************
*********************************************************************/