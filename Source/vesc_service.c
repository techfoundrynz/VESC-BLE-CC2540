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

// VESC Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E (Nordic UART Service)
CONST uint8 vescServiceUUID[ATT_UUID_SIZE] =
{ 
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
  0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

// TX Characteristic UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E (Notify - device to app)
CONST uint8 vescTxCharUUID[ATT_UUID_SIZE] =
{ 
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
  0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

// RX Characteristic UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E (Write - app to device)
CONST uint8 vescRxCharUUID[ATT_UUID_SIZE] =
{ 
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
  0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
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
static CONST gattAttrType_t vescService = { ATT_UUID_SIZE, vescServiceUUID };

// TX Characteristic Properties
static uint8 vescTxCharProps = GATT_PROP_NOTIFY;

// TX Characteristic Value
static uint8 vescTxCharValue[VESC_MAX_DATA_LEN] = {0};

// TX Characteristic Configs (will be allocated dynamically)
static gattCharCfg_t *vescTxCharConfig = NULL;

// Placeholder for TX Config pointer in attribute table
static uint8 *vescTxCharConfigPtr = NULL;

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
        { ATT_UUID_SIZE, vescTxCharUUID },
        0, 
        0, 
        (uint8 *)vescTxCharValue 
      },

      // TX Characteristic configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&vescTxCharConfigPtr
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
        { ATT_UUID_SIZE, vescRxCharUUID },
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

// Calculate number of attributes at compile time
#define VESC_NUM_ATTRS (sizeof(vescAttrTbl) / sizeof(gattAttribute_t))

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

  // Allocate Client Characteristic Configuration table
  vescTxCharConfig = (gattCharCfg_t *)osal_mem_alloc(sizeof(gattCharCfg_t) * linkDBNumConns);
  
  if (vescTxCharConfig == NULL)
  {
    return (bleMemAllocError);
  }

  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, vescTxCharConfig);
  
  // Update the pointer that the attribute table references
  vescTxCharConfigPtr = (uint8 *)vescTxCharConfig;

  // Register with Link DB to receive link status change callback
  VOID linkDB_Register(vescService_HandleConnStatusCB);
  
  // Register GATT attribute list and CBs with GATT Server App
  status = GATTServApp_RegisterService(vescAttrTbl, 
                                        VESC_NUM_ATTRS,
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
    // Find the TX characteristic value handle in the attribute table
    // TX Characteristic Value is at index 2 (Service=0, TX Decl=1, TX Value=2)
    noti.handle = vescAttrTbl[2].handle;
    
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
 
  if (pAttr->type.len == ATT_UUID_SIZE)
  {
    // 128-bit UUID - compare full UUID
    if (osal_memcmp(pAttr->type.uuid, vescTxCharUUID, ATT_UUID_SIZE) ||
        osal_memcmp(pAttr->type.uuid, vescRxCharUUID, ATT_UUID_SIZE))
    {
      // TX and RX characteristics don't have read permissions
      *pLen = 0;
      status = SUCCESS;
    }
    else
    {
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch (uuid)
    {
      default:
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
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
  
  if (pAttr->type.len == ATT_UUID_SIZE)
  {
    // 128-bit UUID - check if it's RX characteristic
    if (osal_memcmp(pAttr->type.uuid, vescRxCharUUID, ATT_UUID_SIZE))
    {
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
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    
    switch (uuid)
    {
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