/**
 * vesc_service.c
 * 
 * VESC BLE Service - Nordic UART Service compatible
 * Based exactly on TI simpleGATTprofile pattern
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

#define VESC_MAX_DATA_LEN   20
#define VESC_TX_VALUE_IDX   2

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Nordic UART Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
CONST uint8 vescServiceUUID[ATT_UUID_SIZE] =
{ 
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
  0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

// TX Characteristic UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
CONST uint8 vescTxCharUUID[ATT_UUID_SIZE] =
{ 
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
  0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

// RX Characteristic UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
CONST uint8 vescRxCharUUID[ATT_UUID_SIZE] =
{ 
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
  0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

/*********************************************************************
 * LOCAL VARIABLES
 */

static vescServiceCB_t vescServiceCB = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Service attribute
static CONST gattAttrType_t vescService = { ATT_UUID_SIZE, vescServiceUUID };

// TX Characteristic
static uint8 vescTxCharProps = GATT_PROP_NOTIFY;
static uint8 vescTxCharValue[VESC_MAX_DATA_LEN] = {0};
static uint8 vescTxCharUserDesc[] = "TX";

// TX CCCD - pointer, allocated dynamically
static gattCharCfg_t *vescTxCharConfig;

// RX Characteristic
static uint8 vescRxCharProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_WRITE;
static uint8 vescRxCharValue[VESC_MAX_DATA_LEN] = {0};
static uint8 vescRxCharUserDesc[] = "RX";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t vescAttrTbl[] = 
{
  // [0] Service Declaration
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&vescService
  },

  // [1] TX Characteristic Declaration
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ, 
    0,
    &vescTxCharProps 
  },

  // [2] TX Characteristic Value
  { 
    { ATT_UUID_SIZE, vescTxCharUUID },
    0,
    0, 
    vescTxCharValue 
  },

  // [3] TX Characteristic CCCD
  { 
    { ATT_BT_UUID_SIZE, clientCharCfgUUID },
    GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
    0, 
    (uint8 *)&vescTxCharConfig  // Pointer to pointer - will be dereferenced
  },
  
  // [4] TX Characteristic User Description
  { 
    { ATT_BT_UUID_SIZE, charUserDescUUID },
    GATT_PERMIT_READ, 
    0, 
    vescTxCharUserDesc 
  },      

  // [5] RX Characteristic Declaration
  { 
    { ATT_BT_UUID_SIZE, characterUUID },
    GATT_PERMIT_READ, 
    0,
    &vescRxCharProps 
  },

  // [6] RX Characteristic Value
  { 
    { ATT_UUID_SIZE, vescRxCharUUID },
    GATT_PERMIT_WRITE, 
    0, 
    vescRxCharValue 
  },

  // [7] RX Characteristic User Description
  { 
    { ATT_BT_UUID_SIZE, charUserDescUUID },
    GATT_PERMIT_READ, 
    0, 
    vescRxCharUserDesc 
  },
};

#define VESC_NUM_ATTRS  (sizeof(vescAttrTbl) / sizeof(vescAttrTbl[0]))

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
 * SERVICE CALLBACKS
 */
CONST gattServiceCBs_t vescServiceCBs =
{
  vescService_ReadAttrCB,
  vescService_WriteAttrCB,
  NULL
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

bStatus_t vescService_AddService(void)
{
  uint8 status;
  
  // Allocate CCCD table - MUST match linkDBNumConns
  vescTxCharConfig = (gattCharCfg_t *)osal_mem_alloc(sizeof(gattCharCfg_t) * linkDBNumConns);
  if (vescTxCharConfig == NULL)
  {
    return bleMemAllocError;
  }
  
  // Initialize CCCD
  GATTServApp_InitCharCfg(INVALID_CONNHANDLE, vescTxCharConfig);

  // Register connection status callback
  linkDB_Register(vescService_HandleConnStatusCB);
  
  // Register service
  status = GATTServApp_RegisterService(vescAttrTbl, 
                                       VESC_NUM_ATTRS,
                                       GATT_MAX_ENCRYPT_KEY_SIZE,
                                       &vescServiceCBs);

  return status;
}

void vescService_RegisterRxCallback(vescServiceCB_t callback)
{
  vescServiceCB = callback;
}

bStatus_t vescService_SendNotification(uint16 connHandle, uint8 *pData, uint16 len)
{
  attHandleValueNoti_t noti;
  uint16 value;
  bStatus_t status;
  
  if (vescTxCharConfig == NULL)
  {
    return bleNotReady;
  }
  
  // Check if notifications enabled
  value = GATTServApp_ReadCharCfg(connHandle, vescTxCharConfig);
  if (value != GATT_CLIENT_CFG_NOTIFY)
  {
    return bleNotReady;
  }
  
  if (len > VESC_MAX_DATA_LEN)
  {
    len = VESC_MAX_DATA_LEN;
  }
  
  // Allocate buffer
  noti.pValue = (uint8 *)GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, len, NULL);
  if (noti.pValue == NULL)
  {
    return bleNoResources;
  }
  
  // Build notification
  noti.handle = vescAttrTbl[VESC_TX_VALUE_IDX].handle;
  noti.len = len;
  osal_memcpy(noti.pValue, pData, len);
  
  // Send
  status = GATT_Notification(connHandle, &noti, FALSE);
  if (status != SUCCESS)
  {
    GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
  }
  
  return status;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static bStatus_t vescService_ReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr, 
                                        uint8 *pValue, uint8 *pLen, uint16 offset, 
                                        uint8 maxLen, uint8 method)
{
  bStatus_t status = SUCCESS;
  
  if (offset > 0)
  {
    return ATT_ERR_ATTR_NOT_LONG;
  }

  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    
    if (uuid == GATT_CLIENT_CHAR_CFG_UUID)
    {
      *pLen = 2;
      osal_memcpy(pValue, pAttr->pValue, 2);
    }
    else
    {
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else
  {
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
  }

  return status;
}

static bStatus_t vescService_WriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                         uint8 *pValue, uint8 len, uint16 offset,
                                         uint8 method)
{
  bStatus_t status = SUCCESS;
  
  if (offset > 0)
  {
    return ATT_ERR_ATTR_NOT_LONG;
  }
  
  if (pAttr->type.len == ATT_BT_UUID_SIZE)
  {
    uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
    
    if (uuid == GATT_CLIENT_CHAR_CFG_UUID)
    {
      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                              offset, GATT_CLIENT_CFG_NOTIFY);
      if (status == SUCCESS)
      {
        uint16 charCfg = BUILD_UINT16(pValue[0], pValue[1]);
        extern void vescBLE_SetNotificationsEnabled(uint8 enabled);
        vescBLE_SetNotificationsEnabled(charCfg == GATT_CLIENT_CFG_NOTIFY);
      }
    }
    else
    {
      // Let GATT server handle other 16-bit UUIDs
      status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else if (pAttr->type.len == ATT_UUID_SIZE)
  {
    // Check if this is the RX characteristic (128-bit UUID)
    if (osal_memcmp(pAttr->type.uuid, vescRxCharUUID, ATT_UUID_SIZE) == TRUE)
    {
      if (len > VESC_MAX_DATA_LEN)
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
      else
      {
        // Copy data to characteristic value
        osal_memcpy(pAttr->pValue, pValue, len);
        
        // Forward to UART callback
        if (vescServiceCB != NULL)
        {
          vescServiceCB(pValue, len);
        }
        
        status = SUCCESS;
      }
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_FOUND;
    }
  }
  else
  {
    status = ATT_ERR_INVALID_HANDLE;
  }

  return status;
}

static void vescService_HandleConnStatusCB(uint16 connHandle, uint8 changeType)
{ 
  if (connHandle != LOOPBACK_CONNHANDLE)
  {
    if ((changeType == LINKDB_STATUS_UPDATE_REMOVED) ||
        ((changeType == LINKDB_STATUS_UPDATE_STATEFLAGS) && 
         (!linkDB_Up(connHandle))))
    { 
      GATTServApp_InitCharCfg(connHandle, vescTxCharConfig);
      
      extern void vescBLE_SetNotificationsEnabled(uint8 enabled);
      vescBLE_SetNotificationsEnabled(FALSE);
    }
  }
}

/*********************************************************************
*********************************************************************/