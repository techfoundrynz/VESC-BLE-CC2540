/**
 * @file    nus_service.c
 * @brief   Nordic UART Service (NUS) for CC2540
 */

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "nus_service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED    8

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Nordic UART Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
CONST uint8 nusServiceUUID[ATT_UUID_SIZE] =
{
    NUS_SERVICE_UUID_BYTES
};

// RX Characteristic UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E
CONST uint8 nusRxCharUUID[ATT_UUID_SIZE] =
{
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E
};

// TX Characteristic UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E
CONST uint8 nusTxCharUUID[ATT_UUID_SIZE] =
{
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E
};

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Application callback
static nusServiceCBs_t *nusService_AppCBs = NULL;

// RX data buffer 
static uint8 nusRxData[NUS_MAX_DATA_LEN] = {0};
static uint8 nusRxDataLen = 0;

// TX data length tracking
static uint8 nusTxDataLen = 0;

/*********************************************************************
 * Profile Attributes - variables
 */

// NUS Service attribute
static CONST gattAttrType_t nusService = { ATT_UUID_SIZE, nusServiceUUID };

// RX Characteristic Properties
static uint8 nusRxCharProps = GATT_PROP_WRITE | GATT_PROP_WRITE_NO_RSP;

// RX Characteristic Value
static uint8 nusRxCharValue[NUS_MAX_DATA_LEN] = {0};

// RX Characteristic User Description
static uint8 nusRxCharUserDesp[] = "UART RX";

// TX Characteristic Properties
static uint8 nusTxCharProps = GATT_PROP_NOTIFY;

// TX Characteristic Value
static uint8 nusTxCharValue[NUS_MAX_DATA_LEN] = {0};

// TX Characteristic Configuration (for notifications)
static gattCharCfg_t nusTxCharConfig[GATT_MAX_NUM_CONN];

// TX Characteristic User Description
static uint8 nusTxCharUserDesp[] = "UART TX";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t nusAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] =
{
    // NUS Service Declaration
    {
        { ATT_BT_UUID_SIZE, primaryServiceUUID }, 
        GATT_PERMIT_READ,                         
        0,                                        
        (uint8 *)&nusService                      
    },
    
    // RX Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &nusRxCharProps
    },
    
    // RX Characteristic Value
    {
        { ATT_UUID_SIZE, nusRxCharUUID },
        GATT_PERMIT_WRITE,
        0,
        nusRxCharValue
    },
    
    // RX Characteristic User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        nusRxCharUserDesp
    },
    
    // TX Characteristic Declaration
    {
        { ATT_BT_UUID_SIZE, characterUUID },
        GATT_PERMIT_READ,
        0,
        &nusTxCharProps
    },
    
    // TX Characteristic Value
    {
        { ATT_UUID_SIZE, nusTxCharUUID },
        0,  // No direct
        0,
        nusTxCharValue
    },
    
    // TX Characteristic Configuration 
    {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)nusTxCharConfig
    },
    
    // TX Characteristic User Description
    {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        nusTxCharUserDesp
    }
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 nusReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                           uint8 *pValue, uint8 *pLen, uint16 offset,
                           uint8 maxLen);
static bStatus_t nusWriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                uint8 *pValue, uint8 len, uint16 offset);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// NUS Service Callbacks
CONST gattServiceCBs_t nusServiceCBs =
{
    nusReadAttrCB,  
    nusWriteAttrCB,  
    NULL             
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @fn      NUS_AddService
 * @brief   Initializes the Nordic UART Service by registering
 *          GATT attributes with the GATT server.
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 * @return  Success or Failure
 */
bStatus_t NUS_AddService(uint32 services)
{
    uint8 status = SUCCESS;
    
    
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, nusTxCharConfig);
    
    if (services & NUS_SERVICE)
    {
        // Register GATT attribute list and callbacks with GATT Server App
        status = GATTServApp_RegisterService(nusAttrTbl,
                                             GATT_NUM_ATTRS(nusAttrTbl),
                                             &nusServiceCBs);
    }
    
    return (status);
}

/**
 * @fn      NUS_RegisterAppCBs
 * @brief   Registers the application callback function.
 * @param   appCallbacks - pointer to application callbacks.
 * @return  Success or Failure
 */
bStatus_t NUS_RegisterAppCBs(nusServiceCBs_t *appCallbacks)
{
    if (appCallbacks)
    {
        nusService_AppCBs = appCallbacks;
        return (SUCCESS);
    }
    else
    {
        return (bleAlreadyInRequestedMode);
    }
}

/**
 * @fn      NUS_SetParameter
 * @brief   Set a NUS parameter.
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write
 * @return  bStatus_t
 */
bStatus_t NUS_SetParameter(uint8 param, uint8 len, void *value)
{
    bStatus_t ret = SUCCESS;
    
    switch (param)
    {
        case NUS_TX_DATA:
            if (len <= NUS_MAX_DATA_LEN)
            {
                VOID osal_memcpy(nusTxCharValue, value, len);
                nusTxDataLen = len;
                
                // Send notification if enabled
                GATTServApp_ProcessCharCfg(nusTxCharConfig, nusTxCharValue,
                                          FALSE, nusAttrTbl, 
                                          GATT_NUM_ATTRS(nusAttrTbl),
                                          INVALID_TASK_ID);
            }
            else
            {
                ret = bleInvalidRange;
            }
            break;
            
        default:
            ret = INVALIDPARAMETER;
            break;
    }
    
    return (ret);
}

/**
 * @fn      NUS_GetParameter
 * @brief   Get a NUS parameter.
 * @param   param - Profile parameter ID
 * @param   len - pointer to length of data
 * @param   value - pointer to data
 * @return  bStatus_t
 */
bStatus_t NUS_GetParameter(uint8 param, uint8 *len, void *value)
{
    bStatus_t ret = SUCCESS;
    
    switch (param)
    {
        case NUS_RX_DATA:
            *len = nusRxDataLen;
            VOID osal_memcpy(value, nusRxData, nusRxDataLen);
            break;
            
        case NUS_TX_DATA:
            *len = nusTxDataLen;
            VOID osal_memcpy(value, nusTxCharValue, nusTxDataLen);
            break;
            
        default:
            ret = INVALIDPARAMETER;
            break;
    }
    
    return (ret);
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/**
 * @fn      nusReadAttrCB
 * @brief   Read an attribute.
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be read
 * @param   pLen - length of data to be read
 * @param   offset - offset of the first octet to be read
 * @param   maxLen - maximum length of data to be read
 * @return  Success or Failure
 */
static uint8 nusReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                           uint8 *pValue, uint8 *pLen, uint16 offset,
                           uint8 maxLen)
{
    bStatus_t status = SUCCESS;
    
    // Suppress unused parameter warning
    (void)connHandle;
    (void)maxLen;
    
    // Make sure it's not a blob operation
    if (offset > 0)
    {
        return (ATT_ERR_ATTR_NOT_LONG);
    }
    
    if (pAttr->type.len == ATT_UUID_SIZE)
    {
        // 128-bit UUID
        if (osal_memcmp(pAttr->type.uuid, nusTxCharUUID, ATT_UUID_SIZE))
        {
            // TX characteristic - return current value
            *pLen = nusTxDataLen;
            VOID osal_memcpy(pValue, pAttr->pValue, nusTxDataLen);
        }
        else
        {
            // Should never get here
            *pLen = 0;
            status = ATT_ERR_ATTR_NOT_FOUND;
        }
    }
    else
    {
        // 16-bit UUID
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
    }
    
    return (status);
}

/**
 * @fn      nusWriteAttrCB
 * @brief   Validate attribute data prior to a write operation
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @return  Success or Failure
 */
static bStatus_t nusWriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                uint8 *pValue, uint8 len, uint16 offset)
{
    bStatus_t status = SUCCESS;
    
    if (pAttr->type.len == ATT_BT_UUID_SIZE)
    {
        // 16-bit UUID
        uint16 uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
        
        switch (uuid)
        {
            case GATT_CLIENT_CHAR_CFG_UUID:
                // Client Characteristic Configuration
                status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                        offset, GATT_CLIENT_CFG_NOTIFY);
                break;
                
            default:
                status = ATT_ERR_ATTR_NOT_FOUND;
                break;
        }
    }
    else if (pAttr->type.len == ATT_UUID_SIZE)
    {
        // 128-bit UUID
        if (osal_memcmp(pAttr->type.uuid, nusRxCharUUID, ATT_UUID_SIZE))
        {
            // RX Characteristic - data received from BLE central
            if (offset == 0)
            {
                if (len > NUS_MAX_DATA_LEN)
                {
                    status = ATT_ERR_INVALID_VALUE_SIZE;
                }
            }
            else
            {
                status = ATT_ERR_ATTR_NOT_LONG;
            }
            
            if (status == SUCCESS)
            {
                // Store received data
                VOID osal_memcpy(nusRxData, pValue, len);
                nusRxDataLen = len;
                
                // Notify application
                if (nusService_AppCBs && nusService_AppCBs->pfnNusServiceCB)
                {
                    nusService_AppCBs->pfnNusServiceCB(NUS_RX_DATA);
                }
            }
        }
        else
        {
            status = ATT_ERR_ATTR_NOT_FOUND;
        }
    }
    else
    {
        // Invalid UUID length
        status = ATT_ERR_INVALID_HANDLE;
    }
    
    return (status);
}

/*********************************************************************
*********************************************************************/
