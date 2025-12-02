/**
 * CC2540 VESC BLE UART Bridge Firmware
 * UART Configuration: 115200 baud, 8N1
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_uart.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "linkdb.h"
#include "peripheral.h"
#include "gapbondmgr.h"
#include "vesc_ble_bridge.h"
#include "vesc_service.h"

/*********************************************************************
 * CONSTANTS
 */

// Device name
#define DEVICE_NAME                   "VESC BLE"

// VESC UART configuration
#define VESC_UART_BAUD                HAL_UART_BR_115200
#define VESC_UART_PORT                HAL_UART_PORT_0

// BLE Connection parameters - optimized for VESC
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     6     // 7.5ms (units of 1.25ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     12    // 15ms (units of 1.25ms)
#define DEFAULT_DESIRED_SLAVE_LATENCY         0
#define DEFAULT_DESIRED_CONN_TIMEOUT          200   // 2s (units of 10ms)

// BLE Advertising parameters
#define DEFAULT_ADVERTISING_INTERVAL          800   // 500ms (units of 0.625ms)
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ
#define DEFAULT_MITM_MODE                     FALSE
#define DEFAULT_BONDING_MODE                  TRUE
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// UART Buffer sizes
#define UART_RX_BUFFER_SIZE                   512
#define UART_TX_BUFFER_SIZE                   512

// BLE MTU size (default for CC2540)
#define BLE_MTU_SIZE                          20

// Task events
#define VESC_UART_RX_EVT                      0x0001
#define VESC_UART_TX_EVT                      0x0002
#define VESC_START_DEVICE_EVT                 0x0004
#define VESC_PERIODIC_EVT                     0x0008

// Periodic event period (ms)
#define VESC_PERIODIC_EVT_PERIOD              50

// Maximum retry attempts for BLE notifications
#define MAX_NOTIFICATION_RETRIES              3

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Task ID
uint8 vescBLE_TaskID;

/*********************************************************************
 * LOCAL VARIABLES
 */

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  0x09,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  'V', 'E', 'S', 'C', ' ', 'B', 'L', 'E',  // Device name
  
  // Connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
  LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
  HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
  
  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes)
static uint8 advertData[] =
{
  // Flags
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
  
  // 128-bit Service UUID - Nordic UART Service
  0x11,   // length of this data (16 bytes + 1 byte type)
  GAP_ADTYPE_128BIT_COMPLETE,
  0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
  0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = DEVICE_NAME;

// Connection handle
static uint16 gapConnHandle = INVALID_CONNHANDLE;

// BLE notification enabled flag
static uint8 notificationsEnabled = FALSE;

// Pending notification buffer (for retry logic)
static uint8 pendingNotification[BLE_MTU_SIZE];
static uint16 pendingNotificationLen = 0;
static uint8 notificationRetries = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void vescBLE_ProcessOSALMsg(osal_event_hdr_t *pMsg);
static void vescBLE_ProcessGATTMsg(gattMsgEvent_t *pMsg);
static void peripheralStateNotificationCB(gaprole_States_t newState);
static void vescBLE_HandleKeys(uint8 shift, uint8 keys);
static void vescBLE_ProcessUARTData(void);
static void vescBLE_UARTCallback(uint8 port, uint8 event);
static void vescBLE_SendNotification(uint8 *data, uint16 len);
static void vescBLE_PeriodicTask(void);

// GATT Service callbacks
static void vescBLE_RxDataCallback(uint8 *data, uint16 len);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t vescBLE_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller
};

// GAP Bond Manager Callbacks
static gapBondCBs_t vescBLE_BondMgrCBs =
{
  NULL,                     // Passcode callback
  NULL                      // Pairing state callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
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
void vescBLE_Init(uint8 task_id)
{
  vescBLE_TaskID = task_id;
  
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16), &desired_max_interval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16), &desired_slave_latency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16), &desired_conn_timeout);
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
  
  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    
    GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32), &passkey);
    GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8), &pairMode);
    GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8), &mitm);
    GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8), &ioCap);
    GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8), &bonding);
  }
  
  // Initialize GATT attributes
  GGS_AddService(GATT_ALL_SERVICES);           // GAP
  GATTServApp_AddService(GATT_ALL_SERVICES);   // GATT attributes
  
  // Add VESC service
  vescService_AddService();
  
  // Setup UART
  halUARTCfg_t uartConfig;
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = VESC_UART_BAUD;
  uartConfig.flowControl          = FALSE;
  uartConfig.flowControlThreshold = 0;
  uartConfig.rx.maxBufSize        = UART_RX_BUFFER_SIZE;
  uartConfig.tx.maxBufSize        = UART_TX_BUFFER_SIZE;
  uartConfig.idleTimeout          = 6;
  uartConfig.intEnable            = TRUE;
  uartConfig.callBackFunc         = vescBLE_UARTCallback;
  HalUARTOpen(VESC_UART_PORT, &uartConfig);
  
  // Register callback with VESC Service
  vescService_RegisterRxCallback(vescBLE_RxDataCallback);
  
  // Start the Device
  VOID GAPRole_StartDevice(&vescBLE_PeripheralCBs);
  
  // Start Bond Manager
  VOID GAPBondMgr_Register(&vescBLE_BondMgrCBs);
  
  // Start periodic event
  osal_start_timerEx(vescBLE_TaskID, VESC_PERIODIC_EVT, VESC_PERIODIC_EVT_PERIOD);
}

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
uint16 vescBLE_ProcessEvent(uint8 task_id, uint16 events)
{
  VOID task_id; // OSAL required parameter that isn't always used
  
  if (events & SYS_EVENT_MSG)
  {
    uint8 *pMsg;
    
    if ((pMsg = osal_msg_receive(vescBLE_TaskID)) != NULL)
    {
      vescBLE_ProcessOSALMsg((osal_event_hdr_t *)pMsg);
      
      // Release the OSAL message
      VOID osal_msg_deallocate(pMsg);
    }
    
    // Return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  if (events & VESC_START_DEVICE_EVT)
  {
    // Start the Device
    VOID GAPRole_StartDevice(&vescBLE_PeripheralCBs);
    
    return (events ^ VESC_START_DEVICE_EVT);
  }
  
  if (events & VESC_UART_RX_EVT)
  {
    // Process UART RX data
    vescBLE_ProcessUARTData();
    
    return (events ^ VESC_UART_RX_EVT);
  }
  
  if (events & VESC_UART_TX_EVT)
  {
    // Process UART TX data (currently unused, handled by callback)
    
    return (events ^ VESC_UART_TX_EVT);
  }
  
  if (events & VESC_PERIODIC_EVT)
  {
    // Periodic task for housekeeping
    vescBLE_PeriodicTask();
    
    // Restart timer
    osal_start_timerEx(vescBLE_TaskID, VESC_PERIODIC_EVT, VESC_PERIODIC_EVT_PERIOD);
    
    return (events ^ VESC_PERIODIC_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      vescBLE_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void vescBLE_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
  switch (pMsg->event)
  {
    case KEY_CHANGE:
      vescBLE_HandleKeys(((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys);
      break;
      
    case GATT_MSG_EVENT:
      vescBLE_ProcessGATTMsg((gattMsgEvent_t *)pMsg);
      break;
      
    default:
      // Do nothing
      break;
  }
}

/*********************************************************************
 * @fn      vescBLE_ProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void vescBLE_ProcessGATTMsg(gattMsgEvent_t *pMsg)
{
  // Handle GATT messages if needed
  // Can add MTU exchange handling here if needed
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB(gaprole_States_t newState)
{
  switch (newState)
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        
        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        
        // LED indication
        HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
      }
      break;
      
    case GAPROLE_ADVERTISING:
      {
        // LED indication - blink
        HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
      }
      break;
      
    case GAPROLE_CONNECTED:
      {
        // Get connection handle
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
        
        // LED indication - solid
        HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
        
        // Reset pending notification state
        pendingNotificationLen = 0;
        notificationRetries = 0;
      }
      break;
      
    case GAPROLE_WAITING:
      {
        // Disconnected - clear connection handle
        gapConnHandle = INVALID_CONNHANDLE;
        notificationsEnabled = FALSE;
        pendingNotificationLen = 0;
        
        // LED indication - off
        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
      }
      break;
      
    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        // Disconnected - clear connection handle
        gapConnHandle = INVALID_CONNHANDLE;
        notificationsEnabled = FALSE;
        pendingNotificationLen = 0;
        
        // LED indication - off
        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
      }
      break;
      
    case GAPROLE_ERROR:
      {
        // LED indication - error
        HalLedSet(HAL_LED_1, HAL_LED_MODE_FLASH);
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      vescBLE_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events.
 *
 * @return  none
 */
static void vescBLE_HandleKeys(uint8 shift, uint8 keys)
{
  // Handle key presses if needed
  // Could add functionality to clear bonding, etc.
}

/*********************************************************************
 * @fn      vescBLE_UARTCallback
 *
 * @brief   UART callback function
 *
 * @param   port - UART port
 * @param   event - UART event
 *
 * @return  none
 */
static void vescBLE_UARTCallback(uint8 port, uint8 event)
{
  if (event & (HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT))
  {
    // Trigger event to process received data
    osal_set_event(vescBLE_TaskID, VESC_UART_RX_EVT);
  }
}

/*********************************************************************
 * @fn      vescBLE_ProcessUARTData
 *
 * @brief   Process data received from UART
 *
 * @return  none
 */
static void vescBLE_ProcessUARTData(void)
{
  uint8 buffer[BLE_MTU_SIZE];
  uint16 numBytes = 0;
  
  // Only process if connected and notifications enabled
  if (gapConnHandle == INVALID_CONNHANDLE || !notificationsEnabled)
  {
    return;
  }
  
  // Read available data from UART (up to MTU size)
  numBytes = HalUARTRead(VESC_UART_PORT, buffer, BLE_MTU_SIZE);
  
  if (numBytes > 0)
  {
    // Send data over BLE
    vescBLE_SendNotification(buffer, numBytes);
    
    // Check if more data is available
    if (Hal_UART_RxBufLen(VESC_UART_PORT) > 0)
    {
      // Trigger another read
      osal_set_event(vescBLE_TaskID, VESC_UART_RX_EVT);
    }
  }
}

/*********************************************************************
 * @fn      vescBLE_RxDataCallback
 *
 * @brief   Callback from VESC Service when data is received from BLE
 *
 * @param   data - pointer to received data
 * @param   len - length of received data
 *
 * @return  none
 */
static void vescBLE_RxDataCallback(uint8 *data, uint16 len)
{
  // Send received BLE data to UART
  HalUARTWrite(VESC_UART_PORT, data, len);
}

/*********************************************************************
 * @fn      vescBLE_SendNotification
 *
 * @brief   Send notification to BLE central device
 *
 * @param   data - pointer to data to send
 * @param   len - length of data
 *
 * @return  none
 */
static void vescBLE_SendNotification(uint8 *data, uint16 len)
{
  bStatus_t status;
  
  // Check if we have a pending notification
  if (pendingNotificationLen > 0)
  {
    // Already have pending data, skip this one
    // This prevents buffer overflow
    return;
  }
  
  // Send notification via VESC Service
  status = vescService_SendNotification(gapConnHandle, data, len);
  
  if (status != SUCCESS)
  {
    // Store for retry
    if (len <= BLE_MTU_SIZE)
    {
      osal_memcpy(pendingNotification, data, len);
      pendingNotificationLen = len;
      notificationRetries = 0;
    }
  }
}

/*********************************************************************
 * @fn      vescBLE_PeriodicTask
 *
 * @brief   Periodic housekeeping task
 *
 * @return  none
 */
static void vescBLE_PeriodicTask(void)
{
  // Retry pending notification if any
  if (pendingNotificationLen > 0 && gapConnHandle != INVALID_CONNHANDLE && notificationsEnabled)
  {
    bStatus_t status = vescService_SendNotification(gapConnHandle, pendingNotification, pendingNotificationLen);
    
    if (status == SUCCESS)
    {
      // Clear pending notification
      pendingNotificationLen = 0;
      notificationRetries = 0;
    }
    else
    {
      notificationRetries++;
      
      if (notificationRetries >= MAX_NOTIFICATION_RETRIES)
      {
        // Give up after max retries
        pendingNotificationLen = 0;
        notificationRetries = 0;
      }
    }
  }
  
  // Check if there's pending UART data to process
  if (Hal_UART_RxBufLen(VESC_UART_PORT) > 0)
  {
    osal_set_event(vescBLE_TaskID, VESC_UART_RX_EVT);
  }
}

/*********************************************************************
 * @fn      vescBLE_SetNotificationsEnabled
 *
 * @brief   Enable/disable notifications (called by VESC Service)
 *
 * @param   enabled - TRUE to enable, FALSE to disable
 *
 * @return  none
 */
void vescBLE_SetNotificationsEnabled(uint8 enabled)
{
  notificationsEnabled = enabled;
  
  // Clear any pending notifications when disabled
  if (!enabled)
  {
    pendingNotificationLen = 0;
    notificationRetries = 0;
  }
}

/*********************************************************************
*********************************************************************/