/**
 * CC2540 VESC BLE UART Bridge Firmware
 * UART Configuration: 115200 baud, 8N1
 * 
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
#define VESC_UART_PORT                HAL_UART_PORT_1

// BLE Connection parameters - optimized for VESC
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     24    // 30ms (units of 1.25ms)
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     40    // 50ms (units of 1.25ms)
#define DEFAULT_DESIRED_SLAVE_LATENCY         0
#define DEFAULT_DESIRED_CONN_TIMEOUT          200   // 2s (units of 10ms)

// BLE Advertising parameters
#define DEFAULT_ADVERTISING_INTERVAL          160   // 100ms (units of 0.625ms) 
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ
#define DEFAULT_MITM_MODE                     FALSE
#define DEFAULT_BONDING_MODE                  TRUE
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT

// Connection Pause Peripheral time value (in seconds) 
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

// UART Buffer sizes
#define UART_RX_BUFFER_SIZE                   128
#define UART_TX_BUFFER_SIZE                   128

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

// Current GAP state
static gaprole_States_t gapProfileState = GAPROLE_INIT;

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
  // Flags - Use GENERAL discoverable mode (advertises indefinitely)
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
  
  // CRITICAL: Disable power management to prevent sleep
  osal_pwrmgr_device(PWRMGR_ALWAYS_ON);
  
  // Setup the GAP 
  VOID GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);
  
  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;
    
    uint16 gapRole_AdvertOffTime = 0;
    
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
    
    // Set the GAP Role Parameters
    GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &initial_advertising_enable);
    GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16), &gapRole_AdvertOffTime);
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
    GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
    GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8), &enable_update_request);
    GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &desired_min_interval);
    GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16), &desired_max_interval);
    GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16), &desired_slave_latency);
    GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16), &desired_conn_timeout);
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
  
  // Set advertising interval 
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
    
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
    GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
  }
  
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
  bStatus_t status = vescService_AddService();

  // DEBUG: Check if service was added successfully
  if (status == SUCCESS)
  {
    // Blink LED 3 times fast to indicate success
    for (uint8 i = 0; i < 3; i++)
    {
      HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
      for(volatile uint32 j = 0; j < 50000; j++);
      HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
      for(volatile uint32 j = 0; j < 50000; j++);
    }
  }
  else
  {
    // Blink LED slowly 5 times to indicate failure
    for (uint8 i = 0; i < 5; i++)
    {
      HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
      for(volatile uint32 j = 0; j < 200000; j++);
      HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
      for(volatile uint32 j = 0; j < 200000; j++);
    }
  }
  // Configure UART1 for Alternative 2 => P1.6=TX, P1.7=RX
PERCFG |= 0x02;        // UART1 Alt 2
P1SEL |= 0xC0;         // P1.6 and P1.7 as peripheral
P1DIR |= 0x40;         // P1.6 (TX) as output
P1DIR &= ~0x80;        // P1.7 (RX) as input
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
  
  // Setup a delayed profile startup (from TI reference)
  osal_set_event(vescBLE_TaskID, VESC_START_DEVICE_EVT);
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
    
    // Start Bond Manager
    VOID GAPBondMgr_Register(&vescBLE_BondMgrCBs);
    
    // Set timer for first periodic event
    osal_start_timerEx(vescBLE_TaskID, VESC_PERIODIC_EVT, VESC_PERIODIC_EVT_PERIOD);
    
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
    // Restart timer
    if (VESC_PERIODIC_EVT_PERIOD)
    {
      osal_start_timerEx(vescBLE_TaskID, VESC_PERIODIC_EVT, VESC_PERIODIC_EVT_PERIOD);
    }
    
    // Periodic task for housekeeping
    vescBLE_PeriodicTask();
    
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
  // Free GATT message (from TI reference)
  GATT_bm_free(&pMsg->msg, pMsg->method);
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *          Following TI reference pattern for advertising restart.
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
        
        // LED off at start
        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
      }
      break;
      
    case GAPROLE_ADVERTISING:
      {
        // Advertising - LED blinking
        HalLedSet(HAL_LED_1, HAL_LED_MODE_FLASH);
      }
      break;
      
    case GAPROLE_CONNECTED:
      {
        GAPRole_GetParameter(GAPROLE_CONNHANDLE, &gapConnHandle);
        
        // Connected - LED solid ON
        HalLedSet(HAL_LED_1, HAL_LED_MODE_ON);
        
        // Reset pending notification state
        pendingNotificationLen = 0;
        notificationRetries = 0;
      }
      break;
      
    case GAPROLE_WAITING:
      {
        // Disconnected - clean up state
        gapConnHandle = INVALID_CONNHANDLE;
        notificationsEnabled = FALSE;
        pendingNotificationLen = 0;
        notificationRetries = 0;
        
        // LED off while waiting
        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
        
        
        uint8 advertEnabled = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advertEnabled);
      }
      break;
      
    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        // Connection timed out - clean up state
        gapConnHandle = INVALID_CONNHANDLE;
        notificationsEnabled = FALSE;
        pendingNotificationLen = 0;
        notificationRetries = 0;
        
        // LED off
        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
        

        uint8 advertEnabled = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advertEnabled);
      }
      break;
      
    case GAPROLE_ERROR:
      {
        // Error - LED off
        HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF);
        
        // Try to recover by re-enabling advertising
        uint8 advertEnabled = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advertEnabled);
      }
      break;
      
    default:
      break;
  }
  
  // Save current state
  gapProfileState = newState;
  
  VOID gapProfileState;  
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
  VOID shift; 
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
    // Drain the buffer even if not connected to prevent overflow
    while (Hal_UART_RxBufLen(VESC_UART_PORT) > 0)
    {
      HalUARTRead(VESC_UART_PORT, buffer, BLE_MTU_SIZE);
    }
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