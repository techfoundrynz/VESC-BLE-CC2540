/**
 * @file    vesc_ble.c
 * 
 * Uses Software UART on P1.6 (TX) and P1.7 (RX)
 */

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "gatt.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "peripheral.h"
#include "gapbondmgr.h"

#include "vesc_ble.h"
#include "nus_service.h"
#include "soft_uart.h"

/*********************************************************************
 * CONSTANTS
 */

#define DEVICE_NAME                     "VESC BLE UART"
#define DEFAULT_ADVERTISING_INTERVAL    64
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     8
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     16
#define DEFAULT_DESIRED_SLAVE_LATENCY         0
#define DEFAULT_DESIRED_CONN_TIMEOUT          100
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define VESC_BLE_PERIODIC_EVT_PERIOD    5   // 5ms polling

#define BLE_MAX_DATA_LEN                20
#define UART_RX_BUF_SIZE                128

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 vesc_ble_TaskID;

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 scanRspData[] =
{
    14,
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'V', 'E', 'S', 'C', ' ', 'B', 'L', 'E', ' ', 'U', 'A', 'R', 'T',
    
    0x05,
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
    
    0x02,
    GAP_ADTYPE_POWER_LEVEL,
    0
};

static uint8 advertData[] =
{
    0x02,
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    
    0x11,
    GAP_ADTYPE_128BIT_MORE,
    NUS_SERVICE_UUID_BYTES
};

static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "VESC BLE UART";

static uint8 uartRxBuf[UART_RX_BUF_SIZE];

static bool bleConnected = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void vesc_ble_ProcessOSALMsg(osal_event_hdr_t *pMsg);
static void peripheralStateNotificationCB(gaprole_States_t newState);
static void nusServiceChangeCB(uint8 paramID);
static void softUartCallback(void);

/*********************************************************************
 * PROFILE CALLBACKS
 */

static gapRolesCBs_t vesc_ble_PeripheralCBs =
{
    peripheralStateNotificationCB,
    NULL
};

static gapBondCBs_t vesc_ble_BondMgrCBs =
{
    NULL,
    NULL
};

static nusServiceCBs_t nusServiceCBs =
{
    nusServiceChangeCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

void VESC_BLE_Init(uint8 task_id)
{
    vesc_ble_TaskID = task_id;
    
    VOID GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);
    
    {
        uint8 initial_advertising_enable = TRUE;
        uint16 gapRole_AdvertOffTime = 0;
        uint8 enable_update_request = TRUE;
        uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
        
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &initial_advertising_enable);
        GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16), &gapRole_AdvertOffTime);
        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData), scanRspData);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
        GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8), &enable_update_request);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16), &desired_min_interval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16), &desired_max_interval);
        GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16), &desired_slave_latency);
        GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16), &desired_conn_timeout);
    }
    
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);
    
    {
        uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    }
    
    {
        uint32 passkey = 0;
        uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8 mitm = FALSE;
        uint8 ioCap = GAPBOND_IO_CAP_NO_INPUT_NO_OUTPUT;
        uint8 bonding = TRUE;
        
        GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32), &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8), &bonding);
    }
    
    GGS_AddService(GATT_ALL_SERVICES);
    GATTServApp_AddService(GATT_ALL_SERVICES);
    DevInfo_AddService();
    NUS_AddService(GATT_ALL_SERVICES);
    
    NUS_RegisterAppCBs(&nusServiceCBs);
    
    // Initialize Software UART on P1.6/P1.7
    SoftUART_Init(softUartCallback);
    
    //HCI_EXT_ClkDivOnHaltCmd(HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT);
    
    osal_set_event(vesc_ble_TaskID, VESC_BLE_START_DEVICE_EVT);
}

uint16 VESC_BLE_ProcessEvent(uint8 task_id, uint16 events)
{
    VOID task_id;
    
    if (events & SYS_EVENT_MSG)
    {
        uint8 *pMsg;
        
        if ((pMsg = osal_msg_receive(vesc_ble_TaskID)) != NULL)
        {
            vesc_ble_ProcessOSALMsg((osal_event_hdr_t *)pMsg);
            VOID osal_msg_deallocate(pMsg);
        }
        
        return (events ^ SYS_EVENT_MSG);
    }
    
    if (events & VESC_BLE_START_DEVICE_EVT)
    {
        VOID GAPRole_StartDevice(&vesc_ble_PeripheralCBs);
        VOID GAPBondMgr_Register(&vesc_ble_BondMgrCBs);
        
        // Start periodic polling - DISABLED for Hardware UART (Event Driven)
        // osal_start_timerEx(vesc_ble_TaskID, VESC_BLE_PERIODIC_EVT, VESC_BLE_PERIODIC_EVT_PERIOD);
        
        return (events ^ VESC_BLE_START_DEVICE_EVT);
    }
    
    if (events & VESC_BLE_PERIODIC_EVT)
    {
        // Poll software UART for incoming data
        SoftUART_Poll();
        
        // Check
        uint8 len = SoftUART_RxBufLen();
        if (bleConnected && len > 0)
        {
            if (len > BLE_MAX_DATA_LEN)
                len = BLE_MAX_DATA_LEN;
            
            uint8 readLen = SoftUART_Read(uartRxBuf, len);
            if (readLen > 0)
            {
                NUS_SetParameter(NUS_TX_DATA, readLen, uartRxBuf);
            }
        }
        
        // Restart timer
        osal_start_timerEx(vesc_ble_TaskID, VESC_BLE_PERIODIC_EVT, VESC_BLE_PERIODIC_EVT_PERIOD);
        
        return (events ^ VESC_BLE_PERIODIC_EVT);
    }
    
    if (events & VESC_BLE_UART_RX_EVT)
    {
        // Process any pending UART data
        uint8 len = SoftUART_RxBufLen();
        if (bleConnected && len > 0)
        {
            if (len > BLE_MAX_DATA_LEN)
                len = BLE_MAX_DATA_LEN;
            
            uint8 readLen = SoftUART_Read(uartRxBuf, len);
            if (readLen > 0)
            {
                NUS_SetParameter(NUS_TX_DATA, readLen, uartRxBuf);
            }
            
            // Check if there is more data remaining
            if (SoftUART_RxBufLen() > 0)
            {
                // Re-schedule event immediately to process next chunk
                osal_set_event(vesc_ble_TaskID, VESC_BLE_UART_RX_EVT);
            }
        }
        
        return (events ^ VESC_BLE_UART_RX_EVT);
    }
    
    return 0;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void vesc_ble_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
    switch (pMsg->event)
    {
        default:
            break;
    }
}

static void peripheralStateNotificationCB(gaprole_States_t newState)
{
    switch (newState)
    {
        case GAPROLE_STARTED:
            break;
            
        case GAPROLE_ADVERTISING:
            bleConnected = FALSE;
            break;
            
        case GAPROLE_CONNECTED:
            bleConnected = TRUE;
            break;
            
        case GAPROLE_WAITING:
        case GAPROLE_WAITING_AFTER_TIMEOUT:
            bleConnected = FALSE;
            {
                uint8 advertEnabled = TRUE;
                GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &advertEnabled);
            }
            break;
            
        case GAPROLE_ERROR:
            break;
            
        default:
            break;
    }
}

static void nusServiceChangeCB(uint8 paramID)
{
    uint8 data[BLE_MAX_DATA_LEN];
    uint8 len = 0;
    
    switch (paramID)
    {
        case NUS_RX_DATA:
            NUS_GetParameter(NUS_RX_DATA, &len, data);
            if (len > 0)
            {
                // Send data to VESC via software UART
                SoftUART_Write(data, len);
            }
            break;
            
        default:
            break;
    }
}

static void softUartCallback(void)
{
    // Trigger event to process UART data
    osal_set_event(vesc_ble_TaskID, VESC_BLE_UART_RX_EVT);
}