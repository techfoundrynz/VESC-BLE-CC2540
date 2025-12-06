/**
 * @file    OSAL_vesc_ble.c
 * @brief   OSAL task setup for VESC BLE UART Bridge
 */

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "OSAL_Tasks.h"

// HAL
#include "hal_types.h"
#include "hal_drivers.h"

// BLE Stack
#include "hci_tl.h"

#if defined ( OSAL_CBTIMER_NUM_TASKS )
  #include "osal_cbtimer.h"
#endif

// L2CAP
#include "l2cap.h"

// GAP
#include "gap.h"
#include "gapgattserver.h"
#include "gapbondmgr.h"

// Peripheral Role - 
#include "peripheral.h"

// GATT
#include "gatt.h"
#include "gattservapp.h"

// Application
#include "vesc_ble.h"

/*********************************************************************
 * GLOBAL VARIABLES
 */


const pTaskEventHandlerFn tasksArr[] =
{
    LL_ProcessEvent,                                  // task 0
    Hal_ProcessEvent,                                 // task 1
    HCI_ProcessEvent,                                 // task 2
#if defined ( OSAL_CBTIMER_NUM_TASKS )
    OSAL_CBTIMER_PROCESS_EVENT( osal_CbTimerProcessEvent ),  // task 3
#endif
    L2CAP_ProcessEvent,                               // task 4
    GAP_ProcessEvent,                                 // task 5
    SM_ProcessEvent,                                  // task 6
    GATT_ProcessEvent,                                // task 7
    GAPRole_ProcessEvent,                             // task 8
    GAPBondMgr_ProcessEvent,                          // task 9
    GATTServApp_ProcessEvent,                         // task 10
    VESC_BLE_ProcessEvent                             // task 11
};

const uint8 tasksCnt = sizeof(tasksArr) / sizeof(tasksArr[0]);
uint16 *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 */

/**
 * @fn      osalInitTasks
 * @brief   This function invokes the initialization function for each task.
 * @return  none
 */
void osalInitTasks(void)
{
    uint8 taskID = 0;

    tasksEvents = (uint16 *)osal_mem_alloc(sizeof(uint16) * tasksCnt);
    osal_memset(tasksEvents, 0, (sizeof(uint16) * tasksCnt));

    // LL Task
    LL_Init(taskID++);

    // HAL Task
    Hal_Init(taskID++);

    // HCI Task
    HCI_Init(taskID++);

#if defined ( OSAL_CBTIMER_NUM_TASKS )
    // Callback Timer Task
    osal_CbTimerInit(taskID);
    taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

    // L2CAP Task
    L2CAP_Init(taskID++);

    // GAP Task
    GAP_Init(taskID++);

    // SM Task
    SM_Init(taskID++);

    // GATT Task
    GATT_Init(taskID++);

    // GAP Role Task (Peripheral)
    GAPRole_Init(taskID++);

    // GAP Bond Manager Task
    GAPBondMgr_Init(taskID++);

    // GATT Server App Task
    GATTServApp_Init(taskID++);

    // VESC BLE Application Task
    VESC_BLE_Init(taskID);
}

/*********************************************************************
*********************************************************************/
