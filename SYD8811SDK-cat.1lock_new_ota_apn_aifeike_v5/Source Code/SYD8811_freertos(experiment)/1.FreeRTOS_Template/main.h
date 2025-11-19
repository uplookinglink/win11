#ifndef __MAIN_H
#define __MAIN_H

#include "ARMCM0.h"
/* freeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"

typedef enum
{
	KEY_QUE_HDL,
	SOFTTIMER_QUE_HDL,
	UART_QUE_HDL,
	QUE_MSG_MAX,
}queue_msg_hdl_t;

typedef enum
{
	CONNECTION_LATENCY_MSG_ID_EVT,
	OTA_MANAGE_MSG_ID_EVT,
	GREEN_LED_MSG_ID_EVT,
	CAT1_HANDLE_MSG_ID_EVT,
	MOTOR_TIMEOUT_MSG_ID_EVT,
	TASK_APP_SLEEP_MSG_ID_EVT,
	TASK_180S_MSG_ID_EVT,
	TASK_PERIOD_MSG_ID_EVT,
	TASK_UPDATE_MSG_ID_EVT,
	AT_RESET_SLEEP_MSG_ID_EVT,
	BLE_CONNECT_TIMEOUT_MSG_ID_EVT,
		
	KEY_NB_MSG_ID_EVT,
	KEY_BLUETOOTH_MSG_ID_EVT,	
  KEY_MOTOR_CHECK_MSG_ID_EVT,
	KEY_POWER_DOWN_MSG_ID_EVT,
	MOTOR_OPEN_MSG_ID_EVT,
	MOTOR_CLOSE_MSG_ID_EVT,
	LOCK_HALL_MSG_ID_EVT,
  BATTERY_DETECT_MSG_ID_EVT,
	LOCK_TIGHTEND_MSG_ID_EVT,
	LOCK_NOT_TIGHTEND_MSG_ID_EVT,
	WATCHDOG_MSG_ID_EVT,
	BLUETOOTH_TIMEOUT_MSG_ID_EVT,
	
	UART0_REV_MSG_ID_EVT,
	UART0_NB_DELAY_MSG_ID_EVT,
	UART0_NB_REPEAT_MSG_ID_EVT,
	UART0_NB_REV_TIMEOUT_MSG_ID_EVT,
	UART0_NB_ERROR_MSG_ID_EVT,
	UART0_NB_POWERDOWN_MSG_ID_EVT,
	UART0_UPDATE_MSG_ID_EVT,
	
	UART1_REV_MSG_ID_EVT,
}msg_id_t;

typedef struct
{
   msg_id_t msg_id;
   uint16_t param1;
   void *param2;
}queue_msg_t;

void vTaskSendQueueEvt(queue_msg_hdl_t hdl, msg_id_t msg_id, uint16_t param, void *context);
extern xQueueHandle queue_msg_hal_array[QUE_MSG_MAX];
void gpio_pin_sleep(uint8_t mode);
void tasklist_monitor(void);
#endif
