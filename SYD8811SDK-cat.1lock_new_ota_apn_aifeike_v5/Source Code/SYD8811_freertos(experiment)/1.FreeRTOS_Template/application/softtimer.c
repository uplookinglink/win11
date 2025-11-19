#include "..\application\softtimer.h"
#include "main.h"
#include "debug.h"
#include "gpio.h"
#include "led_key.h"
#include "lock.h"
#include "Cat1.h"

TimerHandle_t MOTOR_TIMEOUT_TIMER_EVT_ID;       //马达转动超时		
TimerHandle_t GREEN_LED_TIMER_EVT_ID;           //绿灯闪烁
TimerHandle_t BLUE_LED_TIMER_EVT_ID;            //蓝灯闪烁
TimerHandle_t KEY_DEBOUNCE_TIMER_EVT_ID;        //按键消抖
TimerHandle_t BAT_DET_DEBOUNCE_TIMER_EVT_ID;    //按键消抖
TimerHandle_t LOCK_DEBOUNCE_TIMER_EVT_ID;       //关锁检测、开锁成功检测、关锁成功检测、剪断检测消抖
TimerHandle_t NB_TIMEOUT_TIMER_EVT_ID;          //Cat.1发送AT命令超时
TimerHandle_t NB_CMD_INTERVAL_TIMER_EVT_ID;     //Cat.1发送AT命令间隔
TimerHandle_t NB_ERROR_TIMER_EVT_ID;            //Cat.1出错灯闪
TimerHandle_t KEY_HANDLE_TIMER_EVT_ID;          //按键长短按判断
TimerHandle_t NB_DELAY_TIMER_EVT_ID;            //发送AT命令延时
TimerHandle_t LED_ONE_SECOND_TIMER_EVT_ID;      //LED灯亮1s
TimerHandle_t SLEEP_TIMER_EVT_ID;               //低功耗倒计时
TimerHandle_t HALL_ALARM_PROTECT_TIMER_EVT_ID;       
TimerHandle_t HALL_ALARM_DETECT_TIMER_EVT_ID; 
TimerHandle_t BLE_CONNECT_TIMEOUT_TIMER_EVT_ID; //手机连接蓝牙超时
TimerHandle_t HALL_TIGHTEND_TIMER_EVT_ID;
TimerHandle_t OPEN_LOCK_TIMEOUT_ROTATION_TIMER_EVT_ID;

struct SYD_sysTimer syd_sysTimer[EVT_NUM] = {0};
struct SYD_sysRtc syd_rtc[RTCEVT_NUM] = {0};
struct SYD_HISTORY_SETTING  syd_history_setting_str;

extern lock_info_t lock_info;
extern lock_irq_t lock_irq;
extern flash_data_t flash_data;
extern queue_t *pcat1_handle;

/************************************************************************************************************************
* 函数名称: connection_latency_timer_evt_irq_handle()
* 功能说明: 定时器中断事件处理
* 输 入: 无
* 输 出: 无
* 注意事项: 无
************************************************************************************************************************/
void connection_latency_timer_evt_irq_handle(void)
{
	vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, CONNECTION_LATENCY_MSG_ID_EVT, NULL, NULL);
}

/************************************************************************************************************************
* 函数名称: ota_manage_timer_evt_irq_handle()
* 功能说明: 定时器中断事件处理
* 输 入: 无
* 输 出: 无
* 注意事项: 无
************************************************************************************************************************/
static void ota_manage_timer_evt_irq_handle(void)
{
	vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, OTA_MANAGE_MSG_ID_EVT, NULL, NULL);
}

/************************************************************************************************************************
* 函数名称: green_led_timer_evt_irq_handle()   
* 功能说明: 定时器中断事件处理
* 输 入: 无
* 输 出: 无
* 注意事项: 无
************************************************************************************************************************/
static void green_led_timer_evt_irq_handle(void)
{
//	vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, GREEN_LED_MSG_ID_EVT, NULL, NULL);
	GPIO_Pin_Turn(U32BIT(GREEN_LED));
}

/************************************************************************************************************************
* 函数名称: blue_led_timer_evt_irq_handle()
* 功能说明: 定时器中断事件处理
* 输 入: 无
* 输 出: 无
* 注意事项: 无
************************************************************************************************************************/
static void blue_led_timer_evt_irq_handle(void)
{
//	vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, BLUE_LED_MSG_ID_EVT, NULL, NULL);
	GPIO_Pin_Turn(U32BIT(BLUE_LED));
}

/************************************************************************************************************************
* 函数名称: motor_timeout_timer_evt_irq_handle()
* 功能说明: 定时器中断事件处理
* 输 入: 无
* 输 出: 无
* 注意事项: 无
************************************************************************************************************************/
//目前的马达止转开关检测，开锁检测不到，但是关锁能检测到
static void motor_timeout_timer_evt_irq_handle(void) //20ms定时器
{
	static uint16_t cnt = 0;
	cnt++;
	if (cnt >= 50)
	{
		cnt = 0;
		xTimerStop(MOTOR_TIMEOUT_TIMER_EVT_ID, 0);
		uint32_t i1 = GPIO_Pin_Read(U32BIT(MOTOR_STATUS_OPEN));
		uint32_t i2 = GPIO_Pin_Read(U32BIT(MOTOR_STATUS_CLOSE));
		DBGPRINTF("motor timeout timer open = %d close = %d\r\n",i1,i2);
		
		lock_info.motor_status = MOTOR_NON;
		GPIO_Pin_Clear(U32BIT(MotorCtrl_1));
		GPIO_Pin_Clear(U32BIT(MotorCtrl_2)); 
		GPIO_Pin_Set(U32BIT(BLUE_LED));
		if (lock_info.motor_status == MOTOR_CLOSE)
		{
			//关锁超时
			vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, MOTOR_TIMEOUT_MSG_ID_EVT, NULL, NULL);   			
		}
	}
	if(GPIO_Pin_Read(U32BIT(MOTOR_STATUS_CLOSE)) && (lock_info.motor_status == MOTOR_CLOSE)) //马达位置触发，同时检测到关锁
	{
		lock_info.motor_status = MOTOR_NON;
		xTimerStop(MOTOR_TIMEOUT_TIMER_EVT_ID, 0);
		//第一时间停马达
    GAPBBDelayMS(20); //检测到位后，马达多转50ms
		GPIO_Pin_Clear(U32BIT(MotorCtrl_1));
		GPIO_Pin_Clear(U32BIT(MotorCtrl_2)); 		
		DBGPRINTF("motor close timer=%d %d\r\n", cnt, lock_info.sleep_flag); //一般cnt=13左右
		cnt = 0;
	}
	if (GPIO_Pin_Read(U32BIT(MOTOR_STATUS_OPEN)) && (lock_info.motor_status == MOTOR_OPEN))
	{
		lock_info.motor_status = MOTOR_NON;
		xTimerStop(MOTOR_TIMEOUT_TIMER_EVT_ID, 0);
		GPIO_Pin_Clear(U32BIT(MotorCtrl_1));
		GPIO_Pin_Clear(U32BIT(MotorCtrl_2)); 
		DBGPRINTF("motor open timer=%d %d\r\n", cnt, lock_info.sleep_flag);
		cnt = 0;		
	}
	else
	{
		//do nothing
	}	
}

//锁绳拉紧，检测16s
void hall_tightend_timer_evt_irq_handle(void)
{
	lock_info.hall_cnt++;
	if (lock_info.hall_flags == 0)
	{
		if(!GPIO_Pin_Read(U32BIT(HALL_DET)))
		{
			DBGPRINTF("hall_tightend_timer_evt_irq_handle %d\r\n", lock_info.hall_cnt);
			lock_info.hall_flags = 1;
			if (lock_info.ble_working == 0)
			{
				xTimerStop(BLUE_LED_TIMER_EVT_ID, 0);
				GPIO_Pin_Set(U32BIT(BLUE_LED));
			}
			else {
				gatt_send_data(TENSION_CMD);
				xTimerChangePeriod(BLUE_LED_TIMER_EVT_ID, 2000, 0);	
			}
			if (lock_info.tightend_send_flags == 0)
			{
				lock_info.tightend_send_flags = 1;
				vTaskSendQueueEvt(KEY_QUE_HDL, LOCK_TIGHTEND_MSG_ID_EVT, 0xBB, NULL);
			}
		}
		else {
			if (xTimerIsTimerActive(BLUE_LED_TIMER_EVT_ID) == pdFALSE)
			{
				xTimerStart(BLUE_LED_TIMER_EVT_ID, 0);
			}
		}
	}
	else 
	{
		if(GPIO_Pin_Read(U32BIT(HALL_DET)))
		{
			lock_info.hall_flags = 0;
			xTimerChangePeriod(BLUE_LED_TIMER_EVT_ID, 100, 0);
			xTimerStart(BLUE_LED_TIMER_EVT_ID, 0);
		}
	}
	if (lock_info.hall_cnt >= 14)
	{
		lock_info.hall_cnt = 0;
		lock_info.hall_flags = 0;
		if (lock_info.ble_working == 0)
		{
			xTimerStop(BLUE_LED_TIMER_EVT_ID, 0);
			GPIO_Pin_Set(U32BIT(BLUE_LED));
		}
		else {
			xTimerChangePeriod(BLUE_LED_TIMER_EVT_ID, 2000, 0);	
		}
		xTimerStop(HALL_TIGHTEND_TIMER_EVT_ID, 0);
		//未拉紧状态4G上报
		if (GPIO_Pin_Read(U32BIT(HALL_DET)))
		{
			DBGPRINTF("hall_tightend is close\r\n");
			vTaskSendQueueEvt(KEY_QUE_HDL, LOCK_NOT_TIGHTEND_MSG_ID_EVT, 0xBB, NULL);
		}
	}
}

void hall_alarm_protect_timer_evt_irq_handle(void)
{
	DBGPRINTF("hall alarm protect timer\r\n");	
	return;
}
static uint8_t alarm_cnt = 0;
void hall_alarm_detect_timer_evt_irq_handle(void)
{
	if (GPIO_Pin_Read(U32BIT(LOCK_STATUS)) && (GPIO_Pin_Read(U32BIT(MOTOR_STATUS_CLOSE))))
	{
		alarm_cnt++;
		if (alarm_cnt >= 5)
		{
			alarm_cnt = 0;
			xTimerStop(HALL_ALARM_DETECT_TIMER_EVT_ID, 0);
			vTaskSendQueueEvt(KEY_QUE_HDL, LOCK_HALL_MSG_ID_EVT, 0xBB, NULL);
		}
	}
	else {
		alarm_cnt = 0;
		xTimerStop(HALL_ALARM_DETECT_TIMER_EVT_ID, 0);
		DBGPRINTF("lock pick timer sleep alarm_cnt=%d\r\n", alarm_cnt);
		lock_info.sleep_flag = 1;
	}
	return;
}

void ble_connect_timeout_evt_irq_handle(void)
{
	vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, BLE_CONNECT_TIMEOUT_MSG_ID_EVT, NULL, NULL);
}

//马达回转
void open_lock_rotation_timeout_evt_irq_handle(void)
{
		lock_info.motor_status = MOTOR_CLOSE;
		xTimerStart(MOTOR_TIMEOUT_TIMER_EVT_ID, 0);						
		motor_reverse();
		GPIO_Pin_Clear(U32BIT(BLUE_LED));		
}
/************************************************************************************************************************
* 函数名称: app_sleep_timer_evt_irq_handle()
* 功能说明: 定时器中断事件处理
* 输 入: 无
* 输 出: 无
* 注意事项: 无
************************************************************************************************************************/
void app_sleep_timer_evt_irq_handle(void)
{
	vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, TASK_APP_SLEEP_MSG_ID_EVT, NULL, NULL);
}

static void watchdog_timer_evt_irq_handle(void)
{
	vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, WATCHDOG_MSG_ID_EVT, NULL, NULL);
}

static void key_debounce_timer_evt_irq_handle(void)
{	
	if(GPIO_Pin_Read(U32BIT(WakeUp_KEY)))
  {
		 if (xTimerIsTimerActive(GREEN_LED_TIMER_EVT_ID))
     { 
				xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);
				GPIO_Pin_Set(U32BIT(GREEN_LED));
     }
		 if (xTimerIsTimerActive(BLUE_LED_TIMER_EVT_ID))
     {
				xTimerStop(BLUE_LED_TIMER_EVT_ID, 0);
        GPIO_Pin_Set(U32BIT(BLUE_LED));        
     }
		 GPIO_Pin_Clear(U32BIT(BLUE_LED));
     xTimerReset(KEY_HANDLE_TIMER_EVT_ID, 0); //400ms检测一次按键状态
     lock_info.key_cnt = 0;
  }
	DBGPRINTF("key pull down\r\n");
}

static void battery_detect_debounce_timer_evt_irq_handle(void)
{
  if (GPIO_Pin_Read(U32BIT(BAT_DET)))
  {
    vTaskSendQueueEvt(KEY_QUE_HDL, BATTERY_DETECT_MSG_ID_EVT, 0xBB, NULL);
		DBGPRINTF("battery detect timer\r\n");	
  }
	else {
		//do nothing
    //误触发处理
		if ((lock_info.ble_working == 0) && (lock_info.nb_flag == 1) && (lock_info.cat1_working == 0))
			lock_info.sleep_flag = 1;
		DBGPRINTF("battery detect nothing timer\r\n");
	}
	memset(&lock_irq, 0, sizeof(lock_irq_t));
}

static void lock_debounce_timer_evt_irq_handle(void)
{
	if(GPIO_Pin_Read(U32BIT(LOCK_STATUS)) && (lock_info.unlock_flag == 0))
	{
		if ((GPIO_Pin_Read(U32BIT(MOTOR_STATUS_CLOSE))) || (lock_info.open_flags == 0))
		{
			DBGPRINTF("lock_info.motor_status == MOTOR_CLOSE\r\n");
			//关锁状态检测到中断，判断为剪断告警
			if (xTimerIsTimerActive(HALL_ALARM_PROTECT_TIMER_EVT_ID) == pdFALSE)
			{
//				vTaskSendQueueEvt(KEY_QUE_HDL, LOCK_HALL_MSG_ID_EVT, 0xBB, NULL);
				DBGPRINTF("lock pick timer1\r\n");	
				xTimerStart(HALL_ALARM_DETECT_TIMER_EVT_ID, 0);
			}
			else{
				DBGPRINTF("lock pick timer is error\r\n");	
			}
		}
		else{
			//开锁检测
			if (xTimerIsTimerActive(HALL_ALARM_DETECT_TIMER_EVT_ID) == pdTRUE)
			{
				alarm_cnt = 0;
				xTimerStop(HALL_ALARM_DETECT_TIMER_EVT_ID, 0);
			}
			vTaskSendQueueEvt(KEY_QUE_HDL, MOTOR_OPEN_MSG_ID_EVT, 0xBB, NULL); 
			DBGPRINTF("lock open timer\r\n"); 
		}
	}	
	//关锁检测  && (GPIO_Pin_Read(U32BIT(MOTOR_STATUS_CLOSE)
	else if (GPIO_Pin_Read(U32BIT(LOCK_STATUS)) && (lock_info.unlock_flag == 1))
	{
		if (xTimerIsTimerActive(HALL_ALARM_DETECT_TIMER_EVT_ID) == pdTRUE)
		{
			alarm_cnt = 0;
			xTimerStop(HALL_ALARM_DETECT_TIMER_EVT_ID, 0);
		}
		xTimerReset(HALL_ALARM_PROTECT_TIMER_EVT_ID, 0);
		vTaskSendQueueEvt(KEY_QUE_HDL, MOTOR_CLOSE_MSG_ID_EVT, 0xBB, NULL);
		DBGPRINTF("lock close timer\r\n"); 			
	}
	else {
		//误触发处理
		if ((lock_info.ble_working == 0) && (lock_info.nb_flag == 1) && (lock_info.cat1_working == 0) \
			&& (xTimerIsTimerActive(HALL_ALARM_PROTECT_TIMER_EVT_ID) == pdFALSE) && (xTimerIsTimerActive(MOTOR_TIMEOUT_TIMER_EVT_ID) ==	pdFALSE))
			lock_info.sleep_flag = 1;
		//do nothing
		uint32_t tmp = GPIO_Pin_Read(U32BIT(LOCK_STATUS));
		DBGPRINTF("lock state irq nothing timer %d  %d\r\n", tmp, lock_info.unlock_flag);
	}
}

//按键长短按处理
static void key_handle_timer_evt_irq_handle(void) //400ms检测一次
{
	lock_info.key_cnt++;
	if ((lock_info.key_cnt >= 5) && (lock_info.key_cnt < 25))
	{
		GPIO_Pin_Clear(U32BIT(GREEN_LED));
		GPIO_Pin_Set(U32BIT(BLUE_LED));
	}
	else if ((lock_info.key_cnt >= 25) && (lock_info.key_cnt < 50))
	{
		GPIO_Pin_Clear(U32BIT(GREEN_LED));
		GPIO_Pin_Clear(U32BIT(BLUE_LED)); 
	}
  else if ((lock_info.key_cnt >= 50) && (lock_info.key_cnt < 70))
  {
    GPIO_Pin_Set(U32BIT(BLUE_LED));
    GPIO_Pin_Set(U32BIT(GREEN_LED));    
  }
	//一直按着按键，按键有可能已出现故障
	else if (lock_info.key_cnt >= 100)
	{
    GPIO_Pin_Set(U32BIT(BLUE_LED));
    GPIO_Pin_Set(U32BIT(GREEN_LED));
		xTimerStop(KEY_HANDLE_TIMER_EVT_ID, 0);
		lock_info.key_cnt = 0;
    lock_info.sleep_flag = 1;
	}
	else {
		//do nothing
	}
	if(!GPIO_Pin_Read(U32BIT(WakeUp_KEY)))
	{
		xTimerStop(KEY_HANDLE_TIMER_EVT_ID, 0);
		if (lock_info.key_cnt < 5) //400m定时器
		{
			vTaskSendQueueEvt(KEY_QUE_HDL, KEY_BLUETOOTH_MSG_ID_EVT, 0xBB, NULL);			 
			lock_info.work_mode = BLUETOOTH_MODE;
		}
		else if ((lock_info.key_cnt >= 5) && (lock_info.key_cnt < 25))
		{
			GPIO_Pin_Set(U32BIT(GREEN_LED));
			vTaskSendQueueEvt(KEY_QUE_HDL, KEY_NB_MSG_ID_EVT, 0xBB, NULL);	
			lock_info.work_mode = CAT1_MODE;           
		}
    else if ((lock_info.key_cnt >= 25) && (lock_info.key_cnt < 50))
    {
      GPIO_Pin_Set(U32BIT(GREEN_LED));
			GPIO_Pin_Set(U32BIT(BLUE_LED)); 
			vTaskSendQueueEvt(KEY_QUE_HDL, KEY_POWER_DOWN_MSG_ID_EVT, 0xBB, NULL);	
    }
		else
		{
			vTaskSendQueueEvt(KEY_QUE_HDL, KEY_MOTOR_CHECK_MSG_ID_EVT, 0xBB, NULL);	
		}
		lock_info.key_cnt = 0;
		DBGPRINTF("key pull up\r\n");
	}
}

static void nb_delay_timer_evt_irq_handle(void)
{
	vTaskSendQueueEvt(UART_QUE_HDL, UART0_NB_DELAY_MSG_ID_EVT, 0xBB, NULL);
}

static void nb_timeout_timer_evt_irq_handle(void)
{
	vTaskSendQueueEvt(UART_QUE_HDL, UART0_NB_REV_TIMEOUT_MSG_ID_EVT, 0xBB, NULL);
}

static void nb_error_timer_evt_irq_handle(void)
{
	cat1_error_led();
}

static void nb_interval_timer_evt_irq_handle(void)
{
	vTaskSendQueueEvt(UART_QUE_HDL, UART0_NB_REPEAT_MSG_ID_EVT, 0xBB, NULL);
}

static void led_one_second_timer_evt_irq_handle(void)
{
	GPIO_Pin_Set(U32BIT(GREEN_LED) | U32BIT(BLUE_LED));

	if (queue_is_empty(pcat1_handle) == 0)
	{
		vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, CAT1_HANDLE_MSG_ID_EVT, 0xBB, NULL);
	}
}

static void power_down_timer_evt_irq_handle(void)
{
	static uint16_t cnt = 0;
	
	GPIO_Pin_Turn(U32BIT(PWR_CTRL));
	cnt++;
	//如果关机失败，直接重启
	if (cnt == 10000)
	{
		SystemReset();
	}
}
/************************************************************************************************************************
* 函数名称: timers_init()
* 功能说明: 初始化所有定时器事件
* 输 入: 无
* 输 出: 无
* 注意事项: 无
************************************************************************************************************************/
void timers_init(void)
{
		// Initialize timer module.
    SYD_Timer_Init(EVT_NUM, syd_sysTimer);	
    
    Timer_Evt_Creat(OTA_MANAGE_TIMER_EVT_ID, 
                    OTA_MANAGE_TIMER_EVT_INTERVAL, 
                    ota_manage_timer_evt_irq_handle, 
                    EVT_DISABLE_MODE);
		//关机模拟PWM
		Timer_Evt_Creat(POWER_DOWN_TIMER_EVT_ID,                  
								POWER_DOWN_TIMER_EVT_INTERVAL, 
								power_down_timer_evt_irq_handle, 
								EVT_DISABLE_MODE);
								
		//按键长短按判断
		KEY_HANDLE_TIMER_EVT_ID = xTimerCreate((const char*)"Key_HandleTimer",
																						 (TickType_t)KEY_HANDLE_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdTRUE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)key_handle_timer_evt_irq_handle);											
	 									
		//马达转动超时								
		MOTOR_TIMEOUT_TIMER_EVT_ID = xTimerCreate((const char*)"Motor_TimeoutTimer",
																						 (TickType_t)20,
																						 (UBaseType_t)pdTRUE,
																						 (void *)2,
																						 (TimerCallbackFunction_t)motor_timeout_timer_evt_irq_handle);																							 
	  //绿灯闪烁
		GREEN_LED_TIMER_EVT_ID = xTimerCreate((const char*)"GreenLed_PeriodTimer",
																						 (TickType_t)GREEN_LED_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdTRUE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)green_led_timer_evt_irq_handle);											
		//蓝灯闪烁
		BLUE_LED_TIMER_EVT_ID = xTimerCreate((const char*)"BlueLed_PeriodTimer",
																						 (TickType_t)BLUE_LED_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdTRUE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)blue_led_timer_evt_irq_handle);												
										
		//LED灯亮1s							
		LED_ONE_SECOND_TIMER_EVT_ID = xTimerCreate((const char*)"Led_OneSecondTimer",
																						 (TickType_t)LED_ONE_SECOND_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)led_one_second_timer_evt_irq_handle);											
		//按键消抖
		KEY_DEBOUNCE_TIMER_EVT_ID = xTimerCreate((const char*)"Key_DebounceTimer",
																						 (TickType_t)KEY_DEBOUNCE_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)key_debounce_timer_evt_irq_handle);		
		//拆卸电池消抖
		BAT_DET_DEBOUNCE_TIMER_EVT_ID = xTimerCreate((const char*)"BatteryDet_DebounceTimer",
																						 (TickType_t)1000,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)battery_detect_debounce_timer_evt_irq_handle);		                                             
																							
		//关锁检测、开锁成功检测、关锁成功检测、剪断检测消抖
		LOCK_DEBOUNCE_TIMER_EVT_ID = xTimerCreate((const char*)"Gpio_DebounceTimer",
																						 (TickType_t)LOCK_DEBOUNCE_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)lock_debounce_timer_evt_irq_handle);		
	
		//Cat.1发送AT命令超时
		NB_TIMEOUT_TIMER_EVT_ID = xTimerCreate((const char*)"Key_DebounceTimer",
																						 (TickType_t)NB_TIMEOUT_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)nb_timeout_timer_evt_irq_handle);											
		//Cat.1发送AT命令间隔
		NB_CMD_INTERVAL_TIMER_EVT_ID = xTimerCreate((const char*)"Key_DebounceTimer",
																						 (TickType_t)NB_CMD_INTERVAL_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)nb_interval_timer_evt_irq_handle);												
		//Cat.1出错灯闪
		NB_ERROR_TIMER_EVT_ID = xTimerCreate((const char*)"Key_DebounceTimer",
																						 (TickType_t)NB_ERROR_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdTRUE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)nb_error_timer_evt_irq_handle);											
										
		//发送AT命令延时
		NB_DELAY_TIMER_EVT_ID = xTimerCreate((const char*)"Key_DebounceTimer",
																						 (TickType_t)NB_DELAY_TIMER_EVT_INTERVAL,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)nb_delay_timer_evt_irq_handle);	
		//低功耗倒计时
		SLEEP_TIMER_EVT_ID = xTimerCreate((const char*)"SleepTimer",
																						 (TickType_t)10000,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)app_sleep_timer_evt_irq_handle);	  
		//霍尔拉紧采用轮询																		 
		HALL_TIGHTEND_TIMER_EVT_ID = xTimerCreate((const char*)"Hall_TightendTimer",
																						 (TickType_t)1000,
																						 (UBaseType_t)pdTRUE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)hall_tightend_timer_evt_irq_handle);	
		//撬锁告警保护																	 
		HALL_ALARM_PROTECT_TIMER_EVT_ID = xTimerCreate((const char*)"HallAlarmProtectTimer",
																						 (TickType_t)5000,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)hall_alarm_protect_timer_evt_irq_handle);	
		//检测到撬锁后，多次检测锁状态
		HALL_ALARM_DETECT_TIMER_EVT_ID = xTimerCreate((const char*)"HallAlarmDetectTimer",
																						 (TickType_t)500,
																						 (UBaseType_t)pdTRUE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)hall_alarm_detect_timer_evt_irq_handle);																						 

		//手机连接蓝牙超时																		 
		BLE_CONNECT_TIMEOUT_TIMER_EVT_ID = xTimerCreate((const char*)"BLE_connectTimeoutTimer",
																						 (TickType_t)30000,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)ble_connect_timeout_evt_irq_handle);		
		//开锁后，延时5s，回转马达																	 
		OPEN_LOCK_TIMEOUT_ROTATION_TIMER_EVT_ID = xTimerCreate((const char*)"Open_LockRotaionTimeoutTimer",
																						 (TickType_t)4000,
																						 (UBaseType_t)pdFALSE,
																						 (void *)3,
																						 (TimerCallbackFunction_t)open_lock_rotation_timeout_evt_irq_handle);																					 
}

/************************************************************************************************************************
* 函数名称: task_period_rtc_evt_irq_handle()
* 功能说明: RTC中断事件处理
* 输 入: 无
* 输 出: 无
* 注意事项: 无
************************************************************************************************************************/
//	RTC_EVT_Stop(TASK_PERIOD_RTC_EVT_ID);
static void task_period_rtc_evt_irq_handle(void)
{
	lock_info.period_cnt++;
	lock_info.battery_detect_flag = 0;
	if (lock_info.period_cnt >= flash_data.period_time)
	{
		lock_info.sleep_flag = 0;	
		lock_info.period_cnt = 0;
		vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, TASK_PERIOD_MSG_ID_EVT, NULL, NULL);
	}
	else{
		if (lock_info.sleep_flag == 1) //给高能电容一段快速充电的过程
		{
			lock_info.sleep_flag = 0;
			GAPBBDelayMS(200); 
			GPIO_Pin_Set(U32BIT(CAP_BOOST));
			lock_info.sleep_flag = 1;
		}
	}
}

/************************************************************************************************************************
* 函数名称: rtcs_init()
* 功能说明: 初始化所有RTC事件
* 输 入: 无
* 输 出: 无
* 注意事项: 无
************************************************************************************************************************/
void rtcs_init(void)
{	
//  flash_data.period_time = 2;
	//防止耗电严重，最低10hour
	if (flash_data.period_time <= 10) 
		flash_data.period_time = 10;
	lock_info.period_cnt = 0;
	DBGPRINTF("period time = %d\r\n", flash_data.period_time);
	// Initialize rtc module.
	SYD_RTC_Init(RTCEVT_NUM, syd_rtc);
	//周期上报 
	RTC_EVT_Creat(TASK_PERIOD_RTC_EVT_ID,
				  (3600),  //300,//(3600)
				  task_period_rtc_evt_irq_handle,
				  RTCEVT_DISABLE_MODE);  //RTCEVT_ENABLE_MODE没啥用
	//看门狗			
	RTC_EVT_Creat(TASK_WATCHDOG_RTC_EVT_ID,
					TASK_WATCHDOG_RTC_INTERVAL,
					watchdog_timer_evt_irq_handle,
					RTCEVT_DISABLE_MODE);
	//4G升级回馈
//	RTC_EVT_Creat(TASK_UPDATE_RTC_EVT_ID,
//					TASK_UPDATE_RTC_INTERVAL,
//					task_update_rtc_evt_irq_handle,
//					RTCEVT_DISABLE_MODE);

	RTC_EVT_Start(TASK_WATCHDOG_RTC_EVT_ID);
	RTC_EVT_Start(TASK_PERIOD_RTC_EVT_ID);
	
//	RTC_EVT_Start(TASK_UPDATE_RTC_EVT_ID);		
}
