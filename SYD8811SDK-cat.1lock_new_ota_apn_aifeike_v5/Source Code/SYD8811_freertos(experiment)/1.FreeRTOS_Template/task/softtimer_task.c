#include "softtimer_task.h"
#include "..\application\softtimer.h"
#include "main.h"
#include "ble.h"
#include "lib.h"
#include "debug.h"
#include "gpio.h"
#include "led_key.h"
#include "lock.h"
#include "ota.h"
#include "Cat1.h"
#include "pwm.h"
#include "gpadc.h"

extern TimerHandle_t NB_DELAY_TIMER_EVT_ID;
extern TimerHandle_t SLEEP_TIMER_EVT_ID;
extern TimerHandle_t GREEN_LED_TIMER_EVT_ID;
extern TimerHandle_t BLE_CONNECT_TIMEOUT_TIMER_EVT_ID;
extern TimerHandle_t MOTOR_TIMEOUT_TIMER_EVT_ID;
extern TimerHandle_t KEY_HANDLE_TIMER_EVT_ID; 
extern TimerHandle_t LOCK_DEBOUNCE_TIMER_EVT_ID; 
extern TimerHandle_t KEY_DEBOUNCE_TIMER_EVT_ID;
extern TimerHandle_t HALL_TIGHTEND_TIMER_EVT_ID;

extern lock_info_t lock_info;
extern cat1_info_t *p_cat1_info;
extern cat1_recv_t cat1_recv;
extern void gpio_pin_sleep(uint8_t mode);
extern volatile uint8_t cmd_index;
extern queue_t *pcat1_handle;
extern flash_data_t flash_data;

void  ota_manage(void){
	#ifdef _OTA_
	if(ota_state){
		switch(ota_state){
			case 1 : 
				lock_info.ble_update_flags = 1;
				if (lock_info.unlock_flag == 0)
				{
					SystemReset();//锁状态必须是开启才能升级
				}
				xTimerStop(BLE_CONNECT_TIMEOUT_TIMER_EVT_ID, 0);		//关闭蓝牙超时
				CmdFwErase();
				#if defined(_DEBUG_) || defined(_SYD_RTT_DEBUG_)
				dbg_printf("OTA start ERASE\r\n");
				#endif
				ota_state = 2;
				ota_writecnt = 0;
			  GPIO_Pin_Set(U32BIT(GREEN_LED));
				break;
			case 2 : 
			case 8 : 
				ota_writecnt++;
				if(ota_writecnt>0x20)
				{
					#if defined(_DEBUG_) || defined(_SYD_RTT_DEBUG_)
						dbg_printf("OTA faile\r\n");
					#endif
					Timer_Evt_Stop(OTA_MANAGE_TIMER_EVT_ID);
				}
				break;
			case 3 : 
				ota_state=0;
				lock_info.ble_update_flags = 0;
				Timer_Evt_Stop(OTA_MANAGE_TIMER_EVT_ID);
				#if defined(_DEBUG_) || defined(_SYD_RTT_DEBUG_)
					dbg_printf("OTA finish\r\n");
				#endif
				DelayMS(1000);
			#if defined(_DEBUG_) || defined(_SYD_RTT_DEBUG_)
				dbg_printf("Start Reset 1000s\r\n");
			#endif
				SystemReset();
				break;
			case 7 : 
				EraseFlashData(0, 30);  //假设FLASHDATA区域大小为120KB扇区数目为120KB/4KB=30  这里擦除需要比较长的时间，所以APP那端要延时一下
				#if defined(_DEBUG_) || defined(_SYD_RTT_DEBUG_)
				dbg_printf("OTA start\r\n");
				#endif
				ota_state =8;
				ota_writecnt=0;
				break;
			default :
				break;
		}
	}
	#endif
}

void SofttimerTask( void *pvParameters )
{
	queue_msg_t evt;
	queue_msg_hal_array[SOFTTIMER_QUE_HDL] = xQueueCreate(8, sizeof(queue_msg_t));
	
	for( ;; )
	{
		if(xQueueReceive(queue_msg_hal_array[SOFTTIMER_QUE_HDL], &evt, portMAX_DELAY))
		{
			switch ((uint8_t)evt.msg_id)
			{
				case CONNECTION_LATENCY_MSG_ID_EVT:
					BLSetConnectionUpdate(1);
					DBGPRINTF("CONNECTION_LATENCY_MSG_ID_EVT\r\n");
					break;
				//OTA管理
				case OTA_MANAGE_MSG_ID_EVT:
					DBGPRINTF("OTA_MANAGE_MSG_ID_EVT\r\n");
					ota_manage();     //ota 过程管理 操作屏幕等
					Timer_Evt_Clr(OTA_MANAGE_TIMER_EVT_ID);
					break;
			
				case TASK_180S_MSG_ID_EVT:
					#ifdef USER_32K_CLOCK_RCOSC
					LPOCalibration();
					#endif
					DBGPRINTF("TASK_180S_MSG_ID_EVT\r\n");
					break;
				//周期上报
				case TASK_PERIOD_MSG_ID_EVT: 
					DBGPRINTF("nb period send\r\n");
          {
            lock_handle_t data;
            data.opcode = PERIOD_TIRGGER;
            data.state = 0x00;
            queue_en(pcat1_handle, &data);
          }
					if (lock_info.cat1_working & 0x01)
						break;
					lock_info.cat1_working = 1;					
          queue_de(pcat1_handle, &lock_info.data);
          cat1_recv_reset();
					if (xTimerIsTimerActive(SLEEP_TIMER_EVT_ID))
					{
						xTimerStop(SLEEP_TIMER_EVT_ID, 0);
					}          
					if (lock_info.nb_flag & 0x01)
					{
            gpio_pin_sleep(WAKE_UP);
						UartEn(true);
						lock_info.nb_flag = 0;
					}
					GPIO_Pin_Clear(U32BIT(CAP_BOOST));
					DelayMS(100);
					if (lock_info.battery_detect_flag == 0)
					{
						GPADC_start(); 
						lock_info.battery_detect_flag = 1;
					}
//          cat1_power_on();
					if (p_cat1_info->register_status & 0x01)
					{
						DBGPRINTF("Lock period PERIOD_TIRGGER \r\n");
						p_cat1_info->send_lock_cmd();
					}
					else
					{
						ec800g_power_on();
						xTimerStart(NB_DELAY_TIMER_EVT_ID, 0);
					}	
					break;
				//4G升级后上报
				case TASK_UPDATE_MSG_ID_EVT:
					DBGPRINTF("cat1 update send\r\n");
          {
            lock_handle_t data;
            data.opcode = UPDATE_TIRGGER;
            data.state = 0x00;
            queue_en(pcat1_handle, &data);
          }
					if (lock_info.cat1_working & 0x01)
						break;
					lock_info.cat1_working = 1;					
          queue_de(pcat1_handle, &lock_info.data);
          cat1_recv_reset();
					if (xTimerIsTimerActive(SLEEP_TIMER_EVT_ID))
					{
						xTimerStop(SLEEP_TIMER_EVT_ID, 0);
					}          
					if (lock_info.nb_flag & 0x01)
					{
            gpio_pin_sleep(WAKE_UP);
						UartEn(true);
						lock_info.nb_flag = 0;
					}
					GPIO_Pin_Clear(U32BIT(CAP_BOOST));
					DelayMS(100);
					if (lock_info.battery_detect_flag == 0)
					{
						GPADC_start(); 
						lock_info.battery_detect_flag = 1;
					}
//          cat1_power_on();
					if (p_cat1_info->register_status & 0x01)
					{
						DBGPRINTF("Lock update send \r\n");
						p_cat1_info->send_lock_cmd();
					}
					else
					{
						ec800g_power_on();
						xTimerStart(NB_DELAY_TIMER_EVT_ID, 0);
					}						
					break;
//				//Cat.1指示灯
//				case GREEN_LED_MSG_ID_EVT:               
//					GPIO_Pin_Turn(U32BIT(GREEN_LED));
//					break;			
				//cat.1还在工作中，触发关锁
				case CAT1_HANDLE_MSG_ID_EVT:         
					lock_info.cat1_working = 1;
          queue_de(pcat1_handle, &lock_info.data);
					if (xTimerIsTimerActive(SLEEP_TIMER_EVT_ID))
					{
						xTimerStop(SLEEP_TIMER_EVT_ID, 0);
					}
					DBGPRINTF("Lock cat1 work close LOCK_TIRGGER \r\n");
					p_cat1_info->send_lock_cmd();				
					break;	
				//马达转动超时
				case MOTOR_TIMEOUT_MSG_ID_EVT: 
				{
					uint8_t i = 0;
					for (i = 0; i < 20; i++)
					{
						GAPBBDelayMS(10);
						motor_reverse();
						GAPBBDelayMS(40); //检测到位后，马达多转50ms
						GPIO_Pin_Clear(U32BIT(MotorCtrl_1));
						GPIO_Pin_Clear(U32BIT(MotorCtrl_2)); 
						if (GPIO_Pin_Read(U32BIT(MOTOR_STATUS_CLOSE)))
								break;
					}
					DBGPRINTF("motor close timeout = %d\r\n", i);
				}
//					lock_info.sleep_flag = 1;  //不能置1，否则导致马达无法停止
					break;		
				//睡眠事件  
				case TASK_APP_SLEEP_MSG_ID_EVT:        			
					//测试出现死机问题
          if (xTimerIsTimerActive(KEY_HANDLE_TIMER_EVT_ID) == pdTRUE)
          {
            DBGPRINTF("TASK_APP_Sleep  break\r\n");
            break;
          }
          DBGPRINTF("TASK_APP_Sleep\r\n");
					
					xTimerStop(HALL_TIGHTEND_TIMER_EVT_ID, 0);
//					GPIO_Pin_Set(U32BIT(CAP_BOOST));
//					Hpwm_Stop();
					UartEn(false);
          lock_info.nb_flag = 1;
					p_cat1_info->register_status = 0;
					GPIO_Pin_Clear(U32BIT(CAT1_PWR_CTRL));
					
					if (get_gps_state())
						GPIO_Pin_Set(U32BIT(GNSS_PWR_EN));			
					//恢复绿灯2s的周期闪烁
          taskENTER_CRITICAL();
					xTimerChangePeriod(GREEN_LED_TIMER_EVT_ID, 2000, 0);
					xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);
					GPIO_Pin_Set(U32BIT(GREEN_LED));
          taskEXIT_CRITICAL();
					//蓝牙不在工作时，才置标志位
					if ((lock_info.ble_working == 0) && (xTimerIsTimerActive(MOTOR_TIMEOUT_TIMER_EVT_ID) == pdFALSE) && \
          (xTimerIsTimerActive(KEY_DEBOUNCE_TIMER_EVT_ID) == pdFALSE) && (xTimerIsTimerActive(LOCK_DEBOUNCE_TIMER_EVT_ID) == pdFALSE))
					{					
						gpio_pin_sleep(LOW_POWER);
            DBGPRINTF("TASK_APP_Sleep sleep\r\n");
 						lock_info.sleep_flag = 1;
					}          
					break;
				//看门狗喂狗,绿灯有时候出现常亮，一直未找到原因，喂狗增加关闭led
				case WATCHDOG_MSG_ID_EVT:
//					DBGPRINTF("watchdog is done\r\n");
          {
            static uint8_t cnt = 0;
            cnt++;
            feed_dog();
            if (cnt%10 == 0)
            {
              cnt = 0;
              if (xTimerIsTimerActive(GREEN_LED_TIMER_EVT_ID) == pdFALSE)
              { 
                 GPIO_Pin_Set(U32BIT(GREEN_LED));
              }
            }
          }
					break;
				//发送AT命令延时	
				case AT_RESET_SLEEP_MSG_ID_EVT:
					//advance_recv_msg_handle(cat1_recv.buf);
					cmd_index = SUB_QGPSGNMEA;
					p_cat1_info->cmd_next(cmd_index);
					break;
				//APP连接超时
				case BLE_CONNECT_TIMEOUT_MSG_ID_EVT:
				{
					DBGPRINTF("BLE time out 30s  ");
					if(lock_info.ble_connected == 1){
							lock_info.ble_connected = 0;
							DisConnect();							
							DBGPRINTF("DisConnect ");
					}
					struct gap_adv_params adv_params;
					adv_params.type		= ADV_IND;
					adv_params.channel 	= 0x07;  
					adv_params.interval = 480;		
					adv_params.timeout 	= 5; 
					SetAdvParams(&adv_params);	
					DBGPRINTF("SetAdvParams_adv_params ");				
				}
					break;
				default:
					break;
			}
		}
	}
}
