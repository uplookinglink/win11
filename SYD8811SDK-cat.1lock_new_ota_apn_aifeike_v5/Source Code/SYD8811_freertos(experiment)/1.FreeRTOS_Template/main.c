/* BLE */
#include "ARMCM0.h"
#include "config.h"
#include "lib.h"
//#include "SYD_ble_service_Profile.h"
//#include "SYD8811_ZLS_Profile.h"
#include "ble.h"
#include "main.h"
/*drive*/
#include "debug.h"
#include "DebugLog.h"
#include "gpio.h"
#include "led_key.h"
#include "..\application\softtimer.h"
#include "softtimer_task.h"
#include "lock.h"
#include "stdint.h"
#include "gpadc.h"
#include "uart.h"
#include "Cat1.h"
#include "pwm.h"
#include "uart.h"

extern TimerHandle_t NB_DELAY_TIMER_EVT_ID;
extern TimerHandle_t KEY_DEBOUNCE_TIMER_EVT_ID;
extern TimerHandle_t LOCK_DEBOUNCE_TIMER_EVT_ID;
extern TimerHandle_t GREEN_LED_TIMER_EVT_ID;
extern TimerHandle_t MOTOR_TIMEOUT_TIMER_EVT_ID;
extern TimerHandle_t BLUE_LED_TIMER_EVT_ID;
extern TimerHandle_t SLEEP_TIMER_EVT_ID;
extern TimerHandle_t BAT_DET_DEBOUNCE_TIMER_EVT_ID;
extern TimerHandle_t HALL_TIGHTEND_TIMER_EVT_ID;
extern TimerHandle_t OPEN_LOCK_TIMEOUT_ROTATION_TIMER_EVT_ID;
//ADC
extern uint8_t channel;
extern flash_data_t flash_data;
extern queue_t *pcat1_handle;
extern otg_send_t otg_send;
extern otg_recv_t otg_recv;

lock_info_t lock_info = {.status = 0};
lock_irq_t lock_irq;
cat1_info_t *p_cat1_info;

//  SemaphoreHandle_t xMutex;
//  xMutex = xSemaphoreCreateMutex();
//vTaskDelay() 
xQueueHandle queue_msg_hal_array[QUE_MSG_MAX];
/*任务名  运行状态 优先级 堆栈剩余空间 创建顺序
KeyHandle  R  7  181  1
IDLE       R  0  211  4
Tmr Svc    B  8  150  5
Softtimer  B  5  226  2
Uart0Hand  B  6  226  3
*/
void tasklist_monitor(void)
{
	DBGPRINTF("------------------------\r\n");
	char *pdata = (char *)pvPortMalloc(200);
	vTaskList(pdata);
	DBGPRINTF("%s", pdata);
	DBGPRINTF("------------------------\r\n");
	vPortFree(pdata);
}

//有中断触发就重置低功耗标志位
static void gpio_callback(void)
{
	//获取中断管脚
	uint32_t value = GPIO_IRQ_CTRL->GPIO_INT;

	if(value & U32BIT(WakeUp_KEY))
	{
		lock_info.sleep_flag = 0;
		xTimerReset(KEY_DEBOUNCE_TIMER_EVT_ID, 0);
		DBGPRINTF("wake up key irq\r\n");
	}
	//用else if有问题
	else if(value & U32BIT(LOCK_STATUS))
	{
		lock_info.sleep_flag = 0;
		xTimerReset(LOCK_DEBOUNCE_TIMER_EVT_ID, 0);
		DBGPRINTF("lock status irq\r\n");
	}
  else if (value & U32BIT(BAT_DET))
  {
		lock_info.sleep_flag = 0;
		xTimerReset(BAT_DET_DEBOUNCE_TIMER_EVT_ID, 0);
		DBGPRINTF("battery detect irq\r\n");    
  }
	else
	{
		DBGPRINTF("do nothing\r\n");
		//do nothing
	}
}

void adc_init(void)
{
		//GPADC_Manual_Calibration(0x12);   //手动校准函数  改函数只是在前期工程样品上有用 0X12是一个比较合理的项目  在对ADC不是十分苛刻的项目上都可以直接使用
	{
		uint8_t trim=GPADC_Get_Calibration();    //获取芯片内部测试阶段写入的校准值
		//dbg_printf("FT Gpadc Trim:%x\r\n",trim);
		if(trim == 0)   
			GPADC_Manual_Calibration(0x12);   //这里读到芯片内部的校准值为0，表示当前批次没有写入校准值 这里写入0X12即可，0x12是一个比较合理的值
		else 
			GPADC_Manual_Calibration(trim);    //如果芯片内部已经有校准值  这里可以字节写入芯片内部的校准值  该值和0x12不会偏差太多		
	}
	GPADC_Init(AVE_MODE);
}
//芯片SYD8811没有内部下拉功能
void board_init(void)
{
	uint32_t io_input = 0, io_output = 0, io_outlow = 0, io_pull = 0, io_invert = 0;
	
	#if 1
	#ifdef USER_32K_CLOCK_RCOSC 
		PIN_CONFIG->PAD_INPUT_PULL_UP[0] = 0xff; // pull up
	#else
    PIN_CONFIG->PAD_INPUT_PULL_UP[0] = 0xfc; // pull up  gpio 0 1 32K xosc
	#endif
	
	PIN_CONFIG->PAD_INPUT_PULL_UP[1] = 0xff; // pull up
	PIN_CONFIG->PAD_INPUT_PULL_UP[2] = 0xff; // pull up
	PIN_CONFIG->PAD_INPUT_PULL_UP[3] = 0xcf; // pull up SWDIO/SWCLK 复位管脚取消上拉，P21->PAD27
	PIN_CONFIG->PAD_INPUT_PULL_UP[4] = 0xff; // pull up
	#endif
	for(uint8_t i = 0; i < 32; i++)
	{
		switch(i)
		{
			#ifndef USER_32K_CLOCK_RCOSC
			case GPIO_0:
			case GPIO_1:
				 break;
			#endif
			
			//马达控制管脚
			case MotorCtrl_1:
			case MotorCtrl_2:
				io_output |= U32BIT(i);
				io_outlow |= U32BIT(i); 		//defut low
				break;
			//LED灯
			case GREEN_LED:
			case BLUE_LED:
				io_output |= U32BIT(i);
				io_outlow |= U32BIT(i); 		//defut low
				break;

			case PWR_CTRL:
			case CAT1_PWR_CTRL:
			case CAT1_PWR_KEY:
			case CAT1_WAKE_UP:
			case WatchDog:
      case GNSS_PWR_EN:
			case CAP_BOOST:
				io_output |= U32BIT(i);
				break;
			//输入管脚  io_invert |= U32BIT(i); ? io_invert &=~ U32BIT(i)			
			case WakeUp_KEY:
			case MOTOR_STATUS_CLOSE:
			case MOTOR_STATUS_OPEN:
				io_input  |= U32BIT(i);
				io_pull   |= U32BIT(i);
				io_invert |= U32BIT(i);
				break;
			//低功耗考虑，单独设置成浮空状态	
//			case MOTOR_STATUS_CLOSE:
//				io_input  |= U32BIT(i);
//				io_pull   &=~ U32BIT(i);
//				io_invert |= U32BIT(i);				
//				break;
//				io_input  |= U32BIT(i);
//				io_pull   |= U32BIT(i);
//				io_invert |= U32BIT(i);			
      case BAT_DET:
//				io_input  |= U32BIT(i);
//				io_pull   &=~ U32BIT(i);
//				io_invert &=~ U32BIT(i);
//				break;
			
      case HALL_DET:
			case LOCK_STATUS:
				io_input  |= U32BIT(i);
				io_pull   &=~ U32BIT(i);
				io_invert |= U32BIT(i); 			
				break;
			//串口
			case UART0_RX:
			case UART0_TX:
			case MCU_RESET:
				break;

			default:
				io_input  |= U32BIT(i);
				io_pull   |= U32BIT(i);
			break;
		}
	}
  PIN_CONFIG->PAD_2_INPUT_PULL_UP = 0;
	PIN_CONFIG->PAD_7_INPUT_PULL_UP = 0;  	//霍尔检测
	PIN_CONFIG->PAD_8_INPUT_PULL_UP = 0;  	//锁梁检测
	
	PIN_CONFIG->PAD_10_INPUT_PULL_UP = 1; 	//马达开锁
	PIN_CONFIG->PAD_9_INPUT_PULL_UP = 0;  	//马达关锁

	//DEFAULT OUT
	GPIO_Set_Output(io_output);
	PIN_Pullup_Disable(T_QFN_48, io_output);
	
	//DEFAULT OUT HIGHT PIN
	GPIO_Pin_Set(io_output & (~io_outlow));
	//DEFAULT LOW LOW PIN
	GPIO_Pin_Clear(io_outlow);
	
	//INPORT PIN
	GPIO_Set_Input(io_input, io_invert);
	//PIN_Pullup_Disable(T_QFN_48, io_input & (~io_pull));
	// DEFAULT PULL
	PIN_Pullup_Enable(T_QFN_48, io_pull);
} 
//切换管脚配置，1:进入低功耗IO配置  0：退出低功耗IO配置
void gpio_pin_sleep(uint8_t mode)
{	
	if(mode)
	{
		PIN_Pullup_Disable(T_QFN_48, U32BIT(MOTOR_STATUS_CLOSE) | U32BIT(MOTOR_STATUS_OPEN));	
		GPIO_Set_Output(U32BIT(MOTOR_STATUS_CLOSE) | U32BIT(MOTOR_STATUS_OPEN));
		GPIO_Pin_Clear(U32BIT(MOTOR_STATUS_CLOSE) | U32BIT(MOTOR_STATUS_OPEN));
	}
	else
	{
		GPIO_Set_Input(U32BIT(MOTOR_STATUS_CLOSE) | U32BIT(MOTOR_STATUS_OPEN), U32BIT(MOTOR_STATUS_CLOSE) | U32BIT(MOTOR_STATUS_OPEN));	
		PIN_Pullup_Enable(T_QFN_48, U32BIT(MOTOR_STATUS_CLOSE) | U32BIT(MOTOR_STATUS_OPEN));
	}
}

//输入类中断类型处理任务 
void KeyHandle_Task( void *pvParameters )
{
	queue_msg_t evt;
	queue_msg_hal_array[KEY_QUE_HDL] = xQueueCreate(4, sizeof(queue_msg_t));
	
	for( ;; )
	{
		if(xQueueReceive(queue_msg_hal_array[KEY_QUE_HDL], &evt, portMAX_DELAY))
		{
#if TASK_LIST_OPEN		
			tasklist_monitor();
#endif
			switch ((uint8_t)evt.msg_id)
			{	        
				//长按CAT状态
				case KEY_NB_MSG_ID_EVT:
					DBGPRINTF("key nb msg evt\r\n");        
					GPIO_Pin_Clear(U32BIT(GREEN_LED));
					xTimerStart(GREEN_LED_TIMER_EVT_ID, 0);
					//CAT工作时，屏蔽按键
					if (lock_info.cat1_working & 0x01)
						break;
					lock_info.cat1_working = 1;			
          {
            lock_handle_t data;
            data.opcode = BOOT_TIRGGER;
            data.state = 0x00;
            queue_en(pcat1_handle, &data);
          }   
          queue_de(pcat1_handle, &lock_info.data);
					//复位接收BUF
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
					if (p_cat1_info->register_status & 0x01)
					{
						DBGPRINTF("send unlock request\r\n");
						p_cat1_info->send_lock_cmd();
					}
					else
					{
						DBGPRINTF("request delay send at cmd\r\n");
            ec800g_power_on();
						xTimerChangePeriod(GREEN_LED_TIMER_EVT_ID, 2000, 0);
						xTimerStart(NB_DELAY_TIMER_EVT_ID, 0);
					}
          if (lock_info.irq_flags == 1)
          {
          	io_irq_enable((U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS) | U32BIT(BAT_DET)), &gpio_callback);  //U32BIT(MOTOR_STATUS_CLOSE) | U32BIT(MOTOR_STATUS_OPEN)
            lock_info.irq_flags = 0;
            wake_up_souce_cfg(3, 0);
          }             
					break;
				//短按蓝牙状态
				case KEY_BLUETOOTH_MSG_ID_EVT:					
					DBGPRINTF("key bluetooth msg evt\r\n");
					xTimerChangePeriod(BLUE_LED_TIMER_EVT_ID, 2000, 0);
					xTimerStart(BLUE_LED_TIMER_EVT_ID, 0);
					GPIO_Pin_Clear(U32BIT(BLUE_LED));	
				
					uart1_init();
					UartEn(true);
					memset(&otg_recv, 0, sizeof(otg_recv_t));
					//蓝牙工作时，屏蔽按键
					if (lock_info.ble_working & 0x01)
						break;
					lock_info.ble_working = 1;
					GPIO_Pin_Clear(U32BIT(CAP_BOOST));
					DelayMS(100);
          //采集电量
					if (lock_info.battery_detect_flag == 0)
					{
						GPADC_start(); 
						lock_info.battery_detect_flag = 1;
					}        
					gpio_pin_sleep(WAKE_UP);//管脚低功耗
					//临界区
					taskENTER_CRITICAL();
					RFWakeup();					
					DBGPRINTF("key bluetooth msg evt RFwakeup\r\n");
					wake_up_ble_init();		//蓝牙初始化
//					ble_init();
//					DBGPRINTF("key bluetooth msg evt ble init\r\n");
					StartAdv();
//					taskEXIT_CRITICAL();
					gatt_send_data(READ_STATUS);  //更新锁状态到ccd1
          if (lock_info.irq_flags == 1)
          {
          	io_irq_enable((U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS) | U32BIT(BAT_DET)), &gpio_callback);  //U32BIT(MOTOR_STATUS_CLOSE) | U32BIT(MOTOR_STATUS_OPEN)
            lock_info.irq_flags = 0;
            wake_up_souce_cfg(3, 0);
          } 
          taskEXIT_CRITICAL();          
					break;
        //马达回转，纠正马达位置
        case KEY_MOTOR_CHECK_MSG_ID_EVT:
          if (lock_info.unlock_flag == 1)
          {
            motor_reverse();
            GAPBBDelayMS(30);
            motor_stop();
          }
          break;       
				//关机
				case KEY_POWER_DOWN_MSG_ID_EVT:
					flash_data.update_hw = lock_info.hw;
					flash_data.update_sw = lock_info.sw;
					flash_data.cat1_state = 1;
					//保存flash
          write_flash(&flash_data);
					GPIO_Pin_Clear(U32BIT(GREEN_LED));
					GPIO_Pin_Clear(U32BIT(BLUE_LED));
					DelayMS(500);
					GPIO_Pin_Set(U32BIT(GREEN_LED));
					GPIO_Pin_Set(U32BIT(BLUE_LED));	
					DelayMS(500);
					GPIO_Pin_Clear(U32BIT(GREEN_LED));
					GPIO_Pin_Clear(U32BIT(BLUE_LED));		
					DelayMS(500);
					GPIO_Pin_Set(U32BIT(GREEN_LED));
					GPIO_Pin_Set(U32BIT(BLUE_LED));          
					//GPIO模式PWM
					GPIO_Pin_Clear(U32BIT(CAT1_PWR_CTRL));
          DelayMS(2000);
          if (flash_data.shoudown_flags == 0x00)
          {           
            GPIO_Pin_Set(U32BIT(PWR_CTRL));
            Timer_Evt_Start(POWER_DOWN_TIMER_EVT_ID);	
          }
					else{
						SystemReset();
					}
					break;
				//开锁成功
				case MOTOR_OPEN_MSG_ID_EVT:
					lock_info.unlock_flag = 1;		
					GPI_CTRL->GPI_POL &= ~U32BIT(LOCK_STATUS); //上升沿中断				
					DBGPRINTF("lock open evt\r\n");
					//延时5s，让马达回转到关锁为止
					xTimerStart(OPEN_LOCK_TIMEOUT_ROTATION_TIMER_EVT_ID, 0);
          wake_up_souce_cfg(0,0);
					GPIO_Pin_Set(U32BIT(BLUE_LED));					
					if (lock_info.work_mode == BLUETOOTH_MODE)
					{ 
						gatt_send_data(OPEN_STATUS);  
            //DisConnect();
            struct gap_adv_params adv_params;
            adv_params.type		= ADV_IND;
            adv_params.channel 	= 0x07;  
            adv_params.interval = 480;		
            adv_params.timeout 	= 7; 
            SetAdvParams(&adv_params);	    
					}
					//OTG方式开锁
					else if (lock_info.work_mode == OTG_MODE)
					{
						uint8_t index = 0, check_sum = 0;
						memset(&otg_send, 0, sizeof(otg_send_t));
						otg_send.buf[otg_send.len++] = 0xf0;
						otg_send.buf[otg_send.len++] = 0x0f;
						otg_send.buf[otg_send.len++] = OTG_SEND_LOCK_STATUS_CMD;
						index = otg_send.len;
						otg_send.buf[otg_send.len++] = 0x02;
						otg_send.buf[otg_send.len++] = 0x01;
						otg_send.buf[otg_send.len++] = lock_info.bat_val;
						for (uint8_t i = 0; i < 3; i++)
						{
							check_sum ^= otg_send.buf[index+i];
						}
						otg_send.buf[otg_send.len++] = check_sum;
						otg_send.buf[otg_send.len++] = 0xe0;
						otg_send.buf[otg_send.len++] = 0x0e;
						taskENTER_CRITICAL();
						uart1_write((uint8_t *)otg_send.buf, otg_send.len);
						taskEXIT_CRITICAL();
						lock_info.work_mode = BLUETOOTH_MODE;
						hexprint((uint8_t *)otg_send.buf, otg_send.len);
					}
					else
					{
            lock_handle_t data;
            data.opcode = LOCK_TIRGGER;
            data.state = 0x01;
            queue_en(pcat1_handle, &data);
            
            queue_de(pcat1_handle, &lock_info.data);
						if (xTimerIsTimerActive(SLEEP_TIMER_EVT_ID))
						{
							xTimerStop(SLEEP_TIMER_EVT_ID, 0);
						} 
						DBGPRINTF("Lock open LOCK_TIRGGER\r\n");
						
						if (lock_info.nb_flag & 0x01)
						{
							UartEn(true);
							lock_info.nb_flag = 0;
						}
						GPIO_Pin_Clear(U32BIT(CAP_BOOST));						
						if (p_cat1_info->register_status & 0x01)
						{
							DBGPRINTF("Lock open LOCK_TIRGGER\r\n");
							p_cat1_info->send_lock_cmd();
						}
						else
						{
							DBGPRINTF("hall delay send at cmd\r\n");
							ec800g_power_on(); 
							xTimerStart(NB_DELAY_TIMER_EVT_ID, 0);
						} 						
					}
					break;
				//关锁成功
				case MOTOR_CLOSE_MSG_ID_EVT: 
					GPIO_Pin_Clear(U32BIT(CAP_BOOST));
//          motor_stop(); 
					lock_info.unlock_flag = 0;		
					GPI_CTRL->GPI_POL |= U32BIT(LOCK_STATUS);	//下降沿中断					
          GAPBBDelayMS(100);
					lock_info.open_flags = 0;
          GPIO_Pin_Clear(U32BIT(MotorCtrl_1));
          GPIO_Pin_Clear(U32BIT(MotorCtrl_2));				
					DBGPRINTF("motor close evt\r\n");
          wake_up_souce_cfg(0,1);
					GPIO_Pin_Set(U32BIT(BLUE_LED));
					xTimerStart(GREEN_LED_TIMER_EVT_ID, 0);
          
          lock_handle_t data;
          data.opcode = LOCK_TIRGGER;
          data.state = 0x02;
          queue_en(pcat1_handle, &data);
					//不管是Cat.1方式关锁，还是蓝牙方式关锁，都用Cat.1进行上报
          //Cat.1上报关锁成功
          if (lock_info.cat1_working & 0x01)
          {
            DBGPRINTF("locked return\r\n");
          }
          else 
          {
            lock_info.cat1_working = 1;
            queue_de(pcat1_handle, &lock_info.data);
            lock_info.timeout_cnt = 0;
            if (xTimerIsTimerActive(SLEEP_TIMER_EVT_ID))
            {
              xTimerStop(SLEEP_TIMER_EVT_ID, 0);
            }
            if (lock_info.nb_flag & 0x01)
            {
              UartEn(true);
              lock_info.nb_flag = 0;
            }
						GPIO_Pin_Clear(U32BIT(CAP_BOOST));
            if (p_cat1_info->register_status & 0x01)
            {
              DBGPRINTF("Lock close LOCK_TIRGGER \r\n");
              p_cat1_info->send_lock_cmd();
            }
            else
            {
              DBGPRINTF("locked delay send at cmd\r\n");
              ec800g_power_on();
              xTimerStart(NB_DELAY_TIMER_EVT_ID, 0);
            } 
          }		
					
					//拉紧检测
					lock_info.hall_cnt = 0;
					lock_info.hall_flags = 0;
					lock_info.tightend_send_flags = 0;
					xTimerStart(HALL_TIGHTEND_TIMER_EVT_ID, 0);
					xTimerChangePeriod(BLUE_LED_TIMER_EVT_ID, 100, 0);
					xTimerStart(BLUE_LED_TIMER_EVT_ID, 0);  
					
					//蓝牙状态
					if (lock_info.work_mode == BLUETOOTH_MODE)
					{ 
						//7.18 蓝牙ccd1修改
						//if (lock_info.ble_working == 1)
						{
							DBGPRINTF("lock bluetooth msg evt\r\n");
//							xTimerChangePeriod(BLUE_LED_TIMER_EVT_ID, 2000, 0);
//							xTimerStart(BLUE_LED_TIMER_EVT_ID, 0);
//							GPIO_Pin_Clear(U32BIT(BLUE_LED));									
							//蓝牙工作时，屏蔽按键
							if (lock_info.ble_working & 0x01) {
								gatt_send_data(CLOSE_STATUS);
								break;
							}								
							lock_info.ble_working = 1;
							GPIO_Pin_Clear(U32BIT(CAP_BOOST));
							DelayMS(100);
          		//采集电量
							if (lock_info.battery_detect_flag == 0)
							{
								GPADC_start(); 
								lock_info.battery_detect_flag = 1;
							}             
							gpio_pin_sleep(WAKE_UP);//管脚低功耗
							//临界区
							taskENTER_CRITICAL();
							RFWakeup();					
							DBGPRINTF("lock bluetooth msg evt RFwakeup\r\n");
							lock_info.ble_timeout = 30;//蓝牙广播时间
							wake_up_ble_init();		//蓝牙初始化
							//					ble_init();
							//					DBGPRINTF("key bluetooth msg evt ble init\r\n");		
							StartAdv();
							taskEXIT_CRITICAL();							
							gatt_send_data(CLOSE_STATUS);  //更新锁状态到ccd1							
							//struct gap_adv_params adv_params;
							//adv_params.type		= ADV_IND;
							//adv_params.channel 	= 0x07;  
							//adv_params.interval = 480;		
							//adv_params.timeout 	= 0x1e; 
							//SetAdvParams(&adv_params);						  
						}							
					}	
					break;
				//剪断报警
				case  LOCK_HALL_MSG_ID_EVT:
					DBGPRINTF("lock hall msg\r\n");
					lock_info.unlock_flag = 1;
					GPI_CTRL->GPI_POL &= ~U32BIT(LOCK_STATUS); //上升沿中断
          lock_info.irq_flags = 1; 
          {
            lock_handle_t data;
            data.opcode = ALARM_TIRGGER;
            data.state = 0x01;
            queue_en(pcat1_handle, &data);
          }
					if (lock_info.cat1_working & 0x01)
						break;
					lock_info.cat1_working = 1;
          wake_up_souce_cfg(1, 0);
          queue_de(pcat1_handle, &lock_info.data);
          if (xTimerIsTimerActive(SLEEP_TIMER_EVT_ID))
					{
						xTimerStop(SLEEP_TIMER_EVT_ID, 0);
					}          
					if (lock_info.nb_flag & 0x01)
					{
						UartEn(true);
						lock_info.nb_flag = 0;
					}	
					GPIO_Pin_Clear(U32BIT(CAP_BOOST));					
					if (p_cat1_info->register_status & 0x01)
					{
						DBGPRINTF("Lock HALL ALARM_TIRGGER \r\n");
						p_cat1_info->send_lock_cmd();
					}
					else
					{
						DBGPRINTF("hall delay send at cmd\r\n");
            ec800g_power_on(); 
						xTimerStart(NB_DELAY_TIMER_EVT_ID, 0);
					}       					
					break;
				//锁绳拉紧
				case LOCK_TIGHTEND_MSG_ID_EVT:
					DBGPRINTF("lock is tightend msg\r\n");
          {
            lock_handle_t data;
            data.opcode = TIGHTEND_TIRGGER;
            data.state = 0x02;
            queue_en(pcat1_handle, &data);
          }
					if (lock_info.cat1_working & 0x01)
						break;
					lock_info.cat1_working = 1;
          queue_de(pcat1_handle, &lock_info.data);
          if (xTimerIsTimerActive(SLEEP_TIMER_EVT_ID))
					{
						xTimerStop(SLEEP_TIMER_EVT_ID, 0);
					}          
					if (lock_info.nb_flag & 0x01)
					{
						UartEn(true);
						lock_info.nb_flag = 0;
					}	
					GPIO_Pin_Clear(U32BIT(CAP_BOOST));					
					if (p_cat1_info->register_status & 0x01)
					{
						DBGPRINTF("Lock LOCK_TIGHTEND_MSG_ID_EVT \r\n");
						p_cat1_info->send_lock_cmd();
					}
					else
					{
						DBGPRINTF("tightend delay send at cmd\r\n");
            ec800g_power_on(); 
						xTimerStart(NB_DELAY_TIMER_EVT_ID, 0);
					}       					
					break;
				//锁绳未拉紧
				case LOCK_NOT_TIGHTEND_MSG_ID_EVT:
					DBGPRINTF("lock is not tightend msg\r\n");
          {
            lock_handle_t data;
            data.opcode = TIGHTEND_TIRGGER;
            data.state = 0x01;
            queue_en(pcat1_handle, &data);
          }
					if (lock_info.cat1_working & 0x01)
						break;
					lock_info.cat1_working = 1;
          queue_de(pcat1_handle, &lock_info.data);
          if (xTimerIsTimerActive(SLEEP_TIMER_EVT_ID))
					{
						xTimerStop(SLEEP_TIMER_EVT_ID, 0);
					}          
					if (lock_info.nb_flag & 0x01)
					{
						UartEn(true);
						lock_info.nb_flag = 0;
					}	
					GPIO_Pin_Clear(U32BIT(CAP_BOOST));					
					if (p_cat1_info->register_status & 0x01)
					{
						DBGPRINTF("Lock LOCK_TIGHTEND_MSG_ID_EVT \r\n");
						p_cat1_info->send_lock_cmd();
					}
					else
					{
						DBGPRINTF("tension delay send at cmd\r\n");
            ec800g_power_on(); 
						xTimerStart(NB_DELAY_TIMER_EVT_ID, 0);
					}  					
					break;
        //拆卸电池检测
        case BATTERY_DETECT_MSG_ID_EVT:
					DBGPRINTF("battery detect msg\r\n");
          lock_info.irq_flags = 1;
          {
            lock_handle_t data;
            data.opcode = ALARM_TIRGGER;
            data.state = 0x03;
            queue_en(pcat1_handle, &data);
          }          
					if (lock_info.cat1_working & 0x01)
						break;
					lock_info.cat1_working = 1;
          io_irq_disable(U32BIT(BAT_DET));
          wake_up_souce_cfg(2, 0);
          queue_de(pcat1_handle, &lock_info.data);
					if (xTimerIsTimerActive(SLEEP_TIMER_EVT_ID))
					{
						xTimerStop(SLEEP_TIMER_EVT_ID, 0);
					}          
					if (lock_info.nb_flag & 0x01)
					{
						UartEn(true);
						lock_info.nb_flag = 0;
					}
					GPIO_Pin_Clear(U32BIT(CAP_BOOST));					
					if (p_cat1_info->register_status & 0x01)
					{
						DBGPRINTF("battery detect ALARM_TIRGGER \r\n");
						p_cat1_info->send_lock_cmd();
					}
					else
					{
						DBGPRINTF("battery delay send at cmd\r\n");
            ec800g_power_on();
						xTimerStart(NB_DELAY_TIMER_EVT_ID, 0);
					}          
          break;
				default:
					break;
			}
		}
	}
}

void vTaskSendQueueEvt(queue_msg_hdl_t hdl, msg_id_t msg_id, uint16_t param, void *context)
{
	queue_msg_t msg;
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	msg.msg_id = msg_id;
	msg.param1 = param;
	msg.param2 = context;
	
	if (__get_IPSR() != 0)
	{
		xQueueSendFromISR(queue_msg_hal_array[hdl], &msg, &xHigherPriorityTaskWoken);
	}
	else
	{
	  xQueueSend(queue_msg_hal_array[hdl], &msg, 0);
	}
}
	
void vStartThreadTasks( void )
{
	BaseType_t xReturn = pdPASS;/* Create Result*/

//	taskENTER_CRITICAL();
	xReturn = xTaskCreate(KeyHandle_Task, "KeyHandle_Task", configMINIMAL_STACK_SIZE, NULL, 7, (xTaskHandle *) NULL);
	if(pdPASS == xReturn) 
		DBGPRINTF(("Create KeyHandle_Task Success!\r\n"));

	xReturn = xTaskCreate(SofttimerTask, "SofttimerTask", configMINIMAL_STACK_SIZE, NULL, 5, (xTaskHandle *) NULL);
	if(pdPASS == xReturn) 
		DBGPRINTF(("Create SofttimerTask Success!\r\n"));
		
	xReturn = xTaskCreate(UartHandle_Task, "Uart0Handle_Task", configMINIMAL_STACK_SIZE, NULL, 6, (xTaskHandle *) NULL);
	if(pdPASS == xReturn) 
		DBGPRINTF(("Create Uart0Handle_Task Success!\r\n"));

	DBGPRINTF(("SYD8811 freeRTOS Start.\r\n"));
//	taskEXIT_CRITICAL();
}

void test(void)
{
	DBGPRINTF(("hello timer\r\n"));
}
/*-----------------------------------------------------------*/
int main( void )
{	
	__disable_irq();
	MCUClockSwitch(SYSTEM_CLOCK_64M_RCOSC);
	RCOSCCalibration();
#ifdef USER_32K_CLOCK_RCOSC
	ClockSwitch(SYSTEM_32K_CLOCK_RCOSC);
	GAPBBDelayMS(100);
	LPOCalibration();
#else
	ClockSwitch(SYSTEM_32K_CLOCK_XOSC);
#endif
	DebugLogInit();
	board_init();
	hw_gpio_init();
	flash_data_init();
	DBGPRINTF("hello SYD8811 hardware = 0x%02x software = 0x%04x\r\n", flash_data.update_hw, flash_data.update_sw);
	communicate_init();
	ble_init();
	uart0_init();
	uart1_init();
	UartEn(false);
  wake_up_souce_cfg(0,1);

	timers_init();
	rtcs_init();
	adc_init();
//	pwm_init();
	__enable_irq();
	channel = 8;
	GPADC_channel_sel(channel);		
	GPADC_start();
	RFSleep();
	gpio_pin_sleep(LOW_POWER);
   
	p_cat1_info = cat1_init();
	lock_info.nb_flag = 1;
	lock_info.sleep_flag = 1; 
	io_irq_enable((U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS) | U32BIT(BAT_DET)), &gpio_callback);
	vStartThreadTasks();	
	
	/* Start the scheduler. */
	vTaskStartScheduler();
	/* Will only get here if there was not enough heap space to create the
	idle task. */
	return 0;
}

