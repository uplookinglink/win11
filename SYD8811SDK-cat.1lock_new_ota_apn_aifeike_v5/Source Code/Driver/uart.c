#include "uart.h"
#include "gpio.h"
#include "Cat1.h"
#include "main.h"
#include "debug.h"
#include "..\application\softtimer.h"
#include "aes.h"
#include "aes_ecb.h"
#include "led_key.h"

extern TimerHandle_t NB_CMD_INTERVAL_TIMER_EVT_ID;
extern TimerHandle_t MOTOR_TIMEOUT_TIMER_EVT_ID; 
extern cat1_recv_t cat1_recv;
extern cat1_send_t cat1_send;
extern cmd_info_t g_at_cmd;
extern uint8_t cmd_index;
extern lock_info_t lock_info;
extern cat1_info_t *p_cat1_info;
extern aes_ccm_t aesBuf;

otg_recv_t otg_recv;
otg_send_t otg_send;

static UART_CTRL_TYPE * UART_0_CTRL = ((UART_CTRL_TYPE *) UART_0_CTRL_BASE);
static UART_CTRL_TYPE * UART_1_CTRL = ((UART_CTRL_TYPE *) UART_1_CTRL_BASE);

void uart0_init(void)
{
	PIN_CONFIG->PIN_15_SEL = PIN_SEL_UART_RXD0;
	PIN_CONFIG->PIN_14_SEL = PIN_SEL_UART_TXD0; 
		
	PIN_CONFIG->PAD_20_INPUT_PULL_UP = 0;
	PIN_CONFIG->PAD_21_INPUT_PULL_UP = 0;

	UART_0_CTRL->CLK_SEL = 0;		/* 1=32M, 0=16M */
	
	UART_0_CTRL->BAUDSEL = UART_BAUD_115200;
	UART_0_CTRL->FLOW_EN = UART_RTS_CTS_DISABLE;
	
	UART_0_CTRL->RX_INT_MASK = 0;	/* 1=MASK */
	UART_0_CTRL->TX_INT_MASK = 1;	/* 1=MASK */
	
	UART_0_CTRL->PAR_FAIL_INT_MASK = 1; /* 1=MASK */
	UART_0_CTRL->par_rx_even = 1;	/* 1=Even, 0=Odd */
	UART_0_CTRL->par_rx_en = 0;		/* 1=Rx Parity check enable */

	UART_0_CTRL->par_tx_even = 1;	/* 1=Even, 0=Odd */
	UART_0_CTRL->par_tx_en = 0;		/* 1=Tx Parity check enable */

	//clr Int Flag
	UART_0_CTRL->RI = 0;
	UART_0_CTRL->TI = 0;
	UART_0_CTRL->PAR_FAIL = 1;
	UART_0_CTRL->RX_FLUSH = 1;		/* clr rx fifo and set RX_FIFO_EMPTY */

	NVIC_EnableIRQ(UART0_IRQn);
	UART_0_CTRL->UART_EN = 1;
}

/****************************************************************************
UART0串口写函数
参数: uint8_t data 要发送的数据
注意: 
	  使用这个函数不可使能Tx中断
*****************************************************************************/
void uart0_write_one(uint8_t data)
{
	UART_0_CTRL->TX_DATA = data;
	while(UART_0_CTRL->TI == 0);
	UART_0_CTRL->TI = 0;
}

void uart0_write(uint8_t *buf, uint16_t len)
{
	uint16_t i = 0; 
	for(i = 0; i < len; i++)
	{
		uart0_write_one(buf[i]);
	}
}
/****************************************************************************
UART0串口读Rx FIFO函数
参数:
	pcnt - 返回读到的字节数
	pbuf - 指向数据存放的buf
注意: FIFO长度为4，要求接收buf大小至少为4字节
****************************************************************************/
void uart0_read(uint8_t *pcnt, uint8_t *pbuf)
{
	uint8_t i = 0;
	volatile uint8_t dly = 0;		//如果用64M时钟，必须要加delay
	
	while(!UART_0_CTRL->RX_FIFO_EMPTY)
	{
		*(pbuf+i) = UART_0_CTRL->RX_DATA;
		i++;		
		//延时400ns以上
		dly++;
		dly++;
		dly++;		
	}
	*pcnt = i;
}

void uart0_close(void)
{
	UART_0_CTRL->UART_EN = 0;
	NVIC_DisableIRQ(UART0_IRQn);
}


/*****************************************************************************
UART0串口中断函数
注意:
	  RX_FLUSH只能置位RX_FIFO_EMPTY,不能清除RI
*****************************************************************************/
void UART0_IRQHandler(void)
{
	if ((UART_0_CTRL->RI) == 1)
	{
		UART_0_CTRL->RI = 0;		//先清标志，再读数据			
		UART0_RxCpltCallback();
	}	
	
#if 1
	if ((UART_0_CTRL->TI) == 1)
	{
		UART_0_CTRL->TI = 0;
	}
#endif
	if((UART_0_CTRL->PAR_FAIL) == 1)
	{
		UART_0_CTRL->PAR_FAIL = 1;
	}
}

void UART0_RxCpltCallback(void)
{
	uint8_t cnt = 0;
	uint8_t tmp[4] = {0};
	uart0_read(&cnt, tmp);
	if (cat1_recv.len >= CAT1_RX_BUF_SIZE)
	{
		memset(&cat1_recv, 0, sizeof(cat1_recv_t));
		return;
	}
	cat1_recv.buf[cat1_recv.len++] = tmp[0];
	
	if(((uint8_t)cat1_recv.buf[cat1_recv.len-2] == 0x0d) && ((uint8_t )cat1_recv.buf[cat1_recv.len-1] == 0x0a)) 
	{	
		vTaskSendQueueEvt(UART_QUE_HDL, UART0_REV_MSG_ID_EVT, 0xBB, NULL);	
//		memset(&bc26, 0, sizeof(nb_recv_t));
	}
}

void uart1_init(void)
{
	PIN_CONFIG->PIN_26_SEL = PIN_SEL_UART_RXD1;
	PIN_CONFIG->PIN_25_SEL = PIN_SEL_UART_TXD1; 
		
	PIN_CONFIG->PAD_33_INPUT_PULL_UP = 1;
	PIN_CONFIG->PAD_34_INPUT_PULL_UP = 1;

	UART_1_CTRL->CLK_SEL = 0;		/* 1=32M, 0=16M */
	
	UART_1_CTRL->BAUDSEL = UART_BAUD_115200;
	UART_1_CTRL->FLOW_EN = UART_RTS_CTS_DISABLE;
	
	UART_1_CTRL->RX_INT_MASK = 0;	/* 1=MASK */
	UART_1_CTRL->TX_INT_MASK = 1;	/* 1=MASK */
	
	UART_1_CTRL->PAR_FAIL_INT_MASK = 1; /* 1=MASK */
	UART_1_CTRL->par_rx_even = 1;	/* 1=Even, 0=Odd */
	UART_1_CTRL->par_rx_en = 0;		/* 1=Rx Parity check enable */

	UART_1_CTRL->par_tx_even = 1;	/* 1=Even, 0=Odd */
	UART_1_CTRL->par_tx_en = 0;		/* 1=Tx Parity check enable */

	//clr Int Flag
	UART_1_CTRL->RI = 0;
	UART_1_CTRL->TI = 0;
	UART_1_CTRL->PAR_FAIL = 1;
	UART_1_CTRL->RX_FLUSH = 1;		/* clr rx fifo and set RX_FIFO_EMPTY */

	NVIC_EnableIRQ(UART1_IRQn);
	UART_1_CTRL->UART_EN = 1;
}

/****************************************************************************
UART1串口写函数
参数: uint8_t data 要发送的数据
注意: 
	  使用这个函数不可使能Tx中断
*****************************************************************************/
void uart1_write_one(uint8_t data)
{
	UART_1_CTRL->TX_DATA = data;
	while(UART_1_CTRL->TI == 0);
	UART_1_CTRL->TI = 0;
}

void uart1_write(uint8_t *buf, uint16_t len)
{
	uint16_t i = 0; 
	for(i = 0; i < len; i++)
	{
		uart1_write_one(buf[i]);
	}
}
/****************************************************************************
UART1串口读Rx FIFO函数
参数:
	pcnt - 返回读到的字节数
	pbuf - 指向数据存放的buf
注意: FIFO长度为4，要求接收buf大小至少为4字节
****************************************************************************/
void uart1_read(uint8_t *pcnt, uint8_t *pbuf)
{
	uint8_t i = 0;
	volatile uint8_t dly = 0;		//如果用64M时钟，必须要加delay
	
	while(!UART_1_CTRL->RX_FIFO_EMPTY)
	{
		*(pbuf+i) = UART_1_CTRL->RX_DATA;
		i++;		
		//延时400ns以上
		dly++;
		dly++;
		dly++;		
	}
	*pcnt = i;
}

void uart1_close(void)
{
	UART_1_CTRL->UART_EN = 0;
	NVIC_DisableIRQ(UART1_IRQn);
}

/*****************************************************************************
UART1串口中断函数
注意:
	  RX_FLUSH只能置位RX_FIFO_EMPTY,不能清除RI
*****************************************************************************/
void UART1_IRQHandler(void)
{
	if ((UART_1_CTRL->RI) == 1)
	{
		UART_1_CTRL->RI = 0;		//????,????			
		UART1_RxCpltCallback();
	}	
	
#if 1
	if ((UART_1_CTRL->TI) == 1)
	{
		UART_1_CTRL->TI = 0;
	}
#endif
	if((UART_1_CTRL->PAR_FAIL) == 1)
	{
		UART_1_CTRL->PAR_FAIL = 1;
	}
}

void UART1_RxCpltCallback(void)
{
	uint8_t cnt = 0;
	uint8_t tmp[4] = {0};
	uart1_read(&cnt, tmp);
	
	if (otg_recv.len >= OTG_RX_BUF_SIZE)
	{ 
		memset(&otg_recv, 0, sizeof(otg_recv_t));
		DBGPRINTF("111\r\n");
	}
	else 
	{
		if((otg_recv.len == 0) && (tmp[0] != 0xf0)) 
		{
			return;
		}
		otg_recv.buf[otg_recv.len++] = tmp[0];
		if(((uint8_t)otg_recv.buf[otg_recv.len-2] == 0xe0) && ((uint8_t )otg_recv.buf[otg_recv.len-1] == 0x0e)) 
		{	
			vTaskSendQueueEvt(UART_QUE_HDL, UART1_REV_MSG_ID_EVT, 0xBB, NULL);	
		}
	}
}

//串口0接收数据处理
void uart0_recv_data()
{
	uint8_t cmd_state = 0; 
	cmd_state = p_cat1_info->cmd_is_pass(&cat1_recv);
	switch (cmd_state)
	{
	case AT_ERROR: //返回错误，打开定时器
    DBGPRINTF("U+AT_ERROR %s\r\n", cat1_recv.buf);
		if (g_at_cmd.cmd_try_time <= g_at_cmd.cmd_try_max)  
		{
			 xTimerStart(NB_CMD_INTERVAL_TIMER_EVT_ID, 0);
		} 
		else  //操作成功
		{
			if (g_at_cmd.cmd_action & ACTION_OK_NEXT_ERROR)
			{
				//投机的办法
				cmd_index++;
				p_cat1_info->cmd_next(cmd_index);
			}
			else if(cmd_index == SUB_QGPSGNMEA)//没成功获取到地址，跳过位置上报
			{
				cmd_index+=2;
				p_cat1_info->cmd_next(cmd_index);
			}
			else
			{
				//发送容错处理
				p_cat1_info->process_handle(AT_ERROR_HAND);
			}
		}
		break;
	 
	case AT_SUCCESS:
		DBGPRINTF("U+AT_SUCCESS %d\r\n", cmd_index);
		cmd_index++;
		p_cat1_info->cmd_next(cmd_index);
	
//	if (cmd_index == SUB_ICCID)
//	{
//		cmd_index = SUB_QGPS_READ;
//		p_cat1_info->cmd_next(cmd_index);
//	}else{	 
//		cmd_index++;
//		p_cat1_info->cmd_next(cmd_index);
//	}
		break;
	//发送命令 
	case AT_LOCK:
		DBGPRINTF("AT_LOCK\r\n");
		p_cat1_info->process_handle(AT_LOCK);
		break;
	//发送开锁命令
	case AT_UNLOCK:
		DBGPRINTF("AT_UNLOCK\r\n");
		p_cat1_info->process_handle(AT_UNLOCK);
		break; 
	//ftp升级开始
	case AT_UPDATE_START:
		DBGPRINTF("AT_UPDATE_START %d\r\n", cmd_index);
	  lock_info.update_flags = 0x01;
		cmd_index = SUB_QIOPEN;
		p_cat1_info->cmd_next(cmd_index);			 
		break;
	//ftp升级中
	case AT_UPDATEING:
		DBGPRINTF("AT_UPDATEING %d\r\n", cmd_index);
		cmd_index--;
		p_cat1_info->cmd_next(cmd_index);		
		break;
	case AT_NON:
		//do nothing
		break;
		
	default:
		break;
	}
}

void UartHandle_Task(void *pvParameters)
{	
	queue_msg_t evt;
	queue_msg_hal_array[UART_QUE_HDL] = xQueueCreate(5, sizeof(queue_msg_t));
	
	for( ;; )
	{
		if(xQueueReceive(queue_msg_hal_array[UART_QUE_HDL], &evt, portMAX_DELAY))
		{
			switch ((uint8_t)evt.msg_id)
			{
				case UART0_REV_MSG_ID_EVT:
//					DBGPRINTF("%s\r\n", cat1_recv.buf);
					uart0_recv_data();
					break;
				case UART0_NB_DELAY_MSG_ID_EVT:
					DBGPRINTF("start send at cmd\r\n");
					send_at_cmd();
					break;
				//处理错误接收，再次发送数据
				case UART0_NB_REPEAT_MSG_ID_EVT:
					p_cat1_info->repeat_execute(&g_at_cmd);
					break;
				//NB发送命令超时处理
				case UART0_NB_REV_TIMEOUT_MSG_ID_EVT:
					DBGPRINTF("receive is timeout\r\n");
					receive_cmd_timeout();
					break; 
				//NB出错
				case UART0_NB_ERROR_MSG_ID_EVT:
					DBGPRINTF("cat.1 is err\r\n");
					cat1_error_led();
					break;
				case UART0_NB_POWERDOWN_MSG_ID_EVT:
					cmd_index = SUB_QIOPEN; 
					p_cat1_info->cmd_next(cmd_index);
					break;	
				//4G升级校验失败后，再次获取升级文件
				case UART0_UPDATE_MSG_ID_EVT:
					cmd_index = SUB_QFSEEK_START; 
					p_cat1_info->cmd_next(cmd_index);					
					break;
				//串口1接收处理
				case UART1_REV_MSG_ID_EVT:
					DBGPRINTF("UART1_REV_MSG_ID_EVT\r\n");
					uart1_recv_data();
					break;
				default:
					break;
			}
		}
	}	
}

//串口1接收数据处理
void uart1_recv_data(void)
{
	uint8_t i = 0, j = 0, index = 0, checksum = 0;
	hexprint((uint8_t *)otg_recv.buf, otg_recv.len);
	
	for (i = 0; i < otg_recv.len; i++)
	{
		if ((otg_recv.buf[i] == 0xf0) && (otg_recv.buf[i+1] == 0x0f))
			break;
	}
	j = i;
	memset(&otg_send, 0, sizeof(otg_send_t));	
	switch (otg_recv.buf[j+2])
	{
		//获取随机数 f00f10010012e00e   F00F1104694B90C375E00E
		case OTG_RECV_RANDOM_NUM_CMD: 
				DBGPRINTF("OTG_RECV_RANDOM_NUM_CMD\r\n");
				//填充包头
				otg_send.buf[otg_send.len++] = 0xf0;
				otg_send.buf[otg_send.len++] = 0x0f;
				otg_send.buf[otg_send.len++] = OTG_SEND_RANDOM_NUM_CMD;
				index = otg_send.len;
				otg_send.buf[otg_send.len++] = 0x04;
				memcpy(&otg_send.buf[otg_send.len], (uint8_t *)&aesBuf.random , 4);
				otg_send.len += 4;
				for (i = 0; i < 5; i++)
				{
					checksum ^= otg_send.buf[index+i];
				}
				otg_send.buf[otg_send.len++] = checksum;
				otg_send.buf[otg_send.len++] = 0xe0;
				otg_send.buf[otg_send.len++] = 0x0e;
				taskENTER_CRITICAL();
				uart1_write((uint8_t *)otg_send.buf, otg_send.len);
				taskEXIT_CRITICAL();
				hexprint((uint8_t *)otg_send.buf, otg_send.len);
			break;
		//开锁指令  f00f1214D1B2E6EC6D196135836F9EEF8E0A0AD759A70D0013e00e  
		//f00f120f016E040808060600000000000000000013e00e
		case OTG_RECV_UNLOCK_CMD:
		{
			uint32_t readPwd; uint32_t flashPwd;
			uint8_t uart_rx[20] = {0x00};
			uint8_t ret = 0;
			memcpy(uart_rx, &otg_recv.buf[4], 20);	
			hexprint(uart_rx,20);
			DBGPRINTF("OTG_RECV_UNLOCK_CMD\r\n");
#if 1					
			ret = app_aes_ccm_decrypt(aesBuf.plaintext, uart_rx);	////解密RX
			//打印接收到的数据
			hexprint(aesBuf.plaintext,16);	
#else
			ret = 1;
			memcpy(aesBuf.plaintext, uart_rx, 16);
#endif 
			if(ret)
			{
			  revmemcpy((uint8_t *)&readPwd, aesBuf.plaintext+3, 4);
			  revmemcpy((uint8_t *)&flashPwd, (uint8_t *)lock_info.LockPwd, 4);
			  DBGPRINTF("pwd %x\r\n",readPwd);
			  if(readPwd == flashPwd) 
				{
					lock_info.open_flags = 1;
				  lock_info.work_mode = OTG_MODE;
					GPIO_Pin_Clear(U32BIT(BLUE_LED));
					xTimerStart(MOTOR_TIMEOUT_TIMER_EVT_ID, 0);
					lock_info.motor_status = MOTOR_OPEN;
					motor_forward();
				}
			}
		}
			break;
		default:
			break;
	}
	memset(&otg_recv, 0, sizeof(otg_recv_t));
}

//超时处理
void receive_cmd_timeout(void)
{
    if ((cmd_index == SUB_SYNC) || (cmd_index == SUB_CEREG))
    {
        if (g_at_cmd.cmd_try_time <= g_at_cmd.cmd_try_max)  
        {
            p_cat1_info->repeat_execute(&g_at_cmd);
        }
        else
        {
            p_cat1_info->process_handle(AT_ERROR_HAND);
        }
    }
		else if (cmd_index == SUB_QICSGP)
		{
			if (strstr(cat1_recv.buf, "OK"))
			{
				cmd_index++;
				p_cat1_info->cmd_next(cmd_index);
			}
			else
			{
				p_cat1_info->process_handle(AT_ERROR_HAND);
			}			
		}
    else if (cmd_index == SUB_QISEND) //开锁请求上报3条，锁状态上报2条，周期上报与撬锁告警上报2条
    {
        lock_info.timeout_cnt++;
				switch(lock_info.data.opcode)
				{
					case BOOT_TIRGGER:
					case LOCK_TIRGGER:
						if (lock_info.timeout_cnt < 3)   //考虑功耗重发1次，总共发2次
						{
								p_cat1_info->cmd_next(cmd_index);
						} 
						else
						{
								if (strstr(cat1_recv.buf, "SEND OK"))
								{
									 p_cat1_info->process_handle(AT_LOCK);
								}
								else
								{
									p_cat1_info->process_handle(AT_ERROR_HAND);
								}
						}	
						break;
					case PERIOD_TIRGGER:
					case ALARM_TIRGGER:
					case TIGHTEND_TIRGGER:
						if (lock_info.timeout_cnt < 2)   //考虑功耗重发1次，总共发2次
						{
								p_cat1_info->cmd_next(cmd_index);
						} 
						else
						{
								if (strstr(cat1_recv.buf, "SEND OK"))
								{
									cmd_index++;
									p_cat1_info->cmd_next(cmd_index);
								}
								else
								{
									p_cat1_info->process_handle(AT_ERROR_HAND);
								}
						}		
						break;
				}
    }
		else if (cmd_index == SUB_GPS_QISEND)
		{
				if (strstr(cat1_recv.buf, "SEND OK"))
				{
					 p_cat1_info->process_handle(AT_LOCK);
				}
				else
				{
					p_cat1_info->process_handle(AT_ERROR_HAND);
				}		
		}
		else if (cmd_index == SUB_QGPS_READ)
		{			
				if (strstr(cat1_recv.buf, "+QGPS: 1"))
				{
					cmd_index+=2;
					p_cat1_info->cmd_next(cmd_index);		
				}
				else if (strstr(cat1_recv.buf, "+QGPS: 0"))
				{
					cmd_index++;
					p_cat1_info->cmd_next(cmd_index);						
				}					
				else
				{
					p_cat1_info->process_handle(AT_ERROR_HAND);
				}			
		}
		else if (cmd_index == SUB_QGPSGNMEA)
		{
        if (g_at_cmd.cmd_try_time <= g_at_cmd.cmd_try_max+9)  //120
        {
            p_cat1_info->repeat_execute(&g_at_cmd);
        }
        else
        {
            cmd_index+=2;
						p_cat1_info->cmd_next(cmd_index);
        }			
		}
		//有时候出现，成功返回，但是却报错的情况，所以加个流程
		else if (cmd_index == SUB_QIOPEN)
		{
			cmd_index++;
			p_cat1_info->cmd_next(cmd_index);				
		}
		else if (cmd_index == SUB_QFREAD_POLL)
		{			
			p_cat1_info->cmd_next(cmd_index);	
		}
//    //针对个别重复触发串口中断问题
//    else if ((cmd_index == SUB_SUCCESS) || (cmd_index == SUB_END))
//    {
//      //do nothing
//    }
    else 
    {
        p_cat1_info->process_handle(AT_ERROR_HAND);
    }
}

//发送AT命令，操作NB模块
void send_at_cmd(void)
{
  lock_info.sleep_flag = 0;
	cmd_index = SUB_SYNC; 
	p_cat1_info->cmd_next(cmd_index);
}
