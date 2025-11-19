#include "gpadc.h"
#include "debug.h"
#include "gpio.h"
#include "lock.h"
#include "stdlib.h"

GPADC_CTRL_TYPE * GPADC	= ((GPADC_CTRL_TYPE *)	GPADC_CTRL_BASE);
uint8_t channel = 0;

extern lock_info_t lock_info;
extern flash_data_t flash_data;

/**********************************************************
		GPIO2				ADCGP_CH[0]
		GPIO3				ADCGP_CH[1]
		GPIO4				ADCGP_CH[2]
		GPIO5				ADCGP_CH[3]
		GPIO28			ADCGP_CH[4]
		GPIO29			ADCGP_CH[5]
		GPIO30			ADCGP_CH[6]
		GPIO31			ADCGP_CH[7]
		VBAT				ADCGP_CH[8]
		VDVDD_1.25	ADCGP_CH[9]
		VDCDC_1.1		ADCGP_CH[10]	
***********************************************************/
void GPADC_Init(GPADC_MODE mode)
{
//-----------------pin config-------------------
#if 1
	//ADCGP_CH[0] Pin Init
	PIN_CONFIG->PIN_2_SEL = PIN_SEL_GPIO;
//	GPIO_CTRL->GPIO_2_DIR = 1;
//	GPI_CTRL->GPI_2_InE = 0;
//	PIN_CONFIG->PAD_2_INPUT_PULL_UP = 0;	  //上拉禁止
	
//	PIN_CONFIG->PIN_2_SEL = PIN_SEL_ANALOG;	
#endif
#if 0
	//ADCGP_CH[1] Pin Init
	PIN_CONFIG->PIN_3_SEL = PIN_SEL_GPIO;
	GPIO_CTRL->GPIO_3_DIR = 1;
	GPI_CTRL->GPI_3_InE = 0;
	PIN_CONFIG->PAD_3_INPUT_PULL_UP = 0;	  //上拉禁止
	
	PIN_CONFIG->PIN_3_SEL = PIN_SEL_ANALOG;
#endif
#if 0
	//ADCGP_CH[2] Pin Init
	PIN_CONFIG->PIN_4_SEL = PIN_SEL_GPIO;
	GPIO_CTRL->GPIO_4_DIR = 1;
	GPI_CTRL->GPI_4_InE = 0;
	PIN_CONFIG->PAD_4_INPUT_PULL_UP = 0;	  //上拉禁止
	
	PIN_CONFIG->PIN_4_SEL = PIN_SEL_ANALOG;
#endif
#if 0
	//ADCGP_CH[3] Pin Init
	PIN_CONFIG->PIN_5_SEL = PIN_SEL_GPIO;
	GPIO_CTRL->GPIO_5_DIR = 1;
	GPI_CTRL->GPI_5_InE = 0;
	PIN_CONFIG->PAD_5_INPUT_PULL_UP = 0;	  //上拉禁止
	
	PIN_CONFIG->PIN_5_SEL = PIN_SEL_ANALOG;
#endif

#if 0
	//ADCGP_CH[4] Pin Init
	PIN_CONFIG->PIN_28_SEL = PIN_SEL_GPIO;
	GPIO_CTRL->GPIO_28_DIR = 1;
	GPI_CTRL->GPI_28_InE = 0;
	PIN_CONFIG->PAD_36_INPUT_PULL_UP = 0;	  //上拉禁止
	
	PIN_CONFIG->PIN_28_SEL = PIN_SEL_ANALOG;	
#endif
#if 0
	//ADCGP_CH[5] Pin Init
	PIN_CONFIG->PIN_29_SEL = PIN_SEL_GPIO;
	GPIO_CTRL->GPIO_29_DIR = 1;
	GPI_CTRL->GPI_29_InE = 0;
	PIN_CONFIG->PAD_37_INPUT_PULL_UP = 0;	  //上拉禁止
	
	PIN_CONFIG->PIN_29_SEL = PIN_SEL_ANALOG;	
#endif
#if 0
	//ADCGP_CH[6] Pin Init
	PIN_CONFIG->PIN_30_SEL = PIN_SEL_GPIO;
	GPIO_CTRL->GPIO_30_DIR = 1;
	GPI_CTRL->GPI_30_InE = 0;
	PIN_CONFIG->PAD_38_INPUT_PULL_UP = 0;	  //上拉禁止
	
	PIN_CONFIG->PIN_30_SEL = PIN_SEL_ANALOG;	
#endif
#if 0
	//ADCGP_CH[7] Pin Init
	PIN_CONFIG->PIN_31_SEL = PIN_SEL_GPIO;
	GPIO_CTRL->GPIO_31_DIR = 1;
	GPI_CTRL->GPI_31_InE = 0;
	PIN_CONFIG->PAD_39_INPUT_PULL_UP = 0;	  //上拉禁止
	
	PIN_CONFIG->PIN_31_SEL = PIN_SEL_ANALOG;
#endif

	GPADC->CLKRATE = 31;		// 64M/((31+1)*2)= 1M
	GPADC->DA_GPADC_EN = 1;

	GPADC->CHANNEL_SET_NUM = 1;
	GPADC->SCAN_COUNT = 1;		//只能设置为1
	GPADC_channel_sel(channel);
	GPADC->Continue_Scan = 0;

	//set time delay
	GPADC->START_SETTLE = 24;
	//GPADC->CHANNEL_SETTLE = 1; 	//7.52us
	//GPADC->CHANNEL_SETTLE = 2;	//11.29us
	GPADC->CHANNEL_SETTLE = 4;		//18.81us
	//GPADC->CHANNEL_SETTLE = 63;	//240.71us

	if(mode == AVE_MODE)
	{
		//一次scan采集data_length+1次数据,然后再取均值
		GPADC->AVERAGE = 7;
		GPADC->DATA_LENGTH = 7;
	}
	else
	{
		GPADC->AVERAGE = 1;
		GPADC->DATA_LENGTH = 0;
	}
	GPADC->EVENTS = 0;
	GPADC->INTENSET = 1;		//使能中断，标志位才能置位
	NVIC_EnableIRQ(GPADC_IRQn);
}

void GPADC_start(void)
{
	GPADC->DA_GPADC_EN = 1;
	GPADC->TASK_START = 1;
}

/*************************************************************
Note:
	STOP will reset Continue_Scan register
**************************************************************/
void GPADC_stop(void)
{
	if(GPADC->gpadc_fsm_cs)
	{
		GPADC->TASK_STOP = 1;
		GPADC->INTENCLR = 1;		//disable INT
		GPADC->EVENTS = 0;			//clear INT flag
		while(GPADC->gpadc_fsm_cs);
		
		while(GPADC->EVENTS == 0);
		GPADC->EVENTS = 0;			//clr INT flag	
		GPADC->INTENSET = 1;		//Enable INT
	}
}

uint16_t GPADC_get_value(void)
{
	return GPADC->adc_data_hclk;
}

/****************************************************************************
function: select 1 GPADC channel when GPADC is stopped at oneshot/ave mode
para:
		  ch - 0 ~ 10 select channel 0 ~ channel 10
note:	  select GPADC channel when GPADC is stopped	
*********************************************************************************/
void GPADC_channel_sel(uint8_t ch)
{
	if(ch <= 10)
	{
		GPADC->CHANNEL_SET0 = ch;
		GPADC->CHANNEL_SEL = ch;
	}
}

uint8_t bat_val = 0, bat_cnt = 0, bat_cnt1 = 0;
uint8_t get_battery(float bat)
{
	uint8_t cnt = 0;
  static uint8_t old_bat_val_buf[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	static uint8_t old_bat_cnt = 0;
		
	if(bat > 3.1) 
		bat = 3.1;
	if(bat < 2.5) 
		bat = 2.5;	
		
	bat_val = (uint8_t)((bat - 2.5) * 167);
	if (bat_val >= 100) 
		bat_val = 100;
	
	if(old_bat_cnt < 6)
	{
		old_bat_val_buf[old_bat_cnt] = bat_val;
		old_bat_cnt++;	
		GPADC_start();
	}
	else
	{
		GPADC_stop();
		old_bat_val_buf[old_bat_cnt] = bat_val;
		qsort(old_bat_val_buf, 7, sizeof(uint8_t), compDec);  
		bat_val = old_bat_val_buf[3];
		old_bat_cnt = 0;		
		DBGPRINTF("new_bat_val = %d ", bat_val);	
		//电量需要下降趋势
		if ((flash_data.bat_val == 0) || (flash_data.bat_val == 0xff))
			flash_data.bat_val = bat_val;
		
		if (bat_val >= flash_data.bat_val)
		{
			if (bat_val >= (flash_data.bat_val+20)) //更换电池特殊情况
			{
				bat_cnt1++;
				if (bat_cnt1 >= 2)
				{
					bat_cnt1 = 0;
					flash_data.bat_val = bat_val;
				}
				else {
					bat_val = flash_data.bat_val;
				}
			}
			else{
				bat_cnt1 = 0;
				bat_val = flash_data.bat_val;
			}
		}
		else 
		{
			if ((bat_val+10) <= flash_data.bat_val)
			{
				bat_cnt++;
				if (bat_cnt >= 3)
				{
					bat_cnt = 0;
					flash_data.bat_val = bat_val;
				}
				else{
					bat_val = flash_data.bat_val;
				}
			}
			else if (((bat_val+5) > flash_data.bat_val) && (bat_val < flash_data.bat_val))
			{
				bat_cnt = 0;
				flash_data.bat_val = bat_val;
			}
			else
				bat_cnt = 0;
		}
		DBGPRINTF("bat_val = %d cnt = %d flash_data.bat_val = %d\r\n",bat_val, cnt, flash_data.bat_val);		
	}
	return bat_val;
}

int compDec(const void *a, const void *b)
{
	return   *(uint8_t *)a - *(uint8_t *)b;
}

void GPADC_IRQHandler(void)
{
	if(GPADC->EVENTS)			//使能ADC中断，标志位才能置位
	{
		float vat = 0.0;
		uint16_t adc = GPADC_get_value();	
		GPADC->EVENTS = 0;
		GPADC->DA_GPADC_EN = 0;
		adc = adc & 0xfffe;
		//充电电池  2.5V ~ 3.5v
		//锂亚电池  2.4v ~ 3.6v
		if(channel == 8)   //VBAT通道
		{
			vat = (float)adc*3.6/483;
		}
		else   //GPIO通道
		{
			vat = (float)adc*1.75/1024;  
		}
		lock_info.bat_val = get_battery(vat);
		if (lock_info.bat_val < 10)
			lock_info.bat_val = 10;
		dbg_printf("battery value: adc = %d  vat = %4.3f  bat_val = %d\r\n", adc, vat, lock_info.bat_val);
	}	
}
