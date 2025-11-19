#include "pwm.h"
#include "gpio.h"
#include "debug.h"
#include "led_key.h"
#include "string.h"

//static PWM_CTRL_TYPE *	PWM_0_CTRL	= ((PWM_CTRL_TYPE *)	PWM_0_CTRL_BASE);
//static PWM_CTRL_TYPE *	PWM_1_CTRL	= ((PWM_CTRL_TYPE *)	PWM_1_CTRL_BASE);
//static PWM_CTRL_TYPE *	PWM_2_CTRL	= ((PWM_CTRL_TYPE *)	PWM_2_CTRL_BASE);
//static HPWM_CTRL_TYPE * HPWM_CTRL	= ((HPWM_CTRL_TYPE *)	HPWM_CTRL_BASE);

//#define  PWM_0_CTRL  ((PWM_CTRL_TYPE *)PWM_0_CTRL_BASE)
//#define  PWM_1_CTRL  ((PWM_CTRL_TYPE *)PWM_1_CTRL_BASE)
//#define  PWM_2_CTRL  ((PWM_CTRL_TYPE *)PWM_2_CTRL_BASE)
//#define  HPWM_CTRL   ((HPWM_CTRL_TYPE *)HPWM_CTRL_BASE)

/*
pwm0 初始化函数
参数: PWM_x pwm_x 要初始化的pwm模块 参见PWM_MODE枚举
			PWM_PARAMETER pwm_parameter pwm参数 参见PWM_PARAMETER结构体
举例：	
	PWM_PARAMETER pwm_parameter;
	//Flash mode
	pwm_parameter.MODE=flash_mode;
	pwm_parameter.T1=0x10; 	//16 * 1/32=0.5s 实测0.768563250
	pwm_parameter.T2=0x20;  //32 * 1/32=1s	实测1.585201375
	pwm_parameter.T3=0x0000;
	pwm_parameter.N1=0x80;  //永久重复，没有T3
	pwm_parameter.N2=0x80;  //永久重复
  pwm_enable(PWM_0,pwm_parameter);	
*/

void pwm_enable(PWM_x pwm_x,PWM_PARAMETER pwm_parameter)
{
	PWM_CTRL_TYPE *	PWM_CTRL = 0;
	switch(pwm_x)
	{
		case PWM_0:
			PWM_CTRL = PWM_0_CTRL;
			break;
		case PWM_1:
			PWM_CTRL = PWM_1_CTRL;
			break;
		default:
			PWM_CTRL = PWM_2_CTRL;
			break;
	}

	PWM_CTRL->MODE = pwm_parameter.MODE;
	switch(pwm_parameter.MODE)
	{
		case pwm_mode:
			PWM_CTRL->PWM_M = pwm_parameter.PWM_M;
			PWM_CTRL->PWM_N = pwm_parameter.PWM_N;
			break;
		case flash_mode:
			PWM_CTRL->T1 = pwm_parameter.T1;
			PWM_CTRL->T2 = pwm_parameter.T2;
			PWM_CTRL->T3 = pwm_parameter.T3;
		  PWM_CTRL->N1 = pwm_parameter.N1;
			PWM_CTRL->N2 = pwm_parameter.N2;
			break;
		case breath_mode:
			PWM_CTRL->BR_TH_MAX = pwm_parameter.BR_TH_MAX;
			PWM_CTRL->BR_TH_MIN = pwm_parameter.BR_TH_MIN;
			PWM_CTRL->BR_SP = pwm_parameter.BR_SP;
			PWM_CTRL->T4 = pwm_parameter.T4; 			 
			break;	
		default:
			break;
	}
	PWM_CTRL->PAUS = 1;
	PWM_CTRL->LED_PE = 1;
}

void pwm_disable(PWM_x pwm_x)
{
	switch(pwm_x){
		case PWM_0 :
			PWM_0_CTRL->LED_PE = 0;
			break;
		case PWM_1 :
			PWM_1_CTRL->LED_PE = 0;
			break;
		case PWM_2 :
			PWM_2_CTRL->LED_PE = 0;
			break;
	}
}

/*
HPWM 初始化函数
参数: 
	 HPWM_PARAMETER *parameter, 参见HPWM_PARAMETER结构体
*/
void Hpwm_Init(HPWM_PARAMETER *parameter)
{
	Hpwm_Stop();

	/*设计时钟频率64M*/
	HPWM_CTRL->PRESCALER = parameter->prescaler;
	HPWM_CTRL->MODE = parameter->mode;
	HPWM_CTRL->COUNTER_TOP = parameter->period;

	//dbg_printf("0:POLARITY = 0x%08x\r\n",HPWM_CTRL->POLARITY);
	switch(parameter->channel)
	{
		case HPWM_CH0:
			//PIN_CONFIG->PIN_3_SEL = PIN_SEL_HPWM_CH0;
			HPWM_CTRL->CMP_CH0 = parameter->duty;
			if(parameter->polarity == LOW_FIRST)
				HPWM_CTRL->POLARITY &= 0x0e;
			else
				HPWM_CTRL->POLARITY |= 0x01;		
			break;
			
		case HPWM_CH1:
			//PIN_CONFIG->PIN_4_SEL = PIN_SEL_HPWM_CH1;
			HPWM_CTRL->CMP_CH1 = parameter->duty;
			if(parameter->polarity == LOW_FIRST)
				HPWM_CTRL->POLARITY &= 0x0d;
			else
				HPWM_CTRL->POLARITY |= 0x02;
			break;
			
		case HPWM_CH2:
			//PIN_CONFIG->PIN_5_SEL = PIN_SEL_HPWM_CH2;
			HPWM_CTRL->CMP_CH2 = parameter->duty;
			if(parameter->polarity == LOW_FIRST)
				HPWM_CTRL->POLARITY &= 0x0b;
			else
				HPWM_CTRL->POLARITY |= 0x04;
			break;
			
		default:
			//PIN_CONFIG->PIN_6_SEL = PIN_SEL_HPWM_CH3;
			HPWM_CTRL->CMP_CH3 = parameter->duty;
			if(parameter->polarity == LOW_FIRST)
				HPWM_CTRL->POLARITY &= 0x07;
			else
				HPWM_CTRL->POLARITY |= 0x08;
			break;
	}
	//dbg_printf("1:POLARITY = 0x%08x\r\n",HPWM_CTRL->POLARITY);
#if 0
	dbg_printf("CMP_CH0 = %d\r\n",HPWM_CTRL->CMP_CH0);
	dbg_printf("CMP_CH1 = %d\r\n",HPWM_CTRL->CMP_CH1);
	dbg_printf("CMP_CH2 = %d\r\n",HPWM_CTRL->CMP_CH2);
	dbg_printf("CMP_CH3 = %d\r\n",HPWM_CTRL->CMP_CH3);
#endif

	HPWM_CTRL->EVENTS = HPWM_INT_ALL;		//clr INT
	//HPWM_CTRL->INTEN  = HPWM_INT_ALL;
	//HPWM_CTRL->INTEN  = HPWM_INT_PERIOD;
	//HPWM_CTRL->INTEN  = HPWM_INT_TASK_STOP;
	HPWM_CTRL->INTEN  = HPWM_INT_NONE;
	//NVIC_EnableIRQ(HPWM_IRQn);  安装原来的驱动方式，会在HPWM_IRQHandler中断函数中动态修改占空比，这样不方便测试的时候看结果，这里把中断关闭后，显示的波形就是代码设置的波形
}

void Hpwm_Set_duty(HPWM_CHx channels, uint16_t compare)
{
	if(HPWM_CTRL->MODE == UP_MODE)
	{
		if(compare >= HPWM_CTRL->COUNTER_TOP)	//0 <= compare <= COUNTER_TOP-1
		{
			compare = HPWM_CTRL->COUNTER_TOP - 1;
		}
	}
	else
	{
		if(compare >= HPWM_CTRL->COUNTER_TOP-1)
		{
			compare = HPWM_CTRL->COUNTER_TOP-2; //0 <= compare <= COUNTER_TOP-2
		}
	}

	if(channels & HPWM_CH0)
	{
		HPWM_CTRL->CMP_CH0 = compare;
	}
	if(channels & HPWM_CH1)
	{
		HPWM_CTRL->CMP_CH1 = compare;
	}
	if(channels & HPWM_CH2)
	{
		HPWM_CTRL->CMP_CH2 = compare;
	}
	if(channels & HPWM_CH3)
	{
		HPWM_CTRL->CMP_CH3 = compare;
	}
}

//HPWM有4个通道，但这4个通道共用1个周期计数器
void Hpwm_Set_period(uint16_t period)
{
	if(period >= HPWM_CTRL->CMP_CH0)
	{
		HPWM_CTRL->COUNTER_TOP = period; 
	}
}

void Hpwm_Set_mode(HPWM_mode mode)
{
	HPWM_CTRL->MODE = mode;
}

void Hpwm_Start(void)
{
	HPWM_CTRL->START = 1;
}

void Hpwm_Stop(void)
{
	HPWM_CTRL->STOP = 1;
}

void HPWM_IRQHandler(void)
{
	//动态设置占空比
	static uint16_t compare = 1;	
	dbg_printf("HPWM_IRQHandler\r\n");
	
	if(HPWM_CTRL->EVENTS & HPWM_INT_PERIOD)
	{	
		compare++;
		if(compare == HPWM_CTRL->COUNTER_TOP)
		{	
			compare = 1;			
			dbg_printf("COUNTER_TOP Can stop pwm there!\r\n");
		}
		Hpwm_Set_duty(HPWM_CHALL, compare);
	}
	
	if(HPWM_CTRL->EVENTS & HPWM_INT_TASK_STOP)
	{

	}
	HPWM_CTRL->EVENTS = HPWM_INT_ALL;		//clr INT
}

void LPWM_Init(void)
{
	PWM_PARAMETER pwm_parameter;
	
	//配置晶振 - 默认使用内部32K晶振
#if 1
	//Flash mode
	memset((void *)&pwm_parameter, 0x00, sizeof(PWM_PARAMETER));
	pwm_parameter.MODE = flash_mode; 
	pwm_parameter.T1 = 0x10;		//16 * 1/32=0.5s
	pwm_parameter.T2 = 0x20;		//32 * 1/32=1s
	pwm_parameter.T3 = 0x20;	  //32 * 1/32=1s
	pwm_parameter.N1 = 0x80;	 	//指定flash格式 -> 是否有T3 : 0x80(No T3)
	pwm_parameter.N2 = 0x80;	 	//指定flash重复次数
	pwm_enable(PWM_0, pwm_parameter);
#endif

#if 1
	//Breath mode timing
	pwm_parameter.MODE = breath_mode;
	pwm_parameter.BR_TH_MAX = 20;		//20/2=10ms
	pwm_parameter.BR_TH_MIN = 4;   	//4/2=2ms
	pwm_parameter.T4 = 20;	 				//20/2=10ms
	pwm_parameter.BR_SP = 4;  	    //4/32ms = 1/8ms = 125us
	pwm_enable(PWM_1, pwm_parameter);
#endif

#if 1
	//PWM mode timing 
	pwm_parameter.MODE = pwm_mode;
	pwm_parameter.PWM_N = 125;	//n/32ms
	pwm_parameter.PWM_M = 250;	//m/32ms
	pwm_enable(PWM_2, pwm_parameter);
#endif
}

void HPWM_Handler(void)
{
	HPWM_PARAMETER hpwm;
	//common parameters
#if 1	
	//hpwm.prescaler = HPWM_PERSCALER_1;	//计数频率64M/1，周期 0.015625us = 15ns
	hpwm.prescaler = HPWM_PERSCALER_64;	//计数频率64M/64，周期 1us
	//hpwm.prescaler = HPWM_PERSCALER_32768;	//计数频率64M/32768，周期 512us
	//hpwm.prescaler = HPWM_PERSCALER_4;	//计数频率64M/4，周期 0.0625us
	hpwm.mode = UP_MODE;      //计数方向向上
	hpwm.period = 100;				//100*计数周期 = 100us. 如果是UP_DOWN_MODE,周期会加倍
#else
	hpwm.prescaler = HPWM_PERSCALER_16;	//计数频率25M/1，周期 0.64us
	hpwm.mode = UP_DOWN_MODE;
	hpwm.period = 100;				//100*0.64us = 64us
#endif

	//ch0 parameters
	hpwm.channel = HPWM_CH0;
	hpwm.polarity = HIGH_FIRST;
	hpwm.duty = 50;   //设置占空比50%
	Hpwm_Init(&hpwm);
}

void pwm_init(void)
{	
	PIN_Set_GPIO(U32BIT(CAP_BOOST), PIN_SEL_HPWM_CH0);
	HPWM_Handler();
}
