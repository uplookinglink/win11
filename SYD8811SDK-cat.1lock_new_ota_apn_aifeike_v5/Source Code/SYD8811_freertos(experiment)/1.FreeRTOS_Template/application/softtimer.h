#ifndef _SOFTTIMER_H_
#define _SOFTTIMER_H_

#include "ARMCM0.h"

/* timer event num */
#define  EVT_NUM            (0x02)

/* timer id and timer evt interval*/

#define OTA_MANAGE_TIMER_EVT_ID										((uint32_t)0x00000001<<0)
#define OTA_MANAGE_TIMER_EVT_INTERVAL							1000

//#define GREEN_LED_TIMER_EVT_ID										((uint32_t)0x00000001<<2)
#define GREEN_LED_TIMER_EVT_INTERVAL							2000

//#define BLUE_LED_TIMER_EVT_ID											((uint32_t)0x00000001<<4)
#define BLUE_LED_TIMER_EVT_INTERVAL								1000

//#define KEY_DEBOUNCE_TIMER_EVT_ID									((uint32_t)0x00000001<<5)
#define KEY_DEBOUNCE_TIMER_EVT_INTERVAL						(20)

//#define LOCK_DEBOUNCE_TIMER_EVT_ID								((uint32_t)0x00000001<<6)
#define LOCK_DEBOUNCE_TIMER_EVT_INTERVAL					(800)

//#define NB_TIMEOUT_TIMER_EVT_ID										((uint32_t)0x00000001<<0)
#define NB_TIMEOUT_TIMER_EVT_INTERVAL							(3000)

//#define NB_CMD_INTERVAL_TIMER_EVT_ID							((uint32_t)0x00000001<<8)
#define NB_CMD_INTERVAL_TIMER_EVT_INTERVAL				(2000)

//#define NB_ERROR_TIMER_EVT_ID											((uint32_t)0x00000001<<9)
#define NB_ERROR_TIMER_EVT_INTERVAL								(500)

//#define KEY_HANDLE_TIMER_EVT_ID										((uint32_t)0x00000001<<1)
#define KEY_HANDLE_TIMER_EVT_INTERVAL							(400)

//#define NB_DELAY_TIMER_EVT_ID											((uint32_t)0x00000001<<11)
#define NB_DELAY_TIMER_EVT_INTERVAL								(5000)

//#define LED_ONE_SECOND_TIMER_EVT_ID						    ((uint32_t)0x00000001<<12)
#define LED_ONE_SECOND_TIMER_EVT_INTERVAL			    (1000)

#define POWER_DOWN_TIMER_EVT_ID										((uint32_t)0x00000001<<1)
#define POWER_DOWN_TIMER_EVT_INTERVAL							(1)

/* rtc event num */
#define  RTCEVT_NUM			((uint8_t)0x02)

/* rtc id and rtc evt interval*/
#define TASK_WATCHDOG_RTC_EVT_ID  						((uint32_t) 0x00000001<<0)
#define TASK_WATCHDOG_RTC_INTERVAL						14

#define TASK_PERIOD_RTC_EVT_ID  							((uint32_t) 0x00000001<<1)
#define TASK_PERIOD_RTC_INTERVAL							(3600*24)  //300

#define TASK_UPDATE_RTC_EVT_ID  							((uint32_t) 0x00000001<<2)
#define TASK_UPDATE_RTC_INTERVAL							(6)  

#define EVT_ENABLE_MODE     	((uint8_t) 0x01)
#define EVT_DISABLE_MODE    	((uint8_t) 0x00)

#define RTCEVT_ENABLE_MODE 		((uint8_t) 0x01)
#define RTCEVT_DISABLE_MODE 	((uint8_t) 0x00)

struct Array_Node
{  
   uint32_t evt_id; 
};

struct SYD_sysTimer{
	struct Array_Node Evt_Array_Node; 
	void (*Timer_Evt_Handle)(void);//void是函数数组元素返回类型，*是数组参数类型
	uint32_t Evt_Array_Iterval;	 
	uint8_t Evt_Array_On_Off_Mode;
	uint32_t Evt_Array_Trigger_Loop;
	uint32_t Evt_Loop_Start_Point;
};

extern uint32_t TIMER_EVT;
extern void SYD_Timer_Init(uint8_t event_num, struct SYD_sysTimer p_sysTimer[]);
extern void Timer_Evt_Init(void);
extern void Timer_Evt_Start(uint32_t evt_id_para);
extern void Timer_Evt_Stop(uint32_t evt_id_para);
extern void Timer_Evt_Clr(uint32_t evt_id_para);
extern uint8_t Timer_Get_State(uint32_t evt_id_para);
extern void Timer_Evt_Creat(uint32_t evt_id_para,uint32_t evt_interval_para,void *evt_handle_para,uint8_t evt_mode_para);
extern void Timer_Evt_ReStart(uint32_t evt_id_para);
extern void Timer_Evt_ReSet(uint32_t evt_id_para, uint32_t evt_interval_para);

extern void timer_0_enable(uint32_t interval, void *  p_callback); // 32.768KHz
extern void timer_0_disable(void);	
extern void timer_0_delay_32us(uint32_t cnt);

extern void timer_1_enable(uint32_t interval, void *  p_callback); // 32.768KHz
extern void timer_1_disable(void);	
extern void timer_2_enable(uint32_t interval, void *  p_callback); // 32.768KHz
extern void timer_2_disable(void);

extern uint32_t timer_0_get(void);
extern uint32_t timer_1_get(void);
extern uint32_t timer_2_get(void);

typedef enum {
	RTC_INT_CMP0 	= 1,
	RTC_INT_CMP1 	= 2,
	RTC_INT_TICK 	= 4,
	RTC_INT_SECOND 	= 8,
	RTC_INT_ALL  	= 0xF,
	RTC_INT_NUM  	= 4,
}RTC_INT_TYPE;

typedef void (* RTC_IRQ_CALLBACK) (RTC_INT_TYPE type);

#pragma pack(push, 1)
typedef union {
	struct {
		uint8_t second;
		uint8_t minute;
		uint8_t hour;
		uint8_t day;
	}decimal_format;
    uint8_t  u8[4];
    uint32_t u32;
}RTC_TIME_TYPE;
#pragma pack(pop)

struct RTC_Array_Node
{  
	uint32_t RTCEVT_id;
};

struct SYD_HISTORY_SETTING{
	uint32_t SYD_timeSeconds;
};

struct SYD_sysRtc{
	 struct RTC_Array_Node RTCEVT_Array_Node; 
	 void (*RTC_Evt_Handle)(void);
	 uint32_t RTCEVT_Array_Iterval;	 
	 uint8_t RTCEVT_Array_On_Off_Mode;
	 uint32_t RTCEVT_Array_Trigger_Loop;
	 uint32_t RTCEVT_Loop_Start_Point;
};
	
extern uint32_t RTC_RUN(void);
extern uint32_t RTC_EVT;
extern void RTC_SET(uint32_t Iterval);
extern void SYD_RTC_Init(uint8_t event_num, struct SYD_sysRtc p_sysTimer[]);
extern void RTC_EVT_Init(void);
extern void RTC_EVT_Start(uint32_t RTCEVT_id_para);
extern void RTC_EVT_Stop(uint32_t RTCEVT_id_para);
extern void RTC_EVT_Clr(uint32_t RTCEVT_id_para);
extern uint8_t RTC_Get_State(uint32_t RTCEVT_id_para);
extern void RTC_EVT_Creat(uint32_t RTCEVT_id_para,uint32_t RTCEVT_interval_para,void *RTCEVT_handle_para,uint8_t RTCEVT_mode_para);
	
extern void rtc_set_interrupt_callback(RTC_IRQ_CALLBACK cb);
extern void rtc_int_clear(RTC_INT_TYPE type);
extern void rtc_int_enable(RTC_INT_TYPE type);
extern void rtc_int_disable(RTC_INT_TYPE type);
extern void rtc_start(void);
extern void rtc_stop(void);
extern void rtc_clear(void);
extern void rtc_set_prescaler(uint32_t tick, bool adjust_seconds_bit);
extern void rtc_set_seconds_bit(uint32_t order);
extern RTC_TIME_TYPE rtc_get_compare(int id);
extern void rtc_set_compare(int id, RTC_TIME_TYPE *time);
extern RTC_TIME_TYPE rtc_get_calendar(void);
extern void rtc_set_calendar(RTC_TIME_TYPE *time);
extern bool rtc_status(void);
extern uint32_t rtc_interrupt_status(void);
extern void rtc_init(uint32_t tick,RTC_IRQ_CALLBACK cb);
extern void RTC_EVT_whole_minute_setid(uint32_t RTCEVT_id_para);
extern void RTC_EVT_whole_minute_adj(void);

void timers_init(void);
void rtcs_init(void);

void app_sleep_timer_evt_irq_handle(void);
#endif
