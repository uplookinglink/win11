#ifndef _LED_KEY_H_
#define _LED_KEY_H_

#include "ARMCM0.h"

#define BAT_DET 								GPIO_2   //拆卸电池检测
#define PWR_CTRL								GPIO_3	 //PWM控制电源

#define GNSS_PWR_EN							GPIO_5   //GNSS天线电源使能
#define HALL_DET								GPIO_7   //霍尔元件检测  输入浮空  上升沿触发

#define LOCK_STATUS							GPIO_8   //锁梁检测，输入浮空  下降沿触发
#define MOTOR_STATUS_CLOSE			GPIO_9   //关锁检测，下降沿触发
#define MOTOR_STATUS_OPEN				GPIO_10  //开锁检测，下降沿触发

#define WakeUp_KEY							GPIO_11  //按键检测

#define MotorCtrl_1							GPIO_12  //马达控制1
#define MotorCtrl_2 						GPIO_13  //马达控制2

#define UART0_TX           			GPIO_14  //TXD
#define UART0_RX           			GPIO_15  //RXD

#define GREEN_LED								GPIO_16  //输出低、绿灯亮
#define BLUE_LED								GPIO_18  //输出低、蓝灯亮

#define CAP_BOOST								GPIO_27  //高能电容

#define CAT1_PWR_CTRL						GPIO_4   //高电平导通
#define CAT1_PWR_KEY						GPIO_17  //开机键
#define CAT1_WAKE_UP						GPIO_19  //唤醒键

#define MCU_RESET								GPIO_21  //MCU的管脚需要配置
#define WatchDog								GPIO_31  //喂狗，输出低

#endif
