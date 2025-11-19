#ifndef TASK_UART_H
#define TASK_UART_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "Cat1.h"
  
// Task configuration
#define UART_TASK_PRIORITY                     2
#define UART_TASK_STACK_SIZE                   896
  
//buf size
#define UART_RX_BUF_SIZE                       160
#define UART_TX_BUF_SIZE                       10
  
#define UARTTASK_TX_EVENT                 Event_Id_16
#define UARTTASK_RX_CALLBACK_EVENT        Event_Id_17
#define UARTTASK_NB_TX_REPEAT_EVENT       Event_Id_18
#define UARTTASK_NB_RX_TIMEOUT_EVENT      Event_Id_19
#define UARTTASK_NB_ERROR_EVENT           Event_Id_20
#define UARTTASK_NB_POWERDOWN_ENENT       Event_Id_21
  
#define UARTTASK_EVENT_ALL     (UARTTASK_TX_EVENT           |             \
                                UARTTASK_RX_CALLBACK_EVENT  |             \
                                UARTTASK_NB_TX_REPEAT_EVENT |             \
                                UARTTASK_NB_RX_TIMEOUT_EVENT |           \
                                UARTTASK_NB_ERROR_EVENT     |            \
                                UARTTASK_NB_POWERDOWN_ENENT)
  
/*****************************************************
 * 串口任务初始化
*/  

void TaskUART_createTask(void);
void send_at_cmd(void);
void receive_cmd_timeout(void);
/*****************************************************
 * 串口写函数
*/
void TaskUARTdoWrite(uint8_t *buf, uint16_t len, const char* format, ...);

/*****************************************************
 * 串口接收数据回调（包括数据buf及len）
*/
typedef void (*UartRxBufCallback)(uint8_t *buf, uint16_t len);

/*****************************************************
 * 注册串口接收回调任务（将串口接收的数据传给app任务去处理）
*/
void UartTask_RegisterPacketReceivedCallback(UartRxBufCallback callback);

void TaskUARTWrite(uint8_t *buf, uint16_t len);

void TaskUARTPrintf(const char* format, ...);

void ScUARTPrintf(const char* format, ...);

void uart_rece_data(uint8_t *buf, uint16_t len);

void uart_open_init(void);

void enter_low_power(void);

static void nb_uart_clockHandler(UArg arg);

void uart_init(void);

#ifdef __cplusplus
{
#endif // extern "C"

#endif /* task_uart.h */