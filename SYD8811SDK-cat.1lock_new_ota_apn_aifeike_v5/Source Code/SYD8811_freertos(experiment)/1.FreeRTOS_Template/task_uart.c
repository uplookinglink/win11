#include <stdio.h>
#include <string.h>

/*********************************************************************
 * LOCAL PARAMETER
 */

//串口的发送和接收都放一个任务里面？
Task_Struct uartTask;
Char uartTaskStack[UART_TASK_STACK_SIZE];    //手动创建方式为任务分配空间

// Uart configuration
UART_Handle UARTHandle;
UART_Params UARTparams;
uint8_t Uart_TxTempLen;
uint8_t Uart_RxTempBuf[UART_RX_BUF_SIZE];
uint8_t Uart_TxTempBuf[UART_TX_BUF_SIZE];

// Uart -> App  Callback
UartRxBufCallback UartReviceDataCallback;

extern bc26_info_t bc26_info;
extern Queue_Handle appUartMsgQueue;
extern ICall_SyncHandle syncEvent;
extern lock_info_t lock_info;

uint8_t cmd_index;
cmd_info_t g_at_cmd;
nb_recv_t nb_recv;
nb_send_t nb_send;

// Event used to control the UART thread
static Event_Struct uartEvent;
static Event_Handle hUartEvent;
//static Semaphore_Handle  semBinHandle;

Clock_Struct nb_cmd_interval_Clock;
Clock_Struct nb_timeout_periodicClock;
Clock_Struct nb_error_periodicClock;
Clock_Struct nb_powerdown_Clock;

void TaskUART_taskFxn(UArg a0, UArg a1);
void Uart_ReadCallback(UART_Handle handle, void *rxBuf, size_t size);
void Uart_WriteCallback(UART_Handle handle, void *txBuf, size_t size);

/*********************************************************************
 * @fn      TaskUART_createTask
 *
 * @brief   Task creation function for the uart.
 *
 * @param   None.
 *
 * @return  None.
 */
void TaskUART_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = uartTaskStack;
  taskParams.stackSize = UART_TASK_STACK_SIZE;
  taskParams.priority = UART_TASK_PRIORITY;

  Task_construct(&uartTask, TaskUART_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      TaskUART_taskInit
 *
 * @brief   串口初始化
 *
 * @param   None
 *
 * @return  None.
 */
void TaskUART_taskInit(void)
{    
  //发送AT命令的间隔
  Util_constructClock(&nb_cmd_interval_Clock, nb_uart_clockHandler,
                      2000, 0, false, UARTTASK_NB_TX_REPEAT_EVENT);
  
  //发送AT命令超时处理
  Util_constructClock(&nb_timeout_periodicClock, nb_uart_clockHandler,
                      3000, 0, false, UARTTASK_NB_RX_TIMEOUT_EVENT);
   
  //NB出错灯闪
  Util_constructClock(&nb_error_periodicClock, nb_uart_clockHandler,
                      0, 500, false, UARTTASK_NB_ERROR_EVENT);
  //NB复位后再发送关机指令
  Util_constructClock(&nb_powerdown_Clock, nb_uart_clockHandler,
                      6000, 0, false, UARTTASK_NB_POWERDOWN_ENENT);
}

/*********************************************************************
 * @fn      TaskUART_taskFxn
 *
 * @brief   串口任务处理
 *
 * @param   None
 *
 * @return  None.
 */
void TaskUART_taskFxn(UArg a0, UArg a1)
{ 
  //创建事件
  Event_Params evParams;
  Event_Params_init(&evParams);
  Event_construct(&uartEvent, &evParams);
  hUartEvent = Event_handle(&uartEvent);
  
  TaskUART_taskInit();
  uint32_t uart_events;
  while(1)
  {
    uart_events = Event_pend(hUartEvent, Event_Id_NONE, UARTTASK_EVENT_ALL, ICALL_TIMEOUT_FOREVER);
    switch (uart_events)
    {
    case UARTTASK_RX_CALLBACK_EVENT:
      uart_events &= ~UARTTASK_RX_CALLBACK_EVENT;
      UART_read(UARTHandle, Uart_RxTempBuf, UART_RX_BUF_SIZE);
      break;
    //串口发送事件
    case UARTTASK_TX_EVENT:
      uart_events &= ~UARTTASK_TX_EVENT;
      TaskUARTWrite(Uart_TxTempBuf, Uart_TxTempLen);  //串口打印数据
      break;
    //处理错误接收，再次发送信息  
    case UARTTASK_NB_TX_REPEAT_EVENT:
      uart_events &= ~UARTTASK_NB_TX_REPEAT_EVENT;
      repeat_execute(&g_at_cmd);
      break;
     //NB命令发送超时处理
    case UARTTASK_NB_RX_TIMEOUT_EVENT:
      uart_events &= ~UARTTASK_NB_RX_TIMEOUT_EVENT;
      receive_cmd_timeout();
      break;  
    case UARTTASK_NB_ERROR_EVENT:
      uart_events &= ~UARTTASK_NB_ERROR_EVENT;
      nb_error_led();
      break;
    case UARTTASK_NB_POWERDOWN_ENENT:
      uart_events &= ~UARTTASK_NB_POWERDOWN_ENENT;
      nb_power_off();
      break;
      
    default:
      break;
    }
  }
}

/*********************************************************************
 * @fn      Uart_ReadCallback
 *
 * @brief   串口读回调
 *
 * @param   handle -> 串口通道
 *          rxBuf -> 串口接收数据的指针
 *          size -> 串口接收数据的长度
 *
 * @return  None.
 */
void Uart_ReadCallback(UART_Handle handle, void *rxBuf, size_t size)
{   
    //UART_read(UARTHandle, Uart_RxTempBuf, UART_RX_BUF_SIZE);     //再次打开一个串口读
    Event_post(hUartEvent, UARTTASK_RX_CALLBACK_EVENT);
    UartReviceDataCallback(rxBuf, size);  //给app任务一个串口读回调
}

/*********************************************************************
 * @fn      Uart_WriteCallback
 *
 * @brief   串口写回调
 *
 * @param   handle -> 串口通道
 *          txBuf -> 串口发送数据的指针
 *          size -> 串口发送数据的长度
 *
 * @return  None.
 */
void Uart_WriteCallback(UART_Handle handle, void *txBuf, size_t size)
{
  
}

/*********************************************************************
 * @fn      GY_UartTask_Write
 *
 * @brief   串口写函数
 *
 * @param   buf -> 需要写的数据指针
 *          len -> 需要写的数据长度
 *
 * @return  None.
 */
void TaskUARTWrite(uint8_t *buf, uint16_t len)
{
  UART_write(UARTHandle, buf, len);
}

/*********************************************************************
 * @fn      GY_UartTask_Printf
 *
 * @brief   串口写函数（类似系统printf）
 *
 * @param   format -> 不定参数标志位，例如%d,%s等
 *          ... -> 不定参数
 *
 * @return  None.
 */
void TaskUARTPrintf(const char* format, ...)
{
  va_list arg;
  va_start(arg,format);
  uint8_t buf[UART_TX_BUF_SIZE];
  uint16_t len;
  len = vsprintf((char*)buf, format, arg);
  UART_write(UARTHandle, buf, len);
}

//模拟串口调试用
void ScUARTPrintf(const char* format, ...)
{
//  uint8_t buf[UART_TX_BUF_SIZE];
//  va_list arg;
//  va_start(arg,format);
//  uint16_t len;
//  len = vsprintf((char*)buf, format, arg);
//  ScUARTWrite(buf, len);
  return;
}

/*********************************************************************
 * @fn      HwUARTdoWrite
 *
 * @brief   串口写函数（留给APP打印使用）
 *
 * @param   buf -> 数据buf
 *          len -> 数据len
 *          format -> 不定参数标志位，例如%d,%s等
 *          ... -> 不定参数
 *
 * @return  None.
 *
 *          注意：buf与len为字符串打印，format与...为printf打印，不支持同时使用
 */
void TaskUARTdoWrite(uint8_t *buf, uint16_t len, const char* format, ...)
{
  if(buf == NULL)
  {
    va_list arg;
    va_start(arg,format);
    uint8_t pbuf[UART_TX_BUF_SIZE];
    uint16_t plen;
    plen = vsprintf((char*)pbuf, format, arg);
    Uart_TxTempLen = plen;
    memcpy(Uart_TxTempBuf, pbuf, plen);
  }
  else
  {
    Uart_TxTempLen = len;
    memcpy(Uart_TxTempBuf, buf, len);
  }
  Event_post(hUartEvent, UARTTASK_TX_EVENT);
}

/*********************************************************************
 * @fn      GY_UartTask_RegisterPacketReceivedCallback
 *
 * @brief   注册串口接收回调任务（将串口接收的数据传给app任务去处理）
 *
 * @param   callback -> 串口接收数据回调（包括数据buf及len）
 *
 * @return  None.
 */
//注册串口接收回调函数
void UartTask_RegisterPacketReceivedCallback(UartRxBufCallback callback)
{
    UartReviceDataCallback = callback;
}

//发送AT命令，操作NB模块
void send_at_cmd(void)
{
    cmd_index = SUB_SYNC; 
    cmd_next(cmd_index);
}

//串口接收数据处理
void uart_rece_data(uint8_t *buf, uint16_t len)
{
    uint8_t cmd_state = 0;
    if ((nb_recv.len + len) < NB_RX_BUF_SIZE)
    {   
      memcpy(nb_recv.buf + nb_recv.len, buf, len);
      nb_recv.len += len;
    }   
    cmd_state = cmd_is_pass(nb_recv.buf);
    switch (cmd_state)
    {
    case AT_ERROR: //返回错误，打开定时器
        if (g_at_cmd.cmd_try_time <= g_at_cmd.cmd_try_max)  
        {
           Util_startClock(&nb_cmd_interval_Clock);
        } 
        else  //操作成功
        {
           if (g_at_cmd.cmd_action & ACTION_OK_NEXT_ERROR)
           {
              //投机的办法
              cmd_index++;
              cmd_next(cmd_index);
           }
           else 
           {
              //发送容错处理
              BC26_processHandle(AT_ERROR_HAND);
           }
        }
      break;
     
    case AT_SUCCESS:
        //当前命令执行完,防止重复执行
        if (bc26_info.cmd_switch == 0)
        {
            bc26_info.cmd_switch = 1;
            cmd_index++;
            cmd_next(cmd_index);
        }
      break;
    //发送开锁命令  
    case AT_LOCK:
        BC26_processHandle(AT_LOCK);
      break;
    //发送断电命令
    case AT_UNLOCK:
        BC26_processHandle(AT_UNLOCK);
      break;     
    case AT_NON:
      //do nothing
      break;
      
    default:
      break;
    }
}

//超时处理
void receive_cmd_timeout(void)
{
    if (cmd_index == SUB_SYNC)
    {
        if (g_at_cmd.cmd_try_time <= g_at_cmd.cmd_try_max)  
        {
            repeat_execute(&g_at_cmd);
        }
        else
        {
            BC26_processHandle(AT_ERROR_HAND);
        }
    }
    else if (cmd_index >= SUB_CESQ_1)
    {
        if (g_at_cmd.cmd_try_time <= g_at_cmd.cmd_try_max)  
        {
            repeat_execute(&g_at_cmd);
        } 
        else
        {
            //发送错误处理
            BC26_processHandle(AT_ERROR_HAND);
        }
    }
    else 
    {
        //发送错误处理
        BC26_processHandle(AT_ERROR_HAND);
    }
}

static void nb_uart_clockHandler(UArg arg)
{
  // Wake up the uart.
  Event_post(hUartEvent, arg);
}

void uart_open_init(void)
{
    static uint8_t uart_init_flag = 0;
    if (uart_init_flag == 0)
    {
        uart_init();
        UARTHandle = UART_open(Board_UART0, &UARTparams);                   //打开串口通道
        UART_control(UARTHandle, UARTCC26XX_RETURN_PARTIAL_ENABLE, NULL);   //允许接收部分回调
        UART_read(UARTHandle, Uart_RxTempBuf, UART_RX_BUF_SIZE);
        uart_init_flag = 1;
    }
    else
    {       
        PM_SetConstraint();
        uart_init();
        UARTHandle = UART_open(Board_UART0, &UARTparams);                   //打开串口通道
        UART_control(UARTHandle, UARTCC26XX_RETURN_PARTIAL_ENABLE, NULL);   //允许接收部分回调
        UART_read(UARTHandle, Uart_RxTempBuf, UART_RX_BUF_SIZE);
    }
}

void enter_low_power(void)
{
   if (lock_info.low_power == 0)
   {
       lock_info.low_power = 1;
       //复位接收BUF
       nb_recv_reset();  
       UART_readCancel(UARTHandle); 
       UART_close(UARTHandle);
       PM_RelConstraint();
   }
}

void uart_init(void)
{
    UART_init();                                      //初始化模块的串口功能
    UART_Params_init(&UARTparams);                    //初始化串口参数
    UARTparams.baudRate = 115200;                     //串口波特率115200
    UARTparams.dataLength = UART_LEN_8;               //串口数据位8
    UARTparams.stopBits = UART_STOP_ONE;              //串口停止位1
    UARTparams.readDataMode = UART_DATA_BINARY;       //串口接收数据不做处理
    UARTparams.writeDataMode = UART_DATA_TEXT;        //串口发送数据不做处理
    UARTparams.readMode = UART_MODE_CALLBACK;         //串口异步读
    UARTparams.writeMode = UART_MODE_CALLBACK;        //串口异步写
    UARTparams.readEcho = UART_ECHO_OFF;              //串口不回显
    UARTparams.readReturnMode = UART_RETURN_NEWLINE;  //当接收到换行符时，回调
    UARTparams.readCallback = Uart_ReadCallback;      //串口读回调
    UARTparams.writeCallback = Uart_WriteCallback;    //串口写回调    
}