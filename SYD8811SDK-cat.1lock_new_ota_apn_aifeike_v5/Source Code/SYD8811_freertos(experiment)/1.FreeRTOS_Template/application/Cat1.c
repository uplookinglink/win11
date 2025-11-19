#include "Cat1.h"
#include "gpio.h"
#include "uart.h"
#include "led_key.h"
#include "..\application\softtimer.h"
#include "softtimer_task.h"
#include "debug.h"
#include "pwm.h"
#include "main.h"
#include "DebugLog.h"
#include "ble.h"
#include <time.h>
#include "ota.h"

//附着网络
const char *AT_SYNC      = "AT";
const char *AT_CMEE      = "AT+CMEE";
const char *AT_ATE1      = "ATE1";              // 1 打开回显  0 关闭回显
const char *AT_ATE0      = "ATE0"; 
const char *AT_ATI       = "ATI";
const char *AT_CGMI      = "AT+CGMI";
const char *AT_CGMM      = "AT+CGMM";
const char *AT_CGMR      = "AT+CGMR";
const char *AT_CGSN      = "AT+CGSN";
const char *AT_CIMI      = "AT+CIMI";
const char *AT_ICCID     = "AT+ICCID";
const char *AT_CESQ      = "AT+CESQ";
const char *AT_CSQ       = "AT+CSQ";
const char *AT_CGATT     = "AT+CGATT";
const char *AT_CEREG     = "AT+CEREG";
const char *AT_CFUN      = "AT+CFUN";
const char *AT_CGPADDR   = "AT+CGPADDR";
const char *AT_CGREG     = "AT+CGREG";
const char *AT_QLTS			 = "AT+QLTS";
//UDP命令
const char *AT_QICSGP    = "AT+QICSGP";
const char *AT_QIACT     = "AT+QIACT";
const char *AT_QIOPEN    = "AT+QIOPEN";
const char *AT_QISEND    = "AT+QISEND";
const char *AT_QICLOSE   = "AT+QICLOSE";

//LBS命令
const char *AT_QGPSCFG   = "AT+QGPSCFG";
const char *AT_QLBSCFG   = "AT+QLBSCFG";
const char *AT_QLBS      = "AT+QLBS";
//GPS命令
const char *AT_QGPS      = "AT+QGPS"; 
const char *AT_QGPSLOC   = "AT+QGPSLOC";
const char *AT_QGPSEND   = "AT+QGPSEND";
const char *AT_QGPSGNMEA = "AT+QGPSGNMEA";

//FTP升级
const char *AT_QFTPCFG   = "AT+QFTPCFG";
const char *AT_QFTPOPEN  = "AT+QFTPOPEN";
const char *AT_QFTPCWD   = "AT+QFTPCWD";
const char *AT_QFTPSIZE  = "AT+QFTPSIZE";
const char *AT_QFTPGET   = "AT+QFTPGET";
const char *AT_QFTPCLOSE = "AT+QFTPCLOSE";

//文件操作
const char *AT_QFDEL		 = "AT+QFDEL";
const char *AT_QFLDS     = "AT+QFLDS";
const char *AT_QFOPEN    = "AT+QFOPEN";
const char *AT_QFSEEK    = "AT+QFSEEK";
const char *AT_QFREAD    = "AT+QFREAD";

char *udp_head = "1,1,\"UDP\",";
char *udp_tail = ",0,1"; //设置成直吐模式
char ip_buf[50] = {0x00}; 

//固件升級
char filename[30] = {0x00};
char filename_buf[50] = {0x00};
char filehandle_buf[10] = {0x00};
char *filename_com = ",\"COM:\",";
char *filename_end = "0,100";
uint8_t chk_xor = 0;

char file_seek_set_paragram[30] = {0x00};
char file_read_buf[30] = {0x00};
char *filename_sd = ",\"SD:padlock.bin\"";
char *filename_ufs = ",\"UFS:padlock.bin\"";
char *file_open_end = ",2";
char *file_seek_end = ",0,0";
char *file_read_end = ",200";

//UNINET  ctnet 
char *apn_cmnet = "1,1,\"CMNET\",\"\",\"\",1";
char *apn_ctnet = "1,1,\"CTNET\",\"\",\"\",1";
char *apn_uninet = "1,1,\"UNINET\",\"\",\"\",1";

char *apn_symbol = "\",\"";
char *apn_end = "\",1";

//char *apn_test = "1,1,\"zgsytrqxj.iot.xj\",\"\",\"\",1";

//个人：GgP533u185f3247Z
//公司：44732KH1O7OQK9t9
char *token = "\"token\",\"44732KH1O7OQK9t9\"";
char *latorder = "\"latorder\",0";

//UDP服务器端的IP地址与端口号：134.175.50.82 10010  ftp:"152.136.206.32",10021"
//山西世恒 IP=218.26.180.238  port=2028
static char ipaddr[30] = {'"','1','3','4','.','1','7','5','.','5','0','.','8','2','"',',','1','0','0','1','0','\0'};
static char ftpipaddr[30] = {'"','1','5','2','.','1','3','6','.','2','0','6','.','3','2','"',',','1','0','0','2','1','\0'};
//char ipaddr[30] = {'"','2','1','8','.','2','6','.','1','8','0','.','2','3','8','"',',','2','0','2','8','\0'};

extern TimerHandle_t GREEN_LED_TIMER_EVT_ID;
extern TimerHandle_t NB_ERROR_TIMER_EVT_ID;
extern TimerHandle_t NB_TIMEOUT_TIMER_EVT_ID;
extern TimerHandle_t MOTOR_TIMEOUT_TIMER_EVT_ID;
extern TimerHandle_t LED_ONE_SECOND_TIMER_EVT_ID;
extern TimerHandle_t SLEEP_TIMER_EVT_ID;
	
extern lock_info_t lock_info;
extern flash_data_t flash_data;
extern queue_t *pcat1_handle;

cat1_recv_t cat1_recv; 
cat1_send_t cat1_send;
cmd_info_t g_at_cmd;
volatile uint8_t cmd_index;
location_info_t gps_info_old, gps_info;

ftp_update_t ftp_update;

static uint8_t g_send_dataBuf[CAT1_TX_BUF_SIZE] = {0};

uint8_t ble_imei[9] = {0};
uint8_t ble_iccid[10] = {0};
uint8_t cat1_imei[8] = {0};

static char g_location_buf[60] = {0};

static cat1_info_t g_cat1_info;

//命令参数初始化
static void cmd_param_init(CmdHandle cmd_handle, const char *at, char *arg, cmd_type_t type)
{
  if(cmd_handle == NULL)
  {
    return;
  }
  cmd_handle->cmd_try_max = 15;
  cmd_handle->type = type;
  cmd_handle->pArg = arg;
  cmd_handle->timeout = 6000;
  cmd_handle->pCMD = at;
  cmd_handle->cmd_try_time = 0;
  cmd_handle->pExpect_rece = NULL;
  cmd_handle->ok_flag = 0;  //默认OK在后
  cmd_handle->cmd_action = ACTION_OK_EXIT_ERROR;
  return;
}

//获取AT命令
static void cmd_generate(CmdHandle cmd_handle, cat1_send_t *cat1_send)
{
  switch (cmd_handle->type) 
  {
  case CMD_TEST:
    cat1_send->len = snprintf(cat1_send->buf, CAT1_TX_BUF_SIZE, "%s=?\r\n", cmd_handle->pCMD);
    break;
  case CMD_READ:
    cat1_send->len = snprintf(cat1_send->buf, CAT1_TX_BUF_SIZE, "%s?\r\n", cmd_handle->pCMD);
    break;   
  case CMD_EXCUTE:
    cat1_send->len = snprintf(cat1_send->buf, CAT1_TX_BUF_SIZE, "%s\r\n", cmd_handle->pCMD); 
    break;    
  case CMD_SET:
    cat1_send->len = snprintf(cat1_send->buf, CAT1_TX_BUF_SIZE, "%s=%s\r\n", cmd_handle->pCMD, cmd_handle->pArg); 
    break;     
  default:
    break;   
  }
  return;
}

//判断AT命令执行是否成功
static uint8_t ec800g_cmd_is_pass(cat1_recv_t *recv)
{
  uint8_t result = AT_NON; 
  if (g_at_cmd.ok_flag) 
  {
		if (compare_str(recv))	
		{
			if (cmd_index == SUB_QFREAD_START)
			{
				result = AT_SUCCESS;
			}
			else if (cmd_index == SUB_QFREAD_POLL)
			{
				result = AT_UPDATEING;
			}
			else {
				//do nothing
			}			
		}			  
  }
  else
  {
    if(g_at_cmd.pExpect_rece == NULL)
    {
       if (strstr(recv->buf, "OK"))  
       {
          result = AT_SUCCESS;
       }
       else if (strstr(recv->buf, "ERROR"))
       {
          result = AT_ERROR;
       }
       else
       {
          //do nothing
       }    
    }
    else
    {
      if(strstr(recv->buf,"OK"))
      {
        switch (cmd_index)
        {       
          case SUB_CSQ:
            if (strstr(recv->buf, g_at_cmd.pExpect_rece) || strstr(recv->buf, "0,0"))
            {
                result = AT_ERROR;
            }  
            else
            {
                result = AT_SUCCESS;
            }
            break;
          case SUB_CEREG: 
          case SUB_CGATT:
          case SUB_QIOPEN:
					case SUB_QGPS_READ:
					case SUB_QGPSGNMEA:
					case SUB_QFTPOPEN:
					case SUB_QFTPCWD:
					case SUB_QFTPSIZE:
					case SUB_QFTPDOWNLOAD:
            if (strstr(recv->buf, g_at_cmd.pExpect_rece))
            {
                result = AT_SUCCESS;
            }
            break; 
          case SUB_QISEND: 
					case SUB_GPS_QISEND:
            if (strstr(recv->buf, g_at_cmd.pExpect_rece))    
            {
                static uint8_t cnt = 0;
                cnt++;
                if (cnt >= 2)
                {
                    cnt = 0;
										result = g_cat1_info.ops->recv_parse(recv->buf, &flash_data);
                }
            }
            else if (strstr(recv->buf, "FAIL"))
            {
                //发送失败
                result = AT_ERROR;
            }
            else
            {
                //do nothing
            }  
            break; 
          default:
            break;
        }   
      }
    }
  }
  if (strstr(recv->buf, "ERROR"))
  {
		if (cmd_index <= SUB_ATE2)
			result = AT_ERROR;
  }
  return result;
}

//向NB模块发送at指令
static void cat1_send_cmd(CmdHandle cmd_handle)
{
  memset(&cat1_send, 0, sizeof(cat1_send_t)); 
  cmd_generate(cmd_handle, &cat1_send);
  //清空接收缓存
  cat1_recv_reset();
  //串口发送数据
	taskENTER_CRITICAL();
  uart0_write((uint8_t *)cat1_send.buf, cat1_send.len);
	taskEXIT_CRITICAL();
  //开启定时器
  xTimerChangePeriod(NB_TIMEOUT_TIMER_EVT_ID, cmd_handle->timeout, 0);
  cmd_handle->cmd_try_time++;
  return;
}

//设置UDP发送数据长度与IP地址、端口号
static void udp_set_ipaddr(uint8_t *buf, uint8_t len)
{
  char tmp[5] = {0};
  tmp[0] = '1';
  tmp[1] = ',';
  tmp[2] = len/10+0x30;
  tmp[3] = len%10+0x30;
	if (g_cat1_info.gps_enable == 0)
		tmp[4] = ',';
  strcat((char *)buf, tmp);
	if (g_cat1_info.gps_enable == 0)
		strcat((char *)buf, ipaddr);
}

//发送AT命令
static void ec800g_cmd_next(uint8_t index)
{
  switch (index)
  {
    //发送AT命令，验证模块是否启动
    case SUB_SYNC:
      cmd_param_init(&g_at_cmd, AT_SYNC, NULL, CMD_EXCUTE);
      g_at_cmd.cmd_try_time = 10;
      cat1_send_cmd(&g_at_cmd);      
      break;    
    //开启错误报告与错误代码提示   
    case SUB_CMEE:
      cmd_param_init(&g_at_cmd, AT_CMEE, "1", CMD_SET);
      cat1_send_cmd(&g_at_cmd); 
      break;
    //打开回显    
    case SUB_ATE1:  
      cmd_param_init(&g_at_cmd, AT_ATE1, NULL, CMD_EXCUTE);
      cat1_send_cmd(&g_at_cmd);
      break;
    //获取软件版本号 
    case SUB_ATI: 
      cmd_param_init(&g_at_cmd, AT_ATI, NULL, CMD_EXCUTE);
      cat1_send_cmd(&g_at_cmd);
      break;
    //获取制造商ID
    case SUB_CGMI:
			if (strstr(cat1_recv.buf, "EC800M"))
			{
				DBGPRINTF("EC800M mode %d\r\n", flash_data.gps_flags);
				g_cat1_info.gps_enable = 1;
			}
			else {
				DBGPRINTF("EC800G mode\r\n");
				g_cat1_info.gps_enable = 0;
			}
			if (get_gps_state())
				GPIO_Pin_Clear(U32BIT(GNSS_PWR_EN));
			
      cmd_param_init(&g_at_cmd, AT_CGMI, NULL, CMD_EXCUTE);
      cat1_send_cmd(&g_at_cmd);
      break;
    //获取模块型号 
    case SUB_CGMM:
      cmd_param_init(&g_at_cmd, AT_CGMM, NULL, CMD_EXCUTE);
      cat1_send_cmd(&g_at_cmd);
      break;
    //获取IMEI号
    case SUB_CGSN:			
      cmd_param_init(&g_at_cmd, AT_CGSN, NULL, CMD_EXCUTE);
      g_at_cmd.timeout = 10000;
      cat1_send_cmd(&g_at_cmd);
      break;
    //获取卡号 
    case SUB_CIMI:
      cmd_param_init(&g_at_cmd, AT_CIMI, NULL, CMD_EXCUTE);
			for (uint8_t i = 0; i < 20; i++)
		  {
				if (cat1_recv.buf[i] == '8')
				{
					memcpy(g_cat1_info.imei, &cat1_recv.buf[i], 15);
					break;
				}					
			}
      //打印测试
      DBGPRINTF("SUB_CIMI: IMEI+ %s\r\n", g_cat1_info.imei);
      imei_trans_byte(g_cat1_info.imei);
      cat1_send_cmd(&g_at_cmd);
      break;  
    //获取卡的ICCID
    case SUB_ICCID:
      cmd_param_init(&g_at_cmd, AT_ICCID, NULL, CMD_EXCUTE);
      cat1_send_cmd(&g_at_cmd);    
      break;
//中国移动:898600 898602 898604 898607;
//中国联通:898601 898606 898609;
//中国电信:898603
    //打开终端,打开模块的射频
   	case SUB_CFUN:
     if (strstr(cat1_recv.buf, "898600") || strstr(cat1_recv.buf, "898602") || strstr(cat1_recv.buf, "898604") || \
          strstr(cat1_recv.buf, "898607"))
      {
        g_cat1_info.apn_flags = APN_CMNET;  
      }
      else if (strstr(cat1_recv.buf, "898603"))
      {
        g_cat1_info.apn_flags = APN_CTNET;  
      }
      else if (strstr(cat1_recv.buf, "898601") || strstr(cat1_recv.buf, "898606") || strstr(cat1_recv.buf, "898609"))
      {
        g_cat1_info.apn_flags = APN_UNINET;
      }
      else
      {
        //do nothing
      }  
			for (uint8_t i = 0; i < 30; i++)
		  {
				if (cat1_recv.buf[i] == '8')
				{
					memcpy(g_cat1_info.iccid, &cat1_recv.buf[i], 20);
					break;
				}					
			}      
      iccid_trans_byte(g_cat1_info.iccid);
//      DBGPRINTF("SUB_CIMI: ICCID+ %s\r\n", g_cat1_info.iccid);
      cmd_param_init(&g_at_cmd, AT_CFUN, "1", CMD_SET); 
      cat1_send_cmd(&g_at_cmd);
      break; 			
    //确认模块的网络注册状态
   	case SUB_CEREG:
      cmd_param_init(&g_at_cmd, AT_CEREG, NULL, CMD_READ);
      g_at_cmd.pExpect_rece = "+CEREG: 0,1";
      cat1_send_cmd(&g_at_cmd);
      break;
    //获取当前信号 
    case SUB_CSQ:
      cmd_param_init(&g_at_cmd, AT_CSQ, NULL, CMD_EXCUTE);
      g_at_cmd.pExpect_rece = "99,99";
      cat1_send_cmd(&g_at_cmd);
      break;
    //确认网络是否被激活 
    case SUB_CGATT:
      //加回显
      {
        char *buf = strstr(cat1_recv.buf, "+CSQ:");
        for (uint8_t i = 0; i < 10; i++)
        {
          if((buf[i] >= '0') && (buf[i] <= '9')) 
          {
            if (buf[i+1] == ',')
            {
              g_cat1_info.dBm = 83 - (buf[i] - 0x30);
            }
            else{
              g_cat1_info.dBm = 83 - ((buf[i] - 0x30) * 10 + (buf[i+1] - 0x30));
            }
            break;
          }
        }
      }
      flash_data.ble_imei[8] = g_cat1_info.dBm;
			lock_info.dBm = g_cat1_info.dBm;
      cmd_param_init(&g_at_cmd, AT_CGATT, NULL, CMD_READ);
      g_at_cmd.pExpect_rece = "CGATT: 1";
      cat1_send_cmd(&g_at_cmd);
      break;
      
///////////////////////////////////////////UDP协议连接////////////////////////////////////////////
           
    //设置APN名称  AT+QICSGP=1,1,"UNINET","","",1 
    case SUB_QICSGP:
		{		
			char apn_buf[50] = {'1', ',', '1', ',', '"'};
			fill_apn_msg(&g_cat1_info, apn_buf);
			cmd_param_init(&g_at_cmd, AT_QICSGP, apn_buf, CMD_SET); 
      cat1_send_cmd(&g_at_cmd);  
		}
      break;
    //激活PDP场景
    case SUB_QIACT:
      cmd_param_init(&g_at_cmd, AT_QIACT, "1", CMD_SET);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
      cat1_send_cmd(&g_at_cmd);     
      break;
		//获取当前时间
		case SUB_QLTS:
      cmd_param_init(&g_at_cmd, AT_QLTS, "1", CMD_SET);
			g_at_cmd.cmd_action = ACTION_OK_NEXT_ERROR;
      cat1_send_cmd(&g_at_cmd);			
			break;		
    //查询IP地址，确保模组已经注网成功
    case SUB_QIACT1:
			//获取当前时间
			if (lock_info.get_time_flags == 0)
			{
				get_current_time(cat1_recv.buf);
				lock_info.get_time_flags = 1;
			}
			g_cat1_info.register_success();
      cmd_param_init(&g_at_cmd, AT_QIACT, NULL, CMD_READ);
      cat1_send_cmd(&g_at_cmd);
      break; 		
    //创建UDP连接
    case SUB_QIOPEN:
//			lock_info.update_flags = 1;
//			flash_data.update_hw = 0x04;
//			flash_data.update_sw = 0x3002;
//			flash_data.update_check = 0x75;
		
			if (lock_info.update_flags == 0)
			{				
				memset(ip_buf, 0, 50);
				strcat(ip_buf, udp_head);
				strcat(ip_buf, ipaddr);
				strcat(ip_buf, udp_tail);
				cmd_param_init(&g_at_cmd, AT_QIOPEN, ip_buf, CMD_SET); 
				g_at_cmd.pExpect_rece = "+QIOPEN: 1,0";
			}
			else
			{
				xTimerChangePeriod(GREEN_LED_TIMER_EVT_ID, 100, 0);
				//打开回显 
				cmd_index = SUB_ATE2;				
				cmd_param_init(&g_at_cmd, AT_ATE1, NULL, CMD_EXCUTE);		
				
//				cmd_index = SUB_QFTPCFG_ACCOUNT;
//				cmd_param_init(&g_at_cmd, AT_QFTPCFG, "\"account\",\"fw2\",\"wvV5uSoV\"", CMD_SET);					
			}			
      cat1_send_cmd(&g_at_cmd);      
      break;
    //关闭回显
    case SUB_ATE0:  
      cmd_param_init(&g_at_cmd, AT_ATE0, NULL, CMD_EXCUTE);
      cat1_send_cmd(&g_at_cmd);
      break;	     
    //设置服务器IP与端口号
    case SUB_QISEND:
    {
      switch (lock_info.data.opcode)
      {
        case BOOT_TIRGGER:
          g_cat1_info.len = 23;
          break;
        case LOCK_TIRGGER:
          g_cat1_info.len = 17;
          break;
        case PERIOD_TIRGGER:
          g_cat1_info.len = 23;
          break;
        case ALARM_TIRGGER:
          g_cat1_info.len = 16;
          break;
				case UPDATE_TIRGGER:
					g_cat1_info.len = 19;
					break;
				case TIGHTEND_TIRGGER:
					g_cat1_info.len = 16;
					break;
        default:
          break;
      }
			memset(g_send_dataBuf, 0, CAT1_TX_BUF_SIZE);
			udp_set_ipaddr(g_send_dataBuf, g_cat1_info.len);
      cmd_param_init(&g_at_cmd, AT_QISEND, (char *)g_send_dataBuf, CMD_SET); 
      cat1_send_cmd(&g_at_cmd);  
      GAPBBDelayMS(50);     
      //使用UDP发送数据
      memset(&cat1_send, 0, sizeof(cat1_send_t)); 
      cat1_recv_reset(); 
      switch (lock_info.data.opcode)
      {
        case BOOT_TIRGGER:
          g_cat1_info.len = 23;
					g_cat1_info.ops->unlock_req(&lock_info, (uint8_t *)cat1_send.buf);
          DBGPRINTF("SUB_QISEND: ----BOOT_TIRGGER\r\n");
          g_at_cmd.pExpect_rece = "+QIURC:";  
          break;
        case LOCK_TIRGGER:
          g_cat1_info.len = 17;
					g_cat1_info.ops->lock_status(&lock_info, (uint8_t *)cat1_send.buf);
          DBGPRINTF("SUB_QISEND: ----LOCK_TIRGGER\r\n");
          g_at_cmd.pExpect_rece = "+QIURC:";  
          break;
        case PERIOD_TIRGGER:
          g_cat1_info.len = 23;
					g_cat1_info.ops->period_report(&lock_info, (uint8_t *)cat1_send.buf);
          DBGPRINTF("SUB_QISEND: ----PERIOD_TIRGGER\r\n");
          g_at_cmd.pExpect_rece = "+QIURC:"; 
          break;
        case ALARM_TIRGGER:
          g_cat1_info.len = 16;
					g_cat1_info.ops->lock_alarm(&lock_info, (uint8_t *)cat1_send.buf);
          DBGPRINTF("SUB_QISEND: ----ALARM_TIRGGER\r\n");
          g_at_cmd.pExpect_rece = "+QIURC:"; 
          break;
				case UPDATE_TIRGGER:
          g_cat1_info.len = 19;
					g_cat1_info.ops->lock_update(&lock_info, (uint8_t *)cat1_send.buf);
          DBGPRINTF("SUB_QISEND: ----UPDATE_TIRGGER\r\n");
          g_at_cmd.pExpect_rece = "+QIURC:"; 	
					break;
				case TIGHTEND_TIRGGER:
          g_cat1_info.len = 16;
					g_cat1_info.ops->lock_tightend(&lock_info, (uint8_t *)cat1_send.buf);
          DBGPRINTF("SUB_QISEND: ----TIGHTEND_TIRGGER\r\n");
          g_at_cmd.pExpect_rece = "SEND OK"; 	
					break;					
        default:
          break;
      }
      cat1_send.len = g_cat1_info.len; 
      taskENTER_CRITICAL();    
      uart0_write((uint8_t *)cat1_send.buf, cat1_send.len);
      taskEXIT_CRITICAL();  
      xTimerChangePeriod(NB_TIMEOUT_TIMER_EVT_ID, 5000, 0);
    }
      break;
    //上传数据成功 
    case SUB_SUCCESS:              
      if (lock_info.period_report_flag & 0x01)
      {
        lock_info.period_report_flag = 0;
      }
//      else
//      {
//				GPIO_Pin_Clear(U32BIT(GREEN_LED));
//				xTimerStart(LED_ONE_SECOND_TIMER_EVT_ID, 0);
//      }
      g_cat1_info.gps_open_flag = 0;
      //定位功能
      if (g_cat1_info.gps_open_flag)
      {
        cmd_index = SUB_QGPS_READ;
      }
      else
      {
				g_cat1_info.error_cnt = 0;
				xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);
				GPIO_Pin_Clear(U32BIT(GREEN_LED));
				xTimerStart(LED_ONE_SECOND_TIMER_EVT_ID, 0);
				xTimerStop(NB_TIMEOUT_TIMER_EVT_ID, 0);
        lock_info.cat1_working = 0;
        cat1_recv_reset();        
        xTimerReset(SLEEP_TIMER_EVT_ID, 0);  
        DBGPRINTF("SUB_SUCCESS: \r\n");
        break; 
      }     
			
		/************************************ 获取GPS位置信息 **********************************/
		//启用或禁止GNSS自启动
//		case SUB_QGPSCFG_AUTOGPS:
//	    cmd_param_init(&g_at_cmd, AT_QGPSCFG, "\"autogps\",0", CMD_SET);
//      cat1_send_cmd(&g_at_cmd);				
//			break;
		//查询GPS的状态
		case SUB_QGPS_READ:
			if (!xTimerIsTimerActive(GREEN_LED_TIMER_EVT_ID))
			{
				xTimerStart(GREEN_LED_TIMER_EVT_ID, 0);
			}
			cmd_param_init(&g_at_cmd, AT_QGPS, NULL, CMD_READ);  
		  g_at_cmd.timeout = 2000; 
			g_at_cmd.pExpect_rece = "+QGPS: 0"; 		
			cat1_send_cmd(&g_at_cmd);
			break;			
		//打开GPS
		case SUB_QGPS:
			cmd_param_init(&g_at_cmd, AT_QGPS, "1", CMD_SET);    
			cat1_send_cmd(&g_at_cmd);
			break;
		//查询GNSS的配置
//		case SUB_QGPSCFG_READ:
//      cmd_param_init(&g_at_cmd, AT_QGPSCFG, "?", CMD_SET);
//      cat1_send_cmd(&g_at_cmd);			
//			break;
//		//配置NMEA输出端口
//		case SUB_QGPSCFG_OUTPORT:
//	    cmd_param_init(&g_at_cmd, AT_QGPSCFG, "\"outport\",uartdebug", CMD_SET);
//      cat1_send_cmd(&g_at_cmd);			
//			break;
		//启动或禁止通过AT+QGPSGNMEA获取NMEA语句  AT+QGPSGNMEA="GGA"
		case SUB_QGPSCFG_NMEASRC:
	    cmd_param_init(&g_at_cmd, AT_QGPSCFG, "\"nmeasrc\",1", CMD_SET);
      cat1_send_cmd(&g_at_cmd);					
			break;
		//配置NMEA语句输出类型
//		case SUB_QGPSCFG_GPSNMEATYPE:
//	    cmd_param_init(&g_at_cmd, AT_QGPSCFG, "\"gpsnmeatype\",63", CMD_SET);
//      cat1_send_cmd(&g_at_cmd);				
//			break;
//		//配置支持GNSS的卫星导航系统
//		case SUB_QGPSCFG_GNSSCONFIG:
//	    cmd_param_init(&g_at_cmd, AT_QGPSCFG, "\"gnssconfig\",1", CMD_SET);
//      cat1_send_cmd(&g_at_cmd);						
//			break;
//		//启用或禁止AP-FLASH快速热启动功能
//		case SUB_QGPSCFG_APFLASH:
//	    cmd_param_init(&g_at_cmd, AT_QGPSCFG, "\"apflash\",1", CMD_SET);
//      cat1_send_cmd(&g_at_cmd);				
//			break; 
		//获取指定的NMEA语句类型
		case SUB_QGPSGNMEA:
	    cmd_param_init(&g_at_cmd, AT_QGPSGNMEA, "\"GGA\"", CMD_SET);
		  g_at_cmd.timeout = 5000; 
			g_at_cmd.pExpect_rece = ",N,"; 
      cat1_send_cmd(&g_at_cmd);
			break;
		//发送GPS信息
		case SUB_GPS_QISEND:
      {
				char *buf = strstr(cat1_recv.buf, ",N,"); 
				if (buf != NULL)
				{
					buf = strstr(cat1_recv.buf, "$GNGGA"); 
					memcpy(g_location_buf, buf, 60);
					//获取定位信息
					get_gps_info(g_location_buf, &gps_info);					
				}
				memset(g_send_dataBuf, 0, CAT1_TX_BUF_SIZE);
				udp_set_ipaddr(g_send_dataBuf, 25);
        cmd_param_init(&g_at_cmd, AT_QISEND, (char *)g_send_dataBuf, CMD_SET); 
        g_at_cmd.timeout = 12000;
        g_at_cmd.cmd_try_time = 13;
        cat1_send_cmd(&g_at_cmd); 
        GAPBBDelayMS(50);       
        memset(&cat1_send, 0, sizeof(cat1_send_t)); 
        cat1_recv_reset(); 
        g_cat1_info.len = 25;
				g_cat1_info.ops->lock_lbs(&lock_info, (uint8_t *)cat1_send.buf, &gps_info);
					
        g_at_cmd.pExpect_rece = "+QIURC:"; 
        cat1_send.len = g_cat1_info.len; 
        taskENTER_CRITICAL();    
        uart0_write((uint8_t *)cat1_send.buf, cat1_send.len);
        taskEXIT_CRITICAL(); 
        xTimerChangePeriod(NB_TIMEOUT_TIMER_EVT_ID, 5000, 0);
        DBGPRINTF("SUB_LBS_QISEND: send \r\n");
      }		
			break;
    //获取定位信息
    case SUB_END:
			xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);        
      lock_info.cat1_working = 0;
			xTimerStop(NB_TIMEOUT_TIMER_EVT_ID, 0);
			{	
				char *buf = strstr(cat1_recv.buf, "+QIURC:"); 
				if (buf != NULL)
				{
					GPIO_Pin_Clear(U32BIT(GREEN_LED));
				}
				else {
					GPIO_Pin_Set(U32BIT(GREEN_LED));
					GPIO_Pin_Clear(U32BIT(BLUE_LED));
				}
			}
			xTimerStart(LED_ONE_SECOND_TIMER_EVT_ID, 0);
      //判断有没有关锁信号触发
      cat1_recv_reset();       
      xTimerReset(SLEEP_TIMER_EVT_ID, 0);  
      DBGPRINTF("SUB_END GPS: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
      break;	

		/************************************ 通过FTP升级 **********************************/	
		//ftp://152.136.206.32:10021 fw lp8623
		//设置用户名和密码
		case SUB_QFTPCFG_ACCOUNT:
	    cmd_param_init(&g_at_cmd, AT_QFTPCFG, "\"account\",\"fw2\",\"wvV5uSoV\"", CMD_SET);
      cat1_send_cmd(&g_at_cmd);				
			break;
		//设置文件类型为ASCII
		case SUB_QFTPCFG_FILETYPE:
	    cmd_param_init(&g_at_cmd, AT_QFTPCFG, "\"filetype\",1", CMD_SET);
      cat1_send_cmd(&g_at_cmd);				
			break;
		//设置为被动传输模式
		case SUB_QFTPCFG_TRANSMODE:
	    cmd_param_init(&g_at_cmd, AT_QFTPCFG, "\"transmode\",1", CMD_SET);
      cat1_send_cmd(&g_at_cmd);				
			break;
		//设置最大响应时间（默认90s）
		case SUB_QFTPCFG_RSPTIMEOUT:
	    cmd_param_init(&g_at_cmd, AT_QFTPCFG, "\"rsptimeout\",90", CMD_SET);
      cat1_send_cmd(&g_at_cmd);				
			break;
		//登入FTP服务器
		case SUB_QFTPOPEN:
			memset(ip_buf, 0, 50);
      strcat(ip_buf, ftpipaddr);
	    cmd_param_init(&g_at_cmd, AT_QFTPOPEN, ip_buf, CMD_SET);
			g_at_cmd.timeout = 10000; 
			g_at_cmd.pExpect_rece = "+QFTPOPEN: 0,0"; 
      cat1_send_cmd(&g_at_cmd);				
			break;
		//设置当前目录
		case SUB_QFTPCWD:
	    cmd_param_init(&g_at_cmd, AT_QFTPCWD, "\"/\"", CMD_SET);
			g_at_cmd.pExpect_rece = "+QFTPCWD: 0,0"; 
      cat1_send_cmd(&g_at_cmd);					
			break;
//		//删除UFS中的padlock.bin文件
//		case SUB_QFDEL:
//	    cmd_param_init(&g_at_cmd, AT_QFDEL, "\"UFS:padlock.bin\"", CMD_SET);
//      cat1_send_cmd(&g_at_cmd);					
//			break;		
		//获取文件大小  +QFTPSIZE: 0,80160  //char *filename = "\"padlock.bin\",\"COM:\",";
		case SUB_QFTPSIZE:
			sprintf(filename, "\"padlock_v%02x_v%04x_%02x.bin\"",flash_data.update_hw,flash_data.update_sw,flash_data.update_check);
	    cmd_param_init(&g_at_cmd, AT_QFTPSIZE, filename, CMD_SET);
			g_at_cmd.timeout = 10000;
			g_at_cmd.pExpect_rece = "+QFTPSIZE: 0"; 
      cat1_send_cmd(&g_at_cmd);					
			break;
		//下载文件
		case SUB_QFTPDOWNLOAD:
		{
			get_file_size(cat1_recv.buf, &ftp_update);
			uint8_t ret = CodeErase();
			DBGPRINTF("CodeErase = %d\r\n", ret);
			memset(filename_buf, 0, 50);
			memcpy(filename_buf, filename, sizeof(filename));
			strcat(filename_buf, filename_ufs);		
	    cmd_param_init(&g_at_cmd, AT_QFTPGET, filename_buf, CMD_SET);
			g_at_cmd.timeout = 10000;
			g_at_cmd.pExpect_rece = "+QFTPGET: 0"; 
      cat1_send_cmd(&g_at_cmd);
		}
			break;
		case SUB_QFLDS:
	    cmd_param_init(&g_at_cmd, AT_QFLDS, "\"UFS\"", CMD_SET);
      cat1_send_cmd(&g_at_cmd);				
			break;
		//打开文件
		case SUB_QFOPEN:
			memset(filename_buf, 0, 50);
			memcpy(filename_buf, &filename_ufs[1], strlen(&filename_ufs[1]));
			strcat(filename_buf, file_open_end);				
	    cmd_param_init(&g_at_cmd, AT_QFOPEN, filename_buf, CMD_SET);
      cat1_send_cmd(&g_at_cmd);	
			break;
		//设置文件的初始地址
		case SUB_QFSEEK_START:
			get_filehandle(cat1_recv.buf);
			memset(filename_buf, 0, 50);
			memcpy(filename_buf, filehandle_buf, sizeof(filehandle_buf));
			strcat(filename_buf, file_seek_end);			
	    cmd_param_init(&g_at_cmd, AT_QFSEEK, filename_buf, CMD_SET);
      cat1_send_cmd(&g_at_cmd);				
			break;
		//读取文件
		case SUB_QFREAD_START:
			memset(file_read_buf, 0, 30);
			memcpy(file_read_buf, filehandle_buf, sizeof(filehandle_buf));
			strcat(file_read_buf, file_read_end);				
	    cmd_param_init(&g_at_cmd, AT_QFREAD, file_read_buf, CMD_SET);
			g_at_cmd.ok_flag = 1;
      cat1_send_cmd(&g_at_cmd);				
			break;
		//设置文件偏移地址
		case SUB_QFSEEK_POLL:
		{
			uint8_t ret = get_file_data(&cat1_recv, &ftp_update);
			if (ret == 2)
			{
				static uint8_t error_cnt = 0;
				error_cnt++;
				if (error_cnt >= 20)
				{
					xTimerStop(NB_TIMEOUT_TIMER_EVT_ID, 0);
					cat1_recv_reset(); 
					lock_info.timeout_cnt = 0;
					lock_info.update_flags = 0;
					xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);
					GPIO_Pin_Clear(U32BIT(BLUE_LED));
					GPIO_Pin_Set(U32BIT(GREEN_LED));
					//亮红LED灯报错
					xTimerStart(NB_ERROR_TIMER_EVT_ID, 0);
					//进入低功耗
					xTimerReset(SLEEP_TIMER_EVT_ID, 0); 
					lock_info.cat1_working = 0; 
					DBGPRINTF("SUB_QFSEEK_POLL error...\r\n");
					break;
				}
			}	
			else if (ret == 0)
			{
				xTimerStop(NB_TIMEOUT_TIMER_EVT_ID, 0);				
				DBGPRINTF("chk_xor:%02x = flash_data.update_check:%02x\r\n", chk_xor, flash_data.update_check);		
				if (chk_xor == flash_data.update_check)
				{
					uint8_t ret;
					ret = CodeUpdate(NULL, NULL, ftp_update.total_size, ftp_update.checksum);
					DBGPRINTF("codeupdate: ret:%d size:%x checksum:%x\r\n", ret, ftp_update.total_size, ftp_update.checksum);	
					//如果升级写入失败，也要清升级标志位，避免一直升级
					if (ret == 1)
					{	
						flash_data.flag += 1;
					}
					write_flash(&flash_data);
					lock_info.update_flags = 0;
					GAPBBDelayMS(1500);
					SystemReset();
				}
				else 
				{
//					memset(&ftp_update, 0, sizeof(ftp_update_t));
					chk_xor = 0;
					GAPBBDelayMS(1500);
					static uint8_t update_error_cnt = 0;
					update_error_cnt++;
					if (update_error_cnt >= 3)
					{		
						xTimerStop(NB_TIMEOUT_TIMER_EVT_ID, 0);
						cat1_recv_reset(); 
						lock_info.timeout_cnt = 0;
						lock_info.update_flags = 0;
						xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);
						GPIO_Pin_Clear(U32BIT(BLUE_LED));
						GPIO_Pin_Set(U32BIT(GREEN_LED));
						//亮红LED灯报错
						xTimerStart(NB_ERROR_TIMER_EVT_ID, 0);
						//进入低功耗
						xTimerReset(SLEEP_TIMER_EVT_ID, 0); 
						lock_info.cat1_working = 0; 					
					}
					else
					{
						memset(file_seek_set_paragram, 0, 30);
						ftp_update.index = 0;
						ftp_update.current_size = 200;
						ftp_update.checksum = 0;
						ftp_update.offset = 0;
						uint8_t ret = CodeErase();
						DBGPRINTF("CodeErase error = %d\r\n", ret);
						vTaskSendQueueEvt(UART_QUE_HDL, UART0_UPDATE_MSG_ID_EVT, 0xBB, NULL);
					}
					break;
				}
			}
		}			
	    cmd_param_init(&g_at_cmd, AT_QFSEEK, file_seek_set_paragram, CMD_SET);
      cat1_send_cmd(&g_at_cmd);			
			break;
		//读取文件
		case SUB_QFREAD_POLL:
	    cmd_param_init(&g_at_cmd, AT_QFREAD, file_read_buf, CMD_SET);
			g_at_cmd.ok_flag = 1;
			g_at_cmd.timeout = 3500; 
      cat1_send_cmd(&g_at_cmd);		
			break;
    default:
      break;
  } 
}

static void ec800g_process_handle(uint8_t cat1_stat)
{
	xTimerStop(NB_TIMEOUT_TIMER_EVT_ID, 0);
  cat1_recv_reset(); 
  lock_info.timeout_cnt = 0;
  switch (cat1_stat)
  {      
  //开锁  
  case AT_UNLOCK:
		xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);
    GPIO_Pin_Set(U32BIT(GREEN_LED));	
		lock_info.open_flags = 1;
//    if (lock_info.unlock_flag == 0)
    {
      GPIO_Pin_Clear(U32BIT(BLUE_LED));
      lock_info.motor_status = MOTOR_OPEN;
      xTimerStart(MOTOR_TIMEOUT_TIMER_EVT_ID, 0);
      motor_forward();
    }
		lock_info.cat1_working = 0; 
    g_cat1_info.error_cnt = 0;    
    break;
  //关锁，掉电指令
  case AT_LOCK:
		if (queue_is_empty(pcat1_handle) == 0)
		{
			vTaskSendQueueEvt(SOFTTIMER_QUE_HDL, CAT1_HANDLE_MSG_ID_EVT, 0xBB, NULL);
		}	
		else
		{
			xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);
			GPIO_Pin_Set(U32BIT(GREEN_LED));
			cmd_index = SUB_SUCCESS;
			//进入低功耗
			xTimerReset(SLEEP_TIMER_EVT_ID, 0); 
			lock_info.cat1_working = 0;
		}
		g_cat1_info.error_cnt = 0;
    break;
  //容错处理  
  case AT_ERROR_HAND: 
//		GPIO_Pin_Clear(U32BIT(CAT1_PWR_CTRL));
//    g_cat1_info.register_status = 0;
//		xTimerChangePeriod(GREEN_LED_TIMER_EVT_ID, 2000, 0);
//    xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);
//    GPIO_Pin_Clear(U32BIT(BLUE_LED));
//    GPIO_Pin_Set(U32BIT(GREEN_LED));
//    //亮红LED灯报错
//    xTimerStart(NB_ERROR_TIMER_EVT_ID, 0);
//    //进入低功耗
//    xTimerReset(SLEEP_TIMER_EVT_ID, 0); 
//    lock_info.cat1_working = 0; 	
//    queue_clear(pcat1_handle);
    
    GPIO_Pin_Clear(U32BIT(CAT1_PWR_CTRL));
    GAPBBDelayMS(300); 
    g_cat1_info.register_status = 0;
    xTimerChangePeriod(GREEN_LED_TIMER_EVT_ID, 2000, 0);
    g_cat1_info.error_cnt++;
    if (g_cat1_info.error_cnt >= 3) 
    {
      g_cat1_info.error_cnt = 0;
      xTimerStop(GREEN_LED_TIMER_EVT_ID, 0);
      GPIO_Pin_Clear(U32BIT(BLUE_LED));
      GPIO_Pin_Set(U32BIT(GREEN_LED));
      //亮红LED灯报错
      xTimerStart(NB_ERROR_TIMER_EVT_ID, 0);
      //进入低功耗
      xTimerReset(SLEEP_TIMER_EVT_ID, 0); 
      lock_info.cat1_working = 0; 
      queue_clear(pcat1_handle);        
    }
    else
    {
      cat1_recv_reset();
			g_cat1_info.power_on();		
      send_at_cmd();
    }
    DBGPRINTF("L505_processHandle AT_ERROR_HAND:~~~~~~~~~~~~~~~~~\r\n");
    break;
  default:
    break;
  }
}

static void ec800g_repeat_execute(CmdHandle cmd_handle)
{
  //清空接收缓存
  cat1_recv_reset();
  //串口发送数据
	taskENTER_CRITICAL();
  uart0_write((uint8_t *)cat1_send.buf, cat1_send.len);
	taskEXIT_CRITICAL();
  //开启定时器
	xTimerChangePeriod(NB_TIMEOUT_TIMER_EVT_ID, cmd_handle->timeout, 0);
  cmd_handle->cmd_try_time++;
}

//注册上平台
static void ec800g_register_success(void)
{
  //打开定时器，闪烁LED
  g_cat1_info.register_status = 1;
	xTimerChangePeriod(GREEN_LED_TIMER_EVT_ID, 500, 0);
}

//清空接收缓存
void cat1_recv_reset(void)
{
  memset(&cat1_recv, 0, sizeof(cat1_recv_t));
}

//EC800G模块开机 PWRKEY拉低至少2s
void ec800g_power_on(void)
{
	GPIO_Pin_Set(U32BIT(CAT1_PWR_CTRL));		
	//打开电源延时10ms
	GAPBBDelayMS(100); 
	GPIO_Pin_Clear(U32BIT(CAT1_PWR_KEY));
	GAPBBDelayMS(1800); 
	GPIO_Pin_Set(U32BIT(CAT1_PWR_KEY));
}

//EC800G关机,开机情况下，PWRKEY拉低至少3s后释放
static void ec800g_power_off(void)
{
	GPIO_Pin_Clear(U32BIT(CAT1_PWR_KEY));	
	GAPBBDelayMS(3500); 
	GPIO_Pin_Set(U32BIT(CAT1_PWR_KEY));
}

//Cat1复位
static void ec800g_hardware_reset(void)
{
//    HwGPIOSet(Board_Cat1Reset, 0);  
//    delay_ms(1500);
//    HwGPIOSet(Board_Cat1Reset, 1);
}

//Cat1出错LED显示
void cat1_error_led(void)
{
	static uint8_t count = 0;
	count++;
	if (count >= 10)
	{
		count = 0;
		xTimerStop(NB_ERROR_TIMER_EVT_ID, 0);
		GPIO_Pin_Set(U32BIT(BLUE_LED));
		GPIO_Pin_Set(U32BIT(GREEN_LED));
//    flash_data.unlock_flags = lock_info.unlock_flag;
		//升级失败，重置版本号
		flash_data.update_hw = lock_info.hw;
		flash_data.update_sw = lock_info.sw;
    write_flash(&flash_data);
		GAPBBDelayMS(1500);
    SystemReset();
	}
	else
	{
		GPIO_Pin_Turn(U32BIT(GREEN_LED));
		GPIO_Pin_Turn(U32BIT(BLUE_LED));
	}
}

void imei_trans_byte(uint8_t *p)
{  
  int i = 0;
  uint8_t buf[8] = {0x00};
  for (i = 0; i < 7; i++)
  {
    buf[i] = ((p[2*i] - 0x30) & 0x0f) << 4 | ((p[2*i+1] - 0x30) & 0x0f);
  }
  buf[i] = ((p[2*i] - 0x30) & 0x0f) << 4;	
  if (compare_buf(buf, flash_data.ble_imei, 8) == 0)
  {
    memcpy(flash_data.ble_imei, buf, 8);
    DBGPRINTF("write flash imei....");
    write_flash(&flash_data);
  }
  //蓝牙协议与Cat.1协议获取IMEI号方式没有统一
  flash_data.imei[0] = ((p[0] - 0x30) & 0x0f);
  for (i = 1; i <= 7; i++)
  {
    flash_data.imei[i] = ((p[2*i-1] - 0x30) & 0x0f) << 4 | ((p[2*i] - 0x30) & 0x0f);
  }  
	memcpy(lock_info.imei, flash_data.imei, 8);
  DBGPRINTF("ble_imei+ ");
  hexprint(flash_data.ble_imei,8);  
}
//89860486102070026794
void iccid_trans_byte(uint8_t *p)
{
  int i = 0; 
  uint8_t tmp[2] = {0};  //89860623570067443051
  for (i = 0; i < 10; i++)
  {
    if ((p[i*2] >= 'a') && (p[i*2] <= 'z'))
    {
      tmp[0] = p[i*2] - 0x57;
    }
    else if ((p[i*2] >= 'A') && (p[i*2] <= 'Z'))
    {
      tmp[0] = p[i*2] - 0x37;
    }
    else
    {
      tmp[0] = p[i*2] - 0x30;
    }
    
    if ((p[i*2+1] >= 'a') && (p[i*2+1] <= 'z'))
    {
      tmp[1] = p[i*2+1] - 0x57;
    }
    else if ((p[i*2+1] >= 'A') && (p[i*2+1] <= 'Z'))
    {
      tmp[1] = p[i*2+1] - 0x37;
    }    
    else
    {
      tmp[1] = p[i*2+1] - 0x30;
    }
    ble_iccid[i] = (tmp[0] & 0x0f) << 4 | (tmp[1] & 0x0f);
  }
  if (compare_buf(lock_info.iccid, ble_iccid, 10) == 0)
  {
    memcpy(lock_info.iccid, ble_iccid, 10);
    DBGPRINTF("write flash iccid....");
    write_flash(&flash_data);    
  }
  DBGPRINTF("ble_iccid+ ");
  hexprint(ble_iccid,10);
  hexprint(lock_info.iccid,10);
}

void ec800g_send_lock_cmd(void)
{
    cmd_index = SUB_QISEND;
    ec800g_cmd_next(cmd_index);
}

void ipaddrBuf_transition_char(uint8_t *pIpaddr, ipaddr_type_t type)
{
    uint8_t j = 1;
    uint8_t divide = 0, residual = 0, tmp = 0;
    uint16_t port = 0;
    uint8_t buf[5] = {0};
    uint8_t i = 0, k = 0;
		char *ip_addr = NULL;
		if (type == UDP_IPADDR)
		{
			ip_addr = ipaddr;
		}
		else if (type == FTP_IPADDR)
		{
			ip_addr = ftpipaddr;
		}
		else{
		
		}
    ip_addr[0] = '"';
    uint8_t *p = pIpaddr;
    for(i = 0; i < 4; i++)
    {
      divide = *pIpaddr / 100;
      residual =  *pIpaddr % 100;
      tmp = residual / 10;
      if (divide != 0)
      {
        ip_addr[j++] = divide + 0x30;
        if (tmp != 0)
        {
            ip_addr[j++] = tmp + 0x30;
        }
        else
        {
            ip_addr[j++] = 0 + 0x30;
        }
      }
      else
      {
          if (tmp != 0)
          {
            ip_addr[j++] = tmp + 0x30;
          }
      }
      ip_addr[j++] = residual % 10 + 0x30;
      ip_addr[j++] = '.';
      pIpaddr++;
    }
    //填充PORT
    j--;
    ip_addr[j++] = '"';
    ip_addr[j++] = ',';
    port = (p[4] << 8) | p[5];
    i = 0;
    while(port != 0)
    {
      buf[i] = port%10;
      port = port/10;
      i++;
    }
    i--;
    for(k = 0; k <= i/2; k++)
    {
      tmp = buf[i-k];
      buf[i-k] = buf[k];
      buf[k] = tmp;
    }
    for(k = 0; k <= i; k++)
    {
      ip_addr[j++] = buf[k] + 0x30;
    }
    ip_addr[j++] = '\0';  
}

uint8_t compare_buf(uint8_t *buf1, uint8_t *buf2, uint8_t len)
{
    uint8_t i = 0;
    for (i = 0; i < len; i++)
    {
        if (buf1[i] != buf2[i])
        {
            break;
        }
    }
    if (i == len)
      return 1;
    return 0;
}

uint8_t get_gps_state(void)
{
	if ((flash_data.gps_flags == 1) && (g_cat1_info.gps_enable == 1))
		return 1;
	else{
		return 0;	
	}
}

//+QGPSGNMEA: $GPGGA,103647.000,3150.721154,N,11711.925873,E,1,02,4.7,59.8,M,-2.0,M,,*77 
void get_gps_info(char *buf, location_info_t *gps_pinfo)
{
	uint8_t i = 0, cnt = 0;
	float tmp = 00.000000;
	uint32_t minute = 0;
	char str[6];
	//获取维度
	while(cnt < 2)
	{
		if (buf[i] == ',')
			cnt++;
		i++;
	}
	gps_info.latitude_degree[0] = '0';
	memcpy(&gps_info.latitude_degree[1], &buf[i], 2);
	tmp = ((buf[i+2] - 0x30)*10 + (buf[i+3] - 0x30)*1 + (buf[i+5] - 0x30)*0.1 + (buf[i+6] - 0x30)*0.01 \
			+(buf[i+7] - 0x30)*0.001 + (buf[i+8] - 0x30)*0.0001 + (buf[i+9] - 0x30)*0.00001 + (buf[i+10] - 0x30)*0.000001)/60;
	
	minute = tmp*1000000;
	for(i = 0; i < 6; i++)
	{
		str[i] = minute%10;
		minute = minute/10;
	}
	uint8_t j = 5;
	for (i = 0; i < 6; i++)
	{
		gps_info.latitude_minute[i] = str[j] + 0x30;
		j--;
	}
	//获取经度
	i = 0;
	cnt = 0;
	while(cnt < 4)
	{
		if (buf[i] == ',')
			cnt++;
		i++;
	}
	memcpy(gps_info.longitude_degree, &buf[i], 3);
	tmp = ((buf[i+3] - 0x30)*10 + (buf[i+4] - 0x30)*1 + (buf[i+6] - 0x30)*0.1 + (buf[i+7] - 0x30)*0.01 \
			+(buf[i+8] - 0x30)*0.001 + (buf[i+9] - 0x30)*0.0001 + (buf[i+10] - 0x30)*0.00001 + (buf[i+11] - 0x30)*0.000001)/60;
	
	minute = tmp*1000000;
	for(i = 0; i < 6; i++)
	{
		str[i] = minute%10;
		minute = minute/10;
	}
	j = 5;
	for (i = 0; i < 6; i++)
	{
		gps_info.longitude_minute[i] = str[j] + 0x30;
		j--;
	}
	DBGPRINTF("gps_info: longitude_degree:%s longitude_minute:%s\r\n", gps_info.longitude_degree, gps_info.longitude_minute);
	DBGPRINTF("gps_info: latitude_degree:%s latitude_minute:%s\r\n", gps_info.latitude_degree, gps_info.latitude_minute);
}

//获取文件描述符
void get_filehandle(char *buf)
{
	char *str = strstr(buf, "+QFOPEN:");	
	if (str != NULL)
	{
		uint8_t i = 0;
		uint8_t j = 0;
		while(str[i] != ':')
		{
			i++;
		}
		i++;
		while(str[i+j] != 0x0d)
		{
			filehandle_buf[j] = str[i+j];
			j++;
		}
		DBGPRINTF("filehandle = %s\r\n", filehandle_buf);
	}
}

//38 30 31 36 30 0D 0A   +QFTPSIZE: 0,80160
void get_file_size(char *buf, ftp_update_t *pdata)
{
	char *str = strstr(buf, "+QFTPSIZE: 0");
	uint8_t i = 0;
	uint32_t sum = 0;
	while(str[i] != ',')
	{
		i++;
	}	
	i++;
	while(str[i] != 0x0d)
	{
		sum = sum*10+(str[i] - 0x30); 
		i++;
	}
	pdata->offset = 0;
	pdata->len = 200;
	pdata->total_size = sum;
	pdata->current_size = pdata->len;
	pdata->last_size = pdata->total_size % pdata->len;
	if (pdata->last_size == 0)
	{
		pdata->page_num = pdata->total_size / pdata->len;
	}
	else
	{
		pdata->page_num = (pdata->total_size / pdata->len)+1;
	}
	DBGPRINTF("update total size:%d last_size:%d page_num:%d\r\n", pdata->total_size,pdata->last_size,pdata->page_num);
}

uint8_t get_file_data(cat1_recv_t *recv, ftp_update_t *pdata)
{
	char str[10] = {0};
	uint8_t result = 0;
	uint8_t ret = 0;
	uint16_t i = 0;
	uint16_t j = 0;
	char *str1 = "CONNECT";
	uint8_t len = strlen(str1);
	
	//获取接收到的数据 0x0d  0x0a
	for (i = 0; i < recv->len; i++)
	{
		if (recv->buf[i] == str1[0])
		{
			for (j = 1; j < len; j++)
			{
				if (recv->buf[i+j] != str1[j])
					break;
			}
			if (j == len)
				break;
		}
	}
	if (i == recv->len)
	{
		DBGPRINTF("get_file_data i == recv->len\r\n");
		return 2;
	}
	
	while(recv->buf[i] != 0x0a)
	{
		i++;
	}
	i++;
	
	if (recv->buf[i+pdata->current_size] != 0x0d)
	{
		DBGPRINTF("get_file_data recv->buf[i+pdata->current_size] != 0x0d\r\n");
		return 2;
	}	
	chk8xor((unsigned char*)&recv->buf[i], pdata->current_size, &chk_xor);	
		
	for(j = 0; j < pdata->current_size ; j++)
	{
//		DBGPRINTF("%02x ", recv->buf[i+j]);
		pdata->checksum += recv->buf[i+j];
	}
	ret = CodeWrite(pdata->offset, pdata->current_size, (uint8_t *)&recv->buf[i]);
	
	DBGPRINTF("get_file_data debug:%d %02x %d %d\r\n",ret,recv->buf[i+pdata->current_size],pdata->offset,pdata->current_size);
	
	pdata->index++;
	if (pdata->index < pdata->page_num-1)
	{
		pdata->offset = pdata->len*pdata->index;
		pdata->current_size = pdata->len;
		result = 1;
	}
	else if(pdata->index == pdata->page_num-1)
	{
		pdata->offset = pdata->len*pdata->index;
		pdata->current_size = pdata->last_size;
		result = 1;
	}
	else 
	{		
		result = 0;
	}
	sprintf(str,",%d,0", pdata->offset);
	memset(file_seek_set_paragram, 0, 30);
	strcat(file_seek_set_paragram, filehandle_buf);
	strcat(file_seek_set_paragram, str);
	DBGPRINTF("updateing offset:%d current_size:%d index:%d\r\n", pdata->offset,pdata->current_size,pdata->index);
	return result;
}

void fill_apn_msg(cat1_info_t *info, char *buf)
{
	//判断是否是默认的APN
	if (flash_data.apn_key != 0x00)
	{
		g_cat1_info.apn_flags = APN_PRIVATE;	
	}
	if (info->apn_flags == APN_CTNET)
	{
		memcpy(buf, apn_ctnet, strlen(apn_ctnet));	
		char *str_ctnet = "CTNET";
		flash_data.apn_msg[0] = strlen(str_ctnet);
		memcpy(&flash_data.apn_msg[1], str_ctnet, flash_data.apn_msg[0]);
	}
	else if (info->apn_flags == APN_UNINET)
	{
		memcpy(buf, apn_uninet, strlen(apn_uninet));	
		char *str_uninet = "UNINET";
		flash_data.apn_msg[0] = strlen(str_uninet);
		memcpy(&flash_data.apn_msg[1], str_uninet, flash_data.apn_msg[0]);
	}
	else if(info->apn_flags == APN_PRIVATE)
	{
		uint8_t len = strlen(apn_symbol);
		memcpy(&buf[5], &flash_data.apn_msg[1], flash_data.apn_msg[0]);
		memcpy(&buf[5+flash_data.apn_msg[0]], apn_symbol, len);
		memcpy(&buf[5+flash_data.apn_msg[0]+len], &flash_data.apn_username[1], flash_data.apn_username[0]);
		memcpy(&buf[5+flash_data.apn_msg[0]+len+flash_data.apn_username[0]], apn_symbol, len);
		memcpy(&buf[5+flash_data.apn_msg[0]+len+flash_data.apn_username[0]+len], &flash_data.apn_pwd[1], flash_data.apn_pwd[0]);
		apn_end[2] = flash_data.apn_key;
		strcat(&buf[5], apn_end);
	}
	else {
		memcpy(buf, apn_cmnet, strlen(apn_cmnet));	
		char *str_cmnet = "CMNET";
		flash_data.apn_msg[0] = strlen(str_cmnet);
		memcpy(&flash_data.apn_msg[1], str_cmnet, flash_data.apn_msg[0]);		
	}	
}
//+QLTS: "2024/11/22,01:18:50+32,0" 		lock_info.period_cnt 18对应凌晨2点
void get_current_time(char *buf)
{
	uint8_t i = 0;
	uint8_t hours = 0;
	char *str = strstr(buf, "+QLTS:");
	if (str != NULL)
	{
		while(str[i] != ',')
		{
			i++;
		}
		i++;
		hours = (str[i]-0x30)*10+(str[i+1]-0x30);
		if (hours <= 18)
		{
			lock_info.period_cnt = flash_data.period_time-(18-hours);
		}
		else{
			lock_info.period_cnt = flash_data.period_time-(hours-18);
		}
		DBGPRINTF("hours = %d period_time = %d period_cnt = %d\r\n", hours, flash_data.period_time, lock_info.period_cnt);  
	}
}

void chk8xor(unsigned char *p_chk, unsigned int len, uint8_t *chk_sum) 
{
	unsigned int i;  
	for(i = 0; i < len; i++)
	{
		*chk_sum ^= p_chk[i]; //p ^=*p++
	}
	DBGPRINTF("chk_sum = 0x%02x\r\n", *chk_sum);  
}

//比较buf中是否含有一字符串  +QFTPGET: 0,200
uint8_t compare_str(cat1_recv_t *recv)
{
	uint16_t i = 0;
	uint8_t len = 0, j = 0;
	char *str = "OK";   //
	len = strlen(str);
	
	for(i = 0; i < recv->len; i++)
	{
		if (recv->buf[i] == str[0])
		{
			if ((recv->buf[i-1] == 0x0a) && (recv->buf[i-2] == 0x0d))
			{
				for (j = 1; j < len; j++)
				{
					if (recv->buf[i+j] != str[j])
						break;
				}
				if (j == len)
					return 1;
			}
		}
	}
	return 0;
}

static cat1_info_t g_cat1_info = {
	.cmd_next = ec800g_cmd_next,
	.process_handle = ec800g_process_handle,
	.cmd_is_pass = ec800g_cmd_is_pass,	
	.register_success = ec800g_register_success,
	.power_on = ec800g_power_on,
	.power_off = ec800g_power_off,
	.hardware_reset = ec800g_hardware_reset,
	.send_lock_cmd = ec800g_send_lock_cmd,
	.repeat_execute = ec800g_repeat_execute,
	.ops = NULL,
};

cat1_info_t *cat1_init(void)
{
	g_cat1_info.ops = udp_msg_init();
	return &g_cat1_info;
}

