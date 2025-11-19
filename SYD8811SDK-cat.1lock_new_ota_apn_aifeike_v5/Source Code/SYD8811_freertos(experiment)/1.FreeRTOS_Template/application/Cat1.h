#ifndef __CAT1_H__
#define __CAT1_H__

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "udp.h"

#define CAT1_TX_BUF_SIZE       90
#define CAT1_RX_BUF_SIZE       350  //230

#define APN_CMNET			0
#define APN_CTNET			1
#define APN_UNINET  	2
#define APN_PRIVATE 	3

#define EC800M

typedef enum
{
  CMD_TEST,         //test命令
  CMD_READ,         //read命令
  CMD_SET,          //set命令
  CMD_EXCUTE        //excute命令
}cmd_type_t;

typedef enum
{
  AT_ERROR,
  AT_SUCCESS,
  AT_UNLOCK,
  AT_LOCK,
  AT_ERROR_HAND,
	AT_UPDATE_START,
	AT_UPDATEING,
  AT_NON,
}at_type_t;

typedef enum
{
  ACTION_OK_EXIT_ERROR,      //错误执行后，退出
  ACTION_OK_NEXT_ERROR       //错误执行后，继续执行下一条指令                               
}cmd_action_t;

//AT指令结构类型
typedef struct
{
  const char    *pCMD;          //  AT指令
  cmd_type_t    type;           // 指令当前属性(TEST,READ,SET,EXCUTE)
  char          *pArg;          // 指令参数
  uint8_t       cmd_try_max;    // 出错尝试次数
  uint8_t       cmd_try_time;   // 已经出次尝试的次数
  uint16_t      timeout;        // 最大超时时间
  cmd_action_t  cmd_action;     // AT指令行为
  char          *pExpect_rece;  //期望得到的回复 
  uint8_t       ok_flag;        //区分期望得到的字符串与OK的先后关系
}cmd_info_t;

typedef struct {
	uint32_t total_size;
	uint32_t page_num;
	uint32_t offset;
	uint32_t index;
	uint8_t len;
	uint8_t current_size;
	uint8_t last_size;
	uint16_t checksum;
}ftp_update_t;

//声明cmd结构指针类型
typedef cmd_info_t *CmdHandle;

//NB模块的操作状态
typedef enum
{
  SUB_NONE = 0,
  SUB_SYNC,
  SUB_CMEE,
  SUB_ATE1,
  SUB_ATI,
  SUB_CGMI,
  SUB_CGMM,
  SUB_CGSN,
  SUB_CIMI,
  SUB_ICCID,
  SUB_CFUN,			//0x0a
//  SUB_CGDCONT,
  SUB_CEREG, 
  SUB_CSQ,
  SUB_CGATT,

  SUB_QICSGP,
  SUB_QIACT,
	SUB_QLTS,
  SUB_QIACT1,  //0x10
	
  SUB_QIOPEN,
  SUB_ATE0,
  SUB_QISEND,		
  SUB_SUCCESS,	
	
	SUB_QGPS_READ,
	SUB_QGPS,

//	SUB_QGPSCFG_AUTOGPS,
//	SUB_QGPS,  //0x1a
//	SUB_QGPSCFG_READ,
//	SUB_QGPSCFG_OUTPORT,
	SUB_QGPSCFG_NMEASRC,
//	SUB_QGPSCFG_GPSNMEATYPE,
//	SUB_QGPSCFG_GNSSCONFIG,
//	SUB_QGPSCFG_AUTOGPS,
//	SUB_QGPSCFG_APFLASH,
	SUB_QGPSGNMEA,
//	SUB_QGPSLOC,
	SUB_GPS_QISEND,
	SUB_END,
	
	SUB_RESERVED_1,
	SUB_RESERVED_2,
	
	SUB_ATE2,
	SUB_QFTPCFG_ACCOUNT,  //33
	SUB_QFTPCFG_FILETYPE,
	SUB_QFTPCFG_TRANSMODE, 
	SUB_QFTPCFG_RSPTIMEOUT,
	SUB_QFTPOPEN,
	SUB_QFTPCWD,
//	SUB_QFDEL,
	SUB_QFTPSIZE,
	SUB_QFTPDOWNLOAD,
	SUB_QFLDS,
	SUB_QFOPEN,
	SUB_QFSEEK_START,
	SUB_QFREAD_START,
	SUB_QFSEEK_POLL,
	SUB_QFREAD_POLL,
}sub_id_t;

//接收数据buf
typedef struct ReceBuf
{
  char buf[CAT1_RX_BUF_SIZE];
  uint16_t len;                            //有效数据长度
}cat1_recv_t;

//发送数据buf
typedef struct SendBuf
{
  char buf[CAT1_TX_BUF_SIZE];
  uint16_t len;
}cat1_send_t;

//定义一个存储BC26的状态信息
typedef struct
{
  uint8_t  dBm;                         //信号
  volatile uint8_t  register_status;    //注册成功标志
  uint8_t  receive_cmd;                 //OneNET协议
  uint8_t  iccid[20];                   //卡号
  uint8_t  imei[16];                    //模块的IMEI号
  uint8_t  len;
	uint8_t status;
	char lon[10];
	char lat[10];
	uint8_t gps_open_flag;
	volatile uint8_t apn_flags;
	uint8_t error_cnt;
	volatile uint8_t gps_enable;
	
	void (*cmd_next)(uint8_t index);
	void (*process_handle)(uint8_t cat1_stat);
	uint8_t (*cmd_is_pass)(cat1_recv_t *recv);
	void (*register_success)(void);
	void (*power_on)(void);
	void (*power_off)(void);
	void (*hardware_reset)(void);	
	void (*send_lock_cmd)(void);
	void (*repeat_execute)(CmdHandle cmd_handle);
	msg_ops_t *ops;
}cat1_info_t;

typedef enum {
	UDP_IPADDR,
	FTP_IPADDR,
}ipaddr_type_t;

void ec800g_power_on(void);

void cat1_recv_reset(void);
void cat1_error_led(void);

void imei_trans_byte(uint8_t *p);
void iccid_trans_byte(uint8_t *p);

void ipaddrBuf_transition_char(uint8_t *pIpaddr, ipaddr_type_t type);

uint8_t compare_buf(uint8_t *buf1, uint8_t *buf2, uint8_t len);

cat1_info_t *cat1_init(void);

void get_current_time(char *buf);
uint8_t get_gps_state(void);
void get_gps_info(char *buf, location_info_t *gps_pinfo);
void get_file_size(char *buf, ftp_update_t *pdata);
uint8_t get_file_data(cat1_recv_t *recv, ftp_update_t *pdata);
void chk8xor(unsigned char *p_chk,unsigned int len,uint8_t * chk_sum);
uint8_t compare_str(cat1_recv_t *recv);
void get_filehandle(char *buf);
void fill_apn_msg(cat1_info_t *info, char *buf);

#endif 
