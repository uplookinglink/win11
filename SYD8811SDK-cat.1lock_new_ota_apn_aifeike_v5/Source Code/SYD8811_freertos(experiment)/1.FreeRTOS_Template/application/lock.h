#ifndef _LOCK_H_
#define _LOCK_H_

#include "gpio.h"

#define SHAKE_TIME 20  //按键防抖时间  -new

#define Z     0
#define N     1

//----------
#define IDLE_MODE           0
#define CAT1_MODE           0x01
#define BLUETOOTH_MODE      0x02
#define OTG_MODE						0x03

#define UNLOCK                	 0x01    //开锁
#define LOCK                  	 0x00    //关锁

#define MOTOR_RESERVE_TIME     60		//马达多转时间

#if N 	//新平台（创想物联锁）蓝牙数据
#define HEAD                     0x01    //数据头
#define SEND_CMD                 0x65    //蓝牙锁上报手机 101 
#define OPEN_LOCK_CMD            0x6e    //开锁命令 110
#define UPDATE_PWD_CMD           0x6f    //更新密码命令 111
#define BLE_TO_MOBILE_CMD        0x70    //蓝牙锁发信息给手机 112
#define MOBILE_TO_BLE_CMD        0x71    //手机发信息给蓝牙锁 113
#define INIT_LOCK_CMD            0x79    //初始化锁121
#define GET_IMEI_CMD             0x7a    //获取IMEI
#define GET_ICCID_CMD            0x7b    //获取ICCID
#define CHANGE_IPADDR_CMD        0x7c    //更改IP地址与PORT
#define GET_IPADDR_CMD           0x7d    //获取IP地址与PORT

#define GET_APN_MSG_CMD          0x80    //获取APN的信息
#define SET_APN_MSG_CMD					 0x81    //设置APN的信息
#define GET_APN_USERNAME_CMD     0x82    //获取APN的用户名
#define SET_APN_USERNAME_CMD     0x83    //设置APN的用户名
#define GET_APN_PWD_CMD          0x84    //获取APN的密码
#define SET_APN_PWD_CMD          0x85    //设置APN的密码
#define RESET_APN_CMD    				 0x86    //重置APN的信息
#define GET_APN_KEY_CMD          0x87    //获取APN的鉴权信息
#define SET_APN_KEY_CMD          0x88    //设置APN的鉴权信息

#define TENSION_CMD					 		 0x8b    //拉紧蓝牙上报

//#define FLASH_LEN                22      //flash的长度

#define LOCK_KEY_SN_FLAG         0x26 //flash存储锁状态头 0xA8 02
#define LOCK_ECB_KEY_FLAG        0xA9    //flash存储ECB加密头
#define READ_ECB_KEY             0x66    //读取静态key
#define CHANGE_ECB_KEY           0x78    //更改静态key

#define CLOSE_STATUS  0x01
#define OPEN_STATUS		0x02

#define LOCKOPENED		111
#define LOCKCLOSED		100
#define PWD_ERROR     110

#define HARD_VER      0x04
#define SOFT_VER_H    0x20
#define SOFT_VER_L    0x01

//锁指令	
#define READ_STATUS    101	//读锁状态
#define OPENLOCK	     110	//开锁
#define UPDATEPWD      111	//更新密码
#define UPDATELOCK     121	//初始化锁
#define READKEY        102	//读取加密静态key
#define CHGKEY         120	//更改加密静态key

//锁flash
#define LOCK_FLASH_ADDR   		0		//起始地址从0开始  最后8K为用户数据 (10000000+0x3E000) 
#define LOCK_PWD_ADDR   		0		//起始地址从0开始  最后8K为用户数据 (10000000+0x3E000) 
#define FLASH_SIZE 				0x2000 	//8k
#define FLASH_LEN 				32 		//暂时用
#endif

#define MOTOR_NON					0
#define MOTOR_OPEN				1
#define MOTOR_CLOSE 			2

#define LOW_POWER    1
#define WAKE_UP      0

#define TASK_LIST_OPEN   0
enum BLE_STATUS{
	  GAP_ADV = 0,            //广播
	  GAP_CONNECTED,          //连接
	  GAP_ADV_END,            //静默
	  GAP_SYS_SLEEP,          //低功耗
};       

typedef enum {
    BOOT_TIRGGER,			//获取发送信息
    LOCK_TIRGGER,			//上报开锁状态
    PERIOD_TIRGGER,		//周期性上报
    ALARM_TIRGGER,		//上报告警信息
		UPDATE_TIRGGER,		//4G升级后上报版本号
		TIGHTEND_TIRGGER,   //锁绳拉紧状态
}tirgger_state_t;        

typedef struct{
	uint8_t flag;
  uint8_t pwd[4]; 
  uint8_t sn[2];
	uint8_t key_flag;   	//26	 
//	uint8_t iccid[10];
	uint8_t ipaddr[6];
	uint16_t period_time;		//周期时间	
 	uint8_t imei[9];			//imei号
 	uint8_t ble_imei[9];	
  uint8_t shoudown_flags;
	uint8_t gps_flags;
	uint8_t update_hw;
	uint16_t update_sw;
	uint8_t update_check;
	uint8_t update_ipaddr[6];
	uint8_t update_flag;
	uint8_t cat1_state;
	char apn_msg[30];
	char apn_username[25];
	char apn_pwd[14];
	char apn_key;
	uint8_t bat_val;
}flash_data_t;    //flash数据结构 FLASH_LEN 32字节 

typedef struct {
	uint8_t apn_msg[30];
	uint8_t apn_username[25];
	uint8_t apn_pwd[14];
	uint8_t apn_key;
}apn_msg_t;

typedef struct{
	uint8_t head[3];
	uint8_t status; 		//状态
	uint8_t batter;			//电量
	uint8_t soft_ver_H;
	uint8_t soft_ver_L;		//软件版本号
	uint8_t hard_ver;		//硬件版本号
	uint8_t sn_H;
	uint8_t sn_L;			//sn
}lock_cmd_status;    //蓝牙通信协议锁状态 

typedef struct{
  uint8_t opcode;
  uint8_t state;
}lock_handle_t;

typedef struct{
	uint8_t bat_val;   
	uint8_t status;
  volatile uint8_t irq_flags;
	
	//new------------------------
	uint8_t LockPwd[4];
	uint8_t flashBuf[FLASH_LEN];
	uint8_t att_status[10];			//通信协议锁状态
	uint8_t nb_error;
	uint8_t period_report_flag;
	volatile uint8_t nb_working;
//	tirgger_state_t tirgger_state;
  lock_handle_t data;
	volatile uint8_t key_cnt;
	volatile uint8_t cat1_working;
	volatile uint8_t ble_working;
	volatile uint8_t work_mode;
	volatile uint8_t sleep_flag;
	volatile uint8_t nb_flag;  //cat.1模块pwm供电标志
	volatile uint8_t motor_status;
	volatile uint16_t period_cnt;
  volatile uint8_t unlock_flag;
  uint8_t timeout_cnt;
  uint8_t ble_timeout;
	volatile uint8_t ble_connected;
	uint8_t imei[9];			//imei号
	uint8_t iccid[10];			//imei号
	uint8_t dBm; 
	uint8_t hw;
	uint16_t sw;
	volatile uint8_t update_flags;
	volatile uint8_t ble_update_flags;
	volatile uint8_t open_flags;
	volatile uint8_t battery_detect_flag;
	uint8_t get_time_flags;
	
	uint8_t hall_cnt;
	uint8_t hall_flags;
	uint8_t tightend_send_flags;
}lock_info_t;

//new
typedef struct{
	uint8_t plaintext[16];
	unsigned int plaintext_len;
	uint8_t associated_data[14];
	unsigned short associated_data_len;
  uint8_t nonce[12];
	const unsigned short nonce_len; 
	uint8_t ciphertext[20]; 
	unsigned int ciphertext_len;                // OUT - The length of the ciphertext, always plaintext_len + mac_len.
  unsigned int mac_len;                        // IN  - The desired length of the MAC, must be 4, 6, 8, 10, 12, 14, or 16.
  uint8_t key[16];                    // IN  - The AES key for encryption.
  int keysize;                       // IN  - The length of the key in bits. Valid values are 128, 192, 256.
	uint32_t random;
}aes_ccm_t;


typedef struct{
	uint8_t plaintext[16];
	unsigned int len;
	uint8_t ciphertext[16];  
	uint8_t raw_data[16];
	uint8_t key1[16];
	uint8_t key2[16];
}aes_t;

typedef struct{
	uint8_t motor_open_flag;
	uint8_t motor_close_flag;
	uint8_t lock_close_flag;
	uint8_t lock_hall_flag;
}lock_irq_t;

typedef struct _queue_t{
  lock_handle_t *data;
  uint8_t size;
  uint8_t max;
  uint8_t front;
  uint8_t rear;
}queue_t;

queue_t *queue_create(int size, int max);
int queue_is_empty(queue_t *q);
int queue_is_full(queue_t *q);
int queue_en(queue_t *q, lock_handle_t *data);
int queue_de(queue_t *q, lock_handle_t *data);
void queue_clear(queue_t *q);

void communicate_init(void);

void motor_brake(void);
void motor_stop(void);
void motor_forward(void);
void motor_reverse(void);
uint32_t Random_4(void);
void ByteToHexStr(char *pbSrc, char *pbDest,  int nLen); //nlen =src lens
void *revmemcpy( void *dst, const void *src, unsigned int len );
void Gatt_Rx_Handle(uint8_t *p_rx);//
//void gatt_send_data(uint8_t cmd, uint8_t id, uint8_t param);
void gatt_send_data(uint8_t cmd);

uint8_t read_flash(flash_data_t *buf);
void write_flash(flash_data_t *buf);

void feed_dog(void);
uint8_t get_lock_status(void);
void hexprintf(uint8_t *in, uint8_t len);
void flash_data_init(void);

uint8_t check_sum(uint8_t *data, uint8_t lenth);
void UserEncryptInit(void);
uint8_t compare_password(uint8_t *password1, uint8_t *password2, uint8_t len);

void aes_test(void);
void delay_ms(uint32_t ui32Count);
void hexprint(uint8_t *in, uint8_t len);

uint8_t app_aes_ccm_decrypt(uint8_t *plaintext,uint8_t *ciphertext);
void ecb_test(void);

#endif
