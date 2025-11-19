#include "lock.h"
#include "led_key.h"
#include "lib.h"
#include "stdint.h"
#include "string.h"
#include "ble.h"
#include "aes.h"
#include "aes_ecb.h"
#include "DebugLog.h"
#include "..\application\softtimer.h"
#include <stdlib.h>
#include "gpadc.h"
#include "lock.h"
#include "main.h"
#include "pwm.h"
#include "Cat1.h"
//#include "xj_aes128.h"

extern TimerHandle_t MOTOR_TIMEOUT_TIMER_EVT_ID;
extern TimerHandle_t BLUE_LED_TIMER_EVT_ID;
extern lock_info_t lock_info;
extern cat1_info_t p_cat1_info;
extern uint8_t advertData[]; //31
extern BLE_APP_gatt ble_gatt_data;
//extern uint8_t ble_imei[];
extern uint8_t ble_iccid[];

uint8_t ownAddress[6] = {0};
flash_data_t flash_data;
uint8_t data_type;
queue_t *pcat1_handle;

apn_msg_t apn_msg = {
	.apn_msg= {0x00},
	.apn_username = {0x00},
	.apn_pwd = {0x00},
	.apn_key = 0x00,
};

bool UserEncrypt(uint8_t *key, uint8_t *rawDataBuf, uint8_t rawDataLength, uint8_t *encryptedData);	//用户加密
bool UserDecrypt(uint8_t *key, uint8_t *encryptedData, uint8_t *deccryptedData);		//用户解密

#if 0
aes_t aesBuf = {
   .plaintext = {0},
   .len = 16,
   .ciphertext = {0},
	 .raw_data = {0},
	 .key1 = {0},
	 .key2 = {0},
};
#endif

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~NEW~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//标识：新增参数or函数

uint8_t EcbKey[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0x0e, 0x02, 0x06, 0x0B, 0x07, 0x15, 0x08, 0x09, 0x0F, 0x4F, 0x3C};//静态key

 aes_ccm_t aesBuf = {
   .plaintext = {0x01, 0x6e, 0x04, 0x08, 0x08, 0x06, 0x06, 0, 0, 0, 0, 0, 0, 0, 0, 0},
   .plaintext_len = 16,
   //.associated_data = { 'l','i','n','k','p','o','w','e','r','8','6','2','3','1' }, //header 
   .associated_data = {0x6c, 0x69, 0x6e, 0x6b, 0x70, 0x6f, 0x77, 0x65, 0x72, 0x38, 0x36, 0x32, 0x33, 0x31},
   .associated_data_len = 14,
   .nonce = {0xf6, 0xb1, 0xcd, 0x64, 0x1c, 0xb0, 0xce, 0xae, 0x08, 0xc0, 0x14, 0x8a},
   .nonce_len = 12, 
   .ciphertext = {0},
   .ciphertext_len = 20,
   .mac_len = 4,
   .key =  {0x2b, 0x7e, 0x15, 0x16, 0x28, 0x0e, 0x02, 0x06, 0x0B, 0x07, 0x15, 0x08, 0x09, 0x0F, 0x4F, 0x3C},  	
   .keysize = 128
};

void handle_chg_pwd(void);			//更改开锁码
void handle_chg_key(void);			//更改加解密key

void Read_aesKey(uint8_t *cmdKey_tmp);//cmdKey_tmp为蓝牙协议CMD102填充

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~new
//标识：新改动内容
//~~~~~~~~~~~~~~~~

//打日志
void hexprintf(uint8_t *in, uint8_t len)
{
#ifdef LOCK_DEGUG
	  uint8_t i = 0;
		for(i = 0;i < len;i++)
		{
			DBGPRINTF("%02x", in[i]);	
		}
		DBGPRINTF("\r\n");
#endif
}
//打日志
void hexprint(uint8_t *in, uint8_t len)
{
	#ifdef LOCK_DEGUG
	  uint8_t i = 0;
			for(i=0;i<len;i++)
		{
			DBGPRINTF("%02x", in[i]);	
		}
		DBGPRINTF("\r\n");
	#endif
}


//马达刹车
void motor_brake(void)
{
	GPIO_Pin_Set(U32BIT(MotorCtrl_1));
	GPIO_Pin_Set(U32BIT(MotorCtrl_2));
}
/*********************************************************************************
功能: 马达停转
参数: void
*********************************************************************************/
void motor_stop(void)
{
  GAPBBDelayMS(MOTOR_RESERVE_TIME); //检测到位后，马达多转30ms
	motor_brake();
//	DelayMS(100);	 
	GAPBBDelayMS(100);
	GPIO_Pin_Clear(U32BIT(MotorCtrl_1));
	GPIO_Pin_Clear(U32BIT(MotorCtrl_2));
}

//马达正转
void motor_forward(void)
{
	GPIO_Pin_Set(U32BIT(MotorCtrl_1));
	GPIO_Pin_Clear(U32BIT(MotorCtrl_2));
}
//马达反转
void motor_reverse(void)
{
	GPIO_Pin_Set(U32BIT(MotorCtrl_2));
	GPIO_Pin_Clear(U32BIT(MotorCtrl_1));
}

/*********************************************************************************
功能: 产生4字节随机数
参数: void
返回：无符号4字节
*********************************************************************************/
uint32_t Random_4(void)
{
	uint32_t buf;
  uint8_t randbuf[4];
	uint8_t i = 0;
	for(i = 0; i < 4; i++)
	{
		randbuf[i] = Rand();
	}
	memcpy((uint8_t *)&buf,(uint8_t *)randbuf,4);
	return buf;
}
//锁初始化流程
//FB0A550110160B1D0B380294FCFCFCFC
//FB0A55012000000000000021FCFCFCFC
//FB05550175057BFCFCFCFCFCFCFCFCFC
//FB0655017901017CFCFCFCFCFCFCFCFC
//FB0655017901027DFCFCFCFCFCFCFCFC
//FB0C5501760006363636363736C2FCFC
//FB07550176040132AEFCFCFCFCFCFCFC   读内存
//FB0A55012100060F0104023EFCFCFCFC   设置密码


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~NEW~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~NEW~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//AES_CCM加密aes_encrypt
uint8_t app_aes_ccm_encypt(uint8_t *plaintext,uint8_t *ciphertext)
{
	return aes_encrypt_ccm(plaintext,aesBuf.plaintext_len,aesBuf.associated_data,aesBuf.associated_data_len,\
	aesBuf.nonce,aesBuf.nonce_len,ciphertext,&aesBuf.ciphertext_len, aesBuf.mac_len,aesBuf.key,aesBuf.keysize);
}

//AES_CCM解密
uint8_t app_aes_ccm_decrypt(uint8_t *plaintext,uint8_t *ciphertext)
{
	int mac_c[4] ={0};
	memcpy(aesBuf.ciphertext,ciphertext,20);
	return aes_decrypt_ccm(ciphertext, 20, aesBuf.associated_data, 14,aesBuf.nonce, 12, plaintext, &aesBuf.plaintext_len,4, mac_c, aesBuf.key, 128);
}

//锁状态初始化
void get_lock_info(void)
{	
	lock_cmd_status *p_att  = (lock_cmd_status *)lock_info.att_status;
		
	p_att->head[0] = 0x01; 
	p_att->head[1] = READ_STATUS; 
	p_att->head[2] = 0x09;
	p_att->batter = lock_info.bat_val;  //取ADC
	p_att->hard_ver = flash_data.update_hw;
	p_att->soft_ver_H = flash_data.update_sw >> 8;
	p_att->soft_ver_L = flash_data.update_sw & 0xff;
	//flash改动
	//p_att->sn_H = lock_info.flashBuf[5]; 
	//p_att->sn_L = lock_info.flashBuf[6];  //从flash里取
	p_att->sn_H = flash_data.sn[0]; 
	p_att->sn_L = flash_data.sn[1];;  //从flash里取
	//获取锁状态
	p_att->status = get_lock_status();
	
	advertData[13] = p_att->sn_H;
	advertData[14] = p_att->sn_L;
	
	advertData[15] = p_att->hard_ver;
	advertData[16] = p_att->soft_ver_H;
	advertData[17] = p_att->soft_ver_L;
	
	//flash改动
	//advertData[24] = lock_info.flashBuf[26]; //key更改标志
	//memcpy((uint8_t *)lock_info.LockPwd,(uint8_t *) (lock_info.flashBuf+1), 4);  //从flash里取开锁密码
	advertData[24] = flash_data.key_flag; //key更改标志
	memcpy(lock_info.LockPwd,flash_data.pwd, 4);  //从flash里取开锁密码
}

//AES_CCM_KEY初始化
void aes_ccm_key_init(uint32_t random)
{
	uint8_t ecbbuf[16] = {0}; //4deee775000000000000000000000000
	uint8_t defult_Key_head[4]={0x2b, 0x7e, 0x15, 0x16 };
	uint8_t defult_Key_chg[12]={0x28, 0x0e, 0x02, 0x06, 0x0B, 0x07, 0x15, 0x08, 0x09, 0x0F, 0x4F, 0x3C};	
	uint32_t Rand_tmp = random; //0x75e7ee4d
	
	memcpy(ecbbuf, (uint8_t *)&Rand_tmp, 4);
	memcpy((uint8_t *)(advertData+9), (uint8_t *)&Rand_tmp, 4);

	memcpy((uint8_t *)aesBuf.key, (uint8_t *)(defult_Key_head), 4); //从flash 里取 aes_key
	memcpy((uint8_t *)(aesBuf.key+4), (uint8_t *)(defult_Key_chg), 12); //从flash 里取 aes_key
//	hexprint(aesBuf.key, 16);
	aes_ecb_encrypt(ecbbuf, aesBuf.key);
//	hexprint(ecbbuf,16);
	
	aesBuf.nonce[0] = ecbbuf[8];
	aesBuf.nonce[1] = ecbbuf[7]; 
	aesBuf.nonce[2] = ecbbuf[11];
	aesBuf.nonce[3] = ecbbuf[5]; 
	aesBuf.nonce[4] = ecbbuf[2];
	aesBuf.nonce[5] = ecbbuf[10];
	aesBuf.nonce[6] = ecbbuf[4];
	aesBuf.nonce[7] = ecbbuf[14];
	aesBuf.nonce[8] = ecbbuf[0];
	aesBuf.nonce[9] = ecbbuf[6];
	aesBuf.nonce[10] = ecbbuf[13]; 
	aesBuf.nonce[11] = ecbbuf[9];  	
}

//BLE通信初始化
void communicate_init(void)
{
//	flash_data_init();	
	aesBuf.random = Random_4();
	aes_ccm_key_init(aesBuf.random);  //计算CCM NONCE
	
  get_lock_info();   							//取锁状态
	memset(aesBuf.plaintext,0,16);
	memcpy(aesBuf.plaintext,lock_info.att_status,10); 
	
  app_aes_ccm_encypt(aesBuf.plaintext,aesBuf.ciphertext);
	memcpy(ble_gatt_data.send_buf, aesBuf.ciphertext, 20);	
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//UUID CCD2收包处理 new
void Gatt_Rx_Handle(uint8_t *p_rx)
{	
	DBGPRINTF("CCD2 app data \r\n");

	hexprint(p_rx,20);
	uint32_t readPwd; uint32_t flashPwd;
	uint8_t ret = app_aes_ccm_decrypt(aesBuf.plaintext, p_rx);	//解密RX
	//打印接收到的数据
	hexprint(aesBuf.plaintext,16);
	if(ret)
	{
		switch(aesBuf.plaintext[1])
		{
			//开锁
      case OPENLOCK:
			  revmemcpy((uint8_t *)&readPwd, (uint8_t *)aesBuf.plaintext+3, 4);
			  revmemcpy((uint8_t *)&flashPwd, (uint8_t *)lock_info.LockPwd, 4);
			  DBGPRINTF("pwd %x\r\n",readPwd);
			  if(readPwd == flashPwd) 
				{
					lock_info.open_flags = 1;
				  lock_info.work_mode = BLUETOOTH_MODE;
					GPIO_Pin_Clear(U32BIT(BLUE_LED));
					xTimerStart(MOTOR_TIMEOUT_TIMER_EVT_ID, 0);
					lock_info.motor_status = MOTOR_OPEN;
					motor_forward();
					DBGPRINTF("ble open lock is successs\r\n");
					hexprint(aesBuf.plaintext,16);
				}
				else 
				{
					DBGPRINTF("pwd != %x\r\n",flashPwd);
					gatt_send_data(PWD_ERROR);
					DBGPRINTF("password is error\r\n");
				}
				break;
			//更新密码
			case UPDATEPWD:
				handle_chg_pwd();
				break;
			//更改KEY
			case UPDATELOCK:
			case CHGKEY:
			  handle_chg_key();
				break; 
			//读取KEY
			case READKEY:
				gatt_send_data(READKEY);
				break;
			//获取IMEI号	
			case GET_IMEI_CMD:
				aesBuf.plaintext[0] = HEAD;
				aesBuf.plaintext[1] = GET_IMEI_CMD;
				aesBuf.plaintext[2] = 0x09;
				hexprint(flash_data.ble_imei,9);
				memcpy(&aesBuf.plaintext[3], flash_data.ble_imei, 9); 
				hexprint(aesBuf.plaintext,16);				
				app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
				BLE_SendData(aesBuf.ciphertext,20);  
				break;
			//获取ICCID
			case GET_ICCID_CMD:
				aesBuf.plaintext[0] = HEAD;
				aesBuf.plaintext[1] = GET_ICCID_CMD;
				aesBuf.plaintext[2] = 0x0A; 
				hexprint(lock_info.iccid,10);
				memcpy(&aesBuf.plaintext[3], lock_info.iccid, 10); 
				hexprint(aesBuf.plaintext,16);				
				app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
				BLE_SendData(aesBuf.ciphertext,20);			
				break;
			case CHANGE_IPADDR_CMD:
				DBGPRINTF("CHANGE_IPADDR: flash_data_old");
				hexprint(flash_data.ipaddr,6);	
        memcpy(flash_data.ipaddr, &aesBuf.plaintext[3], 6);           
        //写flash
        hexprint(aesBuf.plaintext,16);	
        write_flash(&flash_data);
        ipaddrBuf_transition_char(flash_data.ipaddr, UDP_IPADDR);   
				DBGPRINTF("CHANGE_IPADDR: flash_data_new");
				hexprint(flash_data.ipaddr,6);	
        //填写IP与PORT，便于读
        aesBuf.plaintext[0] = HEAD;
        aesBuf.plaintext[1] = CHANGE_IPADDR_CMD;
        aesBuf.plaintext[2] = 0x06;
        memcpy(&aesBuf.plaintext[3], flash_data.ipaddr, 6);  
        app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
        BLE_SendData(aesBuf.ciphertext,20);	
				break;
			case GET_IPADDR_CMD:
        aesBuf.plaintext[0] = HEAD;
        aesBuf.plaintext[1] = GET_IPADDR_CMD;
        aesBuf.plaintext[2] = 0x06;
        memcpy(&aesBuf.plaintext[3], flash_data.ipaddr, 6);   
        app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
        BLE_SendData(aesBuf.ciphertext,20);	
				break;
			//获取APN的信息
			case GET_APN_MSG_CMD:
			{
				DBGPRINTF("GET_APN_MSG_CMD:%d %d\r\n", flash_data.apn_msg[0], aesBuf.plaintext[4]);
				if (flash_data.apn_msg[0] == 0)
				{
					aesBuf.plaintext[0] = HEAD;
					aesBuf.plaintext[1] = GET_APN_MSG_CMD;
					aesBuf.plaintext[2] = 0x00;
					aesBuf.plaintext[3] = 0x00;
					aesBuf.plaintext[4] = 0x00;	
					aesBuf.plaintext[5] = 0x00;
					aesBuf.plaintext[6] = 0x00;
					app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
					BLE_SendData(aesBuf.ciphertext,20);	
				}
				else {
					uint8_t tmp = aesBuf.plaintext[4];
					uint8_t total = 0;
					uint8_t divisor = flash_data.apn_msg[0]/11;
					uint8_t remainder = flash_data.apn_msg[0]%11;
					if (remainder == 0)
						total = divisor;
					else
						total = divisor+1;
					
					if (total > aesBuf.plaintext[4])
					{
						aesBuf.plaintext[0] = HEAD;
						aesBuf.plaintext[1] = GET_APN_MSG_CMD;
						aesBuf.plaintext[2] = 13;
						aesBuf.plaintext[3] = total;
						aesBuf.plaintext[4] = tmp;	
						memcpy(&aesBuf.plaintext[5], &flash_data.apn_msg[1+11*(tmp-1)], 11);
						app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
						BLE_SendData(aesBuf.ciphertext,20);						
					}
					else {
						aesBuf.plaintext[0] = HEAD;
						aesBuf.plaintext[1] = GET_APN_MSG_CMD;
						aesBuf.plaintext[2] = flash_data.apn_msg[0]-11*(tmp-1)+2;
						aesBuf.plaintext[3] = total;
						aesBuf.plaintext[4] = tmp;					
						memcpy(&aesBuf.plaintext[5], &flash_data.apn_msg[1+11*(tmp-1)], aesBuf.plaintext[2]-2);
						app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
						BLE_SendData(aesBuf.ciphertext,20);						
					}	
				}
			}
				break;
			//设置APN的信息
			case SET_APN_MSG_CMD:
				if (aesBuf.plaintext[4] == 1)
				{
					memset(flash_data.apn_msg, 0, 20);
				}
				memcpy(&flash_data.apn_msg[1+flash_data.apn_msg[0]], &aesBuf.plaintext[5], aesBuf.plaintext[2]-2);
				flash_data.apn_msg[0] += (aesBuf.plaintext[2]-2);
				
				DBGPRINTF("SET_APN_MSG_CMD:%d %d %d\r\n",aesBuf.plaintext[2],aesBuf.plaintext[4],flash_data.apn_msg[0]);
				//回复包，收到什么包，回相同包
				app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
				BLE_SendData(aesBuf.ciphertext,20);													
				break;
			//获取APN的用户名
			case GET_APN_USERNAME_CMD:	
			{
				DBGPRINTF("GET_APN_USERNAME_CMD:%d %d\r\n", flash_data.apn_username[0], aesBuf.plaintext[4]);
				if (flash_data.apn_msg[0] == 0)
				{
					aesBuf.plaintext[0] = HEAD;
					aesBuf.plaintext[1] = GET_APN_USERNAME_CMD;
					aesBuf.plaintext[2] = 0x00;
					aesBuf.plaintext[3] = 0x00;
					aesBuf.plaintext[4] = 0x00;
					aesBuf.plaintext[5] = 0x00;					
					aesBuf.plaintext[6] = 0x00;
					app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
					BLE_SendData(aesBuf.ciphertext,20);	
				}
				else {				
					uint8_t tmp = aesBuf.plaintext[4];
					uint8_t total = 0;
					uint8_t divisor = flash_data.apn_username[0]/11;
					uint8_t remainder = flash_data.apn_username[0]%11;
					if (remainder == 0)
						total = divisor;
					else
						total = divisor+1;
					
					if (total > aesBuf.plaintext[4])
					{
						aesBuf.plaintext[0] = HEAD;
						aesBuf.plaintext[1] = GET_APN_USERNAME_CMD;
						aesBuf.plaintext[2] = 13;
						aesBuf.plaintext[3] = total;
						aesBuf.plaintext[4] = tmp;	
						memcpy(&aesBuf.plaintext[5], &flash_data.apn_username[1+11*(tmp-1)], 11);
						app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
						BLE_SendData(aesBuf.ciphertext,20);						
					}
					else {
						aesBuf.plaintext[0] = HEAD;
						aesBuf.plaintext[1] = GET_APN_USERNAME_CMD;
						aesBuf.plaintext[2] = flash_data.apn_username[0]-11*(tmp-1)+2;
						aesBuf.plaintext[3] = total;
						aesBuf.plaintext[4] = tmp;					
						memcpy(&aesBuf.plaintext[5], &flash_data.apn_username[1+11*(tmp-1)], aesBuf.plaintext[2]-2);
						app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
						BLE_SendData(aesBuf.ciphertext,20);						
					}	
				}
			}		
				break;
			//设置APN的用户名  
			case SET_APN_USERNAME_CMD:
				if (aesBuf.plaintext[4] == 1)
				{
					memset(flash_data.apn_username, 0, 25);
				}
				memcpy(&flash_data.apn_username[1+flash_data.apn_username[0]], &aesBuf.plaintext[5], aesBuf.plaintext[2]-2);
				flash_data.apn_username[0] += (aesBuf.plaintext[2]-2);
				
				DBGPRINTF("SET_APN_USERNAME_CMD:%d %d %d\r\n",aesBuf.plaintext[2],aesBuf.plaintext[4],flash_data.apn_username[0]);
				//回复包，收到什么包，回相同包
				app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
				BLE_SendData(aesBuf.ciphertext,20);					
				break;
			//获取APN的密码
			case GET_APN_PWD_CMD:
				DBGPRINTF("GET_APN_PWD_CMD\r\n");
        aesBuf.plaintext[0] = HEAD;
        aesBuf.plaintext[1] = GET_APN_PWD_CMD;
        aesBuf.plaintext[2] = flash_data.apn_pwd[0];
        memcpy(&aesBuf.plaintext[3], &flash_data.apn_pwd[1], flash_data.apn_pwd[0]);   
        app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
        BLE_SendData(aesBuf.ciphertext,20);					
				break;
			//设置APN的密码
			case SET_APN_PWD_CMD:
				DBGPRINTF("SET_APN_PWD_CMD\r\n");
				flash_data.apn_pwd[0] = aesBuf.plaintext[2];
				memcpy(&flash_data.apn_pwd[1], &aesBuf.plaintext[3], flash_data.apn_pwd[0]);
			
			  aesBuf.plaintext[0] = HEAD;
        aesBuf.plaintext[1] = SET_APN_PWD_CMD;
        aesBuf.plaintext[2] = flash_data.apn_pwd[0];
        memcpy(&aesBuf.plaintext[3], &flash_data.apn_pwd[1], flash_data.apn_pwd[0]);  
        app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
        BLE_SendData(aesBuf.ciphertext,20);
				break;
			//重置APN的信息
			case RESET_APN_CMD:
				DBGPRINTF("RESET_APN_CMD\r\n");
				memcpy(flash_data.apn_msg, &apn_msg, sizeof(apn_msg_t));	
				write_flash(&flash_data);		
				BLE_SendData(aesBuf.ciphertext,20);			
				break;
			//获取APN的鉴权信息
			case GET_APN_KEY_CMD:
				DBGPRINTF("GET_APN_KEY_CMD\r\n");
        aesBuf.plaintext[0] = HEAD;
        aesBuf.plaintext[1] = GET_APN_KEY_CMD;
        aesBuf.plaintext[2] = 0x01;
        aesBuf.plaintext[3] = flash_data.apn_key; 
        app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
        BLE_SendData(aesBuf.ciphertext,20);					
				break;
			//设置APN的鉴权信息
			case SET_APN_KEY_CMD:
				DBGPRINTF("SET_APN_KEY_CMD\r\n");
				flash_data.apn_key = aesBuf.plaintext[3];
				hexprint(aesBuf.plaintext,16);	
        write_flash(&flash_data);
			
			  aesBuf.plaintext[0] = HEAD;
        aesBuf.plaintext[1] = SET_APN_KEY_CMD;
        aesBuf.plaintext[2] = 0x01;
        aesBuf.plaintext[3] = flash_data.apn_key; 
        app_aes_ccm_encypt(aesBuf.plaintext, aesBuf.ciphertext);
        BLE_SendData(aesBuf.ciphertext,20);
				break;  			
	    default:
      // do nothing
				break;
		}
	}
	else
	{
		SystemReset();
	}
}

//蓝牙通信发数据CCD1
//蓝牙通信发数据CCD1
void gatt_send_data(uint8_t cmd)
{
	uint8_t tx_tmp[16] = {0};
	DBGPRINTF("gatt_send_data --");
	switch(cmd)
	{
		case CLOSE_STATUS:
			lock_info.att_status[3] = LOCKCLOSED;
			lock_info.att_status[4] = lock_info.bat_val;
			memcpy(tx_tmp,lock_info.att_status,10);		
			DBGPRINTF("CLOSE_STATUS:  ");
			break;
		case OPEN_STATUS:
			lock_info.att_status[3] = LOCKOPENED;
			lock_info.att_status[4] = lock_info.bat_val;
			memcpy(tx_tmp,lock_info.att_status,10);	
			DBGPRINTF("OPEN_STATUS:  ");		
			break;
    case READ_STATUS:
			lock_info.att_status[3] = get_lock_status();
			lock_info.att_status[4] = lock_info.bat_val;
			memcpy(tx_tmp,lock_info.att_status,10);
			DBGPRINTF("READ_STATUS:  ");
			break;
		
		case PWD_ERROR:
		  lock_info.att_status[3] = PWD_ERROR;
			memcpy(tx_tmp,lock_info.att_status,10);
      DBGPRINTF("PWD_ERROR:  ");
		  break;
		
		case READKEY:
			Read_aesKey(tx_tmp);
			DBGPRINTF("READKEY:  ");
			break;
		//拉紧
		case TENSION_CMD:
		{
			uint8_t buf[4] = {0x00};
			buf[0] = 0x01;
			buf[1] = TENSION_CMD;
			buf[2] = 0x01;
			buf[3] = 0x02;
			memcpy(tx_tmp,buf,4);
			DBGPRINTF("BLE_TENSION_CMD:  ");
		}
			break;
	  default:
      // do nothing
			break;
	}
	memset(aesBuf.plaintext,0,16);
	memcpy(aesBuf.plaintext,tx_tmp,16);
	DBGPRINTF("gatt_send_data:  ");
	hexprint(aesBuf.plaintext,16);
	app_aes_ccm_encypt(aesBuf.plaintext,aesBuf.ciphertext);
	BLE_SendData(aesBuf.ciphertext,20);  
}

void flash_data_init(void)
{
	uint8_t tmp = 0;
	read_flash(&flash_data);
	tmp = flash_data.update_flag;
	if (flash_data.flag != LOCK_KEY_SN_FLAG) 
	{
//		memset(&flash_data, 0, sizeof(flash_data_t));
		//flash改动
		flash_data.flag = LOCK_KEY_SN_FLAG;
		flash_data.pwd[0] = 0x08;
		flash_data.pwd[1] = 0x08;
		flash_data.pwd[2] = 0x06;
		flash_data.pwd[3] = 0x06;
		flash_data.sn[0] = 0x00;
		flash_data.sn[1] = 0x00;
		
		flash_data.period_time = 24;
    //ip地址及端口号
    //公司：134.175.50.82  10010
		//陕西：47.92.67.102 5000  
#if 1
    flash_data.ipaddr[0] = 0x86; 
    flash_data.ipaddr[1] = 0xaf; 
    flash_data.ipaddr[2] = 0x32;
    flash_data.ipaddr[3] = 0x52;
    flash_data.ipaddr[4] = 0x27;
    flash_data.ipaddr[5] = 0x1a;		
#endif
#if 0   
    flash_data.ipaddr[0] = 47;
    flash_data.ipaddr[1] = 92;
    flash_data.ipaddr[2] = 67;
    flash_data.ipaddr[3] = 102;
    flash_data.ipaddr[4] = 0x13;
    flash_data.ipaddr[5] = 0x88;
#endif
		//ftp服务器的默认IP  "152.136.206.32",10021"
		flash_data.update_ipaddr[0] = 152;
		flash_data.update_ipaddr[1] = 136;
		flash_data.update_ipaddr[2] = 206;
		flash_data.update_ipaddr[3] = 32;
		flash_data.update_ipaddr[4] = 0x27;
		flash_data.update_ipaddr[5] = 0x25;
				
    flash_data.shoudown_flags = 0;
#ifdef EC800M
		flash_data.gps_flags = 1;
		flash_data.update_hw = 0x05;
		flash_data.update_sw = 0x8001; //抽拉式软锁梁软件版本从0x8001开始定义
#else
		flash_data.gps_flags = 0;
		flash_data.update_hw = 0x03;
		flash_data.update_sw = 0x1009;
#endif
		flash_data.update_check = 0x00; 
		memcpy(flash_data.apn_msg, &apn_msg, sizeof(apn_msg_t));	
		DBGPRINTF("flash init...\r\n");
		write_flash(&flash_data);
	}
	DBGPRINTF("flash init...bat_val = %d\r\n", flash_data.bat_val);
	flash_data.update_hw = 0x05;
	flash_data.update_sw = 0x8001; 
	flash_data.update_flag  = tmp;
	lock_info.hw = flash_data.update_hw;
	lock_info.sw = flash_data.update_sw;
	lock_info.update_flags = 0x00;
	lock_info.ble_update_flags = 0;
	lock_info.battery_detect_flag = 0;
	lock_info.get_time_flags = 0;
  ipaddrBuf_transition_char(flash_data.ipaddr, UDP_IPADDR);
	ipaddrBuf_transition_char(flash_data.update_ipaddr, FTP_IPADDR);
  pcat1_handle = queue_create(sizeof(lock_handle_t), 5);
}

//写flash接口
void write_flash(flash_data_t *buf)
{	
	//uint8_t ret_e = EraseFlashData(LOCK_PWD_ADDR, 2);
	uint8_t len = sizeof(flash_data_t);
	uint8_t ret_w = WriteProfileData(LOCK_FLASH_ADDR, len, (uint8_t *)buf);
	ble_sched_execute();
}

//读flash接口
uint8_t read_flash(flash_data_t *buf)
{   		
	uint8_t len = sizeof(flash_data_t);
	ReadProfileData(LOCK_FLASH_ADDR, len, (uint8_t *)buf);
	return 0;
}

//16进制转字符串
void ByteToHexStr(char *pbSrc, char *pbDest,  int nLen) //nlen =src lens
{
	unsigned char ddl,ddh;
	int i;

	for (i=0; i<nLen; i++)
	{
		ddh = 48 + (unsigned char)pbSrc[i] / 16;
		ddl = 48 + (unsigned char)pbSrc[i] % 16;
		if (ddh > 57) 
			ddh = ddh + 7;
		if (ddl > 57) 
			ddl = ddl + 7;
		pbDest[i*2] = ddh;
		pbDest[i*2+1] = ddl;
	}
	//pbDest[nLen*2] = '\0';
}

//倒CPY
void *revmemcpy( void *dst, const void *src, unsigned int len )
{
  uint8_t *pDst;
  const uint8_t *pSrc;

  pSrc = src;
  pSrc += (len-1);
  pDst = dst;
  while ( len-- )
    *pDst++ = *pSrc--;

  return ( pDst );
}

//喂狗
void feed_dog(void)
{
   GPIO_Pin_Set(U32BIT(WatchDog));
   DelayUS(15);
   GPIO_Pin_Clear(U32BIT(WatchDog));
}

uint8_t get_lock_status(void)
{
	if(lock_info.unlock_flag == 0) 
	{
		lock_info.status = LOCKCLOSED;
		DBGPRINTF("close lock status\r\n");
	}
	else 
	{
		lock_info.status = LOCKOPENED;
		DBGPRINTF("open lock status\r\n");
	}	
	return lock_info.status;
}

#if 0	//中粮相关

uint8_t compare_password(uint8_t *password1, uint8_t *password2, uint8_t len)
{
	uint8_t ret = 0;
	uint8_t i = 0;
	for (i = 0; i < len; i++)
	{
		if (password1[i] != password2[i])
			break;
	}
	if (i == len)
	{
		ret = 1;
	}
	else
	{
		ret = 0;
	}
	return ret;
}

uint8_t check_sum(uint8_t *data, uint8_t lenth)
{
	uint8_t checksum = 0;
	uint8_t i = 0;
	for(i = 0;i < lenth;i ++)
	{	
		checksum += data[i];
	}
	return checksum;
} 

void aes_test(void)
{
  uint8_t key[16] = {0xCA, 0x72, 0x17, 0xEF, 0xAA, 0x26, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa};
  uint8_t buf[16] = {0x40, 0x7c, 0x74, 0x15, 0x41, 0xcc, 0x8a, 0x51, 0x9d, 0x16, 0x73, 0x21, 0x0d, 0xe2, 0xbc, 0xb0};
  uint8_t buf1[17] = {0x00};
  UserDecrypt(key, buf1, buf);	
  hexprintf(buf, 16);
}

#endif

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~NEW~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//更改开锁码
void handle_chg_pwd(void)
{
	uint8_t writePwd[6];
	memcpy((uint8_t *)writePwd, (uint8_t *)aesBuf.plaintext+3, 6);
	//flash改动
	//memcpy ((uint8_t *)(lock_info.flashBuf+1), writePwd, 6);
	flash_data.pwd[0] = writePwd[0];
	flash_data.pwd[1] = writePwd[1];
	flash_data.pwd[2] = writePwd[2];
	flash_data.pwd[3] = writePwd[3];
	flash_data.sn[0] = writePwd[4];
	flash_data.sn[1] = writePwd[5];
	//WriteFlash_PWD(lock_info.flashBuf);
	write_flash(&flash_data);
	DBGPRINTF("chg_w = ");
	hexprint(flash_data.pwd,4);	
	hexprint(flash_data.sn,2);	
	
	read_flash(&flash_data);
	DBGPRINTF("chg_r = ");
	hexprint(flash_data.pwd,4);	
	hexprint(flash_data.sn,2);		
	get_lock_info();   //取锁状态
	setup_adv_data();	//更改广播包
	gatt_send_data(READ_STATUS);	//发送读蓝牙状态指令
}

//更改加解密key
void handle_chg_key(void)
{
//	uint8_t writeKey[13];
//	memcpy((uint8_t *)writeKey, (uint8_t *)aesBuf.plaintext+3, 13);
//	//flash改动
//	//memcpy ((uint8_t *)(lock_info.flashBuf+14), writeKey, 13);
//	memcpy((uint8_t *)(flash_data.key_chg), (uint8_t *)writeKey, 12);
//	flash_data.key_flag = writeKey[12];
//	write_flash(&flash_data);
//	DBGPRINTF("chg_w = ");
//	hexprint(flash_data.key_chg,12);	
	
//	read_flash(&flash_data);
//	DBGPRINTF("chg_r = ");
//	hexprint(flash_data.key_chg,12);	
//	get_lock_info();   //取锁状态
//	setup_adv_data();	//更改广播包
//	aes_ccm_key_init(aesBuf.random);  //计算CCM NONCE
//	gatt_send_data(READKEY);	//发送读蓝牙静态key指令
}

//读取加解密key
void Read_aesKey(uint8_t * cmdKey_tmp)
{
//	cmdKey_tmp[0] = 0x01; cmdKey_tmp[1] = 102; cmdKey_tmp[2] = 0x0d;//head 3字节
//	//flash改动
//	//memcpy ((uint8_t *)(cmdKey_tmp+3),(uint8_t *)(lock_info.flashBuf+14),13);
//	memcpy ((uint8_t *)(cmdKey_tmp+3),(uint8_t *)(flash_data.key_chg),12);
//	cmdKey_tmp[15] = flash_data.key_flag;	
	//DBGPRINTF("r_key");
	//hexprint(cmdKey_tmp,16);	
}

//ms级
void delay_ms(uint32_t ui32Count)
{
   uint16_t i=0; 
   if(ui32Count<=0)
     return ;
   while(ui32Count--)
   {
      i = 6600;
      while(i--) ;    
   }
}


#if 0
//写flash接口
void WriteFlash_PWD(uint8_t *FlashBuf)
{
	uint8_t ret_e = EraseFlashData(LOCK_PWD_ADDR, 2);
    uint16_t crc = 0;
    crc = crc16(FlashBuf, FLASH_LEN-2);
    memcpy((uint8_t *)(FlashBuf+FLASH_LEN-2), (uint8_t *)&crc,2);   
    uint8_t ret_w = WriteFlashData(LOCK_PWD_ADDR, FLASH_LEN, FlashBuf );
}

//读flash接口
uint8_t ReadFlash_PWD(uint8_t *FlashBuf)
{   
    uint8_t mark = LOCK_KEY_SN_FLAG;
    uint16_t tmp = 0;
    uint16_t crc = 0;
    uint8_t tmpBuf[FLASH_LEN];
		
    uint8_t ret_r = ReadFlashData(LOCK_PWD_ADDR, FLASH_LEN, tmpBuf);
    memcpy((uint8_t *)&tmp,(uint8_t *)(tmpBuf+FLASH_LEN-2), 2);
    memset((uint8_t *)FlashBuf, 0, sizeof(FlashBuf));	
    if (tmpBuf[0] == mark)
    {
			crc = crc16(tmpBuf,FLASH_LEN-2);
			if( tmp == crc )
			{
				memcpy((uint8_t *)FlashBuf,(uint8_t *)tmpBuf,FLASH_LEN);
				return 1;
			}
    }	
	  //CHECK head faild   给默认值    	
		uint8_t data[FLASH_LEN] = {0xA8, 0x08, 0x08, 0x06, 0x06, 0x00, 0x00, HARD_VER, SOFT_VER_H, SOFT_VER_L, 0x00, 0x00, 0x00, 0x00};
		uint8_t defult_Key[16] = {0x2b, 0x7e, 0x15, 0x16, 0x28, 0x0e, 0x02, 0x06, 0x0B, 0x07, 0x15, 0x08, 0x09, 0x0F, 0x4F, 0x3C};
		memcpy((uint8_t *)data+10, (uint8_t *)defult_Key, 16);
		memcpy((uint8_t *)FlashBuf, (uint8_t *)data, FLASH_LEN-2);
    WriteFlash_PWD(lock_info.flashBuf);
    memcpy((uint8_t *)lock_info.LockPwd, (uint8_t *)(lock_info.flashBuf+1), 4);  
	  return 2;
}

#endif
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

queue_t *queue_create(int size, int max)
{
  queue_t *q = NULL;
  q = pvPortMalloc(sizeof(queue_t));
  if (q == NULL)
  {
    DBGPRINTF("queue_t malloc is failed\r\n");
    return NULL;
  }
  q->data = pvPortMalloc(size*max);
  if (q->data == NULL)
  {
    DBGPRINTF("q->data malloc is failed \r\n");
		vPortFree(q);
    return NULL;
  }
  q->front = 0;
  q->rear = 0;
  q->size = size;
  q->max = max;
  return q;
}

int queue_is_empty(queue_t *q)
{
	if (q == NULL)
		return 1;
  return q->rear == q->front;
}

int queue_is_full(queue_t *q)
{
  return q->front == (q->rear+1)%q->max;
}

int queue_en(queue_t *q, lock_handle_t *data)
{
	if (q == NULL)
		return 0;
  if (queue_is_full(q))
  {
    DBGPRINTF("queue_is_full \r\n");
    return 0;
  }
  memcpy(q->data + q->rear*q->size, data, q->size);
  q->rear = (q->rear+1)%q->max;
  DBGPRINTF("queue_en %d %d\r\n", data->opcode, data->state);
  return 1;
}

int queue_de(queue_t *q, lock_handle_t *data)
{
	if (q == NULL)
		return 0;
//  if (queue_is_empty(q))
//  {
//    DBGPRINTF("queue_is_empty \r\n");
//    return 0;    
//  }
  memcpy(data, q->data + q->front*q->size, q->size);
  q->front = (q->front+1)%q->max;
  DBGPRINTF("queue_de %d %d\r\n", data->opcode, data->state);
  return 1;
}

void queue_clear(queue_t *q)
{
	if (q == NULL)
		return;
  q->front = 0;
  q->rear = 0;
  memset(q->data, 0 , q->size*q->max);
}

void ecb_test(void)
{
	uint8_t ecbbuf[16] = {0}; //4deee775000000000000000000000000
	uint32_t Rand_tmp = 0x9ab4c733;  //0x33c7b49a;
	memcpy(ecbbuf, (uint8_t *)&Rand_tmp, 4);
	//flash改动
	uint8_t defult_Key_head[4]={0x2b, 0x7e, 0x15, 0x16 };
	uint8_t defult_Key_chg[12]={0x28, 0x0e, 0x02, 0x06, 0x0B, 0x07, 0x15, 0x08, 0x09, 0x0F, 0x4F, 0x3C};
	memcpy((uint8_t *)aesBuf.key, (uint8_t *)(defult_Key_head), 4); //从flash 里取 aes_key
	memcpy((uint8_t *)(aesBuf.key+4), (uint8_t *)(defult_Key_chg), 12); //从flash 里取 aes_key
	hexprint(aesBuf.key, 16);
	aes_ecb_encrypt(ecbbuf, aesBuf.key);
	hexprint(ecbbuf,16);
	
	aesBuf.nonce[0] = ecbbuf[8];
	aesBuf.nonce[1] = ecbbuf[7]; 
	aesBuf.nonce[2] = ecbbuf[11];
	aesBuf.nonce[3] = ecbbuf[5]; 
	aesBuf.nonce[4] = ecbbuf[2];
	aesBuf.nonce[5] = ecbbuf[10];
	aesBuf.nonce[6] = ecbbuf[4];
	aesBuf.nonce[7] = ecbbuf[14];
	aesBuf.nonce[8] = ecbbuf[0];
	aesBuf.nonce[9] = ecbbuf[6];
	aesBuf.nonce[10] = ecbbuf[13]; 
	aesBuf.nonce[11] = ecbbuf[9]; 

//	uint8_t test_buf[16] = {0x01, 0x6e, 0x04, 0x08, 0x08, 0x06, 0x06,0x00,0x00,0x00,0x00,0x00,0,0,0,0};
//	uint8_t p_rx[20] = {0x00};
//	memcpy(aesBuf.plaintext, test_buf, 16);
//	app_aes_ccm_encypt(aesBuf.plaintext,aesBuf.ciphertext);	
//	hexprint(aesBuf.ciphertext,20);

//	memcpy(p_rx, aesBuf.ciphertext, 20);
	uint8_t p_rx[20] = {0x07,0x5e,0x2d,0xaa,0xdc,0xaa,0x67,0xb7,0xd5,0xcc,0xd4,0x95,0x30,0xbe,0x62,0x00,0xc4,0xd2,0xf7,0x92};
	hexprint(p_rx,20);
		
	uint8_t ret = app_aes_ccm_decrypt(aesBuf.plaintext, p_rx);	//解密RX
	if (ret == 0)
	{
		DBGPRINTF("app_aes_ccm_decrypt 0\r\n");
	}
	else{
		DBGPRINTF("app_aes_ccm_decrypt 1\r\n");
	}
	hexprint(aesBuf.plaintext,16);
}
