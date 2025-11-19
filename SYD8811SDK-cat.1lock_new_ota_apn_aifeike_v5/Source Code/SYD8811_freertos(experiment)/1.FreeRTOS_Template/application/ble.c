#define  _BLE_C

/* BLE */
#include "config.h"
#include "debug.h"
#include "SYD_ble_service_Profile.h"
#include "ble.h"
#include "lib.h"
#include "led_key.h"

/* freeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include <string.h>
#include "lock.h"
#include "..\application\softtimer.h"
#include "ota.h"
#include "lock.h"
#include "pwm.h"
#include "main.h"
#include "Cat1.h"
#include "aes_zl.h"

extern TimerHandle_t BLUE_LED_TIMER_EVT_ID;
extern TimerHandle_t SLEEP_TIMER_EVT_ID;
extern TimerHandle_t BLE_CONNECT_TIMEOUT_TIMER_EVT_ID;
extern TimerHandle_t MOTOR_TIMEOUT_TIMER_EVT_ID; 
extern TimerHandle_t KEY_DEBOUNCE_TIMER_EVT_ID;
extern TimerHandle_t LOCK_DEBOUNCE_TIMER_EVT_ID;  
extern TimerHandle_t HALL_TIGHTEND_TIMER_EVT_ID;
extern lock_info_t lock_info;
extern flash_data_t flash_data;
extern uint8_t ownAddress[];
//notify 标志 1代表已经使能 0代表未使能
static uint8_t start_tx = 1;
static uint8_t wechat_tx = 0;
static struct gap_ble_addr dev_addr;

uint8_t update_latency_mode = 0;
uint8_t latency_state = 0;

BLE_APP_gatt ble_gatt_data; 
 
uint8_t SCAN_DATA[] = {0};
uint16_t SCAN_DATA_SZ = 0; 

//MAC地址定义 CA:72:17:EF:AA:10
//Flags(0x02 + 0x01 + 0x06)
//Manufacturer Specific Data(0x09 + 0xff + 电量(1 byte) + 锁状态(1 byte) + 锁编号(6 byte))
//More 128-bit UUIDs available(0x11 + 0x06 + 128 Bit UUID(16 byte))
uint8_t advertData[] = {
	0x02,		// length
	0x01,		// AD Type: Flags
	0x05,		// LE Limited Discoverable Mode & BR/EDR Not Supported
	0x03,		// length
	0x02,		// AD Type: Complete list of 16-bit UUIDs 
	0xD0,		// UUID: Human Interface Device (0x0001)//12
	0xCC,		// UUID: Human Interface Device (0x0001)//18
	0x17,		// length 0X09
	0xFF,		// AD Type: MANUFACTURER SPECIFIC DATA
	
	0x00,
	0x00,
	0x00,
	0x00, 	//  random 4字节
	
	0x00,
	0x00, 	// 开锁 sn       *14
	
	HARD_VER, 	// 硬件版本  *15
	SOFT_VER_H,	
	SOFT_VER_L, //软件版本
	
	0x00,0x00,0x00,0x00,0x00,0x00,      //MAC兼容IOS6
	0x00,                               //key 更该标识  24
	0x00,0x00,0x00,0x00,0x00,0x00, 			//保留6位
};


//填充扫描广播包（设置广播包时使用）更改蓝牙名称
void scanRspData_set_dev_id(uint8_t *scanData, uint8_t *dev_id)//填充扫描广播包
{ 
	uint8_t len = dev_id[0];
	memcpy((uint8_t *)(scanData+2),&dev_id[1], len);
	scanData[0] = len + 1;
	scanData[2+len] = 0x05;
	scanData[3+len] = 0x12;
	scanData[4+len] = LO_UINT16(80);
	scanData[5+len] = HI_UINT16(80);
	scanData[6+len] = LO_UINT16(800);
	scanData[7+len] = HI_UINT16(800);
	scanData[8+len] = 0x02;
	scanData[9+len] = 0x0a;
	scanData[10+len] = 0x0;
}

//Complete local name(len (1byte) + 0x09(1byte) + device name(1-9byte))
//Min and Max values of the connection interval(0x05 + 0x12 + min(2 byte) + max(2byte))
//TX Power Level(0x02(1byte) + power level(1byte) + 0x00(1byte))

//设置广播包（蓝牙初始化时使用）
void setup_adv_data(void)
{
	struct gap_adv_params adv_params;

		uint8_t ScanData[] = {	
		0x0e,// length
		0x09,// AD Type: Complete local name
		'L',
		'i',
		'n',
		'k',
		'p',
		'o',
		'w',
		'e',
		'r',
		'L',
		'o',
		'c',
		'k',
		// connection interval range
		0x05,   // length of this data
		0x12,
		LO_UINT16(80),   // 100ms
		HI_UINT16(80),
		LO_UINT16(800),   // 1s
		HI_UINT16(800),

		// Tx power level
		0x02,   // length of this data
		0x0a,
		0       // 0dBm
	};
	adv_params.type		= ADV_IND;
	adv_params.channel 	= 0x07;  	// advertising channel : 37 & 38 & 39
	adv_params.interval = 480;		//1600; 	// advertising interval : 1000ms (1600 * 0.625ms)
	adv_params.timeout 	= 30;  		// timeout : 30s
	#if 0
	if(lock_info.ble_timeout != NULL& lock_info.ble_timeout != 0){
		adv_params.timeout 	= lock_info.ble_timeout; 
		lock_info.ble_timeout = 60;
	}else{
		lock_info.ble_timeout = 60;
		adv_params.timeout 	= 60;  		// timeout : 30s
	}
	#endif

	SetAdvParams(&adv_params);
	//开启蓝牙广播后30s自动关闭蓝牙
	xTimerStart(BLE_CONNECT_TIMEOUT_TIMER_EVT_ID, 0);
	//advertData[5] = lock_info.bat_val;
	//advertData[6] = lock_info.status;
  	//memcpy(&advertData[7], &flash_data.id[1], 6);
	/* get bluetooth address */
	GetDevAddr(&dev_addr);
//	advertData[7] = dev_addr.addr[5];    //BUG 扫描到的mac显示错误  MAC地址以广播包advertData为准
//	advertData[8] = dev_addr.addr[4];
//	advertData[9] = dev_addr.addr[3];
//	advertData[10] = dev_addr.addr[2];
//	advertData[11] = dev_addr.addr[1];
//	advertData[12] = dev_addr.addr[0];
	advertData[18] = dev_addr.addr[5];	  //BUG 扫描到的mac显示错误  MAC地址以广播包advertData为准
	advertData[19] = dev_addr.addr[4];
	advertData[20] = dev_addr.addr[3];
	advertData[21] = dev_addr.addr[2];
	advertData[22] = dev_addr.addr[1];
	advertData[23] = dev_addr.addr[0];

	ownAddress[0] = dev_addr.addr[5];    //BUG 扫描到的mac显示错误  MAC地址以广播包advertData为准
	ownAddress[1] = dev_addr.addr[4];
	ownAddress[2] = dev_addr.addr[3];
	ownAddress[3] = dev_addr.addr[2];
	ownAddress[4] = dev_addr.addr[1];
	ownAddress[5] = dev_addr.addr[0];	
	SetAdvData(advertData, sizeof(advertData), ScanData, sizeof(ScanData)); 
	//*(uint8_t *)0x40020014 = 0x01;  	//设置ADV_PITV为1，降低功耗
}

void BLSetConnectionUpdate(uint8_t target)
{
	struct gap_link_params  link_app;
	struct gap_smart_update_params smart_params;
	uint8_t buffer_cha1[5] = {0XFC, 0X01, 0X00, 0X00, 0X00};
	GetLinkParameters(&link_app);
	DBGPRINTF("interval:%x latency:%x\r\n",link_app.interval,link_app.latency);
	
	switch(target){
		//快速
		case 0: 
				if((link_app.latency != 0) && (link_app.interval > 0x10)){
					latency_state = 0;
					/* connection parameters */
					smart_params.updateitv_target = 0x0010;  //target connection interval (60 * 1.25ms = 75 ms)
					smart_params.updatesvto = 0x00c8;  //supervisory timeout (400 * 10 ms = 4s)
					smart_params.updatelatency = 0x0000;
					smart_params.updatectrl = SMART_CONTROL_LATENCY | SMART_CONTROL_UPDATE;
					smart_params.updateadj_num = MAX_UPDATE_ADJ_NUM;
					gap_s_smart_update_latency(&smart_params);
				}
				DBGPRINTF(("SetUpdate ota link\r\n"));
				BLE_SendData(buffer_cha1,5);
		break;
		//慢速
		case 1:
				if((link_app.latency < 0x000A) && (link_app.interval < 0x0050)){
					/* connection parameters */
					smart_params.updateitv_target = 0x0050;
					smart_params.updatelatency = 0x000A;
					smart_params.updatesvto = 0x0258;
					smart_params.updatectrl = SMART_CONTROL_LATENCY | SMART_CONTROL_UPDATE;
					smart_params.updateadj_num = MAX_UPDATE_ADJ_NUM;
					gap_s_smart_update_latency(&smart_params);	   
					DBGPRINTF(("SetUpdate ios link\r\n"));
				}
		break;
	}
	#ifdef _OTA_
	smart_params.updatectrl=SMART_CONTROL_LATENCY | SMART_CONTROL_UPDATE;
	smart_params.updateadj_num=MAX_UPDATE_ADJ_NUM;
	gap_s_smart_update_latency(&smart_params);
   #if defined(_DEBUG_) || defined(_SYD_RTT_DEBUG_)
	dbg_printf("smart_params interval:%x latency:%x svto:%x\r\n",smart_params.updateitv_target,smart_params.updatelatency,smart_params.updatesvto);
	#endif
	#endif
}

static void ble_gatt_read(struct gap_att_read_evt evt)
{
	if(evt.uuid == BLE_DEVICE_NAME_UUID)
	{
		uint8_t gatt_buf[16]={'L', 'o', 'c', 'k'};
//    if (flash_data.modify_name_flag == 0)
    {
        char id_buf[12] = {0};
        ByteToHexStr((char *)ownAddress, id_buf, 6);
        memcpy((uint8_t *)(&gatt_buf[4]), id_buf, 12);
    }
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 
		SetGATTReadRsp(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_UART_NOTIFY_UUID)
	{
		//uint8_t gatt_buf[20]={1, 2};
    DBGPRINTF("BLE_UART_NOTIFY_UUID\r\n");
		uint8_t gatt_buf_sz = 20;//sizeof(gatt_buf);     
		SetGATTReadRsp(gatt_buf_sz, ble_gatt_data.send_buf);
	}
	else if(evt.uuid == BLE_APPEARANCE_UUID)
	{
		uint8_t gatt_buf[]={0xff, 0xff};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 
		SetGATTReadRsp(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_MANUFACTURER_NAME_STRING_UUID)
	{
		uint8_t gatt_buf[]={'L','P',' ','I', 'n', 'c', '.'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 
		SetGATTReadRsp(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_MODEL_NUMBER_STRING_UUID)
	{
		uint8_t gatt_buf[]={'B', 'L', 'E', ' ', '1', '.', '0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 
		SetGATTReadRsp(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_SERIAL_NUMBER_STRING_UUID)
	{
		uint8_t gatt_buf[]={'1','.','0','.','0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 
		SetGATTReadRsp(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_HARDWARE_REVISION_STRING_UUID)
	{
		uint8_t gatt_buf[]={'2','.','0','0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 
		SetGATTReadRsp(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_FIRMWARE_REVISION_STRING_UUID)
	{
		uint8_t gatt_buf[]={'3','.','0','0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 
		SetGATTReadRsp(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_SOFTWARE_REVISION_STRING_UUID)
	{
		uint8_t gatt_buf[]={'4','.','0','0'};
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 
		SetGATTReadRsp(gatt_buf_sz, gatt_buf);
	}
	else if(evt.uuid == BLE_PNP_ID_UUID)
	{
		uint8_t gatt_buf[]={ 0x02, 						//		Vendor ID Source
						    0x3a,0x09,					//		USB Vendor ID
						    0x05,0x0a,					//		Product ID
						    0x02,0x00					//		Product Version
												 };
		uint8_t gatt_buf_sz = sizeof(gatt_buf); 
		SetGATTReadRsp(gatt_buf_sz, gatt_buf);
	}
	#ifdef _OTA_
	else if(evt.uuid == BLE_OTA_Read_Write_UUID)
	{
		uint8_t sz=0;
		uint8_t rsp[sizeof(struct hci_evt)]={0};
		ota_rsp(rsp, &sz);		
		SetGATTReadRsp(sz, rsp);
	}
	#endif
}

//接收函数
static void ble_gatt_write(struct gap_att_write_evt evt)
{
	#ifdef _OTA_
	if(evt.uuid== BLE_OTA_Read_Write_UUID)
	{
		update_latency_mode=0;
		ota_cmd(evt.data, evt.sz);
	} else
	#endif
	DBGPRINTF("receive app data %x\r\n",evt.uuid);//  BLE RX 
	if(evt.uuid== BLE_UART_Write_UUID)
	{
    DBGPRINTF("BLE_UART_Write_UUID \r\n");
		Gatt_Rx_Handle((uint8_t*)evt.data);
	}
}

//发送函数
uint8_t BLE_SendData(uint8_t *buf, uint8_t len)
{
  //启用通知
	start_tx = 1;
	if(start_tx == 1)
	{
		struct gap_att_report report;
		if(wechat_tx)
		{
			//report.primary = BLE_WECHAT;
			//report.uuid = BLE_WECHAT_Indication_UUID;
			//report.hdl = BLE_WECHAT_Indication_VALUE_HANDLE;					
			//report.value = BLE_GATT_INDICATION;
			//if(len > 20) 
			//	len = 20;
			//return GATTDataSend(BLE_GATT_INDICATION, &report, len, buf);
		}
		else
		{
			memcpy(ble_gatt_data.send_buf, buf, 20);//同步给读属性
			report.primary = BLE_UART;
			report.uuid = BLE_UART_NOTIFY_UUID;
			report.hdl = BLE_UART_NOTIFY_VALUE_HANDLE;					
			report.value = BLE_GATT_NOTIFICATION;
			return GATTDataSend(BLE_GATT_NOTIFICATION, &report, len, buf);
			//DBGPRINTF("tx %x %x\r\n",start_tx,BLE_UART);//  BLE RX 
		}
	}
	return 0;
}

//蓝牙状态回调函数
void ble_evt_callback(struct gap_ble_evt *p_evt)
{
	//广播结束
	if(p_evt->evt_code == GAP_EVT_ADV_END)
	{		
		DBGPRINTF(("GAP_EVT_ADV_END\r\n"));
		xTimerStop(BLUE_LED_TIMER_EVT_ID, 0);
    xTimerStop(BLE_CONNECT_TIMEOUT_TIMER_EVT_ID, 0);
				
		GPIO_Pin_Set(U32BIT(BLUE_LED));	
    GPIO_Pin_Set(U32BIT(GREEN_LED));
		taskENTER_CRITICAL();
		RFSleep();
		taskEXIT_CRITICAL();
		if ((lock_info.nb_flag == 1) && (xTimerIsTimerActive(MOTOR_TIMEOUT_TIMER_EVT_ID) == pdFALSE) && \
   (xTimerIsTimerActive(KEY_DEBOUNCE_TIMER_EVT_ID) == pdFALSE) && (xTimerIsTimerActive(LOCK_DEBOUNCE_TIMER_EVT_ID) == pdFALSE))
		{
			gpio_pin_sleep(LOW_POWER);		
			GPIO_Pin_Clear(U32BIT(CAT1_PWR_CTRL));	
			UartEn(false);
			if (get_gps_state())
				GPIO_Pin_Set(U32BIT(GNSS_PWR_EN));

      DBGPRINTF(("GAP_EVT_ADV_END sleep\r\n"));      
			lock_info.sleep_flag = 1;	
		}
    lock_info.ble_working = 0;	
	}
	//蓝牙处于连接状态
	else if(p_evt->evt_code == GAP_EVT_CONNECTED)	
	{		
		latency_state = 0;
		update_latency_mode = 0;
		//718，改成恒定30s关闭
		//xTimerStart(BLE_CONNECT_TIMEOUT_TIMER_EVT_ID, 0);
		lock_info.ble_connected = 1;
		xTimerChangePeriod(BLUE_LED_TIMER_EVT_ID, 300, 0);
		DBGPRINTF("GAP_EVT_CONNECTED\r\n");		
//		BLSetConnectionUpdate(1);
	}
	//蓝牙断开连接
	else if(p_evt->evt_code == GAP_EVT_DISCONNECTED)
	{	
		//718，改成恒定30s关闭
		//xTimerStop(BLE_CONNECT_TIMEOUT_TIMER_EVT_ID, 0);		//关闭蓝牙超时
		xTimerChangePeriod(BLUE_LED_TIMER_EVT_ID, 2000, 0);						 //连接状态
		StartAdv();
		lock_info.ble_connected = 0;
		if (lock_info.ble_update_flags) //蓝牙升级过程中，强行中断处理
			SystemReset();
		DBGPRINTF("GAP_EVT_DISCONNECTED(%02x)\r\n",p_evt->evt.disconn_evt.reason);	
	}
	else if(p_evt->evt_code == GAP_EVT_ATT_HANDLE_CONFIGURE)
	{					
		if(p_evt->evt.att_handle_config_evt.uuid == BLE_UART)
		{
//			if(p_evt->evt.att_handle_config_evt.hdl == (BLE_WECHAT_Indication_VALUE_HANDLE + 1))
//			{			
//				if(p_evt->evt.att_handle_config_evt.value == BLE_GATT_NOTIFICATION)
//				{
//					DBGPRINTF(("start_tx enable\r\n"));
//					start_tx = 1;
//				}
//				else
//				{			
//					start_tx = 0;
//				}
//			}
		}	
		else if(p_evt->evt.att_handle_config_evt.uuid == BLE_UART)
		{
//			if(p_evt->evt.att_handle_config_evt.hdl == (BLE_WECHAT_Indication_VALUE_HANDLE + 1))
//			{
//				if(p_evt->evt.att_handle_config_evt.value == BLE_GATT_NOTIFICATION)
//					wechat_tx = 1;
//				else
//					wechat_tx = 0;
//			}
		}
//DBGPRINTF(("GAP_EVT_ATT_HANDLE_CONFIGURE uuid:(%02x)\r\n",p_evt->evt.att_handle_config_evt.uuid));
	}
	//接收手机下发的数据
	else if(p_evt->evt_code == GAP_EVT_ATT_WRITE)
	{
		ble_gatt_write(p_evt->evt.att_write_evt);
	}
	else if(p_evt->evt_code == GAP_EVT_ATT_READ)
	{
		ble_gatt_read(p_evt->evt.att_read_evt);
    DBGPRINTF("GAP_EVT_ATT_READ uuid:(%02x)\r\n",p_evt->evt.att_write_evt.uuid);
	}
	else if(p_evt->evt_code == GAP_EVT_ATT_HANDLE_CONFIRMATION)
	{
//DBGPRINTF(("GAP_EVT_ATT_HANDLE_CONFIRMATION uuid:(%02x)\r\n",p_evt->evt.att_handle_config_evt.uuid));
	}
	else if(p_evt->evt_code == GAP_EVT_ENC_START)
	{
		DBGPRINTF(("GAP_EVT_ENC_START\r\n"));
	}
	else if(p_evt->evt_code == GAP_EVT_L2CAP_UPDATE_RSP)
	{	
		switch(p_evt->evt.l2cap_update_rsp_evt.result)
		{
			case CONN_PARAMS_ACCEPTED:
			{
				DBGPRINTF("\r\nupdate rsp ACCEPTED.");
			}
			break;
			case CONN_PARAMS_REJECTED:
			{
				DBGPRINTF("update rsp REJECTED.");
			}
			break; 
			case CONN_PARAM_SMART_TIMEROUT:
			{
				DBGPRINTF("Update rsp TIMEROUT.");
			}
			break;
			case CONN_PARAM_SMART_SUCCEED:
			{
				struct gap_link_params  link_app;
				GetLinkParameters(&link_app);
				DBGPRINTF("\r\nupdate rsp SUCCEED.");
				DBGPRINTF("\r\ninterval:%fms, latency:%d.\r\n", (float)link_app.interval*1.25, link_app.latency);
			}
			break;
			case CONN_PARAM_LATENCY_ENABLE:
			{
				//DBGPRINTF("\r\nEnable latency.\r\n");
			}
			break;
			case CONN_PARAM_LATENCY_DISABLE:
			{
				//DBGPRINTF("\r\nDisable latency.");
			}
			break;
		}
	}
	#ifdef _OTA_
	else if(p_evt->evt_code == GAP_EVT_CONN_UPDATE_COMP)
	{
		struct gap_link_params link;
		GetLinkParameters(&link);
		DBGPRINTF("CONN_UPDATE_COMP: %d, %d, %d\r\n", link.interval, link.latency, link.svto);
	}
	#endif
}

void ble_init(void)
{	
	struct gap_att_report_handle *g_report;
	struct smp_pairing_req sec_params;
	struct gap_evt_callback evt;	

	BleInit();
	GetGATTReportHandle(&g_report);

	/* security parameters */
	sec_params.io = IO_DISPLAY_ONLY;
	sec_params.oob = OOB_AUTH_NOT_PRESENT;
	sec_params.flags = AUTHREQ_BONDING;
	sec_params.mitm = 1;
	sec_params.max_enc_sz = 16;
	sec_params.init_key = 0;
	sec_params.rsp_key = (SMP_KEY_MASTER_IDEN | SMP_KEY_ADDR_INFO);
	SetSecParams(&sec_params);
	
	evt.evt_mask = (GAP_EVT_CONNECTION_SLEEP | GAP_EVT_CONNECTION_INTERVAL);
	evt.p_callback = &ble_evt_callback;
	SetEvtCallback(&evt);

	/* Bond Manager (MAX:10) */
	SetBondManagerIndex(0x00);
	//设置广播包
	//setup_adv_data();
	//JLINK接上也不能进入低功耗
	/* Configure the wake up source */
//  struct gap_wakeup_config pw_cfg;
//	pw_cfg.wakeup_type     = SLEEP_WAKEUP;
//	pw_cfg.wdt_wakeup_en   = false;
//	pw_cfg.rtc_wakeup_en   = true;
//	pw_cfg.timer_wakeup_en = false;
//	pw_cfg.gpi_wakeup_en   = true;
//	pw_cfg.gpi_wakeup_cfg  = U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS);
//	WakeupConfig(&pw_cfg);
	GetDevAddr(&dev_addr);
	ownAddress[0] = dev_addr.addr[5];    //BUG 扫描到的mac显示错误  MAC地址以广播包advertData为准
	ownAddress[1] = dev_addr.addr[4];
	ownAddress[2] = dev_addr.addr[3];
	ownAddress[3] = dev_addr.addr[2];
	ownAddress[4] = dev_addr.addr[1];
	ownAddress[5] = dev_addr.addr[0];	
	DBGPRINTF("DeAES 1(%x_%x_%x_%x_%x_%x)\r\n",ownAddress[0],ownAddress[1],ownAddress[2],ownAddress[3],ownAddress[4],ownAddress[5]);
}

void wake_up_souce_cfg(uint8_t index, uint8_t flag)
{
  struct gap_wakeup_config pw_cfg;
  pw_cfg.wakeup_type     = SLEEP_WAKEUP;
  pw_cfg.wdt_wakeup_en   = false;
  pw_cfg.rtc_wakeup_en   = true;
  pw_cfg.timer_wakeup_en = false;
  pw_cfg.gpi_wakeup_en   = true;
  switch(index)
  {
    case 0:
    if (flag)
    {
      pw_cfg.gpi_wakeup_cfg  = U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS) | U32BIT(BAT_DET); 
    }
    else
    {
      pw_cfg.gpi_wakeup_cfg  = U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS) | U32BIT(BAT_DET);
    }
      break;
    case 1:
      pw_cfg.gpi_wakeup_cfg  = U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS) | U32BIT(BAT_DET); 
      break;
    case 2:
      pw_cfg.gpi_wakeup_cfg  = U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS); 
      break;
    case 3:
      if ((!GPIO_Pin_Read(U32BIT(BAT_DET))))
      {
        pw_cfg.gpi_wakeup_cfg  = U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS) | U32BIT(BAT_DET); 
      }
      else
      {
        pw_cfg.gpi_wakeup_cfg  = U32BIT(WakeUp_KEY) | U32BIT(LOCK_STATUS) | U32BIT(BAT_DET); 
      }
    default:
      break;
  }  
  WakeupConfig(&pw_cfg); 
}

//蓝牙初始化（main初始化时使用）
void wake_up_ble_init(void)
{
	struct gap_att_report_handle *g_report;
	struct smp_pairing_req sec_params;
	struct gap_evt_callback evt;	

	BleInit();
	GetGATTReportHandle(&g_report);

	/* security parameters */
	sec_params.io = IO_DISPLAY_ONLY;
	sec_params.oob = OOB_AUTH_NOT_PRESENT;
	sec_params.flags = AUTHREQ_BONDING;
	sec_params.mitm = 1;
	sec_params.max_enc_sz = 16;
	sec_params.init_key = 0;
	sec_params.rsp_key = (SMP_KEY_MASTER_IDEN | SMP_KEY_ADDR_INFO);
	SetSecParams(&sec_params);
	
	evt.evt_mask = (GAP_EVT_CONNECTION_SLEEP | GAP_EVT_CONNECTION_INTERVAL);
	evt.p_callback = &ble_evt_callback;
	SetEvtCallback(&evt);

	/* Bond Manager (MAX:10) */
	SetBondManagerIndex(0x00);
	setup_adv_data();
}
