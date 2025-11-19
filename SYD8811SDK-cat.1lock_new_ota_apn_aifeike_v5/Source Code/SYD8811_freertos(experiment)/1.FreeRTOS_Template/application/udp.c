#include "udp.h"
#include "gpio.h"
#include "led_key.h"
#include "debug.h"
#include "main.h"
#include "DebugLog.h"
#include "Cat1.h"

uint8_t udp_buf[E_CAT1_ATTR_TYPE_NUM][8] = {0};
	
static cat1_attr_t  s_cat1_attr_array[E_CAT1_ATTR_TYPE_NUM] = 
{
  {E_CAT1_ATTR_TYPE_lockStatus,      0, 0, 0x0001,  1, 0, NULL},
  {E_CAT1_ATTR_TYPE_batteryLevel,    0, 0, 0x0002,  1, 0, NULL},
  {E_CAT1_ATTR_TYPE_rssi,            0, 0, 0x0004,  1, 0, NULL},
  {E_CAT1_ATTR_TYPE_softwareVersion, 1, 0, 0x0008,  2, 0, NULL},
  {E_CAT1_ATTR_TYPE_hardwareVersion, 1, 0, 0x0010,  2, 0, NULL},
  {E_CAT1_ATTR_TYPE_longitude,       1, 0, 0x0020,  5, 0, NULL},
  {E_CAT1_ATTR_TYPE_latitude,        1, 0, 0x0040,  5, 0, NULL},
  {E_CAT1_ATTR_TYPE_alarm,           0, 0, 0x0080,  1, 0, NULL},
  {E_CAT1_ATTR_TYPE_positionReport,  0, 0, 0x0100,  1, 0, NULL},        
  {E_CAT1_ATTR_TYPE_periodReport,    0, 0, 0x0200,  1, 0, NULL},
  {E_CAT1_ATTR_TYPE_update,       	 1, 0, 0x0400,  4, 0, NULL},
  {E_CAT1_ATTR_TYPE_imsi,            1, 0, 0x0800,  8, 0, NULL},
  {E_CAT1_ATTR_TYPE_hall,            0, 0, 0x1000,  1, 0, NULL},
  {E_CAT1_ATTR_TYPE_ip,              1, 0, 0x2000,  6, 0, NULL},
  {E_CAT1_ATTR_TYPE_shutdownCtrl,    0, 0, 0x4000,  1, 0, NULL},
	{E_CAT1_ATTR_TYPE_gps,             0, 0, 0x8000,  1, 0, NULL},
};

static int cal_crc(char *data, uint8_t len)
{
  uint8_t i;
  uint16_t reg_crc = 0xffff;
  while(len--) {
    reg_crc ^= *data++;
    for(i = 0; i < 8; i++) {
      if(reg_crc & 0x01) {
        reg_crc = (reg_crc >> 1) ^ 0xA001;
      } else {
        reg_crc = reg_crc >> 1;
      }
    }
  }
  return reg_crc;
}

//获取发送信息
static void udp_send_unlock_req(lock_info_t *info, uint8_t *send_buf)
{
  uint8_t tmp[30] = {0};
  uint16_t crc = 0;
  uint16_t dataType = 0;
  uint8_t count = 0;
  //包头
  tmp[0] = CAT1_HEAD;   
  //开锁请求 
  tmp[1] = CAT1_UNLOCK_REQUEST_CMD;   
  //IMEI号
  memcpy(&tmp[2], info->imei, 8);
  //长度
  tmp[10] = 10;    
  /*
  E_CAT1_ATTR_TYPE_lockStatus,
  E_CAT1_ATTR_TYPE_batteryLevel,
  E_CAT1_ATTR_TYPE_rssi,
  E_CAT1_ATTR_TYPE_softwareVersion,
  E_CAT1_ATTR_TYPE_hardwareVersion,*/
  //数据段 101f
  dataType = s_cat1_attr_array[E_CAT1_ATTR_TYPE_lockStatus].attr_type | s_cat1_attr_array[E_CAT1_ATTR_TYPE_batteryLevel].attr_type | \
             s_cat1_attr_array[E_CAT1_ATTR_TYPE_rssi].attr_type | s_cat1_attr_array[E_CAT1_ATTR_TYPE_softwareVersion].attr_type  |  \
             s_cat1_attr_array[E_CAT1_ATTR_TYPE_hardwareVersion].attr_type|s_cat1_attr_array[E_CAT1_ATTR_TYPE_hall].attr_type;
  tmp[11] = (dataType >> 8) & 0xff;
  tmp[12] = dataType & 0xff;

  count = s_cat1_attr_array[E_CAT1_ATTR_TYPE_lockStatus].attr_length + 12;

  if (GPIO_Pin_Read(U32BIT(MOTOR_STATUS_CLOSE)))
  {
     tmp[count] = 0x02;           //关锁状态
  }
  else
  {
    tmp[count] = 0x01;            //开锁状态
  }
  count += s_cat1_attr_array[E_CAT1_ATTR_TYPE_batteryLevel].attr_length;
  tmp[count] = info->bat_val;      //电量 
  count += s_cat1_attr_array[E_CAT1_ATTR_TYPE_rssi].attr_length;
  tmp[count] = info->dBm; 
  
  count++;
  tmp[count++] = info->sw >> 8;    //软件版本号
  tmp[count++] = info->sw & 0xff;
  tmp[count++] = 0x00;				     //硬件版本号
  tmp[count++] = info->hw;	  
//	tmp[count++] = 0x01;      //关闭状态	
  if (GPIO_Pin_Read(U32BIT(HALL_DET)))
  { 
    tmp[count++] = 0x01;       //打开状态
  }
  else
  {
    tmp[count++] = 0x02;      //关闭状态
  }
  //crc校验
  crc = cal_crc((char *)tmp, count);
  tmp[count++] = (uint8_t)(crc >> 8);	
  tmp[count++] = (uint8_t)crc;  
  memcpy(send_buf, tmp, count);
  DBGPRINTF("SUB_QISEND udp_send_unlock_req: ");
  hexprint(tmp, count);
}
// AB 0A 08 63 85 30 54 75 15 48 09 00 97 01 51 60 20 07 00 0A 49 53
// AB 0C 08 63 85 30 54 75 15 48 03 00 01 01 21 0F
//上报开锁状态
static void udp_send_lock_status(lock_info_t *info, uint8_t *send_buf)
{
  uint8_t tmp[20] = {0};
  uint16_t crc = 0;
  uint16_t dataType = 0;
  uint8_t count = 0;
  //包头
  tmp[0] = CAT1_HEAD;   
  tmp[1] = CAT1_LOCK_STATUS_CMD;   
  //IMEI号
  memcpy(&tmp[2], info->imei, 8);
  //长度
  tmp[10] = 4;    
  /*
  E_CAT1_ATTR_TYPE_lockStatus,
  */
  //数据段
  dataType = s_cat1_attr_array[E_CAT1_ATTR_TYPE_lockStatus].attr_type | s_cat1_attr_array[E_CAT1_ATTR_TYPE_hall].attr_type;
  tmp[11] = (dataType >> 8) & 0xff;
  tmp[12] = dataType & 0xff;
  count = s_cat1_attr_array[E_CAT1_ATTR_TYPE_lockStatus].attr_length + 12;
  tmp[count++] = info->data.state;     
	
	tmp[count++] = 0x01;      //关闭状态   
  //crc校验
  crc = cal_crc((char *)tmp, count);
  tmp[count++] = (uint8_t)(crc >> 8);	
  tmp[count++] = (uint8_t)crc;  
  memcpy(send_buf, tmp, count);
  DBGPRINTF("SUB_QISEND udp_send_lock_status: ");
  hexprint(tmp,count);
}

//上报告警信息
static void udp_send_lock_alarm(lock_info_t *info, uint8_t *send_buf)
{
  uint8_t tmp[20] = {0};
  uint16_t crc = 0;
  uint16_t dataType = 0;
  uint8_t count = 0;
  //包头
  tmp[0] = CAT1_HEAD;   
  tmp[1] = CAT1_LOCK_ALARM_CMD;   
  //IMEI号
  memcpy(&tmp[2], info->imei, 8);
  //长度
  tmp[10] = 3;    
  /*
  E_CAT1_ATTR_TYPE_alarm,
  */
  //数据段
  dataType = s_cat1_attr_array[E_CAT1_ATTR_TYPE_alarm].attr_type;
  tmp[11] = (dataType >> 8) & 0xff;
  tmp[12] = dataType & 0xff;
  count = s_cat1_attr_array[E_CAT1_ATTR_TYPE_alarm].attr_length + 12;
  tmp[count++] = info->data.state;        

  //crc校验
  crc = cal_crc((char *)tmp, count);
  tmp[count++] = (uint8_t)(crc >> 8);	
  tmp[count++] = (uint8_t)crc;  
  memcpy(send_buf, tmp, count);
  DBGPRINTF("SUB_QISEND alarm:%d: ", info->data.state);
  hexprint(tmp,count);
}

//周期性上报  AB0E0860355079119852 0a 101F 02 60 34 1008 0003 02 C1CC
static void udp_send_period_report(lock_info_t *info, uint8_t *send_buf)
{
  uint8_t tmp[30] = {0};
  uint16_t crc = 0;
  uint16_t dataType = 0;
  uint8_t count = 0;
  //包头
  tmp[0] = CAT1_HEAD;   
  tmp[1] = CAT1_LOCK_PERIOD_CMD;   
  //IMEI号
  memcpy(&tmp[2], info->imei, 8);
  //长度
  tmp[10] = 10;    
  /*
  E_CAT1_ATTR_TYPE_lockStatus,
  E_CAT1_ATTR_TYPE_batteryLevel,
  E_CAT1_ATTR_TYPE_rssi,
	E_CAT1_ATTR_TYPE_softwareVersion,
  E_CAT1_ATTR_TYPE_hardwareVersion,
	E_CAT1_ATTR_TYPE_hall
  */
  //数据段 1007
  dataType = s_cat1_attr_array[E_CAT1_ATTR_TYPE_lockStatus].attr_type | s_cat1_attr_array[E_CAT1_ATTR_TYPE_batteryLevel].attr_type | \
             s_cat1_attr_array[E_CAT1_ATTR_TYPE_rssi].attr_type | s_cat1_attr_array[E_CAT1_ATTR_TYPE_softwareVersion].attr_type |   \
						 s_cat1_attr_array[E_CAT1_ATTR_TYPE_hardwareVersion].attr_type |s_cat1_attr_array[E_CAT1_ATTR_TYPE_hall].attr_type;
  tmp[11] = (dataType >> 8) & 0xff;
  tmp[12] = dataType & 0xff;

  count = s_cat1_attr_array[E_CAT1_ATTR_TYPE_lockStatus].attr_length + 12;
  
	if (GPIO_Pin_Read(U32BIT(MOTOR_STATUS_CLOSE)) || (info->unlock_flag == 0))
  {
     tmp[count] = 0x02;            //关锁状态
  }
  else
  {
		if(GPIO_Pin_Read(U32BIT(MOTOR_STATUS_OPEN)))
		{
			tmp[count] = 0x01;            //开锁状态
		}
    else{
			tmp[count] = 0x02; 
		} 
  }	
	
  count += s_cat1_attr_array[E_CAT1_ATTR_TYPE_batteryLevel].attr_length;
  tmp[count] = info->bat_val;     //电量 
  count += s_cat1_attr_array[E_CAT1_ATTR_TYPE_rssi].attr_length;
  tmp[count++] = info->dBm; 
	
  tmp[count++] = info->sw >> 8;    //软件版本号
  tmp[count++] = info->sw & 0xff;
  tmp[count++] = 0x00;				     //硬件版本号
  tmp[count++] = info->hw;	
	tmp[count++] = 0x01;      //关闭状态

  //crc校验
  crc = cal_crc((char *)tmp, count);
  tmp[count++] = (uint8_t)(crc >> 8);	
  tmp[count++] = (uint8_t)crc;  
  memcpy(send_buf, tmp, count);
  DBGPRINTF("SUB_QISEND udp_send_period_report: ");
  hexprint(tmp,count);
}

//AB0F08685580662081800C00601138354030022611322037CE
//AB0F08685580662081800C0060113837311002261154701502
//AB0F08685580662081800C00601138355330022611469042CA
// AB0F08603550791198520C006011383649405226118600140B

//位置信息上报
static uint8_t udp_send_lock_lbs(lock_info_t *info, uint8_t *send_buf, location_info_t *lbs_pinfo)
{
  uint8_t tmp[30] = {0};
  uint16_t crc = 0;
  uint16_t dataType = 0;

  //包头
  tmp[0] = CAT1_HEAD;   
  tmp[1] = CAT1_LOCK_POSITION_CMD;   
  //IMEI号
  memcpy(&tmp[2], info->imei, 8);
  //长度
  tmp[10] = 12;    
  /*
  E_CAT1_ATTR_TYPE_longitude,
  E_CAT1_ATTR_TYPE_latitude，
  */
  //数据段
  dataType = s_cat1_attr_array[E_CAT1_ATTR_TYPE_longitude].attr_type | s_cat1_attr_array[E_CAT1_ATTR_TYPE_latitude].attr_type;
  tmp[11] = (dataType >> 8) & 0xff;
  tmp[12] = dataType & 0xff;
  //经度
  tmp[13] = ((lbs_pinfo->longitude_degree[0] - 0x30) & 0x0f) << 4 | ((lbs_pinfo->longitude_degree[1] - 0x30) & 0x0f);
  tmp[14] = ((lbs_pinfo->longitude_degree[2] - 0x30) & 0x0f) << 4 | ((lbs_pinfo->longitude_minute[0] - 0x30) & 0x0f);
  tmp[15] = ((lbs_pinfo->longitude_minute[1] - 0x30) & 0x0f) << 4 | ((lbs_pinfo->longitude_minute[2] - 0x30) & 0x0f);
  tmp[16] = ((lbs_pinfo->longitude_minute[3] - 0x30) & 0x0f) << 4 | ((lbs_pinfo->longitude_minute[4] - 0x30) & 0x0f);
  tmp[17] = ((lbs_pinfo->longitude_minute[5] - 0x30) & 0x0f) << 4;
  //纬度
  tmp[18] = ((lbs_pinfo->latitude_degree[0] - 0x30) & 0x0f) << 4 | ((lbs_pinfo->latitude_degree[1] - 0x30) & 0x0f);
  tmp[19] = ((lbs_pinfo->latitude_degree[2] - 0x30) & 0x0f) << 4 | ((lbs_pinfo->latitude_minute[0] - 0x30) & 0x0f);
  tmp[20] = ((lbs_pinfo->latitude_minute[1] - 0x30) & 0x0f) << 4 | ((lbs_pinfo->latitude_minute[2] - 0x30) & 0x0f);
  tmp[21] = ((lbs_pinfo->latitude_minute[3] - 0x30) & 0x0f) << 4 | ((lbs_pinfo->latitude_minute[4] - 0x30) & 0x0f);
  tmp[22] = ((lbs_pinfo->latitude_minute[5] - 0x30) & 0x0f) << 4;
    
  //crc校验
  crc = cal_crc((char *)tmp, 23);
  tmp[23] = (uint8_t)(crc >> 8);	
  tmp[24] = (uint8_t)crc;  
  memcpy(send_buf, tmp, 25);
  DBGPRINTF("SUB_QISEND lbs: ");
  hexprint((uint8_t *)send_buf,25);
	return 1;
}

//4G升级后上报
static void udp_send_update(lock_info_t *info, uint8_t *send_buf)
{
  uint8_t tmp[30] = {0};
  uint16_t crc = 0;
  uint16_t dataType = 0;
  uint8_t count = 0;
  //包头
  tmp[0] = CAT1_HEAD;   
  tmp[1] = CAT1_LOCK_UPDATE_CMD;   
  //IMEI号
  memcpy(&tmp[2], info->imei, 8);
  //长度
  tmp[10] = 6;    
  /*
	E_CAT1_ATTR_TYPE_softwareVersion,
  E_CAT1_ATTR_TYPE_hardwareVersion,
  */
  //数据段 0018
  dataType = s_cat1_attr_array[E_CAT1_ATTR_TYPE_softwareVersion].attr_type | s_cat1_attr_array[E_CAT1_ATTR_TYPE_hardwareVersion].attr_type;
  tmp[11] = (dataType >> 8) & 0xff;
  tmp[12] = dataType & 0xff;
	
  count = 13;
  tmp[count++] = info->sw >> 8;    //软件版本号
  tmp[count++] = info->sw & 0xff;
  tmp[count++] = 0x00;				     //硬件版本号
  tmp[count++] = info->hw;	
 
  //crc校验
  crc = cal_crc((char *)tmp, count);
  tmp[count++] = (uint8_t)(crc >> 8);	
  tmp[count++] = (uint8_t)crc;  
  memcpy(send_buf, tmp, count);
  DBGPRINTF("SUB_QISEND udp_send_update: ");
  hexprint(tmp,count);
}

//锁绳拉紧状态上报
static void udp_send_tightend(lock_info_t *info, uint8_t *send_buf)
{
  uint8_t tmp[30] = {0};
  uint16_t crc = 0;
  uint16_t dataType = 0;
  uint8_t count = 0;
  //包头
  tmp[0] = CAT1_HEAD;   
  tmp[1] = CAT1_TIGHTEND_CMD;   
  //IMEI号
  memcpy(&tmp[2], info->imei, 8);
  //长度
  tmp[10] = 3;    
  /*
	E_CAT1_ATTR_TYPE_hall,
  */
  //数据段 0018
  dataType = s_cat1_attr_array[E_CAT1_ATTR_TYPE_hall].attr_type;
  tmp[11] = (dataType >> 8) & 0xff;
  tmp[12] = dataType & 0xff;
	
  count = 13;
	tmp[count++] = info->data.state;  

  //crc校验
  crc = cal_crc((char *)tmp, count);
  tmp[count++] = (uint8_t)(crc >> 8);	
  tmp[count++] = (uint8_t)crc;  
  memcpy(send_buf, tmp, count);
  DBGPRINTF("SUB_QISEND udp_send_tightend: ");
  hexprint(tmp,count);
}

//处理接收的信息  
//2B51495552433A202272 6563 76 22 2C 31 2C 31 37 0D 0A AB0B0860061061084290 04 02 01 03 37 20 1A  0D 0A 
static uint8_t udp_recv_msg_parse(char *recv_buf, flash_data_t *flash_data)
{
  char *tmp = NULL;
  char *str = "+QIURC:";
  uint8_t len = 0;
  uint8_t ret = 0;
  uint16_t type = 0;
	msg_info_t msg_info;
  
  tmp = strstr(recv_buf, str);
  DBGPRINTF("recv msg\r\n");
  if (tmp)
  {   
    len = (*(tmp+17) - 0x30) * 10 + (*(tmp+18) - 0x30);
    for (uint8_t i = 0; i <= 200; i++)
    {
      if ((uint8_t)*tmp == 0xab)
        break;
      tmp++;
    }
		DBGPRINTF("recv msg %d\r\n", len);
    memcpy(&msg_info, tmp, len);
		for (uint8_t i = 0; i < len; i++)
		{
			DBGPRINTF("%02x ", tmp[i]);
		}
    DBGPRINTF("\r\n");
// 860355079119852
//    uint8_t test_buf[] = {0xAB, 0x0C, 0x08, 0x60, 0x35, 0x50, 0x79, 0x11, 0x98, 0x52, 0x03, 0x80, 0x00, 0x00, 0xaa, 0xbb};
//    memcpy(&msg_info, test_buf, 16);
    if (msg_info.head == CAT1_HEAD)
    {
      //比对IMEI号
      for (int i = 0; i < 8; i++)
      {
        if (msg_info.imei[i] != flash_data->imei[i])
        {
          return AT_LOCK;
        }
      }
      type = (msg_info.data[0] << 8) | msg_info.data[1];
      len = 0;
      for (int i = E_CAT1_ATTR_TYPE_lockStatus; i < E_CAT1_ATTR_TYPE_NUM; i++)
      {
        if ((type & s_cat1_attr_array[i].attr_type) != 0)
        {
          s_cat1_attr_array[i].isAlive = 1;
          if (!s_cat1_attr_array[i].isArray)
          {
            s_cat1_attr_array[i].attr_data = msg_info.data[2+len];
          }
          else
          {
            s_cat1_attr_array[i].attr_ptr = udp_buf[i];
            memcpy(s_cat1_attr_array[i].attr_ptr, &msg_info.data[2+len], s_cat1_attr_array[i].attr_length);
          }
					len += s_cat1_attr_array[i].attr_length;
        }
        else
        {
          s_cat1_attr_array[i].isAlive = 0;
        }
      }
      switch(msg_info.cmd)
      {
        //开锁指令
        case CAT1_UNLOCK_CMD:
          //开锁指令下发
          DBGPRINTF("CAT1_UNLOCK_CMD: \r\n");
          if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_lockStatus].isAlive)
          {
            if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_lockStatus].attr_data == 0x03)
            {
              ret = AT_UNLOCK;
            }
          }
          //关机控制
          if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_shutdownCtrl].isAlive)
          {
            if (flash_data->shoudown_flags != s_cat1_attr_array[E_CAT1_ATTR_TYPE_shutdownCtrl].attr_data)
            {
              flash_data->shoudown_flags = s_cat1_attr_array[E_CAT1_ATTR_TYPE_shutdownCtrl].attr_data;
              write_flash(flash_data);
              DBGPRINTF("shutdown flag = %d\r\n", flash_data->shoudown_flags);
            }
          } 
					//GPS功能是否使能
          if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_gps].isAlive)
          {
            if (flash_data->gps_flags != s_cat1_attr_array[E_CAT1_ATTR_TYPE_gps].attr_data)
            {
              flash_data->gps_flags = s_cat1_attr_array[E_CAT1_ATTR_TYPE_gps].attr_data;
              write_flash(flash_data);
              DBGPRINTF("gps flag = %d\r\n", flash_data->gps_flags);
            }
          }
          //更改IP与PORT
          break;
        //应答指令
        case CAT1_LOCK_STATUS_CMD:
        case CAT1_LOCK_ALARM_CMD:
        case CAT1_LOCK_POSITION_CMD:
				case CAT1_LOCK_UPDATE_CMD:
				case CAT1_TIGHTEND_CMD:
          ret = AT_SUCCESS;
          //关机控制
          if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_shutdownCtrl].isAlive)
          {
            if (flash_data->shoudown_flags != s_cat1_attr_array[E_CAT1_ATTR_TYPE_shutdownCtrl].attr_data)
            {
              flash_data->shoudown_flags = s_cat1_attr_array[E_CAT1_ATTR_TYPE_shutdownCtrl].attr_data;
              write_flash(flash_data);
              DBGPRINTF("shutdown flag = %d\r\n", flash_data->shoudown_flags);
            }
          } 
					//GPS功能是否使能
          if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_gps].isAlive)
          {
						if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_gps].attr_data == 0x00)
						{
							ret = AT_LOCK;
							DBGPRINTF("gps is disable\r\n");
						}
						else if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_gps].attr_data == 0x01)
						{
							ret = AT_SUCCESS;
							DBGPRINTF("gps is enable\r\n");
						}
						else {
							ret = AT_LOCK;
						}
          }
//					else {
//							ret = AT_LOCK;
//							DBGPRINTF("gps is disable......\r\n");
//					}			
          DBGPRINTF("CAT1_LOCK_SUCCESS: \r\n");
          break;
        //位置信息查询指令
        case CAT1_INQUIRE_POSITION_CMD:
          if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_positionReport].attr_data == 0x40)
          {
            ret = AT_SUCCESS;
          }
          break;
				case CAT1_LOCK_PERIOD_CMD:
          ret = AT_SUCCESS;
          //关机控制
          if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_shutdownCtrl].isAlive)
          {
            if (flash_data->shoudown_flags != s_cat1_attr_array[E_CAT1_ATTR_TYPE_shutdownCtrl].attr_data)
            {
              flash_data->shoudown_flags = s_cat1_attr_array[E_CAT1_ATTR_TYPE_shutdownCtrl].attr_data;
              write_flash(flash_data);
              DBGPRINTF("shutdown flag = %d\r\n", flash_data->shoudown_flags);
            }
          } 
					//GPS功能是否使能
          if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_gps].isAlive)
          {
            if (flash_data->gps_flags != s_cat1_attr_array[E_CAT1_ATTR_TYPE_gps].attr_data)
            {
              flash_data->gps_flags = s_cat1_attr_array[E_CAT1_ATTR_TYPE_gps].attr_data;
              write_flash(flash_data);
              DBGPRINTF("gps flag = %d\r\n", flash_data->gps_flags);
            }
          }		
					//远程升级 AB 0E 08 60 35 50 79 11 98 52 06 04 00 04 20 01 5B 6C A2 0D 0A
          if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].isAlive)
          {
						DBGPRINTF("updata old %02x %04x\r\n", flash_data->update_hw,flash_data->update_sw);
						DBGPRINTF("updata new %02x %02x %02x\r\n", s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[0],\
						s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[1], s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[2]);
						
            if ((flash_data->update_hw == s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[0]) &&
								(flash_data->update_sw != (((s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[1]) << 8) +
								s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[2])))
            {
							flash_data->update_hw = s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[0];
							flash_data->update_sw = (s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[1]<<8) \
							+s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[2];
							flash_data->update_check = s_cat1_attr_array[E_CAT1_ATTR_TYPE_update].attr_ptr[3];
							//升级后，开机上报升级状态标志位
							flash_data->update_flag = 1;
							ret = AT_UPDATE_START;
              DBGPRINTF("update paragram hw:%02x sw:%04x check:%02x\r\n",flash_data->update_hw, \
											flash_data->update_sw, flash_data->update_check);
            }
          }
					//更改FTP服务器的IP
					if (s_cat1_attr_array[E_CAT1_ATTR_TYPE_ip].isAlive)
					{
						ipaddrBuf_transition_char(s_cat1_attr_array[E_CAT1_ATTR_TYPE_ip].attr_ptr, FTP_IPADDR);
						DBGPRINTF("ftp ip = ");
						for (uint8_t i = 0; i < s_cat1_attr_array[E_CAT1_ATTR_TYPE_ip].attr_length; i++)
						{
							DBGPRINTF("%x ", s_cat1_attr_array[E_CAT1_ATTR_TYPE_ip].attr_ptr[i]);
						}
						DBGPRINTF("\r\n");
					}
          DBGPRINTF("CAT1_LOCK_PERIOD_CMD: \r\n");					
					break;
        default:
          break;
      }
    }
  } 
  else
  {
    ret = AT_LOCK;
  }
  return ret;
}

int string_to_hex(uint8_t *str, uint8_t * hex, int len)
{
	int i;
	for(i = 0; i < len; i++)
	{
		if(str[2 * i] >= '0' && str [2 * i] <= '9')
    {
			hex[i] = str[2 * i] - '0';
    }
		else if(str[2 * i] >= 'A' && str[2 * i] <= 'F')
    {
			hex[i] = str[2 * i] - 'A'+ 0x0a;
    }
		else if(str[2 * i] >= 'a' && str[2 * i] <= 'f')
    {
			hex[i] = str[2 * i] - 'a'+ 0x0a;
    }
		else
			return 1;

		if(str[2 * i + 1] >= '0' && str[2 * i + 1] <= '9')
    {
			hex[i] = (hex[i] << 4) | (str[2 * i + 1] - '0');
    }
		else if(str[2 * i + 1] >= 'A' && str[2 * i + 1] <= 'F')
    {
			hex[i] = (hex[i] << 4) | (str[2 * i + 1] - 'A'+ 0x0a);
    }
		else if(str[2 * i + 1] >= 'a' && str[ 2 * i + 1] <= 'f')
    {
			hex[i] = (hex[i] << 4) | (str[2 * i + 1] - 'a'+ 0x0a);
    }
		else
			return 1;
	}
	return 0;
}

uint16_t hex_to_string(uint8_t* hex, uint16_t len, uint8_t* str)
{
    int8_t data_high = 0;
    int8_t data_low = 0;
    uint16_t i = 0;
    for(i = 0; i < len; i++)
    {
      data_high = ((hex[i] >> 4) & 0x0F);
      data_low = hex[i] & 0x0F;

      if(data_high <= 9 && data_high >= 0)
      {
        data_high += 0x30;
      }
      else if((data_high >= 10) && (data_high <= 15))
      {
        data_high += 0x37;
      }

      if(data_low <= 9 && data_low >= 0)
      {
        data_low += 0x30;
      }
      else if((data_low >= 10) && (data_low <= 15))
      {
        data_low += 0x37;
      }
      str[2 * i] = data_high;
      str[2 * i + 1] = data_low; 
    }
    return i * 2;
}

static msg_ops_t msg_ops = {
	.unlock_req = udp_send_unlock_req,
	.lock_status = udp_send_lock_status,
	.lock_alarm = udp_send_lock_alarm,
	.period_report = udp_send_period_report,
	.lock_lbs = udp_send_lock_lbs,
	.recv_parse = udp_recv_msg_parse,
	.lock_update = udp_send_update,
	.lock_tightend = udp_send_tightend,
};

msg_ops_t *udp_msg_init(void)
{
	return &msg_ops;
}
