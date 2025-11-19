#ifndef __UDP_H__
#define __UDP_H__

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "lock.h"

#define CAT1_HEAD                       0xab
#define CAT1_UNLOCK_REQUEST_CMD         0x0a	//开锁上报
#define CAT1_UNLOCK_CMD                 0x0b	//下发开锁
#define CAT1_LOCK_STATUS_CMD            0x0c	//状态上报
#define CAT1_LOCK_ALARM_CMD             0x0d	//告警上报
#define CAT1_LOCK_PERIOD_CMD            0x0e	//周期上报
#define CAT1_LOCK_POSITION_CMD          0x0f	//位置上报
#define CAT1_INQUIRE_POSITION_CMD       0x10	//下发位置查询
#define CAT1_LOCK_UPDATE_CMD						0x11  //4G升级后，版本号上报
#define CAT1_TIGHTEND_CMD								0x12  //拉紧上报

typedef struct {
  char longitude_degree[3];
  char longitude_minute[6];
  uint8_t longitude_len;
  char latitude_degree[3];
  char latitude_minute[6];
  uint8_t latitude_len;
}location_info_t;

typedef enum
{
  E_CAT1_ATTR_TYPE_lockStatus,
  E_CAT1_ATTR_TYPE_batteryLevel,
  E_CAT1_ATTR_TYPE_rssi,
  E_CAT1_ATTR_TYPE_softwareVersion,
  E_CAT1_ATTR_TYPE_hardwareVersion,
  E_CAT1_ATTR_TYPE_longitude,
  E_CAT1_ATTR_TYPE_latitude,
  E_CAT1_ATTR_TYPE_alarm,
  E_CAT1_ATTR_TYPE_positionReport,
  E_CAT1_ATTR_TYPE_periodReport,
  E_CAT1_ATTR_TYPE_update,
  E_CAT1_ATTR_TYPE_imsi,
  E_CAT1_ATTR_TYPE_hall,
  E_CAT1_ATTR_TYPE_ip,
  E_CAT1_ATTR_TYPE_shutdownCtrl,
	E_CAT1_ATTR_TYPE_gps,
  E_CAT1_ATTR_TYPE_NUM   
}E_CAT1_ATTR_TYPE;

typedef struct
{
  uint8_t   type;
  uint8_t   isArray;
  uint8_t   isAlive;
  uint16_t  attr_type;
  uint8_t   attr_length;
  uint8_t   attr_data;
  uint8_t   *attr_ptr;
}cat1_attr_t;

typedef struct
{
  uint8_t head;
  uint8_t cmd;
  uint8_t imei[8];
  uint8_t len;
  uint8_t data[20];
}msg_info_t;

typedef struct {
	void (*unlock_req)(lock_info_t *info, uint8_t *send_buf);
	void (*lock_status)(lock_info_t *info, uint8_t *send_buf);
	void (*lock_alarm)(lock_info_t *info, uint8_t *send_buf);
	void (*period_report)(lock_info_t *info, uint8_t *send_buf);
	void (*lock_update)(lock_info_t *info, uint8_t *send_buf);
	uint8_t (*lock_lbs)(lock_info_t *info, uint8_t *send_buf, location_info_t *lbs_pinfo);
	void (*lock_tightend)(lock_info_t *info, uint8_t *send_buf);
	uint8_t (*recv_parse)(char *recv_buf, flash_data_t *flash_data);
}msg_ops_t;

msg_ops_t *udp_msg_init(void);

#endif 
