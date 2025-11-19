#ifndef _UART_H_
#define _UART_H_

#include "ARMCM0.h" 

typedef struct snb_t {
	__IO uint8_t uart_rx_buf[180];
	__IO uint8_t uart_rx_data;
	__IO uint8_t uart_rx_flag;
    uint8_t uart_rx_cnt;
}__attribute__((packed)) nb_t;

enum UART_BAUD_SEL {
	UART_BAUD_1200		= 0x00,
	UART_BAUD_2400		= 0x01,
	UART_BAUD_4800		= 0x02,
	UART_BAUD_9600		= 0x03,
	UART_BAUD_14400		= 0x04,
	UART_BAUD_19200		= 0x05,
	UART_BAUD_38400		= 0x06,
	UART_BAUD_57600		= 0x07,
	UART_BAUD_115200	= 0x08,
	UART_BAUD_230400	= 0x09,
	UART_BAUD_460800	= 0x0A,
	UART_BAUD_921600	= 0x0B,
};

enum UART_FLOWCTRL {
	UART_RTS_CTS_ENABLE		= 0x01,
	UART_RTS_CTS_DISABLE	= 0x00,
};

#define OTG_TX_BUF_SIZE       32
#define OTG_RX_BUF_SIZE    		32

#define OTG_RECV_RANDOM_NUM_CMD  				0x10
#define OTG_SEND_RANDOM_NUM_CMD  				0x11
#define OTG_RECV_UNLOCK_CMD			 				0x12
#define OTG_SEND_LOCK_STATUS_CMD				0x13

typedef struct OtgReceBuf
{
  uint8_t buf[OTG_RX_BUF_SIZE];
  uint8_t len;       //有效数据长度              
}otg_recv_t;

//发送数据BUF
typedef struct OtgSendBuf
{
  uint8_t buf[OTG_TX_BUF_SIZE];
  uint8_t len;
}otg_send_t;

extern void uart0_init(void);
extern void uart0_write_one(uint8_t data);
extern void uart0_read(uint8_t *pcnt, uint8_t *pbuf);
extern void uart0_close(void);
extern void uart0_write(uint8_t *buf, uint16_t len);

extern void uart1_init(void);
extern void uart1_write_one(uint8_t  data);
extern void uart1_read(uint8_t *pcnt, uint8_t *pbuf);
extern void uart1_close(void);
extern void uart1_write(uint8_t *buf, uint16_t len);

void UartHandle_Task(void *pvParameters);
void receive_cmd_timeout(void);
void uart0_recv_data(void);
void uart1_recv_data(void);
void send_at_cmd(void);
void UART0_RxCpltCallback(void);
void UART1_RxCpltCallback(void);

#endif
