#ifndef __UART_DMA_H__
#define __UART_DMA_H__

/*
 *������߼���
 *  1.ʵ��USART3_DMA�ĳ�ʼ��������ң�������ݵĴ��ڣ�,ʹ��RC_Init
 *  2.���������ж�����������(���飬���մ����ź�))�����뵽RC_Init��
 *  3.�����Զ������жϴ�����ʱ�ڶ���һ�����ݴ����������ݵĲ�������ǰ�涨�������������
 * �Լ�һ���ṹ��������մ���������
 *  4.ʵ��printf���������ṹ���ڵı������ݵ�printf�У�printf���dmaʵ�֣�,���͸����Զ�
*/


#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"
#include "stm32f4xx_hal_dma.h"

#define SBUS_RX_BUF_NUM 36          //��װֵ
#define RC_FRAME_LENGTH 18          //һ֡�����ݴ�С

/*----------------------- ���̽ṹ�� -----------------------*/
typedef struct
{
    uint16_t v;         //��������
} key_t;
typedef struct
{
    int16_t x;          //���x��
    int16_t y;          //���y��
    int16_t z;          //���z��
    uint8_t press_l;    //������
    uint8_t press_r;    //����Ҽ�
} mouse_t;

/*----------------------- ң�����ṹ�� -----------------------*/
typedef struct
{
    uint16_t ch[4];     //0~3,4��ͨ��
    uint8_t s[2];       //�������أ�ֵ�ķ�Χ0~3
} rc_info_t;

/*----------------------- �ܿ��ƽṹ�� -----------------------*/
typedef struct
{
    rc_info_t rc;       //ң��������
    mouse_t   mouse;    //��겿��
    key_t     key;      //���̲���
} RC_ctrl_t;




void RC_Init(uint8_t * rx_buf_1,uint8_t * rx_buf_2,uint16_t dam_buf_number);
void usart1_tx_dma_init(void);
void print_rc_ctrl(const RC_ctrl_t *rc);
void usart_printf(const char *str,...);
void print_rc_ctrl(const RC_ctrl_t * rc);
void usart1_tx_dma_enable(uint8_t *data, uint16_t len);

#endif