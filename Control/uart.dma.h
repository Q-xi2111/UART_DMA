#ifndef __UART_DMA_H__
#define __UART_DMA_H__

/*
 *整体的逻辑：
 *  1.实现USART3_DMA的初始化（接收遥控器数据的串口）,使用RC_Init
 *  2.在主函数中定义两个变量(数组，接收串口信号))，放入到RC_Init中
 *  3.后续自动进行中断处理，此时在定义一个数据处理函数，传递的参数就是前面定义的两个变量，
 * 以及一个结构体变量接收处理后的数据
 *  4.实现printf函数，将结构体内的变量传递到printf中（printf配合dma实现）,发送给电脑端
*/


#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"
#include "stm32f4xx_hal_dma.h"

#define SBUS_RX_BUF_NUM 36          //重装值
#define RC_FRAME_LENGTH 18          //一帧的数据大小

/*----------------------- 键盘结构体 -----------------------*/
typedef struct
{
    uint16_t v;         //按键部分
} key_t;
typedef struct
{
    int16_t x;          //鼠标x轴
    int16_t y;          //鼠标y轴
    int16_t z;          //鼠标z轴
    uint8_t press_l;    //鼠标左键
    uint8_t press_r;    //鼠标右键
} mouse_t;

/*----------------------- 遥控器结构体 -----------------------*/
typedef struct
{
    uint16_t ch[4];     //0~3,4个通道
    uint8_t s[2];       //两个开关，值的范围0~3
} rc_info_t;

/*----------------------- 总控制结构体 -----------------------*/
typedef struct
{
    rc_info_t rc;       //遥控器部分
    mouse_t   mouse;    //鼠标部分
    key_t     key;      //键盘部分
} RC_ctrl_t;




void RC_Init(uint8_t * rx_buf_1,uint8_t * rx_buf_2,uint16_t dam_buf_number);
void usart1_tx_dma_init(void);
void print_rc_ctrl(const RC_ctrl_t *rc);
void usart_printf(const char *str,...);
void print_rc_ctrl(const RC_ctrl_t * rc);
void usart1_tx_dma_enable(uint8_t *data, uint16_t len);

#endif