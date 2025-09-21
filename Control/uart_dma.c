#include "uart.dma.h"
extern DMA_HandleTypeDef hdma_usart3_rx;

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
RC_ctrl_t rc_ctrl;
/**
 * @brief   将 SBUS 数据帧解析为遥控器结构体
 * @param   sbus_buf  指向 DMA 收到的 18 字节 SBUS 数据帧（只读，禁止改写）
 * @param   rc_ctrl   解析结果保存的目标结构体
 * @note    帧格式：Futaba SBUS（反向电平，波特率 100 k，8E2）
 *          本函数假定上层已做字节取反（0x00?0xFF）和字节对齐
 *          数据以小端方式拼合，11 bit/通道，共 16 通道 + 2 bit 状态标志
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{

/*  位表摘要（LSB 0 起步）
 *  ch0   0-10
 *  ch1  11-21
 *  ch2  22-32
 *  ch3  33-43
 *  S1   44-45
 *  S2   46-47
 *  mouseX  48-63
 *  mouseY  64-79
 *  mouseZ  80-95
 *  mouseL  96-103
 *  mouseR 104-111
 *  key    112-127
 */

    if (sbus_buf == NULL || rc_ctrl == NULL) return;

    /* 遥控通道 11 bit × 4 */
    uint32_t raw;
    /* ch0  0-10 */
    raw  = sbus_buf[0] | (sbus_buf[1] << 8);
    rc_ctrl->rc.ch[0] = raw & 0x07FF;

    /* ch1 11-21 */
    raw  = (sbus_buf[1] >> 3) | (sbus_buf[2] << 5);
    rc_ctrl->rc.ch[1] = raw & 0x07FF;

    /* ch2 22-32 */
    raw  = (sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10);
    rc_ctrl->rc.ch[2] = raw & 0x07FF;

    /* ch3 33-43 */
    raw  = (sbus_buf[4] >> 1) | (sbus_buf[5] << 7);
    rc_ctrl->rc.ch[3] = raw & 0x07FF;

    /* 开关 2 bit × 2  → 位 44-47 */
    raw = (sbus_buf[5] >> 5) | (sbus_buf[6] << 3);   
    rc_ctrl->rc.s[0] = (raw >> 0) & 0x03;   // S1
    rc_ctrl->rc.s[1] = (raw >> 2) & 0x03;   // S2

    /* 鼠标 16/16/16/8/8 bit */
    rc_ctrl->mouse.x      = (int16_t)(sbus_buf[6] >> 2 | (sbus_buf[7]  << 6) | (sbus_buf[8]  << 14));
    rc_ctrl->mouse.y      = (int16_t)((sbus_buf[8] >> 2) | (sbus_buf[9]  << 6) | (sbus_buf[10] << 14));
    rc_ctrl->mouse.z      = (int16_t)((sbus_buf[10]>> 2) | (sbus_buf[11] << 6) | (sbus_buf[12] << 14));
    rc_ctrl->mouse.press_l= (sbus_buf[12] >> 2) & 0x01;
    rc_ctrl->mouse.press_r= (sbus_buf[13] >> 2) & 0x01;

    /* 键盘 16 bit */
    rc_ctrl->key.v        = (sbus_buf[14] >> 2) | (sbus_buf[15] << 6) | (sbus_buf[16] << 14);

    /* 零点偏移 */
    enum { RC_CH_VALUE_OFFSET = 1024};
    for (int i = 0; i < 4; ++i) rc_ctrl->rc.ch[i] -= RC_CH_VALUE_OFFSET;
}

/*
    *@功能串口3接收初始化
    *@1.实现的功能：
        *@1.打开 USART3 的 DMA 接收闸门（DMAR）
        *@2.打开空闲中断（IDLE）
        *@3.停 DMA → 配寄存器（PAR、M0AR/M1AR、NDTR、DBM）→ 再开 DMA
    *@2.各类寄存器值：    
        *@PAR寄存器指向DMA搬运的数据源（永远指向PAR）
        *@MOAR,M1AR指的是两个接收地址，会让DMA的搬运在MOAR,M1AR之间来回切换
        *@NDTR指的是转换的次数
        *@DBM就是置1完成DMA在两个接收地址之间来回切换
    *@3.rx_buf_1,rx_buf_2为接收区的地址，dam_buf_number为转运的数量
*/
void RC_Init(uint8_t * rx_buf_1,uint8_t * rx_buf_2,uint16_t dam_buf_number){
    SET_BIT(huart3.Instance->CR3,USART_CR3_DMAR);                   //打开串口DMA接收
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);                     //打开串口3的空闲中断    

    //关闭DMA,接下来要配置DMA相关参数
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR& DMA_SxCR_EN){
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    //寄存器的位为32
    hdma_usart3_rx.Instance->PAR=(uint32_t)&(USART3->DR);           //设定搬运寄存器的地址  ，&将寄存器的地址取出来
    hdma_usart3_rx.Instance->M0AR=(uint32_t)rx_buf_1;               //设定转运区  
    hdma_usart3_rx.Instance->M1AR=(uint32_t)rx_buf_2;             
    hdma_usart3_rx.Instance->NDTR=(uint32_t)dam_buf_number;         //配置转运数量

    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);             //启动双缓存模式

    __HAL_DMA_ENABLE(&hdma_usart3_rx);                               //使能DMA  
}



/**
 * @brief  USART3 全局中断服务函数
 *         采用**空闲中断 + DMA 双缓冲**方式接收 SBUS 数据帧
 * @note   1. 接收完成后触发 UART_FLAG_IDLE
 *         2. 根据 DMA_SxCR_CT 判断当前使用的是 M0AR 还是 M1AR
 *         3. 计算实际接收长度 → 处理数据 → 切换缓冲区 → 重启 DMA
 * 之后不需要太多判断了，在sbus_to_rc中实现了对数据的处理
 */
void USART3_IRQHandler(void)
{
    //此时接收到一个字节，不进行处理
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);   // 清除所有错误标志及RXNE
    }

    //之前初始化打开了空闲中断，如果不是收到字节就是表示一帧数据发送完成
    else if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))          //确定IDLE空闲为置1
    {
        static uint16_t this_time_rx_len = 0;                       // 本次接收长度

        __HAL_UART_CLEAR_PEFLAG(&huart3);                           //清处IDLE标志
        //接下来判断缓存区，注意DMA_SxCR_CT是常量，当CR对应为0（RESET）时，使用的是MOAR
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            //关闭 DMA，准备处理数据 
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //计算实际数据,缓冲区大小-实际剩余数量
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //重新装载计数器
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

    
            //重启DMA，开始往 Memory 1 写数据 
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            //如果符合，则进行处理
            if (this_time_rx_len == SBUS_RX_BUF_NUM)
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);   // 处理 Memory 0 的数据
        }

        /* 2.2 当前目标缓冲区是 Memory 1（CT = 1） */
        else        
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            /* 切换缓冲区：把 CT 清 0，下次 DMA 使用 Memory 0 */
            hdma_usart3_rx.Instance->CR &= ~DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);   // 处理 Memory 1 的数据
        }
    }
}

void usart1_tx_dma_init(void)
{
    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
}



extern DMA_HandleTypeDef hdma_usart1_tx;

//使能串口一的发送功能
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    //clear flag
    //清除标志位
   __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_tx));
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_usart1_tx));
    //set data address
    //设置数据地址
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    //set data length
    //设置数据长度
    hdma_usart1_tx.Instance->NDTR = len;

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



/**
 * 实现类似printf的效果
 */
void usart_printf(const char *str,...){
    static uint8_t max_tx_buf[128];
    va_list ap;
    uint16_t length;
    va_start(ap,str);                           //ap不会指向字符串，而是指向str后面的可变变量，要理解函数是一个栈区
    //str中有填充位，ap会补全这些填充位，然后将填充好的字符串给max_tx_buf，然后vsprintf的返回值位max_tx_buf字符串的长度
    length=vsprintf((char*)max_tx_buf,str,ap);        
    va_end(ap);
    //从max_tx__buf中搬运数据进行发送
		HAL_UART_Transmit_DMA(&huart1,max_tx_buf,length);

    //usart1_tx_dma_enable(max_tx_buf, length);
}


void print_rc_ctrl(const RC_ctrl_t *rc){
    usart_printf(
        "ch0:%d\r\n"
        "ch1:%d\r\n"
        "ch2:%d\r\n"
        "ch3:%d\r\n"
        "s1:%d\r\n"
        "s2:%d\r\n"
        "mouse_x:%d\r\n"
        "mouse_y:%d\r\n"
        "mouse_z:%d\r\n"
        "press_l:%d\r\n"
        "press_r:%d\r\n"
        "key:%d\r\n",
        rc->rc.ch[0],
        rc->rc.ch[1],
        rc->rc.ch[2],
        rc->rc.ch[3],
        rc->rc.s[0],
        rc->rc.s[1],
        rc->mouse.x,
        rc->mouse.y,
        rc->mouse.z,
        rc->mouse.press_l,
        rc->mouse.press_r,
        rc->key.v);
}

