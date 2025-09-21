#include "uart.dma.h"
extern DMA_HandleTypeDef hdma_usart3_rx;

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
RC_ctrl_t rc_ctrl;
/**
 * @brief   �� SBUS ����֡����Ϊң�����ṹ��
 * @param   sbus_buf  ָ�� DMA �յ��� 18 �ֽ� SBUS ����֡��ֻ������ֹ��д��
 * @param   rc_ctrl   ������������Ŀ��ṹ��
 * @note    ֡��ʽ��Futaba SBUS�������ƽ�������� 100 k��8E2��
 *          �������ٶ��ϲ������ֽ�ȡ����0x00?0xFF�����ֽڶ���
 *          ������С�˷�ʽƴ�ϣ�11 bit/ͨ������ 16 ͨ�� + 2 bit ״̬��־
 */
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{

/*  λ��ժҪ��LSB 0 �𲽣�
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

    /* ң��ͨ�� 11 bit �� 4 */
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

    /* ���� 2 bit �� 2  �� λ 44-47 */
    raw = (sbus_buf[5] >> 5) | (sbus_buf[6] << 3);   
    rc_ctrl->rc.s[0] = (raw >> 0) & 0x03;   // S1
    rc_ctrl->rc.s[1] = (raw >> 2) & 0x03;   // S2

    /* ��� 16/16/16/8/8 bit */
    rc_ctrl->mouse.x      = (int16_t)(sbus_buf[6] >> 2 | (sbus_buf[7]  << 6) | (sbus_buf[8]  << 14));
    rc_ctrl->mouse.y      = (int16_t)((sbus_buf[8] >> 2) | (sbus_buf[9]  << 6) | (sbus_buf[10] << 14));
    rc_ctrl->mouse.z      = (int16_t)((sbus_buf[10]>> 2) | (sbus_buf[11] << 6) | (sbus_buf[12] << 14));
    rc_ctrl->mouse.press_l= (sbus_buf[12] >> 2) & 0x01;
    rc_ctrl->mouse.press_r= (sbus_buf[13] >> 2) & 0x01;

    /* ���� 16 bit */
    rc_ctrl->key.v        = (sbus_buf[14] >> 2) | (sbus_buf[15] << 6) | (sbus_buf[16] << 14);

    /* ���ƫ�� */
    enum { RC_CH_VALUE_OFFSET = 1024};
    for (int i = 0; i < 4; ++i) rc_ctrl->rc.ch[i] -= RC_CH_VALUE_OFFSET;
}

/*
    *@���ܴ���3���ճ�ʼ��
    *@1.ʵ�ֵĹ��ܣ�
        *@1.�� USART3 �� DMA ����բ�ţ�DMAR��
        *@2.�򿪿����жϣ�IDLE��
        *@3.ͣ DMA �� ��Ĵ�����PAR��M0AR/M1AR��NDTR��DBM���� �ٿ� DMA
    *@2.����Ĵ���ֵ��    
        *@PAR�Ĵ���ָ��DMA���˵�����Դ����Զָ��PAR��
        *@MOAR,M1ARָ�����������յ�ַ������DMA�İ�����MOAR,M1AR֮�������л�
        *@NDTRָ����ת���Ĵ���
        *@DBM������1���DMA���������յ�ַ֮�������л�
    *@3.rx_buf_1,rx_buf_2Ϊ�������ĵ�ַ��dam_buf_numberΪת�˵�����
*/
void RC_Init(uint8_t * rx_buf_1,uint8_t * rx_buf_2,uint16_t dam_buf_number){
    SET_BIT(huart3.Instance->CR3,USART_CR3_DMAR);                   //�򿪴���DMA����
    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);                     //�򿪴���3�Ŀ����ж�    

    //�ر�DMA,������Ҫ����DMA��ز���
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR& DMA_SxCR_EN){
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    //�Ĵ�����λΪ32
    hdma_usart3_rx.Instance->PAR=(uint32_t)&(USART3->DR);           //�趨���˼Ĵ����ĵ�ַ  ��&���Ĵ����ĵ�ַȡ����
    hdma_usart3_rx.Instance->M0AR=(uint32_t)rx_buf_1;               //�趨ת����  
    hdma_usart3_rx.Instance->M1AR=(uint32_t)rx_buf_2;             
    hdma_usart3_rx.Instance->NDTR=(uint32_t)dam_buf_number;         //����ת������

    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);             //����˫����ģʽ

    __HAL_DMA_ENABLE(&hdma_usart3_rx);                               //ʹ��DMA  
}



/**
 * @brief  USART3 ȫ���жϷ�����
 *         ����**�����ж� + DMA ˫����**��ʽ���� SBUS ����֡
 * @note   1. ������ɺ󴥷� UART_FLAG_IDLE
 *         2. ���� DMA_SxCR_CT �жϵ�ǰʹ�õ��� M0AR ���� M1AR
 *         3. ����ʵ�ʽ��ճ��� �� �������� �� �л������� �� ���� DMA
 * ֮����Ҫ̫���ж��ˣ���sbus_to_rc��ʵ���˶����ݵĴ���
 */
void USART3_IRQHandler(void)
{
    //��ʱ���յ�һ���ֽڣ������д���
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);   // ������д����־��RXNE
    }

    //֮ǰ��ʼ�����˿����жϣ���������յ��ֽھ��Ǳ�ʾһ֡���ݷ������
    else if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))          //ȷ��IDLE����Ϊ��1
    {
        static uint16_t this_time_rx_len = 0;                       // ���ν��ճ���

        __HAL_UART_CLEAR_PEFLAG(&huart3);                           //�崦IDLE��־
        //�������жϻ�������ע��DMA_SxCR_CT�ǳ�������CR��ӦΪ0��RESET��ʱ��ʹ�õ���MOAR
        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            //�ر� DMA��׼���������� 
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //����ʵ������,��������С-ʵ��ʣ������
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //����װ�ؼ�����
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

    
            //����DMA����ʼ�� Memory 1 д���� 
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            //������ϣ�����д���
            if (this_time_rx_len == SBUS_RX_BUF_NUM)
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);   // ���� Memory 0 ������
        }

        /* 2.2 ��ǰĿ�껺������ Memory 1��CT = 1�� */
        else        
        {
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            /* �л����������� CT �� 0���´� DMA ʹ�� Memory 0 */
            hdma_usart3_rx.Instance->CR &= ~DMA_SxCR_CT;

            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if (this_time_rx_len == RC_FRAME_LENGTH)
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);   // ���� Memory 1 ������
        }
    }
}

void usart1_tx_dma_init(void)
{
    //enable the DMA transfer for the receiver request
    //ʹ��DMA���ڽ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);
}



extern DMA_HandleTypeDef hdma_usart1_tx;

//ʹ�ܴ���һ�ķ��͹���
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);
    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    //clear flag
    //�����־λ
   __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart1_tx));
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_usart1_tx));
    //set data address
    //�������ݵ�ַ
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    //set data length
    //�������ݳ���
    hdma_usart1_tx.Instance->NDTR = len;

    //enable DMA
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



/**
 * ʵ������printf��Ч��
 */
void usart_printf(const char *str,...){
    static uint8_t max_tx_buf[128];
    va_list ap;
    uint16_t length;
    va_start(ap,str);                           //ap����ָ���ַ���������ָ��str����Ŀɱ������Ҫ��⺯����һ��ջ��
    //str�������λ��ap�Ჹȫ��Щ���λ��Ȼ�����õ��ַ�����max_tx_buf��Ȼ��vsprintf�ķ���ֵλmax_tx_buf�ַ����ĳ���
    length=vsprintf((char*)max_tx_buf,str,ap);        
    va_end(ap);
    //��max_tx__buf�а������ݽ��з���
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

