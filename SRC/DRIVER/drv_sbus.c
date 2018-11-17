/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     drv_subs.c
 * @说明     sbus接收机协议解析
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "drv_sbus.h"
#include "drv_usart.h"
#include "drv_ppm.h"

struct sbus_dat {
    uint32_t start : 8;
    uint32_t chan1 : 11;
    uint32_t chan2 : 11;
    uint32_t chan3 : 11;
    uint32_t chan4 : 11;
    uint32_t chan5 : 11;
    uint32_t chan6 : 11;
    uint32_t chan7 : 11;
    uint32_t chan8 : 11;
    uint32_t chan9 : 11;
    uint32_t chan10 : 11;
    uint32_t chan11 : 11;
    uint32_t chan12 : 11;
    uint32_t chan13 : 11;
    uint32_t chan14 : 11;
    uint32_t chan15 : 11;
    uint32_t chan16 : 11;
} __attribute__ ((__packed__));

union
{
    uint8_t  raw[25];
    struct sbus_dat msg;
} sbus;

RCDATA_t sbusData;

static void Sbus_Decode(uint8_t data);
static RcDataCallback rcDataCallbackFunc;

/**********************************************************************************************************
*函 数 名: Sbus_Init
*功能说明: sbus数据解析初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Sbus_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //设置SBUS串口接收中断回调函数（即数据协议解析函数）
    Usart_SetIRQCallback(SBUS_UART, Sbus_Decode);

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin   = SBUS_INV_PIN;
    GPIO_Init(SBUS_INV_GPIO, &GPIO_InitStructure);

    if(SBUS_INV == 1)
        GPIO_SetBits(SBUS_INV_GPIO, SBUS_INV_PIN);
    else
        GPIO_ResetBits(SBUS_INV_GPIO, SBUS_INV_PIN);
}

/**********************************************************************************************************
*函 数 名: Sbus_Decode
*功能说明: sbus协议解析
*形    参: 输入数据
*返 回 值: 无
**********************************************************************************************************/
static void Sbus_Decode(uint8_t data)
{
    static uint32_t lastTime;
    static uint32_t dataCnt  = 0;
    static uint8_t  initFlag = 0;

    if(GetSysTimeMs() < 2000)
        return;

    uint32_t deltaT = GetSysTimeMs() - lastTime;
    lastTime = GetSysTimeMs();

    //数据间隔大于3ms则可以认为新的一帧开始了
    if(deltaT > 3)
    {
        dataCnt = 0;
    }

    //接收数据
    sbus.raw[dataCnt++] = data;

    //每帧数据长度为25
    if(dataCnt == 25)
    {
        //判断帧头帧尾是否正确
        if(sbus.raw[0] != 0x0F || sbus.raw[24] != 0)
            return;

        //每个通道数据占据11个字节，这里使用了字节对齐的方式来进行解析
        //转换摇杆数据量程为[1000:2000]
        sbusData.roll     = sbus.msg.chan1 * 0.625f + 880;
        sbusData.pitch    = sbus.msg.chan2 * 0.625f + 880;
        sbusData.throttle = sbus.msg.chan3 * 0.625f + 880;
        sbusData.yaw      = sbus.msg.chan4 * 0.625f + 880;
        sbusData.aux1     = sbus.msg.chan5 * 0.625f + 880;
        sbusData.aux2     = sbus.msg.chan6 * 0.625f + 880;
        sbusData.aux3     = sbus.msg.chan7 * 0.625f + 880;
        sbusData.aux4     = sbus.msg.chan8 * 0.625f + 880;
        sbusData.aux5     = sbus.msg.chan9 * 0.625f + 880;
        sbusData.aux6     = sbus.msg.chan10 * 0.625f + 880;
        sbusData.aux7     = sbus.msg.chan11 * 0.625f + 880;
        sbusData.aux8     = sbus.msg.chan12 * 0.625f + 880;
        sbusData.aux8     = sbus.msg.chan13 * 0.625f + 880;
        sbusData.aux10    = sbus.msg.chan14 * 0.625f + 880;
        sbusData.aux11    = sbus.msg.chan15 * 0.625f + 880;
        sbusData.aux12    = sbus.msg.chan16 * 0.625f + 880;

        //一帧数据解析完成
        if(rcDataCallbackFunc != 0)
            (*rcDataCallbackFunc)(sbusData);

        if(!initFlag)
        {
            //禁用PPM输入
            PPM_Disable();
            initFlag = 1;
        }
    }
}


/**********************************************************************************************************
*函 数 名: Sbus_SetRcDataCallback
*功能说明: 设置遥控数据处理回调函数
*形    参: 回调函数
*返 回 值: 无
**********************************************************************************************************/
void Sbus_SetRcDataCallback(RcDataCallback rcDataCallback)
{
    rcDataCallbackFunc = rcDataCallback;
}

/**********************************************************************************************************
*函 数 名: Sbus_Disable
*功能说明: SBUS输入关闭
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Sbus_Disable(void)
{
    USART_TypeDef * usart[] = {USART1, USART2, USART3, UART4, UART5, USART6};
    
    if(SBUS_UART <= 6)
    {
        USART_ClearITPendingBit(usart[SBUS_UART - 1], USART_IT_RXNE);
        USART_ITConfig(usart[SBUS_UART - 1], USART_IT_RXNE, DISABLE);
        USART_Cmd(usart[SBUS_UART - 1], DISABLE);
    }
}
