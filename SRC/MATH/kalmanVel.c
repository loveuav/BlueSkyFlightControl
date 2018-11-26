/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     kalmanVel.c
 * @说明     6阶速度卡尔曼滤波算法
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.11
**********************************************************************************************************/
#include "kalmanVel.h"
#include "matrix6.h"

//单位矩阵
static float I[6][6] = {{1,0,0,0,0,0},
    {0,1,0,0,0,0},
    {0,0,1,0,0,0},
    {0,0,0,1,0,0},
    {0,0,0,0,1,0},
    {0,0,0,0,0,1}
};

static void KalmanVelSlidWindowUpdate(KalmanVel_t* kalman);

/**********************************************************************************************************
*函 数 名: KalmanVelUpdate
*功能说明: 6阶速度卡尔曼滤波器更新
*形    参: 卡尔曼结构体指针 速度输出 bias输出 输入量（加速度） 观测矩阵 时间间隔 融合标志位
*返 回 值: 无
**********************************************************************************************************/
void KalmanVelUpdate(KalmanVel_t* kalman, Vector3f_t* velocity, Vector3f_t* bias, Vector3f_t accel,
                     float observe[6], float deltaT, bool fuseFlag)
{
    //用于存放计算结果的临时矩阵
    float S[6][6], m1[6][6], m2[6][6], m3[6][6], m4[6][6], m5[6][6];
    float v1[6], v2[6];
    static float input[6] = {0, 0, 0, 0, 0, 0};
    static float stateDelay[6] = {0, 0, 0, 0, 0, 0};

    //更新输入量
    input[0] = accel.x;
    input[1] = accel.y;
    input[2] = accel.z;

    //更新输入转移矩阵
    kalman->b[0][0] = kalman->b[1][1] = kalman->b[2][2] = deltaT * GRAVITY_ACCEL * 100;

    //更新状态转移矩阵
    kalman->f[0][3] = kalman->f[1][4] = kalman->f[2][5] = deltaT;

    //1:状态预估计 Xk = Fk*Xk-1 + Bk*Uk
    Matrix6MulVector6(kalman->f, kalman->state, v1);
    Matrix6MulVector6(kalman->b, input, v2);
    Vector6f_Add(v1, v2, kalman->state);

    //bias限幅
    kalman->state[3] = ConstrainFloat(kalman->state[3], -50, 50);
    kalman->state[4] = ConstrainFloat(kalman->state[4], -50, 50);
    kalman->state[5] = ConstrainFloat(kalman->state[5], -50, 50);

    //状态窗口更新
    KalmanVelSlidWindowUpdate(kalman);

    //当观测值未更新时不进行融合
    if(fuseFlag == true)
    {
        //2：误差协方差矩阵预更新 Pk = Fk*Pk-1*FkT + Qk
        Matrix6_Mul(kalman->f, kalman->covariance, m1);
        Matrix6_Mul(m1, kalman->f_t, m2);
        Matrix6_Add(m2, kalman->q, kalman->covariance);

        //取出窗口中的状态量
        stateDelay[0] = kalman->stateSlidWindow[kalman->slidWindowSize - kalman->fuseDelay[0]].x;
        stateDelay[1] = kalman->stateSlidWindow[kalman->slidWindowSize - kalman->fuseDelay[1]].y;
        stateDelay[2] = kalman->stateSlidWindow[kalman->slidWindowSize - kalman->fuseDelay[2]].z;
        stateDelay[3] = kalman->stateSlidWindow[kalman->slidWindowSize - kalman->fuseDelay[3]].z;
        stateDelay[4] = kalman->stateSlidWindow[kalman->slidWindowSize - kalman->fuseDelay[4]].z;

        //3：计算残差矩阵 Yk = Zk - Hk*Xk
        //Matrix6MulVector6(kalman->h, stateDelay, v1);
        //Vector6f_Sub(observe, v1, kalman->residual);
        Vector6f_Sub(observe, stateDelay, kalman->residual);

        //4：Sk = Hk*Pk*HkT + Rk
        Matrix6_Mul(kalman->h, kalman->covariance, m1);
        Matrix6_Mul(m1, kalman->h_t, m2);
        Matrix6_Add(m2, kalman->r, S);

        //5：计算卡尔曼增益 Kk = Pk*HkT*Sk-1
        if(!Matrix6_Det(S, m1)) return;
        Matrix6_Mul(kalman->covariance, kalman->h_t, m2);
        Matrix6_Mul(m2, m1,kalman->gain);

        //6：修正当前状态 Xk = Xk + Kk*Yk
        Matrix6MulVector6(kalman->gain, kalman->residual, v1);
        Vector6f_Add(kalman->state, v1, kalman->state);

        //7：更新协方差矩阵 Pk = (I-Kk*Hk)*Pk*(I-Kk*Hk)T + Kk*Rk*KkT
        Matrix6_Mul(kalman->gain, kalman->h, m1);
        Matrix6_Sub(I, m1, m2);
        Matrix6_Tran(m2, m3);
        Matrix6_Mul(m2, kalman->covariance, m4);
        Matrix6_Mul(m4, m3, m5);
        Matrix6_Mul(kalman->gain, kalman->r, m1);
        Matrix6_Tran(kalman->gain, m2);
        Matrix6_Mul(m1, m2, m3);
        Matrix6_Add(m5, m3, kalman->covariance);
    }

    //输出速度和bias
    velocity->x = kalman->state[0];
    velocity->y = kalman->state[1];
    velocity->z = kalman->state[2];
    bias->x     = kalman->state[3];
    bias->y     = kalman->state[4];
    bias->z     = kalman->state[5];
}

/**********************************************************************************************************
*函 数 名: KalmanVelStateTransMatSet
*功能说明: 状态转移矩阵设置
*形    参: 卡尔曼结构体指针 状态转移矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanVelStateTransMatSet(KalmanVel_t* kalman, float f[6][6])
{
    uint8_t i, j;

    //状态转移矩阵赋值
    for(i=0; i<6; i++)
    {
        for(j=0; j<6; j++)
            kalman->f[i][j] = f[i][j];
    }

    //计算状态转移矩阵的转置
    Matrix6_Tran(kalman->f, kalman->f_t);
}

/**********************************************************************************************************
*函 数 名: KalmanVelObserveMapMatSet
*功能说明: 观测映射矩阵设置
*形    参: 卡尔曼结构体指针 观测映射矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanVelObserveMapMatSet(KalmanVel_t* kalman, float h[6][6])
{
    uint8_t i, j;

    //观测映射矩阵赋值
    for(i=0; i<6; i++)
    {
        for(j=0; j<6; j++)
            kalman->h[i][j] = h[i][j];
    }

    //计算观测映射矩阵的转置
    Matrix6_Tran(kalman->h, kalman->h_t);
}

/**********************************************************************************************************
*函 数 名: KalmanVelCovarianceMatSet
*功能说明: 协方差矩阵设置
*形    参: 卡尔曼结构体指针 协方差矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanVelCovarianceMatSet(KalmanVel_t* kalman, float p[6][6])
{
    uint8_t i, j;

    //协方差矩阵赋值
    for(i=0; i<6; i++)
    {
        for(j=0; j<6; j++)
            kalman->covariance[i][j] = p[i][j];
    }
}

/**********************************************************************************************************
*函 数 名: KalmanVelQMatSet
*功能说明: 过程噪声协方差矩阵设置
*形    参: 卡尔曼结构体指针 协方差矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanVelQMatSet(KalmanVel_t* kalman, float q[6][6])
{
    uint8_t i, j;

    //协方差矩阵赋值
    for(i=0; i<6; i++)
    {
        for(j=0; j<6; j++)
            kalman->q[i][j] = q[i][j];
    }
}

/**********************************************************************************************************
*函 数 名: KalmanVelRMatSet
*功能说明: 测量噪声协方差矩阵设置
*形    参: 卡尔曼结构体指针 协方差矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanVelRMatSet(KalmanVel_t* kalman, float r[6][6])
{
    uint8_t i, j;

    //协方差矩阵赋值
    for(i=0; i<6; i++)
    {
        for(j=0; j<6; j++)
            kalman->r[i][j] = r[i][j];
    }
}

/**********************************************************************************************************
*函 数 名: KalmanVelBMatSet
*功能说明: 输入控制矩阵设置
*形    参: 卡尔曼结构体指针 输入控制矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanVelBMatSet(KalmanVel_t* kalman, float b[6][6])
{
    uint8_t i, j;

    for(i=0; i<6; i++)
    {
        for(j=0; j<6; j++)
            kalman->b[i][j] = b[i][j];
    }
}

/**********************************************************************************************************
*函 数 名: KalmanVelSlidWindowUpdate
*功能说明: 卡尔曼状态量滑动窗口更新
*形    参: 卡尔曼结构体指针
*返 回 值: 无
**********************************************************************************************************/
static void KalmanVelSlidWindowUpdate(KalmanVel_t* kalman)
{
    uint16_t i;

    for(i=0; i<kalman->slidWindowSize-1; i++)
    {
        kalman->stateSlidWindow[i] = kalman->stateSlidWindow[i+1];
    }

    kalman->stateSlidWindow[kalman->slidWindowSize - 1].x = kalman->state[0];
    kalman->stateSlidWindow[kalman->slidWindowSize - 1].y = kalman->state[1];
    kalman->stateSlidWindow[kalman->slidWindowSize - 1].z = kalman->state[2];
}

/**********************************************************************************************************
*函 数 名: KalmanVelUseMeasurement
*功能说明: 卡尔曼观测量使能开关
*形    参: 卡尔曼结构体指针 观测量序号 使能标志
*返 回 值: 无
**********************************************************************************************************/
void KalmanVelUseMeasurement(KalmanVel_t* kalman, uint8_t num, bool flag)
{
    switch(num)
    {
    case 0:
        kalman->h[0][0] = flag;
        break;

    case 1:
        kalman->h[1][1] = flag;
        break;

    case 2:
        kalman->h[2][2] = flag;
        break;

    case 3:
        kalman->h[3][2] = flag;
        break;

    case 4:
        kalman->h[4][2] = flag;
        break;

    default:
        break;
    }
    
    //计算观测映射矩阵的转置
    Matrix6_Tran(kalman->h, kalman->h_t);
}


