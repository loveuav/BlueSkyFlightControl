/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     kalman3.c
 * @说明     3阶卡尔曼滤波算法
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05
**********************************************************************************************************/
#include "kalman3.h"
#include "matrix3.h"

//单位矩阵
static float I[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

static void KalmanSlidWindowUpdate(Kalman_t* kalman);

/**********************************************************************************************************
*函 数 名: KalmanUpdate
*功能说明: 三阶卡尔曼算法更新
*形    参: 卡尔曼结构体指针 输入矩阵 观测矩阵 更新标志位(为真时才融合，否则只进行状态预估)
*返 回 值: 无
**********************************************************************************************************/
void KalmanUpdate(Kalman_t* kalman, Vector3f_t input, Vector3f_t observe, bool flag)
{
    //用于存放计算结果的临时矩阵
    float S[9], m1[9], m2[9], m3[9], m4[9], m5[9];

    //1:状态预估计 Xk = Fk*Xk-1 + Bk*Uk
    kalman->state = Vector3f_Add(Matrix3MulVector3(kalman->f, kalman->state), Matrix3MulVector3(kalman->b, input));

    //状态窗口更新
    KalmanSlidWindowUpdate(kalman);

    //当观测值未更新时不进行融合，退出本函数
    if(flag == false)
    {
        return;
    }

    //2：误差协方差矩阵预更新 Pk = Fk*Pk-1*FkT + Qk
    Matrix3_Mul(kalman->f, kalman->covariance, m1);
    Matrix3_Mul(m1, kalman->f_t, m2);
    Matrix3_Add(m2, kalman->q, kalman->covariance);

    //取出窗口中的状态量
    Vector3f_t statusDelay;
    statusDelay.x = kalman->statusSlidWindow[kalman->slidWindowSize - kalman->fuseDelay.x].x;
    statusDelay.y = kalman->statusSlidWindow[kalman->slidWindowSize - kalman->fuseDelay.y].y;
    statusDelay.z = kalman->statusSlidWindow[kalman->slidWindowSize - kalman->fuseDelay.z].z;

    //3：计算残差矩阵 Yk = Zk - Hk*Xk
    kalman->residual = Vector3f_Sub(observe, Matrix3MulVector3(kalman->h, statusDelay));

    //4：Sk = Hk*Pk*HkT + Rk
    Matrix3_Mul(kalman->h, kalman->covariance, m1);
    Matrix3_Mul(m1, kalman->h_t, m2);
    Matrix3_Add(m2, kalman->r, S);

    //5：计算卡尔曼增益 Kk = Pk*HkT*Sk-1
    Matrix3_Det(S, m1);
    Matrix3_Mul(kalman->covariance, kalman->h_t, m2);
    Matrix3_Mul(m2, m1,kalman->gain);

    //6：修正当前状态 Xk = Xk + Kk*Yk
    kalman->state = Vector3f_Add(kalman->state, Matrix3MulVector3(kalman->gain, kalman->residual));

    //7：更新协方差矩阵 Pk = (I-Kk*Hk)*Pk*(I-Kk*Hk)T + Kk*Rk*KkT
    Matrix3_Mul(kalman->gain, kalman->h, m1);
    Matrix3_Sub(I, m1, m2);
    Matrix3_Tran(m2, m3);
    Matrix3_Mul(m2, kalman->covariance, m4);
    Matrix3_Mul(m4, m3, m5);
    Matrix3_Mul(kalman->gain, kalman->r, m1);
    Matrix3_Tran(kalman->gain, m2);
    Matrix3_Mul(m1, m2, m3);
    Matrix3_Add(m5, m3, kalman->covariance);
}

/**********************************************************************************************************
*函 数 名: KalmanStateTransMatSet
*功能说明: 状态转移矩阵设置
*形    参: 卡尔曼结构体指针 状态转移矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanStateTransMatSet(Kalman_t* kalman, float* f)
{
    uint8_t i;

    //状态转移矩阵赋值
    for(i=0; i<9; i++)
    {
        kalman->f[i] = f[i];
    }
    //计算状态转移矩阵的转置
    Matrix3_Tran(kalman->f, kalman->f_t);
}

/**********************************************************************************************************
*函 数 名: KalmanObserveMapMatSet
*功能说明: 观测映射矩阵设置
*形    参: 卡尔曼结构体指针 观测映射矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanObserveMapMatSet(Kalman_t* kalman, float* h)
{
    uint8_t i;

    //观测映射矩阵赋值
    for(i=0; i<9; i++)
    {
        kalman->h[i] = h[i];
    }
    //计算观测映射矩阵的转置
    Matrix3_Tran(kalman->h, kalman->h_t);
}

/**********************************************************************************************************
*函 数 名: KalmanCovarianceMatSet
*功能说明: 协方差矩阵设置
*形    参: 卡尔曼结构体指针 协方差矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanCovarianceMatSet(Kalman_t* kalman, float* p)
{
    uint8_t i;

    //协方差矩阵赋值
    for(i=0; i<9; i++)
    {
        kalman->covariance[i] = p[i];
    }
}

/**********************************************************************************************************
*函 数 名: KalmanQMatSet
*功能说明: 过程噪声协方差矩阵设置
*形    参: 卡尔曼结构体指针 协方差矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanQMatSet(Kalman_t* kalman, float* q)
{
    uint8_t i;

    //协方差矩阵赋值
    for(i=0; i<9; i++)
    {
        kalman->q[i] = q[i];
    }
}

/**********************************************************************************************************
*函 数 名: KalmanRMatSet
*功能说明: 测量噪声协方差矩阵设置
*形    参: 卡尔曼结构体指针 协方差矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanRMatSet(Kalman_t* kalman, float* r)
{
    uint8_t i;

    //协方差矩阵赋值
    for(i=0; i<9; i++)
    {
        kalman->r[i] = r[i];
    }
}

/**********************************************************************************************************
*函 数 名: KalmanBMatSet
*功能说明: 输入控制矩阵设置
*形    参: 卡尔曼结构体指针 输入控制矩阵
*返 回 值: 无
**********************************************************************************************************/
void KalmanBMatSet(Kalman_t* kalman, float* b)
{
    uint8_t i;

    for(i=0; i<9; i++)
    {
        kalman->b[i] = b[i];
    }
}

/**********************************************************************************************************
*函 数 名: KalmanSlidWindowUpdate
*功能说明: 卡尔曼状态量滑动窗口更新
*形    参: 卡尔曼结构体指针
*返 回 值: 无
**********************************************************************************************************/
static void KalmanSlidWindowUpdate(Kalman_t* kalman)
{
    for(uint16_t i=0; i<kalman->slidWindowSize-1; i++)
    {
        kalman->statusSlidWindow[i] = kalman->statusSlidWindow[i+1];
    }
    kalman->statusSlidWindow[kalman->slidWindowSize - 1] = kalman->state;
}




