/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     LevenbergMarquardt.c
 * @说明     Levenberg-Marquardt算法，求解非线性方程最优解，结合了高斯牛顿法和梯度下降
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.08
**********************************************************************************************************/
#include "LevenbergMarquardt.h"

/*
本算法的详细讲解：
https://blog.csdn.net/loveuav/article/details/81592870
*/

//LM算法因子
double lm_lambda;

static void UpdateMatrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
static void ResetMatrices(float dS[6], float JS[6][6]);
static void GaussEliminateSolveDelta(float dS[6], float JS[6][6], float delta[6]);

/**********************************************************************************************************
*函 数 名: LevenbergMarquardt
*功能说明: Levenberg-Marquardt算法求解传感器误差方程，得到校准参数
*形    参: 传感器采样数据（6组） 零偏误差指针 比例误差指针 方程初解 数据向量长度
*返 回 值: 无
**********************************************************************************************************/
void LevenbergMarquardt(Vector3f_t inputData[6], Vector3f_t* offset, Vector3f_t* scale, float initBeta[6], float length)
{
    uint32_t cnt    = 0;
    double   eps    = 1e-10;
    double   change = 100.0;
    double   changeTemp = 100.0;
    float    data[3];
    float    beta[6];      //方程解
    float    delta[6];     //迭代步长
    float    JtR[6];       //梯度矩阵
    float    JtJ[6][6];    //海森矩阵

    //方程解赋初值
    for(uint8_t i=0; i<6; i++)
    {
        beta[i] = initBeta[i];
    }

    //LM因子初始化
    lm_lambda = 0.1;

    //开始迭代，当迭代步长小于eps时结束计算，得到方程近似最优解
    while(change > eps)
    {
        //矩阵初始化
        ResetMatrices(JtR, JtJ);

        //计算误差方程函数的梯度JtR和海森矩阵JtJ
        for(uint8_t i=0; i<6; i++)
        {
            data[0] = inputData[i].x;
            data[1] = inputData[i].y;
            data[2] = inputData[i].z;
            UpdateMatrices(JtR, JtJ, beta, data);
        }

        //高斯消元法求解方程：(JtJ + lambda*I) * delta = JtR，得到delta
        GaussEliminateSolveDelta(JtR, JtJ, delta);

        //计算迭代步长
        changeTemp = delta[0]*delta[0] +
                     delta[0]*delta[0] +
                     delta[1]*delta[1] +
                     delta[2]*delta[2] +
                     delta[3]*delta[3] / (beta[3]*beta[3]) +
                     delta[4]*delta[4] / (beta[4]*beta[4]) +
                     delta[5]*delta[5] / (beta[5]*beta[5]);

        //根据步长大小更新LM因子
        //LM因子为0时，算法退化为高斯牛顿法，LM因子变大时，退化为步长较小的梯度下降法
        if(changeTemp < change)
        {
            //LM因子减小
            lm_lambda /= 3;

            //更新方程解
            for(uint8_t i=0; i<6; i++)
            {
                beta[i] -= delta[i];
            }

            change = changeTemp;
        }
        else
        {
            //LM因子增大
            lm_lambda *= 3;
            lm_lambda = ConstrainFloat(lm_lambda, 0, 1e10);
        }

        //限制迭代次数
        if(cnt++ > 1000)
            break;
    }

    //更新校准参数
    scale->x  = beta[3] * length;
    scale->y  = beta[4] * length;
    scale->z  = beta[5] * length;
    offset->x = beta[0];
    offset->y = beta[1];
    offset->z = beta[2];
}

/**********************************************************************************************************
*函 数 名: UpdateMatrices
*功能说明: 更新梯度与海森矩阵
*形    参: 梯度矩阵 海森矩阵 方程解 观测数据
*返 回 值: 无
**********************************************************************************************************/
static void UpdateMatrices(float JtR[6], float JtJ[6][6], float beta[6], float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];

    //误差方程：residual = length² - ((x-b0) * b3)² + ((y-b1) * b4)² + ((z-b2) * b5)²
    //前面对参数进行了归一化，所以这里的length == 1
    for(j=0; j<3; j++)
    {
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        //计算残差
        residual -= b*b*dx*dx;

        //计算雅可比矩阵
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }

    for(j=0; j<6; j++)
    {
        //计算函数梯度
        JtR[j] += jacobian[j]*residual;

        for(k=0; k<6; k++)
        {
            //计算海森矩阵（简化形式，省略二阶偏导），即雅可比矩阵与其转置的乘积
            JtJ[j][k] += jacobian[j]*jacobian[k];
        }
    }
}

/**********************************************************************************************************
*函 数 名: ResetMatrices
*功能说明: 重置矩阵
*形    参: 梯度矩阵 海森矩阵
*返 回 值: 无
**********************************************************************************************************/
static void ResetMatrices(float JtR[6], float JtJ[6][6])
{
    int16_t j,k;
    for(j=0; j<6; j++)
    {
        JtR[j] = 0.0f;
        for(k=0; k<6; k++)
        {
            JtJ[j][k] = 0.0f;
        }
    }
}

/**********************************************************************************************************
*函 数 名: GaussEliminateSolveDelta
*功能说明: 高斯消元法求解方程: (JtJ + lambda*I) * delta = JtR
*形    参: 梯度矩阵 海森矩阵 delta
*返 回 值: 无
**********************************************************************************************************/
static void GaussEliminateSolveDelta(float JtR[6], float JtJ[6][6], float delta[6])
{
    int16_t i,j,k;
    float mu;

    //加入LM因子
    for(i=0; i<6; i++)
    {
        JtJ[i][i] += lm_lambda;
    }

    //逐次消元，将线性方程组转换为上三角方程组
    for(i=0; i<6; i++)
    {
        //若JtJ[i][i]不为0，将该列在JtJ[i][i]以下的元素消为0
        for(j=i+1; j<6; j++)
        {
            mu = JtJ[i][j] / JtJ[i][i];
            if(mu != 0.0f)
            {
                JtR[j] -= mu * JtR[i];
                for(k=j; k<6; k++)
                {
                    JtJ[k][j] -= mu * JtJ[k][i];
                }
            }
        }
    }

    //回代得到方程组的解
    for(i=5; i>=0; i--)
    {
        JtR[i] /= JtJ[i][i];
        JtJ[i][i] = 1.0f;

        for(j=0; j<i; j++)
        {
            mu = JtJ[i][j];
            JtR[j] -= mu * JtR[i];
            JtJ[i][j] = 0.0f;
        }
    }

    for(i=0; i<6; i++)
    {
        delta[i] = JtR[i];
    }
}



