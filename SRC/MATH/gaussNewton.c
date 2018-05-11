/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     gaussNewton.c
 * @说明     用于求解非线性方程最优解的高斯牛顿算法
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "gaussNewton.h"

static void GaussNewtonUpdateMatrices(float dS[6], float JS[6][6], float beta[6], float data[3]);
static void GaussNewtonResetMatrices(float dS[6], float JS[6][6]);
static void GaussNewtonFindDelta(float dS[6], float JS[6][6], float delta[6]);

/**********************************************************************************************************
*函 数 名: GaussNewtonCalibrate
*功能说明: 高斯牛顿法求解传感器误差方程，得到校准参数
*形    参: 传感器采样数据（6组） 零偏误差指针 比例误差指针 数据向量长度 最大迭代次数
*返 回 值: 无
**********************************************************************************************************/
void GaussNewtonCalibrate(Vector3f_t inputData[6],Vector3f_t* offset, Vector3f_t* scale, 
     float length, int16_t maxIteration)
{
    int16_t i;
    int16_t num_iterations = 0;
    float eps = 0.000000001;
    float change = 100.0;
    float data[3];
    float beta[6];
    float delta[6];
    float ds[6];
    float JS[6][6];

    // reset
    beta[0] = beta[1] = beta[2] = 0;
    beta[3] = beta[4] = beta[5] = length;
    
    while(num_iterations < maxIteration && change > eps) 
    {
        num_iterations++;

        GaussNewtonResetMatrices(ds, JS);

        for( i=0; i<6; i++ ) 
        {
            data[0] = inputData[i].x;
            data[1] = inputData[i].y;
            data[2] = inputData[i].z;
            GaussNewtonUpdateMatrices(ds, JS, beta, data);
        }

        GaussNewtonFindDelta(ds, JS, delta);

        change = delta[0]*delta[0] +
                 delta[0]*delta[0] +
                 delta[1]*delta[1] +
                 delta[2]*delta[2] +
                 delta[3]*delta[3] / (beta[3]*beta[3]) +
                 delta[4]*delta[4] / (beta[4]*beta[4]) +
                 delta[5]*delta[5] / (beta[5]*beta[5]);

        for( i=0; i<6; i++ ) 
        {
            beta[i] -= delta[i];
        }
    }

    scale->x = beta[3] * length;
    scale->y = beta[4] * length;
    scale->z = beta[5] * length;
    offset->x = beta[0] * scale->x;
    offset->y = beta[1] * scale->y;
    offset->z = beta[2] * scale->z;
}

static void GaussNewtonUpdateMatrices(float dS[6], float JS[6][6], float beta[6], float data[3])
{
    int16_t j, k;
    float dx, b;
    float residual = 1.0;
    float jacobian[6];
    
    for(j=0; j<3; j++) 
    {
        //计算残差 
        b = beta[3+j];
        dx = (float)data[j] - beta[j];
        residual -= b*b*dx*dx;
        
        jacobian[j] = 2.0f*b*b*dx;
        jacobian[3+j] = -2.0f*b*dx*dx;
    }
    
    for(j=0; j<6; j++) 
    {
        dS[j] += jacobian[j]*residual;
        for(k=0; k<6; k++) 
        {
            JS[j][k] += jacobian[j]*jacobian[k];
        }
    }
}

static void GaussNewtonResetMatrices(float dS[6], float JS[6][6])
{
    int16_t j,k;
    for( j=0; j<6; j++ ) 
    {
        dS[j] = 0.0f;
        for( k=0; k<6; k++ ) 
        {
            JS[j][k] = 0.0f;
        }
    }
}

static void GaussNewtonFindDelta(float dS[6], float JS[6][6], float delta[6])
{
    //Solve 6-d matrix equation JS*x = dS
    //first put in upper triangular form
    int16_t i,j,k;
    float mu;

    //make upper triangular
    for( i=0; i<6; i++ )
    {
        //eliminate all nonzero entries below JS[i][i]
        for( j=i+1; j<6; j++ )
        {
            mu = JS[i][j]/JS[i][i];
            if( mu != 0.0f ) 
            {
                dS[j] -= mu*dS[i];
                for( k=j; k<6; k++ )
                {
                    JS[k][j] -= mu*JS[k][i];
                }
            }
        }
    }

    //back-substitute
    for( i=5; i>=0; i-- )
    {
        dS[i] /= JS[i][i];
        JS[i][i] = 1.0f;
        
        for( j=0; j<i; j++ )
        {
            mu = JS[i][j];
            dS[j] -= mu*dS[i];
            JS[i][j] = 0.0f;
        }
    }

    for( i=0; i<6; i++ )
    {
        delta[i] = dS[i];
    }
}



























