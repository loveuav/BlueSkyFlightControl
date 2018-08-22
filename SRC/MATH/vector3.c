/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     vector3.c
 * @说明     3维向量结构体的定义与相关运算函数
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "vector3.h"

/**********************************************************************************************************
*函 数 名: Vector3f_Normalize
*功能说明: 向量归一化
*形    参: Vector3f型
*返 回 值: 无
**********************************************************************************************************/
void Vector3f_Normalize(Vector3f_t* vector)
{
    float mag = Pythagorous3(vector->x, vector->y, vector->z);
    
    vector->x /= mag;
    vector->y /= mag;
    vector->z /= mag;
}

/**********************************************************************************************************
*函 数 名: Vector3iTo3f
*功能说明: 向量类型转换
*形    参: Vector3i型
*返 回 值: Vector3f型
**********************************************************************************************************/
Vector3f_t Vector3iTo3f(Vector3i_t vector)
{
	Vector3f_t v_tmp;
	v_tmp.x = (float)vector.x;
	v_tmp.y = (float)vector.y;	
	v_tmp.z = (float)vector.z;	
	return v_tmp;
}

/**********************************************************************************************************
*函 数 名: Vector3fTo3i
*功能说明: 向量类型转换
*形    参: Vector3f型
*返 回 值: Vector3i型
**********************************************************************************************************/
Vector3i_t Vector3fTo3i(Vector3f_t vector)
{
	Vector3i_t v_tmp;
	v_tmp.x = (int16_t)vector.x;
	v_tmp.y = (int16_t)vector.y;	
	v_tmp.z = (int16_t)vector.z;	
	return v_tmp;
}

/**********************************************************************************************************
*函 数 名: Vector3f_Add
*功能说明: 浮点型向量加法
*形    参: 向量v1 向量v2
*返 回 值: 向量和
**********************************************************************************************************/
Vector3f_t Vector3f_Add(Vector3f_t v1, Vector3f_t v2)
{
    Vector3f_t v_tmp;
    
    v_tmp.x = v1.x + v2.x;
    v_tmp.y = v1.y + v2.y;
    v_tmp.z = v1.z + v2.z;
    
    return v_tmp;
}

/**********************************************************************************************************
*函 数 名: Vector3f_Sub
*功能说明: 浮点型向量减法
*形    参: 向量v1 向量v2
*返 回 值: 向量差
**********************************************************************************************************/
Vector3f_t Vector3f_Sub(Vector3f_t v1, Vector3f_t v2)
{
    Vector3f_t v_tmp;
    
    v_tmp.x = v1.x - v2.x;
    v_tmp.y = v1.y - v2.y;
    v_tmp.z = v1.z - v2.z;
    
    return v_tmp;
}

/**********************************************************************************************************
*函 数 名: VectorCrossProduct
*功能说明: 三维向量的叉积运算
*形    参: 向量a 向量b
*返 回 值: 叉积运算结果
**********************************************************************************************************/
Vector3f_t VectorCrossProduct(Vector3f_t a, Vector3f_t b)
{
	Vector3f_t c;
	
	c.x = a.y * b.z - b.y * a.z;
	c.y = a.z * b.x - b.z * a.x;
	c.z = a.x * b.y - b.x * a.y;
	
	return c;
}

/**********************************************************************************************************
*函 数 名: Matrix3MulVector3
*功能说明: 三维矩阵与三维向量相乘
*形    参: 三维矩阵 三维向量 
*返 回 值: 乘积
**********************************************************************************************************/
Vector3f_t Matrix3MulVector3(float* m, Vector3f_t vector)
{
    Vector3f_t v_tmp;
    
    v_tmp.x = m[0] * vector.x + m[1] * vector.y + m[2] * vector.z;
    v_tmp.y = m[3] * vector.x + m[4] * vector.y + m[5] * vector.z;
    v_tmp.z = m[6] * vector.x + m[7] * vector.y + m[8] * vector.z;
    
    return v_tmp;
}

/**********************************************************************************************************
*函 数 名: EulerAngleToDCM
*功能说明: 欧拉角转方向余弦矩阵（参考系到机体系）
*形    参: 欧拉角 矩阵指针
*返 回 值: 无
**********************************************************************************************************/
void EulerAngleToDCM(Vector3f_t angle, float* dcM)
{
    Vector3f_t cos, sin;
    
    cos.x = cosf(angle.x);
    cos.y = cosf(angle.y);
    cos.z = cosf(angle.z);   
    sin.x = sinf(angle.x);
    sin.y = sinf(angle.y);
    sin.z = sinf(angle.z);    

    dcM[0] = cos.y * cos.z; 
    dcM[1] = sin.z * cos.x + sin.x * sin.y * cos.z; 
    dcM[2] = -sin.x * sin.z + sin.y * cos.x * cos.z; 
    dcM[3] = -sin.z * cos.y;
    dcM[4] = cos.x * cos.z - sin.x * sin.y * sin.z;
    dcM[5] = -sin.x * cos.z - sin.y * sin.z * cos.x; 
    dcM[6] = -sin.y;
    dcM[7] = sin.x * cos.y;
    dcM[8] = cos.x * cos.y;
}

/**********************************************************************************************************
*函 数 名: EulerAngleToDCM_T
*功能说明: 欧拉角转方向余弦矩阵（机体系到参考系）
*形    参: 欧拉角 矩阵指针
*返 回 值: 无
**********************************************************************************************************/
void EulerAngleToDCM_T(Vector3f_t angle, float* dcM)
{
    Vector3f_t cos, sin;
    
    cos.x = cosf(angle.x);
    cos.y = cosf(angle.y);
    cos.z = cosf(angle.z);   
    sin.x = sinf(angle.x);
    sin.y = sinf(angle.y);
    sin.z = sinf(angle.z);    

    dcM[0] = cos.y * cos.z; 
    dcM[1] = -sin.z * cos.y;
    dcM[2] = -sin.y;
    dcM[3] = sin.z * cos.x + sin.x * sin.y * cos.z; 
    dcM[4] = cos.x * cos.z - sin.x * sin.y * sin.z;
    dcM[5] = sin.x * cos.y;
    dcM[6] = -sin.x * sin.z + sin.y * cos.x * cos.z; 
    dcM[7] = -sin.x * cos.z - sin.y * sin.z * cos.x;
    dcM[8] = cos.x * cos.y;
}

/**********************************************************************************************************
*函 数 名: VectorRotateToBodyFrame
*功能说明: 三维向量旋转至机体系
*形    参: 三维向量 角度变化量（弧度）
*返 回 值: 旋转后的三维向量
**********************************************************************************************************/
Vector3f_t VectorRotateToBodyFrame(Vector3f_t vector, Vector3f_t deltaAngle)
{
    float dcMat[9];

    //欧拉角转为方向余弦矩阵
    EulerAngleToDCM(deltaAngle, dcMat);

    //方向余弦矩阵乘以向量，得到旋转后的新向量
    return Matrix3MulVector3(dcMat, vector);
}

/**********************************************************************************************************
*函 数 名: VectorRotateToEarthFrame
*功能说明: 三维向量旋转至参考系
*形    参: 三维向量 角度变化量（弧度）
*返 回 值: 旋转后的三维向量
**********************************************************************************************************/
Vector3f_t VectorRotateToEarthFrame(Vector3f_t vector, Vector3f_t deltaAngle)
{
    float dcMat[9];

    //欧拉角转为方向余弦矩阵
    EulerAngleToDCM_T(deltaAngle, dcMat);

    //方向余弦矩阵乘以向量，得到旋转后的新向量
    return Matrix3MulVector3(dcMat, vector);
}

/**********************************************************************************************************
*函 数 名: AccVectorToEulerAngle
*功能说明: 根据重力加速度向量在机体系上的投影计算俯仰和横滚角
*形    参: 姿态角指针 加速度向量
*返 回 值: 无
**********************************************************************************************************/
void AccVectorToRollPitchAngle(Vector3f_t* angle, Vector3f_t vector)
{
    //加速度向量归一化
    Vector3f_Normalize(&vector);
    
	angle->x = -SafeArcsin(vector.y);       //横滚角
	angle->y = atan2f(vector.x, vector.z);  //俯仰角
}

/**********************************************************************************************************
*函 数 名: MagVectorToEulerAngle
*功能说明: 根据地磁场向量在机体系上的投影计算偏航角
*形    参: 姿态角指针 地磁场向量
*返 回 值: 无
**********************************************************************************************************/
void MagVectorToYawAngle(Vector3f_t* angle, Vector3f_t vector)
{
	angle->z = -atan2f(vector.y, vector.x);     //偏航角
}



