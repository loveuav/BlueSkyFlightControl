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
*函 数 名: Vector3iTo3f
*功能说明: 向量类型转换
*形    参: Vector3i型
*返 回 值: Vector3f型
**********************************************************************************************************/
Vector3f_t Vector3iTo3f(Vector3i_t vector)
{
	Vector3f_t temp;
	temp.x = (float)vector.x;
	temp.y = (float)vector.y;	
	temp.z = (float)vector.z;	
	return temp;
}

/**********************************************************************************************************
*函 数 名: Vector3fTo3i
*功能说明: 向量类型转换
*形    参: Vector3f型
*返 回 值: Vector3i型
**********************************************************************************************************/
Vector3i_t Vector3fTo3i(Vector3f_t vector)
{
	Vector3i_t temp;
	temp.x = (int16_t)vector.x;
	temp.y = (int16_t)vector.y;	
	temp.z = (int16_t)vector.z;	
	return temp;
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
*函 数 名: Vector3MulMatrix3
*功能说明: 三维向量与三维矩阵相乘
*形    参: 三维向量 三维矩阵
*返 回 值: 乘积
**********************************************************************************************************/
Vector3f_t Vector3MulMatrix3(Vector3f_t vector, float* m)
{
    Vector3f_t v_tmp;
    
    v_tmp.x = vector.x * m[0] + vector.y * m[3] + vector.z * m[6];
    v_tmp.y = vector.x * m[1] + vector.y * m[4] + vector.z * m[7];
    v_tmp.z = vector.x * m[2] + vector.y * m[5] + vector.z * m[8];
    
    return v_tmp;
}

/**********************************************************************************************************
*函 数 名: VectorRotate
*功能说明: 三维向量的旋转
*形    参: 三维向量 角度变化量（弧度）
*返 回 值: 旋转后的三维向量
**********************************************************************************************************/
Vector3f_t VectorRotate(Vector3f_t vector, Vector3f_t deltaAngle)
{
    float dcMat[9];
    Vector3f_t cos, sin;

    //欧拉角转化为方向余弦矩阵
    cos.x = cosf(deltaAngle.x);
    cos.y = cosf(deltaAngle.y);
    cos.z = cosf(deltaAngle.z);   
    sin.x = sinf(deltaAngle.x);
    sin.y = sinf(deltaAngle.y);
    sin.z = sinf(deltaAngle.z);      
    dcMat[0] = cos.y * cos.z; 
    dcMat[1] = -cos.y * sin.z;
    dcMat[2] = sin.y;
    dcMat[3] = sin.z * cos.x + cos.z * sin.x * sin.y;
    dcMat[4] = cos.z * cos.x - sin.z * sin.x * sin.y;
    dcMat[5] = -sin.x * cos.y;
    dcMat[6] = sin.z * sin.x - cos.z * cos.x * sin.y;
    dcMat[7] = cos.z * sin.x + sin.z * cos.x * sin.y;
    dcMat[8] = cos.y * cos.x;

    //向量乘以方向余弦矩阵，得到旋转后的新向量
    return Vector3MulMatrix3(vector, dcMat);
}


