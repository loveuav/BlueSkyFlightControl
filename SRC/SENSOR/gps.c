/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     gps.c
 * @说明     GPS数据预处理
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "gps.h"
#include "declination.h"
#include "ublox.h"
#include "board.h"
#include "faultDetect.h"

#define LAT  0 //纬度 latitude
#define LON  1 //经度 longitude

typedef struct{
	float acc;
	uint8_t satNum;
	uint8_t status;         //GPS状态：1表示已经成功定位并达到定位精度
	
	double latitude;
	double longitude;
	
	double homePosition[2];
	float magDeclination;
	
	Vector3f_t velocity;
	Vector3f_t position;
	
}GPS_t;

static uint8_t GpsCheckStatus(uint8_t satNum, float acc);
static void GpsSetHomePosition(void);
static void GpsCalcPositionChanged(Vector3f_t* deltaDis, double lat1, double lon1, double lat2, double lon2);
static void GpsCalcVelocity(double lat, double lon);
void TransVelToBodyFrame(Vector3f_t velEf, Vector3f_t* velBf, float yaw);
void TransVelToEarthFrame(Vector3f_t velBf, Vector3f_t* velEf, float yaw);
static void GpsDetectCheck(float gpsTime);

GPS_t gps;

/**********************************************************************************************************
*函 数 名: GpsDataPreTreat
*功能说明: GPS数据预处理
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GpsDataPreTreat(void)
{
    UBLOX_t gpsRaw = Ublox_GetData();
    
    gps.acc = gpsRaw.hAcc;
    gps.satNum = gpsRaw.numSV;
    
    //检查GPS状态，判断当前定位精度是否满足使用要求
    gps.status = GpsCheckStatus(gps.satNum, gps.acc);

	gps.latitude = gpsRaw.latitude;
	gps.longitude = gpsRaw.longitude;  
	
    //若GPS已经定位，则
    if(gps.status)
    {
        //以Home点为坐标原点计算GPS位置（cm）
        GpsCalcPositionChanged(&gps.position, gps.latitude, gps.longitude, gps.homePosition[LAT], gps.homePosition[LON]);	 

        //通过坐标变化计算移动速度
        //如果直接从GPS模块获取速度值则无需调用此函数
        //GpsCalcVelocity(gps.latitude, gps.longitude);  
        gps.velocity.x = gpsRaw.velN;
        gps.velocity.y = gpsRaw.velE;        
    }
	
	//判断GPS模块连接状况
	GpsDetectCheck(gpsRaw.time);
    
}

/**********************************************************************************************************
*函 数 名: GpsCheckStatus
*功能说明: GPS状态及精度判断
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static uint8_t GpsCheckStatus(uint8_t satNum, float acc)
{
	static uint8_t firstFix = 0;
	static uint16_t gpsFixCnt = 0;
    static uint8_t status = 0;
		
	if(!status){
		if(satNum >= 6 && (acc < 2.5f && acc > 0) ){			
			gpsFixCnt++;
			
			if(gpsFixCnt > 10){
				status = 1;
				
				if(!firstFix){
					firstFix = 1;
					//设置Home点坐标
					GpsSetHomePosition();
				}		
				gpsFixCnt = 0;
			}
		}
	}
	else{
		if(satNum <= 4 || acc > 3.5f){
			status = 0;
		}
	}
    
    return status;
}

/**********************************************************************************************************
*函 数 名: GpsSetHomePosition
*功能说明: 设置Home点坐标 并计算当地磁偏角
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void GpsSetHomePosition(void)
{	
    gps.homePosition[LAT] = gps.latitude;
    gps.homePosition[LON] = gps.longitude;		
    //使用经纬度坐标计算磁偏角
    gps.magDeclination = CompassGetDeclination(gps.latitude, gps.longitude);
}

/**********************************************************************************************************
*函 数 名: GpsCalcPositionChanged
*功能说明: 计算两个坐标间的距离
*形    参: 距离值 纬度1 经度1 纬度2 经度2
*返 回 值: 无
**********************************************************************************************************/
static void GpsCalcPositionChanged(Vector3f_t* deltaDis, double lat1, double lon1, double lat2, double lon2)
{
	double rads = Radians(abs(lat1));
	double gpsLngDownScale = cosf(rads);

	deltaDis->y = (int32_t)((lon1 - lon2) * gpsLngDownScale * 11131950);
	deltaDis->x = (int32_t)((lat1 - lat2) * 11131950);	
}

/**********************************************************************************************************
*函 数 名: GpsCalcVelocity
*功能说明: 计算GPS速度 单位：cm/s
*形    参: 纬度坐标 经度坐标
*返 回 值: 无
**********************************************************************************************************/
static void GpsCalcVelocity(double lat, double lon)
{
	double deltaDis[2];
	static double lastGPSPosition[2] = {0,0};
	static uint8_t init = 0;
	double rads;
	double gpsLngDownScale;	
	
	static uint32_t previousT;
	float  deltaT = (GetSysTimeUs() - previousT) * 1e-6;	
	previousT = GetSysTimeUs();		

    if(init){
        rads = Radians((abs(gps.latitude)));
        gpsLngDownScale = cosf(rads);
        
        deltaDis[LON] = (gps.longitude - lastGPSPosition[LON]) * gpsLngDownScale * 11131950;
        deltaDis[LAT] = (gps.latitude - lastGPSPosition[LAT]) * 11131950;
        
        gps.velocity.y = deltaDis[LON] / deltaT;
        gps.velocity.x = deltaDis[LAT] / deltaT;
    }
    init = 1;
    
    lastGPSPosition[LON] = gps.longitude;
    lastGPSPosition[LAT] = gps.latitude;
}

/**********************************************************************************************************
*函 数 名: TransVelToBodyFrame
*功能说明: 将地理坐标系下的速度转化到机体坐标系
*形    参: 地理坐标系下的速度，机体坐标系速度指针，航向角 
*返 回 值: 无
**********************************************************************************************************/
void TransVelToBodyFrame(Vector3f_t velEf, Vector3f_t* velBf, float yaw)
{
	float sinYaw = sinf(Radians(yaw));
	float cosYaw = cosf(Radians(yaw));	
	
	velBf->x = velEf.x * cosYaw + velEf.y * sinYaw;
	velBf->y = -velEf.x * sinYaw + velEf.y * cosYaw;			
}

/**********************************************************************************************************
*函 数 名: TransVelToEarthFrame
*功能说明: 将机体速度转化到地理坐标系
*形    参: 机体坐标系下的速度，地理坐标系速度指针，航向角 
*返 回 值: 无
**********************************************************************************************************/
void TransVelToEarthFrame(Vector3f_t velBf, Vector3f_t* velEf, float yaw)
{
	float sinYaw = sinf(Radians(yaw));
	float cosYaw = cosf(Radians(yaw));	
	
	velEf->x = velBf.x * cosYaw - velBf.y * sinYaw;
	velEf->y = velBf.x * sinYaw + velBf.y * cosYaw;			
	velEf->z = velBf.z;
}

/**********************************************************************************************************
*函 数 名: GetMagDeclination
*功能说明: 获取磁偏角
*形    参: 无 
*返 回 值: 磁偏角度值
**********************************************************************************************************/
float GetMagDeclination(void)
{
    return gps.magDeclination;
}

/**********************************************************************************************************
*函 数 名: GpsGetFixStatus
*功能说明: 获取GPS定位状态，为真表示GPS已定位并且定位精度达到要求
*形    参: 无 
*返 回 值: 状态
**********************************************************************************************************/
bool GpsGetFixStatus(void)
{
    return gps.status;
}

/**********************************************************************************************************
*函 数 名: GpsGetVelocity
*功能说明: 获取GPS的测量速度
*形    参: 无 
*返 回 值: 速度值 单位cm/s
**********************************************************************************************************/
Vector3f_t GpsGetVelocity(void)
{
    return gps.velocity;
}

/**********************************************************************************************************
*函 数 名: GpsGetPosition
*功能说明: 获取GPS的以Home点为坐标原点的测量位置
*形    参: 无 
*返 回 值: 位置值 单位cm
**********************************************************************************************************/
Vector3f_t GpsGetPosition(void)
{
    return gps.position;   
}

/**********************************************************************************************************
*函 数 名: GpsDetectCheck
*功能说明: 检测GPS模块连接是否正常，通过检测GPS时间变化来判断
*形    参: GPS时间 
*返 回 值: 无
**********************************************************************************************************/
static void GpsDetectCheck(float gpsTime)
{
	static uint32_t cnt;
	static float lastGpsTime = 0;
	
	if(gpsTime == lastGpsTime)
	{
		cnt++;
		
		if(cnt > 10)
		{
			//未检测到GPS模块
			FaultDetectSetError(GPS_UNDETECTED);
		}
	}
	else
	{
		cnt = 0;
		FaultDetectResetError(GPS_UNDETECTED);
	}
	
	lastGpsTime = gpsTime;	
}




