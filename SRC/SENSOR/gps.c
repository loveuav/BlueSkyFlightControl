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

#define USE_VELNED

typedef struct{
	float acc;
	uint8_t satNum;
	uint8_t status;         //GPS状态：1表示已经成功定位并达到定位精度
	
	double latitude;
	double longitude;
	
	double homePosition[2];
    double originPosition[2];
	float magDeclination;
	
	Vector3f_t velocity;
	Vector3f_t position;
	
}GPS_t;

static void GpsCheckStatus(uint8_t* status, uint8_t satNum, float acc);
static void GpsSetOriginPosition(void);
static void GpsCalcPositionChanged(Vector3f_t* deltaDis, double lat1, double lon1, double lat2, double lon2);
void TransVelToBodyFrame(Vector3f_t velEf, Vector3f_t* velBf, float yaw);
void TransVelToEarthFrame(Vector3f_t velBf, Vector3f_t* velEf, float yaw);
static void GpsDetectCheck(float gpsTime);

#ifndef USE_VELNED
static void GpsCalcVelocity(double lat, double lon);
#endif

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
    GpsCheckStatus(&gps.status, gps.satNum, gps.acc);

	gps.latitude = gpsRaw.latitude;
	gps.longitude = gpsRaw.longitude;  
	
    //若GPS已经定位，则
    if(gps.status)
    {
        //将gps经纬度坐标转换为本地坐标系位置（cm） 
        GpsTransToLocalPosition(&gps.position, gps.latitude, gps.longitude);
        
        //通过坐标变化计算移动速度
        //如果直接从GPS模块获取速度值则无需调用此函数
        #ifdef USE_VELNED
        gps.velocity.x = gpsRaw.velN;
        gps.velocity.y = gpsRaw.velE; 
        #else
        GpsCalcVelocity(gps.latitude, gps.longitude);  
        #endif
               
    }
	
	//判断GPS模块连接状况
	GpsDetectCheck(gpsRaw.time);
}

/**********************************************************************************************************
*函 数 名: GpsCheckStatus
*功能说明: GPS状态及精度判断
*形    参: 状态指针 卫星数量 水平定位精度
*返 回 值: 无
**********************************************************************************************************/
static void GpsCheckStatus(uint8_t* status, uint8_t satNum, float acc)
{
	static uint8_t firstFix = 0;
	static uint16_t gpsFixCnt = 0;
		
	if(*status == 0)
    {
		if(satNum >= 6 && (acc < 2.5f && acc > 0) )
        {			
			gpsFixCnt++;
			
			if(gpsFixCnt > 10)
            {
				*status = 1;
				
				if(!firstFix)
                {
					firstFix = 1;
                    
                    //设置坐标系原点坐标
                    GpsSetOriginPosition();
                    
					//重置Home点坐标
					GpsResetHomePosition();                    
				}		
				gpsFixCnt = 0;
			}
		}
	}
	else
    {
		if(satNum <= 4 || acc > 5.0f || FaultDetectGetErrorStatus(GPS_UNDETECTED))
        {
			*status = 0;
		}
	}
}

/**********************************************************************************************************
*函 数 名: GpsResetHomePosition
*功能说明: 重置Home点坐标
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void GpsResetHomePosition(void)
{	
    if(gps.status)
    {
        gps.homePosition[LAT] = gps.latitude;
        gps.homePosition[LON] = gps.longitude;
    }		
}

/**********************************************************************************************************
*函 数 名: GpsSetHomePosition
*功能说明: 设置本地坐标系坐标原点位置，并计算当地磁偏角
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
static void GpsSetOriginPosition(void)
{	
    gps.originPosition[LAT] = gps.latitude;
    gps.originPosition[LON] = gps.longitude;		
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
*函 数 名: GpsTransToLocalPosition
*功能说明: 将经纬度坐标转换为本地坐标系位置
*形    参: 位置 纬度 经度
*返 回 值: 无
**********************************************************************************************************/
void GpsTransToLocalPosition(Vector3f_t* position, double lat, double lon)
{
	GpsCalcPositionChanged(position, lat, lon, gps.originPosition[LAT], gps.originPosition[LON]);	
}

/**********************************************************************************************************
*函 数 名: GpsCalcVelocity
*功能说明: 计算GPS速度 单位：cm/s
*形    参: 纬度坐标 经度坐标
*返 回 值: 无
**********************************************************************************************************/
#ifndef USE_VELNED
static void GpsCalcVelocity(double lat, double lon)
{
	double deltaDis[2];
	static double lastGPSPosition[2] = {0,0};
	static uint8_t init = 0;
	double rads;
	double gpsLngDownScale;	
	
	static uint64_t previousT;
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
#endif

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
*函 数 名: GetDirectionOfTwoPoint
*功能说明: 计算两个坐标点之间的方向
*形    参: 源坐标点 目标坐标点
*返 回 值: 方向
**********************************************************************************************************/
float GetDirectionOfTwoPoint(Vector3f_t point1, Vector3f_t point2)
{
	float direction;
	float deltaDisX, deltaDisY;
	
	deltaDisX = point2.x - point1.x;
	deltaDisY = point2.y - point1.y;
	
	direction = Degrees(atan2f(deltaDisY, deltaDisX));
	direction = WrapDegree360(direction);
    
	return direction;
}

/**********************************************************************************************************
*函 数 名: GetDirectionToHome
*功能说明: 计算Home点方向
*形    参: 无
*返 回 值: 方向
**********************************************************************************************************/
float GetDirectionToHome(Vector3f_t position)
{
    return GetDirectionOfTwoPoint(position, GetHomePosition());
}

/**********************************************************************************************************
*函 数 名: GetDistanceToHome
*功能说明: 计算Home点距离
*形    参: 无
*返 回 值: 距离
**********************************************************************************************************/
float GetDistanceToHome(Vector3f_t position)
{
    Vector3f_t homePosition;
    float      distance;
    
    homePosition = GetHomePosition();
    
    distance = Pythagorous2(position.x - homePosition.x, position.y - homePosition.y);
    
    return distance;
}

/**********************************************************************************************************
*函 数 名: GetHomePosition
*功能说明: 获取Home点位置（本地坐标系）
*形    参: 无
*返 回 值: 位置
**********************************************************************************************************/
Vector3f_t GetHomePosition(void)
{	
    Vector3f_t homePosition;
    
    GpsTransToLocalPosition(&homePosition, gps.homePosition[LAT], gps.homePosition[LON]);
    
    return homePosition;
}

/**********************************************************************************************************
*函 数 名: GetHomePosition
*功能说明: 获取Home点经纬度坐标
*形    参: 纬度指针 经度指针
*返 回 值: 无
**********************************************************************************************************/
void GetHomeLatitudeAndLongitude(double* lat, double* lon)
{	
    *lat = gps.homePosition[LAT];
    *lon = gps.homePosition[LON];
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
*函 数 名: GpsGetAccuracy
*功能说明: 获取GPS水平定位精度
*形    参: 无 
*返 回 值: 精度值，单位为m
**********************************************************************************************************/
float GpsGetAccuracy(void)
{
    return gps.acc;
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




