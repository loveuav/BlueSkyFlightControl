/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     missionControl.c
 * @说明     自主控制功能，如自动下降、自动起飞、自动返航、自动航线等
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.05 
**********************************************************************************************************/
#include "missionControl.h"
#include "flightStatus.h"
#include "flightControl.h"
#include "board.h"
#include "navigation.h"
#include "ahrs.h"
#include "gps.h"
#include "rc.h"

static Vector3f_t posCtlTarget;

static uint8_t rthStep;
static float rthHeight   = 3000;  //预设返航高度30m
static float rthWaitTime = 1000;  //返航回到Home点后的悬停等待时间

void AutoLand(void);
void ReturnToHome(void);

/**********************************************************************************************************
*函 数 名: MissionControl
*功能说明: 自主控制任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void MissionControl(void)
{
    RCCOMMAND_t rcCommand;
    uint8_t flightMode;
    RCTARGET_t rcTarget;    
    
    static float rollRate  = (float)MAXANGLE / MAXRCDATA;
    static float pitchRate = (float)MAXANGLE / MAXRCDATA;   
    
    //获取当前飞行模式
    flightMode = GetFlightMode();    

    //判断当前飞行模式并选择执行对应的飞行任务
    if(flightMode == AUTOLAND)        
    {
        //自动降落
        AutoLand();
    }
    else if(flightMode == RETURNTOHOME)  
    {
        //自动返航     
        ReturnToHome();
    }
    else
    {
        //在非任务控制模式下，更新当前位置目标
        posCtlTarget = GetCopterPosition();
        
        //返航步骤标志位复位
        rthStep = 0;
    }

    //获取摇杆数据
    rcCommand = GetRcCommad();

    //通用控制部分，将摇杆量转换为横滚俯仰的目标控制角度
    rcTarget.roll  = rcCommand.roll  * rollRate;
    rcTarget.pitch = rcCommand.pitch * pitchRate;   
    
    //设置目标控制量
    SetRcTarget(rcTarget);  
}

/**********************************************************************************************************
*函 数 名: AutoLand
*功能说明: 自动降落
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void AutoLand(void)
{
    static float velCtlTarget = 0;
    float alttitude = GetCopterPosition().z;
    
    //使能航向锁定
    SetYawHoldStatus(ENABLE);
    
    //GPS可用时，降落过程中保持位置不变，若不可用，降落过程中飞机姿态摇杆可控
    if(GpsGetFixStatus() == true)
    {
        //使能位置控制
        SetPosCtlStatus(ENABLE);
        
        //更新位置控制目标
        SetPosOuterCtlTarget(posCtlTarget);
    }

    //直接控制速度，禁用高度控制
    SetAltCtlStatus(DISABLE);
    
    //更新高度控制状态
    SetAltControlStatus(ALT_CHANGED);
    
    //减速降落
    //在没有对地测距传感器的情况下，只能大致判断高度，提前进行减速
    if(alttitude < 500)
    {
        velCtlTarget = velCtlTarget * 0.99f - 65.0f * 0.01f; 
    }
    else if(alttitude < 1000)
    {
        velCtlTarget = velCtlTarget * 0.99f - 80.0f * 0.01f;         
    }
    else if(alttitude < 5000) 
    {
        velCtlTarget = velCtlTarget * 0.99f - 200.0f * 0.01f;         
    }    
    else
    {
        velCtlTarget = velCtlTarget * 0.99f - 250.0f * 0.01f;         
    }    
}

/**********************************************************************************************************
*函 数 名: ReturnToHome
*功能说明: 自动返航
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ReturnToHome(void)
{
    /********************************************自动返航控制逻辑**********************************************
    
    1.先判断当前Home点距离，若小于一定值，则直接转入自动降落模式
    2.记录当前飞机航向，然后更改航向目标为Home方向（机头朝Home点）
    3.判断当前高度，若小于返航高度则上升，反之进入下一步
    4.转为控速模式，开始返航，根据Home点距离调节飞行速度
    5.Home点距离小于一定值时转为位置控制模式
    6.到达Home点上方，将航向目标变为初始记录值，并等待一定时间（预设值），转入自动降落模式，返航完毕
    
    全程检测GPS定位状态，若失去定位则转入自动降落模式
    **********************************************************************************************************/
    
    static uint32_t waitTime;
    static float originYaw;
    static float distanceToHome;
    static float directionToHome;
    Vector3f_t position;
    static Vector3f_t velCtlTarget; 
    
    //获取当前位置
    position = GetCopterPosition();
    //计算Home点距离
    distanceToHome  = Pythagorous2(position.x, position.y);
    //计算Home点方向
    directionToHome = GetDirectionToHome(position);
    
    switch(rthStep)
    {
        case RTH_STEP_START:
            //记录起始航向
            originYaw = GetCopterAngle().z; 
            
            //判断当前Home点距离，小于一定值则直接进入自动降落模式
            if(distanceToHome < 888)
                SetFlightMode(AUTOLAND);    
            
            //使能位置控制
            SetPosCtlStatus(ENABLE);  
              
            rthStep = RTH_STEP_TURN;
            break;
        
        case RTH_STEP_TURN:
            //设置Home方向为航向目标
            SetYawCtlTarget(directionToHome);
            
            if(abs(directionToHome - GetCopterAngle().z) < 3)
            {
                rthStep = RTH_STEP_CLIMB;
            }
            break;
 
        case RTH_STEP_CLIMB:
            //若当前高度小于返航高度，则更新高度控制目标，反之保持当前高度
            if(position.z < rthHeight)
            {
                //设置高度目标为返航高度
                SetAltOuterCtlTarget(posCtlTarget.z); 
            }

            //到达目标高度后进入下一步骤
            if(position.z - rthHeight > -50)
            {
                rthStep = RTH_STEP_FLIGHT_VEL;
            }
            break;
            
        case RTH_STEP_FLIGHT_VEL:
            //位置控制失能
            SetPosCtlStatus(DISABLE);  
            
            //不断更新航向目标为Home方向
            SetYawCtlTarget(directionToHome);   
            
            //根据Home点距离调整返航飞行速度
            if(distanceToHome < 1500)
                velCtlTarget.x = velCtlTarget.x * 0.99f + 150.0f * 0.01f;	
            else if(distanceToHome < 2000)
                velCtlTarget.x = velCtlTarget.x * 0.99f + 200.0f * 0.01f;	
            else if(distanceToHome < 3000)
                velCtlTarget.x = velCtlTarget.x * 0.99f + 300.0f * 0.01f;	
            else if(distanceToHome < 8000)
                velCtlTarget.x = velCtlTarget.x * 0.99f + 500.0f * 0.01f;				
            else
                velCtlTarget.x = velCtlTarget.x * 0.993f + 800.0f * 0.007f;
            
            velCtlTarget.y = 0;
            
            //更新速度控制目标    
            SetPosInnerCtlTarget(velCtlTarget);  

            //Home点距离小于10m时，转入位置控制模式
            if(distanceToHome < 1000)
            {
                rthStep = RTH_STEP_FLIGHT_POS;
                
                //更新位置控制目标
                posCtlTarget = GetCopterPosition();     
                
                //使能位置控制
                SetPosCtlStatus(ENABLE);  
            }
            break;

        case RTH_STEP_FLIGHT_POS:
            posCtlTarget.x -= posCtlTarget.x * 0.01f;
            posCtlTarget.y -= posCtlTarget.y * 0.01f;
          
            //Home点距离小于1m则进入下一步
            if(distanceToHome < 100)
            {
                rthStep = RTH_STEP_TURN_BACK;
                posCtlTarget.x = 0;
                posCtlTarget.y = 0;
            }
            
            //设置位置控制目标
            SetPosOuterCtlTarget(posCtlTarget);
            break;
            
        case RTH_STEP_TURN_BACK:
            //将航向控制目标设为初始记录值
            SetYawCtlTarget(originYaw);   
            
            if(abs(originYaw - GetCopterAngle().z) < 3)
            {
                rthStep = RTH_STEP_LOITER;
                waitTime = GetSysTimeMs();
            }
            break;

        case RTH_STEP_LOITER:    
            //等待预定的时间
            if(GetSysTimeMs() - waitTime > rthWaitTime)
            {
                //返航完毕，转入自动降落模式
                SetFlightMode(AUTOLAND); 
                //重置返航步骤标志位
                rthStep = RTH_STEP_START;
            }
            break;
            
        default:
            break;
    }
    
    //使能高度控制
    SetAltCtlStatus(ENABLE);      
    //使能航向锁定
    SetYawHoldStatus(ENABLE);
    
    //GPS不可用时直接转入自动降落模式
    if(GpsGetFixStatus() == false)
    {    
        SetFlightMode(AUTOLAND); 
    }
}




