/**********************************************************************************************************
                                天穹飞控 —— 致力于打造中国最好的多旋翼开源飞控
                                Github: github.com/loveuav/BlueSkyFlightControl
                                技术讨论：bbs.loveuav.com/forum-68-1.html
 * @文件     mavlinkNotice.c
 * @说明     消息提示定义
 * @版本  	 V1.0
 * @作者     BlueSky
 * @网站     bbs.loveuav.com
 * @日期     2018.07
**********************************************************************************************************/
#include "mavlinkNotice.h"
#include "common/mavlink.h"

//提示文本字符串长度不能超过50
const MAV_NOTICE_t mavNotice[MAV_NOTICE_NUM] =
{
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] calibration started: 2 gyro"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] calibration started: 2 accel"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] calibration started: 2 mag"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] calibration started: 2 level"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] calibration done:"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] calibration failed:"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <0>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <10>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <20>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <30>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <40>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <50>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <60>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <70>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <80>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <90>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] progress <100>"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] up orientation detected"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] down orientation detected"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] left orientation detected"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] right orientation detected"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] front orientation detected"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] back orientation detected"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] up side done, rotate to a different side"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] down side done, rotate to a different side"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] left side done, rotate to a different side"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] right side done, rotate to a different side"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] front side done, rotate to a different side"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "[cal] back side done, rotate to a different side"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Welcome to use the BlueSky FlightControl!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Initialization Start:"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Initialization finish! The Drone is ready to fly!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Sensor heating Start:"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Sensor heating Finish!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Sensor heating is Disable!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Sensor check start:"
    },
    {
        .severity = MAV_SEVERITY_ERROR,
        .strings  = "Sensor check error!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Sensor check finish!"
    },
    {
        .severity = MAV_SEVERITY_ERROR,
        .strings  = "Gyro sensor undetected!"
    },
    {
        .severity = MAV_SEVERITY_ERROR,
        .strings  = "Gyro sensor uncalibrated!"
    },
    {
        .severity = MAV_SEVERITY_WARNING,
        .strings  = "Gyro sensor is need to calibrate!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Gyro sensor is OK!"
    },
    {
        .severity = MAV_SEVERITY_ERROR,
        .strings  = "Accel sensor uncalibrated!"
    },
    {
        .severity = MAV_SEVERITY_WARNING,
        .strings  = "Accel sensor is need to calibrate!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Accel sensor is OK!"
    },
    {
        .severity = MAV_SEVERITY_ERROR,
        .strings  = "Mag sensor undetected!"
    },
    {
        .severity = MAV_SEVERITY_ERROR,
        .strings  = "Mag sensor uncalibrated!"
    },
    {
        .severity = MAV_SEVERITY_WARNING,
        .strings  = "Mag sensor is need to calibrate!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Mag sensor is OK!"
    },
    {
        .severity = MAV_SEVERITY_ERROR,
        .strings  = "Baro sensor undetected!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "Baro sensor is OK!"
    },
    {
        .severity = MAV_SEVERITY_ERROR,
        .strings  = "GPS undetected!"
    },
    {
        .severity = MAV_SEVERITY_INFO,
        .strings  = "GPS is OK!"
    },
};


/**********************************************************************************************************
*函 数 名: GetMavNoticeValue
*功能说明: 获取mavlink提示消息内容
*形    参: 消息序号
*返 回 值: 无
**********************************************************************************************************/
MAV_NOTICE_t GetMavNoticeValue(uint8_t noticeNum)
{
    return mavNotice[noticeNum];
}
