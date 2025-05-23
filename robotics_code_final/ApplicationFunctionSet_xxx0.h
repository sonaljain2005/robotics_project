/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-19 15:46:13
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#ifndef _ApplicationFunctionSet_xxx0_H_
#define _ApplicationFunctionSet_xxx0_H_

#include <arduino.h>

class ApplicationFunctionSet
{
public:
  void ApplicationFunctionSet_Init(void);
  void ApplicationFunctionSet_Obstacle(void);           //避障
  void ApplicationFunctionSet_SensorDataUpdate(void);
  void ApplicationFunctionSet_SerialPortDataAnalysis(void);
  
private:
  volatile uint16_t UltrasoundData_mm; //超声波数据
  volatile uint16_t UltrasoundData_cm; //超声波数据
  boolean UltrasoundDetectionStatus = false;

  // Line Tracking
  volatile float TrackingData_L;
  volatile float TrackingData_M;
  volatile float TrackingData_R;
  boolean TrackingDetectionStatus_R = false;
  boolean TrackingDetectionStatus_M = false;
  boolean TrackingDetectionStatus_L = false;
public:
  //If the value read by the photoelectric sensor is within 250 ~ 850,
  //the photoelectric sensor is in the black line.
  uint16_t TrackingDetection_S = 250;
  uint16_t TrackingDetection_E = 850;
  //This is the minimum value obtained by the photoelectric sensor
  //if the car leaves the ground
  uint16_t TrackingDetection_V = 950;
// END LINE TRACKING
public:
  boolean Car_LeaveTheGround = true;
  const int ObstacleDetection = 20;
};
extern ApplicationFunctionSet Application_FunctionSet;
#endif
