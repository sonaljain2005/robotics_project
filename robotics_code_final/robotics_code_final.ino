/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:55:26
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */

// Edit Summary:
//  - DeviceDriverSet_xxx.h: Set PIN_Servo_z to 53
//  - DeviceDriverSet_xxx0.cpp: Updated DeviceDriverSet_Servo::DeviceDriverSet_Servo_control function
//  - ApplicationFunctionSet_xxx0.cpp
//    - Commented out lines 346-350
//    - Added servo functionality on lines 613-620
#include <avr/wdt.h>
#include "DeviceDriverSet_xxx0.h"
#include "ApplicationFunctionSet_xxx0.h"

// uint16_t xJoy = 0;
// uint16_t yJoy = 0;
void setup()
{
  
  // put your setup code here, to run once:
  Application_FunctionSet.ApplicationFunctionSet_Init();
}

void loop()
{
  Application_FunctionSet.ApplicationFunctionSet_Obstacle();
}
