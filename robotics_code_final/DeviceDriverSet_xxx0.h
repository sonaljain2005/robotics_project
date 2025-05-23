/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-12 14:45:27
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#ifndef _DeviceDriverSet_xxx0_H_
#define _DeviceDriverSet_xxx0_H_
#include <arduino.h>

/*Motor*/
class DeviceDriverSet_Motor
{
public:
  void DeviceDriverSet_Motor_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Motor_Test(void);
#endif
  void DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //A组电机参数
                                     boolean direction_B, uint8_t speed_B, //B组电机参数
                                     boolean controlED                     //AB使能允许 true
  );                                                                       //电机控制
private:
// Original Pins
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

public:
#define speed_Max 255
#define direction_just true
#define direction_back false
#define direction_void 3

#define Duration_enable true
#define Duration_disable false
#define control_enable true
#define control_disable false
};
/*INFRARED*/

//#include <NewPing.h>
class DeviceDriverSet_IR
{
public:
  void DeviceDriverSet_IR_Init(void);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_ULTRASONIC_Test(void);
#endif

void DeviceDriverSet_US__EDGE_LEFT_Get(uint16_t *US_EDGE_LEFT /*out*/);
void DeviceDriverSet_US__EDGE_RIGHT_Get(uint16_t *US_EDGE_RIGHT /*out*/);

void DeviceDriverSet_US__OBJ_LEFT_Get(uint16_t *US_OBJ_LEFT /*out*/);
void DeviceDriverSet_US__OBJ_RIGHT_Get(uint16_t *US_OBJ_RIGHT /*out*/);
void DeviceDriverSet_US__OBJ_MID_Get(uint16_t *US_OBJ_MID /*out*/);

void DeviceDriverSet_FD_FRONT_LONG_Get(uint16_t *FD_FRONT_LONG_OUT /*out*/);
//void DeviceDriverSet_FD_FRONT_SHORT_Get(uint16_t *FD_FRONT_SHORT_OUT /*out*/);
void DeviceDriverSet_FD_FRONT_RIGHT_Get(uint16_t *FD_FRONT_RIGHT_OUT /*out*/);
void DeviceDriverSet_FD_FRONT_LEFT_Get(uint16_t *FD_FRONT_LEFT_OUT /*out*/);

void DeviceDriverSet_Laser_Toggle(bool Laser_On);

private:
// Edge detection pins US
#define US_EDGE_LEFT_ECHO 31
#define US_EDGE_LEFT_TRIG 30
#define US_EDGE_RIGHT_ECHO 28 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define US_EDGE_RIGHT_TRIG 29 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
// US Object detection pins 
#define US_OBJ_LEFT_ECHO 23
#define US_OBJ_LEFT_TRIG 22
#define US_OBJ_RIGHT_ECHO 26
#define US_OBJ_RIGHT_TRIG 27
#define US_OBJ_MID_ECHO 12
#define US_OBJ_MID_TRIG 13
// Flame detector pins 
#define FD_FRONT_LONG 39
//#define FD_FRONT_SHORT 22
// #define FD_FRONT_RIGHT 51
// #define FD_FRONT_LEFT 50
// Laser "gun" pin
#define Laser_Gun 45

};

/*ITR20001 Detection*/
class DeviceDriverSet_ITR20001
{
public:
  bool DeviceDriverSet_ITR20001_Init(void);
  float DeviceDriverSet_ITR20001_getAnaloguexxx_L(void);
  float DeviceDriverSet_ITR20001_getAnaloguexxx_M(void);
  float DeviceDriverSet_ITR20001_getAnaloguexxx_R(void);
  #if _Test_DeviceDriverSet
    void DeviceDriverSet_ITR20001_Test(void);
  #endif
  private:
  #define PIN_ITR20001xxxL A2
  #define PIN_ITR20001xxxM A1
  #define PIN_ITR20001xxxR A0
};

/*Servo*/
#include <Servo.h>
class DeviceDriverSet_Servo
{
public:
  void DeviceDriverSet_Servo_Init(unsigned int Position_angle);
#if _Test_DeviceDriverSet
  void DeviceDriverSet_Servo_Test(void);
#endif
  void DeviceDriverSet_Servo_control(unsigned int Position_angle);

private:
#define PIN_Servo_z 53
};

#endif
