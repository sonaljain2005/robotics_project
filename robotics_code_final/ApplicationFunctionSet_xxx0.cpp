/*
 * @Author: ELEGOO
 * @Date: 2019-10-22 11:59:09
 * @LastEditTime: 2020-06-28 14:10:45
 * @LastEditors: Changhua
 * @Description: SmartRobot robot tank
 * @FilePath: 
 */
#include <hardwareSerial.h>
#include <stdio.h>
#include <string.h>
#include "ApplicationFunctionSet_xxx0.h"
#include "DeviceDriverSet_xxx0.h"

#include "ArduinoJson-v6.11.1.h" //ArduinoJson
#include "MPU6050_getdata.h"

#define _is_print 1
#define _Test_print 0

ApplicationFunctionSet Application_FunctionSet;

/*硬件设备成员对象序列*/
MPU6050_getdata AppMPU6050getdata;
DeviceDriverSet_Motor AppMotor;
DeviceDriverSet_IR AppIR;
DeviceDriverSet_ITR20001 AppITR20001;
DeviceDriverSet_Servo AppServo;

/*f(x) int */
static boolean
function_xxx(long x, long s, long e) //f(x)
{
  if (s <= x && x <= e)
    return true;
  else
    return false;
}

/*运动方向控制序列*/
enum SmartRobotCarMotionControl
{
  Forward,       //(1)
  Backward,      //(2)
  Left,          //(3)
  Right,         //(4)
  LeftForward,   //(5)
  LeftBackward,  //(6)
  RightForward,  //(7)
  RightBackward, //(8)
  stop_it,        //(9)
  FlipTurn_Clockwise, //10
  FlipTurn_CounterClockwise //11
};               //direction方向:（1）、（2）、 （3）、（4）、（5）、（6）

/*模式控制序列*/
enum SmartRobotCarFunctionalModel
{
  Standby_mode,           /*空闲模式*/
  TraceBased_mode,        /*循迹模式*/
  ObstacleAvoidance_mode, /*避障模式*/
  Follow_mode,            /*跟随模式*/
  Rocker_mode,            /*摇杆模式*/
};

/*控制管理成员*/
struct Application_xxx
{
  SmartRobotCarMotionControl Motion_Control;
  SmartRobotCarFunctionalModel Functional_Mode;
  unsigned long CMD_CarControl_Millis;
  unsigned long CMD_LightingControl_Millis;
};
Application_xxx Application_SmartRobotCarxxx0;;

bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void);
void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit);
void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed);

void ApplicationFunctionSet::ApplicationFunctionSet_Init(void)
{
  bool res_error = true;
  Serial.begin(9600);
  AppMotor.DeviceDriverSet_Motor_Init();
  AppIR.DeviceDriverSet_IR_Init();
  res_error = AppMPU6050getdata.MPU6050_dveInit();
  AppMPU6050getdata.MPU6050_calibration();

  AppITR20001.DeviceDriverSet_ITR20001_Init();

  while (Serial.read() >= 0)
  {
    /*清空串口缓存...*/
  }
  delay(2000);
  Application_SmartRobotCarxxx0.Functional_Mode = ObstacleAvoidance_mode;
  
}


/*
  直线运动控制：
  direction：方向选择 前/后
  directionRecord：方向记录（作用于首次进入该函数时更新方向位置数据，即:yaw偏航）
  speed：输入速度 （0--255）
  Kp：位置误差放大比例常数项（提高位置回复状态的反映，输入时根据不同的运动工作模式进行修改）
  UpperLimit：最大输出控制量上限
*/
static void ApplicationFunctionSet_SmartRobotCarLinearMotionControl(SmartRobotCarMotionControl direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit)
{
  static float Yaw; //偏航
  static float yaw_So = 0;
  static uint8_t en = 110;
  static unsigned long is_time;
  if (en != directionRecord || millis() - is_time > 10)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    AppMPU6050getdata.MPU6050_dveGetEulerAngles(&Yaw);
    is_time = millis();
  }
  if (direction == FlipTurn_Clockwise)
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable);
    return;
  }

  if (direction == FlipTurn_CounterClockwise)
  {
        AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable);
    return;
  }

  //if (en != directionRecord)
  if (en != directionRecord || Application_FunctionSet.Car_LeaveTheGround == false)
  {
    en = directionRecord;
    yaw_So = Yaw;
  }
  //加入比例常数Kp
  int R = (Yaw - yaw_So) * Kp + speed;
  if (R > UpperLimit)
  {
    R = UpperLimit;
  }
  else if (R < 10)
  {
    R = 10;
  }
  int L = (yaw_So - Yaw) * Kp + speed;
  if (L > UpperLimit)
  {
    L = UpperLimit;
  }
  else if (L < 10)
  {
    L = 10;
  }
  // i dont use L and R, used speed instead cause it was messing up
  if (direction == Forward) //前进
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed, // lol
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable);
  }
  else if (direction == Backward) //后退
  {
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable);
  }
}
/*
  运动控制:
  1# direction方向:前行（1）、后退（2）、 左前（3）、右前（4）、后左（5）、后右（6）
  2# speed速度(0--255)
*/
static void ApplicationFunctionSet_SmartRobotCarMotionControl(SmartRobotCarMotionControl direction, uint8_t is_speed)
{
  ApplicationFunctionSet Application_FunctionSet;
  static uint8_t directionRecord = 0;
  uint8_t Kp, UpperLimit;
  uint8_t speed = is_speed;


    Kp = 2;
    // UpperLimit = 180;
    UpperLimit = 100;

  switch (direction)
  {
  case /* constant-expression */
      Forward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                             /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //前进时进入方向位置逼近控制环处理
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Forward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 1;
    }

    break;
  case /* constant-expression */ Backward:
    /* code */
    if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    {
      AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                             /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    }
    else
    { //后退时进入方向位置逼近控制环处理
      ApplicationFunctionSet_SmartRobotCarLinearMotionControl(Backward, directionRecord, speed, Kp, UpperLimit);
      directionRecord = 2;
    }

    break;
  case /* constant-expression */ FlipTurn_Clockwise:
    /* code */
    
    ApplicationFunctionSet_SmartRobotCarLinearMotionControl(FlipTurn_Clockwise, directionRecord, speed, Kp, UpperLimit);
    directionRecord = 10;
    

    break;
    case /* constant-expression */ FlipTurn_CounterClockwise:
    /* code */
    
    ApplicationFunctionSet_SmartRobotCarLinearMotionControl(FlipTurn_CounterClockwise, directionRecord, speed, Kp, UpperLimit);
    directionRecord = 11;
    

    break;
  case /* constant-expression */ Left:
    /* code */
    directionRecord = 3;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ Right:
    /* code */
    directionRecord = 4;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftForward:
    /* code */
    directionRecord = 5;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ LeftBackward:
    /* code */
    directionRecord = 6;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed / 2, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightForward:
    /* code */
    directionRecord = 7;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_just, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_just, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ RightBackward:
    /* code */
    directionRecord = 8;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_back, /*speed_A*/ speed / 2,
                                           /*direction_B*/ direction_back, /*speed_B*/ speed, /*controlED*/ control_enable); //Motor control
    break;
  case /* constant-expression */ stop_it:
    /* code */
    directionRecord = 9;
    AppMotor.DeviceDriverSet_Motor_control(/*direction_A*/ direction_void, /*speed_A*/ 0,
                                           /*direction_B*/ direction_void, /*speed_B*/ 0, /*controlED*/ control_enable); //Motor control
    break;
  default:
    directionRecord = 11;
    break;
  }
}

static void delay_xxx(uint16_t _ms)
{
  for (unsigned long i = 0; i < _ms; i++)
  {
    delay(1);
  }
}

static bool ApplicationFunctionSet_SmartRobotCarLeaveTheGround(void)
{
if (AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R() >
  Application_FunctionSet.TrackingDetection_V &&
  AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M() >
  Application_FunctionSet.TrackingDetection_V &&
  AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L() >
  Application_FunctionSet.TrackingDetection_V)
  {
    Application_FunctionSet.Car_LeaveTheGround = false;
    return false;
  } else {
    Application_FunctionSet.Car_LeaveTheGround = true;
    return true;
  }
}


/*
  避障功能
*/
bool justWasOnTape = false;
bool scanning = false;
bool targetFound = false;
uint64_t startTime = 0;
uint64_t scanTime = 0;
uint16_t normal_speed = 40;

// Variables for sweeping
int servoPos = 0;           // Current position of the servo
int stepSize = 4;           // How much to move the servo each step
int sweepDelay = 20;        // Time between steps (in milliseconds)
unsigned long lastMoveTime = 0; // Tracks the last time the servo moved
bool sweepingForward = true; // Direction of the sweep
bool sweepingSearchLeft = false;
bool sweepingSearchRight = false;
uint64_t sweepingSearchTime = 0;
int servoOrigin = 90;
uint16_t get_FD_FRONT_LONG_Out;
uint16_t get_FD_FRONT_RIGHT_Out;
uint16_t get_FD_FRONT_LEFT_Out;
uint8_t switc_ctrl = 0;

int scanningRightEndServoPos = 0;
int scanningLeftEndServoPos = 0;

uint16_t get_US_EDGE_Left;
uint16_t get_US_EDGE_Right;

uint16_t get_US_OBJ_Left;
uint16_t get_US_OBJ_Right;
uint16_t get_US_OBJ_Mid;

// laser
unsigned long lastConeSearchTime = 0;
unsigned long lastLaserShootTime = 0;
int laserSearchStep = 0;
bool readyToFire = false;
bool shootingLaser = false;
int laserDelay = 40;
bool coneSearch = false;
int coolDownDelay = 1500; // 1.5s
bool coolDown = false;
unsigned long lastCoolDownTime = 0;
int maxLaserSearchStep = 8;

void ApplicationFunctionSet::ApplicationFunctionSet_Obstacle(void)
{
  static boolean first_is = true;
  if (Application_SmartRobotCarxxx0.Functional_Mode == ObstacleAvoidance_mode)
  {
    
    if (Car_LeaveTheGround == false)
    {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      return;
    }

    AppIR.DeviceDriverSet_US__EDGE_LEFT_Get(&get_US_EDGE_Left /*out*/);
    AppIR.DeviceDriverSet_US__EDGE_RIGHT_Get(&get_US_EDGE_Right /*out*/);

    AppIR.DeviceDriverSet_US__OBJ_RIGHT_Get(&get_US_OBJ_Right /*out*/);
    AppIR.DeviceDriverSet_US__OBJ_LEFT_Get(&get_US_OBJ_Left /*out*/);
    AppIR.DeviceDriverSet_US__OBJ_MID_Get(&get_US_OBJ_Mid /*out*/);

    // -- CUSTOM
    bool edgeOrObstacleDetected = false;
    
    // If the short range detector doesn't see the light then set basking to false so regular functions can continue
//    if (get_US_OBJ_Mid > 12)
//    {
//      basking = false;
//    }
    
    // -- EDGE DETECTION
    if (get_US_EDGE_Left > 12 && get_US_EDGE_Right > 12) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_Clockwise, 100); // new
      delay_xxx(1000); // 1000 means about 180 turn
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // new
      edgeOrObstacleDetected = true;
      justWasOnTape = false;
      Serial.println("US Edge Detection Left & Right");
    } else if (get_US_EDGE_Right > 12) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_CounterClockwise, 100); // new
      delay_xxx(500); // 500 means about 90 turn
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // new
      edgeOrObstacleDetected = true;
      justWasOnTape = false;
      Serial.println("US Edge Detection Right");
    } else if (get_US_EDGE_Left > 12) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_Clockwise, 100); // new
      delay_xxx(500); // 500 means about 90 turn
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); // new
      edgeOrObstacleDetected = true;
      justWasOnTape = false;
      Serial.println("US Edge Detection Left");
    }
    
    // i commented out obstacle detection for now.
    if (get_US_OBJ_Right <= 12 && !sweepingSearchRight && !sweepingSearchLeft){
      if (targetFound) {
        return;
      }
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_CounterClockwise, 100);
      delay_xxx(500);
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      edgeOrObstacleDetected = true;
      justWasOnTape = false;
      Serial.print("US Right Obj Out: ");
      Serial.println(get_US_OBJ_Right);
    } else if (get_US_OBJ_Left <= 12 && !sweepingSearchRight && !sweepingSearchLeft) {
//      if (basking) {
//        return;
//      }
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_Clockwise, 100);
      delay_xxx(500);
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      edgeOrObstacleDetected = true;
      justWasOnTape = false;
      Serial.print("US Left Obj Out: ");
      Serial.println(get_US_OBJ_Left);
    } 
      else if (get_US_OBJ_Mid <= 12) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_Clockwise, 100);
      delay_xxx(500);
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      edgeOrObstacleDetected = true;
      justWasOnTape = false;
      Serial.print("US Mid Obj Out: ");
      Serial.println(get_US_OBJ_Mid);
    }

     // don't bother line tracking if we have bigger fish to fry
     if (edgeOrObstacleDetected) {
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
      return;
    }

    // LINE TRACKING
    static boolean timestamp = true;
    static boolean BlindDetection = true;
    static unsigned long MotorRL_time = 0;
    // if (Application_SmartRobotCarxxx0.Functional_Mode == TraceBased_mode)
    
    if (Car_LeaveTheGround == false) //车子离开地面了？
    {
//      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
//      return;
    }
    float getAnaloguexxx_L = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_L();
    float getAnaloguexxx_M = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_M();
    float getAnaloguexxx_R = AppITR20001.DeviceDriverSet_ITR20001_getAnaloguexxx_R();
    
    if (function_xxx(getAnaloguexxx_L, TrackingDetection_S, TrackingDetection_E) 
    && function_xxx(getAnaloguexxx_R, TrackingDetection_S, TrackingDetection_E) 
    && function_xxx(getAnaloguexxx_M, TrackingDetection_S, TrackingDetection_E)) //不在黑线上的时候。。。
    {
      justWasOnTape = true;
      scanning = false;
      if (timestamp == true) //获取时间戳 timestamp
      {
        timestamp = false;
        MotorRL_time = millis();
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      }
      unsigned long m=millis();
//        Serial.println(m - MotorRL_time);
      /*Blind Detection*/
      // if ((function_xxx((m - MotorRL_time), 0, 200) || function_xxx((m - MotorRL_time), 1600, 2000)) && BlindDetection == true)
      // {
      //   ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 100);
      // }
      if (((function_xxx((m - MotorRL_time), 0, 8000))) && true)
      {
        ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_CounterClockwise, 100);
        delay_xxx(40); // turn just a lil
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); 
      }
      else if ((function_xxx((m - MotorRL_time), 8000, 9000))) // Blind Detection ...s ?
      {
        BlindDetection = false;
        timestamp = true;
        ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
        ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
      }
      
    }
    else if (function_xxx(getAnaloguexxx_M, TrackingDetection_S, TrackingDetection_E))
    {
      justWasOnTape = true;
      scanning = false;
      /*控制左右电机转动：实现匀速直行*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(getAnaloguexxx_R, TrackingDetection_S, TrackingDetection_E))
    {
      justWasOnTape = true;
      scanning = false;
      /*控制左右电机转动：前右*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Right, 80);
      timestamp = true;
      BlindDetection = true;
    }
    else if (function_xxx(getAnaloguexxx_L, TrackingDetection_S, TrackingDetection_E))
    {
      justWasOnTape = true;
      scanning = false;
      /*控制左右电机转动：前左*/
      ApplicationFunctionSet_SmartRobotCarMotionControl(Left, 80);
      timestamp = true;
      BlindDetection = true;
    } else {
     // This code prints out the values of scanning and justWasOnTape
     // They are each boolean values and will output 1 if true, and 0 if false
      if (justWasOnTape) {
        // scanning within this block
        if (!scanning) {
          ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0); 
          // set timestamp
          startTime = millis();
          scanning = true;
        }

        // based on timestamp, scan in various directions
        scanTime = millis() - startTime;
        
        if (scanTime <= 350) {
          ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_Clockwise, 100);
        } else if (scanTime <= 1050) {
          ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_CounterClockwise, 100);
        } else if (scanTime <= 1400) {
          // turn clockwise to middle
          ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_Clockwise, 100);  
        } else {
          scanning = false;
          justWasOnTape = false;
        }
        return;
      }
        if (!sweepingSearchRight && !sweepingSearchLeft)
        {
          ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
          ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
        }
    }
     
    // END LINE TRACKING


    // IN CASE OF COOLDOWN FOR SERVO
    if (coolDown && millis() - lastCoolDownTime >= coolDownDelay) {
      coolDown = false;
    } else if (coolDown) {
      return;
    }


    // If front detection finds beacon -> turn towards beacon and drive forward until basking beacon is active then stop moving
    // If back detection finds beacon -> do a 180 - Servo position (front detection should find beacon) and drive forward until basking then stop moving

    // Check if it's time to move the servo
    if (targetFound)
    {
      // COMMENT OUT THE FOLLOWING LINE IF VERSING OTHER ROBOTS ;)
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    }
    else if (millis() - lastMoveTime >= sweepDelay) {
      // Update the time of the last movement
      lastMoveTime = millis();
  
      // Move the servo in the current direction
      if (sweepingForward) {
        servoPos += stepSize;
        if (servoPos >= 180) { // Reverse direction at the end
          sweepingForward = false;
        }
      } else {
        servoPos -= stepSize;
        if (servoPos <= 0) { // Reverse direction at the start
          sweepingForward = true;
        }
      }
  
      // Write the new position to the servo
      AppServo.DeviceDriverSet_Servo_control(servoPos);
    }

    
    AppIR.DeviceDriverSet_FD_FRONT_LONG_Get(&get_FD_FRONT_LONG_Out);

    if (shootingLaser){
      if (millis() - lastLaserShootTime >= 500) {
        if (get_FD_FRONT_LONG_Out == 0) {
          Serial.println("Missed!");
        } else {
          Serial.println("HEADSHOT!!!!!!!!!");
          delay(2000);
          
          ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
          ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_CounterClockwise, 100);
          delay_xxx(1500);
          ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
        }

        AppIR.DeviceDriverSet_Laser_Toggle(false);
        shootingLaser = false;
        targetFound = false;
      }

      return;
    }

    int leftLimit = 0;
    int rightLimit = 0;
    if (get_FD_FRONT_LONG_Out == 0) {
      int maxCounter = 0;
      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
      targetFound = true;
      while (get_FD_FRONT_LONG_Out == 0){
        AppIR.DeviceDriverSet_FD_FRONT_LONG_Get(&get_FD_FRONT_LONG_Out);
        servoPos -= 2;
        AppServo.DeviceDriverSet_Servo_control(servoPos);
        delay(75);
        maxCounter ++;
        if (maxCounter >= 100) {
          targetFound = false;
          return;
        }
      }
      maxCounter = 0;
      leftLimit = servoPos;

      AppIR.DeviceDriverSet_FD_FRONT_LONG_Get(&get_FD_FRONT_LONG_Out);
      
      while (get_FD_FRONT_LONG_Out == 1) {
        AppIR.DeviceDriverSet_FD_FRONT_LONG_Get(&get_FD_FRONT_LONG_Out);
        servoPos += 2;
        AppServo.DeviceDriverSet_Servo_control(servoPos);
        delay(75);
        maxCounter ++;
        if (maxCounter >= 100) {
          targetFound = false;
          return;
        }
      }
      maxCounter = 0;
      while (get_FD_FRONT_LONG_Out == 0){
        AppIR.DeviceDriverSet_FD_FRONT_LONG_Get(&get_FD_FRONT_LONG_Out);
        servoPos += 2;
        AppServo.DeviceDriverSet_Servo_control(servoPos);
        delay(75);
        maxCounter ++;
        if (maxCounter >= 100) {
          targetFound = false;
          return;
        }
      }
      maxCounter = 0;

      rightLimit = servoPos;
      servoPos = (leftLimit + rightLimit) / 2 - 2;
      AppServo.DeviceDriverSet_Servo_control(servoPos);
      delay(75);

      AppIR.DeviceDriverSet_FD_FRONT_LONG_Get(&get_FD_FRONT_LONG_Out);
      if (get_FD_FRONT_LONG_Out == 0) {
        // shoot da lazor
        shootingLaser = true;
        AppIR.DeviceDriverSet_Laser_Toggle(true);
        lastLaserShootTime = millis();
      } else {
        // play sad sound
        targetFound = false;
      }
    } else {
      targetFound = false;
      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
    }

    // if (millis() - lastConeSearchTime >= sweepDelay) {
    //   lastConeSearchTime = millis();
    //   AppIR.DeviceDriverSet_FD_FRONT_LONG_Get(&get_FD_FRONT_LONG_Out);
    //   AppIR.DeviceDriverSet_FD_FRONT_LEFT_Get(&get_FD_FRONT_LEFT_Out);
    //   AppIR.DeviceDriverSet_FD_FRONT_RIGHT_Get(&get_FD_FRONT_RIGHT_Out);

    //   if (get_FD_FRONT_LONG_Out == 0) {
    //     targetFound = true;
    //     if (get_FD_FRONT_LEFT_Out == 0) { // Middle and left are active
    //       // turn servo right
    //       servoPos += stepSize;
    //       AppServo.DeviceDriverSet_Servo_control(servoPos);
    //     } else if (get_FD_FRONT_RIGHT_Out == 0) { // Middle and right are active
    //       // turn servo left
    //       servoPos -= stepSize;
    //       AppServo.DeviceDriverSet_Servo_control(servoPos);
    //     } else { // Only middle is active
    //       if (shootingLaser) { //already shooting
    //         // servoPos += random(-4, 4); // the design is very human
    //         // AppServo.DeviceDriverSet_Servo_control(servoPos);
    //       } else { // wasn't shooting before
    //         // shoot 
    //         shootingLaser = true;
    //         AppIR.DeviceDriverSet_Laser_Toggle(true);
    //       }
        
    //     }

    //   } else if (get_FD_FRONT_LEFT_Out == 0) {
    //     targetFound = true;
    //     servoPos += stepSize;
    //     AppServo.DeviceDriverSet_Servo_control(servoPos);
    //   } else if (get_FD_FRONT_RIGHT_Out == 0) {
    //     targetFound = true;
    //     servoPos -= stepSize;
    //     AppServo.DeviceDriverSet_Servo_control(servoPos);
    //   } else {
    //     if (shootingLaser) {
    //       Serial.println("HEADSHOT!!!");
    //       targetFound = false;
    //       delay(1000);
    //       shootingLaser = false;
    //       AppIR.DeviceDriverSet_Laser_Toggle(false);
    //       ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    //       ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_CounterClockwise, 100);
    //       delay_xxx(1500);
    //       ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
          
    //     } else {
    //       targetFound = false;
    //       ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    //       ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
    //     }
    //   }
    // }



    //  Flame Detection Logic:
//    - 1 = NOT DETECTING
//    - 0 = DETECTING
//
//    - Two modes for presentation
//      1. When we're presenting solo the robot will stop before performing the following routine
//      2. When we're presenting against a team the robot will keep moving  
//
//      - FD front detected
//        - Set boolean "searching for target" to true
//        - If 30 < servoPos < 150
  //        - Shoot laser
  //        - If we just saw it, then we don't see it anymore once the laser has been fired, headshot boolean = true
  //        - Else if we shoot and can still see the target
  //          - Repeat above routine while moving the servo left and right in larger and larger increments until headshot = true
  //          - range = +- 16 degrees from the origin 
  //          - If we stop seeing target before reaching new shoot position
  //            - Skip this shoot attempt
  //     - If headshot = true
  //        - Play audio
  //        - Turn around 180 degrees 
//       - Else 
//          - Turn towards direction of target
//
//    Laser Firing Code:
//       digitalWrite(laserPin, LOW); // Turns laser off
//       delay(250);
//       digitalWrite(laserPin, HIGH); // Turns laser on 
//       delay(250);

    

    // // IF CONE INCREMENT TIMER IS DONE, GET READY TO FIRE
    // if (targetFound && !shootingLaser && millis() - lastConeSearchTime >= sweepDelay) {
    //   readyToFire = true;
    // }

    // AppIR.DeviceDriverSet_FD_FRONT_LONG_Get(&get_FD_FRONT_LONG_Out);
    // if (get_FD_FRONT_LONG_Out == 0) // IF DETECTING
    // {
    //   targetFound = true;
    //   if (laserSearchStep == 0 && !shootingLaser) {
    //     readyToFire = true;
    //   }

    //   // checking if we missed
    //   if (shootingLaser && millis() - lastLaserShootTime >= laserDelay) {
    //     shootingLaser = false;
    //     readyToFire = false;
    //     AppIR.DeviceDriverSet_Laser_Toggle(false);
    //     Serial.println('We missed, so laser off');

    //     laserSearchStep ++;

    //     // if we miss after all attemps then activate cool down.
    //     if (laserSearchStep >= 5) {
    //       coolDown = true;
    //       lastCoolDownTime = millis();
    //       targetFound = false;
    //       laserSearchStep = 0;
    //       Serial.println('mission failed, entering cooldown mode');
    //     }
        
    //   } else if (readyToFire) {
    //     // OPEN FIRE
    //     ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    //     AppIR.DeviceDriverSet_Laser_Toggle(true);
    //     shootingLaser = true;
    //     if (!shootingLaser) {
    //       lastLaserShootTime = millis();
    //       // Serial.println("Laser on");
    //     }
    //   }
      
    // }
    // else if (shootingLaser) { // NOT DETECTING WHILE SHOOTING
    //   Serial.println("HEADSHOT ;)");
    //   delay(3000);
    //   AppIR.DeviceDriverSet_Laser_Toggle(false);
    //   shootingLaser = false;
    //   targetFound = false;
    //   laserSearchStep = 0;

    //   ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
    //   ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_CounterClockwise, 100);
    //   delay_xxx(2000);
    //   ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
    // }
    // else // NOT DETECTING
    // {
    //   if (targetFound) {
    //     if (laserSearchStep == 0) {
    //       Serial.println("Target lost 😡 ");
    //       targetFound = false;
    //     } else if (laserSearchStep >= maxLaserSearchStep) {
    //       Serial.println("Target lost after completing search");
    //       targetFound = false;
    //       laserSearchStep = 0;

    //     } else {
    //       Serial.print("Missed while in case ");
    //       Serial.println(laserSearchStep);
    //       laserSearchStep ++;
    //     }
    //   }
    // }

    // if (targetFound && !shootingLaser && millis() - lastConeSearchTime >= sweepDelay) {
    //   lastConeSearchTime = millis();
    //   readyToFire = false;
    //   // Scan from -16 to 16 for a total cone of 32 degrees
    //   // switch (laserSearchStep) {
    //   //   case 0:
    //   //     // shoot da lazor straight ahead so no movement needed
    //   //     Serial.println("CASE 0");
    //   //     break;
    //   //   case 1:
    //   //     servoPos += 8;
    //   //     Serial.println("CASE 1");
    //   //     break;
    //   //   case 2:
    //   //     servoPos -= 16;
    //   //     Serial.println("CASE 2");
    //   //     break;
    //   //   case 3:
    //   //     servoPos += 24;
    //   //     Serial.println("CASE 3");
    //   //     break;
    //   //   case 4:
    //   //     servoPos -= 32;
    //   //     Serial.println("CASE 4");
    //   //     break;
    //   //   default:
    //   //     Serial.println("Never should have come here...");
    //   //     break;
    //   // }

    //   switch(laserSearchStep) {
    //             case 0:
    //       // shoot da lazor straight ahead so no movement needed
    //       Serial.println("CASE 0");
    //       break;
    //     case 1:
    //       servoPos += 4;
    //       Serial.println("CASE 1");
    //       break;
    //     case 2:
    //       servoPos -= 8;
    //       Serial.println("CASE 2");
    //       break;
    //     case 3:
    //       servoPos += 12;
    //       Serial.println("CASE 3");
    //       break;
    //     case 4:
    //       servoPos -= 16;
    //       Serial.println("CASE 4");
    //       break;
    //     case 5:
    //       servoPos += 20;
    //       Serial.println("Case 5");
    //       break;
    //     case 6:
    //       servoPos -= 24;
    //       Serial.println("Case 6");
    //       break;
    //     case 7:
    //       servoPos += 28;
    //       Serial.println("Case 7");
    //       break;
    //     case 8:
    //       servoPos -= 32;
    //       Serial.println("Case 8");
    //       break;
    //     default:
    //       Serial.println("Never should have come here...");
    //       break;
    //   }
    //   laserSearchStep++;
    //   AppServo.DeviceDriverSet_Servo_control(servoPos);
    // } else if (targetFound && !shootingLaser) {
    //   // readyToFire = false;
    // }
//    

//    if (millis() - sweepingSearchTime >= 5000){
//      sweepingSearchRight = false;
//      sweepingSearchLeft = false;
//    }
//
//    if (get_FD_FRONT_LONG_Out == 0 && get_US_OBJ_Mid <= 12)
//    {
//      Serial.println("basking atm");
//      ApplicationFunctionSet_SmartRobotCarMotionControl(stop_it, 0);
//      basking = true;
//    }
//    else if (get_FD_FRONT_LONG_Out == 0) 
//    {
//      ApplicationFunctionSet_SmartRobotCarMotionControl(Forward, normal_speed);
//      sweepingSearchRight = false;
//      sweepingSearchLeft = false;
//    }
//    else if (get_FD_FRONT_RIGHT_Out == 0)
//    {
//      ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_Clockwise, 45);
//      sweepingSearchRight = true;
//      sweepingSearchTime = millis();
//    }
//    else if (get_FD_FRONT_LEFT_Out == 0)
//    {
//      ApplicationFunctionSet_SmartRobotCarMotionControl(FlipTurn_CounterClockwise, 45);
//      sweepingSearchLeft = true;
//      sweepingSearchTime = millis();
//    }

    return;

    // -- END CUSTOM

  }
}
