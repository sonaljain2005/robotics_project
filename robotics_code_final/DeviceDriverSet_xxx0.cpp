/*
   @Author: ELEGOO
   @Date: 2019-10-22 11:59:09
   @LastEditTime: 2020-06-12 16:36:20
   @LastEditors: Changhua
   @Description: SmartRobot robot tank
   @FilePath:
*/
#include "DeviceDriverSet_xxx0.h"

Servo myservo; // create servo object to control a servo
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Init(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z, 500, 2400); //500: 0 degree  2400: 180 degree
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle); //sets the servo position according to the 90（middle）
  delay(500);

}
#if _Test_DeviceDriverSet
void DeviceDriverSet_Servo::DeviceDriverSet_Servo_Test(void)
{
  for (;;)
  {
    myservo.attach(PIN_Servo_z);
    myservo.write(180);
    delay(500);
    myservo.write(0);
    delay(500);
  }
}
#endif

/*0.17sec/60degree(4.8v)*/
//void DeviceDriverSet_Servo::DeviceDriverSet_Servo_control(unsigned int Position_angle)
//{
//  myservo.attach(PIN_Servo_z);
//  myservo.write(Position_angle);
//  delay(450);
//  myservo.detach();
//}

void DeviceDriverSet_Servo::DeviceDriverSet_Servo_control(unsigned int Position_angle)
{
  myservo.attach(PIN_Servo_z);
  myservo.write(Position_angle);
}

/*Motor control*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Init(void)
{
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
}

#if _Test_DeviceDriverSet
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_Test(void)
{
  digitalWrite(PIN_Motor_AIN_1, LOW);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, HIGH);
  analogWrite(PIN_Motor_PWMB, 100);

  delay(3000);
  digitalWrite(PIN_Motor_AIN_1, HIGH);
  analogWrite(PIN_Motor_PWMA, 100);
  digitalWrite(PIN_Motor_BIN_1, LOW);
  analogWrite(PIN_Motor_PWMB, 100);
  delay(3000);
}
#endif

/*
  Motor_control：AB / 方向、速度
*/
/*
  Motor_control：AB / 方向、速度
*/
void DeviceDriverSet_Motor::DeviceDriverSet_Motor_control(boolean direction_A, uint8_t speed_A, //A组电机参数
    boolean direction_B, uint8_t speed_B, //B组电机参数
    boolean controlED                     //AB使能允许 true
                                                         )                                     //电机控制
{

  if (controlED == control_enable) //使能允许？
  {
    digitalWrite(PIN_Motor_STBY, HIGH);
    { //A...Right

      switch (direction_A) //方向控制
      {
        case direction_just:
          digitalWrite(PIN_Motor_AIN_1, HIGH);
          analogWrite(PIN_Motor_PWMA, speed_A);
          break;
        case direction_back:

          digitalWrite(PIN_Motor_AIN_1, LOW);
          analogWrite(PIN_Motor_PWMA, speed_A);
          break;
        case direction_void:
          analogWrite(PIN_Motor_PWMA, 0);
          digitalWrite(PIN_Motor_STBY, LOW);
          break;
        default:
          analogWrite(PIN_Motor_PWMA, 0);
          digitalWrite(PIN_Motor_STBY, LOW);
          break;
      }
    }

    { //B...Left
      switch (direction_B)
      {
        case direction_just:
          digitalWrite(PIN_Motor_BIN_1, HIGH);

          analogWrite(PIN_Motor_PWMB, speed_B);
          break;
        case direction_back:
          digitalWrite(PIN_Motor_BIN_1, LOW);
          analogWrite(PIN_Motor_PWMB, speed_B);
          break;
        case direction_void:
          analogWrite(PIN_Motor_PWMB, 0);
          digitalWrite(PIN_Motor_STBY, LOW);
          break;
        default:
          analogWrite(PIN_Motor_PWMB, 0);
          digitalWrite(PIN_Motor_STBY, LOW);
          break;
      }
    }
  }
  else
  {
    digitalWrite(PIN_Motor_STBY, LOW);
    return;
  }
}


/*INFRARED*/
//#include <NewPing.h>
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
// IR_PIN_L/R INPUT takes information from a digital pin and determines if the sensor has gone over an edge
void DeviceDriverSet_IR::DeviceDriverSet_IR_Init(void)
{
  // Edge detection pins signal set
  pinMode(US_EDGE_LEFT_ECHO, INPUT);
  pinMode(US_EDGE_LEFT_TRIG, OUTPUT);
  pinMode(US_EDGE_RIGHT_ECHO, INPUT);
  pinMode(US_EDGE_RIGHT_TRIG, OUTPUT);
  
  // Obj US detection pins signal set
  pinMode(US_OBJ_LEFT_ECHO, INPUT);
  pinMode(US_OBJ_LEFT_TRIG, OUTPUT);
  pinMode(US_OBJ_RIGHT_ECHO, INPUT);
  pinMode(US_OBJ_RIGHT_TRIG, OUTPUT);
  pinMode(US_OBJ_MID_ECHO, INPUT);
  pinMode(US_OBJ_MID_TRIG, OUTPUT);
  
  // Flame detection pins signal set
//  pinMode(FD_FRONT_LONG, INPUT);
//  pinMode(FD_FRONT_SHORT, INPUT);
//  pinMode(FD_FRONT_RIGHT, INPUT);
//  pinMode(FD_FRONT_LEFT, INPUT);

  // Laser pin signal set
  pinMode(Laser_Gun, OUTPUT);
}

void DeviceDriverSet_IR::DeviceDriverSet_Laser_Toggle(bool Laser_On){
  if (Laser_On){
    digitalWrite(Laser_Gun, HIGH);
  }
  else{
    digitalWrite(Laser_Gun, LOW);
  }
}

void DeviceDriverSet_IR::DeviceDriverSet_US__EDGE_LEFT_Get(uint16_t *US_EDGE_LEFT /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(US_EDGE_LEFT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_EDGE_LEFT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_EDGE_LEFT_TRIG, LOW);
  tempda_x = ((unsigned int)pulseIn(US_EDGE_LEFT_ECHO, HIGH) / 58);
  *US_EDGE_LEFT = tempda_x;
}

void DeviceDriverSet_IR::DeviceDriverSet_US__EDGE_RIGHT_Get(uint16_t *US_EDGE_RIGHT /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(US_EDGE_RIGHT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_EDGE_RIGHT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_EDGE_RIGHT_TRIG, LOW);
  tempda_x = ((unsigned int)pulseIn(US_EDGE_RIGHT_ECHO, HIGH) / 58);
  *US_EDGE_RIGHT = tempda_x;
}

void DeviceDriverSet_IR::DeviceDriverSet_US__OBJ_LEFT_Get(uint16_t *US_OBJ_LEFT /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(US_OBJ_LEFT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OBJ_LEFT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OBJ_LEFT_TRIG, LOW);
  tempda_x = ((unsigned int)pulseIn(US_OBJ_LEFT_ECHO, HIGH) / 58);
  *US_OBJ_LEFT = tempda_x;
  // Serial.println(tempda_x);
  // sonar.ping() / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).
}

void DeviceDriverSet_IR::DeviceDriverSet_US__OBJ_RIGHT_Get(uint16_t *US_OBJ_RIGHT /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(US_OBJ_RIGHT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OBJ_RIGHT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OBJ_RIGHT_TRIG, LOW);
  tempda_x = ((unsigned int)pulseIn(US_OBJ_RIGHT_ECHO, HIGH) / 58);
  *US_OBJ_RIGHT = tempda_x;
  // Serial.println(tempda_x);
  // sonar.ping() / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).
}

void DeviceDriverSet_IR::DeviceDriverSet_US__OBJ_MID_Get(uint16_t *US_OBJ_MID /*out*/)
{
  unsigned int tempda_x = 0;
  digitalWrite(US_OBJ_MID_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_OBJ_MID_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_OBJ_MID_TRIG, LOW);
  tempda_x = ((unsigned int)pulseIn(US_OBJ_MID_ECHO, HIGH) / 58);
  *US_OBJ_MID = tempda_x;
  // Serial.println(tempda_x);
  // sonar.ping() / US_ROUNDTRIP_CM; // Send ping, get ping time in microseconds (uS).
}

void DeviceDriverSet_IR::DeviceDriverSet_FD_FRONT_LONG_Get(uint16_t *FD_FRONT_LONG_OUT /*out*/)
{
  *FD_FRONT_LONG_OUT = digitalRead(FD_FRONT_LONG);
}

// void DeviceDriverSet_IR::DeviceDriverSet_FD_FRONT_RIGHT_Get(uint16_t *FD_FRONT_RIGHT_OUT /*out*/)
// {
//   *FD_FRONT_RIGHT_OUT = digitalRead(FD_FRONT_RIGHT);
// }

// void DeviceDriverSet_IR::DeviceDriverSet_FD_FRONT_LEFT_Get(uint16_t *FD_FRONT_LEFT_OUT /*out*/)
// {
//   *FD_FRONT_LEFT_OUT = digitalRead(FD_FRONT_LEFT);
// }

bool DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_Init(void)
{
  pinMode(PIN_ITR20001xxxL, INPUT);
  pinMode(PIN_ITR20001xxxM, INPUT);
  pinMode(PIN_ITR20001xxxR, INPUT);
  return false;
}

float DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_L(void)
{
  return analogRead(PIN_ITR20001xxxL);
}
float DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_M(void)
{
  return analogRead(PIN_ITR20001xxxM);
}
float DeviceDriverSet_ITR20001::DeviceDriverSet_ITR20001_getAnaloguexxx_R(void)
{
  return analogRead(PIN_ITR20001xxxR);
}
