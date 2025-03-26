#include <Wire.h>         //调用IIC库函数
#include "MH_TCS34725.h"  //调用颜色识别传感器库函数
#include "gyro.h"

#ifdef __AVR__
#include <avr/power.h>
#endif
MH_TCS34725 tcs = MH_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);  //设置颜色传感器采样周期50毫秒
//针脚值
#define pin_leftgray 16
#define pin_rightgray 17
#define gray_left 450  //白天灰度阈值
#define gray_right 300
//#define grayvalue 450  //晚上灰度阈值

#define pin_leftlow 6
#define pin_lefthigh 5

#define pin_rightlow 10
#define pin_righthigh 9

#define pin_redlight 14

//电机电压值
#define 0 0
#define trace_forward 160    //直行电压
#define trace_turn_back 165  //转弯反转轮
#define trace_turn_forward 235  //转弯正向轮
#define slope_up 200   //斜坡值
#define stage_up 200   //上台阶
#define stage_reverse 50     //反向
#define slope_top 110       //斜坡顶端
#define stage_top 80
#define stage_down 50  //下坡值
#define modify 25
#define slow_value 80
#define slope_down 40
int target_color = 1;
bool flag_right = false;
bool flag_left = false;
int current_color[3];
int mode = 0;  //当前所在关卡


//灰度检查
int CheckGray() {
  int leftvalue = analogRead(pin_leftgray);
  int rightvalue = analogRead(pin_rightgray);
  if (leftvalue > gray_left && rightvalue > gray_right) return 0;    //正常直行
  if (leftvalue <= gray_left && rightvalue <= gray_right) return 1;  //黑线or爬坡
  if (leftvalue > gray_left && rightvalue <= gray_right) return 2;   //右转
  if (leftvalue <= gray_left && rightvalue > gray_right) return 3;   //左转
}




void StopCar() {
  flag_right = false;
  flag_left = false;

  analogWrite(pin_leftlow, 0);
  analogWrite(pin_lefthigh, 0);

  analogWrite(pin_rightlow, 0);
  analogWrite(pin_righthigh, 0);
}

//前进
void forward() {
  flag_right = false;
  flag_left = false;

  analogWrite(pin_leftlow, 0);
  analogWrite(pin_lefthigh, trace_forward);

  analogWrite(pin_rightlow, 0);
  analogWrite(pin_righthigh, trace_forward);
}

void readyforward() {

  flag_right = false;
  flag_left = false;

  analogWrite(pin_leftlow, 0);
  analogWrite(pin_lefthigh, slow_value);

  analogWrite(pin_rightlow, 0);
  analogWrite(pin_righthigh, slow_value);
}

//左转
void TurnLeft() {
  flag_right = false;
  flag_left = true;
  int back_gap = trace_turn_back / 10;
  int slow_gap = -trace_forward / 10;
  int l_high_val = trace_forward;
  int l_low_val = 0;

  for (int i = 0; i < 10; i++) {
    l_low_val += back_gap;
    l_high_val += slow_gap;
    analogWrite(pin_lefthigh, l_high_val + modify * 2);
    analogWrite(pin_leftlow, l_low_val);
    analogWrite(pin_righthigh, trace_turn_forward);
    delay(5);
  }
}

//右转
void TurnRight() {
  flag_right = true;
  flag_left = false;
  int back_gap = trace_turn_back / 10;
  int slow_gap = -trace_forward / 10;
  int r_low_val = 0;
  int r_high_val = trace_forward;

  for (int i = 0; i < 10; i++) {
    r_low_val += back_gap;
    r_high_val += slow_gap;
    analogWrite(pin_righthigh, r_high_val);
    analogWrite(pin_rightlow, r_low_val);
    analogWrite(pin_lefthigh, trace_turn_forward);
    delay(5);
  }
}

//准备爬坡
void readyforclim() {
  int count = 10;
  while (count--) {
    mydelay(50);
    switch (CheckGray()) {
      case 0: readyforward(); break;
      case 1: readyforward(); break;
      case 2: TurnRight(); break;
      case 3: TurnLeft(); break;
    }
    mydelay(100);
    StopCar();
  }
  //forward();
}

void BackForward() {
  flag_right = false;
  flag_left = false;

  analogWrite(pin_leftlow, stage_down + 20);
  analogWrite(pin_lefthigh, 0);

  analogWrite(pin_rightlow, stage_down + 20);
  analogWrite(pin_righthigh, 0);
}

void StageForward(int value) {
  analogWrite(pin_leftlow, 0);
  analogWrite(pin_lefthigh, value);
  analogWrite(pin_rightlow, 0);
  analogWrite(pin_righthigh, value);
  mydelay(100);
  // analogWrite(pin_leftlow, 0);
  // analogWrite(pin_lefthigh, stage_reverse);
  // analogWrite(pin_rightlow, 0);
  // analogWrite(pin_righthigh, stage_reverse);
  // mydelay(10);
  adjust_servo();
}

void StageForward(int value) {
  analogWrite(pin_leftlow, 0);
  analogWrite(pin_lefthigh, value);
  analogWrite(pin_rightlow, 0);
  analogWrite(pin_righthigh, value);
  mydelay(100);
  adjust_servo();
  float diff = cal_angle();
  if (diff > 10) {
    analogWrite(pin_lefthigh, 0);
    analogWrite(pin_leftlow, stage_down);
    analogWrite(pin_righthigh, 0);
    analogWrite(pin_rightlow, stage_down);
  }
}

//爬斜坡
void stage() {
  //首先暂停
  //ReadGyroData();

  StopCar();
  //delaygyro();
  mydelay(1000);
  //调整一下

  BackForward();
  mydelay(500);
  readyforclim();
  StopCar();
  mydelay(1000);
  //地上
  while (CheckGray() != 1) {
    adjust_servo();
    StageForward(stage_up);
    mydelay(20);   //在前轮上冲时，留足够时间冲上第一个台阶，防止小车退回去使diff=0直接进入下坡阶段 原来是:mydelay(20)
  }
  //前轮抬起
  int count = 0;
  float diff = cal_angle();  //陀螺仪角度差值绝对值
  while (diff > 8) {
    adjust_servo();
    StageForward(stage_up);
    mydelay(20);
    diff=cal_angle();
  }
  StopCar();
  for(int i=0;i<50;i++)
  {
    adjust_servo();
    mydelay(20);
  }
  while(diff<=8)
  {
    StageForward(stage_top);
    adjust_servo();
    mydelay(200);
  }
  while(diff>8||CheckGray() == 1)
  {
    StageForward(downstage);
    delay(500);
    StopCar();
    adjust_servo();
    mydelay(800);
  }
  //开始爬第一个台阶
  // while (count != 1) {
  //   adjust_servo();
  //   StageForward(stage_up);
  //   mydelay(20);
  //   if (count == 0 && angle_servo_up < 100 && angle_servo_up > 80) count++;
  //   //if(count==1&&angle_servo_up < 80)count++
  // }
  // delay(1000);
  // //上到了第一个台阶
  // while (angle_servo_up < 100 && angle_servo_up > 80) {
  //   StageForward(stage_up);
  //   adjust_servo();
  //   mydelay(20);
  // }
  // count = 0;
  // //开始爬第二个台阶
  // while (count != 1) {
  //   adjust_servo();
  //   StageForward(stage_up);
  //   mydelay(20);
  //   if (count == 0 && angle_servo_up <100 && angle_servo_up > 80) count++;
  //   //if(count==1&&angle_servo_up < 80)count++
  // }
  // delay(1000);
  // StopCar();
  // delay(1000);
  // int down_count = 0;
  // while (down_count != 1) {
  //   StageForward(stage_top);
  //   adjust_servo();
  //   mydelay(20);
  //   if (down_count == 0 && angle_servo_up > 100) down_count++;
  // }
  //   while ((angle_servo_up<80||angle_servo_up>100)) {
  //   StageForward(downstage);
  //   delay(200);
  //   StopCar();
  //   adjust_servo();
  //   mydelay(800);
  // }
  // while (CheckGray() == 1 && (angle_servo_up<80||angle_servo_up>100)) {
  //   StageForward(downstage);
  //   delay(200);
  //   StopCar();
  //   adjust_servo();
  //   mydelay(800);
  // }
  mode++;
  mydelay(2000);
  //结束
}

void ClimbSlope() {
  //首先暂停
  StopCar();
  delay(1000);
  //调整一下
  BackForward();
  delay(500);
  readyforclim();
  //delay(1000);
  //爬坡发力
  //adjust_servo();

  analogWrite(pin_leftlow, 0);
  analogWrite(pin_lefthigh, slope_up + modify);
  analogWrite(pin_rightlow, 0);
  analogWrite(pin_righthigh, slope_up);

  while (CheckGray() != 1) {
    adjust_servo();
    analogWrite(pin_leftlow, 0);
    analogWrite(pin_lefthigh, slope_up + modify);
    analogWrite(pin_rightlow, 0);
    analogWrite(pin_righthigh, slope_up);
  }
  //开始下坡
  while (true) {
    adjust_servo();
    if (angle_servo_up < 80) {
      Serial.println("mode1");
      analogWrite(pin_leftlow, 0);
      analogWrite(pin_lefthigh, slope_up + modify);
      analogWrite(pin_rightlow, 0);
      analogWrite(pin_righthigh, slope_up);
    } else if (angle_servo_up > 100) {
      Serial.println("mode2");
      analogWrite(pin_leftlow, 0);
      analogWrite(pin_lefthigh, slope_down + modify);
      analogWrite(pin_rightlow, 0);
      analogWrite(pin_righthigh, slope_down);
    } else {
      Serial.println("mode3");
      analogWrite(pin_leftlow, 0);
      analogWrite(pin_lefthigh, slope_top + modify);
      analogWrite(pin_rightlow, 0);
      analogWrite(pin_righthigh, slope_top);
    }
    mydelay(10);
    if (CheckGray() != 1) {
      adjust_servo();
      for (int i = 0; i < 25; i++) {
        mydelay(20);
        adjust_servo();
      }
      mode++;
      return;
    }
  }
}

void getclourinfo() {
  //Serial.println("ok");
  uint16_t clear, red, green, blue;          //分别定义用于存储红、绿、蓝三色值变量
  tcs.getRGBC(&red, &green, &blue, &clear);  //将原始R/G/B值转换为色温（以度为单位）
  tcs.lock();                                //禁用中断（可省略）
  uint32_t sum = clear;
  float r, g, b;
  r = red;
  r /= sum;

  g = green;
  g /= sum;

  b = blue;
  b /= sum;

  r *= 256;
  g *= 256;
  b *= 256;
  current_color[0] = r;
  current_color[1] = g;
  current_color[2] = b;
  Serial.print("\t");  ////////////////////////////////////////////

  Serial.print((int)r);
  Serial.print("\t");  // 在串口中分别打印

  Serial.print((int)g);
  Serial.print("\t");  // 红、绿、蓝三色

  Serial.print((int)b);  // 值

  Serial.println();  //////////////////////////////////////////////
}

void slowforward() {

  flag_right = false;
  flag_left = false;

  analogWrite(pin_leftlow, 0);
  analogWrite(pin_lefthigh, slow_value);

  analogWrite(pin_rightlow, 0);
  analogWrite(pin_righthigh, slow_value);
}

bool checkcolor() {
  int max = current_color[0];
  int temp = 0;
  for (int i = 1; i < 3; i++) {
    if (current_color[i] > max) {
      max = current_color[i];
      temp = i;
    }
  }
  if (temp == target_color) return true;
  return false;
}

void testclour() {
  StopCar();
  mydelay(1000);
  //移动板子
  slowforward();
  while (digitalRead(14)) {
    mydelay(50);
  }
  StopCar();
  mydelay(1000);
  tcs.begin();
  getclourinfo();
  int max = current_color[0];
  int temp = 0;
  for (int i = 1; i < 3; i++) {
    if (current_color[i] > max) {
      max = current_color[i];
      temp = i;
    }
  }


  target_color = temp;
  slowforward();
  while (!digitalRead(14)) {
    slowforward();
    delay(50);
  }
  // mydelay(1500);
  while (true) {
    slowforward();
    while (digitalRead(14)) {
      delay(50);
    }
    StopCar();
    delay(500);
    tcs.begin();
    getclourinfo();

    if (checkcolor()) {
      slowforward();
      while (!digitalRead(14)) {
        delay(50);
      }
      delay(500);
      StopCar();
      //

      //
      pourball();
      break;
    } else {
      while (!digitalRead(14)) {
        slowforward();
        delay(50);
      }
    }
  }

  mode++;
}


void blackline() {
  //delay(100);
  StopCar();
  delay(100);

  BackForward();
  delay(100);

  //if ((mode==1||mode==2)&&checkisturning()) return;
  if (mode == 0) ClimbSlope();
  else if (mode == 1) stage();
  else if (mode == 2) testclour();
}
