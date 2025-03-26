#include "Timer2ServoPwm.h"
Timer2Servo currentservo;
//Servo upservo;
//Servo downservo;
int angle_servo_up;
unsigned char Re_buf[11], counter = 0;
int sign = 0;
  float a[3], w[3], angle[3], T;
  float angle_init_gyro;



void ReadGyroData() {
  if (sign) {
    sign = 0;
    switch (Re_buf[1]) {
      case 0x51:
        a[0] = (short(Re_buf[3] << 8 | Re_buf[2])) / 32768.0 * 16;
        a[1] = (short(Re_buf[5] << 8 | Re_buf[4])) / 32768.0 * 16;
        a[2] = (short(Re_buf[7] << 8 | Re_buf[6])) / 32768.0 * 16;
        T = (short(Re_buf[9] << 8 | Re_buf[8])) / 340.0 + 36.25;
        break;
      case 0x52:
        w[0] = (short(Re_buf[3] << 8 | Re_buf[2])) / 32768.0 * 2000;
        w[1] = (short(Re_buf[5] << 8 | Re_buf[4])) / 32768.0 * 2000;
        w[2] = (short(Re_buf[7] << 8 | Re_buf[6])) / 32768.0 * 2000;
        T = (short(Re_buf[9] << 8 | Re_buf[8])) / 340.0 + 36.25;
        break;
      case 0x53:
        angle[0] = (short(Re_buf[3] << 8 | Re_buf[2])) / 32768.0 * 180;
        angle[1] = (short(Re_buf[5] << 8 | Re_buf[4])) / 32768.0 * 180;
        angle[2] = (short(Re_buf[7] << 8 | Re_buf[6])) / 32768.0 * 180;
        T = (short(Re_buf[9] << 8 | Re_buf[8])) / 340.0 + 36.25;


        Serial.print("a:");
        Serial.print(a[0]);
        Serial.print(" ");
        Serial.print(a[1]);
        Serial.print(" ");
        Serial.print(a[2]);
        Serial.print(" ");
        Serial.print("w:");
        Serial.print(w[0]);
        Serial.print(" ");
        Serial.print(w[1]);
        Serial.print(" ");
        Serial.print(w[2]);
        Serial.print(" ");
        Serial.print("angle:");
        Serial.print(angle[0]);
        Serial.print(" ");
        Serial.print(angle[1]);
        Serial.print(" ");
        Serial.print(angle[2]);
        Serial.print(" ");
        Serial.print("T:");
        Serial.println(T);

        break;
    }
  }
}
float cal_angle() {
  int diff = 0;
  if (angle[0] > 130) {
    diff = 180 - angle[0];
    if (angle_init_gyro < 0) diff = angle_init_gyro + 180 + diff;
    else diff = diff - (180 - angle_init_gyro);
  } else if (angle[0] < -130) {
    diff = angle[0] + 180;
    if (angle_init_gyro < angle[0]) diff = angle[0] - angle_init_gyro;
    else if (angle_init_gyro > angle[0] && angle_init_gyro < -130) diff = angle_init_gyro - angle[0];
    else diff = diff + (180 - angle_init_gyro);
  } else return 0;
  return diff;
}
void mydelay(int interval) {
  unsigned long previousMillis = millis();
  unsigned long currentMillis = millis();  // 获取当前时间
  // 检查是否达到延时间隔

  while (currentMillis - previousMillis < interval) {
    
    currentMillis = millis();
  }

  // 其他代码可以继续执行
}

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();  // 读取并丢弃串口缓冲区中的数据
  }
}

void restartSerial() {
  Serial.end();          // 关闭串口
  mydelay(5);           // 等待一段时间确保串口已关闭
  clearSerialBuffer();   // 清空串口缓冲区中的数据
  Serial.begin(115200);  // 重新打开串口，设置波特率
}

void delaygyro() {
  restartSerial();
  mydelay(10);
  if (serialEventRun) 
  serialEventRun();
  ReadGyroData();
}

// void delay_read(int interval) {
//   int count = interval / 20;
// myloop:
//   delaygyro();
//   count--;
//   if (count <= 0) return;
//   else goto myloop;
// }


void adjust_servo() {
  delaygyro();
  float diff = cal_angle();
  Serial.println(diff);
  if (diff < 5) {
    angle_servo_up = 90;
    currentservo.write(angle_servo_up);
    return;
  }
  if (angle_init_gyro < 0) {
    if (angle[0] > 130 || angle[0] < angle_init_gyro)
    {
      
      angle_servo_up = 90 - diff;
    }
     
    else 
    {
     
     angle_servo_up = 90 + diff;
    }
  } 
  else {
    if(angle[0]>130&&angle[0]<angle_init_gyro)     
    {
     
     angle_servo_up = 90 - diff;
    }    
    else //if (angle[0] < -130 || angle[0] > angle_init_gyro)
    {
    
      angle_servo_up = 90 + diff;
    }
  }
  if (angle_servo_up >= 50 && angle_servo_up <= 130) currentservo.write(angle_servo_up);
}



void serialEvent() {

  while (Serial.available()) {
    //char inChar = (char)Serial.read(); Serial.print(inChar); //Output Original Data, use this code
    Re_buf[counter] = (unsigned char)Serial.read();
    if (counter == 0 && Re_buf[0] != 0x55) {
      return;  // 第0号数据不是帧头
    }

    counter++;
    if (counter == 11)  // 接收到11个数据
    {
      counter = 0;  // 重新赋值，准备下一帧数据的接收
      sign = 1;
    }
  }
}

void pourball() {
  currentservo.detach();
  currentservo.attach(11);
  currentservo.write(120);
  delay(1000);
  currentservo.detach();
  currentservo.attach(7);
  currentservo.write(150);
  delay(1000);
  currentservo.detach();
  currentservo.attach(7);
  currentservo.write(90);
  delay(1000);

  currentservo.detach();
  currentservo.attach(11);
  currentservo.write(180);
  delay(1000);
  currentservo.detach();
}