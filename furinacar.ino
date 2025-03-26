#include "trailing.h"



void setup() {

  pinMode(pin_leftgray, INPUT);   //左灰度
  pinMode(pin_rightgray, INPUT);  //右灰度

  pinMode(pin_leftlow, OUTPUT);  //左轮
  pinMode(pin_lefthigh, OUTPUT);

  pinMode(pin_rightlow, OUTPUT);  //右轮
  pinMode(pin_righthigh, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(pin_redlight, INPUT);  //近红外

  mode = 0;
  currentservo.detach();
  currentservo.attach(11);
  currentservo.write(180);
  delay(1000);
  currentservo.detach();
  currentservo.attach(7);
  currentservo.write(90);
  delay(1000);
  restartSerial();
  //Serial.begin(115200);

  while (true) {
    delaygyro();
    if (angle[0] > -130 && angle[0] < 130) {
      mydelay(15);
      continue;
    } else {
      angle_init_gyro = angle[0];
      break;
    }
  }
  forward();
}

void loop() {

  // delay(30);
  // Serial.print("left:");
  // Serial.print(analogRead(pin_leftgray));
  //   Serial.print("right:");
  // Serial.println(analogRead(pin_rightgray));
  //adjust_servo();
  //ReadGyroData();
  // delay_read(100);

  mydelay(20);
  if(!flag_right&&!flag_left)adjust_servo();

  switch (CheckGray()) {
    case 0: forward(); break;
    case 1: blackline(); break;
    case 2:
      if(!flag_right)TurnRight();
      break;
    case 3:
      if(!flag_left)TurnLeft();
      break;
  }
}
