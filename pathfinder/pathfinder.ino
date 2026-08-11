/*
 * 高中组.ino — 由 Mixly 文件 "高中组.mix" 转换而来
 * ============================================================
 * 源文件: 高中组.mix (Mixly 3.0, board = Arduino Nano / ATmega328P)
 * 转换日期: 由转换脚本自动生成
 *
 * 说明:
 *   Mixly 的 .mix 文件 = XML 积木块(<xml>) + 配置(<config>) + 生成的
 *   Arduino 代码(<code>, base64 编码)。本文件即 <code> 段解码后的
 *   代码, 唯一修改是把中文标识符改成 ASCII 标识符——因为 Arduino IDE
 *   自带的 avr-gcc 7.3.0 编译器不支持 UTF-8 中文标识符 (会报 stray
 *   character 错误), 而 Mixly 3.0 内置编译器较新所以能编译。
 *
 * 中文标识符 → ASCII 对照表 (与 Mixly 积木里的中文名对应):
 *   速度            -> speed
 *   校正            -> correction
 *   调整方向        -> direction
 *   系统时间        -> systemTime
 *   my_1白~my_4白   -> my_1_white ~ my_4_white
 *   my_1黑~my_4黑   -> my_1_black ~ my_4_black
 *   my_1阈值~my_4阈值 -> my_1_threshold ~ my_4_threshold
 *   __获取阈值      -> getThresholds
 *   ____停止程序    -> stopProgram
 *   __直行          -> goStraight
 *   _后退           -> goBackward
 *   _左转           -> turnLeft
 *   _右转           -> turnRight
 *   _左转圈         -> spinLeft
 *   _右转圈         -> spinRight
 *   转弯遇线停      -> turnUntilLine
 *   巡线找左右路口  -> lineFollowJunction
 *   巡线时间        -> lineFollowTime
 *   RGB_红/绿/蓝    -> RGB_Red / RGB_Green / RGB_Blue
 *   舵机控制_D9     -> servoControl_D9
 *   蜂鸣器报警_D8   -> buzzerAlarm_D8
 *   正反            -> forward
 *   初始速度        -> initSpeed
 *   直行校正（-20-20）-> straightCorrection
 *   闪烁次数        -> blinkCount
 *   间隔时间        -> interval
 *   报警次数        -> alarmCount
 *   时间            -> timeMs
 *   角度(0-180)     -> angle_0_180
 *   1-左#2-右       -> turnDir
 *
 * 程序逻辑 (与积木完全一致): 光电循迹小车比赛程序。
 *   上电 -> 获取阈值(白/黑) -> 巡线 -> 转弯 -> 找路口 -> ... (见 loop())
 *   引脚分配:
 *     D8  蜂鸣器(低有效)   D11/D12/D13 RGB灯(低有效)
 *     D9  舵机(Servo)      D10 按键输入
 *     D2/D4 + PWM D3  左电机 (TB6612)
 *     D5/D7 + PWM D6  右电机 (TB6612)
 *     A0~A3 4路光电传感器
 * ============================================================
 */
#include <Servo.h>

Servo servo_9;
volatile int my_1_white;
volatile int my_2_white;
volatile int my_3_white;
volatile int my_4_white;
volatile int my_1_black;
volatile int my_2_black;
volatile int my_3_black;
volatile int my_4_black;
volatile int my_1_threshold;
volatile int my_2_threshold;
volatile int my_3_threshold;
volatile int my_4_threshold;
volatile int speed;
volatile int correction;
volatile bool direction;
volatile float systemTime;

void getThresholds(bool forward, int initSpeed, int straightCorrection) {
  direction = forward;
  speed = initSpeed;
  correction = straightCorrection;
  while (digitalRead(10) == 1) {
    my_1_white = analogRead(A0);
    my_2_white = analogRead(A1);
    my_3_white = analogRead(A2);
    my_4_white = analogRead(A3);
  }
  for (int i = 1; i <= 3; i = i + (1)) {
    digitalWrite(12, LOW);
    delay(75);
    digitalWrite(12, HIGH);
    delay(75);
  }
  while (digitalRead(10) == 1) {
    my_1_black = analogRead(A0);
    my_2_black = analogRead(A1);
    my_3_black = analogRead(A2);
    my_4_black = analogRead(A3);
  }
  for (int i = 1; i <= 3; i = i + (1)) {
    digitalWrite(12, LOW);
    delay(75);
    digitalWrite(12, HIGH);
    delay(75);
  }
  my_1_threshold = (my_1_white + my_1_black) / 2;
  my_2_threshold = (my_2_white + my_2_black) / 2;
  my_3_threshold = (my_3_white + my_3_black) / 2;
  my_4_threshold = (my_4_white + my_4_black) / 2;
  while (digitalRead(10) == 1) {
  }
}

void stopProgram() {
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  analogWrite(3, 0);
  digitalWrite(5, LOW);
  digitalWrite(7, HIGH);
  analogWrite(6, 0);
  while(true);
}

void RGB_Red(int blinkCount, int interval) {
  for (int i = (1); i <= (blinkCount); i = i + (1)) {
    digitalWrite(12, LOW);
    delay(interval);
    digitalWrite(12, HIGH);
    delay(100);
  }
}

void RGB_Green(int blinkCount, int interval) {
  for (int i = (1); i <= (blinkCount); i = i + (1)) {
    digitalWrite(11, LOW);
    delay(interval);
    digitalWrite(11, HIGH);
    delay(interval);
  }
}

void RGB_Blue(int blinkCount, int interval) {
  for (int i = (1); i <= (blinkCount); i = i + (1)) {
    digitalWrite(13, LOW);
    delay(interval);
    digitalWrite(13, HIGH);
    delay(interval);
  }
}

void servoControl_D9(int angle_0_180) {
  servo_9.write(angle_0_180);
  delay(500);
  delay(800);
}

void buzzerAlarm_D8(int alarmCount, int interval) {
  for (int i = (1); i <= (alarmCount); i = i + (1)) {
    digitalWrite(8, HIGH);
    delay(interval);
    digitalWrite(8, LOW);
    delay(interval);
  }
}

void goStraight(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, speed);
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, (speed + correction));
  delay(timeMs);
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

void goBackward(int timeMs) {
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, speed);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, (speed + correction));
  delay(timeMs);
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

void turnRight(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, speed);
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, 0);
  delay(timeMs);
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

void spinRight(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, 120);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 120);
  delay(timeMs);
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

void turnLeft(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, 0);
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, speed);
  delay(timeMs);
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

void spinLeft(int timeMs) {
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 120);
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, 120);
  delay(timeMs);
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

void turnUntilLine(int turnDir) {
  if (turnDir == 1) {
    while (analogRead(A0) > my_1_threshold) {
      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, 80);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, 80);
    }
    while (analogRead(A2) > my_3_threshold) {
      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, 80);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, 80);
    }
    digitalWrite(2, (!direction));
    digitalWrite(4, direction);
    analogWrite(3, 0);
    digitalWrite(5, direction);
    digitalWrite(7, (!direction));
    analogWrite(6, 0);

  }
  if (turnDir == 2) {
    while (analogRead(A3) > my_4_threshold) {
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, 80);
      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, 80);
    }
    while (analogRead(A1) > my_2_threshold) {
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, 80);
      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, 80);
    }
    digitalWrite(2, (!direction));
    digitalWrite(4, direction);
    analogWrite(3, 0);
    digitalWrite(5, direction);
    digitalWrite(7, (!direction));
    analogWrite(6, 0);

  }
}

void lineFollowJunction(int turnDir) {
  if (turnDir == 1) {
    while (analogRead(A0) > my_1_threshold) {
      if (analogRead(A1) > my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);
        digitalWrite(4, (!direction));
        analogWrite(3, 0);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, speed);

      }
      if (analogRead(A1) > my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, 0);

      }
    }

  }
  if (turnDir == 2) {
    while (analogRead(A3) > my_4_threshold) {
      if (analogRead(A1) > my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);
        digitalWrite(4, (!direction));
        analogWrite(3, 0);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, speed);

      }
      if (analogRead(A1) > my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, 0);

      }
    }

  }
  digitalWrite(2, LOW);
  digitalWrite(4, HIGH);
  analogWrite(3, 0);
  digitalWrite(5, HIGH);
  digitalWrite(7, LOW);
  analogWrite(6, 0);
}

void lineFollowTime(int timeMs) {
  systemTime = millis();
  while (millis() - systemTime < timeMs) {
    if (analogRead(A1) > my_2_threshold && analogRead(A2) > my_3_threshold) {
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, speed);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, (speed + correction));

    }
    if (analogRead(A1) < my_2_threshold && analogRead(A2) < my_3_threshold) {
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, speed);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, (speed + correction));

    }
    if (analogRead(A1) < my_2_threshold && analogRead(A2) > my_3_threshold) {
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, 0);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, speed);

    }
    if (analogRead(A1) > my_2_threshold && analogRead(A2) < my_3_threshold) {
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, speed);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, 0);

    }
  }
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  analogWrite(3, 0);
  digitalWrite(5, LOW);
  digitalWrite(7, HIGH);
  analogWrite(6, 0);
}

void setup() {
  pinMode(8, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  servo_9.attach(9);
  my_1_white = 0;
  my_2_white = 0;
  my_3_white = 0;
  my_4_white = 0;
  my_1_black = 0;
  my_2_black = 0;
  my_3_black = 0;
  my_4_black = 0;
  my_1_threshold = 0;
  my_2_threshold = 0;
  my_3_threshold = 0;
  my_4_threshold = 0;
  speed = 150;
  correction = 0;
  direction = 0;
  systemTime = 0;
  digitalWrite(8, LOW);
  digitalWrite(11, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(13, HIGH);
  servo_9.write(180);
  delay(0);
  pinMode(10, INPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
}

void loop() {
  getThresholds(0, 150, 7);
  buzzerAlarm_D8(2, 150);
  goStraight(300);
  lineFollowTime(700);
  spinRight(500);
  delay(3500);
  turnUntilLine(1);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(2);
  lineFollowTime(700);
  delay(1500);
  buzzerAlarm_D8(2, 150);
  turnUntilLine(1);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  delay(1500);
  buzzerAlarm_D8(2, 150);
  spinRight(200);
  turnUntilLine(2);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  goStraight(300);
  spinLeft(500);
  delay(3500);
  turnUntilLine(2);
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  delay(2000);
  buzzerAlarm_D8(100, 10);
  goBackward(400);
  turnUntilLine(2);
  lineFollowJunction(2);
  delay(2000);
  goBackward(500);
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(2);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(1);
  spinRight(300);
  goStraight(900);
  delay(2000);
  buzzerAlarm_D8(2, 150);

}