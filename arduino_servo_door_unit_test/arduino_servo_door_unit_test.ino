#include <Servo.h>

/* ============================================================
 * 物料舱门拉杆独立单元测试
 *
 * 已确认硬件：
 *   D9  = 舱门拉杆机械臂（180°抬杆，135°下杆）
 *   D13 = 摄像机舵机，本测试不控制
 *   D10 = 测试启动按键（按下为LOW，沿用小车现有接法）
 *
 * 动作只执行一次：
 *   初始抬杆 → 按D10 → 小幅后退 → 下杆 → 后退拉门
 *   → 停车 → 抬杆 → 向前离开 → 停车
 *
 * 注意：本程序是独立测试，不包含循迹、视觉或K230通信。
 * ============================================================ */

Servo doorServo;

const uint8_t DOOR_SERVO_PIN = 9;
const uint8_t START_BUTTON_PIN = 10;

// TB6612电机引脚，与pathfinder_commented一致。
const uint8_t LEFT_IN1_PIN = 2;
const uint8_t LEFT_IN2_PIN = 4;
const uint8_t LEFT_PWM_PIN = 3;
const uint8_t RIGHT_IN1_PIN = 5;
const uint8_t RIGHT_IN2_PIN = 7;
const uint8_t RIGHT_PWM_PIN = 6;

// D9舱门机械臂的实测角度。
const uint8_t SERVO_UP_ANGLE = 180;
const uint8_t SERVO_DOWN_ANGLE = 135;

// 第一轮测试使用较低速度和保守时间，后续按实车距离调整。
const uint8_t MOTOR_SPEED = 100;
const unsigned long INITIAL_BACKWARD_MS = 150;
const unsigned long PULL_BACKWARD_MS = 1000;
const unsigned long LEAVE_FORWARD_MS = 500;
const unsigned long SERVO_SETTLE_MS = 1000;

bool testFinished = false;

void stopMotors() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

void driveBackward(unsigned long durationMs) {
  // 对应主程序direction=0时的goBackward极性。
  digitalWrite(LEFT_IN1_PIN, HIGH);
  digitalWrite(LEFT_IN2_PIN, LOW);
  digitalWrite(RIGHT_IN1_PIN, LOW);
  digitalWrite(RIGHT_IN2_PIN, HIGH);
  analogWrite(LEFT_PWM_PIN, MOTOR_SPEED);
  analogWrite(RIGHT_PWM_PIN, MOTOR_SPEED);

  delay(durationMs);
  stopMotors();
}

void driveForward(unsigned long durationMs) {
  // 对应主程序direction=0时的goStraight极性。
  digitalWrite(LEFT_IN1_PIN, LOW);
  digitalWrite(LEFT_IN2_PIN, HIGH);
  digitalWrite(RIGHT_IN1_PIN, HIGH);
  digitalWrite(RIGHT_IN2_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, MOTOR_SPEED);
  analogWrite(RIGHT_PWM_PIN, MOTOR_SPEED);

  delay(durationMs);
  stopMotors();
}

void moveDoorServo(uint8_t angle) {
  doorServo.write(angle);
  delay(SERVO_SETTLE_MS);
}

void waitForStartButton() {
  Serial.println(F("Place the car at the stopped door position."));
  Serial.println(F("Press D10 once to run the sequence."));

  // 沿用现有小车按键：空闲HIGH，按下LOW。
  while (digitalRead(START_BUTTON_PIN) == HIGH) {
    delay(10);
  }
  delay(30);

  // 等待松开，避免按键保持按下影响观察。
  while (digitalRead(START_BUTTON_PIN) == LOW) {
    delay(10);
  }
  delay(100);
}

void runDoorPullSequence() {
  Serial.println(F("STEP 1: short backward adjustment"));
  driveBackward(INITIAL_BACKWARD_MS);
  delay(300);

  Serial.println(F("STEP 2: lower D9 door arm to 135 degrees"));
  moveDoorServo(SERVO_DOWN_ANGLE);

  Serial.println(F("STEP 3: reverse to pull the door outward"));
  driveBackward(PULL_BACKWARD_MS);
  delay(300);

  Serial.println(F("STEP 4: raise D9 door arm to 180 degrees"));
  moveDoorServo(SERVO_UP_ANGLE);

  Serial.println(F("STEP 5: drive forward and leave"));
  driveForward(LEAVE_FORWARD_MS);
  stopMotors();

  Serial.println(F("DONE: sequence finished; it will not repeat."));
}

void setup() {
  Serial.begin(115200);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  pinMode(LEFT_IN1_PIN, OUTPUT);
  pinMode(LEFT_IN2_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_IN1_PIN, OUTPUT);
  pinMode(RIGHT_IN2_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  stopMotors();

  doorServo.attach(DOOR_SERVO_PIN);
  moveDoorServo(SERVO_UP_ANGLE);

  Serial.println(F("Door pull unit test ready."));
}

void loop() {
  if (testFinished) {
    stopMotors();
    delay(100);
    return;
  }

  waitForStartButton();
  runDoorPullSequence();
  testFinished = true;
}
