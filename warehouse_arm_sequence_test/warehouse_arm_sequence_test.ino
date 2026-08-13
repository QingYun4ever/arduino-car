/*
 * ============================================================================
 * warehouse_arm_sequence_test.ino
 * 高中组“检查仓门”机械臂动作序列独立验证程序（Arduino Nano）
 * ============================================================================
 *
 * 【规则对应】
 *   任务5：检查仓门（20分）
 *   机器人前往库房物料舱门处，通过机械臂对指定舱门门锁进行检测和操作。
 *   高中组将门锁推拉杆拉出，即表示该任务完成。
 *
 * 【本程序只验证动作顺序】
 *   抬杆 → 往前 → 下杆 → 后退拉出推拉杆 → 抬杆 → 后退脱离
 *   → 下杆复位 → 向前走人 → 两声鸣笛 → 停车
 *
 * 不使用摄像头、不识别位置、不循迹、不判断是否已经到达仓门。
 * “往前/后退”的距离仅由文件顶部的时间参数临时控制，必须在空旷场地
 * 或把车轮架空后先测试。确定实际赛道动作后，应该把本程序中的动作函数
 * 复制到新的路线工作副本，而不要修改稳定的 pathfinder_basic_route。
 *
 * 【已确认引脚】
 *   D9             左机械臂舵机信号（D8留给蜂鸣器，不能冲突）
 *   D8             蜂鸣器（HIGH为鸣叫）
 *   D10            启动按键（HIGH为按下）
 *   D2/D4/D3       左电机 AIN1/AIN2/PWMA
 *   D5/D7/D6       右电机 BIN1/BIN2/PWMB
 *
 * 【使用】
 *   1. 接好舵机独立5~6V电源，Nano和舵机电源必须共地。
 *   2. 上传后保持机械臂默认水平姿态，电机不动；按D10后才执行动作。
 *   3. 确认机械臂、车轮和门锁附近没有危险，再按D10一次执行整套动作。
 *   4. 完成后两声鸣笛并停车；需要重测请复位/重新上电。
 * ============================================================================
 */

#include <Servo.h>

/* =========================== 需要实测调整的参数 ============================ */

// Servo.write() 的0~180只是控制位置，不等于机械臂的真实几何角度。
// 保留当前已试过的50作为抬杆基准；首次只使用相差10的保守小范围。
// 确认方向后，每次以5为单位微调；若物理运动方向相反，直接交换这两个数值。
const int ARM_UP_ANGLE = 50;       // 抬杆：避开推拉杆（待实测）
const int ARM_DOWN_ANGLE = 60;     // 下杆：进入/挂住推拉杆位置（待实测）
const unsigned long ARM_MOVE_SETTLE_MS = 850;

// 下列仅是“无视觉、无距离检测”的初始时间，必须在实车上逐步调整。
const int DRIVE_SPEED = 125;
const int RIGHT_SPEED_TRIM = 7;
const unsigned long FORWARD_TO_LOCK_MS = 600; // 下杆前，向仓门靠近
const unsigned long PULL_LATCH_BACK_MS = 700; // 下杆后，后退拉出推拉杆
const unsigned long REVERSE_CLEAR_MS = 500;   // 抬杆后，再后退脱离仓门
const unsigned long LEAVE_FORWARD_MS = 650;   // 下杆复位后，向前离开

/* ================================ 固定引脚 ================================= */

const uint8_t PIN_ARM_SERVO = 9;
const uint8_t PIN_BUZZER = 8;
const uint8_t PIN_START_BUTTON = 10;

const uint8_t PIN_LEFT_IN1 = 2;
const uint8_t PIN_LEFT_IN2 = 4;
const uint8_t PIN_LEFT_PWM = 3;
const uint8_t PIN_RIGHT_IN1 = 5;
const uint8_t PIN_RIGHT_IN2 = 7;
const uint8_t PIN_RIGHT_PWM = 6;

Servo leftArm;

int clampPwm(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return value;
}

// signedSpeed > 0 为小车逻辑前进；< 0 为逻辑后退。
void setMotors(int leftSignedSpeed, int rightSignedSpeed) {
  if (leftSignedSpeed >= 0) {
    digitalWrite(PIN_LEFT_IN1, LOW);
    digitalWrite(PIN_LEFT_IN2, HIGH);
  } else {
    digitalWrite(PIN_LEFT_IN1, HIGH);
    digitalWrite(PIN_LEFT_IN2, LOW);
  }

  if (rightSignedSpeed >= 0) {
    digitalWrite(PIN_RIGHT_IN1, HIGH);
    digitalWrite(PIN_RIGHT_IN2, LOW);
  } else {
    digitalWrite(PIN_RIGHT_IN1, LOW);
    digitalWrite(PIN_RIGHT_IN2, HIGH);
  }

  analogWrite(PIN_LEFT_PWM, clampPwm(abs(leftSignedSpeed)));
  analogWrite(PIN_RIGHT_PWM, clampPwm(abs(rightSignedSpeed)));
}

void stopMotors() {
  setMotors(0, 0);
}

void driveFor(int leftSpeed, int rightSpeed, unsigned long durationMs) {
  setMotors(leftSpeed, rightSpeed);
  delay(durationMs);
  stopMotors();
  delay(150); // 每一步先明确停住，再进行下一步，便于观察和保护机构
}

void driveForwardFor(unsigned long durationMs) {
  driveFor(DRIVE_SPEED, DRIVE_SPEED + RIGHT_SPEED_TRIM, durationMs);
}

void driveBackwardFor(unsigned long durationMs) {
  driveFor(-DRIVE_SPEED, -(DRIVE_SPEED + RIGHT_SPEED_TRIM), durationMs);
}

void setArmAngle(int angle, const __FlashStringHelper *label) {
  Serial.println(label);
  leftArm.write(angle);
  delay(ARM_MOVE_SETTLE_MS);
}

void beepTwice() {
  for (uint8_t i = 0; i < 2; ++i) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(130);
    digitalWrite(PIN_BUZZER, LOW);
    delay(130);
  }
}

bool startButtonPressed() {
  return digitalRead(PIN_START_BUTTON) == HIGH;
}

void waitForStart() {
  Serial.println(F("等待D10启动按键；按下后执行一次仓门机械臂动作。"));
  while (!startButtonPressed()) {
    delay(5);
  }
  delay(20); // 消抖
  while (startButtonPressed()) {
    delay(5);
  }
  delay(250);
}

void runWarehouseDoorSequence() {
  // 1. 抬杆：让机械臂以安全高度接近门锁。
  setArmAngle(ARM_UP_ANGLE, F("1/8 抬杆"));

  // 2. 往前：车身靠近门锁推拉杆。
  Serial.println(F("2/8 往前靠近仓门"));
  driveForwardFor(FORWARD_TO_LOCK_MS);

  // 3. 下杆：使机械臂落到推拉杆的操作高度。
  setArmAngle(ARM_DOWN_ANGLE, F("3/8 下杆挂住推拉杆"));

  // 4. 后退：通过车身后退，将门锁推拉杆拉出。
  Serial.println(F("4/8 后退，拉出门锁推拉杆"));
  driveBackwardFor(PULL_LATCH_BACK_MS);

  // 5. 抬杆：脱离已拉出的推拉杆，避免继续勾住。
  setArmAngle(ARM_UP_ANGLE, F("5/8 抬杆脱离推拉杆"));

  // 6. 后退：使车身离开仓门区域。
  Serial.println(F("6/8 后退脱离仓门"));
  driveBackwardFor(REVERSE_CLEAR_MS);

  // 7. 下杆：机械臂回到收回/待机位置。
  setArmAngle(ARM_DOWN_ANGLE, F("7/8 下杆复位"));

  // 8. 走人：恢复前进，后续完整路线中将从这里接入下一段循迹。
  Serial.println(F("8/8 向前离开"));
  driveForwardFor(LEAVE_FORWARD_MS);

  stopMotors();
  beepTwice();
  Serial.println(F("仓门动作序列完成，已停车。"));
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_START_BUTTON, INPUT);

  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  stopMotors();

  // 默认水平就是上电初始姿态：只attach，不调用Servo.write()改变位置。
  // 上电不执行机械臂预动作或电机动作，只有按D10后才会开始完整序列。
  leftArm.attach(PIN_ARM_SERVO);

  Serial.println(F("warehouse_arm_sequence_test 已启动。"));
  waitForStart();
  runWarehouseDoorSequence();
}

void loop() {
  // 测试动作只执行一次，之后始终停车。复位/重新上电后才可再次测试。
  stopMotors();
  delay(100);
}
