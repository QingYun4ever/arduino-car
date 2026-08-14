/*
 * ============================================================
 * pathfinder_commented_full_vision.ino —— 朋友完整路线+四任务视觉+舱门副本
 * ============================================================
 *
 * 【本文件说明】
 *   - 本文件复制自朋友完成的pathfinder_commented完整基础+拓展路线。
 *   - 保持原巡线、转向、等待及六台光刻机拓展路线参数不变。
 *   - 增加K230四任务通信，并接入D9拉门、D13低头和六台光刻机识别。
 *   - 原pathfinder_commented.ino保持不变。
 *
 * 【赛事背景】智能芯片与自动驾驶专项赛·高中组决赛
 *   - 规则文件: D:\dev\Robot\小车\高中组比赛规则.docx
 *   - 基础任务100分: 开始工作5 / 岗位巡查30 / 重点消防15 /
 *     货品管理20 / 检查仓门20 / 返回充电10
 *   - 拓展任务30分: 设备巡检(6台光刻机+3锥桶)
 *   - 每轮4分钟, 两轮取最高分; 鸣笛2声或闪灯2次 = 常见"动作完成"信号
 *
 * 【硬件接线】(Arduino Nano)
 *   ┌──────────┬──────────┬──────────────────────────┐
 *   │ 引脚      │ 外设      │ 说明                      │
 *   ├──────────┼──────────┼──────────────────────────┤
 *   │ D8       │ 蜂鸣器    │ 低电平有效(LOW=响)          │
 *   │ D11      │ 任务bit0 │ 经分压接K230 IO33            │
 *   │ D12      │ UART RX  │ 接K230 IO9/UART1_TX          │
 *   │ D13      │ 摄像机舵机│ 禁止作为RGB数字输出           │
 *   │ A4       │ 任务bit1 │ 经分压接K230 IO32            │
 *   │ D9       │ 舱门机械臂│ 180°抬杆、135°下杆，执行拉门任务 │
 *   │ D10      │ 按键      │ 校准时按三次确认白/黑采样    │
 *   │ D2/D4/D3│ 左电机    │ TB6612 AIN1/AIN2 + PWMA    │
 *   │ D5/D7/D6│ 右电机    │ TB6612 BIN1/BIN2 + PWMB    │
 *   │ A0~A3   │ 光电传感器 │ A0左外 A1中左 A2中右 A3右外 │
 *   └──────────┴──────────┴──────────────────────────┘
 *   TB6612 要点: VM接电池、VCC=5V、STBY必须拉高;
 *   转动 = 方向脚一高一低 + 对应PWM脚给速度, 单拉一脚无效。
 *
 * 【传感器判线逻辑】
 *   每个传感器先标定: 白场采样值(white) + 黑线采样值(black),
 *   阈值 = (白+黑)/2。运行中 analogRead > 阈值 = 压到黑线。
 *   A1/A2 是循迹主力(中两路), A0/A3 是左右边界路口检测。
 *
 * 【电机逻辑速查】以左电机(D2/D4/D3)为例:
 *   digitalWrite(2, dir);   → AIN1
 *   digitalWrite(4, !dir);  → AIN2
 *   analogWrite(3, 速度);   → PWMA (0~255)
 *   两方向脚取反即反转; speed 是基础速度, correction 是右轮
 *   直线补偿(直行时右轮 = speed+correction, 抵消电机差异跑偏)。
 *
 * 【调参入口】全部在 loop() 开头的 getThresholds(正反, 速度, 校正):
 *   - 正反: 0=正向, 1=反向(与电机接线匹配)
 *   - 速度: 循迹基准速度 (现150)
 *   - 校正: 右轮补偿 -20~+20 (现7)
 * ============================================================
 */
#include <Servo.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

Servo servo_9;                  // 舵机对象: 机械臂/门锁推拉杆 (D9)
Servo cameraServo_13;           // 摄像机俯仰舵机 (D13)

const uint8_t CAMERA_SERVO_PIN = 13;
const uint8_t CAMERA_UP_ANGLE = 0;
const uint8_t CAMERA_DOWN_ANGLE = 24;

/* ---- K230四任务通信：[IO32 IO33] ----
 * 00空闲，01性别，10消防颜色；11按持续时间区分：
 * 500ms=晶圆桶形状，1200ms=光刻机状态牌。 */
const uint8_t K230_TASK_BIT0_PIN = 11;     // Nano D11 -> K230 IO33
const uint8_t K230_TASK_BIT1_PIN = A4;     // Nano A4  -> K230 IO32
const uint8_t K230_UART_RX_PIN = 12;       // K230 IO9 -> Nano D12
const uint8_t K230_UNUSED_TX_PIN = A5;     // SoftwareSerial占位，不接线
const unsigned long K230_UART_BAUDRATE = 57600;
const unsigned long K230_TASK_HIGH_MS = 500;
const unsigned long K230_LITHOGRAPHY_HIGH_MS = 1200;
// SHAPE后下一次GENDER需要按需重新加载两个人脸模型，预留模型切换时间。
const unsigned long K230_RESULT_TIMEOUT_MS = 9000;
const uint8_t K230_MAX_ATTEMPTS = 2;

/* ---- 已通过舱门单模块及上干路测试的参数 ---- */
const uint8_t DOOR_SERVO_UP_ANGLE = 180;
const uint8_t DOOR_SERVO_DOWN_ANGLE = 135;
const int DOOR_PULL_SPEED = 100;
const unsigned long DOOR_INITIAL_BACKWARD_MS = 150;
const unsigned long DOOR_PULL_BACKWARD_MS = 1000;
const unsigned long DOOR_SERVO_SETTLE_MS = 1000;
const unsigned long DOOR_CLEAR_RIGHT_MS = 350;
const unsigned long DOOR_CLEAR_ALIGN_SETTLE_MS = 200;
const unsigned long DOOR_TO_ROUTE_SETTLE_MS = 500;

const uint8_t VISION_TASK_IDLE = 0;
const uint8_t VISION_TASK_GENDER = 1;
const uint8_t VISION_TASK_FIRE = 2;
const uint8_t VISION_TASK_SHAPE = 3;

SoftwareSerial k230Serial(K230_UART_RX_PIN, K230_UNUSED_TX_PIN);

struct VisionResult {
  bool received;
  char task[8];
  char label[16];
  long value;
};

/* ---- 4路光电传感器: 白场/黑线/阈值 三组采样值 ----
 *   A0=左外  A1=中左  A2=中右  A3=右外 */
volatile int my_1_white;        // A0 白场采样
volatile int my_2_white;        // A1 白场采样
volatile int my_3_white;        // A2 白场采样
volatile int my_4_white;        // A3 白场采样
volatile int my_1_black;        // A0 黑线采样
volatile int my_2_black;        // A1 黑线采样
volatile int my_3_black;        // A2 黑线采样
volatile int my_4_black;        // A3 黑线采样
volatile int my_1_threshold;    // A0 判定阈值 (白+黑)/2
volatile int my_2_threshold;    // A1 判定阈值
volatile int my_3_threshold;    // A2 判定阈值
volatile int my_4_threshold;    // A3 判定阈值

/* ---- 全局控制参数(在 getThresholds 里赋值) ---- */
volatile int speed;             // 循迹基准速度 (0~255)
volatile int correction;        // 右轮直线补偿, 直行时右轮=speed+correction
volatile bool direction;        // 电机正反: 与接线匹配的"前进"方向标志
volatile float systemTime;      // 巡线计时起点 (millis快照, 用于超时退出)

/* ============================================================
 * 校准: 上电后按提示按键采样白/黑, 计算4路阈值
 *   forward = 正反标志   initSpeed = 基准速度   straightCorrection = 右轮补偿
 *   流程: 按键(按住采样白) → 红灯闪3次 → 按键(按住采样黑) → 红灯闪3次
 *         → 算阈值 → 再按一次键开始比赛
 * ============================================================ */
void getThresholds(bool forward, int initSpeed, int straightCorrection) {
  direction = forward;
  speed = initSpeed;
  correction = straightCorrection;
  while (digitalRead(10) == 1) {          // 按键按下期间采样白场
    my_1_white = analogRead(A0);
    my_2_white = analogRead(A1);
    my_3_white = analogRead(A2);
    my_4_white = analogRead(A3);
  }
  // D12已用于K230 UART接收，保留原3次闪灯的450ms节奏但不驱动D12。
  Serial.println(F("White calibration captured; move sensors to black."));
  delay(450);
  while (digitalRead(10) == 1) {          // 按键按下期间采样黑线
    my_1_black = analogRead(A0);
    my_2_black = analogRead(A1);
    my_3_black = analogRead(A2);
    my_4_black = analogRead(A3);
  }
  // D12已用于K230 UART接收；只保留原450ms等待。
  Serial.println(F("Black calibration captured."));
  delay(450);
  my_1_threshold = (my_1_white + my_1_black) / 2;   // 阈值=白黑中点
  my_2_threshold = (my_2_white + my_2_black) / 2;
  my_3_threshold = (my_3_white + my_3_black) / 2;
  my_4_threshold = (my_4_white + my_4_black) / 2;
  while (digitalRead(10) == 1) {          // 最后按一次: 松手即开始比赛
  }
}

/* ============================================================
 * 停止程序: 两电机立即停转, 然后死循环挂起(不再执行任何动作)
 * ============================================================ */
void stopProgram() {
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  analogWrite(3, 0);                      // 左电机: AIN1=H AIN2=L PWMA=0
  digitalWrite(5, LOW);
  digitalWrite(7, HIGH);
  analogWrite(6, 0);                      // 右电机: BIN1=L BIN2=H PWMB=0
  while(true);                            // 原地待命, 防失控
}

/* ============================================================
 * D11/D12/D13已分别用于任务bit0、K230 UART RX和摄像机舵机。
 * 保留三个旧RGB函数名只为兼容原结构；它们不再操作引脚。
 * ============================================================ */
void RGB_Red(int blinkCount, int interval) {
  for (int i = 0; i < blinkCount; ++i) {
    delay(interval);
    delay(100);
  }
}

void RGB_Green(int blinkCount, int interval) {
  for (int i = 0; i < blinkCount; ++i) {
    delay(interval);
    delay(interval);
  }
}

void RGB_Blue(int blinkCount, int interval) {
  for (int i = 0; i < blinkCount; ++i) {
    delay(interval);
    delay(interval);
  }
}

/* ============================================================
 * 舵机控制 (D9): 转到指定角度, 用于"检查仓门"任务拉出门锁推拉杆
 *   angle_0_180 = 0~180°
 * ============================================================ */
void servoControl_D9(int angle_0_180) {
  servo_9.write(angle_0_180);             // 转到位
  delay(500);                             // 保持0.5s
  delay(800);                             // 再保持0.8s
}

/* ============================================================
 * 蜂鸣器报警 (D8, 低有效): 响 alarmCount 次
 *   规则对应: 2声 = 男技术员/蓝消防箱/红标牌/设备故障;
 *             1声 = 绿标牌/设备正常
 * ============================================================ */
void buzzerAlarm_D8(int alarmCount, int interval) {
  for (int i = (1); i <= (alarmCount); i = i + (1)) {
    digitalWrite(8, HIGH);                // 响
    delay(interval);
    digitalWrite(8, LOW);                 // 停
    delay(interval);
  }
}

/* ============================================================
 * K230四任务通信
 * ============================================================ */
void stopDriveForVision() {
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

void setVisionTaskCode(uint8_t taskCode) {
  digitalWrite(K230_TASK_BIT0_PIN, (taskCode & 0x01) ? HIGH : LOW);
  digitalWrite(K230_TASK_BIT1_PIN, (taskCode & 0x02) ? HIGH : LOW);
}

void clearK230Input() {
  while (k230Serial.available() > 0) {
    k230Serial.read();
  }
}

bool readK230Line(char *buffer, size_t bufferSize, unsigned long timeoutMs) {
  size_t length = 0;
  unsigned long startedAt = millis();

  while (millis() - startedAt < timeoutMs) {
    while (k230Serial.available() > 0) {
      char value = (char)k230Serial.read();
      if (value == '\r') {
        continue;
      }
      if (value == '\n') {
        if (length == 0) {
          continue;
        }
        buffer[length] = '\0';
        return true;
      }
      if (length + 1 < bufferSize) {
        buffer[length++] = value;
      }
    }
  }

  buffer[length] = '\0';
  return false;
}

bool parseVisionResult(
    char *line,
    const char *expectedTask,
    VisionResult *result) {
  char *messageType = strtok(line, ",");
  char *task = strtok(NULL, ",");
  char *label = strtok(NULL, ",");
  char *valueText = strtok(NULL, ",");

  if (messageType == NULL || strcmp(messageType, "RESULT") != 0 ||
      task == NULL || strcmp(task, expectedTask) != 0 ||
      label == NULL || valueText == NULL) {
    return false;
  }

  strncpy(result->task, task, sizeof(result->task) - 1);
  result->task[sizeof(result->task) - 1] = '\0';
  strncpy(result->label, label, sizeof(result->label) - 1);
  result->label[sizeof(result->label) - 1] = '\0';
  result->value = atol(valueText);
  result->received = true;
  return true;
}

bool requestVisionTask(
    uint8_t taskCode,
    const char *expectedTask,
    VisionResult *result,
    unsigned long taskHighMs = K230_TASK_HIGH_MS) {
  result->received = false;
  result->task[0] = '\0';
  result->label[0] = '\0';
  result->value = 0;

  stopDriveForVision();
  delay(300);
  clearK230Input();
  setVisionTaskCode(VISION_TASK_IDLE);
  delay(50);
  setVisionTaskCode(taskCode);
  delay(taskHighMs);
  setVisionTaskCode(VISION_TASK_IDLE);

  Serial.print(F("VISION TASK -> "));
  Serial.print(expectedTask);
  Serial.print(F(" code="));
  Serial.println(taskCode);

  unsigned long startedAt = millis();
  char line[64];
  while (millis() - startedAt < K230_RESULT_TIMEOUT_MS) {
    unsigned long remaining = K230_RESULT_TIMEOUT_MS -
                              (millis() - startedAt);
    if (!readK230Line(line, sizeof(line), remaining)) {
      break;
    }

    Serial.print(F("K230 RX <- "));
    Serial.println(line);

    if (strncmp(line, "RESULT,", 7) == 0) {
      char parsedLine[64];
      strncpy(parsedLine, line, sizeof(parsedLine) - 1);
      parsedLine[sizeof(parsedLine) - 1] = '\0';
      if (parseVisionResult(parsedLine, expectedTask, result)) {
        return true;
      }
    }
  }

  Serial.print(F("VISION TIMEOUT: "));
  Serial.println(expectedTask);
  return false;
}

bool runVisionTaskWithRetry(
    uint8_t taskCode,
    const char *expectedTask,
    VisionResult *result) {
  for (uint8_t attempt = 1; attempt <= K230_MAX_ATTEMPTS; ++attempt) {
    Serial.print(F("Vision attempt "));
    Serial.print(attempt);
    Serial.print(F("/"));
    Serial.print(K230_MAX_ATTEMPTS);
    Serial.print(F(" for "));
    Serial.println(expectedTask);

    if (requestVisionTask(taskCode, expectedTask, result) &&
        strcmp(result->label, "U") != 0) {
      return true;
    }
    delay(300);
  }
  return false;
}

void inspectGenderAtZone(uint8_t zoneNumber) {
  VisionResult result;
  Serial.print(F("GENDER zone "));
  Serial.println(zoneNumber);

  if (!runVisionTaskWithRetry(
          VISION_TASK_GENDER, "GENDER", &result)) {
    Serial.println(F("GENDER unavailable; continue route."));
    return;
  }

  Serial.print(F("GENDER result="));
  Serial.print(result.label);
  Serial.print(F(" confidence="));
  Serial.println(result.value);
  if (strcmp(result.label, "M") == 0) {
    buzzerAlarm_D8(2, 150);
  }
}

void inspectFireExtinguisher() {
  VisionResult result;
  Serial.println(F("FIRE task"));

  if (!runVisionTaskWithRetry(VISION_TASK_FIRE, "FIRE", &result)) {
    Serial.println(F("FIRE unavailable; continue route."));
    return;
  }

  Serial.print(F("FIRE result="));
  Serial.print(result.label);
  Serial.print(F(" largest_pixels="));
  Serial.println(result.value);
  if (strcmp(result.label, "BLUE") == 0) {
    buzzerAlarm_D8(2, 150);
  }
}

void inspectWaferContainerShape() {
  VisionResult result;
  Serial.println(F("SHAPE goods-task candidate"));

  if (!runVisionTaskWithRetry(VISION_TASK_SHAPE, "SHAPE", &result)) {
    Serial.println(F("SHAPE unavailable; continue route."));
    return;
  }

  Serial.print(F("SHAPE result="));
  Serial.print(result.label);
  Serial.print(F(" score="));
  Serial.println(result.value);
  // Cylinder/tube对应的比赛动作尚未指定，本次只记录结果，不擅自鸣笛。
}

void inspectLithographyMachine(uint8_t machineNumber) {
  VisionResult result;
  bool recognized = false;

  Serial.print(F("LITHO machine "));
  Serial.println(machineNumber);

  for (uint8_t attempt = 1; attempt <= K230_MAX_ATTEMPTS; ++attempt) {
    Serial.print(F("LITHO attempt "));
    Serial.print(attempt);
    Serial.print(F("/"));
    Serial.println(K230_MAX_ATTEMPTS);

    // 物理任务码仍为11；1200ms长脉冲与500ms晶圆桶形状区分。
    if (requestVisionTask(
            VISION_TASK_SHAPE,
            "LITHO",
            &result,
            K230_LITHOGRAPHY_HIGH_MS) &&
        strcmp(result.label, "U") != 0) {
      recognized = true;
      break;
    }
    delay(300);
  }

  if (!recognized) {
    Serial.println(F("LITHO unavailable; no buzzer, continue route."));
    return;
  }

  Serial.print(F("LITHO result="));
  Serial.print(result.label);
  Serial.print(F(" largest_pixels="));
  Serial.println(result.value);

  if (strcmp(result.label, "RED") == 0) {
    buzzerAlarm_D8(2, 150);       // 红色故障牌：鸣笛2声
  } else if (strcmp(result.label, "GREEN") == 0) {
    buzzerAlarm_D8(1, 150);       // 绿色正常牌：鸣笛1声
  }
}

/* ============================================================
 * 直行 timeMs 毫秒 (两轮同向, 右轮带校正补偿)
 *   例: goStraight(300) = 前进约300ms
 * ============================================================ */
void goStraight(int timeMs) {
  digitalWrite(2, direction);             // 左电机 AIN1=方向
  digitalWrite(4, (!direction));          //       AIN2=反向
  analogWrite(3, speed);                  //       PWMA=基准速度
  digitalWrite(5, (!direction));          // 右电机 BIN1=反向
  digitalWrite(7, direction);             //       BIN2=方向
  analogWrite(6, (speed + correction));   //       PWMB=速度+右轮补偿
  delay(timeMs);                          // 持续行进
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 后退 timeMs 毫秒 (两轮反向, 与直行相反的极性)
 * ============================================================ */
void goBackward(int timeMs) {
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, speed);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, (speed + correction));
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 右转 timeMs: 右轮停、左轮转 → 走弧线(非原地)
 * ============================================================ */
void turnRight(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, speed);                  // 左轮转
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, 0);                      // 右轮停
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 原地右转(右转圈) timeMs: 两轮反向 → 绕车中心自转
 * ============================================================ */
void spinRight(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, 120);                    // 左轮前进
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 120);                    // 右轮后退
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 左转 timeMs: 左轮停、右轮转 → 走弧线(非原地)
 * ============================================================ */
void turnLeft(int timeMs) {
  digitalWrite(2, direction);
  digitalWrite(4, (!direction));
  analogWrite(3, 0);                      // 左轮停
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, speed);                  // 右轮转
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 原地左转(左转圈) timeMs: 两轮反向 → 绕车中心自转
 * ============================================================ */
void spinLeft(int timeMs) {
  digitalWrite(2, (!direction));
  digitalWrite(4, direction);
  analogWrite(3, 120);                    // 左轮后退
  digitalWrite(5, (!direction));
  digitalWrite(7, direction);
  analogWrite(6, 120);                    // 右轮前进
  delay(timeMs);
  digitalWrite(2, (!direction));          // —— 停车 ——
  digitalWrite(4, direction);
  analogWrite(3, 0);
  digitalWrite(5, direction);
  digitalWrite(7, (!direction));
  analogWrite(6, 0);
}

/* ============================================================
 * 转弯遇线停 turnDir: 低速(80)行进, 依次等传感器压线后停止
 *   用于路口/弯道对线对齐。
 *   turnDir=1(左): 等 A0(左外)压线 → 等 A2(中右)压线 → 停
 *   turnDir=2(右): 等 A3(右外)压线 → 等 A1(中左)压线 → 停
 *   注意: 电机极性与 goBackward 相同, 具体行进方向需实测确认
 * ============================================================ */
void turnUntilLine(int turnDir) {
  if (turnDir == 1) {
    while (analogRead(A0) > my_1_threshold) {   // A0 仍压线
      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, 80);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, 80);
    }
    while (analogRead(A2) > my_3_threshold) {   // A2 仍压线
      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, 80);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, 80);
    }
    digitalWrite(2, (!direction));              // —— 停车 ——
    digitalWrite(4, direction);
    analogWrite(3, 0);
    digitalWrite(5, direction);
    digitalWrite(7, (!direction));
    analogWrite(6, 0);

  }
  if (turnDir == 2) {
    while (analogRead(A3) > my_4_threshold) {   // A3 仍压线
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, 80);
      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, 80);
    }
    while (analogRead(A1) > my_2_threshold) {   // A1 仍压线
      digitalWrite(2, direction);
      digitalWrite(4, (!direction));
      analogWrite(3, 80);
      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, 80);
    }
    digitalWrite(2, (!direction));              // —— 停车 ——
    digitalWrite(4, direction);
    analogWrite(3, 0);
    digitalWrite(5, direction);
    digitalWrite(7, (!direction));
    analogWrite(6, 0);

  }
}

/* ============================================================
 * 舱门任务并衔接回干路
 *   - 门区动作使用已验证速度100：后退150ms、下杆、后退1000ms拉门；
 *   - 拉门后禁止向前，先抬杆，再原地右转350ms避开拉杆；
 *   - turnUntilLine(2)只负责对到干路；完成后恢复主路线速度和补偿。
 * ============================================================ */
void pullDoorAndJoinMainRoad() {
  int savedSpeed = speed;
  int savedCorrection = correction;

  speed = DOOR_PULL_SPEED;
  correction = 0;

  Serial.println(F("DOOR 1: reverse 150ms for rod clearance"));
  goBackward(DOOR_INITIAL_BACKWARD_MS);
  delay(300);

  Serial.println(F("DOOR 2: lower D9 arm to 135 degrees"));
  servo_9.write(DOOR_SERVO_DOWN_ANGLE);
  delay(DOOR_SERVO_SETTLE_MS);

  Serial.println(F("DOOR 3: reverse 1000ms to pull door outward"));
  goBackward(DOOR_PULL_BACKWARD_MS);

  Serial.println(F("DOOR 4: stopped; raise D9 arm to 180 degrees"));
  servo_9.write(DOOR_SERVO_UP_ANGLE);
  delay(DOOR_SERVO_SETTLE_MS);
  stopDriveForVision();

  // 不能直接向前，否则会再次顶到拉杆并把门推回去。
  Serial.println(F("DOOR 5: spin right 350ms away from pulled door"));
  spinRight(DOOR_CLEAR_RIGHT_MS);

  Serial.println(F("DOOR 6: align to main road with turnUntilLine(2)"));
  turnUntilLine(2);
  delay(DOOR_CLEAR_ALIGN_SETTLE_MS);

  speed = savedSpeed;
  correction = savedCorrection;
  delay(DOOR_TO_ROUTE_SETTLE_MS);
  Serial.println(F("DOOR PASS: aligned to main road; resume existing route"));
}

/* ============================================================
 * 巡线找左右路口 turnDir: 循迹行驶直到外侧传感器脱线/路口
 *   turnDir=1(左): 以 A0(左外)为停止条件, 期间用 A1/A2 转向循迹
 *   turnDir=2(右): 以 A3(右外)为停止条件, 期间用 A1/A2 转向循迹
 *   转向逻辑(A1/A2 中两路):
 *     两路都压线      → 直行
 *     两路都脱线      → 直行(可能过路口)
 *     A1脱/A2压(偏右) → 左轮停、右轮转 → 向右修正
 *     A1压/A2脱(偏左) → 左轮转、右轮停 → 向左修正
 *   结束后两电机停转
 * ============================================================ */
void lineFollowJunction(int turnDir) {
  if (turnDir == 1) {
    while (analogRead(A0) > my_1_threshold) {   // 左外路未到, 继续巡线
      if (analogRead(A1) > my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);             // 都在线上: 直行
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);             // 都脱线(路口): 直行
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);             // 偏右: 左停右转
        digitalWrite(4, (!direction));
        analogWrite(3, 0);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, speed);

      }
      if (analogRead(A1) > my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);             // 偏左: 左转右停
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, 0);

      }
    }

  }
  if (turnDir == 2) {
    while (analogRead(A3) > my_4_threshold) {   // 右外路未到, 继续巡线
      if (analogRead(A1) > my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);             // 都在线上: 直行
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);             // 都脱线(路口): 直行
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, (speed + correction));

      }
      if (analogRead(A1) < my_2_threshold && analogRead(A2) > my_3_threshold) {
        digitalWrite(2, direction);             // 偏右: 左停右转
        digitalWrite(4, (!direction));
        analogWrite(3, 0);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, speed);

      }
      if (analogRead(A1) > my_2_threshold && analogRead(A2) < my_3_threshold) {
        digitalWrite(2, direction);             // 偏左: 左转右停
        digitalWrite(4, (!direction));
        analogWrite(3, speed);
        digitalWrite(5, (!direction));
        digitalWrite(7, direction);
        analogWrite(6, 0);

      }
    }

  }
  digitalWrite(2, LOW);                         // —— 结束: 停车 ——
  digitalWrite(4, HIGH);
  analogWrite(3, 0);
  digitalWrite(5, HIGH);
  digitalWrite(7, LOW);
  analogWrite(6, 0);
}

/* ============================================================
 * 定时巡线 timeMs 毫秒: 用 A1/A2 中两路循迹, millis 超时退出
 *   转向逻辑同 lineFollowJunction (直行/修正两档)
 *   结束后停车
 * ============================================================ */
void lineFollowTime(int timeMs) {
  systemTime = millis();                        // 计时起点
  while (millis() - systemTime < timeMs) {      // 未超时则继续
    if (analogRead(A1) > my_2_threshold && analogRead(A2) > my_3_threshold) {
      digitalWrite(2, direction);               // 都在线上: 直行
      digitalWrite(4, (!direction));
      analogWrite(3, speed);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, (speed + correction));

    }
    if (analogRead(A1) < my_2_threshold && analogRead(A2) < my_3_threshold) {
      digitalWrite(2, direction);               // 都脱线(路口): 直行
      digitalWrite(4, (!direction));
      analogWrite(3, speed);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, (speed + correction));

    }
    if (analogRead(A1) < my_2_threshold && analogRead(A2) > my_3_threshold) {
      digitalWrite(2, direction);               // 偏右: 左停右转
      digitalWrite(4, (!direction));
      analogWrite(3, 0);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, speed);

    }
    if (analogRead(A1) > my_2_threshold && analogRead(A2) < my_3_threshold) {
      digitalWrite(2, direction);               // 偏左: 左转右停
      digitalWrite(4, (!direction));
      analogWrite(3, speed);
      digitalWrite(5, (!direction));
      digitalWrite(7, direction);
      analogWrite(6, 0);

    }
  }
  digitalWrite(2, HIGH);                        // —— 超时: 停车 ——
  digitalWrite(4, LOW);
  analogWrite(3, 0);
  digitalWrite(5, LOW);
  digitalWrite(7, HIGH);
  analogWrite(6, 0);
}
/* ============================================================
 * 定时巡线后退 timeMs 毫秒
 *
 * A1 / A2 继续负责循迹纠偏
 * 电机方向改为后退方向
 * 时间到后停车
 *
 * 用法：
 * lineFollowBackwardTime(900);
 * ============================================================ */
void lineFollowBackwardTime(int timeMs) {

  systemTime = millis();

  while (millis() - systemTime < timeMs) {

    /* A1 / A2 状态相同：直线后退 */
    if (
      analogRead(A1) > my_2_threshold &&
      analogRead(A2) > my_3_threshold
    ) {

      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, speed);

      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, speed + correction);
    }


    /* A1 / A2 都进入另一状态：仍直线后退 */
    if (
      analogRead(A1) < my_2_threshold &&
      analogRead(A2) < my_3_threshold
    ) {

      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, speed);

      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, speed + correction);
    }


    /* A1 < 阈值，A2 > 阈值 */
    if (
      analogRead(A1) < my_2_threshold &&
      analogRead(A2) > my_3_threshold
    ) {

      /* 左轮停，右轮后退 */
      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, 0);

      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, speed);
    }


    /* A1 > 阈值，A2 < 阈值 */
    if (
      analogRead(A1) > my_2_threshold &&
      analogRead(A2) < my_3_threshold
    ) {

      /* 左轮后退，右轮停 */
      digitalWrite(2, (!direction));
      digitalWrite(4, direction);
      analogWrite(3, speed);

      digitalWrite(5, direction);
      digitalWrite(7, (!direction));
      analogWrite(6, 0);
    }
  }


  /* 时间到，停车 */
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  analogWrite(3, 0);

  digitalWrite(5, LOW);
  digitalWrite(7, HIGH);
  analogWrite(6, 0);
}
/* ============================================================
 * setup: 初始化原路线硬件、K230四任务通信和两路舵机
 *   - D11/A4为任务bit0/bit1，D12为K230 UART接收
 *   - D9执行舱门任务；D13在拓展区低头24°观察光刻机
 * ============================================================ */
void setup() {
  Serial.begin(115200);
  k230Serial.begin(K230_UART_BAUDRATE);
  pinMode(8, OUTPUT);                           // 蜂鸣器
  pinMode(K230_TASK_BIT0_PIN, OUTPUT);          // D11 -> K230 IO33
  pinMode(K230_TASK_BIT1_PIN, OUTPUT);          // A4  -> K230 IO32
  pinMode(K230_UART_RX_PIN, INPUT);             // D12 <- K230 IO9
  setVisionTaskCode(VISION_TASK_IDLE);          // 上电保持00空闲
  servo_9.attach(9);                            // D9舱门机械臂
  cameraServo_13.attach(CAMERA_SERVO_PIN);       // D13摄像机舵机
  my_1_white = 0;                               // —— 变量清零 ——
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
  speed = 150;                                  // 默认基准速度
  correction = 0;                               // 默认无补偿
  direction = 0;                                // 默认正向
  systemTime = 0;
  digitalWrite(8, LOW);                         // 蜂鸣器保持朋友原初始状态
  setVisionTaskCode(VISION_TASK_IDLE);
  servo_9.write(DOOR_SERVO_UP_ANGLE);           // D9机械臂保持抬起
  cameraServo_13.write(CAMERA_UP_ANGLE);         // 上电先将摄像机摆正到0°抬起位
  Serial.println(F("Camera servo D13: UP 0 degrees; wait before calibration"));
  delay(1000);                                  // 等摄像机到位后才进入白/黑校准
  pinMode(10, INPUT);                           // 校准按键
  pinMode(2, OUTPUT);                           // 左电机 AIN1
  pinMode(4, OUTPUT);                           // 左电机 AIN2
  pinMode(3, OUTPUT);                           // 左电机 PWMA
  pinMode(5, OUTPUT);                           // 右电机 BIN1
  pinMode(7, OUTPUT);                           // 右电机 BIN2
  pinMode(6, OUTPUT);                           // 右电机 PWMB
}

/* ============================================================
 * loop: 朋友完成的固定基础+拓展路线，插入K230四类视觉任务和舱门动作。
 *   段1校准仍为 getThresholds(0, 150, 7)。
 *   基础区执行性别、消防、晶圆桶和D9舱门任务。
 *   拓展区D13低头24°，在六台光刻机原观察点识别红/绿并按规则鸣笛。
 * ============================================================ */
void loop() {
  
  /* ===== 段1: 阈值校准(上电操作, 见 getThresholds) ===== */
  getThresholds(0, 150, 7);         // 正反=0 速度=150 右轮校正=7
  /* ===== 段2: 开始工作(规则任务1: 鸣笛2声后驶离启动区) ===== */
  buzzerAlarm_D8(2, 150);           // 鸣笛2声
  goStraight(300);                  // 驶离启动区
  /* ===== 段3: 第一段巡线 ===== */
  lineFollowTime(770);              // 循迹700ms
  spinRight(650);                   // 原地右转(转弯)
  delay(3500);                      // 等待3.5s(推测: 等车身摆正/现场节奏)
  turnUntilLine(1);                 // 左向对线
  lineFollowJunction(1);            // 循迹找左侧路口
  goStraight(500);
  turnUntilLine(2);                 // 右向对线
  lineFollowJunction(2);            // 循迹找右侧路口
  goStraight(200);
  turnUntilLine(2);
  lineFollowTime(1000);              // 循迹700ms
  delay(1500);                      // 等待1.5s
  /* ===== 段4: 岗位巡查候选点 ===== */
  inspectGenderAtZone(4);           // 01：性别识别，M鸣笛2声
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
  inspectFireExtinguisher();        // 10：消防颜色，BLUE鸣笛2声
  spinRight(200);                   // 原地右转
  turnUntilLine(2);
  lineFollowJunction(1);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  goStraight(300);
  /* ===== 段5: 大转弯+长等待(推测: 跨区) ===== */
  spinLeft(800);                    // 原地左转
  delay(3500);                      // 等待3.5s
  inspectGenderAtZone(5);           // 01：第5区岗位巡查候选点
  turnUntilLine(2);
  turnUntilLine(2);
  lineFollowJunction(2);
  goStraight(300);
  turnUntilLine(1);
  lineFollowJunction(2);
  delay(2000);                      // 等待2s
  inspectWaferContainerShape();     // 11：货品/晶圆桶候选点，只记录形状结果
  /* ===== 段6: 舱门拉出并右转对到干路 ===== */
  buzzerAlarm_D8(100, 10);          // 保留朋友路线原有急促提示
  pullDoorAndJoinMainRoad();        // 替代旧后退400ms+直接对线，禁止向前推回拉杆

  // 以下6号人脸及后续路线保持原样；舱门函数只负责回到干路。
  lineFollowJunction(2);
  delay(2000);
  inspectGenderAtZone(6);           // 01：第6区岗位巡查候选点
  goBackward(500);                  // 再后退500ms
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
  /* ===== 段7: 冲刺+收尾 ===== */
  spinRight(400);                   // 原地右转
  goStraight(900);                  // 长直行900ms(冲刺?)
  delay(2000);                      // 等待2s
  buzzerAlarm_D8(2, 150);           // 最后鸣笛2声(推测: 结束信号)

  // 进入拓展任务区前将摄像机低头到已经单测通过的24°。
  cameraServo_13.write(CAMERA_DOWN_ANGLE);
  Serial.println(F("Camera servo D13: DOWN 24 degrees for lithography"));
  delay(1000);

  /* ============================================================
   *                  拓展任务：6台光刻机
   *
   * 已知实测标尺：
   * 4区L右转后完整直线路段约 lineFollowTime(1000)
   *
   * 90°任务观察转向暂按 750ms
   * 180°暂按 1500ms
   * 每台观察 3000ms
   *
   * 2号和4号在同一个道路中间观察点完成
   * ============================================================ */


  /* ============================================================
   * 1. 从充电站退回充电T
   * ============================================================ */

  goBackward(900);             // 与进入充电站的900ms对应
  delay(200);

  spinRight(500);             // 原地掉头约180°
  turnUntilLine(2);
  delay(200);

  /* ============================================================
   * 2. 从充电T向拓展区走一点，到1号光刻机观察位置
   *
   * 这一段地图上比较短，先估400ms
   * ============================================================ */


  // ---- 看1号光刻机 ----
  spinLeft(750);              // 左转约90°
  goStraight(1000);
  delay(200);
  spinRight(700); 
  inspectLithographyMachine(1);
  spinRight(700);  
               // 回到原道路朝向
  delay(200);
  lineFollowJunction(1);
  goStraight(300); 
  turnUntilLine(1);  
  



  /* ============================================================
   * 3. 按原路线：回正后左转90°
   * ============================================================ */

  lineFollowJunction(2);
  goStraight(300); 
  turnUntilLine(2);            // 光电右转，重新找到横向道路


  /* ============================================================
   * 4. 前进到2号、4号之间的共同观察位置
   *
   * 根据地图比例：
   * 这一段约为4区完整1000ms路段的不到一半
   * 第一版先用450ms
   * ============================================================ */

  lineFollowTime(1200);
  delay(200);


  /* ============================================================
   * 5. 同一位置观察2号和4号
   *
   * 当前先右转看2号
   * 再转180°看道路另一侧的4号
   * 最后再转90°恢复原行驶方向
   * ============================================================ */

  // ---- 看2号 ----
  spinRight(720);
  inspectLithographyMachine(2);
  turnUntilLine(2);
  delay(200);

  // ---- 从2号直接转向4号 ----
  spinRight(710);
  inspectLithographyMachine(4);
  turnUntilLine(2);
  delay(200);

  // ---- 从4号恢复到原来的道路朝向 ----


  /* ============================================================
   * 6. 继续巡线到十字路口，然后右转
   *
   * 不用时间估算距离，让外侧光电找十字
   * ============================================================ */

  lineFollowJunction(2);       // 到十字
  goStraight(300);             // 向十字中心前探
  turnUntilLine(2);            // 十字右转


  /* ============================================================
   * 7. 到下一个T字，按要求继续直行
   *
   * T字不转弯：
   * 找到横线 → 固定穿过 → 继续巡线
   * ============================================================ */

  lineFollowTime(1000); 
  lineFollowJunction(1);       // 到T字
  goStraight(1000);             // 直接通过T字
  delay(200);


  /* ============================================================
   * 8. T字后继续一小段，到3号观察位置
   *
   * 第一版按地图比例估450ms
   * ============================================================ */




  /* ============================================================
   * 9. 看3号
   *
   * 你已经确认采用B方案：
   * 右转90看3号
   * 再继续右转90
   * 两次右转合计180°，直接形成返程方向
   * ============================================================ */

  spinRight(700);
  goStraight(150); 
  inspectLithographyMachine(3);
  spinRight(700);              // 不回正，继续右转，直接掉头
    delay(200);


  /* ============================================================
   * 10. 原路返回刚才那个T字
   * ============================================================ */

  lineFollowJunction(1);
  goStraight(300);
  lineFollowTime(1000);              // 直行穿过T，不转


  /* ============================================================
   * 11. 继续返回十字路口，然后左转
   * ============================================================ */

  lineFollowJunction(1);       // 到十字
  goStraight(300);
  turnUntilLine(1);            // 十字左转


  /* ============================================================
   * 12. 直行到下一个L型，然后右转
   *
   * L型前探沿用你当前已经测试过的200ms思路
   * ============================================================ */

  lineFollowJunction(1);       // 找L型
  goStraight(400);             // 进入L型拐点
  spinRight(800);           // L型右转


  /* ============================================================
   * 13. L右转以后继续前进
   *
   * 原路线这里会去单独看4号。
   * 现在4号已经在步骤5完成，所以这里直接通过。
   *
   * 根据地图比例先分成两段，方便实车单独调：
   * 第一段：到原4号附近
   * 第二段：继续向最后两个光刻机方向
   * ============================================================ */

  goStraight(2400);
  delay(200);


  /* ============================================================
   * 14. 看5号
   *
   * 按之前确定的路线：
   * 左转看5号 → 回正
   * ============================================================ */

  spinLeft(700);
  inspectLithographyMachine(5);
  spinRight(1300);


  /* ============================================================
   * 15. 从5号继续到6号
   *
   * 地图上两台之间不远，先估450ms
   * ============================================================ */

  goStraight(450);


  /* ============================================================
   * 16. 看6号，完成拓展任务
   * ============================================================ */

  inspectLithographyMachine(6);

  cameraServo_13.write(CAMERA_UP_ANGLE);
  Serial.println(F("Camera servo D13: UP 0 degrees"));
  delay(1000);
  stopProgram();
}
