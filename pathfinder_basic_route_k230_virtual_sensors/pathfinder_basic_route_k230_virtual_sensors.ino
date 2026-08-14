/*
 * ============================================================================
 * pathfinder_basic_route_k230_virtual_sensors.ino
 * 高中组基础任务：Nano路线状态机 + K230正下视四路虚拟光电
 *
 * 独立派生自pathfinder_basic_route，不修改原基线。路线表、循迹、路口确认、
 * 转向找线和电机控制仍在Nano；A0~A3读取改为K230的S,s0,s1,s2,s3帧。
 * ============================================================================
 *
 * 本文件是全新编写的工作草案：只把旧 pathfinder.ino 作为“引脚接线和
 * 已验证的电机极性”参考，不沿用旧程序的长时间硬编码动作序列。
 *
 * 【不可修改的基准】
 *   D:\dev\Robot\小车\src\pathfinder\pathfinder.ino
 * 【本工作版本】
 *   D:\dev\Robot\小车\src\pathfinder_basic_route\pathfinder_basic_route.ino
 *
 * --------------------------------------------------------------------------
 * 当前功能边界（按确认后的比赛基础路线）
 * --------------------------------------------------------------------------
 *   起点（地图最左端黑框）
 *     → 车间5区 → 车间4区 → 仓库/仓门区域
 *     → 上方重点消防（灭火器）区域 → 车间6区
 *     → 右上角充电站（终点）
 *
 * 每个中途目标点的验证动作：
 *   1. 车身留在黑线位置；2. 原地转向面向物件；3. 停2秒；4. 蜂鸣器2声；
 *   5. 原地转回，重新接入原来的黑线。
 * 充电站为终点：面向充电站、停2秒、滴2声后，保持停车。
 *
 * 不做：灭火、仓门机械臂、物品管理、摄像头识别、随机标牌识别、设备巡检。
 *
 * --------------------------------------------------------------------------
 * 硬件接线（Arduino Nano + TB6612FNG + 四路光电传感器）
 * --------------------------------------------------------------------------
 * D2/D4/D3 : 左电机 AIN1 / AIN2 / PWMA
 * D5/D7/D6 : 右电机 BIN1 / BIN2 / PWMB
 * A0~A3    : 光电传感器（A0左外、A1中左、A2中右、A3右外）
 * D8       : 蜂鸣器（本程序保持与原代码相同：HIGH 为一次鸣笛）
 * D10      : 启动按键（HIGH表示按下）
 * D11      : K230 UART1 TX(GPIO9)输入，57600波特率
 * D12      : SoftwareSerial占位TX，保持不连接K230
 *
 * 【接线脚已统一为D11】
 * 本项目曾同时存在两套互不兼容的配对，是"车完全不动"的直接原因之一：
 *   k230_uart_motor_receiver.ino        协议 M,L,R   接收脚 D11
 *   本文件（旧版）                        协议 S,s..   接收脚 D12
 * 已实车跑通的 k230_gray_first_cross_right_turn_test 走的是 D11，
 * 因此这里统一到 D11，硬件不用再动线。
 * 配套的K230脚本必须是 k230_gray_virtual_4sensor_test.py（发送 S,s0,s1,s2,s3）。
 * 若K230上跑的是 v6/v7/v8 或第一个十字单测（发送 M,L,R），本程序会永远
 * 停在setup()等待，车不会动 —— 两端协议必须成对。
 *
 * --------------------------------------------------------------------------
 * 第一次实车前必须做的事
 * --------------------------------------------------------------------------
 * 1. 确认 TB6612 的 STBY 为高、VM 接电机电池。
 * 2. 上传后按串口提示：白底按键采样 → 黑线按键采样 → 按键出发。
 * 3. 先把 ROUTE_LAST_STEP 设为较小值逐段试跑；确认一段再放开一段。
 * 4. 按实车结果只调“可调参数”区的时间、速度、补偿；不要用盲目长延时
 *    替代循迹。主路线转弯始终靠传感器重新找到黑线。
 * ============================================================================
 */

#include <SoftwareSerial.h>

/* ============================= 可调参数区 ================================= */

// ---- 引脚 ----
const uint8_t PIN_BUZZER = 8;
const uint8_t PIN_BUTTON = 10;

const uint8_t PIN_LEFT_IN1 = 2;
const uint8_t PIN_LEFT_IN2 = 4;
const uint8_t PIN_LEFT_PWM = 3;
const uint8_t PIN_RIGHT_IN1 = 5;
const uint8_t PIN_RIGHT_IN2 = 7;
const uint8_t PIN_RIGHT_PWM = 6;

// K230 TX接Nano D11（与已实车验证的第一个十字单测同一根线）。
// D12仅为SoftwareSerial占位TX，不接K230。
const uint8_t PIN_K230_RX = 11;
const uint8_t PIN_UNUSED_TX = 12;
const unsigned long K230_BAUD = 57600UL;
const unsigned long K230_SENSOR_TIMEOUT_MS = 220UL;
const uint8_t K230_RX_BUFFER_SIZE = 24;

// ---- 按键和蜂鸣器电平 ----
// 这两个电平沿用原 pathfinder.ino 的实际写法；若实物表现相反，只改这里。
const uint8_t BUTTON_ACTIVE_LEVEL = HIGH;
const uint8_t BUZZER_ON_LEVEL = HIGH;
const uint8_t BUZZER_OFF_LEVEL = LOW;

// ---- 电机与循迹 ----
const int BASE_SPEED = 145;             // 正常循迹速度（0~255）
const int RIGHT_SPEED_TRIM = 7;         // 右轮补偿：右轮速度 = BASE_SPEED + 本值
const int TURN_SPEED = 105;             // 路口原地转向速度
const int STEER_SPEED = 125;            // 发现偏线时，单轮修正的速度
const int TURN_ENTER_MS = 75;           // 进入路口中心后再开始原地转向
const int TURN_IGNORE_OLD_LINE_MS = 130;// 转向初期忽略当前旧线，防止过早判定成功
const int TURN_UTURN_IGNORE_MS = 430;   // 掉头至少旋转的时间
const int TURN_TIMEOUT_MS = 1800;       // 转向找不到新线时的安全超时
const int TURN_SETTLE_MS = 90;          // 找到新线后，低速向前稳定的时间
const int LOST_LINE_TIMEOUT_MS = 220;   // 所有传感器持续脱线的最大允许时间

// ---- 路口判定 ----
const int JUNCTION_MIN_TRAVEL_MS = 220; // 离开上一路口的最短时间，防重复检测
const int JUNCTION_TIMEOUT_MS = 8000;   // 找下一个路口最长时间，超时立即停车
const uint8_t JUNCTION_CONFIRM_COUNT = 5;

// K230端已完成灰度阈值和“ROI几乎填满”判断，Nano无需白/黑模拟标定。

// ---- 面向物件的原地旋转 ----
// 这是“原地朝物件转向”的角度标定值，不负责主路线转弯；主路线转弯靠找线。
const int FACE_QUARTER_TURN_MS = 265;    // 实车标定 90° 所需的原地转向时间
const int INSPECTION_HOLD_MS = 2000;     // 面向物件停留 2 秒
const int BEEP_ON_MS = 130;
const int BEEP_OFF_MS = 130;

// ---- 到目标物件的短距离定位时间（小车仍在循迹，不是盲开） ----
// 数值依据 map.png 的线网关系给出“第一版起点”；必须在实车上逐段微调。
const int MS_FROM_5_TURN_TO_WORKSHOP_5 = 520;
const int MS_FROM_4_TURN_TO_WORKSHOP_4 = 430;
const int MS_FROM_WAREHOUSE_TURN_TO_DOOR = 330;
const int MS_FROM_FIRE_TURN_TO_EXTINGUISHER = 260;
const int MS_FROM_6_BRANCH_TO_WORKSHOP_6 = 180;
const int MS_FROM_CHARGER_TURN_TO_END = 220;

/* =========================== 路线表控制区 ==================================
 * 0 = 只完成标定后停车（检查电机/传感器前可用）
 * 其余值 = 执行 route[] 中从第0步到该下标的步骤，随后安全停车。
 * 第一次建议写 8~12，只验证“起点→5区→4区”；确认后逐步增大。
 * 正常完整运行改成 ROUTE_STEP_COUNT - 1。
 */
const int ROUTE_LAST_STEP = 8;   // 默认只验证“起点→5区→4区→掉头”；确认后改为 -1 执行全程

/* ========================== 固定实现区 ===================================== */

enum class Turn : int8_t {
  STRAIGHT = 0,
  LEFT = 1,
  RIGHT = 2,
  UTURN = 3
};

enum class RouteAction : uint8_t {
  FOLLOW_TO_JUNCTION,
  TAKE_TURN,
  FOLLOW_FOR_MS,
  INSPECT_AND_RETURN,
  INSPECT_AND_STOP
};

struct RouteStep {
  RouteAction action;
  int value;               // 转向类型、循迹时间，或面向物件的相对转向类型
  // 固定为 nullptr：路线说明保留在源代码注释中，不能存普通字符串占用 Nano SRAM。
  const char *description;
};

/*
 * map.png 的线路拓扑（第一版路线草表）：
 *
 * 起点向东 → 左侧主十字口向南 → 5区（在线上朝西看）
 *            → 左下弯向西 → 4区 → 掉头返回
 *            → 回主线向东 → 中部下行/东行 → 仓门支线
 *            → 原路退出 → 中部上方消防支线
 *            → 回到中部 → 6区 → 右侧主线 → 北上 → 充电站。
 *
 * FOLLOW_TO_JUNCTION 只在检测到外侧传感器压线的路口/拐角后结束。
 * TAKE_TURN 会先进入路口，再原地转向，直到中间传感器重新找到下一段黑线。
 * 因此主线路段不会用固定转向时间“硬闯”而离开黑线。
 */
const RouteStep route[] = {
  // ---------- 起点 → 车间5区 ----------
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_FOR_MS,      MS_FROM_5_TURN_TO_WORKSHOP_5, nullptr},
  {RouteAction::INSPECT_AND_RETURN, (int)Turn::RIGHT,       nullptr},

  // ---------- 车间5区 → 车间4区 ----------
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_FOR_MS,      MS_FROM_4_TURN_TO_WORKSHOP_4, nullptr},
  {RouteAction::INSPECT_AND_RETURN, (int)Turn::LEFT,        nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::UTURN,       nullptr},

  // ---------- 车间4区 → 仓库/仓门 ----------
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::LEFT,        nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::LEFT,        nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_FOR_MS,      MS_FROM_WAREHOUSE_TURN_TO_DOOR, nullptr},
  {RouteAction::INSPECT_AND_RETURN, (int)Turn::RIGHT,       nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::UTURN,       nullptr},

  // ---------- 仓库 → 上方消防（灭火器） ----------
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::LEFT,        nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::LEFT,        nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::LEFT,        nullptr},
  {RouteAction::FOLLOW_FOR_MS,      MS_FROM_FIRE_TURN_TO_EXTINGUISHER, nullptr},
  {RouteAction::INSPECT_AND_RETURN, (int)Turn::RIGHT,       nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::UTURN,       nullptr},

  // ---------- 消防 → 车间6区 ----------
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::LEFT,        nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::LEFT,        nullptr},
  {RouteAction::FOLLOW_FOR_MS,      MS_FROM_6_BRANCH_TO_WORKSHOP_6, nullptr},
  {RouteAction::INSPECT_AND_RETURN, (int)Turn::LEFT,        nullptr},

  // ---------- 车间6区 → 右上角充电站 ----------
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::LEFT,        nullptr},
  {RouteAction::FOLLOW_TO_JUNCTION, JUNCTION_MIN_TRAVEL_MS, nullptr},
  {RouteAction::TAKE_TURN,          (int)Turn::RIGHT,       nullptr},
  {RouteAction::FOLLOW_FOR_MS,      MS_FROM_CHARGER_TURN_TO_END, nullptr},
  {RouteAction::INSPECT_AND_STOP,   (int)Turn::LEFT,        nullptr}
};

const int ROUTE_STEP_COUNT = sizeof(route) / sizeof(route[0]);
const int EFFECTIVE_ROUTE_LAST_STEP =
    (ROUTE_LAST_STEP < 0 || ROUTE_LAST_STEP >= ROUTE_STEP_COUNT)
        ? ROUTE_STEP_COUNT - 1
        : ROUTE_LAST_STEP;

SoftwareSerial k230Serial(PIN_K230_RX, PIN_UNUSED_TX);
char k230RxBuffer[K230_RX_BUFFER_SIZE];
uint8_t k230RxLength = 0;
bool k230DiscardingOverflow = false;
unsigned long lastK230SensorMs = 0;
bool k230SensorValid = false;

// 每解析成功一帧自增。K230每40ms发一帧(25Hz)，而本程序每3ms读一次(330Hz)，
// 同一帧会被重复读到十几次。路口确认必须只统计"新帧"，否则
// JUNCTION_CONFIRM_COUNT=5 会在15ms内被同一帧刷满，去抖完全失效。
uint16_t k230FrameSeq = 0;

// 仅用于setup()的诊断打印：区分"一个字节没收到"和"收到了但解析失败"。
unsigned long k230ByteCount = 0;
unsigned long k230BadFrameCount = 0;

struct SensorState {
  bool leftOuter;   // A0
  bool leftInner;   // A1
  bool rightInner;  // A2
  bool rightOuter;  // A3
};

/* ============================ 基础硬件函数 ================================= */

int clampPwm(int value) {
  if (value < 0) return 0;
  if (value > 255) return 255;
  return value;
}

// signedSpeed > 0 = 小车逻辑前进；signedSpeed < 0 = 小车逻辑后退。
// 两电机因安装方向相反，正向时两组方向脚的电平排列也相反。
void setMotors(int leftSignedSpeed, int rightSignedSpeed) {
  int leftPwm = clampPwm(abs(leftSignedSpeed));
  int rightPwm = clampPwm(abs(rightSignedSpeed));

  if (leftSignedSpeed >= 0) {             // 左轮前进：D2=L, D4=H（参考原程序）
    digitalWrite(PIN_LEFT_IN1, LOW);
    digitalWrite(PIN_LEFT_IN2, HIGH);
  } else {                                // 左轮后退
    digitalWrite(PIN_LEFT_IN1, HIGH);
    digitalWrite(PIN_LEFT_IN2, LOW);
  }

  if (rightSignedSpeed >= 0) {            // 右轮前进：D5=H, D7=L（参考原程序）
    digitalWrite(PIN_RIGHT_IN1, HIGH);
    digitalWrite(PIN_RIGHT_IN2, LOW);
  } else {                                // 右轮后退
    digitalWrite(PIN_RIGHT_IN1, LOW);
    digitalWrite(PIN_RIGHT_IN2, HIGH);
  }

  analogWrite(PIN_LEFT_PWM, leftPwm);
  analogWrite(PIN_RIGHT_PWM, rightPwm);
}

void stopMotors() {
  setMotors(0, 0);
}

void driveForward() {
  setMotors(BASE_SPEED, BASE_SPEED + RIGHT_SPEED_TRIM);
}

void rotateLeft() {
  setMotors(-TURN_SPEED, TURN_SPEED);
}

void rotateRight() {
  setMotors(TURN_SPEED, -TURN_SPEED);
}

void buzzerOn() {
  digitalWrite(PIN_BUZZER, BUZZER_ON_LEVEL);
}

void buzzerOff() {
  digitalWrite(PIN_BUZZER, BUZZER_OFF_LEVEL);
}

void beep(uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    buzzerOn();
    delay(BEEP_ON_MS);
    buzzerOff();
    delay(BEEP_OFF_MS);
  }
}

bool buttonPressed() {
  return digitalRead(PIN_BUTTON) == BUTTON_ACTIVE_LEVEL;
}

void waitForButtonPressAndRelease(const __FlashStringHelper *prompt) {
  Serial.println(prompt);
  stopMotors();
  while (!buttonPressed()) {
    delay(5);
  }
  delay(20);                               // 简单消抖
  while (buttonPressed()) {
    delay(5);
  }
  delay(120);
}

/* ========================== K230四路传感器输入 ============================= */

SensorState latestSensors = {false, false, false, false};

bool parseSensorFrame(char *line, SensorState &result) {
  // 严格格式：S,0,1,1,0；只接受0或1。
  if (line[0] != 'S' || line[1] != ',') return false;
  uint8_t value[4];
  uint8_t position = 2;
  for (uint8_t i = 0; i < 4; ++i) {
    if (line[position] != '0' && line[position] != '1') return false;
    value[i] = line[position] - '0';
    ++position;
    if (i < 3) {
      if (line[position] != ',') return false;
      ++position;
    }
  }
  if (line[position] == '\r') ++position;
  if (line[position] != '\0') return false;

  result.leftOuter = value[0];
  result.leftInner = value[1];
  result.rightInner = value[2];
  result.rightOuter = value[3];
  return true;
}

// 只负责收帧和解析，不碰电机。
// 早期版本在坏帧/溢出时直接 stopMotors()，会与 takeTurn() 每3ms下发的
// 旋转命令互相打架，造成转向抖动。安全性由 readSensors() 的超时统一负责。
void receiveK230Sensors() {
  while (k230Serial.available() > 0) {
    char incoming = (char)k230Serial.read();
    ++k230ByteCount;

    if (k230DiscardingOverflow) {
      if (incoming == '\n') {
        k230DiscardingOverflow = false;
        k230RxLength = 0;
      }
      continue;
    }

    if (incoming == '\n') {
      k230RxBuffer[k230RxLength] = '\0';
      SensorState parsed;
      if (parseSensorFrame(k230RxBuffer, parsed)) {
        latestSensors = parsed;
        lastK230SensorMs = millis();
        k230SensorValid = true;
        ++k230FrameSeq;
      } else {
        ++k230BadFrameCount;
        // 不在这里停车：坏帧只是不更新数据，超时逻辑会兜底。
      }
      k230RxLength = 0;
      continue;
    }

    if (k230RxLength >= K230_RX_BUFFER_SIZE - 1) {
      ++k230BadFrameCount;
      k230RxLength = 0;
      k230DiscardingOverflow = true;
      continue;
    }

    k230RxBuffer[k230RxLength++] = incoming;
  }
}

// 长时间等待期间必须持续排空SoftwareSerial：其RX缓冲区只有64字节，
// K230以25Hz发10字节/帧(约250字节/秒)，一次 delay(2000) 就会溢出约500字节，
// 恢复后还要额外丢弃残帧。用本函数替代裸 delay()，且不改动电机输出。
void delayWithSerialPump(unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    receiveK230Sensors();
    delay(1);
  }
}

// 等到K230送来一帧全新的数据。用于长动作（旋转、停留）之后，
// 避免下一步拿着动作开始前的旧状态去做判断。超时返回false。
bool waitForFreshSensorFrame(unsigned long timeoutMs) {
  uint16_t startSeq = k230FrameSeq;
  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    receiveK230Sensors();
    if (k230FrameSeq != startSeq) return true;
    delay(1);
  }
  return false;
}

SensorState readSensors() {
  receiveK230Sensors();

  if (!k230SensorValid || millis() - lastK230SensorMs > K230_SENSOR_TIMEOUT_MS) {
    stopMotors();
    SensorState lost = {false, false, false, false};
    return lost;
  }

  return latestSensors;
}

bool anySensorOnLine(const SensorState &sensors) {
  return sensors.leftOuter || sensors.leftInner ||
         sensors.rightInner || sensors.rightOuter;
}

bool innerSensorSeesLine(const SensorState &sensors) {
  return sensors.leftInner || sensors.rightInner;
}

bool looksLikeJunction(const SensorState &sensors) {
  // 正常细线主要由 A1/A2 检出；外侧 A0 或 A3 压线通常表示拐角/路口。
  return (sensors.leftOuter || sensors.rightOuter) && innerSensorSeesLine(sensors);
}

/* ================================ 循迹控制 ================================= */

// 返回 false 表示四路传感器都没有读到黑线。
bool followLineStep() {
  SensorState sensors = readSensors();

  if (!anySensorOnLine(sensors)) {
    stopMotors();
    return false;
  }

  // 与旧程序相同的两中间传感器判断，但封装为独立的循迹一步。
  if ((sensors.leftInner && sensors.rightInner) ||
      (!sensors.leftInner && !sensors.rightInner)) {
    driveForward();                          // 在线中央，或正越过较宽路口
  } else if (sensors.leftInner && !sensors.rightInner) {
    setMotors(STEER_SPEED, 0);               // 线偏左：左轮转、右轮停，向左修正
  } else {
    setMotors(0, STEER_SPEED + RIGHT_SPEED_TRIM); // 线偏右：左轮停、右轮转，向右修正
  }
  return true;
}

// 在仍然循迹的前提下持续行驶指定时间；连续失线则安全停车。
bool followFor(unsigned long durationMs) {
  unsigned long start = millis();
  unsigned long lostSince = 0;

  while (millis() - start < durationMs) {
    if (followLineStep()) {
      lostSince = 0;
    } else {
      if (lostSince == 0) lostSince = millis();
      if (millis() - lostSince > LOST_LINE_TIMEOUT_MS) {
        return false;
      }
    }
    delay(3);
  }
  stopMotors();
  return true;
}

// 沿当前黑线走到下一个路口或直角拐角。timeout 前找不到路口则失败停车。
bool followToJunction(unsigned long minTravelMs) {
  unsigned long start = millis();
  unsigned long lostSince = 0;
  uint8_t junctionHits = 0;
  uint16_t lastCountedSeq = k230FrameSeq;

  while (millis() - start < JUNCTION_TIMEOUT_MS) {
    SensorState sensors = readSensors();

    if (!anySensorOnLine(sensors)) {
      stopMotors();
      if (lostSince == 0) lostSince = millis();
      if (millis() - lostSince > LOST_LINE_TIMEOUT_MS) return false;
    } else {
      lostSince = 0;
      // 单独调用循迹逻辑，避免在路口前盲目高速冲出黑线。
      if ((sensors.leftInner && sensors.rightInner) ||
          (!sensors.leftInner && !sensors.rightInner)) {
        driveForward();
      } else if (sensors.leftInner) {
        setMotors(STEER_SPEED, 0);
      } else {
        setMotors(0, STEER_SPEED + RIGHT_SPEED_TRIM);
      }

      // 关键：只有真正收到新的一帧才参与路口计数。
      // 否则同一帧会在15ms内被重复统计5次，JUNCTION_CONFIRM_COUNT形同虚设，
      // 一帧噪声就能误判成路口。
      if (k230FrameSeq != lastCountedSeq) {
        lastCountedSeq = k230FrameSeq;
        if (millis() - start >= minTravelMs && looksLikeJunction(sensors)) {
          ++junctionHits;
          if (junctionHits >= JUNCTION_CONFIRM_COUNT) {
            stopMotors();
            return true;
          }
        } else {
          junctionHits = 0;
        }
      }
    }
    delay(3);
  }
  stopMotors();
  return false;
}

// 在已进入路口的位置原地转向，直到 A1/A2 中至少一路重新识别到下一段黑线。
bool takeTurn(Turn turn) {
  if (turn == Turn::STRAIGHT) {
    return followFor(TURN_SETTLE_MS);
  }

  // 左/右转时先让两中间传感器进入交叉区域中心，避免在原线段上误判成功。
  // 掉头常发生在支线尽头：不能再向前冲，否则会真的离开黑线，因此原地旋转。
  if (turn != Turn::UTURN) {
    driveForward();
    delay(TURN_ENTER_MS);
    stopMotors();
  }

  unsigned long start = millis();
  unsigned long minimumRotation =
      (turn == Turn::UTURN) ? TURN_UTURN_IGNORE_MS : TURN_IGNORE_OLD_LINE_MS;

  while (millis() - start < TURN_TIMEOUT_MS) {
    if (turn == Turn::LEFT || turn == Turn::UTURN) {
      rotateLeft();
    } else {
      rotateRight();
    }

    SensorState sensors = readSensors();
    if (millis() - start >= minimumRotation && innerSensorSeesLine(sensors)) {
      stopMotors();
      return followFor(TURN_SETTLE_MS);
    }
    delay(3);
  }

  stopMotors();
  return false;
}

// 相对物件的“看一眼”转向：只按已标定的90°时间旋转，车身位置留在黑线处。
// 旋转期间同样要排空串口，否则264ms就能填满SoftwareSerial的64字节缓冲区。
void rotateForFacing(Turn relativeTurn) {
  if (relativeTurn == Turn::LEFT) {
    rotateLeft();
    delayWithSerialPump(FACE_QUARTER_TURN_MS);
  } else if (relativeTurn == Turn::RIGHT) {
    rotateRight();
    delayWithSerialPump(FACE_QUARTER_TURN_MS);
  } else if (relativeTurn == Turn::UTURN) {
    rotateRight();
    delayWithSerialPump((unsigned long)FACE_QUARTER_TURN_MS * 2);
  }
  stopMotors();
}

Turn oppositeTurn(Turn turn) {
  if (turn == Turn::LEFT) return Turn::RIGHT;
  if (turn == Turn::RIGHT) return Turn::LEFT;
  return turn;
}

void inspectWaypointAndReturn(Turn faceTurn) {
  stopMotors();
  rotateForFacing(faceTurn);                 // 原地面向目标物件
  delayWithSerialPump(INSPECTION_HOLD_MS);   // 停2秒（期间持续排空串口）
  beep(2);                                   // 验证：滴滴两声
  rotateForFacing(oppositeTurn(faceTurn));   // 转回原先黑线方向
  stopMotors();
  // 转回后K230的数据可能还是转向途中的旧帧，等一帧新的再交给下一步。
  waitForFreshSensorFrame(K230_SENSOR_TIMEOUT_MS);
}

void inspectFinalAndStop(Turn faceTurn) {
  stopMotors();
  rotateForFacing(faceTurn);
  delayWithSerialPump(INSPECTION_HOLD_MS);
  beep(2);
  stopMotors();
  Serial.println(F("路线完成，已在充电站保持停车。"));
  while (true) {
    delay(100);
  }
}

void failSafeStop(int failedStep) {
  stopMotors();
  Serial.print(F("安全停车：路线第 "));
  Serial.print(failedStep);
  Serial.println(F(" 步失败。"));

  // 打印失败瞬间的四路状态，用于区分两类完全不同的故障：
  //   S=0000 且帧数一直在涨 -> K230虚拟传感器几何/阈值不对，正常在线上也不触发
  //                            （先跑 k230_gray_line_width_measure 重定参数）
  //   帧数不涨            -> 串口断了
  Serial.print(F("  最后状态 S="));
  Serial.print(latestSensors.leftOuter ? '1' : '0');
  Serial.print(latestSensors.leftInner ? '1' : '0');
  Serial.print(latestSensors.rightInner ? '1' : '0');
  Serial.print(latestSensors.rightOuter ? '1' : '0');
  Serial.print(F("  累计帧数="));
  Serial.print(k230FrameSeq);
  Serial.print(F("  坏帧="));
  Serial.println(k230BadFrameCount);
  if (k230FrameSeq > 0 && !anySensorOnLine(latestSensors)) {
    Serial.println(F("  -> 串口正常但四格全为0：虚拟传感器从未压到黑线，"
                     "是ROI几何/填充阈值问题，不是接线问题。"));
  }

  // 五次短鸣表示失线、超时或转向后找不到黑线；等待人工处理。
  while (true) {
    beep(5);
    delay(1000);
  }
}

/* ============================== 路线状态机 ================================= */

bool executeStep(const RouteStep &step) {
  switch (step.action) {
    case RouteAction::FOLLOW_TO_JUNCTION:
      return followToJunction((unsigned long)step.value);

    case RouteAction::TAKE_TURN:
      return takeTurn((Turn)step.value);

    case RouteAction::FOLLOW_FOR_MS:
      return followFor((unsigned long)step.value);

    case RouteAction::INSPECT_AND_RETURN:
      inspectWaypointAndReturn((Turn)step.value);
      return true;

    case RouteAction::INSPECT_AND_STOP:
      inspectFinalAndStop((Turn)step.value);
      return true; // 实际不会到达此行
  }
  return false;
}

void runRoute() {
  Serial.println(F("===== 开始基础任务循迹路线 ====="));
  for (int stepIndex = 0; stepIndex <= EFFECTIVE_ROUTE_LAST_STEP; ++stepIndex) {
    Serial.print(F("["));
    Serial.print(stepIndex);
    Serial.print(F("/"));
    Serial.print(ROUTE_STEP_COUNT - 1);
    Serial.println(F("]"));

    if (!executeStep(route[stepIndex])) {
      failSafeStop(stepIndex);
    }
  }

  // 用于逐段试跑：到设置的最后一步即停车，不会擅自执行后续未知路线。
  stopMotors();
  Serial.println(F("已达到 ROUTE_LAST_STEP，安全停车。"));
  while (true) {
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("pathfinder_basic_route + K230 virtual 4 sensors"));

  pinMode(PIN_BUZZER, OUTPUT);
  buzzerOff();
  pinMode(PIN_BUTTON, INPUT); // 保持原项目的外接按键接法（按下=HIGH）

  pinMode(PIN_LEFT_IN1, OUTPUT);
  pinMode(PIN_LEFT_IN2, OUTPUT);
  pinMode(PIN_LEFT_PWM, OUTPUT);
  pinMode(PIN_RIGHT_IN1, OUTPUT);
  pinMode(PIN_RIGHT_IN2, OUTPUT);
  pinMode(PIN_RIGHT_PWM, OUTPUT);
  stopMotors();

  k230Serial.begin(K230_BAUD);
  Serial.print(F("等待K230四路状态 S,s0,s1,s2,s3 @D"));
  Serial.print(PIN_K230_RX);
  Serial.print(F(" "));
  Serial.print(K230_BAUD);
  Serial.println(F(" baud ..."));

  // 这里绝不能是无声的死循环：旧版本一旦接线脚或协议不匹配，就会永远
  // 停在这一行，表现为"车完全不动、不响、串口没有任何后续输出"，
  // 无法判断是没收到数据还是收到了但解析失败。
  unsigned long waitStart = millis();
  unsigned long lastReport = millis();
  while (!k230SensorValid) {
    receiveK230Sensors();
    stopMotors();

    if (millis() - lastReport > 2000) {
      lastReport = millis();
      Serial.print(F("  等待中 "));
      Serial.print((millis() - waitStart) / 1000);
      Serial.print(F("s | 收到字节="));
      Serial.print(k230ByteCount);
      Serial.print(F(" 坏帧="));
      Serial.print(k230BadFrameCount);
      Serial.print(F(" 当前缓冲=["));
      k230RxBuffer[k230RxLength] = '\0';
      Serial.print(k230RxBuffer);
      Serial.println(F("]"));

      // 直接给出结论，不用再猜。
      if (k230ByteCount == 0) {
        Serial.println(F("  -> 一个字节都没收到：查K230 TX是否接在D11、"
                         "是否共地、K230脚本是否在运行。"));
      } else if (k230BadFrameCount > 0) {
        Serial.println(F("  -> 收到数据但解析失败：K230多半在发 M,L,R（v6/v8/"
                         "十字单测），本程序只认 S,s0,s1,s2,s3。"
                         "请改跑 k230_gray_virtual_4sensor_test.py。"));
      }
    }
    delay(2);
  }
  Serial.print(F("K230状态已收到（用时 "));
  Serial.print(millis() - waitStart);
  Serial.println(F(" ms），开始虚拟四路循迹路线。"));
}

void loop() {
  // setup() 完成标定后只运行一次完整状态机；完成/异常均在函数内保持停车。
  runRoute();
}
