/*
 * ============================================================================
 * pathfinder_basic_route.ino
 * 高中组固定路线四光电循迹验证程序（第一版）
 * ============================================================================
 *
 * 本文件是全新编写的工作草案：只把旧 pathfinder.ino 作为“引脚接线和
 * 已验证的电机极性”参考，不沿用旧程序的长时间硬编码动作序列。
 *
 * 【已验证旧基线参考（仅用于接线和电机极性）】
 *   D:\dev\Robot\小车\src\pathfinder_commented\pathfinder_commented.ino
 * 【本工作版本】
 *   D:\dev\Robot\小车\src\pathfinder_basic_route\pathfinder_basic_route.ino
 *
 * --------------------------------------------------------------------------
 * 当前功能边界（固定路线验证，不等于完整比赛任务）
 * --------------------------------------------------------------------------
 *   起点（地图最左端黑框）
 *     → 车间5区 → 车间4区 → 仓库/仓门区域
 *     → 上方重点消防（灭火器）区域 → 车间6区
 *     → 右上角充电站（终点）
 *
 * 每个中途目标点的验证动作：
 *   1. 车身留在黑线位置；2. 原地转向面向物件；3. 停2秒；4. 蜂鸣器2声；
 *   5. 原地转回原先行进朝向；由后续 FOLLOW 步骤继续循迹并重新接入黑线。
 * 本验证程序中的充电站占位动作：面向充电站、停2秒、蜂鸣2声后保持停车。
 *
 * 不做：正式比赛的启动提示和差异化识别响应、仓门机械臂、货品管理、
 *       摄像头识别、随机策略物识别、拓展区设备巡检。

 *
 * --------------------------------------------------------------------------
 * 硬件接线（Arduino Nano + TB6612FNG + 四路光电传感器）
 * --------------------------------------------------------------------------
 * D2/D4/D3 : 左电机 AIN1 / AIN2 / PWMA
 * D5/D7/D6 : 右电机 BIN1 / BIN2 / PWMB
 * A0~A3    : 光电传感器（A0左外、A1中左、A2中右、A3右外）
 * D8       : 蜂鸣器（本程序保持与原代码相同：HIGH 为一次鸣笛）
 * D10      : 标定/启动按键（保持与原代码相同：HIGH 表示按下）
 *
 * --------------------------------------------------------------------------
 * 上传或分段复测前必须确认的事
 * --------------------------------------------------------------------------
 * 1. 确认 TB6612 的 STBY 为高、VM 接电机电池。
 * 2. 上传后按串口提示：白底按键采样 → 黑线按键采样 → 按键出发。
 * 3. 先把 ROUTE_LAST_STEP 设为较小值逐段试跑；确认一段再放开一段。
 * 4. 按实车结果只调“可调参数”区的时间、速度、补偿；不要用盲目长延时
 *    替代循迹。主路线转弯始终靠传感器重新找到黑线。
 * ============================================================================
 */

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

const uint8_t SENSOR_PIN[4] = {A0, A1, A2, A3};

// ---- 按键和蜂鸣器电平 ----
// 这两个电平沿用已验证旧基线的实际写法；若实物表现相反，只改这里。
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
const int TURN_SETTLE_MS = 90;          // 找到新线后，按正常循迹逻辑前行稳定的时间
const int LOST_LINE_TIMEOUT_MS = 220;   // 所有传感器持续脱线的最大允许时间

// ---- 路口判定 ----
const int JUNCTION_MIN_TRAVEL_MS = 220; // 每次查找开始后，允许确认路口前的最短行驶时间
const int JUNCTION_TIMEOUT_MS = 8000;   // 找下一个路口最长时间，超时立即停车
const uint8_t JUNCTION_CONFIRM_COUNT = 5;

// ---- 标定 ----
const uint8_t CALIBRATION_SAMPLES = 40;
const uint8_t MIN_CALIBRATION_DELTA = 18;  // 白/黑采样差太小则拒绝出发

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
 * 非负值 = 执行 route[] 中从第0步到该下标的步骤，随后安全停车。
 * 0 会执行第0步（从起点循迹到第一个路口），不是“只标定不移动”。
 * 写 7 只验证“起点→5区→4区”；写 8 会在此基础上继续完成掉头。
 * 确认后可逐步增大；完整运行可设为 -1 或 ROUTE_STEP_COUNT - 1。
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
 * FOLLOW_TO_JUNCTION 达到最短行驶时间后，需检测到外侧与内侧传感器同时
 * 压线，并连续确认指定次数，才把当前位置判定为路口/拐角。
 * TAKE_TURN 的左/右转会先进入路口再原地转向；掉头不前冲，直接原地转向。
 * 转向持续到中间传感器重新找到下一段黑线。
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

// 每路传感器的“白/黑”采样值和运行阈值。
int whiteValue[4] = {0, 0, 0, 0};
int blackValue[4] = {0, 0, 0, 0};
int thresholdValue[4] = {0, 0, 0, 0};
bool blackIsHigh[4] = {true, true, true, true};

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

/* ============================== 传感器标定 ================================= */

void sampleSurface(int destination[4]) {
  long sum[4] = {0, 0, 0, 0};
  for (uint8_t sample = 0; sample < CALIBRATION_SAMPLES; ++sample) {
    for (uint8_t i = 0; i < 4; ++i) {
      sum[i] += analogRead(SENSOR_PIN[i]);
    }
    delay(5);
  }
  for (uint8_t i = 0; i < 4; ++i) {
    destination[i] = sum[i] / CALIBRATION_SAMPLES;
  }
}

void printCalibration() {
  Serial.println(F("传感器标定结果: A0 A1 A2 A3"));
  Serial.print(F("  白: "));
  for (uint8_t i = 0; i < 4; ++i) { Serial.print(whiteValue[i]); Serial.print(' '); }
  Serial.println();
  Serial.print(F("  黑: "));
  for (uint8_t i = 0; i < 4; ++i) { Serial.print(blackValue[i]); Serial.print(' '); }
  Serial.println();
  Serial.print(F("阈值: "));
  for (uint8_t i = 0; i < 4; ++i) { Serial.print(thresholdValue[i]); Serial.print(' '); }
  Serial.println();
}

void calibrateLineSensors() {
  waitForButtonPressAndRelease(F("[标定1/3] 四个传感器全部放在白底，按D10按键。"));
  sampleSurface(whiteValue);
  beep(1);

  waitForButtonPressAndRelease(F("[标定2/3] 四个传感器全部放在黑线，按D10按键。"));
  sampleSurface(blackValue);

  for (uint8_t i = 0; i < 4; ++i) {
    thresholdValue[i] = (whiteValue[i] + blackValue[i]) / 2;
    blackIsHigh[i] = blackValue[i] > whiteValue[i];
    if (abs(blackValue[i] - whiteValue[i]) < MIN_CALIBRATION_DELTA) {
      Serial.print(F("标定失败: A"));
      Serial.print(i);
      Serial.println(F(" 的白/黑差异过小，请调整距离/光照后重新上电。"));
      stopMotors();
      while (true) {
        beep(3);
        delay(800);
      }
    }
  }
  printCalibration();
  beep(2);

  waitForButtonPressAndRelease(F("[标定3/3] 放回起点黑线，按D10按键后开始路线。"));
}

bool sensorOnBlackLine(uint8_t index) {
  int reading = analogRead(SENSOR_PIN[index]);
  return blackIsHigh[index] ? reading > thresholdValue[index]
                            : reading < thresholdValue[index];
}

SensorState readSensors() {
  SensorState sensors;
  sensors.leftOuter = sensorOnBlackLine(0);
  sensors.leftInner = sensorOnBlackLine(1);
  sensors.rightInner = sensorOnBlackLine(2);
  sensors.rightOuter = sensorOnBlackLine(3);
  return sensors;
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

// 相对物件的“看一眼”转向：左/右按90°标定时间，掉头按其2倍时间旋转。
void rotateForFacing(Turn relativeTurn) {
  if (relativeTurn == Turn::LEFT) {
    rotateLeft();
    delay(FACE_QUARTER_TURN_MS);
  } else if (relativeTurn == Turn::RIGHT) {
    rotateRight();
    delay(FACE_QUARTER_TURN_MS);
  } else if (relativeTurn == Turn::UTURN) {
    rotateRight();
    delay(FACE_QUARTER_TURN_MS * 2);
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
  delay(INSPECTION_HOLD_MS);                 // 停2秒
  beep(2);                                   // 验证：滴滴两声
  rotateForFacing(oppositeTurn(faceTurn));   // 转回原先黑线方向
  stopMotors();
}

void inspectFinalAndStop(Turn faceTurn) {
  stopMotors();
  rotateForFacing(faceTurn);
  delay(INSPECTION_HOLD_MS);
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
  Serial.println(F("pathfinder_basic_route: 高中组基础循迹路线"));

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

  calibrateLineSensors();
}

void loop() {
  // setup() 完成标定后只运行一次路线状态机；完成/异常均在函数内保持停车。
  runRoute();
}
