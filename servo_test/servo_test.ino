/*
 * ============================================================================
 * servo_test.ino —— D9 机械臂舵机调用验证程序（Arduino Nano）
 * ============================================================================
 *
 * 用途：单独验证 Servo 库、D9 信号线和机械臂可达角度。
 * 本程序不控制电机、不循迹、不执行比赛路线。
 *
 * 接线：
 *   舵机信号线（通常黄/橙/白） → Nano D9
 *   舵机VCC（通常红）          → 独立稳定的5~6V电源
 *   舵机GND（通常棕/黑）        → 电源GND，并与Nano GND共地
 *
 * 注意：
 *   1. 不要用 Nano 的 USB 5V 直接带动大扭矩机械臂舵机；容易掉电复位。
 *   2. 先让机械臂周围没有碰撞物、推拉杆没有卡住，再测试。
 *   3. 旧 pathfinder.ino 的初始位置为 180°，本程序沿用该安全起点。
 *   4. 上传后打开“串口监视器”，波特率设为 115200，换行选择“新行”。
 *
 * 串口命令：
 *   0~180 + 回车 : 转到指定角度，例如输入 90
 *   h            : 显示帮助
 *   p            : 显示当前目标角度
 *   c            : 依次测试 180° → 90° → 0° → 90° → 180°（慢速）
 *   r            : 回到 180° 初始/收回位置
 *   x            : 停止输出（detach，舵机不再保持扭矩）
 *   e            : 恢复输出，并回到 180°
 * ============================================================================
 */

#include <Servo.h>

const uint8_t SERVO_PIN = 9;
const int SAFE_HOME_ANGLE = 180;  // 与旧 pathfinder.ino 一致；实测后可改
const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const unsigned long SETTLE_MS = 800;

Servo armServo;
int currentAngle = SAFE_HOME_ANGLE;
bool servoAttached = false;

void printHelp() {
  Serial.println();
  Serial.println(F("----- D9 舵机测试命令 -----"));
  Serial.println(F("输入 0~180 并回车：转到指定角度，例如 90"));
  Serial.println(F("h：帮助   p：当前角度   c：180-90-0-90-180慢速测试"));
  Serial.println(F("r：回到180°   x：停止输出(detach)   e：恢复输出"));
  Serial.println(F("---------------------------"));
}

int clampAngle(int angle) {
  if (angle < MIN_ANGLE) return MIN_ANGLE;
  if (angle > MAX_ANGLE) return MAX_ANGLE;
  return angle;
}

void attachServoIfNeeded() {
  if (!servoAttached) {
    armServo.attach(SERVO_PIN);
    servoAttached = true;
    Serial.println(F("D9 舵机输出已恢复。"));
  }
}

void moveToAngle(int requestedAngle) {
  int targetAngle = clampAngle(requestedAngle);
  attachServoIfNeeded();

  Serial.print(F("D9 舵机: "));
  Serial.print(currentAngle);
  Serial.print(F("° -> "));
  Serial.print(targetAngle);
  Serial.println(F("°"));

  armServo.write(targetAngle);
  currentAngle = targetAngle;
  delay(SETTLE_MS);  // 留足机械臂完成到位的时间
}

void runSlowRangeTest() {
  Serial.println(F("开始慢速范围测试；如发生卡住、碰撞或异响，立即断开舵机电源。"));
  moveToAngle(180);
  delay(500);
  moveToAngle(90);
  delay(500);
  moveToAngle(0);
  delay(500);
  moveToAngle(90);
  delay(500);
  moveToAngle(180);
  Serial.println(F("范围测试完成，已回到180°。"));
}

void detachServo() {
  if (servoAttached) {
    armServo.detach();
    servoAttached = false;
    Serial.println(F("D9 舵机已停止输出；机械臂现在可以被手动移动。"));
  } else {
    Serial.println(F("D9 舵机当前已经是 detach 状态。"));
  }
}

void handleCommand(char command) {
  switch (command) {
    case 'h':
    case 'H':
    case '?':
      printHelp();
      break;

    case 'p':
    case 'P':
      Serial.print(F("当前目标角度: "));
      Serial.print(currentAngle);
      Serial.println(F("°"));
      break;

    case 'c':
    case 'C':
      runSlowRangeTest();
      break;

    case 'r':
    case 'R':
      moveToAngle(SAFE_HOME_ANGLE);
      break;

    case 'x':
    case 'X':
      detachServo();
      break;

    case 'e':
    case 'E':
      attachServoIfNeeded();
      moveToAngle(SAFE_HOME_ANGLE);
      break;

    case '\r':
    case '\n':
    case ' ':
      break; // 忽略串口监视器发送的换行和空格

    default:
      Serial.println(F("未知命令；输入 h 查看帮助。"));
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // 明确写入180°，与旧比赛程序的机械臂初始位置保持一致。
  attachServoIfNeeded();
  moveToAngle(SAFE_HOME_ANGLE);

  Serial.println(F("servo_test 已启动：D9 机械臂舵机验证。"));
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  // 数字命令：直接输入角度，例如“135”后回车。
  char first = Serial.peek();
  if (first >= '0' && first <= '9') {
    int requestedAngle = Serial.parseInt();
    moveToAngle(requestedAngle);
    return;
  }

  // 字母命令：每次取一个字符处理。
  handleCommand(Serial.read());
}
