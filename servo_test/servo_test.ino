/*
 * ============================================================================
 * servo_test.ino —— D9 单次舵机命令验证（Arduino Nano）
 * ============================================================================
 *
 * 本次已确认的实物接线：
 *   左机械臂舵机信号线  -> Nano D9
 *
 * 程序作用：上电只attach D9，保持默认水平姿态；按一次D10后，仅执行一次
 * Servo.write(50)。不控制右舵机、不控制TB6612电机、不自动重复。
 *
 * 供电必须满足：
 *   - 舵机红线：独立稳定的 5~6V 电源（大扭矩机械臂不能依赖Nano USB 5V）
 *   - 舵机棕/黑线：舵机电源 GND
 *   - Nano GND 和舵机电源 GND：必须共地
 *   - 拔掉USB后：Nano本身仍需要由VIN、5V或其他外部方式供电。
 *
 * D9 不与基础路线中的蜂鸣器引脚冲突；D10为启动按键（按下=HIGH）。
 *
 * 安全说明：
 *   上电不会写入任何指定角度。只有按D10后才执行唯一的一次 write(50)，
 *   用来观察“50”对左臂究竟是抬杆还是下杆。需要重新触发请复位/重新上电。
 * ============================================================================
 */

#include <Servo.h>

const uint8_t ARM_SERVO_PIN = 9;
const uint8_t PIN_START_BUTTON = 10;
const int TEST_ANGLE = 50;

Servo armServo;
bool commandHasRun = false;

bool startButtonPressed() {
  return digitalRead(PIN_START_BUTTON) == HIGH;
}

void setup() {
  pinMode(PIN_START_BUTTON, INPUT);
  // 默认水平就是机械臂的上电初始姿态，因此只attach，不调用Servo.write()。
  armServo.attach(ARM_SERVO_PIN);
}

void loop() {
  // 只在人工按D10后，对左臂发出唯一一条明确的舵机位置命令。
  if (!commandHasRun && startButtonPressed()) {
    delay(20); // 消抖
    if (startButtonPressed()) {
      armServo.write(TEST_ANGLE);
      commandHasRun = true;
    }
  }
  delay(10);
}
