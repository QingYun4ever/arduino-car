/*
 * ============================================================================
 * servo_test.ino —— D13 单舵机自动摆动验证（Arduino Nano）
 * ============================================================================
 *
 * 本次只测试的实物接线：
 *   右机械臂舵机信号线  -> Nano D13
 *
 * 程序作用：上传后不需要USB串口，D13 舵机持续在20°~160°之间往返摆动。
 * 用它确认：D13信号引脚、舵机供电、共地和机械臂是否可以运动。
 *
 * 供电必须满足：
 *   - 舵机红线：独立稳定的 5~6V 电源（大扭矩机械臂不能依赖Nano USB 5V）
 *   - 舵机棕/黑线：舵机电源 GND
 *   - Nano GND 和舵机电源 GND：必须共地
 *   - 拔掉USB后：Nano本身仍需要由VIN、5V或其他外部方式供电。
 *
 * D13 同时连接板载LED；但这是独立测试程序，D13会专门输出Servo控制脉冲，
 * 不影响其他程序。
 *
 * 安全说明：
 *   先使机械臂周围无障碍、推拉杆未卡住。首次使用20°~160°避免撞限位；
 *   确认安全后，才把 SWEEP_MIN_ANGLE / SWEEP_MAX_ANGLE 改成 0 / 180。
 * ============================================================================
 */

#include <Servo.h>

const uint8_t ARM_SERVO_PIN = 13;

const int SWEEP_MIN_ANGLE = 20;
const int SWEEP_MAX_ANGLE = 160;
const int SWEEP_STEP_DEGREES = 1;
const unsigned long SWEEP_STEP_MS = 20;
const unsigned long ENDPOINT_HOLD_MS = 300;

Servo armServo;
int sweepAngle = 90;
int sweepDirection = 1;

void writeArmAngle(int angle) {
  armServo.write(angle);
}

void setup() {
  armServo.attach(ARM_SERVO_PIN);

  // 上电后先停在中间位置，再开始缓慢摆动。
  writeArmAngle(sweepAngle);
  delay(600);
}

void loop() {
  writeArmAngle(sweepAngle);
  delay(SWEEP_STEP_MS);

  sweepAngle += sweepDirection * SWEEP_STEP_DEGREES;
  if (sweepAngle >= SWEEP_MAX_ANGLE) {
    sweepAngle = SWEEP_MAX_ANGLE;
    sweepDirection = -1;
    delay(ENDPOINT_HOLD_MS);
  } else if (sweepAngle <= SWEEP_MIN_ANGLE) {
    sweepAngle = SWEEP_MIN_ANGLE;
    sweepDirection = 1;
    delay(ENDPOINT_HOLD_MS);
  }
}
