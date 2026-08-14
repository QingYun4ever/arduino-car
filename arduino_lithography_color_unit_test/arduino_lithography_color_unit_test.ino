#include <Servo.h>

/* ============================================================
 * 光刻机颜色识别配套摄像机低头测试
 *
 * D13：摄像机舵机
 *   0°  = 抬头
 *   24° = 低头
 *
 * 上传后：先保持0°一秒，再转到24°并持续保持。
 * 同时在K230运行09_lithography_color_preview.py进行颜色识别。
 * ============================================================ */

const uint8_t CAMERA_SERVO_PIN = 13;
const uint8_t CAMERA_UP_ANGLE = 0;
const uint8_t CAMERA_DOWN_ANGLE = 24;

Servo cameraServo;

void setup() {
  Serial.begin(115200);

  cameraServo.attach(CAMERA_SERVO_PIN);
  cameraServo.write(CAMERA_UP_ANGLE);
  Serial.println(F("Camera servo: UP 0 degrees"));
  delay(1000);

  cameraServo.write(CAMERA_DOWN_ANGLE);
  Serial.println(F("Camera servo: DOWN 24 degrees"));
  Serial.println(F("Hold this position for lithography color test."));
  delay(1000);
}

void loop() {
  // 保持舵机连接并维持24°，不重复摆动。
  delay(1000);
}
