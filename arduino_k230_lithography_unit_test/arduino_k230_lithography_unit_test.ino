#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================
 * K230光刻机状态牌识别与比赛鸣笛独立测试
 *
 * 两位状态按[IO32 IO33]解释。本测试使用11长脉冲：
 *   11保持1200ms = 光刻机状态牌
 *   11保持500ms  = 晶圆桶形状（本测试不使用）
 *
 * 接线保持不变：
 *   Nano D11-S --5V转3.3V分压--> K230 IO33
 *   Nano A4-S  --5V转3.3V分压--> K230 IO32
 *   K230 IO9/UART1_TX ----------> Nano D12/SoftwareSerial RX
 *   Nano GND --------------------> K230 GND
 *   D11/A4接口的5V均不接，K230 IO10物理不接。
 *
 * K230必须运行12_four_task_vision_service.py，返回：
 *   BUSY,LITHO
 *   RESULT,LITHO,RED|GREEN|U,<largest_pixels>
 *
 * 比赛动作：红色故障牌鸣笛2声；绿色正常牌鸣笛1声。
 * ============================================================ */

const uint8_t K230_RX_PIN = 12;
const uint8_t K230_UNUSED_TX_PIN = A5;
const uint8_t TASK_BIT0_PIN = 11;  // IO33
const uint8_t TASK_BIT1_PIN = A4;  // IO32
const uint8_t BUZZER_PIN = 8;

const unsigned long K230_BAUDRATE = 57600;
const unsigned long USB_BAUDRATE = 115200;
const unsigned long LITHOGRAPHY_TRIGGER_HIGH_MS = 1200;
const unsigned long RESPONSE_TIMEOUT_MS = 4500;
const unsigned long TEST_INTERVAL_MS = 5000;

SoftwareSerial k230Serial(K230_RX_PIN, K230_UNUSED_TX_PIN);

void setTaskCode(uint8_t code) {
  digitalWrite(TASK_BIT0_PIN, (code & 0x01) ? HIGH : LOW);
  digitalWrite(TASK_BIT1_PIN, (code & 0x02) ? HIGH : LOW);
}

void clearK230Input() {
  while (k230Serial.available() > 0) {
    k230Serial.read();
  }
}

void buzzerAlarm(uint8_t count, unsigned int intervalMs) {
  for (uint8_t index = 0; index < count; index++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(intervalMs);
    digitalWrite(BUZZER_PIN, LOW);
    delay(intervalMs);
  }
}

bool parseLithographyResult(
    char *line,
    char *color,
    size_t colorSize,
    unsigned long *largestPixels
) {
  char *messageType = strtok(line, ",");
  char *taskType = strtok(NULL, ",");
  char *colorText = strtok(NULL, ",");
  char *pixelsText = strtok(NULL, ",");

  if (messageType == NULL || strcmp(messageType, "RESULT") != 0 ||
      taskType == NULL || strcmp(taskType, "LITHO") != 0 ||
      colorText == NULL || pixelsText == NULL) {
    return false;
  }

  strncpy(color, colorText, colorSize - 1);
  color[colorSize - 1] = '\0';
  *largestPixels = strtoul(pixelsText, NULL, 10);
  return true;
}

bool requestLithography(
    char *color,
    size_t colorSize,
    unsigned long *largestPixels
) {
  char line[80];
  uint8_t lineLength = 0;

  setTaskCode(0);
  delay(100);
  clearK230Input();

  setTaskCode(3);  // [IO32 IO33] = 11
  delay(LITHOGRAPHY_TRIGGER_HIGH_MS);
  setTaskCode(0);

  Serial.println(F("TASK -> 11 LONG LITHO (1200ms)"));
  unsigned long startedAt = millis();

  while (millis() - startedAt < RESPONSE_TIMEOUT_MS) {
    while (k230Serial.available() > 0) {
      char received = (char)k230Serial.read();

      if (received == '\n') {
        if (lineLength == 0) {
          continue;
        }
        if (line[lineLength - 1] == '\r') {
          lineLength--;
        }
        line[lineLength] = '\0';

        Serial.print(F("RX <- "));
        Serial.println(line);

        if (strncmp(line, "RESULT,LITHO,", 13) == 0) {
          return parseLithographyResult(
              line,
              color,
              colorSize,
              largestPixels
          );
        }
        lineLength = 0;

      } else if (lineLength < sizeof(line) - 1) {
        line[lineLength++] = received;
      } else {
        lineLength = 0;
      }
    }
  }

  Serial.println(F("TIMEOUT: no LITHO result received"));
  return false;
}

void setup() {
  Serial.begin(USB_BAUDRATE);
  k230Serial.begin(K230_BAUDRATE);
  k230Serial.listen();

  pinMode(TASK_BIT0_PIN, OUTPUT);
  pinMode(TASK_BIT1_PIN, OUTPUT);
  setTaskCode(0);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println(F("K230 lithography color+buzzer unit test ready."));
  Serial.println(F("RED -> buzzer x2; GREEN -> buzzer x1."));
  delay(2000);
}

void loop() {
  char color[8] = "U";
  unsigned long largestPixels = 0;

  if (requestLithography(color, sizeof(color), &largestPixels)) {
    Serial.print(F("LITHO COLOR = "));
    Serial.print(color);
    Serial.print(F("  PIXELS = "));
    Serial.println(largestPixels);

    if (strcmp(color, "RED") == 0) {
      Serial.println(F("RED fault sign: buzzer x2"));
      buzzerAlarm(2, 150);
    } else if (strcmp(color, "GREEN") == 0) {
      Serial.println(F("GREEN normal sign: buzzer x1"));
      buzzerAlarm(1, 150);
    } else {
      Serial.println(F("Unknown sign: no buzzer"));
    }
  }

  delay(TEST_INTERVAL_MS);
}
