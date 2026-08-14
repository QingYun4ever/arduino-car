#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================
 * K230晶圆桶形状识别：IO32/IO33二进制11触发测试
 *
 * 两位状态按[IO32 IO33]解释：
 *   00 空闲
 *   01 性别（D11高，A4低）
 *   10 消防颜色（A4高，D11低）
 *   11 晶圆桶形状（D11和A4同时高）
 *
 * 接线保持不变：
 *   Nano D11-S --5V转3.3V分压--> K230 IO33
 *   Nano A4-S  --5V转3.3V分压--> K230 IO32
 *   K230 IO9/UART1_TX ----------> Nano D12/SoftwareSerial RX
 *   Nano GND --------------------> K230 GND
 *   D11/A4接口的5V均不接，K230 IO10物理不接。
 *
 * K230返回：
 *   BUSY,SHAPE
 *   RESULT,SHAPE,Cylinder|tube|U,<score_percent>
 * ============================================================ */

const uint8_t K230_RX_PIN = 12;
const uint8_t K230_UNUSED_TX_PIN = A5;
const uint8_t TASK_BIT0_PIN = 11;  // IO33
const uint8_t TASK_BIT1_PIN = A4;  // IO32

const unsigned long K230_BAUDRATE = 57600;
const unsigned long USB_BAUDRATE = 115200;
const unsigned long TASK_HIGH_MS = 500;
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

bool parseShapeResult(
    char *line,
    char *label,
    size_t labelSize,
    uint8_t *scorePercent
) {
  char *messageType = strtok(line, ",");
  char *taskType = strtok(NULL, ",");
  char *labelText = strtok(NULL, ",");
  char *scoreText = strtok(NULL, ",");

  if (messageType == NULL || strcmp(messageType, "RESULT") != 0 ||
      taskType == NULL || strcmp(taskType, "SHAPE") != 0 ||
      labelText == NULL || scoreText == NULL) {
    return false;
  }

  strncpy(label, labelText, labelSize - 1);
  label[labelSize - 1] = '\0';

  long parsedScore = strtol(scoreText, NULL, 10);
  if (parsedScore < 0) {
    parsedScore = 0;
  }
  if (parsedScore > 100) {
    parsedScore = 100;
  }
  *scorePercent = (uint8_t)parsedScore;
  return true;
}

bool requestShape(
    char *label,
    size_t labelSize,
    uint8_t *scorePercent
) {
  char line[80];
  uint8_t lineLength = 0;

  setTaskCode(0);
  delay(100);
  clearK230Input();

  setTaskCode(3);  // [IO32 IO33] = 11
  delay(TASK_HIGH_MS);
  setTaskCode(0);

  Serial.println(F("TASK -> 11 SHAPE (IO32=1, IO33=1)"));
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

        if (strncmp(line, "RESULT,SHAPE,", 13) == 0) {
          return parseShapeResult(
              line,
              label,
              labelSize,
              scorePercent
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

  Serial.println(F("TIMEOUT: no SHAPE result received"));
  return false;
}

void setup() {
  Serial.begin(USB_BAUDRATE);
  k230Serial.begin(K230_BAUDRATE);
  k230Serial.listen();

  pinMode(TASK_BIT0_PIN, OUTPUT);
  pinMode(TASK_BIT1_PIN, OUTPUT);
  setTaskCode(0);

  Serial.println(F("K230 wafer shape unit test ready."));
  Serial.println(F("Task bits [IO32 IO33] = 11."));
  delay(2000);
}

void loop() {
  char label[16] = "U";
  uint8_t scorePercent = 0;

  if (requestShape(label, sizeof(label), &scorePercent)) {
    Serial.print(F("SHAPE = "));
    Serial.print(label);
    Serial.print(F("  SCORE = "));
    Serial.print(scorePercent);
    Serial.println('%');
  }

  delay(TEST_INTERVAL_MS);
}
