#include <SoftwareSerial.h>
#include <string.h>
#include <stdlib.h>

/* ============================================================
 * K230灭火器颜色识别独立通信测试
 *
 * 接线：
 *   Nano A4-S --5V转3.3V分压--> K230 IO32（灭火器触发）
 *   K230 IO9/UART1_TX ----------> Nano D12（结果接收）
 *   Nano GND --------------------> K230 GND
 *   Nano A4接口的5V不接；K230 IO10物理不接。
 *
 * Arduino每5秒触发一次。K230返回：
 *   BUSY,FIRE
 *   RESULT,FIRE,RED|GREEN|BLUE|U,<largest_pixels>
 *
 * 识别到BLUE时，沿用比赛动作鸣笛2声。
 * ============================================================ */

const uint8_t K230_RX_PIN = 12;
const uint8_t K230_UNUSED_TX_PIN = A5;
const uint8_t FIRE_TRIGGER_PIN = A4;
const uint8_t BUZZER_PIN = 8;

const unsigned long K230_BAUDRATE = 57600;
const unsigned long USB_BAUDRATE = 115200;
const unsigned long TRIGGER_HIGH_MS = 500;
const unsigned long RESPONSE_TIMEOUT_MS = 4000;
const unsigned long TEST_INTERVAL_MS = 5000;

SoftwareSerial k230Serial(K230_RX_PIN, K230_UNUSED_TX_PIN);

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

bool parseFireResult(char *line, char *color, size_t colorSize,
                     unsigned long *largestPixels) {
  char *messageType = strtok(line, ",");
  char *taskType = strtok(NULL, ",");
  char *colorText = strtok(NULL, ",");
  char *pixelsText = strtok(NULL, ",");

  if (messageType == NULL || strcmp(messageType, "RESULT") != 0 ||
      taskType == NULL || strcmp(taskType, "FIRE") != 0 ||
      colorText == NULL || pixelsText == NULL) {
    return false;
  }

  strncpy(color, colorText, colorSize - 1);
  color[colorSize - 1] = '\0';
  *largestPixels = strtoul(pixelsText, NULL, 10);
  return true;
}

bool requestFireColor(char *color, size_t colorSize,
                      unsigned long *largestPixels) {
  char line[80];
  uint8_t lineLength = 0;

  clearK230Input();

  digitalWrite(FIRE_TRIGGER_PIN, HIGH);
  delay(TRIGGER_HIGH_MS);
  digitalWrite(FIRE_TRIGGER_PIN, LOW);

  Serial.println(F("TRIGGER -> K230 IO32 FIRE"));
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

        if (strncmp(line, "RESULT,FIRE,", 12) == 0) {
          return parseFireResult(
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

  Serial.println(F("TIMEOUT: no FIRE result received"));
  return false;
}

void setup() {
  Serial.begin(USB_BAUDRATE);
  k230Serial.begin(K230_BAUDRATE);
  k230Serial.listen();

  pinMode(FIRE_TRIGGER_PIN, OUTPUT);
  digitalWrite(FIRE_TRIGGER_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println(F("K230 extinguisher color unit test ready."));
  Serial.println(F("A4 -> IO32; IO9 -> D12; common GND."));
  delay(2000);
}

void loop() {
  char color[8] = "U";
  unsigned long largestPixels = 0;

  if (requestFireColor(color, sizeof(color), &largestPixels)) {
    Serial.print(F("FIRE COLOR = "));
    Serial.print(color);
    Serial.print(F("  PIXELS = "));
    Serial.println(largestPixels);

    if (strcmp(color, "BLUE") == 0) {
      Serial.println(F("BLUE extinguisher: buzzer x2"));
      buzzerAlarm(2, 150);
    }
  }

  delay(TEST_INTERVAL_MS);
}
