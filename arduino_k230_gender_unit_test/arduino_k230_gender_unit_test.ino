/*
 * Arduino Nano IO33-triggered K230 gender-result unit test.
 *
 * K230 runs:
 *   k230_gender_unit_test/05_face_gender_uart_service.py
 *
 * Signal flow:
 *   Nano D11 HIGH pulse -> divider/level shifter -> K230 IO33
 *   K230 IO9 UART1_TXD -> Nano D12 RX
 *   K230 sends BUSY\n, then RESULT,M|F|U,<confidence_percent>\n
 *
 * Wiring:
 *   Nano D11 -> 5 V to 3.3 V divider/level shifter -> K230 IO33
 *   K230 IO9 (UART1_TXD, 3.3 V) -> Nano D12 (RX)
 *   K230 GND -> Nano GND
 */

#include <SoftwareSerial.h>
#include <string.h>

const uint8_t K230_RX_PIN = 12;
const uint8_t UNUSED_SOFTWARE_TX_PIN = A5;
const uint8_t K230_TRIGGER_PIN = 11;

const unsigned long USB_BAUDRATE = 115200;
const unsigned long K230_BAUDRATE = 57600;
const unsigned long REQUEST_INTERVAL_MS = 5000;
const unsigned long RESPONSE_TIMEOUT_MS = 3500;
const unsigned long TRIGGER_HIGH_MS = 100;

SoftwareSerial k230Serial(K230_RX_PIN, UNUSED_SOFTWARE_TX_PIN);

char receiveBuffer[65];
uint8_t receiveLength = 0;
bool waitingForResult = false;
unsigned long requestStartedAt = 0;
unsigned long lastTestFinishedAt = 0;
unsigned long requestCount = 0;

void clearK230Input() {
  while (k230Serial.available() > 0) {
    k230Serial.read();
  }
  receiveLength = 0;
}

void triggerGenderRecognition() {
  clearK230Input();

  requestCount++;
  digitalWrite(K230_TRIGGER_PIN, HIGH);
  delay(TRIGGER_HIGH_MS);
  digitalWrite(K230_TRIGGER_PIN, LOW);

  requestStartedAt = millis();
  waitingForResult = true;

  Serial.print(F("TRIGGER -> IO33 #"));
  Serial.println(requestCount);
}

void processLine(const char *line) {
  Serial.print(F("RX <- "));
  Serial.println(line);

  if (strcmp(line, "BUSY") == 0) {
    Serial.println(F("K230 is measuring..."));
    return;
  }

  if (strncmp(line, "RESULT,", 7) == 0) {
    waitingForResult = false;
    lastTestFinishedAt = millis();

    Serial.println(F("Result format: RESULT,M/F/U,confidence"));
    Serial.println(F("M=male, F=female, U=unknown"));
    return;
  }

  if (strcmp(line, "K230_READY") == 0) {
    Serial.println(F("K230 gender service is ready"));
  }
}

void readK230Lines() {
  while (k230Serial.available() > 0) {
    char received = (char)k230Serial.read();

    if (received == '\r') {
      continue;
    }

    if (received == '\n') {
      if (receiveLength > 0) {
        receiveBuffer[receiveLength] = '\0';
        processLine(receiveBuffer);
        receiveLength = 0;
      }
    } else if (receiveLength < sizeof(receiveBuffer) - 1) {
      receiveBuffer[receiveLength++] = received;
    } else {
      receiveLength = 0;
      Serial.println(F("ERROR: received line was too long"));
    }
  }
}

void setup() {
  Serial.begin(USB_BAUDRATE);
  k230Serial.begin(K230_BAUDRATE);
  k230Serial.listen();

  pinMode(K230_TRIGGER_PIN, OUTPUT);
  digitalWrite(K230_TRIGGER_PIN, LOW);

  Serial.println();
  Serial.println(F("Arduino -> IO33 triggered K230 gender test"));
  Serial.println(F("Place a male/female sign in front of the camera."));
  Serial.println(F("The first trigger will be sent after 5 seconds."));

  lastTestFinishedAt = millis();
}

void loop() {
  readK230Lines();

  unsigned long now = millis();

  if (waitingForResult &&
      now - requestStartedAt >= RESPONSE_TIMEOUT_MS) {
    waitingForResult = false;
    lastTestFinishedAt = now;
    Serial.println(F("TIMEOUT: no RESULT received after IO33 trigger"));
  }

  if (!waitingForResult &&
      now - lastTestFinishedAt >= REQUEST_INTERVAL_MS) {
    triggerGenderRecognition();
  }
}
