/*
 * Arduino Nano -> K230 IO33 trigger unit test.
 * K230 replies through its existing one-way UART link.
 *
 * Wiring:
 *   Nano D11 -> 5 V to 3.3 V divider/level shifter -> K230 IO33
 *   K230 IO9 UART1_TXD (3.3 V) -> Nano D12 RX
 *   Nano GND -> K230 GND
 *
 * Run on K230:
 *   k230_gender_unit_test/04_gpio_trigger_uart_reply.py
 */

#include <SoftwareSerial.h>
#include <string.h>

const uint8_t K230_RX_PIN = 12;
const uint8_t UNUSED_SOFTWARE_TX_PIN = A5;
const uint8_t K230_TRIGGER_PIN = 11;

const unsigned long USB_BAUDRATE = 115200;
const unsigned long K230_BAUDRATE = 57600;
const unsigned long TRIGGER_INTERVAL_MS = 2000;
const unsigned long RESPONSE_TIMEOUT_MS = 1000;
const unsigned long TRIGGER_HIGH_MS = 100;

SoftwareSerial k230Serial(K230_RX_PIN, UNUSED_SOFTWARE_TX_PIN);

char receiveBuffer[65];
uint8_t receiveLength = 0;
bool waitingForReply = false;
unsigned long triggerStartedAt = 0;
unsigned long lastTestFinishedAt = 0;
unsigned long triggerCount = 0;

void clearK230Input() {
  while (k230Serial.available() > 0) {
    k230Serial.read();
  }
  receiveLength = 0;
}

void sendTriggerPulse() {
  clearK230Input();

  triggerCount++;
  digitalWrite(K230_TRIGGER_PIN, HIGH);
  delay(TRIGGER_HIGH_MS);
  digitalWrite(K230_TRIGGER_PIN, LOW);

  triggerStartedAt = millis();
  waitingForReply = true;

  Serial.print(F("TRIGGER -> IO33 #"));
  Serial.println(triggerCount);
}

void processLine(const char *line) {
  Serial.print(F("RX <- "));
  Serial.println(line);

  if (strcmp(line, "TRIGGER_OK") == 0) {
    waitingForReply = false;
    lastTestFinishedAt = millis();
    Serial.println(F("PASS: IO33 trigger and UART return are both working"));
    return;
  }

  if (strcmp(line, "TRIGGER_READY") == 0) {
    Serial.println(F("K230 trigger test is ready"));
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
  Serial.println(F("Arduino -> K230 IO33 trigger unit test"));
  Serial.println(F("First trigger will be sent after 2 seconds."));

  lastTestFinishedAt = millis();
}

void loop() {
  readK230Lines();

  unsigned long now = millis();

  if (waitingForReply &&
      now - triggerStartedAt >= RESPONSE_TIMEOUT_MS) {
    waitingForReply = false;
    lastTestFinishedAt = now;
    Serial.println(F("TIMEOUT: K230 did not acknowledge IO33 trigger"));
  }

  if (!waitingForReply &&
      now - lastTestFinishedAt >= TRIGGER_INTERVAL_MS) {
    sendTriggerPulse();
  }
}
