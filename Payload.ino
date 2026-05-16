#include <BMP280.h>
#include <SoftwareSerial.h>
//#include <SD.h>
#include <LoRa.h>
#include <Servo.h>
#include <Wire.h>

// Servos
const Servo rightMotor;
const Servo leftMotor;
const byte defaultRightMotor = 128;
const byte defaultLeftMotor = 128;
const byte rolledPosition = 183;
const byte unrolledPosition = 73;
const byte downPosition = 200;
const byte upPosition = 50;
const byte servoDelay = 30;
unsigned long lastServoMove;

// GPS
const uint32_t GPSBaud = 115200;
SoftwareSerial GpsSerial(6, 4);
const char gpsHeader[] = "$GNGGA,";
float latitude;
float longitude;

// BMP
BMP280 bmp = BMP280(0x76);
const unsigned pressure = 1013, apogeeDelay = 10000;
BMP280::eStatus_t bmpStatus = BMP280::eStatusErr;
unsigned long lastStartAlt;
float startAlt;
float altitude;
unsigned long apogeeTime;

// SD Card
/*const char filename[] = "payload.csv";
File file;
String dataBuffer;
const byte chipSelect = 10;
bool sd = false;*/

// Rates
unsigned long lastTransmit;
const unsigned transmitDelay = 500, maxTries = 10;

// From RC
unsigned long lastSignal;
byte mode;
int rssi;
enum Direction {
  STRAIGHT,
  LEFT,
  RIGHT,
  DOWN,
  UP
};
Direction direction = STRAIGHT;

// LoRa
// byte buffer[4];

// LED
const uint8_t ledPin = 2;
unsigned long lastLedSeq;
uint8_t ledSeq = 0;
const unsigned fastBlink = 500, slowBlink = 1000;
unsigned ledSeqDelay = slowBlink;

// State
enum State {
  PREPARING = 1,
  READY = 2,
  TAKINGOFF = 3,
  GLIDING = 4
};
State state = PREPARING;
bool killSwitch = false;

void setup() {
  // Debug
  Serial.begin(9600);

  // Servos
  rightMotor.attach(5);
  leftMotor.attach(3);
  rightMotor.writeMicroseconds(map(defaultRightMotor, 0, 255, 1000, 2000) + 1);
  leftMotor.writeMicroseconds(map(defaultLeftMotor, 0, 255, 1000, 2000) + 1);

  // SD
  //dataBuffer.reserve(128);
  /*for (unsigned i = 0; i < maxTries && !sd; i++) {
    sd = SD.begin(chipSelect);
    Serial.print(".");
  }*/
  //if (sd) file = SD.open(filename, FILE_WRITE);

  // BMP
  Wire.begin();
  for (unsigned i = 0; i < maxTries && bmpStatus != BMP280::eStatusOK; i++) {
    bmpStatus = bmp.begin();
    Serial.print(",");
    delay(100);
  }
  if (bmpStatus == BMP280::eStatusOK) {
    bmp.setCtrlMeasMode(BMP280::eCtrlMeasModeNormal);
    bmp.setCtrlMeasSamplingTemp(BMP280::eSampling_X2);
    bmp.setCtrlMeasSamplingPress(BMP280::eSampling_X16);
    bmp.setConfigFilter(BMP280::eConfigFilter_X16);
    bmp.setConfigTStandby(BMP280::eConfigTStandby_250);
    startAlt = initialiseAlt();
    altitude = startAlt;
  }

  // LoRa
  LoRa.setPins(9, 7, 8);
  while (!LoRa.begin(433E6)) {
    Serial.print(";");
    delay(10);
  }
  LoRa.enableCrc();
  LoRa.setSyncWord(0xD5);
  //LoRa.setTxPower(20);

  // GPS
  //GpsSerial.begin(GPSBaud);

  // LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  delay(500);
  digitalWrite(ledPin, LOW);
}

void loop() {
  const unsigned long now = millis();

  /*if (sd) {
    const uint16_t chunkSize = file.availableForWrite();
    if (chunkSize && (dataBuffer.length() >= chunkSize || dataBuffer.length() >= 90)) {
      saveToSD(chunkSize);
    }
  }*/

  if ((now - lastTransmit) >= transmitDelay) {
    checkVerticalSpeed(now);
    //transmitToRC(now);
    lastTransmit = now;
  }

  const unsigned packetSize = LoRa.parsePacket();
  if (packetSize) {
    if (receiveFromRC()) {
      if (state == PREPARING) state = READY;
      //Serial.println("Receive " + String(now - lastSignal));
      lastSignal = now;
    }
    flushLoRa();
  }

  if (now - lastLedSeq >= ledSeqDelay && ledSeq != 2) {
    updateLed();
    lastLedSeq = now;
  }

  // Refresh base alt every 10 minutes
  if ((state == PREPARING || state == READY) && (now - lastStartAlt) >= 1000 * 60 * 10) {
    startAlt = getAltitude();

    lastStartAlt = now;
  }

  if ((now - lastServoMove) >= servoDelay) {
    updateServos();
    lastServoMove = now;
  }

  if (lastSignal && (now - lastSignal) >= 10000) {
    if (state == GLIDING) {
      direction = LEFT;
    }
    killSwitch = true;
  } else {
    killSwitch = false;
  }

  /*if (GpsSerial.available() > 0) {
    parseGpsData();
  }*/
}

int readInt(const Stream& str, const uint8_t length) {
  char buff[length + 1];
  str.readBytes(buff, length);
  buff[length] = "\0";
  return atoi(buff);
}

String readUntil(const Stream& str, const char ch) {
  String s;
  for (uint8_t i = 0; i < 15 && str.available() > 0; i++) {
    const char read = str.read();
    if (read == ch) {
      break;
    }
    s += read;
  }
  return s;
}

bool skipUntil(const Stream& str, const char ch) {
  for (uint8_t i = 0; i < 15 && str.available() > 0; i++) {
    if (str.read() == ch) {
      return true;
    }
  }
  return false;
}

bool findGpsHeader() {
  uint8_t seq = 0;
  while (GpsSerial.available() > 0 && seq < sizeof(gpsHeader) / sizeof(char) - 1) {
    const char ch = GpsSerial.read();
    if (ch == gpsHeader[seq]) {
      seq++;
    } else if (seq != 0) {
      seq = 0;
    }
  }
  return seq == sizeof(gpsHeader) / sizeof(char) - 1;
}

void moveServo(const Servo& serv, const int8_t delta) {
  const byte pos = getServoPosition(serv);
  if (pos + delta < 0 || pos + delta > 255) {
    return;
  }
  //Serial.println("Move servo");
  serv.writeMicroseconds(map(pos + delta, 0, 255, 1000, 2000) + delta);
}

void moveServoTo(const Servo& serv, const byte target) {
  const byte pos = getServoPosition(serv);
  if (pos < target) moveServo(serv, 1);
  else if (pos > target) moveServo(serv, -1);
}

byte getServoPosition(const Servo& serv) {
  return map(serv.readMicroseconds(), 1000, 2000, 0, 255);
}

void updateServos() {
  switch (direction) {
    case STRAIGHT:
      moveServoTo(rightMotor, defaultRightMotor);
      moveServoTo(leftMotor, defaultLeftMotor);
      break;
    case LEFT:
      moveServoTo(rightMotor, 255 - unrolledPosition);
      moveServoTo(leftMotor, rolledPosition);
      break;
    case RIGHT:
      moveServoTo(rightMotor, 255 - rolledPosition);
      moveServoTo(leftMotor, unrolledPosition);
      break;
    case DOWN:
      moveServoTo(rightMotor, 255 - downPosition);
      moveServoTo(leftMotor, downPosition);
      break;
    case UP:
      moveServoTo(rightMotor, 255 - upPosition);
      moveServoTo(leftMotor, upPosition);
  }
}

bool receiveFromRC() {
  if (LoRa.read() != 0xC4) return false;
  /*const byte rcData = LoRa.read();
  mode = rcData & 0b1;
  direction = rcData >> 1;*/
  mode = LoRa.parseInt();
  direction = LoRa.parseInt();

  rssi = LoRa.packetRssi();
  //Serial.println(rssi);

  return true;
}

void flushLoRa() {
  while (LoRa.available() > 0) {
    LoRa.read();
  }
}

void updateLed() {
  ledSeq = ledSeq == 0 ? 1 : 0;
  digitalWrite(ledPin, ledSeq);
  if (state == PREPARING) {
    ledSeqDelay = slowBlink;
  } else if (state == READY) {
    ledSeq = 2;
    digitalWrite(ledPin, HIGH);
  }
  if (killSwitch) {
    if (ledSeq == 2) ledSeq = 0;
    ledSeqDelay = fastBlink;
  }
}

float initialiseAlt() {
  delay(1000);
  float lastAlt;
  for (uint8_t i = 0; i < 10; i++) {
    const float currAlt = getAltitude();
    if (abs(lastAlt - currAlt) <= 1.0f) {
      return currAlt;
    }
    lastAlt = currAlt;
    delay(500);
  }
  return lastAlt;
}

float getAltitude() {
  return 44330.0f * (1.0f - pow((float)bmp.getPressure() / 100.0f / pressure, 0.1903f));
}

void transmitToRC(const unsigned long& now) {
  /*if (sd) {
    dataBuffer += String(now) + "," + String(altitude, 1) + "," + String(latitude, 4) + "," + String(longitude, 4) + "," + getServoPosition(leftMotor) + "," + getServoPosition(rightMotor) + "\n";
  }*/

  const byte motor1 = 255 - getServoPosition(rightMotor);
  const byte motor2 = getServoPosition(leftMotor);
  if (!LoRa.beginPacket(/*true*/)) return;
  LoRa.write(0xA4);
  LoRa.print(state);
  LoRa.print(",");
  LoRa.print(motor1);
  LoRa.print(",");
  LoRa.print(motor2);
  LoRa.print(",");
  LoRa.print(altitude);
  LoRa.print(",");
  LoRa.print(String(latitude, 5));
  LoRa.print(",");
  LoRa.print(String(longitude, 5));
  /*LoRa.write(0xA4);
  LoRa.write(state);
  LoRa.write(motor1);
  LoRa.write(motor2);
  writeFloat(altitude);
  writeFloat(latitude);
  writeFloat(longitude);*/
  LoRa.endPacket();
  //LoRa.idle();
  //Serial.println("Send");
}

void checkVerticalSpeed(const unsigned long& now) {
  const float vz = (getAltitude() - altitude) / ((float)(now - lastTransmit) / 1000.0f);  // m/s
  altitude = getAltitude();

  if (apogeeTime) {
    if (altitude - startAlt <= 300.0f || now - apogeeTime >= apogeeDelay) {
      state = GLIDING;
    }
    return;
  }

  if (state == READY || state == PREPARING) {
    if (vz > 5.0f || altitude - startAlt > 20.0f) {
      state = TAKINGOFF;
    }

    if (vz < -5.0f) {
      apogeeTime = now;
    }
  }

  if (state == TAKINGOFF) {
    if (vz < -0.1f) {
      apogeeTime = now;
    }
  }
}

/*void saveToSD(uint16_t chunkSize) {
  if (chunkSize > dataBuffer.length()) {
    chunkSize = dataBuffer.length();
  }
  file.write(dataBuffer.c_str(), chunkSize);
  file.flush();
  dataBuffer.remove(0, chunkSize);
}*/

/*void writeFloat(const float& fl) {
  memcpy(buffer, &fl, 4);
  LoRa.write(buffer, 4);
}*/

void parseGpsData() {
  if (!findGpsHeader()) return;
  if (!skipUntil(GpsSerial, ',')) return;  // Time

  float ltemp = readInt(GpsSerial, 2);
  if (!ltemp) return;
  float mm = GpsSerial.parseFloat(SKIP_NONE);
  if (!mm) return;
  ltemp += mm / 60.0f;
  GpsSerial.read();  // ","
  if (GpsSerial.read() == 'S') ltemp = -ltemp;
  if (latitude && abs(latitude - ltemp) > 0.4f) return;  // Corrupt data detection
  latitude = ltemp;

  if (!skipUntil(GpsSerial, ',')) return;

  ltemp = readInt(GpsSerial, 3);
  if (!ltemp) return;
  mm = GpsSerial.parseFloat(SKIP_NONE);
  if (!mm) return;
  ltemp += mm / 60.0f;
  if (!ltemp) return;
  GpsSerial.read();  // ","
  if (GpsSerial.read() == 'W') ltemp = -ltemp;
  if (longitude && abs(longitude - ltemp) > 0.4f) return;  // Corrupt data detection
  longitude = ltemp;

  if (state == PREPARING) state = READY;
}