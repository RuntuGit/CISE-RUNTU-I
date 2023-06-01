#pragma once

#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Servo.h>
#include <XBee.h>
#include <ArduinoJson.h>

#define MPU9250_IMU_ADDRESS 0x68
#define MAGNETIC_DECLINATION 1.63 // To be defined by user
#define INTERVAL_MS_PRINT 1000

#define GPS_RX_PIN 1
#define GPS_TX_PIN 0
#define XBEE_RX_PIN 3
#define XBEE_TX_PIN 2
#define SERVO_PWM 13
#define BUZZER_PIN 28

class GPS_NEO6M {
  public:
  TinyGPSPlus gps;
  bool available;

  void begin();
  // Read variables
  void readGPS();
  // Print
  void showLocation();
  void showDateTime();
};

class SERVOMOTOR{
  public:
  Servo servoMotor;

  void begin();
  // Movement
  void activar_giro(uint8_t ang);
};

class BUZZER{
  public:
  uint16_t frec;

  void begin();
  void config(uint16_t frec);
  void setAlarm(uint16_t timer);
};

class XBEE_S2C{
  public:
  XBee xbee = XBee();
  uint8_t payload[40];
  XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x4221248B); // SH + SL Address of receiving XBee
  ZBTxStatusResponse txStatus = ZBTxStatusResponse();
  ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
  XBeeResponse response = XBeeResponse();
  // create reusable response objects for responses we expect to handle 
  ZBRxResponse rx = ZBRxResponse();  
  AtCommandRequest atRequest = AtCommandRequest(shCmd);
  AtCommandResponse atResponse = AtCommandResponse();
  // indicators
  int statusLed = 13;
  int errorLed = 13;
  
  void begin(bool debug);
  void sendAtCommand();
  void send(StaticJsonDocument<200> &doc);
  String receive();

  // Serial high
  uint8_t shCmd[2] = {'S','H'};
  // Serial low
  uint8_t slCmd[2] = {'S','L'};
  // association status
  uint8_t assocCmd[2] = {'A','I'};
};
