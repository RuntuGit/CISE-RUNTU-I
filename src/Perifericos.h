#pragma once

// #include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "MPU9250.h"
#include <Servo.h>
#include <XBee.h>
#include <ArduinoJson.h>

#define MPU9250_IMU_ADDRESS 0x68
#define MAGNETIC_DECLINATION 1.63 // To be defined by user
#define INTERVAL_MS_PRINT 1000

#define GPS_RX_PIN 25
#define GPS_TX_PIN 24
#define XBEE_RX_PIN 0
#define XBEE_TX_PIN 1
#define SERVO_PWM 29
#define BUZZER_PIN 28
#define VOLTAGE_PIN 14

int Leer_Voltaje(void);

class IMU_MPU9250 {
  public:
  bool available;
  bool debug;

  float accel[3];
  float gyro[3];
  float mag[3];
  float orient[3]; // pitch, roll, yaw
  // MPU9250_asukiaaa mySensor;
  MPU9250 mpu;
  unsigned long lastPrintMillis = 0;

  IMU_MPU9250();
  IMU_MPU9250(bool debug);
  void begin();
  void readData();
  void waitMag();
  // Print
  void showAcc();
  void showGyro();
  void showMag();
  void showRawInfo();
  void showOrientation();
};

class BME_280{
  public:
    bool debug;
    Adafruit_BMP280 bme; //I2c
    float pressure;
    float temperature;
    float altitude;

    void begin();
    void readData();
    void showData();
};

class GPS_NEO6M {
  public:
  TinyGPSPlus gps;
  bool available;
  bool debug;

  void begin();
  // Read variables
  void readGPS();
  // Print
  void showLocation();
  void showDateTime();
};

class SERVOMOTOR{
  public:
    bool debug;
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
  XBeeAddress64 addr64; // SH + SL Address of receiving XBee
  ZBTxStatusResponse txStatus;
  ZBTxRequest zbTx;
  XBeeResponse response;  
  ZBRxResponse rx;  
  // create reusable response objects for responses we expect to handle 
  AtCommandRequest atRequest = AtCommandRequest(shCmd);
  AtCommandResponse atResponse = AtCommandResponse();
  // indicators
  int statusLed = 13;
  int errorLed = 13;
  
  void begin(unsigned long SH,unsigned long SL,bool debug);
  void sendAtCommand();
  void sendJson(StaticJsonDocument<200> &doc);
  void sendMsg(String msg);
  void validateSend();
  String receive();

  // Serial high
  uint8_t shCmd[2] = {'S','H'};
  // Serial low
  uint8_t slCmd[2] = {'S','L'};
  // association status
  uint8_t assocCmd[2] = {'A','I'};
};
