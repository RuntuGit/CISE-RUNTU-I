#pragma once
#include "CansatState.h"
#include "Perifericos.h"
#include "TeensyThreads.h"

#define SAMPLING_TIME 100

class CansatState;

class Cansat{
public:
  // Periféricos
  XBEE_S2C xbee_ground;
  XBEE_S2C xbee_second;
  SERVOMOTOR servo;
  BME_280 bme;
  IMU_MPU9250 imu_9250;
  GPS_NEO6M gps_6m;
  BUZZER buzz;
  // Datos de la misión
  float altura_max;
  float dist_cs;
  float dir_cs;
  float ground;
  // Flags
  bool start;
  int env;
  // Transmisión de datos
  StaticJsonDocument<200> doc;
  // Debug
  bool debug;
  bool sim;

public:
	Cansat(bool sim, bool debug);
  void startThreads();  
  void setAltitude(float value);
  float getAltitude();
  void Xbee_read();
  void Xbee_send();
  // FSM
	inline CansatState* getCurrentState() const { return currentState; }
	void toggle();
  void setState(CansatState& newState);

private:
  int threadGPS_;
  int threadGY_;

	CansatState* currentState;
  static void GPSthread(void* arg);
  static void GYthread(void* arg);
};
