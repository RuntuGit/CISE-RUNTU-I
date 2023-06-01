#include "Cansat.h"
#include "ConcreteCansatStates.h"

void timer(){
  static int t = micros();
  Serial.print("Timer: ");
  Serial.println(micros()-t);
}

Cansat::Cansat(bool sim, bool debug){
  if (debug) Serial.print("Cansat initializing");
  this->debug = debug;
  this->sim = sim;
  this->start = false;
  this->env = 1;
  this->servo.begin();
	currentState = &Idle::getInstance();
}

void Cansat::setAltitude(float value){
  bme.altitude = value;
}

float Cansat::getAltitude(){
  return bme.altitude;
}

void Cansat::startThreads(){
  int t = millis();
  if (this->debug) Serial.print("Initializing sensors");
  // threadGPS_ = threads.addThread(GPSthread, this);
  threadGY_ = threads.addThread(GYthread, this);
  while(!(this->imu_9250.available)){ //this->gps_6m.available && 
    // Serial.print("GPS: ");
    // Serial.println(this->gps_6m.available);
    // Serial.print("IMU: ");
    // Serial.println(this->imu_9250.available);
    delay(100);
  }
  this->start = true;
  delay(100);
  this->ground = this->getAltitude();
  if(this->debug){
    Serial.print("Altura base: ");
    Serial.println(this->ground);
  }
}

void Cansat::GPSthread(void* dir){
  Cansat* cansat = static_cast<Cansat*>(dir);  
  cansat->gps_6m.begin();
  Serial.print("GPS_Init");
  while(cansat->gps_6m.available) {
    cansat->gps_6m.readGPS();
    threads.delay(1000);
  }
}

void Cansat::GYthread(void* dir){  
  Cansat* cansat = static_cast<Cansat*>(dir);
  if (cansat->debug) Serial.println("Begin IMU");
  
  cansat->imu_9250 = IMU_MPU9250(cansat->debug);
  cansat->bme.begin();
  cansat->imu_9250.waitMag();

  // Test  
  if (cansat->sim){
  cansat->setAltitude(2600);
  Serial.print(cansat->getAltitude());
  cansat->imu_9250.accel[2] = 3; 
  }
  int change = 5;  
  
  while(1) {

    if(!cansat->sim){
      cansat->imu_9250.readData();
      cansat->bme.readData();
      if (cansat->debug){
        // cansat->imu_9250.showRawInfo();
        // cansat->imu_9250.showOrientation();
        // cansat->bme.showData();
      }
    }
    else{ // Simulation
      cansat->setAltitude(cansat->getAltitude() + change);
      if (cansat->getAltitude() >= 3000){
        change = -5;
        cansat->imu_9250.accel[2] = -3;
      }   
      if (cansat->getAltitude() < 2700 && cansat->imu_9250.accel[2] < 0){
        change = 0;
        cansat->imu_9250.accel[2] = 1;
      }   
    }
    threads.delay(100);
  }
}

void Cansat::Xbee_send(){
  StaticJsonDocument<200> doc;
  doc["T"] = roundf(bme.temperature * double(10)) / double(10);//bme.temperature roundf(bme.temperature * 100) / 100);
  doc["p"] = roundf(bme.pressure * double(10)) / double(10);//bme.pressure roundf(bme.pressure * 100) / 100);
  doc["h"] = (int)(bme.altitude);//bme.altitude
  doc["X"] = roundf(imu_9250.accel[0] * double(10)) / double(10); //roundf(imu_9250.accel[0] * double(100)) / double(100)
  doc["Y"] = roundf(imu_9250.accel[1] * double(10)) / double(10);
  doc["Z"] = roundf(imu_9250.accel[2] * double(100)) / double(100);
  doc["D"] = roundf(dist_cs * double(10)) / double(10);
  doc["A"] = roundf(dir_cs * double(10)) / double(10);
  doc["V"] = Leer_Voltaje();
  this->xbee_ground.sendJson(doc);
}

// Máquina de estados
void Cansat::toggle(){
	// Delegate the task of determining the next state to the current state!
	currentState->toggle(this);
}

void Cansat::setState(CansatState& newState){
	currentState->exit(this);  // do stuff before we change state
	currentState = &newState;  // change state
	currentState->enter(this); // do stuff after we change state
}