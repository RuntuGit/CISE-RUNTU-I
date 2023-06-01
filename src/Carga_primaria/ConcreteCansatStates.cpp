#include "ConcreteCansatStates.h"

float ASCEND;
float DROP;
float GYRO;
float IMPACT;
float LAND;

double d = 0;
double a = 0;

float calc_velZ(Cansat* cansat){
  static int t = millis();
  static float vel_z = 0;
  vel_z = -(cansat->imu_9250.accel[2]-1)*9.8*SAMPLING_TIME/1000 + vel_z;
  // Serial.println(cansat->debug);
  if(cansat->debug && millis()-t > 500){ //
    Serial.print("Velocidad en z: ");
    Serial.println(vel_z);
    t = millis();    
  }
  return vel_z;
}

void show_state(Cansat* cansat){
  static int t2 = millis();
  if(millis()-t2 > 5000){
    cansat->getCurrentState()->name();
    t2=millis();
  }  
}

/*Start State*/
  void Start::name(){
    Serial.println("State: Start");
  }

  void Start::enter(Cansat* cansat){
    Serial.println("\nInicio de la misión");
    /* Inicialización de alturas para el cambio de estado*/
    // Altura necesaria para el cambio al estado de ascenso
    ASCEND = cansat->ground+2;
    // Altura necesaria para el cambio al estado de caida libre
    DROP = cansat->ground+395;
    // Altura necesaria para el cambio al estado de caida con autogiro
    GYRO = cansat->ground+255;
    // Altura necesaria para el cambio al estado de impacto inminente
    IMPACT = cansat->ground+10;
    // Altura necesaria para el cambio al estado de aterrizaje realizado
    LAND = cansat->ground+2;
  }

  void Start::toggle(Cansat* cansat){
    // Serial.println(cansat->imu_9250.accel[2]);
    if (calc_velZ(cansat) >= 0.08 and cansat->bme.altitude > ASCEND){ //Cambiar!!!!!
      // Start -> Ascending
      cansat->setState(Ascending::getInstance());
    }
    if(cansat->debug) show_state(cansat);
  }

  CansatState& Start::getInstance(){
    static Start singleton;
    return singleton;
  }

/*Ascending State*/
  void Ascending::name(){
    Serial.println("State: Ascending");
  }

  void Ascending::enter(Cansat* cansat){
    Serial.println("\nDespege del satélite");
  }

  void Ascending::toggle(Cansat* cansat){
    static float altura_max = 0;
    if (altura_max<cansat->bme.altitude){
      altura_max = cansat->bme.altitude;
    }
    if (cansat->bme.altitude > DROP and calc_velZ(cansat) < 0){
      // Ascending -> FreeFall
      cansat->setState(FreeFall::getInstance());
      cansat->altura_max = altura_max;
      altura_max = 0;
    }
    if(cansat->debug) show_state(cansat);
  }

  CansatState& Ascending::getInstance(){
    static Ascending singleton;
    return singleton;
  }

/*Free Fall State*/
  void FreeFall::name(){
    Serial.println("State: FreeFall");
  }

  void FreeFall::enter(Cansat* cansat){
    Serial.println("\nIniciando caida libre");
  }

  void FreeFall::toggle(Cansat* cansat){
    static int16_t t = 0;
    if (t==0){
      t = millis();
    }
    delay(1000);
    if (cansat->bme.altitude < GYRO or millis()-t > 6000){
      // FreeFall -> GyroFall
      cansat->setState(GyroFall::getInstance());
      t = 0;
    }
    if(cansat->debug) show_state(cansat);
  }

  CansatState& FreeFall::getInstance(){
    static FreeFall singleton;
    return singleton;
  }

/*Gyro State*/
  void GyroFall::name(){
    Serial.println("State: GyroFall");
  }

  void GyroFall::enter(Cansat* cansat){
    cansat->xbee_second.sendMsg("Start");
    delay(20);
    //cansat->Xbee_send();
    delay(200);
    Serial.println("\nLiberando autogiro");
  }

  void GyroFall::toggle(Cansat* cansat){
    if (cansat->bme.altitude < IMPACT){
      // GyroFall -> Landing
      cansat->setState(Landing::getInstance());
    }
    delay(500);
    cansat->xbee_second.sendMsg("M");
    cansat->env = 0;
    delay(1000);
    String msg = cansat->xbee_second.receive();
    delay(500);
    cansat->env = 1;

    StaticJsonDocument<200> pos;
    DeserializationError error = deserializeJson(pos, msg);
    // if (error) {
    //   Serial.print("Failed to parse JSON message: ");
    //   Serial.println(error.c_str());
    //   return;
    // }
    // Calculate distance and direction
    d = cansat->gps_6m.gps.distanceBetween(19.310566372707406, -99.1741482800582, (double)pos["lat"], (double)pos["lon"]);
    a = cansat->gps_6m.gps.courseTo(19.310566372707406, -99.1741482800582, (double)pos["lat"], (double)pos["lon"]);
    // Test
    // d = cansat->gps_6m.gps.distanceBetween(cansat->gps_6m.gps.location.lat(), cansat->gps_6m.gps.location.lng(), 19.33192273292393, -99.19224015515003);
    // a = cansat->gps_6m.gps.courseTo(cansat->gps_6m.gps.location.lat(), cansat->gps_6m.gps.location.lng(), 19.33192273292393, -99.19224015515003);
    //if (cansat->debug){
      Serial.println("Posición carga secundaria:");
      Serial.print("Distancia a la carga secundaria: ");  
      Serial.println(d);  
      Serial.print("Dirección a la carga secundaria: ");  
      Serial.println(a);
    //}
    if(d!=0){
      cansat->dist_cs = d;
      cansat->dir_cs = a;
    }
    if(cansat->debug) show_state(cansat);
  }

  CansatState& GyroFall::getInstance(){
    static GyroFall singleton;
    return singleton;
  }

/*Landing State*/
  void Landing::name(){
    Serial.println("State: Landing");
  }

  void Landing::enter(Cansat* cansat){
    // cansat->buzz.setAlarm(1000);
  }

  void Landing::toggle(Cansat* cansat){
    if (cansat->bme.altitude < LAND and calc_velZ(cansat) > -0.5 and calc_velZ(cansat) < 0.5){
      // Landing -> Idle
      cansat->setState(Idle::getInstance());
    }
    if(cansat->debug) show_state(cansat);
  }

  void Landing::exit(Cansat* cansat){
    Serial.println("\nAterrizaje realizado");
    // cansat->buzz.setAlarm(1000);
  }

  CansatState& Landing::getInstance(){
    static Landing singleton;
    return singleton;
  }

/*Idle State*/
  void Idle::name(){
    Serial.println("State: Idle");
  }

  void Idle::enter(Cansat* cansat){
    Serial.println("\nIdle");
    // cansat->buzz.setAlarm(1000);
  }
  void Idle::toggle(Cansat* cansat){
    // Idle -> Start
    if(cansat->debug) show_state(cansat);
    delay(5000);
    if(cansat->debug) show_state(cansat);
    cansat->setState(Start::getInstance());
  }

  CansatState& Idle::getInstance(){
    static Idle singleton;
    return singleton;
  }