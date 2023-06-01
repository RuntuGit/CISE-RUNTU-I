#include "Perifericos.h"

XBEE_S2C xbee;
GPS_NEO6M gps;
SERVOMOTOR servoMotor;

bool debug = false;

String mensaje = "";

void setup() {
  Serial.begin(9600);
  xbee.begin(true);
  // gps.begin();
  servoMotor.begin();
}

void loop() {
  // Espera a la señal de separación de la carga primaria
  servoMotor.activar_giro(45);
  servoMotor.activar_giro(0);/*
  while(mensaje[0] != 'S'){
    mensaje = xbee.receive();
    if (debug){
      Serial.println("No se recibe ninguna mensaje");
    }
  }
  // Activación de servomotor
  servoMotor.activar_giro(45);
  if (debug){
    Serial.print("Done");
  }

  while (1){
    // Espera mensaje de carga primaria
    while(mensaje[0] != 'M'){
      mensaje = xbee.receive();
      if (debug){
        Serial.println("No hay ninguna petición de envío");
      }
    }
    mensaje[0] = "";
    
    //gps.readGPS();
    //gps.showLocation();
    StaticJsonDocument<200> doc;
    doc["lat"] = 19;
    doc["lon"] = 72;
    xbee.send(doc);
  }*/
    delay(200);
  
}
