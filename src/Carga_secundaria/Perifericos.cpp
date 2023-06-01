#include "Perifericos.h"

/* GPS */
  bool flag = false;
  SoftwareSerial SerialGPS(GPS_RX_PIN, GPS_TX_PIN);

  void GPS_NEO6M::begin(){
    SerialGPS.begin(9600);
    this->available = false;
    while (SerialGPS.available() <= 0){
      if (millis() > 5000 && this->gps.charsProcessed() < 10){
        Serial.println("GPS NOT DETECTED!");
        break;     
      }
    }
    while(!this->gps.location.isValid()){
      this->readGPS();
      Serial.print(".");
    }
    this->available = true;
  }

  void GPS_NEO6M::readGPS(){
    while (1){
      while (SerialGPS.available() > 0)
        if (this->gps.encode(SerialGPS.read())){
    Serial.println("hi");
          return;
        }
    }
  }

  void GPS_NEO6M::showLocation(){
    Serial.print("Location: ");
    this->readGPS();
    if (!this->gps.location.isValid()){
      Serial.println("Not Available");
      return;    
    }
    Serial.print("\nLatitude: ");
    Serial.print(this->gps.location.lat(), 15);
    Serial.print("\tLongitude: ");
    Serial.print(this->gps.location.lng(), 15);
    Serial.print("\tAltitude: ");
    Serial.println(this->gps.altitude.meters());
    Serial.println("");
  }

  void GPS_NEO6M::showDateTime(){
    this->readGPS();
    Serial.print("Date: ");
    if (this->gps.date.isValid()){
      Serial.print(String(this->gps.date.month())+"/"+String(this->gps.date.day())+"/"+String(this->gps.date.year()));
    }
    else{
      Serial.println("Not Available");
    }

    Serial.print("Time: ");
    if (this->gps.time.isValid()){
      if (this->gps.time.hour() < 10) Serial.print(F("0"));
      Serial.print(this->gps.time.hour());
      Serial.print(":");
      if (this->gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(this->gps.time.minute());
      Serial.print(":");
      if (this->gps.time.second() < 10) Serial.print(F("0"));
      Serial.print(this->gps.time.second());
      Serial.print(".");
      if (this->gps.time.centisecond() < 10) Serial.print(F("0"));
      Serial.println(this->gps.time.centisecond());
    }
    else{
      Serial.println("Not Available");
    }
    Serial.println();
  }

//
/* Servo */
  void SERVOMOTOR::begin(){
    this->servoMotor.attach(SERVO_PWM);
  }

  void SERVOMOTOR::activar_giro(uint8_t angulo){
    this->servoMotor.write(angulo);
    delay(1000);
  }
//
/* BUZZER */
  void BUZZER::begin(){
    this->frec = 440;
    pinMode(BUZZER_PIN, OUTPUT);
  }

  void BUZZER::config(uint16_t frec){
    this->frec = frec;  
  }

  void BUZZER::setAlarm(uint16_t timer){
    tone(BUZZER_PIN, this->frec);  
    delay(timer);
    noTone(BUZZER_PIN);  
  }
//
/* XBEE */
  SoftwareSerial xbeeSerial(XBEE_RX_PIN, XBEE_TX_PIN); // RX, TX
  void XBEE_S2C::begin(bool debug=false){
    pinMode(statusLed, OUTPUT);
    pinMode(errorLed, OUTPUT);

    xbeeSerial.begin(9600);
    xbee.setSerial(xbeeSerial);
    
    if (debug){
      Serial.println("Starting up!");
      // get SH
      sendAtCommand();
      // set command to SL
      atRequest.setCommand(slCmd);  
      sendAtCommand();
    }
  }

  void XBEE_S2C::sendAtCommand() {
    Serial.println("Sending command to the XBee");

    // send the command
    xbee.send(atRequest);

    // wait up to 5 seconds for the status response
    if (xbee.readPacket(5000)) {
      // got a response!

      // should be an AT command response
      if (xbee.getResponse().getApiId() == AT_COMMAND_RESPONSE) {
        xbee.getResponse().getAtCommandResponse(atResponse);

        if (atResponse.isOk()) {
          Serial.print("Command [");
          Serial.print(atResponse.getCommand()[0]);
          Serial.print(atResponse.getCommand()[1]);
          Serial.println("] was successful!");

          if (atResponse.getValueLength() > 0) {
            Serial.print("Command value length is ");
            Serial.println(atResponse.getValueLength(), DEC);

            Serial.print("Command value: ");
            
            for (int i = 0; i < atResponse.getValueLength(); i++) {
              Serial.print(atResponse.getValue()[i], HEX);
              Serial.print(" ");
            }

            Serial.println("");
          }
        } 
        else {
          Serial.print("Command return error code: ");
          Serial.println(atResponse.getStatus(), HEX);
        }
      } else {
        Serial.print("Expected AT response but got ");
        Serial.print(xbee.getResponse().getApiId(), HEX);
      }   
    } else {
      // at command failed
      if (xbee.getResponse().isError()) {
        Serial.print("Error reading packet.  Error code: ");  
        Serial.println(xbee.getResponse().getErrorCode());
      } 
      else {
        Serial.print("No response from radio");  
      }
    }
  }

  void XBEE_S2C::send(StaticJsonDocument<200> &doc){
    // Serialize the JSON object to a string
    String payloadString;
    serializeJson(doc, payloadString);

    // Convert the payload string to a uint8_t array
    uint8_t payload1[payloadString.length()];
    payloadString.getBytes(payload1, payloadString.length()+1);
    zbTx.setPayload(payload1);
    zbTx.setPayloadLength(sizeof(payload1));
    xbee.send(zbTx);
    Serial.print(sizeof(payload1));
    Serial.println("Enviado. ");

    // flash TX indicator
    //flashLed(statusLed, 1, 1000);

    // after sending a tx request, we expect a status response
    // wait up to half second for the status response
    if (xbee.readPacket(5000)) {
      // got a response!
      Serial.print("Respuesta: ");

      // should be a znet tx status            	
      if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
        xbee.getResponse().getZBTxStatusResponse(txStatus);

        // get the delivery status, the fifth byte
        if (txStatus.getDeliveryStatus() == SUCCESS) {
          // success.  time to celebrate
          Serial.println("ENVIADO CORRECTAMENTE");
          
        } else {
          // the remote XBee did not receive our packet. is it powered on?
          
        }
      }
    } else if (xbee.getResponse().isError()) {
      Serial.print("Error code: ");  
      Serial.println(xbee.getResponse().getErrorCode());
    } else {
      // local XBee did not provide a timely TX Status Response -- should not happen
      Serial.println("No deberia pasar");
    }    
  }

  String XBEE_S2C::receive(){
    xbee.readPacket(500);
    if (xbee.getResponse().isAvailable()) {
      Serial.println("\nGot something!");
      // got something
            
      if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
      // got a zb rx packet
      // now fill our zb rx class
        xbee.getResponse().getZBRxResponse(rx);
        Serial.println("Got an rx packet!");

        Serial.print("packet length is ");
        Serial.println(rx.getPacketLength(), DEC);
        String mensaje;
        
        for (int i = 0; i < rx.getDataLength(); i++) {
            char a = (char)rx.getData()[i];
            mensaje = mensaje+a;
          }
        Serial.println(String(mensaje));
        return String(mensaje);
      }
    } else if (xbee.getResponse().isError()) {
        Serial.print("error code:");
        Serial.println(xbee.getResponse().getErrorCode());
        return "Error";
      }  
  return "";
  }
//
