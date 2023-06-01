#include "Perifericos.h"

/* IMU */
  IMU_MPU9250::IMU_MPU9250(){
      this->available = false;
  }
  IMU_MPU9250::IMU_MPU9250(bool debug){
    this->available = false;
    this->debug = debug;
    // Begin
    Wire.begin();
    Serial.println("Starting...");
    MPU9250Setting setting;
    // Sample rate must be at least 2x DLPF rate
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G1000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_250HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    this->mpu.setup(MPU9250_IMU_ADDRESS, setting);

    this->mpu.setMagneticDeclination(MAGNETIC_DECLINATION);
    this->mpu.selectFilter(QuatFilterSel::MADGWICK);
    this->mpu.setFilterIterations(15);

    Serial.println("Calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    delay(5000);

    Serial.println("Calibrating...");
    mpu.calibrateAccelGyro();

    Serial.println("IMU Ready!");
    this->available = true;

  }

  // Reading data
  void IMU_MPU9250::readData(){
    if (this->mpu.update()) {
      // Acceleration
      this->accel[0] = this->mpu.getAccX();
      this->accel[1] = this->mpu.getAccY();
      this->accel[2] = this->mpu.getAccZ();
      // Gyro
      this->gyro[0] = this->mpu.getGyroX();
      this->gyro[1] = this->mpu.getGyroY();
      this->gyro[2] = this->mpu.getGyroZ();
      // Magnetometer
      this->mag[0] = this->mpu.getMagX();
      this->mag[1] = this->mpu.getMagY();
      this->mag[2] = this->mpu.getMagZ();
      // Orientation
      this->orient[0] = this->mpu.getPitch();
      this->orient[1] = this->mpu.getRoll();
      this->orient[2] = this->mpu.getYaw();
      }
  }

  void IMU_MPU9250::waitMag(){
    do{
      if (this->mpu.update()) {
        this->mag[0] = this->mpu.getMagX();
        this->mag[1] = this->mpu.getMagY();
        this->mag[2] = this->mpu.getMagZ();
      }  
    }while(this->mag[0]==0);
    
    Serial.println("Magnetometer working");  
  }

  // Printing
  void IMU_MPU9250::showAcc(){
    Serial.print("accelX: " + String(this->accel[0]));
    Serial.print("\taccelY: " + String(this->accel[1]));
    Serial.print("\taccelZ: " + String(this->accel[2]));
  }

  void IMU_MPU9250::showGyro(){
    Serial.print("\tgyroX: " + String(this->gyro[0]));
    Serial.print("\tgyroY: " + String(this->gyro[1]));
    Serial.print("\tgyroZ: " + String(this->gyro[2]));
  }

  void IMU_MPU9250::showMag(){
    Serial.print("\tmagX: " + String(this->mag[0]));
    Serial.print("\tmaxY: " + String(this->mag[1]));
    Serial.print("\tmagZ: " + String(this->mag[2]));
  }

  void IMU_MPU9250::showRawInfo(){
    this->readData();
    this->showAcc();
    this->showGyro();
    this->showMag();
    Serial.println();
  }

  void IMU_MPU9250::showOrientation(){
    Serial.print(this->orient[0]);
    Serial.print(",");
    Serial.print(this->orient[1]);
    Serial.print(",");
    Serial.println(this->orient[2]);
  }

//

/* BME */
  void BME_280::begin(){
    this->bme.begin(0x76);
  }

  void BME_280::readData(){
    this->temperature = this->bme.readTemperature();
    this->pressure = this->bme.readPressure()/1000;
    this->altitude = this->bme.readAltitude(1013.25);
  }

  void BME_280::showData(){
    Serial.print("\tTemperature(*C): ");
    Serial.print(bme.readTemperature());

    Serial.print("\tPressure(hPa): ");
    Serial.print(bme.readPressure()/100);

    Serial.print("\tApproxAltitude(m): ");
    Serial.print(bme.readAltitude(1013.25)); // this should be adjusted to your local forcase
  }
//

/* GPS */
  bool flag = false;
  SoftwareSerial SerialGPS(GPS_RX_PIN, GPS_TX_PIN);

  void GPS_NEO6M::begin(){
    int t = millis();
    int count = 0;

    this->available = false;
    SerialGPS.begin(9600);
    while (SerialGPS.available() <= 0){
      if (millis() > 5000 && this->gps.charsProcessed() < 10){
        Serial.println("GPS NOT DETECTED!");
        break;     
      }
    }
    while(!this->gps.location.isValid()){
      this->readGPS();
      if(millis()-t > 60000){
        count++;
        t = millis();
      }
      if (count == 2) break;
    }
    this->available = true;
    Serial.println("\nLocation Fixed!");
  }

  void GPS_NEO6M::readGPS(){
    while (1){
      while (SerialGPS.available() > 0)
        if (this->gps.encode(SerialGPS.read())){
          Serial.print(".");
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
  void XBEE_S2C::begin(unsigned long SH, unsigned long SL, bool debug){
    pinMode(statusLed, OUTPUT);
    pinMode(errorLed, OUTPUT);

    addr64 = XBeeAddress64(SH, SL); // SH + SL Address of receiving XBee
    txStatus = ZBTxStatusResponse();
    zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
    response = XBeeResponse();  
    ZBRxResponse rx = ZBRxResponse();  
    // create reusable response objects for responses we expect to handle 
    atRequest = AtCommandRequest(shCmd);
    atResponse = AtCommandResponse();

    xbeeSerial.begin(9600);
    xbee.setSerial(xbeeSerial);
    
    if (debug){
      Serial.println("Starting up!");
      // get SH
      sendAtCommand();
      // set command to SL
      atRequest.setCommand(slCmd);  
      sendAtCommand();
      delay(1000);
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

  void XBEE_S2C::validateSend(){
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

  void XBEE_S2C::sendJson(StaticJsonDocument<200> &doc){
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
    Serial.println(payloadString);

    // flash TX indicator
    //flashLed(statusLed, 1, 1000);

    // after sending a tx request, we expect a status response
    this->validateSend();
  }

  void XBEE_S2C::sendMsg(String msg){
    // Convert the payload string to a uint8_t array
    uint8_t payload1[msg.length()];
    msg.getBytes(payload1, msg.length()+1);
    zbTx.setPayload(payload1);
    zbTx.setPayloadLength(sizeof(payload1));
    xbee.send(zbTx);
    Serial.print(sizeof(payload1));
    Serial.println("Enviado. ");
    Serial.println(msg);

    // flash TX indicator
    //flashLed(statusLed, 1, 1000);

    // after sending a tx request, we expect a status response
    this->validateSend();
  }

  String XBEE_S2C::receive(){
    xbee.readPacket(50);
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
        
        return mensaje;
      }
    } else if (xbee.getResponse().isError()) {
      Serial.print("error code:");
      Serial.println(xbee.getResponse().getErrorCode());
    }
    return "";  
  }
//

/* SENSOR DE VOLTAJE*/
int Leer_Voltaje(void){
  int val = (int)((double)analogRead(VOLTAGE_PIN)/1023*100);
  return val;
}