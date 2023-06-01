#include "Cansat.h"
#include <XBee.h>
#include "Perifericos.h"
#include "TeensyThreads.h"
#include "ConcreteCansatStates.h"

#define TRANSMIT_DATA_TIME 1500

const int LED = 13;

Cansat cansat = Cansat(false, true); // (sim, debug)
uint32_t tiempo;
float change = 5;

// Hilo
void State_thread(){
  Serial.println("Hola q ase");
  while (1){
    cansat.toggle();
    threads.yield();
  }
}

void timer_blink(int min){
  int cont = 0;
  int timer = 0;
  do{
    digitalWrite(LED, HIGH);
    delay(500);  
    digitalWrite(LED, LOW);
    delay(500);  
    cont++;
    if(cont==60){
      cont = 0;
      timer++;
    }
  }while(min > timer);
}

void setup(){
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(3000);
  digitalWrite(LED, LOW);
  // timer_blink(1); // Modificar para la misión
  Serial.begin(9600);
  // cansat.xbee_ground.begin(0x0013A200, 0x422121B5, true); 
  cansat.xbee_ground.begin(0x0013A200, 0x422121B5, false); // test
  // cansat.xbee_second.begin(0x0013A200, 0x420389F8, false);
  cansat.xbee_second.begin(0x0013A200, 0x422124C6, false); // pato
 
  threads.setSliceMicros(100); 
  cansat.startThreads();

  //threads.setTimeSlice(threads.addThread(State_thread), 100);
  threads.addThread(State_thread);

  digitalWrite(LED, HIGH);
  tiempo = millis();
}

void loop(){
  if (millis()-tiempo > TRANSMIT_DATA_TIME && cansat.env == 1){
    cansat.Xbee_send();
     tiempo = millis();
     Serial.print("Altura:");
     Serial.println(cansat.getAltitude());
  }
}
