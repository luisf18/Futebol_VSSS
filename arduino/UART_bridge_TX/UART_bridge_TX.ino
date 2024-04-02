#include "ESPNOWSerial.h"
// https://github.com/luisf18/ESPNOWSerial

String ESPNOW_rcv_str = "";
void ESPNOWSerial_callback(const uint8_t *mac, const uint8_t *data, int size){
  if( size <= 0 || size > 255 ) return;
  for(int i=0;i<size;i++) ESPNOW_rcv_str += (char)data[i]; //ESPNOWSerial.readString();
}


void setup() {
  
  Serial.begin(115200);
  Serial.setTimeout(20);

  // ESPNOWSerial
  ESPNOWSerial.begin();
  ESPNOWSerial.setTimeout(20);
  ESPNOWSerial.setWriteDelay(10);
  ESPNOWSerial.canReciveFrom_anyDevice();

  ESPNOWSerial.printf( "ESPNOWSerial, Hello World!\n" );
  Serial.println( "TX bridge init" );

  //ESPNOWSerial.setReciveCallback( ESPNOWSerial_callback );

  pinMode(2,OUTPUT);

}

void loop() {

  if( digitalRead(0) == LOW ){
    digitalWrite(2,HIGH);
    Serial.println("stopping");
    ESPNOWSerial.println( "stop" );
    delay(20);
    ESPNOWSerial.println( "stop" );
    digitalWrite(2,LOW);
  }
  
  if( Serial.available() > 0){
    String msg = Serial.readStringUntil('\n');
    ESPNOWSerial.println( msg );
  }

  if( ESPNOWSerial.available() > 0){
    String msg = ESPNOWSerial.readString();
    Serial.println( msg );
  }

  if( ESPNOW_rcv_str.length() > 0 ){
    String result = ESPNOW_rcv_str;
    ESPNOW_rcv_str = "";
    Serial.println( result );
  }

}
