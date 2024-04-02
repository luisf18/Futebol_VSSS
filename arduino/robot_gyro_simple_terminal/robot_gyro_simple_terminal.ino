// ================================================================================
// Modificações futuras
// ================================================================================


/*/
// ================================================================================
// BLUETOOTH
// ================================================================================
#include "BluetoothSerial.h"
const char *pin = "1234";
String device_name = "Joões Lindos";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
/*/

// ================================================================================
// ESPNOWSerial
// ================================================================================
#include "ESPNOWSerial.h"
String ESPNOW_rcv_str = "";
void ESPNOWSerial_callback(const uint8_t *mac, const uint8_t *data, int size){
  if( size <= 0 || size > 255 ) return;
  //ESPNOW_rcv_str = (const char*) data; //ESPNOWSerial.readString();
  for(int i=0;i<size;i++) ESPNOW_rcv_str += (char)data[i];
}

// ================================================================================
// Motores
// ================================================================================
#include "DRV8833.h"
DRV8833   motor  = DRV8833(
  13,
  14,
  27,
  4
);

// ================================================================================
// Medição de tensão
// ================================================================================
bool voltage_alert = true;
float voltage(){
  return (4.61e-3)*analogRead(34)+0.915;
}

// ================================================================================
// IMU
// ================================================================================
#include "IMU.h"


// ================================================================================
// CONTROLADOR PID
// ================================================================================

float absf( float v ){ return ( v < 0 ? -v : v ); }

String pid_log = "";
String pid_log_plot = "";

// PID struct
class control_pid{
  public:
  float kp = 1;
  float ki = 0;
  float kd = 0;
  uint32_t last_ms = 0;
  float last_erro = 0;
  float P = 0;
  float I = 0;
  float D = 0;
  float Imax = 1000;
  float loop(float erro){
    uint32_t ms = millis();
    uint32_t dt = ms - last_ms; if( dt == 0 ) dt = 1;
    P = erro;
    D = 1000.0*( erro - last_erro )/(float)dt;
    //I = constrain( I+ki*0.001*erro*dt, -I_MAX, I_MAX );
    I = constrain( I+0.001*erro*dt, -Imax, Imax );
    last_erro = erro;
    last_ms = ms;
    //return P*kp + I*ki + kd*D;
    return kp*P + I*ki + kd*D;
  }
  void init(){
    I = 0;
  }
};

control_pid PID_w;
float pid_speed   = 0; // linear speed
float pid_speed_w = 0; // angular speed

int pid_w( int lin, float w ){

  // -------------------------------------------------------------------------------------------------------------------------------------
  // executa controlador
  // -------------------------------------------------------------------------------------------------------------------------------------
  float w_real = (absf(IMU.g.gyro.z) < 0.01 ? 0.0 : IMU.g.gyro.z);
  int dif = PID_w.loop( w - w_real );
  int vl = 0, vr = 0;

  if( abs(lin) < 200 ){
    vl = lin + dif*0.5;
    vr = lin - dif*0.5;
  }else if( lin > 0 ){
    if( dif < 0 ){
      vl = lin+dif;
      vr = lin;
    }else{
      vl = lin;
      vr = lin - dif;
    }
  }else{
    if( dif > 0 ){
      vl = lin+dif;
      vr = lin;
    }else{
      vl = lin;
      vr = lin - dif;
    }
  }

  vl = constrain( vl, -1023, 1023 );
  vr = constrain( vr, -1023, 1023 );

  // -------------------------------------------------------------------------------------------------------------------------------------
  // Se estiver girado não mexe
  // -------------------------------------------------------------------------------------------------------------------------------------
  if( absf(IMU.ac_z_filter) > 4.0 ) motor.move( vl, vr );
  else motor.stop();

  // -------------------------------------------------------------------------------------------------------------------------------------
  // LOGGING
  // -------------------------------------------------------------------------------------------------------------------------------------
  pid_log =  "[pid: [ speed: " + String(vl) + "L * " + String(vr) + "R ] [ angle: " + String(IMU.gyro_z_angle*(180.0/PI)) + " deg ] [ P: " + String(PID_w.P) + " (kp:" + String(PID_w.kp) + " ) D: " + String(PID_w.D) + " (kd:" + String(PID_w.kd) + " ) I: " + String(PID_w.I) + " (ki:" + String(PID_w.ki) + " ) ] ]\n";
  
  float pc_E = ( w == 0.0 ? 0.0 : 100.0*PID_w.P/(float)w );
  float SUM = absf(PID_w.P*PID_w.kp) + absf(PID_w.I*PID_w.ki) + absf(PID_w.D*PID_w.kd);
  float pc_P = ( SUM == 0.0 ? 0.0 : constrain( 100.0*PID_w.P*PID_w.kp/SUM, -200, 200 ));
  float pc_I = ( SUM == 0.0 ? 0.0 : constrain( 100.0*PID_w.I*PID_w.ki/SUM, -200, 200 ));
  float pc_D = ( SUM == 0.0 ? 0.0 : constrain( 100.0*PID_w.D*PID_w.kd/SUM, -200, 200 ));
  float pc_O = 100.0*(float)dif/2048.0;

  pid_log_plot =
    //String(IMU.gyro_z_angle*(180.0/PI)) + " " +
    String(IMU.g.gyro.z) + " " +
    String(pc_E) + " " +
    String(pc_P) + " " +
    String(pc_I) + " " +
    String(pc_D) + " " +
    String(pc_O) + " " +
    String(PID_w.P) + " " +
    String(PID_w.I) + " " +
    String(PID_w.D) + " " +
    String(PID_w.kp) + " " +
    String(PID_w.ki) + " " +
    String(PID_w.kd) + " " +
    String(vl) + " " +
    String(vr) + "\n";

  return dif;
}

// ================================================================================
// TESTE e MEDIÇÃO DO CONTROLADOR PID
// ================================================================================

String pid_test( int lin, float w, uint16_t n, uint16_t dt ){
  String result = "";
  String line = "n t lin w gyro out\n";
  result += line;
  uint32_t timeout = 0;
  int i=0;
  while(i<n){
    IMU.update();
    int out = pid_w( lin, w );
    if( millis() >= timeout ){
      i++;
      timeout = millis() + dt;
      line = String(i) + " " + String(millis()) + " " + String( lin ) + " " + String( w ) + " " + String( IMU.g.gyro.z ) + " " + String( out ) + "\n";
      result += line;
      Serial.print(line);
    }
  }
  motor.stop();
  ESPNOWSerial.println( result );
  return result;
}


// ================================================================================
// CONTROLE DE ESTADOS
// ================================================================================

bool running = false;

void start(){
  motor.bip( 3, 50, 2000 );
  motor.stop();
  PID_w.init();
  running = true;
}

void stop(){
  motor.stop();
  motor.bip( 2, 200, 2000 );
  running = false;
}

void run() {
  if( running ){
    pid_w( pid_speed, pid_speed_w );
  }
}


// ================================================================================
// TERMINAL
// ================================================================================

template<typename T>
String str_array(const char *nome, T * arr, uint32_t sz){
    String str = String(nome) + " = [ ";
    for (int i=0; i<sz-1; i++) str += ( String(arr[i]) + " " );
    str += String(arr[sz-1]) + " ]";
    return str;
}

String terminal( const char *const cmd ){
    
    char key[20]; float x,y,z;
    int i = sscanf(cmd,"%s %f %f %f ",key,&x,&y,&z);
    if(i==0) return "> fail";

    String resposta = "> ";

    //-- comandos MCU --//
    if (strcmp(key,"reset") == 0 ) {
        ESP.restart();
    }else if ( strcmp(key,"mac") == 0 || strcmp(key,"MAC") == 0 ) {
        resposta += "ESP Board MAC Address:  " + WiFi.macAddress() + "\n";
    }else if ( strcmp(key,"digitalWrite") == 0 ) {
        resposta += "digitalWrite: ";
        if( i > 2 ){
          x = constrain( x, 0, 40 ); // pin
          y = constrain( y, 0, 1  ); // state
          digitalWrite(x,y);
          resposta += "pin " + String( (int)x ) + " -> " + String((int)y) + "\n";
        }else{
          resposta += "fail\n";
        }
    }else if ( strcmp(key,"digitalRead") == 0 ) {
        resposta += "digitalRead: ";
        if( i > 1 ){
          x = constrain( x, 0, 40 ); // pin
          resposta += "pin " + String( (int)x ) + " -> " + String( digitalRead((int)x) ) + "\n";
        }else{
          resposta += "fail\n";
        }
    }
    
    //-- sensoriamento --//
    else if (strcmp(key,"voltage") == 0 ) {
        resposta += "Voltage: " + String(voltage()) + " V\n";
    }else if (strcmp(key,"IMU") == 0 ) {
        //IMU.update();
        resposta += ( "ac(x,y,z)[m/s²]: "    + String(IMU.a.acceleration.x,2) + " " + String(IMU.a.acceleration.y,2) + " " + String(IMU.a.acceleration.z,2) + "\n" );
        resposta += ( "gyro(x,y,z)[rad/s]: " + String(IMU.g.gyro.x        ,2) + " " + String(IMU.g.gyro.y        ,2) + " " + String(IMU.g.gyro.z        ,2) + "\n" );
        resposta += ( "temp[°C]: "           + String(IMU.temp.temperature,2) + "\n" );
    }

    //-- help --//
    else if (strcmp(key,"help") == 0 ) {
        resposta += "help list:\n";
        resposta += "speed_tg:\t" + String(pid_speed  ) + "\n";
        resposta += "speed_w:\t"  + String(pid_speed_w) + "\n";
        resposta += "Kp:\t"       + String(PID_w.kp) + "\n";
        resposta += "Ki:\t"       + String(PID_w.ki) + "\n";
        resposta += "Kd:\t"       + String(PID_w.kd) + "\n";
        resposta += "P:\t"        + String(PID_w.P) + "\n";
        resposta += "I:\t"        + String(PID_w.I) + "\n";
        resposta += "D:\t"        + String(PID_w.D) + "\n";
        resposta += "Imax:\t"     + String(PID_w.Imax) + "\n";
        resposta += "outros comandos:\n";
        resposta += "reset\n";
        resposta += "start\n";
        resposta += "stop\n";
        resposta += "speed\n";
        resposta += "w\n";
        resposta += "help\n";
    }

    //-- motor --//
    else if (strcmp(key,"move") == 0 ) {
             if(i == 2) motor.move( constrain(x,-1023,1023), constrain(x,-1023,1023) );
        else if(i >= 3) motor.move( constrain(x,-1023,1023), constrain(y,-1023,1023) );
        resposta += "move: " + String(motor.read(0)) + " " + String(motor.read(1)) + "\n";
        if( i==4 ){
          z = abs(z);
          z = constrain(z,0,10000);
          resposta += String(z) + " ms ";
          delay(z);
          motor.stop();
        }
        resposta += "\n";
    }else if (strcmp(key,"bip") == 0 ) {
        x = ( i > 1 ? constrain(x,1,100 ) : 2    ); // n
        y = ( i > 2 ? constrain(y,1,2000) : 100  ); // delay
        z = ( i > 3 ? constrain(z,1,1e9 ) : 2000 ); // frequência
        motor.bip( x, y, z );
        resposta += "bip\n";
    }else if (strcmp(key,"motor.stop") == 0 ) {
        resposta += "stop\n";
        motor.stop();
    }else if (strcmp(key,"motor.off") == 0 ) {
        resposta += "off\n";
        motor.off();
    }

    //-- controle de estado --//
    else if (strcmp(key,"start") == 0 ) {
        resposta += "start";
        start();
    } else if (strcmp(key,"stop") == 0 ) {
        resposta += "stop";
        stop();
    }
    
    //-- constante do controlador --//
    else if (strcmp(key,"kp") == 0) {
        if(i==2) PID_w.kp = x;
        resposta += "Kp: " + String(PID_w.kp) + "\n";
    } else if (strcmp(key,"ki") == 0 ) {
        if(i==2) PID_w.ki = x;
        resposta += "Ki: " + String(PID_w.ki) + "\n";
    } else if (strcmp(key,"kd") == 0 ) {
        if(i==2) PID_w.kd = x;
        resposta += "Kd: " + String(PID_w.kd) + "\n";
    }
    
    //-- variaveis de estado do controlador --//
    else if (strcmp(key,"P") == 0) {
        if(i==2) PID_w.P = x;
        resposta += "P: " + String(PID_w.P) + "\n";
    } else if (strcmp(key,"I") == 0 ) {
        if(i==2) PID_w.I = x;
        resposta += "I: " + String(PID_w.I) + "\n";
    } else if (strcmp(key,"D") == 0 ) {
        if(i==2) PID_w.D = x;
        resposta += "D: " + String(PID_w.D) + "\n";
    } else if (strcmp(key,"speed") == 0 ) {
        if(i==2) pid_speed = constrain(x,-1023,1023);
        resposta += "speed: " + String(pid_speed) + "\n";
    } else if (strcmp(key,"w") == 0 ) {
        if(i==2) pid_speed_w = x;
        resposta += "speed_w: " + String(pid_speed_w) + "\n";
    }

    return resposta;
}


// ================================================================================
// MAIN
// ================================================================================

void setup() {

  // Serial
  Serial.begin(115200);
  Serial.setTimeout(50);

  // ESPNOWSerial
  ESPNOWSerial.begin();
  ESPNOWSerial.setTimeout(20);
  ESPNOWSerial.setWriteDelay(10);
  ESPNOWSerial.canReciveFrom_anyDevice();
  ESPNOWSerial.printf( "ESPNOWSerial, Hello World!\n" );

  IMU.begin();

  // Begin Motors
  motor.begin();
  motor.sound_vol(12);

  // Chek motores
  motor.bip( 2, 100, 2000, 0 ); delay(50); // Motor 0
  motor.bip( 2, 100, 2000, 1 ); delay(50); // Motor 1

  // Alerta de inicialização
  motor.bip( 1, 50, 5500 );
  motor.bip( 1, 50, 3500 );
  motor.bip( 1, 50, 1500 );

  // calibração da IMU, deixe o robô parado!
  motor.bip( 5, 50, 5500 );
  IMU.calibrate();
  motor.bip( 5, 50, 1500 );

  // inicia o controlador de velocidade angular
  PID_w.kp = 100;
  PID_w.ki = 1500;
  PID_w.kd = 0.8;
  PID_w.Imax = 1;
  
  //ESPNOWSerial.setReciveCallback( ESPNOWSerial_callback );

}

void loop() {

  // IMU
  IMU.update();

  // pid loop
  run();

  // Alerta de baixa de baixa tensão
  if( voltage_alert ){
    if( voltage() > 2.0 && voltage() < 6.8 ){
      ESPNOWSerial.printf( "\n[ LOW BAT!! %f ]", voltage() );
      motor.bip(1,200,2000);
      motor.bip(1,200,5000);
    }
  }

  //if( digitalRead(0) == LOW ){
  //  if( running ) stop(); else start();
  //  delay(200);
  //}

  // terminal pela serial
  //if(Serial.available() > 0){
  while(Serial.available() > 0){
    String CMD = Serial.readStringUntil(';');
    //String CMD = SerialBT.readString();
    Serial.println(CMD);
    ESPNOWSerial.println( CMD );
    String resposta = terminal(CMD.c_str());
    Serial.println(resposta);
    ESPNOWSerial.println(resposta);
  }

  //// terminal pelo bluetooth
  //if(SerialBT.available() > 0){
  //  String CMD = SerialBT.readString();
  //  //String CMD = SerialBT.readString();
  //  Serial.println("command: " + CMD);
  //  SerialBT.println("command: " + CMD);
  //  String resposta = terminal(CMD.c_str());
  //  Serial.println(resposta);
  //  SerialBT.println(resposta);
  //}

  // terminal pelo ESPNOW
  //if(ESPNOWSerial.available() > 0){
  while( ESPNOWSerial.available() > 0 ){
    String CMD = ESPNOWSerial.readStringUntil(';');
    //String CMD = SerialBT.readString();
    Serial.println(CMD);
    ESPNOWSerial.println( CMD );
    String resposta = terminal(CMD.c_str());
    Serial.println(resposta);
    ESPNOWSerial.println(resposta);
  }

}
