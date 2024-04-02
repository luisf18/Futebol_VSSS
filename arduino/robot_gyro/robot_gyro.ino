// Basic demo for accelerometer readings from Adafruit MPU6050

#include "DRV8833.h"
#include "IMU.h"

bool voltage_alert = true;
float voltage(){
  return (4.61e-3)*analogRead(34)+0.915;
}

//        RL: -14.41  LR: 14.45  L: 7.39  R: -7.11 
// MAX -> RL: -15.69  LR: 15.69  L: 9.48  R: -9.53

// PID: kp = 0.03 kd = 70000.00

/*  Motores  */
DRV8833   motor  = DRV8833(
  13,
  14,
  27,
  4
);


float acx_0 = 0.0;
float ac[20];
float speed;
float speed_RL, speed_LR, speed_L, speed_R;
float speed_RL_max, speed_LR_max, speed_L_max, speed_R_max;

float w[20];

void get_gyro_z_curve( int vl, int vr, uint32_t dt ){
  motor.move(vl,vr);
  for( int i=0;i<sizeof(w)/sizeof(float); i++ ){
    IMU.update();
    w[i] = IMU.g.gyro.z;
    delay(dt);
  }
  motor.stop();
}

float get_gyro_z( int vl, int vr, int n, uint32_t dt, uint32_t dead_time ){
  float speed = 0;
  motor.move(vl,vr);
  delay(dead_time);
  for( int i=0;i<n; i++ ){
    IMU.update();
    speed += IMU.g.gyro.z/(float) n;
    delay(dt);
  }
  motor.stop();
  return speed;
}

float get_speed( int vl, int vr, uint32_t dt ){
  int n = sizeof(ac)/sizeof(float);
  float speed = 0;
  motor.stop();
  delay(200);
  acx_0 = 0.0;
  for( int i=0;i<10; i++ ){ IMU.update(); acx_0 += IMU.a.acceleration.x/10.0; delay(5); }
  motor.move(vl,vr);
  uint32_t t0 = millis();
  for( int i=0;i<n; i++ ){
    delay(dt);
    IMU.update();
    uint32_t t = millis();
    ac[i] = (IMU.a.acceleration.x - acx_0);
    speed += 0.001*ac[i]*(t-t0);
  }
  motor.stop();
  return speed;
}


///////////////////////////////////////////////////////////
/// PID                                                ////
///////////////////////////////////////////////////////////

int    pid_auto = false;
double pid_auto_angle = 0, pid_auto_speed = 0;

float absf( float v ){ return ( v < 0 ? -v : v ); }

String pid_log = "";
String pid_log_plot = "";

// PID struct
typedef struct control_pid_t{
  float kp = 1;
  float ki = 0;
  float kd = 0;
  uint32_t last_ms = 0;
  float last_erro = 0;
  float P = 0;
  float I = 0;
  float D = 0;
  float I_MAX = 1000;
  float loop(float erro){
    uint32_t ms = millis();
    uint32_t dt = ms - last_ms; if( dt == 0 ) dt = 1;
    P = erro;
    D = 1000.0*( erro - last_erro )/(float)dt;
    //I = constrain( I+ki*0.001*erro*dt, -I_MAX, I_MAX );
    I = constrain( I+0.001*erro*dt, -I_MAX, I_MAX );
    last_erro = erro;
    last_ms = ms;
    //return P*kp + I*ki + kd*D;
    return kp*P + I*ki + kd*D;
  }
}control_pid_t;

control_pid_t PID_angle;
control_pid_t PID_mix_real;
control_pid_t PID_mix_set;
control_pid_t PID_mix;

void pid_angle( int lin, float angle_set ){
  //IMU.update();
  
  int dif = PID_angle.loop( angle_set - IMU.gyro_z_angle );
  int vl = 0, vr = 0;

  // set 0 / angle + / erro - / dif - ==> angle+ -> vl+  vr- ==> angle- -> vl- vr+
  // angle + -> vl+  vr-
  // angle - -> vl-  vr+

  if( abs(lin) < 300 ){
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

  if( absf(IMU.ac_z_filter) > 4.0 ) motor.move( constrain( vl, -1023, 1023 ), constrain( vr, -1023, 1023 ) );
  else motor.stop();

  //pid_log =  "[pid: [ speed: " + String(vl) + "L * " + String(vr) + "R ] [ angle: " + String(IMU.gyro_z_angle*(180.0/PI)) + " deg ] [ P: " + String(P) + " (kp:" + String(kp) + " ) D: " + String(PID_angle.D) + " (kd:" + String(PID_angle.kd) + " ) ] dt: " + String(dt)+ " ms ]\n";
  pid_log =  "[pid: [ speed: " + String(vl) + "L * " + String(vr) + "R ] [ angle: " + String(IMU.gyro_z_angle*(180.0/PI)) + " deg ] [ P: " + String(PID_angle.P) + " (kp:" + String(PID_angle.kp) + " ) D: " + String(PID_angle.D) + " (kd:" + String(PID_angle.kd) + " ) ] ]\n";
  Serial.print( pid_log );
  
}


control_pid_t PID_w;

int pid_w( int lin, float w ){

  float w_real = (absf(IMU.g.gyro.z) < 0.01 ? 0.0 : IMU.g.gyro.z);
  int dif = PID_w.loop( w - w_real );
  int vl = 0, vr = 0;

  // set 0 / angle + / erro - / dif - ==> angle+ -> vl+  vr- ==> angle- -> vl- vr+
  // angle + -> vl+  vr-
  // angle - -> vl-  vr+

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

  if( absf(IMU.ac_z_filter) > 4.0 ) motor.move( vl, vr );
  else motor.stop();

  //pid_log =  "[pid: [ speed: " + String(vl) + "L * " + String(vr) + "R ] [ angle: " + String(IMU.gyro_z_angle*(180.0/PI)) + " deg ] [ P: " + String(P) + " (kp:" + String(kp) + " ) D: " + String(PID_angle.D) + " (kd:" + String(PID_angle.kd) + " ) ] dt: " + String(dt)+ " ms ]\n";
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
  //Serial.print( pid_log );
  //200.00 1000.00 0.80
  return dif;
}



void plot(){
  Serial.println("gyro:");
  for(int i=0;i<sizeof(w)/sizeof(float);i++) Serial.printf( "%.04f\n", w[i] );
  Serial.println("aceleration:");
  for(int i=0;i<sizeof(ac)/sizeof(float);i++) Serial.printf( "%.04f\n", ac[i] );
  Serial.printf("acx_0: %0.4f m/s²\n",acx_0);
  Serial.printf("Speed: %0.4f m/s\n",speed);
  Serial.println("gyro z yaw:");
  Serial.printf(
    "RL: %.02f  LR: %.02f  L: %.02f  R: %.02f",
    speed_RL, speed_LR, speed_L, speed_R
  );
  Serial.printf(
    " MAX -> RL: %.02f  LR: %.02f  L: %.02f  R: %.02f\n",
    speed_RL_max, speed_LR_max, speed_L_max, speed_R_max
  );
}

String plot_gz(){
  return String(millis()) + " " + String( IMU.g_raw.gyro.z, 6 ) + "\n";
}

String test_result = "";

void test( int lin, float w, uint16_t n, uint16_t dt ){
  test_result = "";
  String line = "n t lin w gyro out\n";
  test_result += line;
  uint32_t timeout = 0;
  int i=0;
  while(i<n){
    IMU.update();
    int out = pid_w( lin, w );
    if( millis() >= timeout ){
      i++;
      timeout = millis() + dt;
      line = String(i) + " " + String(millis()) + " " + String( lin ) + " " + String( w ) + " " + String( IMU.g.gyro.z ) + " " + String( out ) + "\n";
      test_result += line;
      Serial.print(line);
    }
  }
  motor.stop();
  ESPNOWSerial.println( test_result );
}

// loop command
String   loop_str = "";
uint32_t loop_dt = 100;
uint32_t loop_last_ms = 0;

#include "Terminal_local.h"

String ESPNOW_rcv_str = "";
void ESPNOWSerial_callback(const uint8_t *mac, const uint8_t *data, int size){
  if( size <= 0 || size > 255 ) return;
  //ESPNOW_rcv_str = (const char*) data; //ESPNOWSerial.readString();
  for(int i=0;i<size;i++) ESPNOW_rcv_str += (char)data[i];
}

void setup(void) {

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

  // adiciona os comandos locais ao terminal
  terminal_command_list_add( terminal_espnow );
  terminal_command_list_add( terminal_pid );
  terminal_command_list_add( terminal_bot );

  // inicia o controlador de velocidade angular
  PID_w.kp = 100;
  PID_w.ki = 1500;
  PID_w.kd = 0.8;
  PID_w.I_MAX = 1;
  terminal_new( T_PID_t, "pid_w", (uint32_t) &PID_w, 1, true );

  ESPNOWSerial.setReciveCallback( ESPNOWSerial_callback );

}

void loop() {

  // Alerta de baixa de baixa tensão
  if( voltage_alert ){
    if( voltage() > 2.0 && voltage() < 6.8 ){
      ESPNOWSerial.printf( "\n[ LOW BAT!! %f ]", voltage() );
      motor.bip(1,200,2000);
      motor.bip(1,200,5000);
    }
  }
  
  IMU.update();

  if( pid_auto == 1 ){
    pid_w( pid_auto_speed, pid_auto_angle );
  }

  // teste
  if(!digitalRead(0)){

    motor.stop();

    speed = get_speed( 1023,  1023, 10 );

    motor.bip( 10, 120, 1500 ); // 1200
    motor.bip( 10,  80, 1500 ); // 800
    motor.bip( 10,  50, 3500 ); // 500

    const int v = 500;
    speed_RL = get_gyro_z( -v,  v, 10, 150, 500 );
    speed_LR = get_gyro_z(  v, -v, 10, 150, 500 );
    speed_L  = get_gyro_z(  v,  0, 10, 150, 500 );
    speed_R  = get_gyro_z(  0,  v, 10, 150, 500 );

    const int v_max = 1023;
    speed_RL_max = get_gyro_z( -v_max,  v_max, 10, 150, 500 );
    speed_LR_max = get_gyro_z(  v_max, -v_max, 10, 150, 500 );
    speed_L_max  = get_gyro_z(  v_max,      0, 10, 150, 500 );
    speed_R_max  = get_gyro_z(      0,  v_max, 10, 150, 500 );

    get_gyro_z_curve( -1000, 1000, 20 );

  }

  // loop do terminal via Serial
  Terminal_loop();

  // ESPNOW recive command
  if( ESPNOW_rcv_str.length() > 0 ){
    String in = ESPNOW_rcv_str;
    ESPNOW_rcv_str = "";
    String result = "";
    terminal_command( &in, &result );
    Serial.println( result );
    ESPNOWSerial.print( result );
  }

  // loop command
  if( loop_str.length() > 0 ){
    if( loop_dt > 8 && millis() > (loop_last_ms+loop_dt) ){
      loop_last_ms = millis();
      String in     = loop_str;
      String result = "";
      terminal_command( &in, &result );
      //Serial.println( result );
      //ESPNOWSerial.print( result );
    }
  }

}

