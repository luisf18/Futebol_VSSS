#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;


class IMU_class{
  public:
  uint32_t last_ms = 0;
  float kgz = 0;
  float kgz0 = 0;

  bool begin(void){

    // Try to initialize!
    if (!mpu.begin()) {
      Serial.println("Failed to find MPU6050 chip");
      return false;
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    Serial.print("Accelerometer range set to: ");
    switch (mpu.getAccelerometerRange()) {
    case MPU6050_RANGE_2_G:
      Serial.println("+-2G");
      break;
    case MPU6050_RANGE_4_G:
      Serial.println("+-4G");
      break;
    case MPU6050_RANGE_8_G:
      Serial.println("+-8G");
      break;
    case MPU6050_RANGE_16_G:
      Serial.println("+-16G");
      break;
    }
    mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
    Serial.print("Gyro range set to: ");
    switch (mpu.getGyroRange()) {
    case MPU6050_RANGE_250_DEG:
      Serial.println("+- 250 deg/s");
      break;
    case MPU6050_RANGE_500_DEG:
      Serial.println("+- 500 deg/s");
      break;
    case MPU6050_RANGE_1000_DEG:
      Serial.println("+- 1000 deg/s");
      break;
    case MPU6050_RANGE_2000_DEG:
      Serial.println("+- 2000 deg/s");
      break;
    }

    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.print("Filter bandwidth set to: ");
    switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
    }

    Serial.println("");
    delay(100);
    return true;
  }

  sensors_event_t a_raw, g_raw, temp_raw;
  sensors_event_t a, g, temp;
  float gyro_z_angle = 0;
  float ac_z_filter  = 0;

  void calibrate(){
    uint32_t ms = millis();
    mpu.getEvent(&a, &g, &temp);
    float gz0 = g.gyro.z;
    delay(2000);
    mpu.getEvent(&a, &g, &temp);
    kgz = (g.gyro.z - gz0)/(millis()-ms);
    Serial.printf( "kgz: %f kgz0: %f \n", kgz, kgz0 );
    kgz0 = (g.gyro.z + gz0)/2.0;
  }

  void update(){
    uint32_t ms = millis();
    mpu.getEvent(&a_raw, &g_raw, &temp_raw);
    a = a_raw;
    g = g_raw;
    temp = temp_raw;
    uint32_t dt = ms-last_ms;
    g.gyro.z -= dt*kgz;
    g.gyro.z -= kgz0;

    ac_z_filter = a.acceleration.z*0.1 + ac_z_filter*0.9;
    
    gyro_z_angle += 0.001*dt*( (g.gyro.z >= 0.0 ? g.gyro.z : -g.gyro.z ) < 0.01 ? 0.0 : g.gyro.z ); // radianos
    last_ms = ms;

  }

  String str( ){ return str(true,true,true); }
  String str( bool Ac, bool G, bool Temp ){

    String result = "";

    if( Ac   ) result += ( " " + String(a.acceleration.x,2) + " " + String(a.acceleration.y,2) + " " + String(a.acceleration.z,2) );
    if( G    ) result += ( " " + String(g.gyro.x,2) + " " + String(g.gyro.y,2) + " " + String(g.gyro.z,2) );
    if( Temp ) result += ( " " + String(temp.temperature,2) );

    return result;

  }

};


IMU_class IMU;