#include <Arduino.h>
#include "MPU9250.h"

#define FIFO_SIZE 50

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

float ax, ay, az, gx, gy, gz, mx, my, mz, temp ; 
float yaw=0, pitch=0, roll=0 ; 
float gyro_pitch=0, gyro_roll=0, gyro_yaw=0 ; 
bool first_time_gyro = true ; 
bool first_time_mag = true ; 
float dt = 0.01 ; 
float alpha = 0.1 ; 
float Pk_yaw = 0.9 ; 
float Pk_pitch = 0.9 ; 
float Pk_roll = 0.9 ; 
float Q_yaw = 0.002 ; 
float Q_pitch = 0.00001 ; 
float Q_roll = 0.00001 ; 
float R_yaw = 0.01 ; 
float R_pitch = 100 ; 
float R_roll = 100 ; 
float Kk_yaw = 0.5 ; 
float Kk_pitch = 0.5 ; 
float Kk_roll = 0.5 ; 
float accel_roll_fifo[FIFO_SIZE] = {} ; 
float accel_roll_ux = 0 ; 
float accel_roll_variance = 0 ; 
float gx_fifo[FIFO_SIZE] = {} ; 
float gx_ux = 0 ; 
float gx_variance = 0 ; 

float accel_pitch_fifo[FIFO_SIZE] = {} ; 
float accel_pitch_ux = 0 ; 
float accel_pitch_variance = 0 ; 
float gy_fifo[FIFO_SIZE] = {} ; 
float gy_ux = 0 ; 
float gy_variance = 0 ; 

float mag_yaw_fifo[FIFO_SIZE] = {} ; 
float mag_yaw_ux = 0 ; 
float mag_yaw_variance = 0 ; 

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
}

void loop() {
  unsigned int t1 = millis() ; 
  // read the sensor
  IMU.readSensor();

  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT();
  my = IMU.getMagY_uT();
  mz = IMU.getMagZ_uT();
  temp = IMU.getTemperature_C();

  // Magnetometer Hard Iron Calibration
  mx += 3.345 ; 
  my -= 58.07 ; 
  mz += 17.31 ; 

  float accel_pitch = ((9.0/8.0) * asin(ax/sqrt(ax*ax + ay*ay + az*az))) + 2.5*DEG_TO_RAD ;                  // Valid (Website)
  // float accel_pitch = atan2(ax, sqrt(ay*ay + az*az)) ;                     // My Guess
  // float accel_roll = atan2(ay, az) ;                                       // dont Know (Website)
  // float accel_roll = asin(ay/sqrt(ax*ax + az*az + ay*ay)) ;                // My Guess
  float accel_roll = ((9.0/8.2) * atan2(ay, sqrt(ax*ax + az*az))) + 1.0*DEG_TO_RAD ;                         // GPT (but for pitch !!)
  accel_roll_ux = accel_roll ; 
  for (int i=0 ; i<FIFO_SIZE-1 ; i++)
  {
    accel_roll_fifo[i] = accel_roll_fifo[i+1] ; 
    accel_roll_ux += accel_roll_fifo[i] ; 
  }
  accel_roll_fifo[FIFO_SIZE-1] = accel_roll ; 
  accel_roll_ux /= float(FIFO_SIZE) ; 
  accel_roll_variance = 0 ; 
  for (int i=0 ; i<FIFO_SIZE ; i++)
  {
    accel_roll_variance += (accel_roll_fifo[i] - accel_roll_ux) * (accel_roll_fifo[i] - accel_roll_ux) ; 
  }
  accel_roll_variance /= (FIFO_SIZE-1) ; 


  accel_pitch_ux = accel_pitch ; 
  for (int i=0 ; i<FIFO_SIZE-1 ; i++)
  {
    accel_pitch_fifo[i] = accel_pitch_fifo[i+1] ; 
    accel_pitch_ux += accel_pitch_fifo[i] ; 
  }
  accel_pitch_fifo[FIFO_SIZE-1] = accel_pitch ; 
  accel_pitch_ux /= float(FIFO_SIZE) ; 
  accel_pitch_variance = 0 ; 
  for (int i=0 ; i<FIFO_SIZE ; i++)
  {
    accel_pitch_variance += (accel_pitch_fifo[i] - accel_pitch_ux) * (accel_pitch_fifo[i] - accel_pitch_ux) ; 
  }
  accel_pitch_variance /= (FIFO_SIZE-1) ; 



  gx_ux = gx ; 
  for (int i=0 ; i<FIFO_SIZE-1 ; i++)
  {
    gx_fifo[i] = gx_fifo[i+1] ; 
    gx_ux += gx_fifo[i] ; 
  }
  gx_fifo[FIFO_SIZE-1] = gx ; 
  gx_ux /= float(FIFO_SIZE) ; 
  gx_variance = 0 ; 
  for (int i=0 ; i<FIFO_SIZE ; i++)
  {
    gx_variance += (gx_fifo[i] - gx_ux) * (gx_fifo[i] - gx_ux) ; 
  }
  gx_variance /= (FIFO_SIZE-1) ; 


  gy_ux = gy ; 
  for (int i=0 ; i<FIFO_SIZE-1 ; i++)
  {
    gy_fifo[i] = gy_fifo[i+1] ; 
    gy_ux += gy_fifo[i] ; 
  }
  gy_fifo[FIFO_SIZE-1] = gy ; 
  gy_ux /= float(FIFO_SIZE) ; 
  gy_variance = 0 ; 
  for (int i=0 ; i<FIFO_SIZE ; i++)
  {
    gy_variance += (gy_fifo[i] - gy_ux) * (gy_fifo[i] - gy_ux) ; 
  }
  gy_variance /= (FIFO_SIZE-1) ; 

  // float mag_yaw = atan2(my, mx) ; 

  if (first_time_gyro)
  {
    pitch = accel_pitch ; 
    roll = accel_roll ; 
    // yaw = mag_yaw ; 
    first_time_gyro = false ; 
  }

  gyro_pitch = pitch + (gy * dt);
  gyro_roll = roll - (gx * dt);
  // gyro_yaw = gyro_yaw + (gz * dt);

  Q_pitch = sqrt(gy_variance) ; 
  R_pitch = 500*sqrt(accel_pitch_variance) ; 
  float Pk_p = Pk_pitch + Q_pitch ; 
  Kk_pitch = (Pk_p) / (Pk_p + R_pitch) ; 

  Q_roll = sqrt(gx_variance) ; 
  R_roll = 500*sqrt(accel_roll_variance) ; 
  float Pk_r = Pk_roll + Q_roll ; 
  Kk_roll = (Pk_r) / (Pk_r + R_roll) ; 

  // Tilt Compensated Yaw
  float Ym = my*cos(roll) + mz*sin(roll) ; 
  float Xm = mx*cos(pitch) - my*sin(roll)*sin(pitch) + mz*cos(roll)*sin(pitch) ; 
  float mag_yaw = atan2(my, mx) ; 

  mag_yaw_ux = mag_yaw ; 
  for (int i=0 ; i<FIFO_SIZE-1 ; i++)
  {
    mag_yaw_fifo[i] = mag_yaw_fifo[i+1] ; 
    mag_yaw_ux += mag_yaw_fifo[i] ; 
  }
  mag_yaw_fifo[FIFO_SIZE-1] = mag_yaw ; 
  mag_yaw_ux /= float(FIFO_SIZE) ; 
  mag_yaw_variance = 0 ; 
  for (int i=0 ; i<FIFO_SIZE ; i++)
  {
    mag_yaw_variance += (mag_yaw_fifo[i] - mag_yaw_ux) * (mag_yaw_fifo[i] - mag_yaw_ux) ; 
  }
  mag_yaw_variance /= (FIFO_SIZE-1) ; 

  // float Pk_y = Pk_yaw + Q_yaw ; 
  // Kk_yaw = (Pk_y) / (Pk_y + R_yaw) ; 

  // Kalman Filter
  pitch = gyro_pitch + Kk_pitch * (accel_pitch - gyro_pitch) ; 
  roll = gyro_roll + Kk_roll * (accel_roll - gyro_roll) ; 
  // yaw = mag_yaw + Kk_yaw * (gyro_yaw - mag_yaw) ; 
  Pk_pitch = (1 - Kk_pitch) * Pk_p ; 
  Pk_roll = (1 - Kk_roll) * Pk_r ; 
  // Pk_yaw = (1 - Kk_yaw) * Pk_y ; 

  // complementary filter
  // pitch = alpha * gyro_pitch + (1 - alpha) * accel_pitch;
  // roll = alpha * gyro_roll + (1 - alpha) * accel_roll;
  // yaw = alpha * gyro_yaw + (1 - alpha) * mag_yaw;

  unsigned int t2 = millis() ; 

  Serial.print("roll: ") ;
  Serial.print(roll * RAD_TO_DEG) ;  
  Serial.print("   pitch: ") ;
  Serial.print(pitch * RAD_TO_DEG) ;  
  Serial.print("   mag_yaw: ") ;
  Serial.print(mag_yaw_ux * RAD_TO_DEG) ;  
  Serial.print("   time: ") ; 
  Serial.println(t2-t1) ;  

  delay(10);
}