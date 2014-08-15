#include <PID_v1.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "IMU.h"

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

double x_gyro_offset = 0;
double y_gyro_offset = 0;
double Roll_Setpoint = 0;
double Pitch_Setpoint = 0;
double Yaw_Setpoint = 0;

struct Gain Roll_Gains = {10,0,0};
struct Gain Pitch_Gains = {10,0,0};
struct Gain Yaw_Gains = {10,0,0};

int OutputHiLim = 100;
int OutputLoLim = -100;
double Pitch_Output;
double Roll_Output;
double Yaw_Output;
struct Angle Angle_val;
unsigned long init_time = micros();

PID pitchController(&Angle_val.pitch, &Pitch_Output, &Pitch_Setpoint,Pitch_Gains.Kp,Pitch_Gains.Ki,Pitch_Gains.Kd, DIRECT);
PID rollController(&Angle_val.roll, &Roll_Output, &Roll_Setpoint,Roll_Gains.Kp,Roll_Gains.Ki,Roll_Gains.Kd, DIRECT);
PID yawController(&Angle_val.yaw, &Yaw_Output, &Yaw_Setpoint,Yaw_Gains.Kp,Yaw_Gains.Ki,Yaw_Gains.Kd, DIRECT);

void setup()
{      
    initAngleCalc();
    
    pitchController.SetMode(AUTOMATIC);
    pitchController.SetSampleTime(10);
    pitchController.SetOutputLimits(OutputLoLim,OutputHiLim);
    rollController.SetMode(AUTOMATIC);
    rollController.SetOutputLimits(OutputLoLim,OutputHiLim);
    rollController.SetSampleTime(10);
    yawController.SetMode(AUTOMATIC);
    yawController.SetOutputLimits(OutputLoLim,OutputHiLim);
    yawController.SetSampleTime(10);
}


void loop()
{
  
  if(micros()-init_time >= 10000)
  {
  Angle_val = AngleCalc();
  pitchController.Compute();
  rollController.Compute();
  yawController.Compute();
  
  Serial.print(Angle_val.roll_accel,3);
  Serial.print(F(", "));
  Serial.print(Angle_val.roll_gyro,3);
  Serial.print(F(", "));
  Serial.print(Angle_val.roll,3);
  Serial.print(F(", "));
  Serial.print(Angle_val.yaw,3);
  Serial.println(F(""));
  init_time = micros();
  }
}





//======================================
//
//    FUNCTIONS
//
//======================================



void initAngleCalc(){
   IMU_init();
   mag.begin();
 
  struct IMU IMU_val;
  
  for(int i=0; i<10; i++){
  IMU_val = IMU_read();
  x_gyro_offset = IMU_val.x_gyro + x_gyro_offset;
  y_gyro_offset = IMU_val.y_gyro + y_gyro_offset;
  delay(5);
  };
  x_gyro_offset = x_gyro_offset/10;
  y_gyro_offset = y_gyro_offset/10; 
}


struct Angle AngleCalc(){
   struct IMU IMU_val;
  static double angle_x_accel=0;
  static double angle_x_gyro=0;
  static double angle_x_comp=0;
  static double angle_x_comp_prev=0;
  static double angle_y_accel=0;
  static double angle_y_gyro=0;
  static double angle_y_comp=0;
  static double angle_y_comp_prev=0;
  IMU_val = IMU_read();   

  angle_x_accel = (atan2(IMU_val.y_accel,IMU_val.z_accel))*(180/3.1415926); //Accel Angle Calculation
  angle_y_accel = (atan2(-IMU_val.x_accel,sqrt(IMU_val.y_accel*IMU_val.y_accel+IMU_val.z_accel*IMU_val.z_accel)))*(180/3.1415926); //Accel Angle Calculation
  IMU_val.x_gyro = (IMU_val.x_gyro-x_gyro_offset)*1.5;  //Gyro Calibration
  IMU_val.y_gyro = (IMU_val.y_gyro-y_gyro_offset)*.15;  //Gyro Calibration
  angle_x_gyro = ((IMU_val.x_gyro)*0.01)+angle_x_gyro;  //Gyro Angle Calculation (For comparison only)
  angle_y_gyro = ((IMU_val.y_gyro)*0.01)+angle_y_gyro;  //Gyro Angle Calculation (For comparison only)
  angle_x_comp = (((IMU_val.x_gyro*0.01)+angle_x_comp)*0.98)+(angle_x_accel*0.02);  //Complementary Filter
  angle_y_comp = (((IMU_val.y_gyro*0.01)+angle_y_comp)*0.98)+(angle_y_accel*0.02);  //Complementary Filter
 
   if (abs(angle_x_comp - angle_x_comp_prev) > 30)
     angle_x_comp = angle_x_comp_prev; 
     
   if (abs(angle_y_comp - angle_y_comp_prev) > 30)
     angle_y_comp = angle_y_comp_prev; 
     
     
   angle_x_comp_prev = angle_x_comp;
   angle_y_comp_prev = angle_y_comp;
   
   sensors_event_t event; 
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.227;
  heading += declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
    
    if(heading > PI)
    heading = PI-(heading-PI);
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
   
   struct Angle Angle_val;
   
   Angle_val.roll = angle_x_comp;
   Angle_val.pitch = angle_y_comp;
   Angle_val.roll_accel = angle_x_accel;
   Angle_val.pitch_accel = angle_y_accel;
   Angle_val.roll_gyro = angle_x_gyro;
   Angle_val.pitch_gyro = angle_y_gyro;
   Angle_val.yaw = headingDegrees;
  
   return Angle_val;
}


void IMU_init(){
    int error;
  uint8_t c;
  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();
  
  Serial.begin(115200);
  
  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. 
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  
  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);
}

struct IMU IMU_read(){
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  struct IMU IMU_val;
  double angle_accel;
  static double angle_gyro = 0;

  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));

  // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

  IMU_val.x_accel = accel_t_gyro.value.x_accel;
  IMU_val.y_accel = accel_t_gyro.value.y_accel;
  IMU_val.z_accel = accel_t_gyro.value.z_accel;
  IMU_val.x_accel = (IMU_val.x_accel/32767)*2;
  IMU_val.y_accel = (IMU_val.y_accel/32767)*2;
  IMU_val.z_accel = (IMU_val.z_accel/32767)*2;
  
  IMU_val.x_gyro = accel_t_gyro.value.x_gyro;
  IMU_val.y_gyro = accel_t_gyro.value.y_gyro;
  IMU_val.z_gyro = accel_t_gyro.value.z_gyro;
  IMU_val.x_gyro = (IMU_val.x_gyro/32767)*250;
  IMU_val.y_gyro = (IMU_val.y_gyro/32767)*250;
  IMU_val.z_gyro = (IMU_val.z_gyro/32767)*250;
  
  return IMU_val;
}

// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}
