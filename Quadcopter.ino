#include <Wire.h>
#include <math.h>

#include "IMU.h"

double x_gyro_offset = 0;
double y_gyro_offset = 0;


void setup()
{      
    initAngleCalc();
}


void loop()
{
  
  struct Angle Angle_val;
  
  Angle_val = AngleCalc();
  
  Serial.print(Angle_val.x,6);
  Serial.print(F(", "));
  Serial.print(Angle_val.y,6);
  Serial.println(F(""));
  
  delay(10);
}





//======================================
//
//    FUNCTIONS
//
//======================================



void initAngleCalc(){
   IMU_init();
 
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
  IMU_val.x_gyro = (IMU_val.x_gyro-x_gyro_offset)*2.3;  //Gyro Calibration
  IMU_val.y_gyro = (IMU_val.y_gyro-y_gyro_offset)*2.098;  //Gyro Calibration
  //angle_x_gyro = ((IMU_val.x_gyro)*0.01)+angle_x_gyro;  //Gyro Angle Calculation (For comparison only)
  //angle_y_gyro = ((IMU_val.y_gyro)*0.01)+angle_y_gyro;  //Gyro Angle Calculation (For comparison only)
  angle_x_comp = (((IMU_val.x_gyro*0.01)+angle_x_comp)*0.98)+(angle_x_accel*0.02);  //Complementary Filter
  angle_y_comp = (((IMU_val.y_gyro*0.01)+angle_y_comp)*0.98)+(angle_y_accel*0.02);  //Complementary Filter
 
   if (abs(angle_x_comp - angle_x_comp_prev) > 30)
     angle_x_comp = angle_x_comp_prev; 
     
   if (abs(angle_y_comp - angle_y_comp_prev) > 30)
     angle_y_comp = angle_y_comp_prev; 
     
     
   angle_x_comp_prev = angle_x_comp;
   angle_y_comp_prev = angle_y_comp;
   
   struct Angle Angle_val;
   
   Angle_val.x = angle_x_comp;
   Angle_val.y = angle_y_comp;
  
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
