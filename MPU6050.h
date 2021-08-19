/* 
 * Written by Radhi SGHAIER: https://github.com/Rad-hi
 * --------------------------------------------------------
 * Do whatever you want with the code ...
 * If this was ever useful to you, and we happened to meet on the street, 
 * I'll appreciate a cup of dark coffee, no sugar please.
 * --------------------------------------------------------
 * How to use:
 * You'll need an MPU6050 Accelerometer and Gyroscope I²C module, 
 * You'll have to configure the SCL, and SDA pins in the MPU6050.h header file
 * Then follow the template provided in this file.
 * --------------------------------------------------------
 * TODO: 
 *    - Implement the calibration routine since no sensor is perfect, and each
 *      one is slightly different than the other.
 *      (yes, talking about the exact same model)
 *    - Implement code that reads data upon interruption from the MPU6050 module.
 * --------------------------------------------------------
 */

#ifndef __MY_MPU6050_H__
  #define __MY_MPU6050_H__
  
  #include <Wire.h>
  #include "Arduino.h"
  
  /* Device address */
  #define I2C_MPU6050_ADDR      0x68

  /* Pins definition */
  #define I2C_SDA_PIN           18
  #define I2C_SCL_PIN           19

  /* Operation modes */
  #define MPU6050_FULL_INT      0           /* Accelerometer & Gyroscope & Temperature & Interrupt    */
  #define MPU6050_FULL          1           /* Accelerometer & Gyroscope & Temperature & NO Interrupt */
  #define MPU6050_ACC_INT       2           /* Accelerometer only & Interrupt                         */
  #define MPU6050_ACC_ONLY      3           /* Accelerometer only & NO Interrupt                      */
    
  /* MPU6050 Registers [consult the datasheet] */
  #define SMPRT_DIV             0x19        /* Sample rate divider of the clock (MAX ACCEL = 1KHz)    */
  #define GYRO_CONFIG           0x1B        /* Gyroscope config register, set the full scale here     */
  #define ACCEL_CONFIG          0x1C        /* Accelerometer config register, set the full scale here */
  #define INT_ENABLE            0x38        /* Enable interrupt generation, interested in data ready! */
  #define ACCEL_XOUT_H          0x3B        /* Acceleromter data, 16-bit 2's complement value(X,Y,Z)  */ 
  #define ACCEL_XOUT_L          0x3C
  #define ACCEL_YOUT_H          0x3D
  #define ACCEL_YOUT_L          0x3E
  #define ACCEL_ZOUT_H          0x3F
  #define ACCEL_ZOUT_L          0x40
  #define TEMP_OUT_H            0x41        /* Temperature data, 16-bit signed, T°C = Temp/340+36.53  */
  #define TEMP_OUT_L            0x42
  #define GYRO_XOUT_H           0x43        /* Gyroscope data, 16-bit 2's complement value (X, Y, Z)  */
  #define GYRO_XOUT_L           0x44
  #define GYRO_YOUT_H           0x45
  #define GYRO_YOUT_L           0x46
  #define GYRO_ZOUT_H           0x47
  #define GYRO_ZOUT_L           0x48
  #define PWR_MGMT_1            0x6B        /* Power management register(sleep,disable sensors,cycle) */
  #define PWR_MGMT_2            0x6C
  #define WHO_AM_I              0x75        /* I²C address, 7-bit, add[0]=AD0 pin, default add = 0x68 */

  /* I²C configs in the ESP32 */
  #define I2C_PORT_NUM          0           /* The number of I²C ports is chip specific               */
  #define I2C_FREQ_HZ           400000
  #define I2C_TIMEOUT_MS        1000

  /* Data container */
  typedef struct{  
    TwoWire * i2c_handle_mpu6050;           /* Handle to the I²C port on which the imu is mounted     */
    uint8_t addr;                           /* Slave address, 0x68 in our case                        */
    float acc_xyz[3];
    float temp;
    float gyr_xyz[3];
  }MPU6050;

  /* Functions */
  void i2c_init_mpu6050(MPU6050 *imu, TwoWire *i2c_port, uint8_t sda_pin, uint8_t scl_pin); 
  void i2c_setup_mpu6050(MPU6050 *imu, uint8_t mode_); 
  void i2c_read_acc_mpu6050(MPU6050 *imu); 
  void i2c_read_gyr_mpu6050(MPU6050 *imu);
  void i2c_read_temp_mpu6050(MPU6050 *imu);
  void i2c_write_byte_mpu6050(MPU6050 *imu, uint8_t reg_addr, uint8_t data_);
  void i2c_write_read_mpu6050(MPU6050 *imu, uint8_t reg_addr, uint8_t *data_, size_t how_many);
  void i2c_sleep_mpu6050(MPU6050 *imu); 
  float twos_complement(uint16_t num);    /* The ACC and GYR data is returned in twos complement    */
  
#endif // __MY_MPU6050_H__
