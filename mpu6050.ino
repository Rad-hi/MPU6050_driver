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

#include "MPU6050.h"

#define PRINT_RAW_DATA      0 /* If non-null, will print raw data; else will print values in G readings */

TwoWire i2c_two_wire = TwoWire(0);
MPU6050 my_imu;

/* Printing buffer */
char buf[255];

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- STARTED ---");

  /* Ref to MPU6050 type, ref to i2c port, sda pin, scl_pin */
  i2c_init_mpu6050(&my_imu, &i2c_two_wire, I2C_SDA_PIN, I2C_SCL_PIN);

  /* Ref to MPU6050 type, operation mode */
  i2c_setup_mpu6050(&my_imu, MPU6050_ACC_ONLY);
}

void loop() {
  i2c_read_acc_mpu6050(&my_imu);
  
  #if PRINT_RAW_DATA
    sprintf(buf, "Accelerometer data: X = %f, Y = %f, Z = %f", my_imu.acc_xyz[0], my_imu.acc_xyz[1], my_imu.acc_xyz[2]);
  #else  
    sprintf(buf, "Accelerometer data: X = %f, Y = %f, Z = %f", my_imu.acc_xyz[0]/16384, my_imu.acc_xyz[1]/16384, my_imu.acc_xyz[2]/16384);
  #endif
  
  Serial.println(buf);

  delay(500);
}
