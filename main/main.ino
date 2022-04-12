/* 
 * -----------------------------------------------------------------------------
 * MPU6050 Driver written from scratch referencing the datasheet, in C style.
 * -----------------------------------------------------------------------------
 * How to use:
 * - You'll need an MPU6050 Accelerometer and Gyroscope I²C module, 
 * - You'll have to configure the SCL, and SDA pins in the MPU6050.h header file
 *   (and the interrupt pin, if you choose to use it)
 * - Then follow the template provided in this file.
 * -----------------------------------------------------------------------------
 * TODO: 
 *    - Implement the calibration routine since no sensor is perfect, and each
 *      one is slightly different than the other.
 *      (yes, talking about the exact same model)
 * DONE:
 *    - Implement code that reads data upon interruption from the MPU6050 module.
 * -----------------------------------------------------------------------------
 * Author: Radhi SGHAIER: https://github.com/Rad-hi
 * -----------------------------------------------------------------------------
 * Date: 12-04-2022 (12th of April, 2022)
 * -----------------------------------------------------------------------------
 * License: Do whatever you want with the code ...
 *          If this was ever useful to you, and we happened to meet on 
 *          the street, I'll appreciate a cup of dark coffee, no sugar please.
 * -----------------------------------------------------------------------------
 */
 
#include "MPU6050.h"

#define PRINT_RAW_DATA      0 /* If non-null, will print raw data; else will print values in G readings */

TwoWire i2c_two_wire = TwoWire(0);
MPU6050 my_imu;

IMU_DATA_t my_imu_data;

/* Printing buffer */
char buf[255];

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("--- STARTED ---");

  i2c_init_mpu6050(&my_imu, &i2c_two_wire);
  i2c_setup_mpu6050(MPU6050_ACC_INT);
}

void loop() {
  i2c_read_with_interrupt(&my_imu_data);
  
  #if PRINT_RAW_DATA
    sprintf(buf, "Accelerometer data: X = %f, Y = %f, Z = %f", my_imu_data.acc_xyz[0], my_imu_data.acc_xyz[1], my_imu_data.acc_xyz[2]);
  #else  
    sprintf(buf, "Accelerometer data: X = %f, Y = %f, Z = %f", 
                 my_imu_data.acc_xyz[0]/RESOLUTION, 
                 my_imu_data.acc_xyz[1]/RESOLUTION, 
                 my_imu_data.acc_xyz[2]/RESOLUTION);
  #endif
  
  Serial.println(buf);

  delay(500);
}
