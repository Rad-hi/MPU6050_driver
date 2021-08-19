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

/* Utils */

void i2c_init_mpu6050(MPU6050 *imu, TwoWire *i2c_port, uint8_t sda_pin, uint8_t scl_pin){
  imu->addr = I2C_MPU6050_ADDR;
  imu->i2c_handle_mpu6050 = i2c_port;
  imu->i2c_handle_mpu6050->begin(sda_pin, scl_pin, I2C_FREQ_HZ);
}

void i2c_write_byte_mpu6050(MPU6050 *imu, uint8_t reg_addr, uint8_t data_){
  imu->i2c_handle_mpu6050->beginTransmission(imu->addr);
  imu->i2c_handle_mpu6050->write(reg_addr);
  imu->i2c_handle_mpu6050->write(data_);
  imu->i2c_handle_mpu6050->endTransmission();
}

void i2c_write_read_mpu6050(MPU6050 *imu, uint8_t reg_addr, uint8_t *data_, size_t how_many){
  
  imu->i2c_handle_mpu6050->beginTransmission(imu->addr);
  imu->i2c_handle_mpu6050->write(reg_addr);
  imu->i2c_handle_mpu6050->endTransmission();
  
  uint16_t start_reading = millis();
  imu->i2c_handle_mpu6050->requestFrom(imu->addr, how_many);
  
  // Wait until all the bytes are received with a timeout                              
  while(imu->i2c_handle_mpu6050->available() < how_many 
        && (millis() - start_reading < I2C_TIMEOUT_MS)
       );

  for(uint8_t i = 0; i < how_many; i++){
    data_[i] = imu->i2c_handle_mpu6050->read();
  }
}

float twos_complement(uint16_t num){
  if(num & (1 << 15)) return (float)(num | ~((1 << 16) -1)); /* Num is negative --> Apply 2s complement */
  return (float)num;
}

/* Registers manipulations for setting u */

void i2c_setup_mpu6050(MPU6050 *imu, uint8_t mode_){
  
  switch(mode_){
    
    case MPU6050_FULL_INT:  /* Configure the interrupt and fall into full config */
      i2c_write_byte_mpu6050(imu, INT_ENABLE, 0x01); // Data ready interrupt enabled
      
    case MPU6050_FULL:      /* Do a full mode config */
      i2c_write_byte_mpu6050(imu, PWR_MGMT_1, 0x01); // Clock ref source to Gyro's X-axis
      i2c_write_byte_mpu6050(imu, GYRO_CONFIG, 0x00); // No Self-test, full scale +- 250 °/s
      i2c_write_byte_mpu6050(imu, ACCEL_CONFIG, 0x00); // No Self-test, full scale +- 2 g
      break;

    case MPU6050_ACC_INT:   /* Configure the interrupt and fall into Acc-only config */
      i2c_write_byte_mpu6050(imu, INT_ENABLE, 0x01); // Data ready interrupt enabled
      
    case MPU6050_ACC_ONLY:  /* Do an Acc only config */
      i2c_write_byte_mpu6050(imu, PWR_MGMT_1, 0x28); // Disable sleep, enable cycle, disable temp sensor, set internal clock as source
      i2c_write_byte_mpu6050(imu, PWR_MGMT_2, 0xC7); // Cycle = 40Hz, and all gyro axis are put to standby
      i2c_write_byte_mpu6050(imu, ACCEL_CONFIG, 0x00); // No Self-test, full scale +- 2 g
      break;

    default: break;
  }
}

void i2c_sleep_mpu6050(MPU6050 *imu){
  i2c_write_byte_mpu6050(imu, PWR_MGMT_1, 0x40); // Enable sleep
}

/* Reading data functions */

void i2c_read_acc_mpu6050(MPU6050 *imu){
  uint8_t rx_buf[6];
  i2c_write_read_mpu6050(imu, ACCEL_XOUT_H, rx_buf, sizeof(rx_buf));
  imu->acc_xyz[0] = twos_complement((uint16_t)((rx_buf[0] << 8) | rx_buf[1]));
  imu->acc_xyz[1] = twos_complement((uint16_t)((rx_buf[2] << 8) | rx_buf[3]));
  imu->acc_xyz[2] = twos_complement((uint16_t)((rx_buf[4] << 8) | rx_buf[5]));
}

void i2c_read_gyr_mpu6050(MPU6050 *imu){
  uint8_t rx_buf[6];
  i2c_write_read_mpu6050(imu, GYRO_XOUT_H, rx_buf, sizeof(rx_buf));
  imu->gyr_xyz[0] = twos_complement((uint16_t)((rx_buf[0] << 8) | rx_buf[1]));
  imu->gyr_xyz[1] = twos_complement((uint16_t)((rx_buf[2] << 8) | rx_buf[3]));
  imu->gyr_xyz[2] = twos_complement((uint16_t)((rx_buf[4] << 8) | rx_buf[5]));
}

void i2c_read_temp_mpu6050(MPU6050 *imu){
  uint8_t rx_buf[2];
  i2c_write_read_mpu6050(imu, TEMP_OUT_H, rx_buf, sizeof(rx_buf));
  float t = (float)((rx_buf[0] << 8) | rx_buf[1]);
  t = (t/340) + 36.53;
  imu->temp = t;
}
