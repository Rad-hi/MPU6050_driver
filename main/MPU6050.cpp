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
 *      one is slightly different from the other.
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

static MPU6050 * local_imu;
static volatile bool data_ready = false;
static uint8_t local_mode;

/* Utils */

void i2c_init_mpu6050(MPU6050 *imu, TwoWire *i2c_port){
  imu->addr = I2C_MPU6050_ADDR;
  imu->i2c_handle_mpu6050 = i2c_port;
  imu->i2c_handle_mpu6050->begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

  local_imu = imu;
}

static void i2c_write_byte_mpu6050(uint8_t reg_addr, uint8_t data_){
  local_imu->i2c_handle_mpu6050->beginTransmission(local_imu->addr);
  local_imu->i2c_handle_mpu6050->write(reg_addr);
  local_imu->i2c_handle_mpu6050->write(data_);
  local_imu->i2c_handle_mpu6050->endTransmission();
}

static void i2c_write_read_mpu6050(uint8_t reg_addr, uint8_t *data_, size_t how_many){
  
  local_imu->i2c_handle_mpu6050->beginTransmission(local_imu->addr);
  local_imu->i2c_handle_mpu6050->write(reg_addr);
  local_imu->i2c_handle_mpu6050->endTransmission();
  
  uint16_t start_reading = millis();
  local_imu->i2c_handle_mpu6050->requestFrom(local_imu->addr, how_many);
  
  // Wait until all the bytes are received with a timeout                              
  while(local_imu->i2c_handle_mpu6050->available() < how_many 
        && (millis() - start_reading < I2C_TIMEOUT_MS)
       );

  for(uint8_t i = 0; i < how_many; i++){
    data_[i] = local_imu->i2c_handle_mpu6050->read();
  }
}

static float twos_complement(uint16_t num){
  if(num & (1 << 15)) return (float)(num | ~((1 << 16) -1)); /* Num is negative --> Apply 2s complement */
  return (float)num;
}

/* Call this if you're working with interrupts, instead, use the normal reading functions */
void i2c_read_with_interrupt(IMU_DATA_t * data){
  
  static IMU_DATA_t imu_data_reg;
  
  if(data_ready){
    switch(local_mode){
      case MPU6050_FULL_INT:
        i2c_read_acc_mpu6050(&imu_data_reg);
        i2c_read_gyr_mpu6050(&imu_data_reg);
        i2c_read_temp_mpu6050(&imu_data_reg);
      break;
      
      case MPU6050_ACC_INT:
        i2c_read_acc_mpu6050(&imu_data_reg);
      break;
    }
    data_ready = false;
  }

  // We always return the latest read data
  memcpy((void*)data, (void*)&imu_data_reg, sizeof imu_data_reg);
}

static void data_ready_isr(void){
  data_ready = true;
}

/* Registers manipulations for setting u */


#define ENABLE_INTERRUPT(PIN) {\
  pinMode(PIN, INPUT_PULLUP);\
  attachInterrupt(digitalPinToInterrupt(PIN), data_ready_isr, FALLING);\
}

void i2c_setup_mpu6050(uint8_t mode_){
  local_mode = mode_;
  
  switch(mode_){
    
    case MPU6050_FULL_INT:  /* Configure the interrupt and fall into full config */
      i2c_write_byte_mpu6050(INT_ENABLE, 0x01); // Data ready interrupt enabled
      ENABLE_INTERRUPT(I2C_INTERRUPT_PIN)
      
    case MPU6050_FULL:      /* Do a full mode config */
      i2c_write_byte_mpu6050(PWR_MGMT_1, 0x01); // Clock ref source to Gyro's X-axis
      i2c_write_byte_mpu6050(GYRO_CONFIG, 0x00); // No Self-test, full scale +- 250 °/s
      i2c_write_byte_mpu6050(ACCEL_CONFIG, 0x00); // No Self-test, full scale +- 2 g
      break;

    case MPU6050_ACC_INT:   /* Configure the interrupt and fall into Acc-only config */
      i2c_write_byte_mpu6050(INT_ENABLE, 0x01); // Data ready interrupt enabled
      ENABLE_INTERRUPT(I2C_INTERRUPT_PIN);
       
    case MPU6050_ACC_ONLY:  /* Do an Acc only config */
      i2c_write_byte_mpu6050(PWR_MGMT_1, 0x28); // Disable sleep, enable cycle, disable temp sensor, set internal clock as source
      i2c_write_byte_mpu6050(PWR_MGMT_2, 0xC7); // Cycle = 40Hz, and all gyro axis are put to standby
      i2c_write_byte_mpu6050(ACCEL_CONFIG, 0x00); // No Self-test, full scale +- 2 g
      break;

    default: break;
  }
}

void i2c_sleep_mpu6050(){
  i2c_write_byte_mpu6050(PWR_MGMT_1, 0x40); // Enable sleep
}

/* Reading data functions */

void i2c_read_acc_mpu6050(IMU_DATA_t * data){
  uint8_t rx_buf[6];
  i2c_write_read_mpu6050(ACCEL_XOUT_H, rx_buf, sizeof(rx_buf));
  data->acc_xyz[0] = twos_complement((uint16_t)((rx_buf[0] << 8) | rx_buf[1]));
  data->acc_xyz[1] = twos_complement((uint16_t)((rx_buf[2] << 8) | rx_buf[3]));
  data->acc_xyz[2] = twos_complement((uint16_t)((rx_buf[4] << 8) | rx_buf[5]));
}

void i2c_read_gyr_mpu6050(IMU_DATA_t * data){
  uint8_t rx_buf[6];
  i2c_write_read_mpu6050(GYRO_XOUT_H, rx_buf, sizeof(rx_buf));
  data->gyr_xyz[0] = twos_complement((uint16_t)((rx_buf[0] << 8) | rx_buf[1]));
  data->gyr_xyz[1] = twos_complement((uint16_t)((rx_buf[2] << 8) | rx_buf[3]));
  data->gyr_xyz[2] = twos_complement((uint16_t)((rx_buf[4] << 8) | rx_buf[5]));
}

void i2c_read_temp_mpu6050(IMU_DATA_t * data){
  uint8_t rx_buf[2];
  i2c_write_read_mpu6050(TEMP_OUT_H, rx_buf, sizeof(rx_buf));
  float t = (float)((rx_buf[0] << 8) | rx_buf[1]);
  t = (t/340) + 36.53;
  data->temp = t;
}
