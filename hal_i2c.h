// File from element14 EVK sensors example code - no rights reserved

#ifndef __HAL_I2C_H__
#define __HAL_I2C_H__

#include "freedom.h"

/* ----------------------------------------------------------------------------
   -- I2C
   ---------------------------------------------------------------------------- */

void i2c_set_tx_mode(I2C_MemMapPtr p);
void i2c_set_rx_mode(I2C_MemMapPtr p);
void i2c_set_slave_mode(I2C_MemMapPtr p);
void i2c_set_master_mode(I2C_MemMapPtr p);
void i2c_give_nack(I2C_MemMapPtr p);
void i2c_give_ack(I2C_MemMapPtr p);
void i2c_repeated_start(I2C_MemMapPtr p);
void i2c_write_byte(I2C_MemMapPtr p, uint8_t data);
uint8_t i2c_read_byte(I2C_MemMapPtr p);
void i2c_start(I2C_MemMapPtr p);
void i2c_stop(I2C_MemMapPtr p);
void i2c_wait(I2C_MemMapPtr p);
uint16_t i2c_get_ack(I2C_MemMapPtr p);
void hal_i2c_write(I2C_MemMapPtr I2C_id, char dev_addr, char reg, char value);
uint8_t hal_i2c_read(I2C_MemMapPtr I2C_id, char dev_addr, char reg);
uint8_t hal_i2c_BulkRead(I2C_MemMapPtr I2C_id, char dev_addr, char StartReg, char Length, char *buf);

#define I2C_READ  1
#define I2C_WRITE 0
#define I2C0_B  I2C0_BASE_PTR
#define I2C1_B  I2C1_BASE_PTR


#endif //__HAL_I2C_H__

