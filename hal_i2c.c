// File from element14 EVK sensors example code - no rights reserved

#include "hal_i2c.h"
#include "common.h"


void i2c_set_tx_mode(I2C_MemMapPtr p)
{
    p->C1 |= I2C_C1_TX_MASK;

}
void i2c_set_rx_mode(I2C_MemMapPtr p)
{
    p->C1 &= ~I2C_C1_TX_MASK;
}

void i2c_set_slave_mode(I2C_MemMapPtr p)
{
    p->C1  &= ~I2C_C1_MST_MASK;
}
void i2c_set_master_mode(I2C_MemMapPtr p)
{
    p->C1  |=  I2C_C1_MST_MASK;
}

// i2c general

void i2c_give_nack(I2C_MemMapPtr p)
{
    p->C1 |= I2C_C1_TXAK_MASK;
}
void i2c_give_ack(I2C_MemMapPtr p)
{
    p->C1 &= ~I2C_C1_TXAK_MASK;
}
void i2c_repeated_start(I2C_MemMapPtr p)
{
    p->C1     |= 0x04;
}
void i2c_write_byte(I2C_MemMapPtr p, uint8_t data)
{
    p->D = data;
}
uint8_t i2c_read_byte(I2C_MemMapPtr p)
{
    return p->D;
}
void i2c_start(I2C_MemMapPtr p)
{
    i2c_set_master_mode(p);
    i2c_set_tx_mode(p);
}
void i2c_stop(I2C_MemMapPtr p)
{
    i2c_set_slave_mode(p);
    i2c_set_rx_mode(p);
}
void i2c_wait(I2C_MemMapPtr p)
{
    short loops = 500;

    // this is not what the example code did.
    while( (--loops) && !(p->S & I2C_S_IICIF_MASK) )
        ;
    p->S |= I2C_S_IICIF_MASK;
}
uint16_t i2c_get_ack(I2C_MemMapPtr p)
{
    return((p->S & I2C_S_RXAK_MASK) == 0);
}

// -------------------------------------------------
void hal_i2c_init(I2C_MemMapPtr p)
{
    // Enable clocks
    SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTE_MASK;      
    SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;
    SIM_SCGC4 |= SIM_SCGC4_I2C1_MASK;    

    // Configure GPIO for I2C0
    PORTE_PCR24 = PORT_PCR_MUX(5);
    PORTE_PCR25 = PORT_PCR_MUX(5);

    // Pinouts for EVK I2C1
    PORTE_PCR0 = PORT_PCR_MUX(6);
    PORTE_PCR1 = PORT_PCR_MUX(6);

    p->F  = 0x14;                   // Baudrate settings:  ICR=0x14, MULT=0
    p->C1 = I2C_C1_IICEN_MASK;      // Enable:  IICEN = 1<<7
}

void hal_i2c_deinit(I2C_MemMapPtr p)
{
    p->C1 = 0x00;
    SIM_SCGC4 &= ~SIM_SCGC4_I2C0_MASK;
    SIM_SCGC4 &= ~SIM_SCGC4_I2C1_MASK;
}

/*----------------------------------------------------------------------------
*        pause: delay for i2c
 *---------------------------------------------------------------------------*/
static void pause(void)
{
    int n;
    for(n=0; n<100; n++)
        asm("nop");
}

/*----------------------------------------------------------------------------
 *        hal_i2c_read
 *---------------------------------------------------------------------------*/
uint8_t hal_i2c_readX(I2C_MemMapPtr I2C_id, char dev_addr, char reg)
{
    uint8_t result;

    i2c_start(I2C_id);
    i2c_write_byte(I2C_id, dev_addr | I2C_WRITE);
    
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    i2c_write_byte(I2C_id, reg);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    i2c_repeated_start(I2C_id);
    i2c_write_byte(I2C_id, dev_addr | I2C_READ);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    i2c_set_rx_mode(I2C_id);

    i2c_give_nack(I2C_id);
    result = i2c_read_byte(I2C_id);
    i2c_wait(I2C_id);

    i2c_stop(I2C_id);
    result = i2c_read_byte(I2C_id);
    pause();
    return result;
}

/*----------------------------------------------------------------------------
 *        hal_i2c_read
 *---------------------------------------------------------------------------*/
uint8_t hal_i2c_read(I2C_MemMapPtr I2C_id, char dev_addr, char reg)
{
    uint8_t result;

    i2c_start(I2C_id);
    i2c_write_byte(I2C_id, dev_addr | I2C_WRITE);
    
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    i2c_write_byte(I2C_id, reg);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    i2c_repeated_start(I2C_id);
    i2c_write_byte(I2C_id, dev_addr | I2C_READ);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

   
    //Put in Rx Mode
    i2c_set_rx_mode(I2C_id);
    //Ensure TXAK bit is 0
    i2c_give_nack(I2C_id);
    //Dummy read
    result = i2c_read_byte(I2C_id);
    i2c_wait(I2C_id);

    i2c_stop(I2C_id);
    result = i2c_read_byte(I2C_id);
    pause();
    return result;
}

/*----------------------------------------------------------------------------
 *        hal_i2c_bulk_read
 *---------------------------------------------------------------------------*/
uint8_t hal_i2c_BulkRead(I2C_MemMapPtr I2C_id, char dev_addr, char StartReg, char Length, char *buf)
{
    uint8_t result, i;
    //Start
    i2c_start(I2C_id);
    //Send Slave Address for writing
    i2c_write_byte(I2C_id, dev_addr | I2C_WRITE);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);
    
    //Write Register Address
    i2c_write_byte(I2C_id, StartReg);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    //Repeated Start
    i2c_repeated_start(I2C_id);

    //Send Slave Address for reading
    i2c_write_byte(I2C_id, dev_addr | I2C_READ);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    //Put in Rx Mode
    i2c_set_rx_mode(I2C_id);
    //Ensure TXAK bit is 0
    i2c_give_ack(I2C_id);
    //Dummy read
    result = i2c_read_byte(I2C_id);
    i2c_wait(I2C_id);

    for(i=0; i<Length; i++){
      buf[i] = i2c_read_byte(I2C_id);
      i2c_wait(I2C_id);
    }
    
    //i2c_give_nack(I2C_id);
    i2c_stop(I2C_id);
    pause();
    return result;
}


/*----------------------------------------------------------------------------
 *        hal_i2c_write
 *---------------------------------------------------------------------------*/
void hal_i2c_write(I2C_MemMapPtr I2C_id, char dev_addr, char reg, char value)
{
    i2c_start(I2C_id);

    i2c_write_byte(I2C_id, dev_addr|I2C_WRITE);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    i2c_write_byte(I2C_id, reg);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    i2c_write_byte(I2C_id, value);
    i2c_wait(I2C_id);
    i2c_get_ack(I2C_id);

    i2c_stop(I2C_id);
    pause();
}
