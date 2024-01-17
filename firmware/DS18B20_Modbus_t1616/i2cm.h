/*
 * i2cm.h
 *
 * Created: 05.04.2020 20:25:46
 *  Author: Chris
 */ 


#ifndef I2CM_H_
#define I2CM_H_

#include <avr/io.h>
#include <stdbool.h>

// wait time in one wait cycle (in us)
#define I2C_WAITTIME 10
// maximum wait cycles (see above) before a timeout occurs
#define I2C_TIMEOUT 100

#define I2C_ADDR_R(addr) ((addr << 1) | 1)
#define I2C_ADDR_W(addr) ((addr << 1) | 0)

typedef enum
{
    i2c_status_ok,
    i2c_status_busy,
    i2c_status_notfound,
    i2c_status_noowner,
    i2c_status_buserror,
}   i2c_status_e;

void i2cm_init(void);
void i2cm_disable(void);

i2c_status_e i2cm_start(uint8_t address);

inline i2c_status_e i2cm_start_w(uint8_t address)
{
    return i2cm_start(I2C_ADDR_W(address));
}

inline i2c_status_e i2cm_start_r(uint8_t address)
{
    return i2cm_start(I2C_ADDR_R(address));
}

i2c_status_e i2cm_read(uint8_t* data, bool nack);
i2c_status_e i2cm_write(uint8_t data);
i2c_status_e i2cm_stop(void);
i2c_status_e i2cm_readregs(uint8_t address, uint8_t reg, uint8_t* data, uint8_t size);
i2c_status_e i2cm_readreg8(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data);
i2c_status_e i2cm_readreg16be(uint8_t dev_addr, uint8_t reg_addr, uint16_t* data);
i2c_status_e i2cm_writeregs(uint8_t address, uint8_t reg, uint8_t* data, uint8_t size);
i2c_status_e i2cm_writereg8(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
#endif /* I2CM_H_ */
