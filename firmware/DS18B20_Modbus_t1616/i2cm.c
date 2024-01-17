#include "i2cm.h"
#include <util/delay.h>

#define TWI0_BAUD(F_SCL)      (uint8_t)(((((float)F_CPU / (float)F_SCL)) - 10 ) / 2)

void i2cm_init(void)
{
    TWI0.MBAUD = (uint8_t)TWI0_BAUD(100000); /* set MBAUD register */

    TWI0.MCTRLA = 0
        | 0 << TWI_RIEN_bp        /* Read Interrupt Enable: disabled */
        | 0 << TWI_WIEN_bp        /* Write Interrupt Enable: disabled */
        | 0 << TWI_QCEN_bp        /* Quick Command Enable: disabled */
        | 1 << TWI_SMEN_bp        /* Smart Mode Enable: enable */
        | TWI_TIMEOUT_DISABLED_gc /* Bus Timeout Disabled */
        | 1 << TWI_ENABLE_bp;     /* Enable TWI Master: enabled */

    TWI0.MCTRLB |= TWI_FLUSH_bm ; // Purge MADDR and MDATA

    TWI0.SCTRLA = 0 << TWI_ENABLE_bp; // disable I2C slave

    TWI0.MSTATUS |= TWI_BUSSTATE_IDLE_gc ; // Force TWI state machine into IDLE state
    TWI0.MSTATUS |= (TWI_RIF_bm | TWI_WIF_bm) ;
}

void i2cm_disable(void)
{
    TWI0.MCTRLA = 0;
}

i2c_status_e i2cm_start(uint8_t address)
{
    uint8_t timeout = I2C_TIMEOUT;
    while ((TWI0.MSTATUS & TWI_BUSSTATE_gm) == TWI_BUSSTATE_BUSY_gc)
    {
        _delay_us(I2C_WAITTIME);
        timeout--;
        if(timeout == 0)
            return i2c_status_busy;
    }

    TWI0.MADDR = address;

    timeout = I2C_TIMEOUT;
    while(!(TWI0.MSTATUS & (TWI_WIF_bm | TWI_RIF_bm)))
    {
        _delay_us(I2C_WAITTIME);
        timeout--;
        if(timeout == 0)
            return i2c_status_busy;
    }

    if(TWI0.MSTATUS & TWI_RXACK_bm)
    {
        return i2c_status_notfound;
    }

    return i2c_status_ok;
}


i2c_status_e i2cm_read(uint8_t* data, bool nack)
{
    if((TWI0.MSTATUS & TWI_BUSSTATE_gm) != TWI_BUSSTATE_OWNER_gc)
    {
        return i2c_status_noowner;
    }

    uint8_t timeout = I2C_TIMEOUT;
    while(!(TWI0.MSTATUS & TWI_RIF_bm))
    {
        _delay_us(I2C_WAITTIME);
        timeout--;
        if(timeout == 0)
        {
            return i2c_status_busy;
        }
    }

    if(TWI0.MSTATUS & TWI_BUSERR_bm)
        return i2c_status_buserror;

    *data = TWI0.MDATA;

    if(nack)
    {
        TWI0.MCTRLB = TWI_ACKACT_bm;
    }
    else
    {
        TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
    }

    return i2c_status_ok;
}

i2c_status_e i2cm_write(uint8_t data)
{
    if((TWI0.MSTATUS & TWI_BUSSTATE_gm) != TWI_BUSSTATE_OWNER_gc)
    {
        return i2c_status_noowner;
    }

    TWI0.MDATA = data;

    while(!(TWI0.MSTATUS & TWI_WIF_bm));

    if(TWI0.MSTATUS & TWI_BUSERR_bm)
        return i2c_status_buserror;

    if(TWI0.MSTATUS & TWI_RXACK_bm)
        return i2c_status_notfound;

    return i2c_status_ok;
}

i2c_status_e i2cm_stop(void)
{
    uint8_t timeout = I2C_TIMEOUT;
    while((TWI0.MSTATUS & TWI_CLKHOLD_bm) == 0)
    {
        _delay_us(I2C_WAITTIME);
        timeout--;
        if(timeout == 0)
            return i2c_status_busy;
    }

    TWI0.MCTRLB |= TWI_MCMD_STOP_gc;

    return i2c_status_ok;
}

i2c_status_e i2cm_readregs(uint8_t address, uint8_t reg, uint8_t* data, uint8_t size)
{
    i2c_status_e status;

    status = i2cm_start((address << 1) | 0);
    if(status != i2c_status_ok) goto i2c_fail;

    status = i2cm_write(reg);
    if(status != i2c_status_ok) goto i2c_fail;

    status = i2cm_start((address << 1) | 1);
    if(status != i2c_status_ok) goto i2c_fail;

    for(uint8_t i = 0; i < size; i++)
    {
        bool nack = i == (size -1);
        status = i2cm_read(data, nack);
        if(status != i2c_status_ok) goto i2c_fail;
        data++;
    }


i2c_fail:
    i2cm_stop();
    return status;
}

i2c_status_e i2cm_readreg8(uint8_t dev_addr, uint8_t reg_addr, uint8_t* data)
{
    i2c_status_e status;
    status = i2cm_start_w(dev_addr);
    if(status != i2c_status_ok) goto i2c_fail;

    status = i2cm_write(reg_addr);
    if(status != i2c_status_ok) goto i2c_fail;

    status = i2cm_read(data, true);

i2c_fail:
    i2cm_stop();
    return status;
}

i2c_status_e i2cm_readreg16be(uint8_t dev_addr, uint8_t reg_addr, uint16_t* data)
{
    i2c_status_e status;
    status = i2cm_start_w(dev_addr);
    if(status != i2c_status_ok) goto i2c_fail;

    status = i2cm_write(reg_addr);
    if(status != i2c_status_ok) goto i2c_fail;

    uint8_t tmp = 0;

    status = i2cm_read(&tmp, true);
    *data = tmp << 8;
    if(status != i2c_status_ok) goto i2c_fail;
    
    status = i2cm_read(&tmp, false);
    *data |= tmp;

    i2c_fail:
    i2cm_stop();
    return status;
}

i2c_status_e i2cm_writeregs(uint8_t address, uint8_t reg, uint8_t* data, uint8_t size)
{
    i2c_status_e status;

    status = i2cm_start((address << 1) | 0);
    if(status != i2c_status_ok) goto i2c_fail;

    status = i2cm_write(reg);
    if(status != i2c_status_ok) goto i2c_fail;

    for(uint8_t i = 0; i < size; i++)
    {
        status = i2cm_write(*data++);
        if(status != i2c_status_ok) goto i2c_fail;
    }

i2c_fail:
    i2cm_stop();
    return status;
}

i2c_status_e i2cm_writereg8(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    i2c_status_e status;
    status = i2cm_start_w(dev_addr);
    if(status != i2c_status_ok) goto i2c_fail;

    status = i2cm_write(reg_addr);
    if(status != i2c_status_ok) goto i2c_fail;

    status = i2cm_write(data);

    i2c_fail:
    i2cm_stop();
    return status;
}
