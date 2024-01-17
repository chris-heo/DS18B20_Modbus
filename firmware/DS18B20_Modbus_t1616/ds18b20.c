#include "ds18b20.h"
#include "onewire.h"
#include <avr/io.h>

ds18b20_state_t ds18b20_state = ds18b20_state_idle;

void ds18b20_measure(void)
{
    onewire_setbuf(0, ONWI_SKIP_ROM);
    onewire_setbuf(1, ONWI_CONVERT_T);
    onewire_data.wr_cnt = 2;
    onewire_data.rd_cnt = 0;
    onewire_start(0);
}

void ds18b20_read_temp(uint8_t delay)
{
    onewire_setbuf(0, ONWI_SKIP_ROM);
    onewire_setbuf(1, ONWI_READ_T);
    onewire_data.wr_cnt = 2;
    onewire_data.rd_cnt = 9;
    onewire_start(delay);
}

void ds18b20_read_rom(void)
{
    onewire_setbuf(0, ONWI_READ_ROM);
    onewire_data.wr_cnt = 1;
    onewire_data.rd_cnt = 8;
    onewire_data.state = OW_START;
    onewire_start(0);
}

bool ds18b20_tick(void)
{
    if(onewire_busy())
        return false;

    switch(ds18b20_state)
    {
        case ds18b20_state_idle: break;
        case ds18b20_state_done: break;

        case ds18b20_state_start:
            ds18b20_measure();
            ds18b20_state = ds18b20_state_meas;
            break;
        case ds18b20_state_meas:
            ds18b20_read_temp(10);
            ds18b20_state = ds18b20_state_readt;
            break;
        case ds18b20_state_readt:
            ds18b20_read_temp(0);
            ds18b20_state = ds18b20_state_readr;
            break;
        case ds18b20_state_readr:
            ds18b20_state = ds18b20_state_done;
            return true;
    }
    
    return false;
}

bool ds18b20_start(void)
{
    if(ds18b20_state == ds18b20_state_idle || ds18b20_state == ds18b20_state_done)
    {
        ds18b20_state = ds18b20_state_start;
        return true;
    }
    return false;
}

bool ds18b20_isdone(void)
{
    return ds18b20_state == ds18b20_state_done;
}

/**
 * \brief returns the temperature as fixed point in centi (1/100) degree Celsius
 * 
 * \return int16_t temperature
 */
int16_t ds18b20_get_temp_fixed(uint8_t index)
{
    int32_t temp = 0;
    uint8_t mask = 1 << index;
    for(uint8_t i = 0; i < 8; i++)
    {
        temp >>= 1;
        if(onewire_data.buf[0][i] & mask)
        {
            temp |= 0x0080;
        }
        if(onewire_data.buf[1][i] & mask)
        {
            temp |= 0x8000;
        }
    }
    // get the sign
    if(temp & 0x00000800)
        temp |= 0xFFFFF000;
    
    temp *= 625;
    // proper rounding of the value
    if(temp >= 0)
    {
        temp += 50;
    }
    else
    {
        temp -= 50;
    }
    return temp / 100;
}
