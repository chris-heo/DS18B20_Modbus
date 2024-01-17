// Original from Peter Dannegger:
// https://www.mikrocontroller.net/topic/493064
// can be used without restrictions

#include <util/crc16.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "onewire.h"
#include "config.h"

onewire_data_t onewire_data;

#define ONEWIRE_TIMER_OVF_VECT TCA0_OVF_vect
#define ONEWIRE_TIMER_TCNT_MAX 0xFFFF

#define ONEWIRE_PORT PORTA
#define ONEWIRE_PINMASK DS18B20_MAP

//////////////////////////////////////////////////////////////////////////
// GPIO primitives

#define onewire_io_islow() ONEWIRE_PORT.DIR && !ONEWIRE_PORT.OUT

#define onewire_io_read() ONEWIRE_PORT.IN

inline void onewire_io_outlow()
{
    // pin = low
    ONEWIRE_PORT.OUTCLR = ONEWIRE_PINMASK;
    ONEWIRE_PORT.DIRSET = ONEWIRE_PINMASK;
}

inline void onewire_io_tristate()
{
    // pin = tristate
    ONEWIRE_PORT.DIRCLR = ONEWIRE_PINMASK;
}

inline void onewire_io_outhigh()
{
    // parasite power on
    ONEWIRE_PORT.OUTSET = ONEWIRE_PINMASK;
    ONEWIRE_PORT.DIRSET = ONEWIRE_PINMASK;
}

inline void onewire_io_inlow()
{
    // parasite power off
    ONEWIRE_PORT.DIRCLR = ONEWIRE_PINMASK;
    ONEWIRE_PORT.OUTCLR = ONEWIRE_PINMASK;
}

inline void onewire_io_out()
{
    // pin = low
    ONEWIRE_PORT.DIRSET = ONEWIRE_PINMASK;
}

//////////////////////////////////////////////////////////////////////////
// Timer

inline void onewire_timer_set(uint8_t time)
{
    TCA0.SINGLE.CNT = (ONEWIRE_TIMER_TCNT_MAX + 1) - time;
}

inline void onewire_timer_start(void)
{
    TCA0.SINGLE.CTRLB = 0;                   // Mode 0
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV64_gc | (1 << TCA_SINGLE_ENABLE_bp);   // F_CPU / 64
    TCA0.SINGLE.INTCTRL = (1 << TCA_SINGLE_OVF_bp);
}

inline void onewire_timer_stop(void)
{
    TCA0.SINGLE.CTRLA = 0;
}

void onewire_start(uint8_t delay)
{
    onewire_data.delay = delay;
    onewire_data.state = OW_START;
    onewire_data.pbuf = &onewire_data.buf[0][0];
    
    onewire_timer_set(OW_BIT_SLOT);
    onewire_timer_start();
}

//////////////////////////////////////////////////////////////////////////
// buffer handling

void onewire_setbuf(uint8_t pos, uint8_t val)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        if(val & 1)
        {
            onewire_data.buf[pos][i] = 0xFF;
        }
        else
        {
            onewire_data.buf[pos][i] = 0x00;
        }
        val >>= 1;
    }
}

void onewire_get_data(uint8_t index, uint8_t* ptr, uint8_t length)
{
    uint8_t mask = 1 << index;
    for(uint8_t b = 0; b < length; b++)
    for(uint8_t i = 0; i < 8; i++)
    {
        ptr[b] >>= 1;
        if(onewire_data.buf[b][i] & mask)
        ptr[b] |= 0x80;
    }
}

uint8_t onewire_get_byte(uint8_t slot, uint8_t index)
{
    uint8_t result = 0;
    uint8_t mask = 1 << index;
    for(uint8_t i = 0; i < 8; i++)
    {
        result >>= 1;
        if(onewire_data.buf[index][i] & mask)
        {
            result |= 0x80;
        }
    }
    return result;
}

uint8_t onewire_crc(uint8_t cnt)
{
    uint8_t crc = 0;
    uint8_t* pbuf = onewire_data.buf[0];
    while(cnt--)
    {
        crc = _crc_ibutton_update(crc, *pbuf++);
    }
    return crc;
}

//////////////////////////////////////////////////////////////////////////
// communication

ISR(ONEWIRE_TIMER_OVF_VECT)
{
    //flag is not cleared automatically
    TCA0.SINGLE.INTFLAGS = (1<<TCA_SINGLE_OVF_bp);

    if (onewire_io_islow())             // if low
    {
        onewire_io_tristate();
        _delay_us(5);                       // min high time
    }
    uint8_t* pbuf = onewire_data.pbuf;            // for faster access
    switch (++onewire_data.state)
    {
        case OW_RES_LO:                     // 480us low
            if (onewire_data.delay--)                 // n * 100ms
            {
                onewire_data.state = OW_DELAY;
            }
            else
            {
                onewire_io_outlow();
            }
        case OW_RES_HI:                     // 480us high
        default:                            // n * 480µs
            onewire_timer_set(OW_RESET_SLOT);
            return;
        case OW_WR_DONE:                    // 1. command byte finished
            onewire_data.state = OW_WR_BIT0;
        case OW_WR_BIT0:                    // write bit 0
            if (onewire_data.wr_cnt--)
            {
        case OW_WR_BIT1:                    // write bit 1
        case OW_WR_BIT2:
        case OW_WR_BIT3:
        case OW_WR_BIT4:
        case OW_WR_BIT5:
        case OW_WR_BIT6:
        case OW_WR_BIT7:                    // write bit 7
                onewire_io_out();
                _delay_us(2);
                if (*pbuf > 0)
                {
                    onewire_io_tristate();
                }
                pbuf++;
                break;
            }
#ifdef ONEWIRE_PARASITIC_POWER
            onewire_io_outhigh();
#endif
            pbuf = &onewire_data.buf[0][0];
            case OW_RD_DONE:
                onewire_data.state = OW_RD_BIT0;
            if (onewire_data.rd_cnt--)
            {
#ifdef ONEWIRE_PARASITIC_POWER
                //not sure if it is correct to power it off here
                onewire_io_inlow();
#endif
        case OW_RD_BIT1:                    // read bit 1
        case OW_RD_BIT2:
        case OW_RD_BIT3:
        case OW_RD_BIT4:
        case OW_RD_BIT5:
        case OW_RD_BIT6:
        case OW_RD_BIT7:                    // read bit 7
                onewire_io_out();
                _delay_us(1);
                onewire_io_tristate();
                _delay_us(5);
                *pbuf = onewire_io_read();
                pbuf++;
                
                break;
            }
            onewire_timer_stop();
            return;
    }
    onewire_data.pbuf = pbuf;
    onewire_timer_set(OW_BIT_SLOT);
}
