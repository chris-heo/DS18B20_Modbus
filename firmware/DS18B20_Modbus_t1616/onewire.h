// Original from Peter Dannegger:
// https://www.mikrocontroller.net/topic/493064
// can be used without restrictions

#include <avr/io.h>
#include <stdint.h>

#define ONEWIRE_PARASITIC_POWER

void onewire_start(uint8_t delay);
void onewire_setbuf(uint8_t pos, uint8_t val);
uint8_t onewire_crc(uint8_t cnt);
#define onewire_busy()     TCA0.SINGLE.CTRLA          // 0 = one wire finished

typedef struct
{
    uint8_t state;
    uint8_t wr_cnt;
    uint8_t rd_cnt;
    uint8_t delay;        // * 100ms
    uint8_t* pbuf;
    uint8_t buf[9][8];
}   onewire_data_t;

extern onewire_data_t onewire_data;

#define ONWI_SKIP_ROM   0xCC
#define ONWI_READ_ROM   0x33
#define ONWI_CONVERT_T  0x44
#define ONWI_READ_T     0xBE

#define OW_RESET_SLOT   (uint32_t)(F_CPU / 64.0 * 480e-6 + 1.0)
#define OW_BIT_SLOT     (uint32_t)(F_CPU / 64.0 * 60e-6 + 1.0)

enum {
    OW_DELAY,
    OW_START = OW_DELAY + (uint32_t)(F_CPU / 64.0 / OW_RESET_SLOT * 100e-3),
    OW_RES_LO,
    OW_RES_HI,
    OW_WR_BIT0,
    OW_WR_BIT1,
    OW_WR_BIT2,
    OW_WR_BIT3,
    OW_WR_BIT4,
    OW_WR_BIT5,
    OW_WR_BIT6,
    OW_WR_BIT7,
    OW_WR_DONE,
    OW_RD_BIT0,
    OW_RD_BIT1,
    OW_RD_BIT2,
    OW_RD_BIT3,
    OW_RD_BIT4,
    OW_RD_BIT5,
    OW_RD_BIT6,
    OW_RD_BIT7,
    OW_RD_DONE,
};
