#ifndef DS18B20_H_
#define DS18B20_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ds18b20_state_idle = 0,
    ds18b20_state_start = 1,
    ds18b20_state_meas = 2,
    ds18b20_state_readt = 3,
    ds18b20_state_readr = 4,
    ds18b20_state_done = 5,

}   ds18b20_state_t;

bool ds18b20_tick(void);
bool ds18b20_start(void);
bool ds18b20_isdone(void);
int16_t ds18b20_get_temp_fixed(uint8_t index);

#endif /* DS18B20_H_ */
