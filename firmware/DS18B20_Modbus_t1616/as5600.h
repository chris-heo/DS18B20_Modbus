#ifndef AS5600_H_
#define AS5600_H_

#include <stdbool.h>

#define AS5600_ADDR 0x36


typedef enum
{
    as5600_register_config0 = 0x07,
    as5600_register_config1 = 0x08,
    as5600_register_status = 0x0B,
    as5600_register_raw_angle = 0x0C,
    as5600_register_angle = 0x0E,
    as5600_register_agc = 0x1A,
    as5600_register_magnitude = 0x1B,
}   as5600_register_e;

typedef enum
{
    as5600_powermode_nom = 0x00,
    as5600_powermode_lpm1 = 0x01, // polling time 5ms
    as5600_powermode_lpm2 = 0x02, // polling time 20ms
    as5600_powermode_lpm3 = 0x03, // polling time 100ms
}   as5600_powermode_e;

typedef enum
{
    as5600_hysteresis_off = 0x00,
    as5600_hysteresis_1lsb = 0x01,
    as5600_hysteresis_2lsb = 0x02,
    as5600_hysteresis_3lsb = 0x03,
}   as5600_hysteresis_e;

typedef enum
{
    as5600_outputstage_analog = 0x00,
    as5600_outputstage_reducedanalog = 0x01,
    as5600_outputstage_pwm = 0x02,
}   as5600_outputstage_e;

typedef enum
{
    as5600_pwmfreq_115Hz = 0x00,
    as5600_pwmfreq_230Hz = 0x01,
    as5600_pwmfreq_460Hz = 0x02,
    as5600_pwmfreq_920Hz = 0x03,
}   as5600_pwmfreq_e;

typedef enum
{
    as5600_slowfilter_16x = 0x00,
    as5600_slowfilter_8x = 0x01,
    as5600_slowfilter_4x = 0x02,
    as5600_slowfilter_2x = 0x03,
}   as5600_slowfilter_e;

typedef enum
{
    as5600_fastfilterthrs_off = 0x00, //000 = slow filter only
    as5600_fastfilterthrs_6lsb = 0x01, //001 = 6 LSBs
    as5600_fastfilterthrs_7lsb = 0x02, //010 = 7 LSBs
    as5600_fastfilterthrs_9lsb = 0x03, //011 = 9 LSBs
    as5600_fastfilterthrs_18lsb = 0x04, //100 = 18 LSBs
    as5600_fastfilterthrs_21lsb = 0x05, //101 = 21 LSBs
    as5600_fastfilterthrs_24lsb = 0x06, //110 = 24 LSBs
    as5600_fastfilterthrs_10lsb = 0x07, //111 = 10 LSBs
}   as5600_fastfilterthrs_e;

bool as5600_config( as5600_powermode_e powermode, as5600_hysteresis_e hysteresis, as5600_outputstage_e outputstage, as5600_pwmfreq_e pwmfreq, as5600_slowfilter_e slowfilter, as5600_fastfilterthrs_e fastfilterthrs, bool watchdog );

#endif /* AS5600_H_ */
