#ifndef CONFIG_H_
#define CONFIG_H_
#include "as5600.h"
#include "config_helper.h"

//////////////////////////////////////////////////////////////////////////
// Input usage

// currently, only DS18B20 can be set, 
// pulse counter (negative pulses) is active for all channels
// ADC is only active for AIN

#define S1_USAGE USAGE_DS18B20
#define S2_USAGE USAGE_DS18B20
#define S3_USAGE USAGE_DS18B20
#define S4_USAGE USAGE_DS18B20
#define S5_USAGE USAGE_DS18B20
#define S6_USAGE USAGE_DS18B20
#define S7_USAGE USAGE_DS18B20

//////////////////////////////////////////////////////////////////////////
// Modbus

#define MODBUS_BAUDRATE 19200L

#ifndef MODBUS_ADDRESS
#define MODBUS_ADDRESS 10
#endif

// activity LED blink duration in 100 us steps (up to 255)
#define MODBUS_ACT_DURATION 200

// this is a bit of a cludge, register counts should be determined automagically
#define MODBUS_REGS_INPUT_CNT 21
#define MODBUS_REGS_HOLDING_CNT 1

//////////////////////////////////////////////////////////////////////////
// Temperature sensor settings

// Modbus register offsets, 8 input registers are occupied (7 sensors + acquisition counter)
#define MODBUS_REGOFFS_TEMP 0

//////////////////////////////////////////////////////////////////////////
// ADC settings

//#define ADC0_REF VREF_INT_0V55
//#define ADC0_REF VREF_INT_1V1
//#define ADC0_REF VREF_INT_1V5
//#define ADC0_REF VREF_INT_2V5
//#define ADC0_REF VREF_INT_4V34
#define ADC0_REF VREF_VDD

// accumulation for ADC inputs, please note that the values actually get accumulated (no averaging)
#define ADC0_ACCUMULATE ADC_SAMPNUM_ACC8_gc

#define ADC_SCANCHANNELS ADC_MUXPOS_AIN8_gc,

// modbus register offset (depending on adc_scanchannels)
#define MODBUS_REGOFFS_ADC 11

//////////////////////////////////////////////////////////////////////////
// Rotary Encoder settings

// rotary encoder update interval in 100 us steps (up to 65535)
#define ROTENC_UPDATE_INTERVAL 1000

#define ROTENC_POWERMODE as5600_powermode_lpm1
#define ROTENC_HYSTERESIS as5600_hysteresis_off
#define ROTENC_SLOWFILTER as5600_slowfilter_16x
#define ROTENC_FASTFILTER as5600_fastfilterthrs_off
#define ROTENC_WATCHDOG false

// modbus register offset
#define MODBUS_REGOFFS_ROTENC 8
#define MODBUS_REGOFFS_ROTENC_STATUS (MODBUS_REGOFFS_ROTENC+0)
#define MODBUS_REGOFFS_ROTENC_ANGLE  (MODBUS_REGOFFS_ROTENC+1)
#define MODBUS_REGOFFS_ROTENC_MAGNITUDE (MODBUS_REGOFFS_ROTENC+2)


//////////////////////////////////////////////////////////////////////////
// Pulse counter

// pulse counter tick interval in 100 us steps, the qualification periods are based on this (up to 255)
#define PULSECOUNTER_TICK_INTERVAL 5

// threshold of ticks after which the counter shall be incremented, must be less than 255
// PULSECOUNTER_H_THRESHOLD can currently be ignored, only low pulses are counted
#define PULSECOUNTER_H_THRESHOLD 20
#define PULSECOUNTER_L_THRESHOLD 20

// modbus register offset
#define MODBUS_REGOFFS_PULSECOUNTER 12

#endif /* CONFIG_H_ */
