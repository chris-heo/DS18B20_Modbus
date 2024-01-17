#include <avr/io.h>
#include "onewire.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "yaMBSiavr.h"
#include "ds18b20.h"
#include "i2cm.h"
#include "as5600.h"
#include <avr/pgmspace.h>
#include "config.h"
#include <avr/wdt.h>

FUSE_t __fuse __attribute__((section (".fuse"))) =
{
    .WDTCFG = WINDOW_OFF_gc | PERIOD_16CLK_gc,
    .BODCFG = BOD_LVL_BODLEVEL2_gc | BOD_SAMPFREQ_1KHZ_gc | BOD_ACTIVE_ENABLED_gc | BOD_SLEEP_DIS_gc,
    .OSCCFG = (0 << FUSE_OSCLOCK_bp) | FREQSEL_16MHZ_gc,
    .TCD0CFG = (0 << FUSE_CMPDEN_bp) | (0 << FUSE_CMPCEN_bp) | (0 << FUSE_CMPBEN_bp) | (0 << FUSE_CMPAEN_bp) | (0 << FUSE_CMPD_bp) | (0 << FUSE_CMPC_bp) | (0 << FUSE_CMPB_bp) | (0 << FUSE_CMPA_bp),
    .SYSCFG0 = CRCSRC_NOCRC_gc | RSTPINCFG_UPDI_gc | (1 << FUSE_EESAVE_bp),
    .SYSCFG1 = CLKCTRL_CSUT_64K_gc,
    .APPEND = 0x00,
    .BOOTEND = 0x00,
};

volatile uint8_t sys_ticks = 0;

#define led_red_on() PORTC.OUTSET = PIN1_bm
#define led_red_off() PORTC.OUTCLR = PIN1_bm

#define led_green_on() PORTC.OUTSET = PIN0_bm
#define led_green_off() PORTC.OUTCLR = PIN0_bm

uint8_t modbus_act_cnt = 0;

const __flash uint8_t adc_scanchannels[] =
{
    ADC_SCANCHANNELS
};

//////////////////////////////////////////////////////////////////////////
// helper functions

#define array_length(arr) (sizeof(arr) / sizeof(typeof(*arr)))

inline bool postscaler8(uint8_t ticks_passed, uint8_t* postscaler, uint8_t scaling)
{
    *postscaler += ticks_passed;
    if(*postscaler >= scaling)
    {
        *postscaler -= scaling;
        return true;
    }
    return false;
}

inline bool postscaler16(uint8_t ticks_passed, uint16_t* postscaler, uint16_t scaling)
{
    *postscaler += ticks_passed;
    if(*postscaler >= scaling)
    {
        *postscaler -= scaling;
        return true;
    }
    return false;
}

//////////////////////////////////////////////////////////////////////////
// system ticks

void systick_init(void)
{
    TCB0.CTRLA = TCB_CLKSEL_CLKDIV1_gc | (1 << TCB_ENABLE_bp);
    //TCB0.CTRLB = TCB_CNTMODE_0_bm;
    TCB0.INTCTRL = 1 << TCB_CAPT_bp;
    TCB0.CCMP = 1600; // 100µs @ 16 MHz
}

ISR(TCB0_INT_vect)
{
    TCB0.INTFLAGS = 1 << TCB_CAPT_bp;
    modbusTickTimer();
    sys_ticks++;
}

//////////////////////////////////////////////////////////////////////////
// modbus stuff

volatile uint8_t instate = 0;
volatile uint8_t outstate = 0;
volatile uint16_t inputRegisters[MODBUS_REGS_INPUT_CNT];
volatile uint16_t holdingRegisters[MODBUS_REGS_HOLDING_CNT];

void modbusGet(void)
{

    if (modbusGetBusState() & (1<<ReceiveCompleted))
    {
        modbus_act_cnt = MODBUS_ACT_DURATION;
        led_green_off();
        
        switch(rxbuffer[1]) 
        {
            case fcReadCoilStatus: //read single/multiple coils
            {
                modbusExchangeBits(&outstate, 0, 8);
            }
            break;
            
            case fcReadInputStatus: //read single/multiple inputs
            {
                volatile uint8_t inps = 0; //ReadIns();
                modbusExchangeBits(&inps, 0, 8);
            }
            break;
            
            case fcReadHoldingRegisters: //read analog output registers
            {
                modbusExchangeRegisters(holdingRegisters, 0, MODBUS_REGS_HOLDING_CNT);
            }
            break;
            
            case fcReadInputRegisters: //read analog input registers (2 Bytes per register)
            {
                modbusExchangeRegisters(inputRegisters, 0, MODBUS_REGS_INPUT_CNT);
            }
            break;
            
            case fcForceSingleCoil: //write single bit
            {
                modbusExchangeBits(&outstate, 0, 8);
            }
            break;
            
            case fcPresetSingleRegister: //write analog output register (2 Bytes)
            {
                modbusExchangeRegisters(holdingRegisters, 0, MODBUS_REGS_HOLDING_CNT);
            }
            break;
            
            case fcForceMultipleCoils: //write multiple bits
            {
                modbusExchangeBits(&outstate, 0, 8);
            }
            break;
            
            case fcPresetMultipleRegisters: //write multiple analog output registers (2 Bytes each)
            {
                modbusExchangeRegisters(holdingRegisters, 0, 4);
            }
            break;

            case fcEncapsulatedInterfaceTransport:
            {
                modbusHandleDeviceId();
            }
            break;
            
            default: {
                modbusSendException(ecIllegalFunction);
            }
            break;
        }
    }
}

//////////////////////////////////////////////////////////////////////////
// temperatur sensor stuff

typedef enum
{
    tempsensors_idle = 0,
    tempsensors_start = 1,
    tempsensors_ongoing = 2,
    tempsensors_done = 3,
}   tempsensors_e;


inline void tempsensors_invalidate()
{
    for(uint8_t i = 0; i < 7; i++)
    {
        inputRegisters[i] = 0x8000;
    }
}

void tempsensors_tick()
{
    if(((holdingRegisters[0] >> 0) & 0x03) == tempsensors_start)
    {
        // invalidate input registers
        tempsensors_invalidate();

        // wait until modbus transfer is complete, then start conversion
        if((modbusGetBusState() & (1<<Transmitting)) == 0 && ds18b20_start() == true)
        {
            holdingRegisters[0] = (holdingRegisters[0] & ~(0x03)) | ((uint8_t)tempsensors_ongoing << 0);
            led_red_on(); // turn status LED on
        }
    }

    if(ds18b20_tick() == true)
    {
        for(uint8_t i = 0; i < 7; i++)
        {
            int16_t temp = ds18b20_get_temp_fixed(i+1);
            inputRegisters[MODBUS_REGOFFS_TEMP + i] = temp;
        }
        inputRegisters[MODBUS_REGOFFS_TEMP + 7]++; // increase read count
        
        holdingRegisters[0] = (holdingRegisters[0] & ~(0x03)) | ((uint8_t)tempsensors_done << 0);
        led_red_off();
    }
}

//////////////////////////////////////////////////////////////////////////
// rotary encoder stuff

uint8_t rotenc_errors = 1;

void rotenc_tick()
{

    if(rotenc_errors > 5)
    {
        // if there are multiple errors, re-initialize I2C
        i2cm_disable();
        i2cm_init();
    }

    if(rotenc_errors > 0)
    {
        // if there was an error, re-initialize the rotary encoder
        if(as5600_config(
            ROTENC_POWERMODE, ROTENC_HYSTERESIS, 
            as5600_outputstage_analog, as5600_pwmfreq_115Hz, 
            ROTENC_SLOWFILTER, ROTENC_FASTFILTER, ROTENC_WATCHDOG) == false)
        {
            goto rotenc_error;
        }
    }

    uint8_t data[3];
    if(i2cm_readregs(AS5600_ADDR, as5600_register_status, data, sizeof(data)) != i2c_status_ok)
    {
        goto rotenc_error;
    }

    uint8_t status = (data[0] >> 3) & 0x07;
    inputRegisters[MODBUS_REGOFFS_ROTENC_ANGLE] = (data[1] << 8) | data[2]; // angle_raw

    if(i2cm_readregs(AS5600_ADDR, as5600_register_agc, data, sizeof(data)) != i2c_status_ok)
    {
        goto rotenc_error;
    }

    inputRegisters[MODBUS_REGOFFS_ROTENC_STATUS] = (status << 8) | data[0]; // status & gain
    inputRegisters[MODBUS_REGOFFS_ROTENC_MAGNITUDE] = (data[1] << 8) | data[2]; // magnitude

    rotenc_errors = 0;

    return;

rotenc_error:
    rotenc_errors++;

    inputRegisters[MODBUS_REGOFFS_ROTENC_STATUS] = 0x08 << 8;
    inputRegisters[MODBUS_REGOFFS_ROTENC_ANGLE] = 0;
    inputRegisters[MODBUS_REGOFFS_ROTENC_MAGNITUDE] = 0;
}

//////////////////////////////////////////////////////////////////////////
// adc stuff

uint16_t adc_values[array_length(adc_scanchannels)];
uint8_t adc_currchan = 0;

inline void adc_startconv()
{
    ADC0.COMMAND = ADC_STCONV_bm;
}

void adc0_init()
{
#if (ADC0_REF == VREF_INT_0V55)
    VREF.CTRLA = VREF_ADC0REFSEL_0V55_gc;
#elif (ADC0_REF == VREF_INT_1V1)
    VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;
#elif ADC0_REF == VREF_INT_1V5
    VREF.CTRLA = VREF_ADC0REFSEL_1V5_gc;
#elif ADC0_REF == VREF_INT_2V5
    VREF.CTRLA = VREF_ADC0REFSEL_2V5_gc;
#elif ADC0_REF == VREF_INT_4V34
    VREF.CTRLA = VREF_ADC0REFSEL_4V34_gc;
#endif

    VREF.CTRLB = 
#if ADC0_REF != VREF_VDD
    (1<<VREF_ADC0REFEN_bp) | 
#endif
    (0<<VREF_DAC0REFEN_bp);

    ADC0.CTRLB = ADC0_ACCUMULATE;
    //ADC0.CTRLC = (1<<ADC_SAMPCAP_bp) | ADC_REFSEL_INTREF_gc | ADC_PRESC_DIV64_gc;
    ADC0.CTRLC = (1<<ADC_SAMPCAP_bp) | 
#if ADC0_REF == VREF_VDD
    ADC_REFSEL_VDDREF_gc
    //#pragma message ( "VDD used as reference" )
#else
    //#pragma message ( "Internal Ref used as reference" )
    ADC_REFSEL_INTREF_gc
#endif
     | ADC_PRESC_DIV64_gc;
    ADC0.CTRLD =  ADC_INITDLY_DLY16_gc | ADC_ASDV_ASVON_gc | 0;
    ADC0.SAMPCTRL = 0;
    ADC0.CTRLE = ADC_WINCM_NONE_gc;

    ADC0.CTRLA = (1<<ADC_ENABLE_bp) | (0<<ADC_FREERUN_bp) | ADC_RESSEL_10BIT_gc | (0<<ADC_RUNSTBY_bp);

    adc_currchan = 0;
    ADC0.MUXPOS = adc_scanchannels[adc_currchan];
    adc_startconv();
}

uint8_t adc0_tick()
{
    if(ADC0.INTFLAGS & (1<<ADC_RESRDY_bp))
    {
        uint8_t updated_channel = adc_currchan;
        adc_values[adc_currchan] = ADC0.RES;
        ADC0.INTFLAGS |= (1<<ADC_RESRDY_bp);

        adc_currchan++;
        if(adc_currchan >= array_length(adc_scanchannels))
        adc_currchan = 0;

        ADC0.MUXPOS = adc_scanchannels[adc_currchan];
        adc_startconv();

        return updated_channel;
    }
    return 0xFF; // no update
}

void adc_tick()
{
    uint8_t adc_updch = adc0_tick();
    if(adc_updch != 0xFF)
    {
        inputRegisters[MODBUS_REGOFFS_ADC] = adc_values[adc_updch];
        if(PORTB.IN & PIN5_bm)
        {
            inputRegisters[MODBUS_REGOFFS_ADC] |= 0x80;
        }
    }
}

//////////////////////////////////////////////////////////////////////////
/// Pulse counter

typedef struct
{
    uint8_t h_len;
    uint8_t l_len;
    uint16_t h_cnt;
    uint16_t l_cnt;
    bool h_cnt_rollover;
    bool l_cnt_rollover;
}   pulsecounter_ch_t;

#define PULSECOUNTER_CH_COUNT 9

pulsecounter_ch_t pulsecounter_ch[PULSECOUNTER_CH_COUNT];

void pulsecounter_init(void)
{
    for(uint8_t i = 0; i < PULSECOUNTER_CH_COUNT; i++)
    {
        pulsecounter_ch_t *ch = &pulsecounter_ch[i];
        ch->h_len = 0xFF;
        ch->l_len = 0xFF;
        ch->h_cnt = 0;
        ch->l_cnt = 0;
        ch->h_cnt_rollover = false;
        ch->l_cnt_rollover = false;
    }
}

bool pulsecounter_check_ch(uint8_t val, pulsecounter_ch_t *ch)
{
    bool updated = false;
    if(val > 0)
    {
        ch->l_len = 0;
        if(ch->h_len < 0xFF)
        {
            ch->h_len++;

            if(ch->h_len == PULSECOUNTER_H_THRESHOLD)
            {
                ch->h_cnt++;
                updated = true;
                if(ch->h_cnt == 0)
                {
                    ch->h_cnt_rollover = true;
                }
            }
        }
    }
    else
    {
        ch->h_len = 0;
        if(ch->l_len < 0xFF)
        {
            ch->l_len++;

            if(ch->l_len == PULSECOUNTER_L_THRESHOLD)
            {
                updated = true;
                ch->l_cnt++;
                if(ch->l_cnt == 0)
                {
                    ch->l_cnt_rollover = true;
                }
            }
        }
    }
    return updated;
}

void pulsecounter_tick(void)
{
    uint8_t val = PORTA.IN;
    for(uint8_t i = 0; i < 8; i++)
    {
        if(pulsecounter_check_ch(val & 1, &pulsecounter_ch[i]) == true)
        {
            inputRegisters[MODBUS_REGOFFS_PULSECOUNTER + i] = pulsecounter_ch[i].l_cnt;
        }
        val >>= 1;
    }

    val = PORTB.IN & (1<<5);
    uint8_t i = 8;
    pulsecounter_check_ch(val, &pulsecounter_ch[i]);
    inputRegisters[MODBUS_REGOFFS_PULSECOUNTER + i] = pulsecounter_ch[i].l_cnt;
}

int main(void)
{
    CCP = CCP_IOREG_gc;
    CLKCTRL.MCLKCTRLB = /*CLKCTRL_PDIV_4X_gc |*/ (0 << CLKCTRL_PEN_bp);

    PORTA.DIRSET = DS18B20_MAP;

    PORTB.DIRSET = (1<<PIN4_bp) | (1<<PIN2_bp); // UART TX
    PORTB.DIRCLR = (1<<PIN3_bp); // UART RX
    PORTB.PIN3CTRL = (1<<PORT_PULLUPEN_bp); // enable pull-up for UART RX

    PORTC.DIRSET = (1<<PIN1_bp) | (1<<PIN0_bp); // status LEDs

    systick_init();
    uint8_t sys_ticks_last = sys_ticks;

    i2cm_init();
    modbusSetAddress(MODBUS_ADDRESS);
    modbusInit();
    adc0_init();
    tempsensors_invalidate();
    pulsecounter_init();

    //uart_init();
    sei();

    // everything initialized - turn on the status LED
    // it will flicker on modbus activity
    led_green_on();

    uint16_t postscale_i2c = 0;
    uint8_t postscale_pulsecounter = 0;

    while(1)
    {
        uint8_t tmp = sys_ticks;
        uint8_t ticks_passed = tmp - sys_ticks_last;
        sys_ticks_last = tmp;

        modbusGet();

        if(ticks_passed > 0)
        {
            tempsensors_tick();
            adc_tick();
            
            if(postscaler16(ticks_passed, &postscale_i2c, ROTENC_UPDATE_INTERVAL))
            {
                rotenc_tick();
            }

            if(postscaler8(ticks_passed, &postscale_pulsecounter, PULSECOUNTER_TICK_INTERVAL))
            {
                pulsecounter_tick();
            }

            if(modbus_act_cnt > 0)
            {
                if(ticks_passed > modbus_act_cnt)
                {
                    modbus_act_cnt = 0;
                }
                else
                {
                    modbus_act_cnt -= ticks_passed;
                }

                if(modbus_act_cnt == 0)
                    led_green_on();
            }

            wdt_reset();
        }
    }
}

