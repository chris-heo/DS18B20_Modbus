#include "as5600.h"
#include "i2cm.h"

bool as5600_config(
    as5600_powermode_e powermode, as5600_hysteresis_e hysteresis, as5600_outputstage_e outputstage,
    as5600_pwmfreq_e pwmfreq, as5600_slowfilter_e slowfilter, as5600_fastfilterthrs_e fastfilterthrs, bool watchdog
)
{
    uint8_t data[2] = {0};
    data[0] = (watchdog == true ? (1 << 5) : (0 << 5)) | ((uint8_t)fastfilterthrs << 2) | ((uint8_t)slowfilter << 0);
    data[1] = ((uint8_t)pwmfreq << 6) | ((uint8_t)outputstage << 4) | ((uint8_t)hysteresis << 2) | ((uint8_t)powermode << 0);

    if(i2cm_writeregs(AS5600_ADDR, 0x07, data, sizeof(data)) == i2c_status_ok)
    return true;

    return false;
}
