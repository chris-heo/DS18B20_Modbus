#ifndef CONFIG_HELPER_H_
#define CONFIG_HELPER_H_


#define VREF_INT_0V55 0
#define VREF_INT_1V1 1
#define VREF_INT_1V5 2
#define VREF_INT_2V5 3
#define VREF_INT_4V34 4
#define VREF_VDD 5

#define USAGE_NONE 0
#define USAGE_DS18B20 1
//#define USAGE_PULSECOUNT 2
//#define USAGE_ADC 4

#define SENSOR_USAGE(x) (0 \
| (((S1_USAGE) & (x)) > 0 ? (1 << 1) : 0) \
| (((S2_USAGE) & (x)) > 0 ? (1 << 2) : 0) \
| (((S3_USAGE) & (x)) > 0 ? (1 << 3) : 0) \
| (((S4_USAGE) & (x)) > 0 ? (1 << 4) : 0) \
| (((S5_USAGE) & (x)) > 0 ? (1 << 5) : 0) \
| (((S6_USAGE) & (x)) > 0 ? (1 << 6) : 0) \
| (((S7_USAGE) & (x)) > 0 ? (1 << 7) : 0) \
)

#define DS18B20_MAP SENSOR_USAGE(USAGE_DS18B20)

#endif /* CONFIG_HELPER_H_ */
