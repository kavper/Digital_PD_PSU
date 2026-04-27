#ifndef PSU_CONFIG_H
#define PSU_CONFIG_H

/* PWM: HRTIM Timer D, 170 MHz * 32 = 5.44 GHz, PER=27200 -> 200 kHz. */
#define PSU_SW_FREQ_KHZ 200UL

/*
 * ADC/control divider:
 * 4 = PWM 200 kHz, ADC/CV loop 50 kHz.
 * Full 200 kHz control through the current HAL/DMA path starved the OLED/main
 * loop on this project, so 50 kHz is the stable operating point for now.
 */
#define PSU_ADC_DIV 4UL

#if (PSU_SW_FREQ_KHZ == 0UL)
#error "PSU_SW_FREQ_KHZ must be greater than 0"
#endif

#if (PSU_ADC_DIV == 0UL)
#error "PSU_ADC_DIV must be greater than 0"
#endif

enum {
  HRTIM_PERIOD = 27200UL,
  HRTIM_ADC_DIV = PSU_ADC_DIV,
  /* In STM32 HAL postscaler=0 means every trigger, so use div - 1. */
  HRTIM_ADC_POSTSCALER = HRTIM_ADC_DIV - 1UL
};

#define PSU_DEFAULT_TARGET_VOLTAGE_V (0.0f)
#define PSU_DEFAULT_CURRENT_LIMIT_A  (1.0f)

#endif /* PSU_CONFIG_H */
