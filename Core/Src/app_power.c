#include "app_power.h"
#include "control_2p2z.h"
#include "hrtim_pwm.h"
#include "main.h"
#include "pid.h"
#include "power_measurement.h"
#include "stm32g4xx_hal_adc.h"
#include <stdint.h>

#define PSU_VOLTAGE_TARGET_MAX_V (30.0f)
#define PSU_SOFTSTART_TIME_S     (0.050f)
#define PSU_CONTROL_FREQ_HZ      ((float)(HRTIM_PWM_FSW_HZ / HRTIM_ADC_DIV))
#define PSU_DUTY_MAX_START       (0.60f)
#define PSU_DUTY_MAX_NORMAL      (0.80f)

/*
 * Ostrozny startowy 2P2Z dla fcontrol = 50 kHz.
 *
 * Poprzednie wspolczynniki byly policzone dla fcontrol = 200 kHz i
 * crossover ok. 8 kHz. Po przejsciu na 50 kHz dawaly za agresywna petle
 * i potrafily wzbudzac bucka w okolicy kilku kHz pod obciazeniem.
 *
 * Ten wariant zostawia:
 * - fz1 = fz2 ~= 2.32 kHz, okolica rezonansu LC,
 * - fp1 ~= 22.7 kHz, okolica zera ESR,
 * - niski gain K = 0.5, czyli konserwatywny crossover rzedu setek Hz.
 *
 * Po pomiarze odpowiedzi skokowej mozna stopniowo zwiekszac K, np.
 * 0.65 albo 0.75, ale dopiero gdy na oscyloskopie nie ma dzwonienia.
 */
#define PSU_2P2Z_B0 (0.50000000f)
#define PSU_2P2Z_B1 (-0.74711228f)
#define PSU_2P2Z_B2 (0.27908838f)
#define PSU_2P2Z_A1 (-1.05769608f)
#define PSU_2P2Z_A2 (0.05769608f)

extern ADC_HandleTypeDef hadc1;
extern HRTIM_HandleTypeDef hhrtim1;

uint16_t psu_adc_dma_buffer[PSU_ADC_CHANNEL_COUNT] = {0};

static DF22_Controller_t psu_cv_controller;
static PowerMeasurements_t psu_measurements;
static float psu_target_voltage_v = 0.0f;
static float psu_ref_adc_ramp_v = 0.0f;
static uint8_t psu_initialized = 0U;
static uint8_t psu_running = 0U;
static uint8_t psu_adc_dma_running = 0U;

static float psu_clamp(float value, float min_value, float max_value)
{
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

static void psu_reset_control(float duty_max)
{
  psu_ref_adc_ramp_v = 0.0f;
  DF22_Reset(&psu_cv_controller);
  DF22_SetOutputLimits(&psu_cv_controller, HRTIM_PWM_DUTY_MIN, duty_max);
  HRTIM_PWM_SetDuty(HRTIM_PWM_DUTY_MIN);
}

static void psu_start_adc_dma(void)
{
  if (psu_adc_dma_running != 0U) {
    return;
  }

  if (HAL_ADC_Start_DMA(&hadc1,
                        (uint32_t *)psu_adc_dma_buffer,
                        PSU_ADC_CHANNEL_COUNT) != HAL_OK) {
    Error_Handler();
  }

  /*
   * HAL_ADC_Start_DMA enables half-transfer interrupts too. With a 3-sample
   * circular buffer and HRTIM trigger this creates extra IRQ load without
   * adding useful control data.
   */
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT);

  psu_adc_dma_running = 1U;
}

static void psu_stop_adc_dma(void)
{
  if (psu_adc_dma_running == 0U) {
    return;
  }

  (void)HAL_ADC_Stop_DMA(&hadc1);
  psu_adc_dma_running = 0U;
}

static void psu_update_softstart(float target_adc_v)
{
  const float step_scale = 1.0f / (PSU_CONTROL_FREQ_HZ * PSU_SOFTSTART_TIME_S);
  float step = target_adc_v * step_scale;

  if (step < 0.000001f) {
    step = 0.000001f;
  }

  if (psu_ref_adc_ramp_v < target_adc_v) {
    psu_ref_adc_ramp_v += step;
    if (psu_ref_adc_ramp_v > target_adc_v) {
      psu_ref_adc_ramp_v = target_adc_v;
    }
  } else if (psu_ref_adc_ramp_v > target_adc_v) {
    psu_ref_adc_ramp_v -= step;
    if (psu_ref_adc_ramp_v < target_adc_v) {
      psu_ref_adc_ramp_v = target_adc_v;
    }
  }
}

void PSU_Init(void)
{
  if (psu_initialized != 0U) {
    return;
  }

  psu_target_voltage_v = 0.0f;
  psu_ref_adc_ramp_v = 0.0f;
  psu_running = 0U;
  psu_adc_dma_running = 0U;

  DF22_Init(&psu_cv_controller,
            PSU_2P2Z_B0,
            PSU_2P2Z_B1,
            PSU_2P2Z_B2,
            PSU_2P2Z_A1,
            PSU_2P2Z_A2,
            HRTIM_PWM_DUTY_MIN,
            PSU_DUTY_MAX_START);

  HRTIM_PWM_Init(&hhrtim1);
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }

  psu_initialized = 1U;
}

void PSU_Start(void)
{
  if (psu_initialized == 0U) {
    PSU_Init();
  }

  psu_reset_control(PSU_DUTY_MAX_START);
  psu_start_adc_dma();
  HRTIM_PWM_Start();
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
  psu_running = 1U;
}

void PSU_Stop(void)
{
  psu_running = 0U;
  HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
  psu_reset_control(PSU_DUTY_MAX_START);
  HRTIM_PWM_Stop();
  psu_stop_adc_dma();
}

void PSU_SetVoltage(float voltage_v)
{
  psu_target_voltage_v = psu_clamp(voltage_v, 0.0f, PSU_VOLTAGE_TARGET_MAX_V);
}

void PSU_ControlLoopFromAdc(const uint16_t adc_samples[PSU_ADC_CHANNEL_COUNT])
{
  if (adc_samples == 0) {
    return;
  }

  PowerMeasurement_Update(&psu_measurements,
                          adc_samples[PSU_ADC_INDEX_VOUT],
                          adc_samples[PSU_ADC_INDEX_IOUT],
                          adc_samples[PSU_ADC_INDEX_VBOOST]);

  if ((psu_running == 0U) || (psu_target_voltage_v <= 0.0f)) {
    psu_reset_control(PSU_DUTY_MAX_START);
    return;
  }

  const float target_adc_v = psu_target_voltage_v * POWER_VOUT_GAIN;
  psu_update_softstart(target_adc_v);

  const float error_adc_v = psu_ref_adc_ramp_v - psu_measurements.vout_adc_v;
  const float duty = DF22_Update(&psu_cv_controller, error_adc_v);

  HRTIM_PWM_SetDuty(duty);

  if (psu_ref_adc_ramp_v >= target_adc_v) {
    DF22_SetOutputLimits(&psu_cv_controller, HRTIM_PWM_DUTY_MIN, PSU_DUTY_MAX_NORMAL);
  }
}

float PSU_GetMeasuredVoltage(void)
{
  return psu_measurements.vout_v;
}

float PSU_GetMeasuredCurrent(void)
{
  return psu_measurements.iout_a;
}

float PSU_GetMeasuredInputVoltage(void)
{
  return psu_measurements.vboost_v;
}

float PSU_GetDuty(void)
{
  return HRTIM_PWM_GetDuty();
}

float PSU_GetTargetVoltage(void)
{
  return psu_target_voltage_v;
}

float PSU_GetSoftStartVoltage(void)
{
  return psu_ref_adc_ramp_v / POWER_VOUT_GAIN;
}

void PSU_AdcDmaIrqHandler(void)
{
  DEBUG_ADC_ISR_HIGH();
  PSU_ControlLoopFromAdc(psu_adc_dma_buffer);
  DEBUG_ADC_ISR_LOW();
}

uint8_t PSU_IsRunning(void)
{
  return psu_running;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance != ADC1) {
    return;
  }

  PSU_AdcDmaIrqHandler();
}
